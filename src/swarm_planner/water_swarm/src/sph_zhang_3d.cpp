#include "sph_zhang_3d.h"

SPHSystem* sph_planner;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sph_particles");
    ros::NodeHandle nh("~");

    nh.param("sph/particleCount", particleCount, -1);
    nh.param("sph/particleInterval", particleInterval, 0.1);
    nh.param("sph/particleVisScale", particleVisScale, 0.1);
    nh.param("sph/mass", mass, 1.00f);
    nh.param("sph/restDensity", restDensity, 1.0f);
    nh.param("sph/h", h, 0.15f);
    nh.param("sph/g", g, -9.8f);
    nh.param("sph/updateInterval", updateInterval, 0.01);
    nh.param("sph/threshold_dist", threshold_dist, 0.1);
    nh.param("sph/k_den", k_den, -1.0);
    nh.param("sph/k_rep", k_rep, -1.0);
    nh.param("sph/k_fri", k_fri, -1.0);
    nh.param("sph/k_p", k_p, -1.0);  
    nh.param("sph/k_d", k_d, 0.0);  
    nh.param("sph/k_ff", k_ff, 0.0); 
    nh.param("sph/v_max", v_max, -1.0);
    nh.param("sph/a_max", a_max, -1.0);
    nh.param("sph/r_1",   r_1, -1.0);
    nh.param("sph/r_2",   r_2, -1.0);
    nh.param("sph/state_enabled", state_enabled, false);  
    nh.param("sph/vis_role", vis_role, false);  
    nh.param("sph/init_bias_x", init_bias_x, -1.0);  
    nh.param("sph/init_bias_y", init_bias_y, -1.0); 
    timer                 = nh.createTimer(ros::Duration(updateInterval),   timerCallback);
    particles_publisher   = nh.advertise<visualization_msgs::MarkerArray>("particles_vis", 10);
    virtual_particles_publisher = nh.advertise<visualization_msgs::MarkerArray>("virtual_particles_vis", 10);
    swarm_pub             = nh.advertise<common_msgs::Swarm_particles>("/swarm_particles", 10);
    swarm_traj_sub        = nh.subscribe("/swarm_traj", 1000, swarmTrajCallback);
    target_sub            = nh.subscribe("/particle_target", 10,targetCallback);
    force_sub             = nh.subscribe("/particle_force", 10,forceCallback);
    collision_matrix_sub  = nh.subscribe("/check_collision_matrix", 10,collisionMatrixCallback);
    pos_pub               = nh.advertise<geometry_msgs::PoseStamped>("trajectory_position", 10);
    vel_pub               = nh.advertise<geometry_msgs::PoseStamped>("trajectory_velocity", 10);
    initial_pospub        = nh.advertise<visualization_msgs::MarkerArray>("/initial_pose", 1,true);
    odom_publishers.resize(particleCount);
    for (int i = 0; i < particleCount; ++i) {
        std::stringstream ss;
        ss << "/particle" << i << "/odom";
        odom_publishers[i] = nh.advertise<nav_msgs::Odometry>(ss.str(), 10);
    }

    //start sph
    SPHSettings sphSettings(mass, restDensity, h, g);
    sph_planner = new SPHSystem(15, sphSettings, false);

    ROS_INFO("sph_planner node has started.");

    ros::spin();
    return 0;
}
void goalCallback(const common_msgs::Swarm_particles::ConstPtr& msg){}
void collisionMatrixCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{
    int num_rows = msg->layout.dim[0].size;
    int num_cols = msg->layout.dim[1].size;

    Eigen::MatrixXi collision_matrix(num_rows, num_cols);

    for (int i = 0; i < num_rows; ++i) {
        for (int j = 0; j < num_cols; ++j) {
            collision_matrix(i, j) = msg->data[i * num_cols + j];
        }
    }

    sph_planner->updateCollisionMatrix(collision_matrix);
}

void swarmTrajCallback(const common_msgs::Swarm_traj::ConstPtr& msg)
{
    static ros::Time last_recv_time;
    ros::Time current_time = msg->header.stamp;
    if (!last_recv_time.isZero()) {
        double dt = (current_time - last_recv_time).toSec();
        ROS_INFO_STREAM("\033[1;33m [SwarmTraj] Δt since last trajectory: " << dt << " s \033[0m");
    }
    last_recv_time = current_time;  

    common_msgs::Swarm_traj swarmTrajBuffer = *msg;
    sph_planner->processTraj(swarmTrajBuffer);
    ROS_INFO("\033[1;32m RECEIVE SWARM TRAJ. \033[0m");
}


void timerCallback(const ros::TimerEvent&) {
    
    current_time = ros::Time::now();

    if (!sph_planner->started) 
    {
        if ((current_time - last_print_time).toSec() >= 1.0) {
            ROS_INFO("Do not receive the start command, return.");
            last_print_time = current_time; 
        }
        return;
    }
    else
    {   
        float deltaTime = last_time.isZero() ? 0.0 : (current_time - last_time).toSec();
        last_time = current_time;

        sph_planner->update(updateInterval);
    }
}

void targetCallback(const common_msgs::Swarm_particles::ConstPtr& msg)
{
    for (const auto& particle : msg->particles) {
        sph_planner-> targetMap[particle.index] = particle.position;

        ROS_INFO("Stored particle with index %d: [%f, %f, %f]", 
                particle.index, particle.position.x, particle.position.y, particle.position.z);
    }
    receive_target = true;
}

void forceCallback(const common_msgs::Swarm_particles::ConstPtr& msg)
{
    sph_planner->forceMap.clear();
    for (const auto& particle : msg->particles) {
        sph_planner-> forceMap[particle.index] = particle.force;
    }
}

void SPHSystem::processTraj(const common_msgs::Swarm_traj& swarmtraj)
{
    for (const auto& traj : swarmtraj.traj)
    {
        // 判断轨迹是否为空（基于 position 列表是否为空）
        if (!traj.position.empty()) {
            swarmTrajBuffer_[traj.traj_id] = traj;
        }
    }
    ROS_INFO("Swarm trajectory processed, %lu trajectories stored.", swarmTrajBuffer_.size());
}

void SPHSystem::initParticles()
{
    sph_particleCount = particleCount;

    int sideLength = static_cast<int>(std::sqrt(particleCount));
        if (sideLength * sideLength < particleCount) {
            ++sideLength;
        }
    double maxRange = sideLength * particleInterval;
    
        for (int i = 0; i < particleCount; ++i) {
            int row = i / sideLength;
            int col = i % sideLength;
 
            Particle p;
            p.position.x = col * particleInterval-init_bias_x;
            p.position.y = row * particleInterval-init_bias_y;
            p.position.z = 1.0; 
            p.velocity.x = 0.0;
            p.velocity.y = 0.0;
            p.velocity.z = 0.0;
            p.acceleration.x = 0.0;
            p.acceleration.y = 0.0;
            p.acceleration.z = 0.0;
            p.force.x = 0.0;
            p.force.y = 0.0;
            p.force.z = 0.0;
            p.density = settings.selfDens;
            p.pressure = 0.0;
            p.hash = 0; 
            p.index= i;
            std::stringstream ss;
            ss << "Particle " << i + 1;
            p.name.data = ss.str();

            particles.push_back(p);
        } 

    visualization_msgs::MarkerArray marker_array;

    for (const auto& p : particles) {
        visualization_msgs::Marker cube_marker;
        cube_marker.header.frame_id = "world";
        cube_marker.header.stamp = ros::Time::now();
        cube_marker.ns = "particle_cubes";
        cube_marker.id = p.index;
        cube_marker.type = visualization_msgs::Marker::CUBE;
        cube_marker.action = visualization_msgs::Marker::ADD;
        cube_marker.pose.position.x = p.position.x;
        cube_marker.pose.position.y = p.position.y;
        cube_marker.pose.position.z = p.position.z;
        cube_marker.pose.orientation.w = 1.0;
        cube_marker.scale.x = 0.1;  
        cube_marker.scale.y = 0.1;
        cube_marker.scale.z = 0.1;
        cube_marker.color.r = 1.0;
        cube_marker.color.g = 0.5;
        cube_marker.color.b = 1.0;
        cube_marker.color.a = 1.0;

        marker_array.markers.push_back(cube_marker);
    }

    initial_pospub.publish(marker_array);

}

void SPHSystem::findNeighbors() {

    particleNeighborsTable.clear();
    nearestNeighborDistanceMap.clear();

    for (auto& particle : particles) {
        std::vector<std::pair<const Particle*, float>> neighbors;
        double minDistance = std::numeric_limits<double>::max();

        for (auto& other : particles) {
            if (&particle == &other) continue;
            if (collision_matrix_.size() > 0 && collision_matrix_(particle.index, other.index) == 1) 
                {
                    continue;
                } 
            float dx = particle.position.x - other.position.x;
            float dy = particle.position.y - other.position.y;
            float dz = particle.position.z - other.position.z;
            float dist2 = dx * dx + dy * dy + dz * dz;
            float h2 = settings.h * settings.h;

            if (dist2 < 4 * h2) {
                neighbors.push_back(std::make_pair(&other, dist2));
                minDistance = std::min(minDistance, static_cast<double>(dist2));
            }
        }

        particleNeighborsTable[&particle] = neighbors;

        if (!neighbors.empty()) {
            nearestNeighborDistanceMap[&particle] = std::sqrt(minDistance);  
        } else {
            nearestNeighborDistanceMap[&particle] = -1.0; 
        }
    }

}

void SPHSystem::updateParticleRole(){}

void SPHSystem::updateParticleStates()
{
    for (auto& particle : particles) {
        int particleIndex = particle.index;

        if (targetMap.find(particleIndex) == targetMap.end()) {
            continue;
        }
        const auto& targetPosition = targetMap[particleIndex];

        if (swarmTrajBuffer_.find(particleIndex) != swarmTrajBuffer_.end() &&
            !swarmTrajBuffer_[particleIndex].position.empty()) {
        
            particle.state = TRAJ;
        } 
        else
            particle.state = NEAR_TARGET;
    }
}

void SPHSystem::update(float deltaTime) {
	if (!started) return;
    runOnGPU = false;
    updateParticles(particles, particleCount, settings, deltaTime,runOnGPU);
}

void SPHSystem::reset() {
    isInitialReceived = false;
	initParticles();
	started = false;
}

void SPHSystem::start() {
	started = true;
}

void SPHSystem::updateParticles(
    std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
    float deltaTime, const bool onGPU)
{
    if (onGPU) {
        updateParticlesGPU(
            particles, particleCount, settings, deltaTime);
    }
    else {
        updateParticlesCPU(
            particles, particleCount, settings, deltaTime);
    }
}

void SPHSystem::updateParticlesCPU(
    std::vector<Particle> particles, const size_t particleCount, const SPHSettings &settings,
    float deltaTime)
{   
    findNeighbors();

    updateParticleStates();

    // Calculate densities and pressures
    parallelDensityAndPressures();

    // Calculate forces
    parallelForces();

    // Update particle positions and neighbors
    parallelUpdateParticlePositions(deltaTime);

    //rospub control commands
    pubroscmd();
}

void SPHSystem::parallelDensityAndPressures()
{   
    for (auto& particle : particles) {

        float pDensity = 0.0;
        float neighborGrad = 0.0;

        particle.u_den.x = 0.0;
        particle.u_den.y = 0.0;
        particle.u_den.z = 0.0;

        auto& neighbors = particleNeighborsTable[&particle];
        for (auto& neighbor_pair : neighbors) {

            float dist2 = neighbor_pair.second;
            double k = std::sqrt(dist2)/h;
            double q;

            if ( k>=0 && k<=1)
            {
                q = 1-(3/2)*k*k + (3/4)*k*k*k;
            }
            else if (k>1 && k<=2)
            {
                q = (1/4)*(2-k)*(2-k)*(2-k);
            }   
            else
            {
                q = 0;
            }

            pDensity += settings.poly * q;
        }

        particle.density = pDensity + settings.selfDens;
      
        double p = -k_den  * (1/particle.density) * (std::pow(particle.density/settings.restDensity,7) -1 );
        
        for (auto& neighbor_pair : neighbors)
        {
            const Particle* neighbor = neighbor_pair.first;
            float dist2 = neighbor_pair.second;
            float dist = std::sqrt(dist2);

            float dx = particle.position.x - neighbor->position.x;
            float dy = particle.position.y - neighbor->position.y;
            float dz = particle.position.z - neighbor->position.z;
       
            float dir_x = dx / dist;
            float dir_y = dy / dist;
            float dir_z = dz / dist;

            double k = std::sqrt(dist2)/h;
            double q;

            if ( k>=0 && k<=1)
            {
                q = -3*k + (9/4)*k*k;
            }
            else if (k>1 && k<=2)
            {
                q = (-3/4)*(2-k)*(2-k);
            }   
            else
            {
                q = 0;
            }

            neighborGrad = settings.poly * h * q;

            particle.u_den.x += p * neighborGrad * dir_x;
            particle.u_den.y += p * neighborGrad * dir_y;
            particle.u_den.z += p * neighborGrad * dir_z;

        }
    }
}

void SPHSystem::parallelForces()
{
    for (auto& particle : particles) {

        particle.u_rep.x = 0.0;
        particle.u_rep.y = 0.0;
        particle.u_rep.z = 0.0;

        particle.u_fri.x = 0.0; 
        particle.u_fri.y = 0.0; 
        particle.u_fri.z = 0.0; 

        auto& neighbors = particleNeighborsTable[&particle];
        
        for (auto& neighbor_pair : neighbors) {
            const Particle* neighbor = neighbor_pair.first;
            float dist2 = neighbor_pair.second;

            float dist = std::sqrt(dist2);

            float dx = particle.position.x - neighbor->position.x;
            float dy = particle.position.y - neighbor->position.y;
            float dz = particle.position.z - neighbor->position.z;
            
            float dir_x = dx / dist;
            float dir_y = dy / dist;
            float dir_z = dz / dist;

            particle.u_rep.x += k_rep * 1/dist2 * dist * dir_x;
            particle.u_rep.y += k_rep * 1/dist2 * dist * dir_y;
            particle.u_rep.z += k_rep * 1/dist2 * dist * dir_z;

        }
    
        particle.u_fri.x = -k_fri * particle.velocity.x; 
        particle.u_fri.y = -k_fri * particle.velocity.y; 
        particle.u_fri.z = -k_fri * particle.velocity.z; 
    }   
}


void SPHSystem::parallelUpdateParticlePositions(const float deltaTime)
{
    for (size_t i = 0; i < particles.size(); i++) {
        Particle *p = &particles[i];

        common_msgs::Acceleration acceleration;
        acceleration.x = 0.0f;
        acceleration.y = 0.0f;
        acceleration.z = 0.0f;

        auto itt = swarmTrajBuffer_.end(); 
        bool traj_found = false;

        if (p->state == TRAJ || p->state == NEED_TRAJ) {
            itt = swarmTrajBuffer_.find(p->index);
            if (itt != swarmTrajBuffer_.end()) {
                traj_found = true;
            }
        }

        switch (p->state) {
            case NULL_STATE:
                acceleration.x = p->u_den.x + p->u_rep.x + p->u_fri.x;
                acceleration.y = p->u_den.y + p->u_rep.y + p->u_fri.y;
                acceleration.z = p->u_den.z + p->u_rep.z + p->u_fri.z;
                break;

            case NEAR_TARGET:
                acceleration.x = p->u_den.x + p->u_rep.x + p->u_fri.x;
                acceleration.y = p->u_den.y + p->u_rep.y + p->u_fri.y;
                acceleration.z = 0;
                break;

            case ATTRACT:

                break;

            case REPEL:

                break;

            case TRAJ:
            case NEED_TRAJ:
                if (traj_found)
                {
                    const auto& pos = itt->second.position;
                    const auto& vel = itt->second.velocity;
                    const auto& acc = itt->second.acceleration;
                    if (!pos.empty())
                    {
                        Eigen::Vector3d position_error(pos[0].x - p->position.x,
                                                    pos[0].y - p->position.y,
                                                    pos[0].z - p->position.z);

                        Eigen::Vector3d velocity_error(vel[0].x - p->velocity.x,
                                                    vel[0].y - p->velocity.y,
                                                    vel[0].z - p->velocity.z);

                        acceleration.x = p->u_den.x + p->u_rep.x + p->u_fri.x 
                                        + k_p * position_error.x() 
                                        + k_d * velocity_error.x() 
                                        + k_ff * acc[0].x
                                        + forceMap[p->index].x;

                        acceleration.y = p->u_den.y + p->u_rep.y + p->u_fri.y 
                                        + k_p * position_error.y() 
                                        + k_d * velocity_error.y() 
                                        + k_ff * acc[0].y
                                        + forceMap[p->index].y;

                        acceleration.z = k_p * position_error.z() 
                                        + k_d * velocity_error.z() 
                                        + k_ff * acc[0].z
                                        + forceMap[p->index].z;

                        geometry_msgs::PoseStamped pos_msg;
                        pos_msg.header.stamp = ros::Time::now();
                        pos_msg.header.frame_id = "world";
                        pos_msg.pose.position.x = pos[0].x;
                        pos_msg.pose.position.y = pos[0].y;
                        pos_msg.pose.position.z = pos[0].z;
                        pos_msg.pose.orientation.w = 1.0; 
                        pos_pub.publish(pos_msg);

                        geometry_msgs::PoseStamped vel_msg;
                        vel_msg.header.stamp = ros::Time::now();
                        vel_msg.header.frame_id = "world";
                        vel_msg.pose.position.x = vel[0].x;
                        vel_msg.pose.position.y = vel[0].y;
                        vel_msg.pose.position.z = vel[0].z;
                        vel_msg.pose.orientation.w = 1.0;
                        vel_pub.publish(vel_msg);
                    }
                    
                }
                break;
            
            default:
                std::cerr << "Error: Unknown particle state!" << std::endl;
                break;
        }

        acceleration.x = clamp(acceleration.x, a_max);
        acceleration.y = clamp(acceleration.y, a_max);
        acceleration.z = clamp(acceleration.z, a_max);  

        //update acc
        p->acceleration.x = acceleration.x;
        p->acceleration.y = acceleration.y;
        p->acceleration.z = acceleration.z;

        p->velocity.x += acceleration.x * deltaTime;
        p->velocity.y += acceleration.y * deltaTime;
        p->velocity.z += acceleration.z * deltaTime;

        p->velocity.x = clamp(p->velocity.x, v_max);
        p->velocity.y = clamp(p->velocity.y, v_max);
        p->velocity.z = clamp(p->velocity.z, v_max);

        p->position.x += p->velocity.x * deltaTime;
        p->position.y += p->velocity.y * deltaTime;
        p->position.z += p->velocity.z * deltaTime;

        if (p->position.z < 0.0) {
            p->position.z = 0.1;

            double rebound_coeff = 0.5;
            p->velocity.z = -p->velocity.z * rebound_coeff;
            p->acceleration.z = -p->acceleration.z * rebound_coeff;
        }

    }
    
    parallelUpdateParticleTraj();
}

void SPHSystem::pubroscmd() 
{
    visualization_msgs::MarkerArray particles_markers;
    visualization_msgs::MarkerArray state_text_markers; 
    common_msgs::Swarm_particles swarm_msg;  
    int id = 0;
    for (const auto& particle : particles) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "particles";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = particle.position.x;
        marker.pose.position.y = particle.position.y;
        marker.pose.position.z = particle.position.z;
        marker.pose.orientation.w = 1.0;
        marker.id = particle.index;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x = particleVisScale;
        marker.scale.y = particleVisScale;
        marker.scale.z = particleVisScale;
        if (particle.role == LEADER)
        {
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        particles_markers.markers.push_back(marker);

        if (state_enabled)
        {
            visualization_msgs::Marker state_marker;
            state_marker.header.frame_id = "world";
            state_marker.header.stamp = ros::Time::now();
            state_marker.ns = "state_texts";
            state_marker.action = visualization_msgs::Marker::ADD;
            state_marker.pose.position.x = particle.position.x;
            state_marker.pose.position.y = particle.position.y; 
            state_marker.pose.position.z = particle.position.z + particleVisScale + 0.4;
            state_marker.pose.orientation.w = 1.0;
            state_marker.id = particle.index + particles.size(); 
            state_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            state_marker.scale.z = 0.5;
            state_marker.color.a = 1.0;
            state_marker.color.r = 0.0; 
            state_marker.color.g = 0.0;
            state_marker.color.b = 0.0; 
            if (vis_role)
            {
                state_marker.text = std::to_string(particle.index);
            }
            else
            {
                state_marker.text = stateToString(particle.state);
            }
            
            state_text_markers.markers.push_back(state_marker);
        }
        
        common_msgs::Particle swarm_particle;
        swarm_particle.position = particle.position;
        swarm_particle.velocity = particle.velocity;
        swarm_particle.acceleration = particle.acceleration;
        swarm_particle.force = particle.force;
        swarm_particle.density = particle.density;
        swarm_particle.pressure = particle.pressure;
        swarm_particle.index = particle.index;
        swarm_particle.state = particle.state;
        swarm_particle.role  = particle.role;
        swarm_msg.particles.push_back(swarm_particle);
    }

    for (const auto& particle : particles) {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world";
        odom.child_frame_id = "particle" + std::to_string(particle.index);

        odom.pose.pose.position.x = particle.position.x;
        odom.pose.pose.position.y = particle.position.y;
        odom.pose.pose.position.z = particle.position.z;
        odom.pose.pose.orientation.w = 1.0; 

        odom.twist.twist.linear.x = particle.velocity.x;
        odom.twist.twist.linear.y = particle.velocity.y;
        odom.twist.twist.linear.z = particle.velocity.z;

        if (particle.index < odom_publishers.size()) {
            odom_publishers[particle.index].publish(odom);
        }
    }

    particles_publisher.publish(particles_markers);
    particles_publisher.publish(state_text_markers);

    swarm_msg.header.stamp = ros::Time::now(); 
    swarm_msg.header.frame_id = "world";  
    swarm_pub.publish(swarm_msg);
}

void SPHSystem::calaDynamicBound(){}

void SPHSystem::parallelUpdateParticleTraj() {
  
    for (auto& entry : swarmTrajBuffer_) {
        auto& traj = entry.second;

        if (traj.position.empty() || traj.velocity.empty() ||
            traj.acceleration.empty() || traj.jerk.empty()) {
               
            continue;
        }
        traj.position.erase(traj.position.begin());
        traj.velocity.erase(traj.velocity.begin());
        traj.acceleration.erase(traj.acceleration.begin());
        traj.jerk.erase(traj.jerk.begin());
    }
}

void SPHSystem::generateVirtualParticles(const double l, const int particlesPerSide, const common_msgs::Position& apex) {}
bool SPHSystem::isNearVirtualParticle(double x, double y, double z) {}

SPHSystem::SPHSystem(
    size_t particleCubeWidth, const SPHSettings &settings,
    const bool &runOnGPU)
    : sph_particleCubeWidth(particleCubeWidth)
    , settings(settings)
    , runOnGPU(runOnGPU)
{
    started = true;
    initParticles();
    findNeighbors();
   
}

SPHSystem::SPHSystem()
    : sph_particleCubeWidth(0.0),
      settings(),
      started(false),
      runOnGPU(false){}

SPHSystem::~SPHSystem() {
}
