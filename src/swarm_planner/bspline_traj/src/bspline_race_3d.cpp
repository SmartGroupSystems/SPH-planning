#include "bspline_race/bspline_race_3d.h"

namespace FLAG_Race

{
    plan_manager::plan_manager(ros::NodeHandle &nh)
    {
        nh.param("fsm/planInterval", planInterval, -1.0);
        nh.param<std::string>("fsm/cloud_topic", cloud_topic_, "click_map");
        nh.param("fsm/trajVisParam", trajVisParam, -1.0);
        nh.param("fsm/init_bias_x", init_bias_x, -1.0);
        nh.param("fsm/init_bias_y", init_bias_y, -1.0);
        initCallback(nh);
        parallelInit(nh);
    }
    plan_manager::~plan_manager() {}

    void plan_manager::initCallback(ros::NodeHandle &nh)
    {
        particles_sub = nh.subscribe("/swarm_particles", 1000, &plan_manager::particlesCallback,this);
        traj_timer = nh.createTimer(ros::Duration(0.02), &plan_manager::timerCallback,this);
        realloca_timer = nh.createTimer(ros::Duration(0.05), &plan_manager::realloca_timerCallback, this);
        force_timer = nh.createTimer(ros::Duration(0.02), &plan_manager::forceCallback, this);
        traj_puber = nh.advertise<common_msgs::Swarm_traj>("/swarm_traj", 10, true);
        target_pub = nh.advertise<common_msgs::Swarm_particles>("/particle_target", 10);
        force_pub  = nh.advertise<common_msgs::Swarm_particles>("/particle_force", 10);
        goal_sub = nh.subscribe("/move_base_simple/goal", 1000, &plan_manager::goalCallback, this);
        path_vis = nh.advertise<visualization_msgs::Marker>("/path_vis", 10);
        traj_vis = nh.advertise<visualization_msgs::Marker>("/traj_vis", 10);
        collision_matrix_pub = nh.advertise<std_msgs::Int32MultiArray>("/check_collision_matrix", 10);
        collision_marker_pub = nh.advertise<visualization_msgs::Marker>("/particle_collision_lines", 1);
        obstacle_check_timer = nh.createTimer(ros::Duration(0.1), &plan_manager::obstacleCheckCallback, this);

        lastPlanTime = ros::Time::now();
        lastWaitOutputTime = ros::Time::now(); 

    }   

    void plan_manager::obstacleCheckCallback(const ros::TimerEvent&)
    {
        if (has_particle_data_) {
            checkBetweenObstacles(current_particles);
        }
    }

    void plan_manager::particlesCallback(const common_msgs::Swarm_particles::ConstPtr& msg)
    {
        if (isFirstCall) {
            init_particles = *msg; 
            isFirstCall = false;
            has_particle_data_ = true; 
        } 
        else {
            current_particles = *msg; 
        }

    }

    void plan_manager::realloca_timerCallback(const ros::TimerEvent&){}

    void plan_manager::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        ROS_INFO("Received goal: [%f, %f, %f]", msg->pose.position.x, 
                                                msg->pose.position.y, 
                                                msg->pose.position.z);
        particles_goal = init_particles;                                            
        for (size_t i = 0; i < particles_goal.particles.size(); i++)
        {
            particles_goal.particles[i].position.x += msg->pose.position.x + init_bias_x;
            particles_goal.particles[i].position.y += msg->pose.position.y + init_bias_y;
            particles_goal.particles[i].position.z += msg->pose.position.z;
            particles_goal.particles[i].index = init_particles.particles[i].index;

            ROS_INFO("Particle %d's goal: %f, %f,  %f",particles_goal.particles[i].index,
                                                       particles_goal.particles[i].position.x,
                                                       particles_goal.particles[i].position.y,
                                                       particles_goal.particles[i].position.z);
        }
        
        target_pub.publish(particles_goal);

        receive_goal = true;
    }

    void plan_manager::forceCallback(const ros::TimerEvent& event) 
    {        
        pubEsdfForce();
    }

    void plan_manager::timerCallback(const ros::TimerEvent& event) 
    {
        ros::Time currentTime = ros::Time::now(); 

        near_target = true;
        for (const auto& particle : current_particles.particles) {
            if (particle.state != NEAR_TARGET) {
                near_target = false;  // If any particle is not in NEAR_TARGET, set to false
                break;
            }
        }

        if (receive_goal)
        {   
            wait_target = false;
            receive_goal = false;
            for (size_t i = 0; i < swarmParticlesManager.size(); i++)
            {
                swarmParticlesManager[i].is_initialized = false;
            }
            // Start optimization
            ROS_INFO("\033[1;32m START OPTIMIZE_TRAJ! \033[0m");
            optTraj();
            ros::Time optfinishTime = ros::Time::now(); 
            double optcostTime = (optfinishTime - currentTime).toSec();
            ROS_INFO("\033[1;32m OPTIMIZE_FINISH, TOTAL TIME COST: %Fs \033[0m",optcostTime);
            exec_traj = true;
        }

        else if (wait_target || near_target)
        {
            double waitElapsedTime = (currentTime - lastWaitOutputTime).toSec();
            if (waitElapsedTime >= 1.0)
            {
                ROS_INFO("WAIT_TARGET");
                lastWaitOutputTime = currentTime;
            }
        }

        else if (exec_traj)
        {
            double elapsedTime = (currentTime - lastPlanTime).toSec();

            // Check if it's time to replan (every 0.5s or particle NEED_TRAJ)
            if (elapsedTime >= planInterval)
            {
                exec_traj = false;
                need_replan = true;
                ROS_INFO("\033[1;32m TRAJ NEED REPLAN, RETURN. \033[0m");
                lastPlanTime = currentTime;
                return;
            }

            double waitElapsedTime = (currentTime - lastWaitOutputTime).toSec();
            if (waitElapsedTime >= 1.0)
            {
                ROS_INFO("EXEC_TRAJ");
                lastWaitOutputTime = currentTime;
            }
        }

        else if (need_replan)
        {
            ROS_INFO("\033[1;32m REPLAN OPTIMIZE_TRAJ! \033[0m");
            optTraj();
            ros::Time optfinishTime = ros::Time::now(); 
            double optcostTime = (optfinishTime - currentTime).toSec();
            ROS_INFO("\033[1;32m OPTIMIZE_FINISH, TOTAL TIME COST: %Fs \033[0m",optcostTime);

            exec_traj = true;
            need_replan = false;
        }
    }
    void plan_manager::checkBetweenObstacles(const common_msgs::Swarm_particles& current_particles)
    {
        int num_particles = current_particles.particles.size();
        Eigen::MatrixXi collisionMatrix = Eigen::MatrixXi::Zero(num_particles, num_particles);

        visualization_msgs::Marker line_list;
        line_list.header.frame_id = "world"; 
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "particle_lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.03; 

        line_list.color.r = 0.53f;
        line_list.color.g = 0.81f;
        line_list.color.b = 0.98f;
        line_list.color.a = 0.8f;

        for (int i = 0; i < num_particles; ++i) {
            const auto& particle_i = current_particles.particles[i];
            int particleIndex_i = particle_i.index;

            auto it_i = std::find_if(swarmParticlesManager.begin(), swarmParticlesManager.end(),
                                    [&particleIndex_i](const particleManager& manager) {
                                        return manager.particle_index == std::to_string(particleIndex_i);
                                    });

            if (it_i == swarmParticlesManager.end()) {
                ROS_WARN("Particle manager not found for particle index: %d", particleIndex_i);
                continue;
            }

            Eigen::Vector3d position_i(particle_i.position.x, particle_i.position.y, particle_i.position.z);

            for (int j = 0; j < num_particles; ++j) {
                if (i == j) continue; 

                const auto& particle_j = current_particles.particles[j];
                Eigen::Vector3d position_j(particle_j.position.x, particle_j.position.y, particle_j.position.z);

                Eigen::Vector3d direction = position_j - position_i;
                double distance = direction.norm();
                int num_samples = std::ceil(distance / 0.1); 
                Eigen::Vector3d step = direction.normalized() * 0.1;

                bool obstacle_free = true;
                Eigen::Vector3d sample_point = position_i;

                for (int k = 1; k <= num_samples; ++k) {
                    sample_point += step;
                    int occupancy = it_i->sdf_map_->getInflateOccupancy(sample_point);

                    if (occupancy == 1) {
                        obstacle_free = false;
                        break;
                    }
                }

                collisionMatrix(particle_i.index, particle_j.index) = obstacle_free ? 0 : 1;

                if (obstacle_free) {
                    geometry_msgs::Point p_start, p_end;
                    p_start.x = position_i.x();
                    p_start.y = position_i.y();
                    p_start.z = position_i.z();

                    p_end.x = position_j.x();
                    p_end.y = position_j.y();
                    p_end.z = position_j.z();

                    line_list.points.push_back(p_start);
                    line_list.points.push_back(p_end);
                }
                     
            }
        }

        std_msgs::Int32MultiArray collision_matrix_msg;
        collision_matrix_msg.layout.dim.resize(2);
        collision_matrix_msg.layout.dim[0].label = "particles";
        collision_matrix_msg.layout.dim[0].size = num_particles;
        collision_matrix_msg.layout.dim[0].stride = num_particles * num_particles;
        collision_matrix_msg.layout.dim[1].label = "neighbors";
        collision_matrix_msg.layout.dim[1].size = num_particles;
        collision_matrix_msg.layout.dim[1].stride = num_particles;

        collision_matrix_msg.data.resize(num_particles * num_particles);
        for (int i = 0; i < num_particles; ++i) {
            for (int j = 0; j < num_particles; ++j) {
                collision_matrix_msg.data[i * num_particles + j] = collisionMatrix(i, j);
            }
        }

        collision_matrix_pub.publish(collision_matrix_msg);

        //visulization
        collision_marker_pub.publish(line_list);

    }

    void plan_manager::update(const common_msgs::Swarm_particles& particles) 
    {
        current_particles = particles;  
    }
    
    void plan_manager::pubEsdfForce()
    {
        particles_force.particles.clear();
        for (const auto& particle : current_particles.particles) {
            int particleIndex = particle.index;

            auto it = std::find_if(swarmParticlesManager.begin(), swarmParticlesManager.end(),
                                [&particleIndex](const particleManager& manager) {
                                    return manager.particle_index == std::to_string(particleIndex);
                                });

            if (it == swarmParticlesManager.end()) {
                ROS_WARN("Particle manager not found for particle index: %d", particleIndex);
                continue;
            }

            std::shared_ptr<bspline_optimizer> bspline_opt_ = it->bspline_opt_;

            Eigen::Vector3d particle_position;
            particle_position.x() = particle.position.x;
            particle_position.y() = particle.position.y;
            particle_position.z() = particle.position.z;

            common_msgs::Force esdf_force = bspline_opt_->calcGradForce(particle_position);

            common_msgs::Particle particle_with_force;
            particle_with_force.index = particleIndex;
            particle_with_force.force = esdf_force;

            particles_force.particles.push_back(particle_with_force);
        }

        force_pub.publish(particles_force);
    }

    void plan_manager::processParticle(size_t index, const common_msgs::Swarm_particles& particles, 
                                       const common_msgs::Swarm_particles& particles_goal, 
                        std::vector<particleManager>& swarmParticlesManager, 
                        ros::Publisher& path_vis, std::mutex& mtx) {
        Eigen::MatrixXd initial_state(3,3),terminal_state(3,3);//初始，结束P V A
        Eigen::Vector3d start_pt, end_pt, goal_pt, start_v, end_v, start_a;
        common_msgs::Force particle_force;

        swarmParticlesManager[index].curr_time = ros::Time::now();
        if (!swarmParticlesManager[index].is_initialized)
        {
             swarmParticlesManager[index].last_time = ros::Time::now();
        }        
        double time_diff = (swarmParticlesManager[index].curr_time - swarmParticlesManager[index].last_time).toSec();
        int index_to_remove = static_cast<int>(time_diff * swarmParticlesManager[index].spline_->TrajSampleRate) + 5;

        if (!swarmParticlesManager[index].is_initialized) {
            //start_pt, start_v, start_a
            start_pt.x() = particles.particles[index].position.x + 0.000001; // if start is zero, a_star bug
            start_pt.y() = particles.particles[index].position.y + 0.000001;
            start_pt.z() = particles.particles[index].position.z + 0.000001;

            start_v.x() = particles.particles[index].velocity.x;
            start_v.y() = particles.particles[index].velocity.y;
            start_v.z() = particles.particles[index].velocity.z;

            start_a.x() = particles.particles[index].acceleration.x;
            start_a.y() = particles.particles[index].acceleration.y;
            start_a.z() = particles.particles[index].acceleration.z;

            swarmParticlesManager[index].is_initialized = true;
        } 
        else if (index_to_remove >= swarmParticlesManager[index].particle_traj.position.size())
        {
            // ROS_INFO("NEAR TARGET, RETURN.");
            return;
        }
        else {
            swarmParticlesManager[index].particle_traj.position.erase(swarmParticlesManager[index].particle_traj.position.begin(), 
                                                        swarmParticlesManager[index].particle_traj.position.begin() + index_to_remove);
            swarmParticlesManager[index].particle_traj.velocity.erase(swarmParticlesManager[index].particle_traj.velocity.begin(), 
                                                        swarmParticlesManager[index].particle_traj.velocity.begin() + index_to_remove);
            swarmParticlesManager[index].particle_traj.acceleration.erase(swarmParticlesManager[index].particle_traj.acceleration.begin(), 
                                                        swarmParticlesManager[index].particle_traj.acceleration.begin() + index_to_remove);
            swarmParticlesManager[index].particle_traj.jerk.erase(swarmParticlesManager[index].particle_traj.jerk.begin(), 
                                                        swarmParticlesManager[index].particle_traj.jerk.begin() + index_to_remove);

            start_pt.x() = swarmParticlesManager[index].particle_traj.position.front().x;
            start_pt.y() = swarmParticlesManager[index].particle_traj.position.front().y;
            start_pt.z() = swarmParticlesManager[index].particle_traj.position.front().z;

            start_v.x() = swarmParticlesManager[index].particle_traj.velocity.front().x;
            start_v.y() = swarmParticlesManager[index].particle_traj.velocity.front().y;
            start_v.z() = swarmParticlesManager[index].particle_traj.velocity.front().z;

            start_a.x() = swarmParticlesManager[index].particle_traj.acceleration.front().x;
            start_a.y() = swarmParticlesManager[index].particle_traj.acceleration.front().y;
            start_a.z() = swarmParticlesManager[index].particle_traj.acceleration.front().z;
        }

        Eigen::Vector3d currentPos(particles.particles[index].position.x,
                                particles.particles[index].position.y,
                                particles.particles[index].position.z);


        if ((start_pt - currentPos).norm() >1.0) {
            start_pt = currentPos;
        }
        if (start_pt.z()<0)
        {
            start_pt.z() = 0.1;
        }
        
        // Find the matching particle in particles_goal based on the index
        int current_index = particles.particles[index].index;
        for (const auto& goal_particle : particles_goal.particles) {
            if (goal_particle.index == current_index) {
                // Assign end_pt using goal_particle's position
                goal_pt.x() = goal_particle.position.x;
                goal_pt.y() = goal_particle.position.y;
                goal_pt.z() = goal_particle.position.z;
                break;
            }
        }
        // Reset pathfinder and search for the path
        // A*

        //check start and end,if occ, select another one.
        int occupancy1 = swarmParticlesManager[index].sdf_map_->getOccupancy(start_pt);
        int occupancy2 = swarmParticlesManager[index].sdf_map_->getOccupancy(goal_pt);

        // 如果起点被占据，则找到最近的未占据点作为新起点
        if (occupancy1 == 1) {
            Eigen::Vector3d nearest_free_start;
            if (swarmParticlesManager[index].sdf_map_->getNearestFreePoint(start_pt, nearest_free_start)) {
                start_pt = nearest_free_start;
                ROS_WARN("aaaaaaaaaaaaaaaaa");
            } else {
                ROS_WARN("无法找到合适的起点！");
                return;
            }
        }
        if (occupancy2 == 1) {
            Eigen::Vector3d nearest_free_goal;
            if (swarmParticlesManager[index].sdf_map_->getNearestFreePoint(goal_pt, nearest_free_goal)) {
                goal_pt = nearest_free_goal;
                ROS_WARN("aaaaaaaaaaaaaaaaa");
            } else {
                ROS_WARN("无法找到合适的终点！");
                return;
            }
        }

        swarmParticlesManager[index].geo_path_finder_->reset();
        swarmParticlesManager[index].geo_path_finder_->search(start_pt, goal_pt, false, -1.0);
        std::vector<Eigen::Vector3d> path_points = swarmParticlesManager[index].geo_path_finder_->getprunePath();
        std::vector<Eigen::Vector3d> initial_ctrl_ps;
        int num_points_to_take = std::min(static_cast<int>(path_points.size()), 10);
        for (int i = 0; i < num_points_to_take; ++i) {
            initial_ctrl_ps.push_back(path_points[i]);
        }
            visualizePath(path_points, path_vis, swarmParticlesManager[index].particle_index);
        if (initial_ctrl_ps.empty()) {
            return;
        }
        end_pt = initial_ctrl_ps.back();

        initial_state <<    start_pt(0), start_pt(1), start_pt(2),
                            start_v(0),  start_v(1), start_v(2),
                            start_a(0),  start_a(1),start_a(2);

        terminal_state <<   end_pt(0), end_pt(1),end_pt(2),
                            0.0, 0.0,0.0,
                            0.0, 0.0,0.0;

        swarmParticlesManager[index].bspline_opt_->set3DPath2(initial_ctrl_ps);
        swarmParticlesManager[index].spline_->setIniandTerandCpsnum(initial_state,terminal_state,
                                                            swarmParticlesManager[index].bspline_opt_->cps_num_);
        if(swarmParticlesManager[index].bspline_opt_->cps_num_ <= 2*swarmParticlesManager[index].spline_->p_)
        {
            return;
        }
        UniformBspline spline = *swarmParticlesManager[index].spline_;
        swarmParticlesManager[index].bspline_opt_->setSplineParam(spline);
        swarmParticlesManager[index].bspline_opt_->optimize();

        swarmParticlesManager[index].spline_->setControlPoints(swarmParticlesManager[index].bspline_opt_->control_points_);
        swarmParticlesManager[index].spline_->getT();
        UniformBspline p = *swarmParticlesManager[index].spline_;
        UniformBspline v = p.getDerivative();
        UniformBspline a = v.getDerivative();
        UniformBspline j = a.getDerivative();

        //traj
        Eigen::MatrixXd p_ = p.getTrajectory(p.time_);
        Eigen::MatrixXd v_ = v.getTrajectory(p.time_);
        Eigen::MatrixXd a_ = a.getTrajectory(p.time_);
        Eigen::MatrixXd j_ = j.getTrajectory(p.time_);

        common_msgs::BsplineTraj traj_;
        std::vector<Eigen::Vector3d> vis_traj,vis_vel;
        traj_.traj_id = std::stoi(swarmParticlesManager[index].particle_index);
        int N = p_.rows();  
        traj_.position.resize(N);
        traj_.velocity.resize(N);
        traj_.acceleration.resize(N);
        traj_.jerk.resize(N);
        vis_traj.resize(N);
        vis_vel.resize(N);
        
        for (int i = 0; i < N; ++i) {

            geometry_msgs::Point pos;
            pos.x = p_(i, 0);  
            pos.y = p_(i, 1);  
            pos.z = p_(i, 2);       
            traj_.position[i] = pos;

            vis_traj[i].x() = p_(i, 0);
            vis_traj[i].y() = p_(i, 1);
            vis_traj[i].z() = p_(i, 2);

            geometry_msgs::Point vel;
            vel.x = v_(i, 0); 
            vel.y = v_(i, 1); 
            vel.z = v_(i, 2);       
            traj_.velocity[i] = vel;

            vis_vel[i].x() = v_(i, 0);
            vis_vel[i].y() = v_(i, 1);
            vis_vel[i].z() = v_(i, 2);

            geometry_msgs::Point acc;
            acc.x = a_(i, 0);  
            acc.y = a_(i, 1);  
            acc.z = a_(i, 2);      
            traj_.acceleration[i] = acc;

            geometry_msgs::Point jerk;
            jerk.x = j_(i, 0);  
            jerk.y = j_(i, 1);  
            jerk.z = j_(i, 2);       
            traj_.jerk[i] = jerk;
        }

        visualizeTraj(vis_traj, traj_vis, swarmParticlesManager[index].particle_index);

        traj_.header.frame_id = "world";
        traj_.header.stamp = ros::Time::now();
        
        swarmParticlesManager[index].particle_traj = traj_;
        {       
            std::lock_guard<std::mutex> lk(muxSwarm_traj);
            swarm_traj.traj.push_back(traj_);
        }

        swarmParticlesManager[index].last_time = ros::Time::now();
    }

    void plan_manager::optTraj()
    {
        size_t num_particles = current_particles.particles.size();
        std::vector<std::thread> threads;

        {
            std::lock_guard<std::mutex> lk(muxSwarm_traj);
            swarm_traj.traj.clear();
        }
        swarm_traj.header.frame_id = "world";
        swarm_traj.header.stamp = ros::Time::now();
        // Create a thread for each particle
        for (size_t i = 0; i < num_particles; ++i) {
            threads.emplace_back(std::bind(&plan_manager::processParticle, this, i, 
                                            std::cref(current_particles), std::cref(particles_goal),
                                            std::ref(swarmParticlesManager), std::ref(path_vis), std::ref(mtx)));
        }

        // Join all threads
        for (auto& thread : threads) {
            thread.join();
        }

        traj_puber.publish(swarm_traj);
    }

    void plan_manager::parallelInit(ros::NodeHandle &nh) {
        ros::master::V_TopicInfo topic_list;
        ros::master::getTopics(topic_list);
        std::regex topic_pattern("/particle(\\d+)/odom");
        std::smatch match;

        for (const auto &info : topic_list) {
            if (info.datatype == "nav_msgs/Odometry" && std::regex_search(info.name, match, topic_pattern)) {
                try {
                    std::string particle_index = match[1];
                    std::string particle_base = "/particle" + particle_index;
                    std::string odom_topic = particle_base + "/odom";
                    std::string cloud_topic = particle_base + "/local_map";

                    std::cout << "\033[1;33m" << particle_base << "  init: "<< "\033[0m" << std::endl;

                    // //EDT & MAP
                    auto sdf_map_ = std::make_shared<SDFMap>();
                    sdf_map_->initMap(nh, particle_base, odom_topic, cloud_topic);
                    auto edt_environment_ = std::make_shared<EDTEnvironment>();
                    edt_environment_->setMap(sdf_map_);
                    
                    //ASTAR
                    auto geo_path_finder_ = std::make_shared<Astar>();
                    geo_path_finder_->setParam(nh);
                    geo_path_finder_->setEnvironment(edt_environment_);
                    geo_path_finder_->init();

                    //OPT
                    auto bspline_opt_ = std::make_shared<bspline_optimizer>();
                    bspline_opt_->init(nh);
                    bspline_opt_->setEnvironment(edt_environment_);

                    //UNIFORM BSPLINE
                    auto spline_ = std::make_shared<UniformBspline>();
                    spline_->init(nh);

                    particleManager pm {
                        particle_index,
                        sdf_map_,
                        edt_environment_,
                        geo_path_finder_,
                        bspline_opt_,
                        spline_
                    };

                    swarmParticlesManager.push_back(pm);  // 将实例存入向量

                    std::cout << "\033[1;33m" << "-----------------------------------------" << "\033[0m" << std::endl;

                } catch (const std::exception& e) {
                    ROS_ERROR("Exception caught while initializing environments for %s: %s", info.name.c_str(), e.what());
                } catch (...) {
                    ROS_ERROR("Unknown exception caught during initialization for %s", info.name.c_str());
                }
            }
        }
    }

    void plan_manager::visualizePath(const std::vector<Eigen::Vector3d>& path_points, ros::Publisher& marker_pub, const std::string& particle_index) {
        // Lock the mutex to safely visualize the path
        std::lock_guard<std::mutex> lock(mtx); 

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world"; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "path_visualization";
    
        std::hash<std::string> hash_fn;
        marker.id = static_cast<int>(hash_fn(particle_index));

        marker.type = visualization_msgs::Marker::POINTS; 
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1; 
        marker.scale.y = 0.1;

        for (size_t i = 0; i < path_points.size(); ++i) {
            geometry_msgs::Point p;
            p.x = path_points[i].x();
            p.y = path_points[i].y();
            p.z = path_points[i].z();

            marker.points.push_back(p);

            std_msgs::ColorRGBA color;
            if (i == path_points.size() - 1) {
                color.r = 0.0;
                color.g = 1.0;
                color.b = 0.0;
                color.a = 1.0;

            } else {
                color.r = 1.0;
                color.g = 0.0;
                color.b = 0.0;
                color.a = 1.0;
            }

            marker.colors.push_back(color);
        }

        marker_pub.publish(marker);
    }

    void plan_manager::visualizeTraj(const std::vector<Eigen::Vector3d>& traj, ros::Publisher& marker_pub, const std::string& particle_index)
    {
        // Lock the mutex to safely visualize the traj
        std::lock_guard<std::mutex> lock(mtx);  // 锁定

        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "traj_visualization";
        
        std::hash<std::string> hash_fn;
        marker.id = static_cast<int>(hash_fn(particle_index));
        
        marker.type = visualization_msgs::Marker::POINTS; 
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.1; 
        marker.scale.y = 0.1;

        for (size_t i = 0; i < static_cast<size_t>(trajVisParam * traj.size()); ++i) {
            geometry_msgs::Point p;
            p.x = traj[i].x();
            p.y = traj[i].y();
            p.z = traj[i].z();

            marker.points.push_back(p);

            std_msgs::ColorRGBA color;
            color.r = 1.0; 
            color.g = 0.0;  
            color.b = 1.0;  
            color.a = 1.0;  
        
            marker.colors.push_back(color);
        }

        marker_pub.publish(marker);
    }

    std::vector<int> plan_manager::hungarianAlgorithm(const Eigen::MatrixXd& costMatrix){}
    void plan_manager::testInit(ros::NodeHandle& nh){}

}