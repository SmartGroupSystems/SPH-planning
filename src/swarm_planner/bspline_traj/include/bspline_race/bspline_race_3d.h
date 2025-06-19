#ifndef  _BSPLINE_RACE_3D_H
#define  _BSPLINE_RACE_3D_H

//standard
#include <fstream>
#include <string>
#include <regex>
#include <algorithm>
#include <iostream>
#include <math.h>
#include <numeric>
#include <memory>
#include <thread>
#include <mutex>
#include <vector>
#include <Eigen/Dense>

//ros
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Imu.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <ros/topic_manager.h>
#include <std_msgs/String.h>
#include <bspline_race/UniformBspline_3d.h>
#include <bspline_race/bspline_opt.h>
#include "common_msgs/common_msgs.h"
#include <plan_env/edt_environment.h>
#include <path_searching/astar.h>


using namespace std;

struct Point { double x, y, z;};

enum ParticleState {
    NULL_STATE,  
    TRAJ,       
    NEED_TRAJ, 
    ATTRACT,     
    REPEL,       
    NEAR_TARGET 
};

enum ParticleRole {
    LEADER,
    FOLLOWER,
    FREE
};

namespace FLAG_Race
{
    class plan_manager
    {
        public:
            double planInterval;
            double trajVisParam;
            std::string cloud_topic_;
            std::mutex mtx;
            double init_bias_x, init_bias_y;

            std::shared_ptr<UniformBspline> u;

            struct particleManager {
                std::string particle_index;
                std::shared_ptr<SDFMap> sdf_map_;
                std::shared_ptr<EDTEnvironment> edt_environment_;
                std::shared_ptr<Astar> geo_path_finder_;
                std::shared_ptr<bspline_optimizer> bspline_opt_;
                std::shared_ptr<UniformBspline> spline_;
                common_msgs::BsplineTraj particle_traj;
                ros::Time curr_time;  
                ros::Time last_time; 
                bool is_initialized = false; 
            };
            std::vector<particleManager> swarmParticlesManager;

            
            //Traj
            std::mutex muxSwarm_traj;
            common_msgs::Swarm_traj swarm_traj;

            //Particles
            bool isFirstCall = true; 
            bool has_particle_data_ = false;
            common_msgs::Swarm_particles current_particles;
            common_msgs::Swarm_particles init_particles;
            common_msgs::Swarm_particles particles_goal;
            common_msgs::Swarm_particles particles_force;
            std::shared_ptr<ros::AsyncSpinner> force_spinner;

        public:
            //ROS
            ros::Subscriber particles_sub; 
            ros::Publisher  traj_vis;
            ros::Publisher  traj_puber;
            ros::Publisher  waypoint_vis;  
            ros::Publisher  target_pub;
            ros::Publisher  force_pub;
            ros::Timer      traj_timer;
            ros::Timer      realloca_timer;
            ros::Timer      force_timer;
            ros::Subscriber goal_sub;
            ros::Publisher  path_vis;
            ros::Time       lastPlanTime;
            ros::Time       lastWaitOutputTime;
            ros::Publisher  collision_matrix_pub;
            ros::Publisher  collision_marker_pub;
            ros::Timer      obstacle_check_timer;

            //fsm
            bool receive_goal = false;
            bool exec_traj = false;
            bool need_replan = false;
            bool near_target = false;
            bool wait_target = true;
        public:
            plan_manager(){};  
            plan_manager(ros::NodeHandle& nh); 
            ~plan_manager();
            void testInit(ros::NodeHandle& nh); 
            void initCallback(ros::NodeHandle &nh);
            common_msgs::BsplineTraj getSmoothTraj(const std::vector<Point> waypoints);
            void optTraj();
            void parallelInit(ros::NodeHandle &nh);
            void update(const common_msgs::Swarm_particles& particles);
            void particlesCallback(const common_msgs::Swarm_particles::ConstPtr& msg);
            void timerCallback(const ros::TimerEvent& event);
            void forceCallback(const ros::TimerEvent& event);
            void realloca_timerCallback(const ros::TimerEvent&);
            void obstacleCheckCallback(const ros::TimerEvent&);
            void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
            void visualizePath(const std::vector<Eigen::Vector3d>& path_points, ros::Publisher& marker_pub, const std::string& particle_index);
            void visualizeTraj(const std::vector<Eigen::Vector3d>& traj, ros::Publisher& marker_pub, const std::string& particle_index);
            void processParticle(size_t index, const common_msgs::Swarm_particles& init_particles, 
                                       const common_msgs::Swarm_particles& particles_goal, 
                        std::vector<particleManager>& swarmParticlesManager, 
                        ros::Publisher& path_vis, std::mutex& mtx); 
            void pubEsdfForce();
            void checkBetweenObstacles(const common_msgs::Swarm_particles& particles);
            std::vector<int> hungarianAlgorithm(const Eigen::MatrixXd& costMatrix);
    };

}
#endif
