#ifndef  _BSPLINE_OPT_H
#define  _BSPLINE_OPT_H

//Eigen
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>

//STANDARD
#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>
#include <numeric>
#include <string>
#include <memory>

//ROS
#include <ros/ros.h>

//Nlopt optimization
#include <nlopt.hpp>

//自定义
#include <bspline_race/UniformBspline_3d.h>
#include <plan_env/edt_environment.h>
#include "common_msgs/Force.h"
#include "common_msgs/BsplineTraj.h"

using namespace std;

namespace FLAG_Race
{
       class bspline_optimizer
    {
        public:
            int cps_num_;
            int p_order_;
            int Dim_;
            double bspline_interval_;
            double beta_;
            UniformBspline u_;

            double lambda1_,lambda2_,lambda3_;// smooth cost, ESDF cost, feasibility cost
            double max_vel_;
            double max_acc_;
            double opt_maxtime;
            int    opt_maxeval;
           
            std::vector<Eigen::Vector2d> path_;
            std::vector<Eigen::Vector3d> path3D_;
            //ESDF
            Eigen::MatrixXd esdf_map_;
            double map_resolution_;
            double origin_x_, origin_y_;
            double startPoint_x,startPoint_y;

            //nlopt optimizaiton
            Eigen::MatrixXd control_points_;
            int iter_num_;       // iteration of the solver
            int variable_num;
            std::vector<double> best_variable_;
            double safe_distance_;
            double k_force;
            std::mutex mtx;
            double min_cost_;       //
            int    algorithm1_ = 15;             // optimization algorithms for quadratic cost
            int    algorithm2_ = 11;             // optimization algorithms for general cost

            // std::shared_ptr<EDTEnvironment> edt_environment_;
            EDTEnvironment::Ptr edt_environment_;
            
            ros::Publisher marker_pub;
        public:
            bspline_optimizer() {}
            bspline_optimizer(const std::vector<Eigen::Vector2d> &path, const int&Dim,const int &p);
            bspline_optimizer(const int&Dim,const int &p,const double &dist);
            ~bspline_optimizer();

            void init(ros::NodeHandle& nh);
            void setOptParam(const double lambda1,const double lambda2,const double lambda3,
                                                    const double safe_dist);
            void setMapParam(const double &origin_x,const double &origin_y, const double &map_resolution,
                                                    const double &start_x, const double &start_y);
            void setVelAcc(const double vel, const double acc);
            void setSmoothParam(const double lambda1,const double lambda2,
                                                            const double vel, const double acc);
            void setSplineParam(const UniformBspline &u);
            void setEnvironment(const EDTEnvironment::Ptr& env);
            //从mapping读入
            void setEsdfMap(const Eigen::MatrixXd &esdf_map);
            void initialControlPoints(UniformBspline u);

            inline void setDimandOrder(const int&Dim, const int&p)
            {
                Dim_  = Dim;
                p_order_ = p;
            }
            inline void setPath(const std::vector<Eigen::Vector2d> &path)
            {
                path_.clear();
                path_ = path;
                cps_num_ = path.size() + 2*p_order_ -2;
            }

            inline void set3DPath(const std::vector<Eigen::Vector3d> &path) {
                path_.clear(); 

                for (const auto& point : path) {
                    path_.emplace_back(point.x(), point.y()); 
                }
                cps_num_ = path_.size() + 2 * p_order_ - 2;
            }

            inline void set3DPath2(const std::vector<Eigen::Vector3d> &path) {
                path3D_.clear(); 

                for (const auto& point : path) {
                    path3D_.emplace_back(point.x(), point.y(),point.z()); 
                }
                cps_num_ = path3D_.size() + 2 * p_order_ - 2;
            }

   
            void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                                                    Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
            void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient);
            void calcEsdfCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient);   
            double calcDistance(const Eigen::MatrixXd &q);
            Eigen::Vector2d calcGrad(const Eigen::MatrixXd &q);   
            void getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff);
            void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
            void interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, Eigen::Vector2d& grad);
            void interpolateBilinearDist(double values[2][2], const Eigen::Vector2d& diff, double& dist);    
            void combineCost( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine);
            void combineCostSmooth( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine);
            static double costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                                        void* func_data);
            static double costFunctionSmooth(const std::vector<double>& x, std::vector<double>& grad,
                                                        void* func_data);
            void optimize();
            void optimize_withoutesdf();
            void optimizesmooth();//const std::shared_ptr u
            template <typename T> std::vector<size_t> sort_indexes(std::vector<T> &v)
            {   
                std::vector<size_t> idx(v.size());
                iota(idx.begin(), idx.end(), 0);
                sort(idx.begin(), idx.end(),
                [&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });
                return idx;
            }
            template<typename T>  inline T lerp(const T &lo, const T &hi, float t)  { return (lo * (0.1 - t) + hi * t)*10; }

            common_msgs::Force calcGradForce(const Eigen::Vector3d& q_3d);
            double mapForceToKDen(double force_magnitude);
            bool checkTrajCollision(const common_msgs::BsplineTraj traj);
            void publishTrajectory(const std::vector<double> &control_points);
    };
}

#endif