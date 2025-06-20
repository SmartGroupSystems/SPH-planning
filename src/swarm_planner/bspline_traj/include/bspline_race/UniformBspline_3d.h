#ifndef  _UNIFORMBSPLINE_3D_H
#define _UNIFORMBSPLINE_3D_H

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
#include <ros/ros.h>

using namespace std;

namespace FLAG_Race
{
    class UniformBspline
    {
        public://
        int  p_, n_, m_;//p degree， n+1 is the number of control points, m = n+p+1
        Eigen::VectorXd u_ ; // knot vectors
        double beta_;// time scale t(real time) * beta = u
        int D_;// Dimension of control points
        double dist_p;
        double max_vel_;
        int TrajSampleRate;
        public:
            Eigen::MatrixXd control_points_;
            Eigen::MatrixXd A_ini, A_ter; // A_ini*[cp1, cp2, cp3] = [p0, v0, a0] ,similarly A_ter; DIM: 3*3
            Eigen::MatrixXd s_ini_, s_ter_; // initial state, terminal state   DIM: 3*2
            Eigen::Vector2d s_range;
            Eigen::Vector2d t_range;
            double interval_;
            Eigen::VectorXd time_;

        public:
            UniformBspline() {}
            
            UniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                            const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter);
            ~UniformBspline();
            void init(ros::NodeHandle& nh);
            void initUniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                            const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter); 
            void setControlPoints(const Eigen::MatrixXd &ctrl_points);
            void setIniTerMatrix();
            inline void setOrderandBetaandDim(const int &p,const double &beta, const int &D)
            {
                p_ = p; 
                beta_ = beta;
                D_ =D;
            }
            inline void setIniandTerandCpsnum(const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter,const int &n)
            {
                
                s_ini_ = s_ini;
                s_ter_ = s_ter;
                n_ = n-1;
                m_ = p_+n_+1;
                u_ = Eigen::VectorXd::Zero(m_ + 1); //u0 ~ um 
                control_points_ = Eigen::MatrixXd::Zero(n_+1,D_);
                for(int i = 0; i<=m_; i++)
                {
                    u_(i) = i;
                }
                setIniTerMatrix();
                getAvailableSrange();
                getAvailableTrange();
                getInterval();
            }
            Eigen::MatrixXd getTrajectory(const Eigen::VectorXd &t);
            Eigen::Vector3d singleDeboor(const double &u_probe);
            void getAvailableSrange();
            void getAvailableTrange();
            void getInterval();
            void getT();
            UniformBspline getDerivative();
            Eigen::VectorXd getBoundConstraintb();
    };
}


#endif