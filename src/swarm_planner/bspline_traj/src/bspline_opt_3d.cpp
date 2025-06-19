#include <bspline_race/bspline_opt.h>

namespace FLAG_Race
{
    bspline_optimizer::bspline_optimizer(const std::vector<Eigen::Vector2d> &path, const int&Dim,  const int&p)
    {
        path_.clear();
        path_ = path;
        Dim_  = Dim;
        p_order_ = p;
        cps_num_ = path.size() + 2*p_order_ -2;
    }

    bspline_optimizer::bspline_optimizer(const int&Dim,  const int&p, const double &dist)
    {
        Dim_  = Dim;
        p_order_ = p;
        cps_num_ = 2*p_order_+floor(dist/1.0);
    }
    
    bspline_optimizer::~bspline_optimizer(){}
    
    void bspline_optimizer::init(ros::NodeHandle& nh)
    {
        nh.param("planning/traj_order", p_order_, 3);
        nh.param("planning/dimension", Dim_, -1);
        nh.param("planning/max_vel", max_vel_, -1.0);
        nh.param("planning/max_acc", max_acc_, -1.0);
        nh.param("planning/lambda1",lambda1_,-1.0);
        nh.param("planning/lambda2",lambda2_,-1.0);
        nh.param("planning/lambda3",lambda3_,-1.0);
        nh.param("planning/safe_distance",safe_distance_,-1.0);
        nh.param("planning/k_force",k_force,-1.0);
        nh.param("planning/opt_maxeval",opt_maxeval,500);
        nh.param("planning/opt_maxtime",opt_maxtime,0.2);
        std::cout << "\033[1;32m" << "success init Opt module" << "\033[0m" << std::endl;
        marker_pub = nh.advertise<visualization_msgs::Marker>("/optimization_path", 1);
    }

    void bspline_optimizer::setOptParam(const double lambda1,const double lambda2,const double lambda3,
                                                                                    const double safe_dist)
    {
            lambda1_ = lambda1;
            lambda2_ = lambda2;
            lambda3_ = lambda3;
            safe_distance_ = safe_dist;
    }

    void bspline_optimizer::setVelAcc(const double vel, const double acc)
    {
            max_vel_ = vel;
            max_acc_ = acc;
    }

    void bspline_optimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
        this->edt_environment_ = env;
    }

    void bspline_optimizer::setSmoothParam(const double lambda1,const double lambda2,
                                                            const double vel, const double acc)
    {
            lambda1_ = lambda1;
            lambda2_ = lambda2;
            max_vel_ = vel;
            max_acc_ = acc;
    }

    void bspline_optimizer::setEsdfMap(const Eigen::MatrixXd &esdf_map)
    {    
        esdf_map_ = esdf_map;
    }

    void bspline_optimizer::setMapParam(const double &origin_x,const double &origin_y, const double &map_resolution,
                                                                                    const double &start_x, const double &start_y)
    {
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        map_resolution_ = map_resolution;
        startPoint_x = start_x;
        startPoint_y = start_y;
    }

    void bspline_optimizer::setSplineParam(const UniformBspline &u)
    {
        u_ = u;
        bspline_interval_  = u.interval_;
        beta_ = u.beta_;
        control_points_.resize(cps_num_,Dim_);
        Eigen::VectorXd beq_bound = u_.getBoundConstraintb();
        
        for (size_t i = 0; i < Dim_; i++)
        {
                for (size_t j = 0; j < p_order_; j++)
                {
                     control_points_(j,i) = beq_bound(i*cps_num_+j);
                     control_points_((1)*cps_num_-j-1,i) = beq_bound((i+1)*cps_num_-j-1);
                }
        }
            
        for (size_t i = 0; i < cps_num_-2*p_order_; i++)
        {
            control_points_.row(i+p_order_) = path3D_[i+1];

        }
        // std::cout << "control_points_: \n" << control_points_ << std::endl;
    }

    void bspline_optimizer::initialControlPoints(UniformBspline u)
    {
        control_points_.setZero(cps_num_,Dim_);
        Eigen::VectorXd beq_bound = u.getBoundConstraintb();
        for (size_t i = 0; i < Dim_; i++)
        {
        for (size_t j = 0; j < p_order_; j++)
        {
        control_points_(j,i) = beq_bound(i*cps_num_+j);
        control_points_(cps_num_-j-1,i) = beq_bound((i+1)*cps_num_-j-1);
        }
        }
        int insert_num = cps_num_-2*p_order_;
        Eigen::Vector2d start_pos = control_points_.row(p_order_-1);
        Eigen::Vector2d end_pos = control_points_.row(cps_num_-p_order_);
        for (size_t i = 0; i < cps_num_-2*p_order_; i++)
        {
        control_points_(i+p_order_,0) = start_pos(0)+(end_pos(0)-start_pos(0))/(insert_num+1)*(i+1) ;
        control_points_(i+p_order_,1) = start_pos(1)+(end_pos(1)-start_pos(1))/(insert_num+1)*(i+1) ;
        }
       // cout<<control_points_<<endl;
    }

    void bspline_optimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                            Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
    {
        cost = 0.0;
        if (falg_use_jerk)
        {
            Eigen::Vector3d jerk, temp_j;

            for (int i = 0; i < q.cols() - 3; i++)
            {
                /* evaluate jerk */
                jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
                cost += jerk.squaredNorm();
                temp_j = 2.0 * jerk;
                /* jerk gradient */
                gradient.col(i + 0) += -temp_j;
                gradient.col(i + 1) += 3.0 * temp_j;
                gradient.col(i + 2) += -3.0 * temp_j;
                gradient.col(i + 3) += temp_j;
            }
        }
        else
        {
            Eigen::Vector3d acc, temp_acc;

            for (int i = 0; i < q.cols() - 2; i++)
            {
                /* evaluate acc */
                acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
                cost += acc.squaredNorm();
                temp_acc = 2.0 * acc;
                /* acc gradient */
                gradient.col(i + 0) += temp_acc;
                gradient.col(i + 1) += -2.0 * temp_acc;
                gradient.col(i + 2) += temp_acc;
            }
        }
    }

    void bspline_optimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient)
    {
    cost = 0.0;
    /* abbreviation */
    double ts, /*vm2, am2, */ ts_inv2;
    // vm2 = max_vel_ * max_vel_;
    // am2 = max_acc_ * max_acc_;

    ts = bspline_interval_;
    ts_inv2 = 1 / ts / ts;
    /* velocity feasibility */
    for (int i = 0; i < q.cols() - 1; i++)
    {
      Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;
      for (int j = 0; j < 2; j++)
      {
        if (vi(j) > max_vel_)
        {
          cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

          gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
        }
        else if (vi(j) < -max_vel_)
        {
          cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

          gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
          gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
        }
        else
        {
          /* code */
        }
      }
    }
    /* acceleration feasibility */
    for (int i = 0; i < q.cols() - 2; i++)
    {
      Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

      for (int j = 0; j < 2; j++)
      {
        if (ai(j) > max_acc_)
        {
          cost += pow(ai(j) - max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
        }
        else if (ai(j) < -max_acc_)
        {
          cost += pow(ai(j) + max_acc_, 2);

          gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
          gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
        }
        else
        {
          /* code */
        }
      }

    }
    }
    
    void bspline_optimizer::calcEsdfCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient)
    {
        cost = 0.0;
        double  dist;
        Eigen::Vector3d dist_grad;

        for (int i = p_order_; i < q.cols()-p_order_; i++) 
        {
            edt_environment_->evaluateEDTWithGrad(q.col(i), -1.0, dist, dist_grad);

            if (dist_grad.norm() > 1e-4) dist_grad.normalize();
            if (dist < safe_distance_) 
            {
                cost += pow(dist - safe_distance_, 2);
                gradient.col(i) += 2.0 * (dist - safe_distance_) * dist_grad;     
            }
        }   
    }

    common_msgs::Force bspline_optimizer::calcGradForce(const Eigen::Vector3d& q_3d)
    {
        common_msgs::Force force;
        double dist; 
        Eigen::Vector3d dist_grad_3d; 

        edt_environment_->evaluateEDTWithGrad(q_3d, -1.0, dist, dist_grad_3d);

        if (dist >  safe_distance_) {
            force.x = 0.0;
            force.y = 0.0;
            force.z = 0.0;
            force.k_den = 1.0;
            return force;
        }

        double force_magnitude = k_force * std::pow((dist - safe_distance_), 2);        
        Eigen::Vector3d grad_normalized = dist_grad_3d.normalized();

        force.x = force_magnitude * grad_normalized.x();
        force.y = force_magnitude * grad_normalized.y();
        force.z = force_magnitude * grad_normalized.z();

        force.k_den = mapForceToKDen(force_magnitude);

        return force;
    }

    bool bspline_optimizer::checkTrajCollision(const common_msgs::BsplineTraj traj){}

    double bspline_optimizer::mapForceToKDen(double force_magnitude)
    {
        double min_force = 0.0;
        double max_force = 1.0;

        force_magnitude = std::max(min_force, std::min(force_magnitude, max_force));

        return 4.0 - 2.0 * std::pow(force_magnitude, 2);
    }


    void bspline_optimizer::combineCost( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine)
    {
        Eigen::MatrixXd control_points = control_points_.transpose();
        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
              for (size_t i = 0; i < Dim_; i++)
                {
                    control_points(i,j+p_order_) = x[j*Dim_+i];
                } 
            }
        f_combine = 0.0;
        Eigen::MatrixXd grad3D; 
        grad3D.resize(Dim_,cps_num_);
        grad3D.setZero(Dim_,cps_num_);

        double f_smoothness, f_length, f_distance, f_feasibility;
        Eigen::MatrixXd g_smoothness_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        Eigen::MatrixXd g_feasibility_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        Eigen::MatrixXd g_distance_ = Eigen::MatrixXd::Zero(Dim_, cps_num_);
        f_smoothness  = f_feasibility =  f_distance = 0.0;
        // cout<< control_points.transpose()<<endl;
        calcSmoothnessCost(control_points, f_smoothness, g_smoothness_);
    // cout<<"====================calcSmoothnessCost"<<endl;
        calcFeasibilityCost(control_points,f_feasibility,g_feasibility_);
    // cout<<"====================calcFeasibilityCost"<<endl;
        calcEsdfCost(control_points,f_distance,g_distance_);
    // cout<<"====================calcEsdfCost"<<endl;

        f_combine = lambda1_ * f_smoothness + lambda2_*f_feasibility + lambda3_*f_distance;
        grad3D = lambda1_*g_smoothness_ + lambda2_ * g_feasibility_ +lambda3_ * g_distance_ ;
        grad = grad3D.block(0,p_order_,Dim_,cps_num_-2*p_order_);//起点  块大小
    }

    double bspline_optimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                                                                         void* func_data)
    {
        bspline_optimizer* opt = reinterpret_cast<bspline_optimizer*>(func_data);
        Eigen::MatrixXd grad_matrix;
        double cost;
        opt->combineCost(x,grad_matrix,cost);
        opt->iter_num_++;

        for (size_t i = 0; i < grad_matrix.cols(); i++)
            {
                for (size_t j = 0; j <opt->Dim_; j++)
                {
                    // grad.push_back(grad_matrix(j,i)) ;
                    grad[i*opt->Dim_+j] = grad_matrix(j,i);
                }    
            } 
        /* save the min cost result */
        if (cost < opt->min_cost_) {
                opt->min_cost_     = cost;
                opt->best_variable_ = x;
            }
        return cost;
    }

    void bspline_optimizer::optimize()
    {
            /* initialize solver */
            // cout << "/* initialize solver */"<<endl;
            iter_num_        = 0;
            min_cost_        = std::numeric_limits<double>::max();
            
            variable_num = (cps_num_-2*p_order_)*Dim_;
            nlopt::opt opt(nlopt::algorithm(nlopt::LD_LBFGS),variable_num);
            opt.set_min_objective(bspline_optimizer::costFunction,this);
            opt.set_maxeval(opt_maxeval);
            opt.set_maxtime(opt_maxtime);
            opt.set_xtol_rel(1e-5);

            vector<double> lb(variable_num), ub(variable_num);
            vector<double> q(variable_num); 
            for (size_t j = 0; j < cps_num_-2*p_order_; j++)
            {
               for (size_t i = 0; i < Dim_; i++)
                {
                    q[j*Dim_+i] = control_points_(j+p_order_,i);
                }
            }

            const double  bound = 10.0;
            for (size_t i = 0; i <variable_num; i++)
            {
                    lb[i]  = q[i]-bound;
                    ub[i] = q[i]+bound;      
            }
            opt.set_lower_bounds(lb);
            opt.set_upper_bounds(ub);

        try
        {
            double final_cost;
            nlopt::result result = opt.optimize(q, final_cost);
            publishTrajectory(q);     
        }
        catch(std::exception &e)
        {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
        for (size_t j = 0; j < cps_num_-2*p_order_; j++)
        {
            for (size_t i = 0; i < Dim_; i++)
            {
                control_points_(j+p_order_,i) = best_variable_[j*Dim_+i];
            } 
        }
            // cout<< "optimize successfully~"<<endl;
            // cout << "iner:\n"<<control_points_<<endl;
            cout<<"iter num :"<<iter_num_<<endl;
    }

    void bspline_optimizer::publishTrajectory(const std::vector<double>& control_points){}

}