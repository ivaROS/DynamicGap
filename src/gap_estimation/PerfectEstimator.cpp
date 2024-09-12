#include <dynamic_gap/gap_estimation/PerfectEstimator.h>

namespace dynamic_gap 
{
    PerfectEstimator::PerfectEstimator() 
    {
        xFrozen_ << 0.0, 0.0, 0.0, 0.0;
    }

    void PerfectEstimator::initialize(const std::string & side, const int & modelID,
                                      const float & gapPtX, const float & gapPtY,
                                      const ros::Time & tUpdate, const geometry_msgs::TwistStamped & lastRbtVel,
                                      const geometry_msgs::TwistStamped & lastRbtAcc) 
    {
        this->side_ = side;
        this->modelID_ = modelID;
        // ROS_INFO_STREAM_NAMED("GapEstimation", "    initialize model: " << modelID_);

        this->xTilde_ << gapPtX, gapPtY;

        this->lastRbtVel_ = lastRbtVel;
        this->lastRbtAcc_ = lastRbtAcc;
        this->tLastUpdate_ = tUpdate;
    }

    // For transferring an existing model state to a new model
    void PerfectEstimator::transfer(const Estimator & incomingModel)
    {
        this->side_ = incomingModel.side_;
        this->modelID_ = incomingModel.modelID_;

        // ROS_INFO_STREAM_NAMED("GapEstimation", "        transfer model: " << modelID_);

        // COVARIANCE MATRIX
        // covariance/uncertainty of state variables (r_x, r_y, v_x, v_y)
        this->P_kmin1_plus_ = incomingModel.P_kmin1_plus_;
        this->P_k_minus_ = incomingModel.P_k_minus_;
        this->P_k_plus_ = incomingModel.P_k_plus_;

        this->lastRbtVel_ = incomingModel.lastRbtVel_;
        this->lastRbtAcc_ = incomingModel.lastRbtAcc_;
        this->tLastUpdate_ = incomingModel.tLastUpdate_;

        // this->xTilde_ = incomingModel.xTilde_;
        this->x_hat_kmin1_plus_ = incomingModel.x_hat_kmin1_plus_;
        this->x_hat_k_minus_ = incomingModel.x_hat_k_minus_;
        this->x_hat_k_plus_ = incomingModel.x_hat_k_plus_;

        this->G_k_ = incomingModel.G_k_;

        // this->A_ = model.A_;
        // this->STM_ = model.STM_;

        // this->eyes = model.eyes;

        return;
    }

    void PerfectEstimator::update(const Eigen::Vector2f & measurement, 
                                    const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                                    const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs, 
                                    const std::map<std::string, geometry_msgs::Pose> & agentPoses,
                                    const std::map<std::string, geometry_msgs::Vector3Stamped> & agentVels,
                                    const ros::Time & tUpdate)
        {
        
        agentPoses_ = agentPoses;
        agentVels_ = agentVels;

        // acceleration and velocity come in wrt robot frame
        interpIntermediateRbtVels_ = intermediateRbtVels;
        interpIntermediateRbtAccs_ = intermediateRbtAccs;
        // lastRbtVel_ = _current_rbt_vel;
        // lastRbtAcc_ = _current_rbt_acc;

        // dt = scan_dt;
        // life_time += dt;

        // inter_dt = (dt / interpIntermediateRbtVels_.size());

        if (interpIntermediateRbtVels_.size() == 0 || interpIntermediateRbtAccs_.size() == 0)
        {
            ROS_WARN_STREAM_COND_NAMED(interpIntermediateRbtVels_.size() == 0, "    GapEstimation", "interpIntermediateRbtVels_ is empty, no update");
            ROS_WARN_STREAM_COND_NAMED(interpIntermediateRbtAccs_.size() == 0, "    GapEstimation", "interpIntermediateRbtAccs_ is empty, no update");
            return;
        }

        if (interpIntermediateRbtVels_.size() != interpIntermediateRbtAccs_.size())
        {
            // ROS_INFO_STREAM_NAMED("GapEstimation", "interpIntermediateRbtVels_ is of size " << interpIntermediateRbtVels_.size() << " while interpIntermediateRbtAccs_ is of size " << interpIntermediateRbtAccs_.size());
            return;
        }

        // ROS_INFO_STREAM_NAMED("GapEstimation", "update for model " << getID()); // << ", life_time: " << life_time << ", dt: " << dt << ", inter_dt: " << inter_dt);
        // ROS_INFO_STREAM_NAMED("GapEstimation", "x_hat_kmin1_plus_: " << x_hat_kmin1_plus_[0] << ", " << x_hat_kmin1_plus_[1] << ", " << x_hat_kmin1_plus_[2] << ", " << x_hat_kmin1_plus_[3]);
        // ROS_INFO_STREAM_NAMED("GapEstimation", "current_rbt_vel, x_lin: " << lastRbtVel_.twist.linear.x << ", y_lin: " << lastRbtVel_.twist.linear.y << ", z_ang: " << lastRbtVel_.twist.angular.z);

        processEgoRobotVelsAndAccs(tUpdate);

        xTilde_ = measurement;

        // ROS_INFO_STREAM_NAMED("GapEstimation", "linear ego vel: " << lastRbtVel_.twist.linear.x << ", " << lastRbtVel_.twist.linear.y << ", angular ego vel: " << lastRbtVel_.twist.angular.z);
        // ROS_INFO_STREAM_NAMED("GapEstimation", "linear ego acceleration: " << lastRbtAcc_.twist.linear.x << ", " << lastRbtAcc_.twist.linear.y << ", angular ego acc: " << lastRbtAcc_.twist.angular.z);

        x_hat_k_plus_ = updateStateFromEnv();

        x_hat_kmin1_plus_ = x_hat_k_plus_;
        P_kmin1_plus_ = P_k_plus_;
        tLastUpdate_ = tUpdate;
        
        if (interpIntermediateRbtVels_.size() > 0)
            lastRbtVel_ = interpIntermediateRbtVels_.back();
        
        if (interpIntermediateRbtAccs_.size() > 0)
            lastRbtAcc_ = interpIntermediateRbtAccs_.back();

        // ROS_INFO_STREAM_NAMED("GapEstimation", "x_hat_k_plus_: " << x_hat_k_plus_[0] << ", " << x_hat_k_plus_[1] << ", " << x_hat_k_plus_[2] << ", " << x_hat_k_plus_[3]);       
        // ROS_INFO_STREAM_NAMED("GapEstimation", "-----------");

        return;
    }    

    Eigen::Vector4f PerfectEstimator::updateStateFromEnv() 
    {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4f return_x = x_hat_k_plus_;

        
        // ROS_INFO_STREAM_NAMED("GapEstimation", "updating ground truth cartesian state");
        // // ROS_INFO_STREAM_NAMED("GapEstimation", "xTilde_: " << xTilde_[0] << ", " << xTilde_[1]);
        

        return_x[0] = xTilde_[0];
        return_x[1] = xTilde_[1];
        
        float robot_i_odom_dist = 0.0;
        float min_dist = std::numeric_limits<float>::infinity();
        std::string min_key = "N/A";
        std::map<std::string, geometry_msgs::Pose>::iterator itr;
        for (itr = agentPoses_.begin(); itr != agentPoses_.end(); ++itr) 
        {
            geometry_msgs::Pose agentPose = itr->second;
            robot_i_odom_dist = sqrt(pow(agentPose.position.x - x_hat_kmin1_plus_[0], 2) + 
                                     pow(agentPose.position.y - x_hat_kmin1_plus_[1], 2));
            
            if (robot_i_odom_dist < min_dist) 
            {
                min_dist = robot_i_odom_dist;
                min_key = itr->first;
            }
        }
        
        ROS_INFO_STREAM_COND_NAMED(agentPoses_.size() > 0, "GapEstimation", "closest odom: " << agentPoses_[min_key].position.x << ", " 
                                                                                             << agentPoses_[min_key].position.y);
        
        if (min_dist < min_dist_thresh) 
        {    
            // ROS_INFO_STREAM_NAMED("GapEstimation", "attaching to odom");
            
            
            // xTilde_[0] = agentPoses_[min_idx].position.x;
            // xTilde_[1] = agentPoses_[min_idx].position.y;
            // return_x[0] = xTilde_[0];
            // return_x[1] = xTilde_[1];
            return_x[2] = agentVels_[min_key].vector.x - lastRbtVel_.twist.linear.x;
            return_x[3] = agentVels_[min_key].vector.y - lastRbtVel_.twist.linear.y;
        } else 
        {    
            // ROS_INFO_STREAM_NAMED("GapEstimation", "attaching to nothing");
            
            return_x[2] = 0.0 - lastRbtVel_.twist.linear.x;
            return_x[3] = 0.0 - lastRbtVel_.twist.linear.y;         
        }

        return return_x;
    }

    Eigen::Vector4f PerfectEstimator::getState()
    { 
        return x_hat_k_plus_; 
    }
}