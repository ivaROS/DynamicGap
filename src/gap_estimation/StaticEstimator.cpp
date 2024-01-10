// #include <ros/ros.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Quaternion.h>
// #include <turtlesim/Spawn.h>
#include "std_msgs/String.h"
// #include <nav_msgs/Odometry.h>
// #include <sensor_msgs/Imu.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_gap/gap_estimation/StaticEstimator.h>
// #include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <limits>
// #include <sstream>
// #include <unsupported/Eigen/MatrixFunctions>
// #include "/home/masselmeier/Desktop/Research/vcpkg/installed/x64-linux/include/matplotlibcpp.h"
// namespace plt = matplotlibcpp;

namespace dynamic_gap 
{
    StaticEstimator::StaticEstimator(const std::string & side, const int & modelID, 
                                     const float & gapPtX, const float & gapPtY,
                                     const ros::Time & t_update, const geometry_msgs::TwistStamped & lastRbtVel,
                                     const geometry_msgs::TwistStamped & lastRbtAcc) 
    {
        side_ = side;
        modelID_ = modelID;
        initialize(gapPtX, gapPtY, t_update, lastRbtVel, lastRbtAcc);
    }

    void StaticEstimator::initialize(const float & gapPtX, const float & gapPtY,
                                     const ros::Time & t_update, const geometry_msgs::TwistStamped & lastRbtVel,
                                     const geometry_msgs::TwistStamped & lastRbtAcc) 
    {
        // OBSERVATION MATRIX
        H_ << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0;
        H_transpose_ = H_.transpose();
        
        R_scalar = 0.01;
        Q_scalar = 0.5;

        Q_k_ << 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, Q_scalar, 0.0,
               0.0, 0.0, 0.0, Q_scalar;

        // COVARIANCE MATRIX
        // covariance/uncertainty of state variables (r_x, r_y, v_x, v_y)
        // larger P_0 helps with GT values that are non-zero
        // larger P_0 gives more weight to measurements (behaves like Q)
        P_kmin1_plus_ << 0.01, 0.0, 0.0, 0.0,
                        0.0, 0.01, 0.0, 0.0,
                        0.0, 0.0, 0.1, 0.0,
                        0.0, 0.0, 0.0, 0.1;
        P_k_minus_ = P_kmin1_plus_;
        P_k_plus_ = P_kmin1_plus_;

        this->lastRbtVel_ = lastRbtVel;
        this->lastRbtAcc_ = lastRbtAcc;
        this->t_last_update = t_update;


        float gapPtVxRel = -lastRbtVel_.twist.linear.x;
        float gapPtVyRel = -lastRbtVel_.twist.linear.y;
        std::vector<float> measurement{gapPtX, 
                                       gapPtY, 
                                       gapPtVxRel, 
                                       gapPtVyRel};
        
        x_tilde_ << measurement[0], measurement[1];
        x_hat_kmin1_plus_ << measurement[0], measurement[1], measurement[2], measurement[3];
        x_hat_k_minus_ = x_hat_kmin1_plus_; 
        x_hat_k_plus_ = x_hat_kmin1_plus_;
        x_ground_truth << measurement[0], measurement[1], 0.0, 0.0;
        x_ground_truth_gap_only << measurement[0], measurement[1], gapPtVxRel, gapPtVyRel;

        G_k_ << 1.0, 1.0,
             1.0, 1.0,
             1.0, 1.0,
             1.0, 1.0;

        // dt = 0.0;

        A_ << 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0;
        STM_ = A_;

        frozen_x << 0.0, 0.0, 0.0, 0.0;

        // life_time = 0.0;
        // life_time_threshold = 7.5;
        eyes = Eigen::MatrixXf::Identity(4,4);

        plot_dir = "/home/masselmeier/catkin_ws/src/DynamicGap/estimator_plots/";   
        perfect = false;
        plotted = false;
        plot = true;
    }

    void StaticEstimator::processEgoRobotVelsAndAccs(const ros::Time & t_update)
    {
        /*
        // Printing original dt values from intermediate odom measurements
        ROS_INFO_STREAM_NAMED("GapEstimation", "   t_0 - t_last_update difference:" << (intermediateRbtVels_[0].header.stamp - t_last_update).toSec() << " sec");

        for (int i = 0; i < (intermediateRbtVels_.size() - 1); i++)
        {
            ROS_INFO_STREAM_NAMED("GapEstimation", "   t_" << (i+1) << " - t_" << i << " difference: " << (intermediateRbtVels_[i + 1].header.stamp - intermediateRbtVels_[i].header.stamp).toSec() << " sec");
        }

        ROS_INFO_STREAM_NAMED("GapEstimation", "   t_update" << " - t_" << (intermediateRbtVels_.size() - 1) << " difference:" << (t_update - intermediateRbtVels_[intermediateRbtVels_.size() - 1].header.stamp).toSec() << " sec");
        */

        // Tweaking ego robot velocities/acceleration to make sure that updates:
        //      1. Are never negative (backwards in time)
        //      2. Always start from time of last update
        //      3. Always at end of time of incoming laser scan measurement

        // Erasing odometry measurements that are from *before* the last update 
        while (!intermediateRbtVels_.empty() && t_last_update > intermediateRbtVels_[0].header.stamp)
        {
            intermediateRbtVels_.erase(intermediateRbtVels_.begin());
            intermediateRbtAccs_.erase(intermediateRbtAccs_.begin());
        }

        // Inserting placeholder odometry to represent the time of the last update
        intermediateRbtVels_.insert(intermediateRbtVels_.begin(), lastRbtVel_);
        intermediateRbtVels_[0].header.stamp = t_last_update;

        intermediateRbtAccs_.insert(intermediateRbtAccs_.begin(), lastRbtAcc_);
        intermediateRbtAccs_[0].header.stamp = t_last_update;

        // Erasing odometry measurements that occur *after* the incoming laser scan was received
        while (!intermediateRbtVels_.empty() && t_update < intermediateRbtVels_[intermediateRbtVels_.size() - 1].header.stamp)
        {
            intermediateRbtVels_.erase(intermediateRbtVels_.end() - 1);
            intermediateRbtAccs_.erase(intermediateRbtAccs_.end() - 1);
        }

        // Inserting placeholder odometry to represent the time that the incoming laser scan was received
        intermediateRbtVels_.push_back(intermediateRbtVels_.back());
        intermediateRbtVels_.back().header.stamp = t_update;

        for (int i = 0; i < (intermediateRbtVels_.size() - 1); i++)
        {
            float dt = (intermediateRbtVels_[i + 1].header.stamp - intermediateRbtVels_[i].header.stamp).toSec();
            
            ROS_INFO_STREAM_NAMED("GapEstimation", "   t_" << (i+1) << " - t_" << i << " difference: " << dt << " sec");
            
            ROS_INFO_STREAM_COND_NAMED(dt < 0, "GapEstimation", "   ERROR IN TIMESTEP CALCULATION, SHOULD NOT BE NEGATIVE");
        }
    }

    void StaticEstimator::isolateGapDynamics() 
    {
        Eigen::Vector4f cartesian_state = getState();
        
        // fixing position (otherwise can get bugs)
        cartesian_state[0] = x_tilde_[0];
        cartesian_state[1] = x_tilde_[1];

        // update cartesian
        cartesian_state[2] += lastRbtVel_.twist.linear.x;
        cartesian_state[3] += lastRbtVel_.twist.linear.y;
        frozen_x = cartesian_state;

        //std::cout << "modified cartesian state: " << frozen_x[0] << ", " << frozen_x[1] << ", " << frozen_x[2] << ", " << frozen_x[3] << std::endl;
    }

    void StaticEstimator::setRewindState() 
    {
        rewind_x = frozen_x;
    }

    void StaticEstimator::rewindPropagate(const float & rew_dt) 
    {
        Eigen::Vector4f new_rewind_x;     
        new_rewind_x << 0.0, 0.0, 0.0, 0.0;

        Eigen::Vector2f frozen_linear_acc_ego(0.0, 0.0);

        Eigen::Vector2f frozen_linear_vel_ego(0.0, 0.0); 
        float frozen_ang_vel_ego = 0.0;

        float vdot_x_body = frozen_linear_acc_ego[0];
        float vdot_y_body = frozen_linear_acc_ego[1];

        // discrete euler update of state (ignoring rbt acceleration, set as 0)
        new_rewind_x[0] = rewind_x[0] + (rewind_x[2] + rewind_x[1]*frozen_ang_vel_ego)*rew_dt;
        new_rewind_x[1] = rewind_x[1] + (rewind_x[3] - rewind_x[0]*frozen_ang_vel_ego)*rew_dt;
        new_rewind_x[2] = rewind_x[2] + (rewind_x[3]*frozen_ang_vel_ego - vdot_x_body)*rew_dt;
        new_rewind_x[3] = rewind_x[3] + (-rewind_x[2]*frozen_ang_vel_ego - vdot_y_body)*rew_dt;
        rewind_x = new_rewind_x; 
    }

    void StaticEstimator::gapStatePropagate(const float & froz_dt) 
    {
        Eigen::Vector4f new_frozen_x;     
        new_frozen_x << 0.0, 0.0, 0.0, 0.0;

        Eigen::Vector2f frozen_linear_acc_ego(0.0, 0.0);

        Eigen::Vector2f frozen_linear_vel_ego(0.0, 0.0); 
        float frozen_ang_vel_ego = 0.0;

        float vdot_x_body = frozen_linear_acc_ego[0];
        float vdot_y_body = frozen_linear_acc_ego[1];

        // discrete euler update of state (ignoring rbt acceleration, set as 0)
        new_frozen_x[0] = frozen_x[0] + (frozen_x[2] + frozen_x[1]*frozen_ang_vel_ego)*froz_dt;
        new_frozen_x[1] = frozen_x[1] + (frozen_x[3] - frozen_x[0]*frozen_ang_vel_ego)*froz_dt;
        new_frozen_x[2] = frozen_x[2] + (frozen_x[3]*frozen_ang_vel_ego - vdot_x_body)*froz_dt;
        new_frozen_x[3] = frozen_x[3] + (-frozen_x[2]*frozen_ang_vel_ego - vdot_y_body)*froz_dt;
        frozen_x = new_frozen_x; 
    }
    

    Eigen::Vector4f StaticEstimator::integrate() 
    {
        // ROS_INFO_STREAM_NAMED("GapEstimation", "INTEGRATING");
        Eigen::Vector4f x_intermediate = x_hat_kmin1_plus_;
        Eigen::Vector4f new_x = x_hat_kmin1_plus_;

        for (int i = 0; i < (intermediateRbtVels_.size() - 1); i++) 
        {
            ROS_INFO_STREAM_NAMED("GapEstimation", "intermediate step " << i);
            
            float dt = (intermediateRbtVels_[i + 1].header.stamp - intermediateRbtVels_[i].header.stamp).toSec();
            float ang_vel_ego = intermediateRbtVels_[i].twist.angular.z;
            
            ROS_INFO_STREAM_NAMED("GapEstimation", "ang_vel_ego: " << ang_vel_ego);

            float p_dot_x = (x_intermediate[2] + ang_vel_ego*x_intermediate[1]);
            float p_dot_y = (x_intermediate[3] - ang_vel_ego*x_intermediate[0]);
            ROS_INFO_STREAM_NAMED("GapEstimation", "p_dot_x: " << p_dot_x << ", p_dot_y: " << p_dot_y);

            float vdot_x_body = intermediateRbtAccs_[i].twist.linear.x;
            float vdot_y_body = intermediateRbtAccs_[i].twist.linear.y;
            ROS_INFO_STREAM_NAMED("GapEstimation", "vdot_x_body: " << vdot_x_body << ", vdot_y_body: " << vdot_y_body);

            float v_dot_x = (x_intermediate[3]*ang_vel_ego - vdot_x_body);
            float v_dot_y = (-x_intermediate[2]*ang_vel_ego - vdot_y_body);
            ROS_INFO_STREAM_NAMED("GapEstimation", "v_dot_x: " << v_dot_x << ", v_dot_y: " << v_dot_y);

            new_x << x_intermediate[0] + p_dot_x*dt, // r_x
                     x_intermediate[1] + p_dot_y*dt, // r_y
                     x_intermediate[2] + v_dot_x*dt, // v_x
                     x_intermediate[3] + v_dot_y*dt; // v_y
            
            x_intermediate = new_x;
            ROS_INFO_STREAM_NAMED("GapEstimation", "x_intermediate: " << x_intermediate[0] << ", " << x_intermediate[1] << ", " << x_intermediate[2] << ", " << x_intermediate[3]);
        
        }

        return x_intermediate;
    }

    void StaticEstimator::linearize(const int & idx) 
    {
        float dt = (intermediateRbtVels_[idx + 1].header.stamp - intermediateRbtVels_[idx].header.stamp).toSec();    
        float ang_vel_ego = intermediateRbtVels_[idx].twist.angular.z;
        
        A_ << 0.0, ang_vel_ego, 1.0, 0.0,
             -ang_vel_ego, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0, ang_vel_ego,
             0.0, 0.0, -ang_vel_ego, 0.0;

        STM_ = (A_*dt).exp();
    }

    void StaticEstimator::discretizeQ(const int & idx) 
    {

        // ROS_INFO_STREAM_NAMED("GapEstimation", "VxVx: " << cfg_->gap_est.Q_VxVx << ", VyVy: " << cfg_->gap_est.Q_VyVy);
        float dt = (intermediateRbtVels_[idx + 1].header.stamp - intermediateRbtVels_[idx].header.stamp).toSec();

        Q_1 = Q_k_;
        Q_2 = A_ * Q_1 + Q_1 * A_.transpose();
        Q_3 = A_ * Q_2 + Q_2 * A_.transpose();

        dQ_ = (Q_1 * dt) + (Q_2 * dt * dt / 2.0) + (Q_3 * dt * dt * dt / 6.0);
    }

    void StaticEstimator::update(const Eigen::Vector2f & measurement, 
                                 const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                                 const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs, 
                                 const std::vector<geometry_msgs::Pose> & agentPoses,
                                 const std::vector<geometry_msgs::Vector3Stamped> & agentVels,
                                 const ros::Time & t_update) 
    {
        
        agentPoses_ = agentPoses;
        agentVels_ = agentVels;

        // acceleration and velocity come in wrt robot frame
        intermediateRbtVels_ = intermediateRbtVels;
        intermediateRbtAccs_ = intermediateRbtAccs;
        // lastRbtVel_ = _current_rbt_vel;
        // lastRbtAcc_ = _current_rbt_acc;

        // dt = scan_dt;
        // life_time += dt;

        // inter_dt = (dt / intermediateRbtVels_.size());

        if (intermediateRbtVels_.size() == 0 || intermediateRbtAccs_.size() == 0)
        {
            ROS_WARN_STREAM_COND_NAMED(intermediateRbtVels_.size() == 0, "    GapEstimation", "intermediateRbtVels_ is empty, no update");
            ROS_WARN_STREAM_COND_NAMED(intermediateRbtAccs_.size() == 0, "    GapEstimation", "intermediateRbtAccs_ is empty, no update");
            return;
        }

        if (intermediateRbtVels_.size() != intermediateRbtAccs_.size())
        {
            ROS_INFO_STREAM_NAMED("GapEstimation", "intermediateRbtVels_ is of size " << intermediateRbtVels_.size() << " while intermediateRbtAccs_ is of size " << intermediateRbtAccs_.size());
            return;
        }

        ROS_INFO_STREAM_NAMED("GapEstimation", "update for model " << getID()); // << ", life_time: " << life_time << ", dt: " << dt << ", inter_dt: " << inter_dt);
        ROS_INFO_STREAM_NAMED("GapEstimation", "x_hat_kmin1_plus_: " << x_hat_kmin1_plus_[0] << ", " << x_hat_kmin1_plus_[1] << ", " << x_hat_kmin1_plus_[2] << ", " << x_hat_kmin1_plus_[3]);
        ROS_INFO_STREAM_NAMED("GapEstimation", "current_rbt_vel, x_lin: " << lastRbtVel_.twist.linear.x << ", y_lin: " << lastRbtVel_.twist.linear.y << ", z_ang: " << lastRbtVel_.twist.angular.z);

        processEgoRobotVelsAndAccs(t_update);

        // get_intermediateRbtVels__accs();

        x_tilde_ = measurement;
                // << range_bearing_measurement[0]*std::cos(range_bearing_measurement[1]),
                //    range_bearing_measurement[0]*std::sin(range_bearing_measurement[1]);
        
        ROS_INFO_STREAM_NAMED("GapEstimation", "linear ego vel: " << lastRbtVel_.twist.linear.x << ", " << lastRbtVel_.twist.linear.y << ", angular ego vel: " << lastRbtVel_.twist.angular.z);
        ROS_INFO_STREAM_NAMED("GapEstimation", "linear ego acceleration: " << lastRbtAcc_.twist.linear.x << ", " << lastRbtAcc_.twist.linear.y << ", angular ego acc: " << lastRbtAcc_.twist.angular.z);

        x_ground_truth = update_ground_truth_cartesian_state();

        x_hat_k_plus_ = x_ground_truth;


        x_hat_kmin1_plus_ = x_hat_k_plus_;
        P_kmin1_plus_ = P_k_plus_;
        t_last_update = t_update;

        // ROS_INFO_STREAM_NAMED("GapEstimation", "3");


        // ROS_INFO_STREAM_NAMED("GapEstimation", "intermediateRbtVels_ size: " << intermediateRbtVels_.size());
        // ROS_INFO_STREAM_NAMED("GapEstimation", "intermediateRbtAccs_ size: " << intermediateRbtAccs_.size());

        
        if (intermediateRbtVels_.size() > 0)
            lastRbtVel_ = intermediateRbtVels_.back();
        
        if (intermediateRbtAccs_.size() > 0)
            lastRbtAcc_ = intermediateRbtAccs_.back();

        // ROS_INFO_STREAM_NAMED("GapEstimation", "4");

        /*
        ROS_INFO_STREAM_NAMED("GapEstimation", "G_k_: " << G_k_(0, 0) << ", " << G_k_(0, 1));
        ROS_INFO_STREAM_NAMED("GapEstimation", "     " << G_k_(1, 0) << ", " << G_k_(1, 1));
        ROS_INFO_STREAM_NAMED("GapEstimation", "     " << G_k_(2, 0) << ", " << G_k_(2, 1));
        ROS_INFO_STREAM_NAMED("GapEstimation", "     " << G_k_(3, 0) << ", " << G_k_(3, 1));
        */

        ROS_INFO_STREAM_NAMED("GapEstimation", "x_hat_k_plus_: " << x_hat_k_plus_[0] << ", " << x_hat_k_plus_[1] << ", " << x_hat_k_plus_[2] << ", " << x_hat_k_plus_[3]);       
        ROS_INFO_STREAM_NAMED("GapEstimation", "x_GT: " << x_ground_truth[0] << ", " << x_ground_truth[1] << ", " << x_ground_truth[2] << ", " << x_ground_truth[3]);
        ROS_INFO_STREAM_NAMED("GapEstimation", "-----------");

        return;
    }    

    Eigen::Vector4f StaticEstimator::update_ground_truth_cartesian_state() 
    {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4f return_x = x_ground_truth;
        
        ROS_INFO_STREAM_NAMED("GapEstimation", "updating ground truth cartesian state");
        // ROS_INFO_STREAM_NAMED("GapEstimation", "x_tilde_: " << x_tilde_[0] << ", " << x_tilde_[1]);

        return_x[0] = x_tilde_[0];
        return_x[1] = x_tilde_[1];
        x_ground_truth_gap_only[0] = x_tilde_[0];
        x_ground_truth_gap_only[1] = x_tilde_[1];

        ROS_INFO_STREAM_NAMED("GapEstimation", "attaching to nothing");
        
        return_x[2] = 0.0 - lastRbtVel_.twist.linear.x;
        return_x[3] = 0.0 - lastRbtVel_.twist.linear.y;

        x_ground_truth_gap_only[2] = 0.0;
        x_ground_truth_gap_only[3] = 0.0;            

        return return_x;
    }

    Eigen::Vector4f StaticEstimator::getState() 
    {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4f return_x = (perfect) ? x_ground_truth : x_hat_k_plus_;
        
        /*
        if (life_time < 0.25)
        {
            // not trusting early measurements, just treating gap as static
            return_x[2] = 0.0 - lastRbtVel_.twist.linear.x;
            return_x[3] = 0.0 - lastRbtVel_.twist.linear.y;
        }
        */

        return return_x;
    }

    Eigen::Vector4f StaticEstimator::getTrueState() 
    {
        // x state:
        // [r_x, r_y, v_x, v_y]

        return x_ground_truth;
    }

    Eigen::Vector4f StaticEstimator::getGapState() 
    {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4f return_x = frozen_x;
        return return_x;
    }

    Eigen::Vector4f StaticEstimator::getRewindGapState() 
    {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4f return_x = rewind_x;
        return return_x;
    }

    geometry_msgs::TwistStamped StaticEstimator::getRobotVel() 
    {
        return lastRbtVel_;
    }

    int StaticEstimator::getID() 
    {
        return modelID_;
    }

    Eigen::Vector2f StaticEstimator::get_x_tilde() 
    {
        return x_tilde_;
    }
}