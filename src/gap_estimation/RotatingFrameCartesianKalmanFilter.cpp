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
#include <dynamic_gap/gap_estimation/RotatingFrameCartesianKalmanFilter.h>
// #include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <limits>
// #include <sstream>
// #include <unsupported/Eigen/MatrixFunctions>
// #include "/home/masselmeier/Desktop/Research/vcpkg/installed/x64-linux/include/matplotlibcpp.h"
// namespace plt = matplotlibcpp;

namespace dynamic_gap 
{
    RotatingFrameCartesianKalmanFilter::RotatingFrameCartesianKalmanFilter() 
    {
        H_ << 1.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0;
        H_transpose_ = H_.transpose();
        
        R_scalar = 0.01;
        Q_scalar = 0.5;

        Q_k_ << 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, Q_scalar, 0.0,
               0.0, 0.0, 0.0, Q_scalar;
        R_k_ << R_scalar, 0.0,
                0.0, R_scalar;

        G_k_ << 1.0, 1.0,
                1.0, 1.0,
                1.0, 1.0,
                1.0, 1.0;

        A_ << 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0;
        STM_ = A_;

        frozen_x << 0.0, 0.0, 0.0, 0.0;

        eyes = Eigen::MatrixXf::Identity(4,4);

        perfect = false;          
    }

    /*
    RotatingFrameCartesianKalmanFilter::RotatingFrameCartesianKalmanFilter(float gapPtX, float gapPtY, 
                                                                            const ros::Time & t_update, 
                                                                            const geometry_msgs::TwistStamped & lastRbtVel,
                                                                            const geometry_msgs::TwistStamped & lastRbtAcc) 
    {
        initialize(side, modelID, gapPtX, gapPtY, t_update, lastRbtVel, lastRbtAcc);
    }
    */

    RotatingFrameCartesianKalmanFilter::~RotatingFrameCartesianKalmanFilter()
    {
        return;
    }

    // For initializing a new model
    void RotatingFrameCartesianKalmanFilter::initialize(const std::string & side, const int & modelID,
                                                        const float & gapPtX, const float & gapPtY,
                                                        const ros::Time & t_update, const geometry_msgs::TwistStamped & lastRbtVel,
                                                        const geometry_msgs::TwistStamped & lastRbtAcc) 
    {
        this->side_ = side;
        this->modelID_ = modelID;
        ROS_INFO_STREAM_NAMED("GapEstimation", "    initialize model: " << modelID_);

        // COVARIANCE MATRIX
        // covariance/uncertainty of state variables (r_x, r_y, v_x, v_y)
        // larger P_0 helps with GT values that are non-zero
        // larger P_0 gives more weight to measurements (behaves like Q)
        this->P_kmin1_plus_ << 0.01, 0.0, 0.0, 0.0,
                         0.0, 0.01, 0.0, 0.0,
                         0.0, 0.0, 0.10, 0.0,
                         0.0, 0.0, 0.0, 0.10;
        this->P_k_minus_ = P_kmin1_plus_;
        this->P_k_plus_ = P_kmin1_plus_;

        this->lastRbtVel_ = lastRbtVel;
        this->lastRbtAcc_ = lastRbtAcc;
        this->t_last_update = t_update;

        float gapPtVxRel = -lastRbtVel_.twist.linear.x;
        float gapPtVyRel = -lastRbtVel_.twist.linear.y;

        // ROS_INFO_STREAM_NAMED("GapEstimation", "        gapPtVxRel: " << gapPtVxRel << ", gapPtVyRel: " << gapPtVyRel);

        // ROS_INFO_STREAM_NAMED("GapEstimation", "initializing model with x: " << measurement[0] << ", y: " << measurement[1]);

        this->x_tilde_ << gapPtX, gapPtY;
        
        this->x_hat_kmin1_plus_ << gapPtX, gapPtY, gapPtVxRel, gapPtVyRel;
        // ROS_INFO_STREAM_NAMED("GapEstimation", "        x_hat_kmin1_plus_: " << x_hat_kmin1_plus_[0] << ", : " << x_hat_kmin1_plus_[1] << ", " << x_hat_kmin1_plus_[2] << ", " << x_hat_kmin1_plus_[3]);

        this->x_hat_k_minus_ = x_hat_kmin1_plus_; 
        this->x_hat_k_plus_ = x_hat_kmin1_plus_;
        
        this->x_ground_truth << gapPtX, gapPtY, 0.0, 0.0;
        this->x_ground_truth_gap_only << gapPtX, gapPtY, gapPtVxRel, gapPtVyRel;
    }

    // For transferring an existing model state to a new model
    void RotatingFrameCartesianKalmanFilter::transfer(const Estimator & model)
    {
        this->side_ = model.side_;
        this->modelID_ = model.modelID_;

        ROS_INFO_STREAM_NAMED("GapEstimation", "        transfer model: " << modelID_);

        // OBSERVATION MATRIX
        this->H_ = model.H_;
        this->H_transpose_ = model.H_transpose_;

        this->Q_scalar = model.Q_scalar;
        this->R_scalar = model.R_scalar;
        this->Q_k_ = model.Q_k_;
        this->R_k_ = model.R_k_;

        // COVARIANCE MATRIX
        // covariance/uncertainty of state variables (r_x, r_y, v_x, v_y)
        this->P_kmin1_plus_ = model.P_kmin1_plus_;
        this->P_k_minus_ = model.P_k_minus_;
        this->P_k_plus_ = model.P_k_plus_;

        this->lastRbtVel_ = model.lastRbtVel_;
        this->lastRbtAcc_ = model.lastRbtAcc_;
        this->t_last_update = model.t_last_update;

        this->x_tilde_ = model.x_tilde_;
        this->x_hat_kmin1_plus_ = model.x_hat_kmin1_plus_;
        this->x_hat_k_minus_ = model.x_hat_k_minus_;
        this->x_hat_k_plus_ = model.x_hat_k_plus_;

        float gapPtVxRel = -lastRbtVel_.twist.linear.x;
        float gapPtVyRel = -lastRbtVel_.twist.linear.y;
        this->x_ground_truth << x_hat_kmin1_plus_[0], x_hat_kmin1_plus_[1], 0.0, 0.0;
        this->x_ground_truth_gap_only << x_hat_kmin1_plus_[0], x_hat_kmin1_plus_[1], gapPtVxRel, gapPtVyRel;

        this->G_k_ = model.G_k_;

        this->A_ = model.A_;
        this->STM_ = model.STM_;

        this->eyes = model.eyes;

        return;
    }

    void RotatingFrameCartesianKalmanFilter::processEgoRobotVelsAndAccs(const ros::Time & t_update)
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
            
            ROS_WARN_STREAM_COND_NAMED(dt < 0, "GapEstimation", "ERROR IN TIMESTEP CALCULATION, SHOULD NOT BE NEGATIVE");

        }
    }

    void RotatingFrameCartesianKalmanFilter::isolateGapDynamics() 
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

    void RotatingFrameCartesianKalmanFilter::setRewindState() 
    {
        rewind_x = frozen_x;
    }

    void RotatingFrameCartesianKalmanFilter::rewindPropagate(float rew_dt) 
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

    void RotatingFrameCartesianKalmanFilter::gapStatePropagate(float froz_dt) 
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
    

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::integrate() 
    {
        ROS_INFO_STREAM_NAMED("GapEstimation", "    [integrate()]");
        Eigen::Vector4f x_intermediate = x_hat_kmin1_plus_;
        Eigen::Vector4f new_x = x_hat_kmin1_plus_;

        for (int i = 0; i < (intermediateRbtVels_.size() - 1); i++) 
        {
            ROS_INFO_STREAM_NAMED("GapEstimation", "        intermediate step " << i);
            
            float dt = (intermediateRbtVels_[i + 1].header.stamp - intermediateRbtVels_[i].header.stamp).toSec();
            float ang_vel_ego = intermediateRbtVels_[i].twist.angular.z;
            
            ROS_INFO_STREAM_NAMED("GapEstimation", "        ang_vel_ego: " << ang_vel_ego);

            float p_dot_x = (x_intermediate[2] + ang_vel_ego*x_intermediate[1]);
            float p_dot_y = (x_intermediate[3] - ang_vel_ego*x_intermediate[0]);
            ROS_INFO_STREAM_NAMED("GapEstimation", "        p_dot_x: " << p_dot_x << ", p_dot_y: " << p_dot_y);

            float vdot_x_body = intermediateRbtAccs_[i].twist.linear.x;
            float vdot_y_body = intermediateRbtAccs_[i].twist.linear.y;
            ROS_INFO_STREAM_NAMED("GapEstimation", "        vdot_x_body: " << vdot_x_body << ", vdot_y_body: " << vdot_y_body);

            float v_dot_x = (x_intermediate[3]*ang_vel_ego - vdot_x_body);
            float v_dot_y = (-x_intermediate[2]*ang_vel_ego - vdot_y_body);
            ROS_INFO_STREAM_NAMED("GapEstimation", "        v_dot_x: " << v_dot_x << ", v_dot_y: " << v_dot_y);

            new_x << x_intermediate[0] + p_dot_x*dt, // r_x
                     x_intermediate[1] + p_dot_y*dt, // r_y
                     x_intermediate[2] + v_dot_x*dt, // v_x
                     x_intermediate[3] + v_dot_y*dt; // v_y
            
            x_intermediate = new_x;
            ROS_INFO_STREAM_NAMED("GapEstimation", "        x_intermediate: " << x_intermediate[0] << ", " << x_intermediate[1] << ", " << x_intermediate[2] << ", " << x_intermediate[3]);
        
        }

        return x_intermediate;
    }

    void RotatingFrameCartesianKalmanFilter::linearize(int idx) 
    {
        float dt = (intermediateRbtVels_[idx + 1].header.stamp - intermediateRbtVels_[idx].header.stamp).toSec();    
        float ang_vel_ego = intermediateRbtVels_[idx].twist.angular.z;
        
        A_ << 0.0, ang_vel_ego, 1.0, 0.0,
             -ang_vel_ego, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0, ang_vel_ego,
             0.0, 0.0, -ang_vel_ego, 0.0;

        STM_ = (A_*dt).exp();
    }

    void RotatingFrameCartesianKalmanFilter::discretizeQ(int idx) 
    {

        // ROS_INFO_STREAM_NAMED("GapEstimation", "VxVx: " << cfg_->gap_est.Q_VxVx << ", VyVy: " << cfg_->gap_est.Q_VyVy);
        float dt = (intermediateRbtVels_[idx + 1].header.stamp - intermediateRbtVels_[idx].header.stamp).toSec();

        Q_1 = Q_k_;
        Q_2 = A_ * Q_1 + Q_1 * A_.transpose();
        Q_3 = A_ * Q_2 + Q_2 * A_.transpose();

        dQ_ = (Q_1 * dt) + (Q_2 * dt * dt / 2.0) + (Q_3 * dt * dt * dt / 6.0);
    }

    void RotatingFrameCartesianKalmanFilter::update(const Eigen::Vector2f & measurement, 
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
            ROS_INFO_STREAM_NAMED("GapEstimation", "    intermediateRbtVels_ is of size " << intermediateRbtVels_.size() << " while intermediateRbtAccs_ is of size " << intermediateRbtAccs_.size());
            return;
        }


        ROS_INFO_STREAM_NAMED("GapEstimation", "    update for model: " << getID()); // << ", life_time: " << life_time << ", dt: " << dt << ", inter_dt: " << inter_dt);
        ROS_INFO_STREAM_NAMED("GapEstimation", "    x_hat_kmin1_plus_: " << x_hat_kmin1_plus_[0] << ", " << x_hat_kmin1_plus_[1] << ", " << x_hat_kmin1_plus_[2] << ", " << x_hat_kmin1_plus_[3]);
        ROS_INFO_STREAM_NAMED("GapEstimation", "    current_rbt_vel, x_lin: " << lastRbtVel_.twist.linear.x << ", y_lin: " << lastRbtVel_.twist.linear.y << ", z_ang: " << lastRbtVel_.twist.angular.z);

        processEgoRobotVelsAndAccs(t_update);

        // get_intermediateRbtVels__accs();

        x_tilde_ = measurement; 
                // << range_bearing_measurement[0]*std::cos(range_bearing_measurement[1]),
                //    range_bearing_measurement[0]*std::sin(range_bearing_measurement[1]);
        
        ROS_INFO_STREAM_NAMED("GapEstimation", "    linear ego vel: " << lastRbtVel_.twist.linear.x << ", " << lastRbtVel_.twist.linear.y << ", angular ego vel: " << lastRbtVel_.twist.angular.z);
        ROS_INFO_STREAM_NAMED("GapEstimation", "    linear ego acceleration: " << lastRbtAcc_.twist.linear.x << ", " << lastRbtAcc_.twist.linear.y << ", angular ego acc: " << lastRbtAcc_.twist.angular.z);        


        x_ground_truth = update_ground_truth_cartesian_state();

        x_hat_k_minus_ = integrate();
        
        ROS_INFO_STREAM_NAMED("GapEstimation", "    x_hat_k_minus_: " << x_hat_k_minus_[0] << ", " << x_hat_k_minus_[1] << ", " 
                                                                      << x_hat_k_minus_[2] << ", " << x_hat_k_minus_[3]);

        P_intermediate = P_kmin1_plus_;
        new_P = P_kmin1_plus_;
        for (int i = 0; i < (intermediateRbtVels_.size() - 1); i++) 
        {
            linearize(i);

            // ROS_INFO_STREAM_NAMED("GapEstimation", "    A_: " << A_(0, 0) << ", " << A_(0, 1) << ", " << A_(0, 2) << ", " << A_(0, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "        " << A_(1, 0) << ", " << A_(1, 1) << ", " << A_(1, 2) << ", " << A_(1, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "        " << A_(2, 0) << ", " << A_(2, 1) << ", " << A_(2, 2) << ", " << A_(2, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "        " << A_(3, 0) << ", " << A_(3, 1) << ", " << A_(3, 2) << ", " << A_(3, 3));     

            // ROS_INFO_STREAM_NAMED("GapEstimation", "    STM_: " << STM_(0, 0) << ", " << STM_(0, 1) << ", " << STM_(0, 2) << ", " << STM_(0, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "          " << STM_(1, 0) << ", " << STM_(1, 1) << ", " << STM_(1, 2) << ", " << STM_(1, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "          " << STM_(2, 0) << ", " << STM_(2, 1) << ", " << STM_(2, 2) << ", " << STM_(2, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "          " << STM_(3, 0) << ", " << STM_(3, 1) << ", " << STM_(3, 2) << ", " << STM_(3, 3));     

            discretizeQ(i);

            // ROS_INFO_STREAM_NAMED("GapEstimation", "    dQ_: " << dQ_(0, 0) << ", " << dQ_(0, 1) << ", " << dQ_(0, 2) << ", " << dQ_(0, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "         " << dQ_(1, 0) << ", " << dQ_(1, 1) << ", " << dQ_(1, 2) << ", " << dQ_(1, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "         " << dQ_(2, 0) << ", " << dQ_(2, 1) << ", " << dQ_(2, 2) << ", " << dQ_(2, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "         " << dQ_(3, 0) << ", " << dQ_(3, 1) << ", " << dQ_(3, 2) << ", " << dQ_(3, 3));     

            // ROS_INFO_STREAM_NAMED("GapEstimation", "    P_intermediate: " << P_intermediate(0, 0) << ", " << P_intermediate(0, 1) << ", " << P_intermediate(0, 2) << ", " << P_intermediate(0, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "                    " << P_intermediate(1, 0) << ", " << P_intermediate(1, 1) << ", " << P_intermediate(1, 2) << ", " << P_intermediate(1, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "                    " << P_intermediate(2, 0) << ", " << P_intermediate(2, 1) << ", " << P_intermediate(2, 2) << ", " << P_intermediate(2, 3));
            // ROS_INFO_STREAM_NAMED("GapEstimation", "                    " << P_intermediate(3, 0) << ", " << P_intermediate(3, 1) << ", " << P_intermediate(3, 2) << ", " << P_intermediate(3, 3));     

            new_P = STM_ * P_intermediate * STM_.transpose() + dQ_;

            P_intermediate = new_P;
        }
        P_k_minus_ = new_P;
        

        ROS_INFO_STREAM_NAMED("GapEstimation", "    P_k_minus_: " << P_k_minus_(0, 0) << ", " << P_k_minus_(0, 1) << ", " << P_k_minus_(0, 2) << ", " << P_k_minus_(0, 3));
        ROS_INFO_STREAM_NAMED("GapEstimation", "                " << P_k_minus_(1, 0) << ", " << P_k_minus_(1, 1) << ", " << P_k_minus_(1, 2) << ", " << P_k_minus_(1, 3));
        ROS_INFO_STREAM_NAMED("GapEstimation", "                " << P_k_minus_(2, 0) << ", " << P_k_minus_(2, 1) << ", " << P_k_minus_(2, 2) << ", " << P_k_minus_(2, 3));
        ROS_INFO_STREAM_NAMED("GapEstimation", "                " << P_k_minus_(3, 0) << ", " << P_k_minus_(3, 1) << ", " << P_k_minus_(3, 2) << ", " << P_k_minus_(3, 3));     

        ROS_INFO_STREAM_NAMED("GapEstimation", "    x_tilde_: " << x_tilde_[0] << ", " << x_tilde_[1]);
        
        innovation_ = x_tilde_ - H_*x_hat_k_minus_;
        x_hat_k_plus_ = x_hat_k_minus_ + G_k_*innovation_;
        residual_ = x_tilde_ - H_*x_hat_k_plus_;
        
        float sensor_noise_factor = R_scalar * x_tilde_.norm();
        R_k_ << sensor_noise_factor, 0.0,
               0.0, sensor_noise_factor;

        // ROS_INFO_STREAM_NAMED("GapEstimation", "1");

        // ROS_INFO_STREAM_NAMED("GapEstimation", "Rxx: " << cfg_->gap_est.R_xx << ", Ryy: " << cfg_->gap_est.R_yy);
        // R_k_ << cfg_->gap_est.R_xx, 0.0,
        //        0.0, cfg_->gap_est.R_yy;

        // ROS_INFO_STREAM_NAMED("GapEstimation", "H_transpose_: " << H_transpose_(0, 0) << ", " << H_transpose_(0, 1));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "              " << H_transpose_(1, 0) << ", " << H_transpose_(1, 1));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "              " << H_transpose_(2, 0) << ", " << H_transpose_(2, 1));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "              " << H_transpose_(3, 0) << ", " << H_transpose_(3, 1));

        tmp_mat = H_*P_k_minus_*H_transpose_ + R_k_;

        G_k_ = P_k_minus_ * H_transpose_ * tmp_mat.inverse();

        // ROS_INFO_STREAM_NAMED("GapEstimation", "G_k_: " << G_k_(0, 0) << ", " << G_k_(0, 1));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "      " << G_k_(1, 0) << ", " << G_k_(1, 1));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "      " << G_k_(2, 0) << ", " << G_k_(2, 1));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "      " << G_k_(3, 0) << ", " << G_k_(3, 1));

        // ROS_INFO_STREAM_NAMED("GapEstimation", "eyes: " << eyes(0, 0) << ", " << eyes(0, 1) << ", " << eyes(0, 2) << ", " << eyes(0, 3));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "      " << eyes(1, 0) << ", " << eyes(1, 1) << ", " << eyes(1, 2) << ", " << eyes(1, 3));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "      " << eyes(2, 0) << ", " << eyes(2, 1) << ", " << eyes(2, 2) << ", " << eyes(2, 3));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "      " << eyes(3, 0) << ", " << eyes(3, 1) << ", " << eyes(3, 2) << ", " << eyes(3, 3));

        // ROS_INFO_STREAM_NAMED("GapEstimation", "H_: " << H_(0, 0) << ", " << H_(0, 1) << ", " << H_(0, 2) << ", " << H_(0, 3));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "    " << H_(1, 0) << ", " << H_(1, 1) << ", " << H_(1, 2) << ", " << H_(1, 3));
        
        P_k_plus_ = (eyes - G_k_*H_)*P_k_minus_;
    
        // ROS_INFO_STREAM_NAMED("GapEstimation", "2");

        x_hat_kmin1_plus_ = x_hat_k_plus_;
        P_kmin1_plus_ = P_k_plus_;
        t_last_update = t_update;

        // ROS_INFO_STREAM_NAMED("GapEstimation", "3");

        ROS_INFO_STREAM_NAMED("GapEstimation", "    intermediateRbtVels_ size: " << intermediateRbtVels_.size());
        ROS_INFO_STREAM_NAMED("GapEstimation", "    intermediateRbtAccs_ size: " << intermediateRbtAccs_.size());
        
        if (intermediateRbtVels_.size() > 0)
            lastRbtVel_ = intermediateRbtVels_.back();
        
        if (intermediateRbtAccs_.size() > 0)
            lastRbtAcc_ = intermediateRbtAccs_.back();

        // ROS_INFO_STREAM_NAMED("GapEstimation", "4");

        ROS_INFO_STREAM_NAMED("GapEstimation", "    x_hat_k_plus_: " << x_hat_k_plus_[0] << ", " << x_hat_k_plus_[1] << ", " << x_hat_k_plus_[2] << ", " << x_hat_k_plus_[3]);       
        // ROS_INFO_STREAM_NAMED("GapEstimation", "    P_k_plus_: " << P_k_plus_(0, 0) << ", " << P_k_plus_(0, 1) << ", " << P_k_plus_(0, 2) << ", " << P_k_plus_(0, 3));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "               " << P_k_plus_(1, 0) << ", " << P_k_plus_(1, 1) << ", " << P_k_plus_(1, 2) << ", " << P_k_plus_(1, 3));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "               " << P_k_plus_(2, 0) << ", " << P_k_plus_(2, 1) << ", " << P_k_plus_(2, 2) << ", " << P_k_plus_(2, 3));
        // ROS_INFO_STREAM_NAMED("GapEstimation", "               " << P_k_plus_(3, 0) << ", " << P_k_plus_(3, 1) << ", " << P_k_plus_(3, 2) << ", " << P_k_plus_(3, 3));             
        ROS_INFO_STREAM_NAMED("GapEstimation", "    x_GT: " << x_ground_truth[0] << ", " << x_ground_truth[1] << ", " << x_ground_truth[2] << ", " << x_ground_truth[3]);
        ROS_INFO_STREAM_NAMED("GapEstimation", "    -----------");

        return;
    }    

    void RotatingFrameCartesianKalmanFilter::plot_models() 
    {
        /*
        if (plot && !plotted && print) {
            if (life_time <= life_time_threshold) {
                std::vector<double> rel_state{x_hat_k_plus_[0], x_hat_k_plus_[1], x_hat_k_plus_[2], x_hat_k_plus_[3]};
                std::vector<double> gap_only_state{x_hat_k_plus_[0], x_hat_k_plus_[1], x_hat_k_plus_[2] + v_ego[0], x_hat_k_plus_[3] + v_ego[1]};
     
                std::vector<double> ground_truths{x_ground_truth[0], x_ground_truth[1], x_ground_truth[2], x_ground_truth[3]};
                std::vector<double> ground_truths_gap_only{x_ground_truth_gap_only[0], x_ground_truth_gap_only[1], x_ground_truth_gap_only[2], x_ground_truth_gap_only[3]};
     
                std::vector<double> ego_vels{v_ego[0], v_ego[1], v_ego[2]};
                std::vector<double> ego_accels{a_ego[0], a_ego[1], a_ego[2]};
                std::vector<double> times{life_time, dt};
  
                previous_states.push_back(rel_state);
                previous_gap_only_states.push_back(gap_only_state);
                previous_measurements.push_back(ground_truths);
                previous_measurements_gap_only.push_back(ground_truths_gap_only);
                previous_ego_accels.push_back(ego_accels);
                previous_ego_vels.push_back(ego_vels);
                previous_times.push_back(times);      
            } else {
                plot_states();
            }
        }
        */
    }

    /*
    void RotatingFrameCartesianKalmanFilter::plot_states() {
        //std::cout << "in plot states" << std::endl;
        int n = previous_states.size();
        std::vector<double> t(n), r_xs(n), r_ys(n), v_xs(n), v_ys(n), 
                            r_xs_GT(n), r_ys_GT(n), v_xs_GT(n), v_ys_GT(n),
                            v_ego_angs(n), a_ego_angs(n), a_xs(n), a_ys(n), 
                            v_x_egos(n), v_y_egos(n), dts(n), gap_only_r_xs(n),
                            gap_only_r_ys(n), gap_only_v_xs(n), gap_only_v_ys(n),
                            gap_only_r_xs_GT(n), gap_only_r_ys_GT(n), gap_only_v_xs_GT(n), gap_only_v_ys_GT(n);

        for(int i=0; i < previous_states.size(); i++) {
            t.at(i) = previous_times[i][0];
            dts.at(i) = previous_times[i][1];

            r_xs.at(i) = previous_states[i][0];
            r_ys.at(i) = previous_states[i][1];
            v_xs.at(i) = previous_states[i][2];
            v_ys.at(i) = previous_states[i][3];

            gap_only_r_xs.at(i) = previous_gap_only_states[i][0];
            gap_only_r_ys.at(i) = previous_gap_only_states[i][1];
            gap_only_v_xs.at(i) = previous_gap_only_states[i][2];
            gap_only_v_ys.at(i) = previous_gap_only_states[i][3];

            gap_only_r_xs_GT.at(i) = previous_measurements_gap_only[i][0];
            gap_only_r_ys_GT.at(i) = previous_measurements_gap_only[i][1];
            gap_only_v_xs_GT.at(i) = previous_measurements_gap_only[i][2];
            gap_only_v_ys_GT.at(i) = previous_measurements_gap_only[i][3];            

            r_xs_GT.at(i) = previous_measurements[i][0];
            r_ys_GT.at(i) = previous_measurements[i][1];
            v_xs_GT.at(i) = previous_measurements[i][2];
            v_ys_GT.at(i) = previous_measurements[i][3];

            v_x_egos.at(i) = previous_ego_vels[i][0];
            v_y_egos.at(i) = previous_ego_vels[i][1];
            v_ego_angs.at(i) = previous_ego_vels[i][2];

            a_xs.at(i) = previous_ego_accels[i][0];
            a_ys.at(i) = previous_ego_accels[i][1];
            a_ego_angs.at(i) = previous_ego_accels[i][2];
        }

        plt::figure_size(1200, 780);
        plt::scatter(t, v_x_egos, 25.0, {{"label", "v_x_ego"}});
        // plt::scatter(t, v_y_egos, 25.0, {{"label", "v_y_ego"}});
        plt::scatter(t, v_ego_angs, 25.0, {{"label", "v_ang_ego"}});
        plt::scatter(t, a_xs, 25.0, {{"label", "a_x_ego"}});
        plt::scatter(t, a_ego_angs, 25.0, {{"label", "a_ang_ego"}});
        // plt::scatter(t, a_ys, 25.0, {{"label", "a_y_ego"}});
        // plt::scatter(t, dts, 25.0, {{"label", "dts"}});
        plt::xlim(0, 15);
        plt::legend();
        plt::save(plot_dir + std::to_string(index) + "_other_info.png");
        plt::close();
        
        plt::figure_size(1200, 780);
        plt::scatter(t, gap_only_r_xs_GT, 25.0, {{"label", "r_x (GT)"}});
        plt::scatter(t, gap_only_r_xs, 25.0, {{"label", "r_x"}});
        plt::xlim(0, 15);
        plt::legend();
        plt::save(plot_dir + std::to_string(index) + "_r_x.png");
        plt::close();

        plt::figure_size(1200, 780);
        plt::scatter(t, gap_only_r_ys_GT, 25.0, {{"label", "r_y (GT)"}});
        plt::scatter(t, gap_only_r_ys, 25.0, {{"label", "r_y"}});
        plt::xlim(0, 15);
        plt::legend();
        plt::save(plot_dir + std::to_string(index) + "_r_y.png");
        plt::close();

        plt::figure_size(1200, 780);
        plt::scatter(t, gap_only_v_xs_GT, 25.0, {{"label", "v_x (GT)"}});
        plt::scatter(t, gap_only_v_xs, 25.0, {{"label", "v_x"}});
        // plt::scatter(t, v_ego_angs, 25.0, {{"label", "v_ang_ego"}});
        plt::xlim(0, 15);
        plt::legend();
        plt::save(plot_dir + std::to_string(index) + "_v_x.png");
        plt::close();

        plt::figure_size(1200, 780);
        plt::scatter(t, gap_only_v_ys_GT, 25.0, {{"label", "v_y (GT)"}});
        plt::scatter(t, gap_only_v_ys, 25.0, {{"label", "v_y"}});
        // plt::scatter(t, v_ego_angs, 25.0, {{"label", "v_ang_ego"}});
        plt::xlim(0, 15);
        plt::legend();
        plt::save(plot_dir + std::to_string(index) + "_v_y.png");
        plt::close();

        plotted = true;
    }
    */

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::update_ground_truth_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4f return_x = x_ground_truth;

        
        ROS_INFO_STREAM_NAMED("GapEstimation", "updating ground truth cartesian state");
        // ROS_INFO_STREAM_NAMED("GapEstimation", "x_tilde_: " << x_tilde_[0] << ", " << x_tilde_[1]);
        

        return_x[0] = x_tilde_[0];
        return_x[1] = x_tilde_[1];
        x_ground_truth_gap_only[0] = x_tilde_[0];
        x_ground_truth_gap_only[1] = x_tilde_[1];
        
        float robot_i_odom_dist = 0.0;
        float min_dist = std::numeric_limits<float>::infinity();
        int min_idx = -1;
        for (int i = 0; i < agentPoses_.size(); i++) 
        {
            robot_i_odom_dist = sqrt(pow(agentPoses_[i].position.x - x_hat_kmin1_plus_[0], 2) + 
                                     pow(agentPoses_[i].position.y - x_hat_kmin1_plus_[1], 2));
            
            if (robot_i_odom_dist < min_dist) 
            {
                min_dist = robot_i_odom_dist;
                min_idx = i;
            }
        }
        
        ROS_INFO_STREAM_COND_NAMED(agentPoses_.size() > 0, "GapEstimation", "closest odom: " << agentPoses_[min_idx].position.x << ", " << agentPoses_[min_idx].position.y);
        
        float min_dist_thresh = 0.4;
        if (min_dist < min_dist_thresh) 
        {    
            ROS_INFO_STREAM_NAMED("GapEstimation", "attaching to odom");
            
            
            // x_tilde_[0] = agentPoses_[min_idx].position.x;
            // x_tilde_[1] = agentPoses_[min_idx].position.y;
            // return_x[0] = x_tilde_[0];
            // return_x[1] = x_tilde_[1];
            return_x[2] = agentVels_[min_idx].vector.x - lastRbtVel_.twist.linear.x;
            return_x[3] = agentVels_[min_idx].vector.y - lastRbtVel_.twist.linear.y;

            x_ground_truth_gap_only[2] = agentVels_[min_idx].vector.x;
            x_ground_truth_gap_only[3] = agentVels_[min_idx].vector.y;
        } else 
        {    
            ROS_INFO_STREAM_NAMED("GapEstimation", "attaching to nothing");
            
            return_x[2] = 0.0 - lastRbtVel_.twist.linear.x;
            return_x[3] = 0.0 - lastRbtVel_.twist.linear.y;

            x_ground_truth_gap_only[2] = 0.0;
            x_ground_truth_gap_only[3] = 0.0;            
        }

        return return_x;
    }

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::getState() 
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

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::getTrueState() 
    {
        // x state:
        // [r_x, r_y, v_x, v_y]

        return x_ground_truth;
    }

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::getGapState() 
    {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4f return_x = frozen_x;
        return return_x;
    }

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::getRewindGapState() 
    {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4f return_x = rewind_x;
        return return_x;
    }

    geometry_msgs::TwistStamped RotatingFrameCartesianKalmanFilter::getRobotVel() 
    {
        return lastRbtVel_;
    }

    int RotatingFrameCartesianKalmanFilter::getID() 
    {
        return modelID_;
    }

    Eigen::Vector2f RotatingFrameCartesianKalmanFilter::get_x_tilde() 
    {
        return x_tilde_;
    }
}