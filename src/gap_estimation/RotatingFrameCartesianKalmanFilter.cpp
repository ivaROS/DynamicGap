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
    RotatingFrameCartesianKalmanFilter::RotatingFrameCartesianKalmanFilter(std::string _side, int _index, float init_r, float init_beta, 
                            const ros::Time & t_update, const geometry_msgs::TwistStamped & last_ego_rbt_vel,
                            const geometry_msgs::TwistStamped & last_ego_rbt_acc) 
    {
        side = _side;
        index = _index;
        initialize(init_r, init_beta, t_update, last_ego_rbt_vel, last_ego_rbt_acc);
    }

    void RotatingFrameCartesianKalmanFilter::initialize(float init_r, float init_beta,
                                const ros::Time & t_update, const geometry_msgs::TwistStamped & last_ego_rbt_vel,
                                const geometry_msgs::TwistStamped & last_ego_rbt_acc) 
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

        this->last_ego_rbt_vel = last_ego_rbt_vel;
        this->last_ego_rbt_acc = last_ego_rbt_acc;
        this->t_last_update = t_update;


        float v_rel_x = -last_ego_rbt_vel.twist.linear.x;
        float v_rel_y = -last_ego_rbt_vel.twist.linear.y;
        std::vector<float> measurement{init_r * std::cos(init_beta), 
                                        init_r * std::sin(init_beta), 
                                        v_rel_x, 
                                        v_rel_y};
        
        // ROS_INFO_STREAM("initializing model with x: " << measurement[0] << ", y: " << measurement[1]);

        x_tilde_ << measurement[0], measurement[1];
        x_hat_kmin1_plus_ << measurement[0], measurement[1], measurement[2], measurement[3];
        x_hat_k_minus_ = x_hat_kmin1_plus_; 
        x_hat_k_plus_ = x_hat_kmin1_plus_;
        x_ground_truth << measurement[0], measurement[1], 0.0, 0.0;
        x_ground_truth_gap_only << measurement[0], measurement[1], v_rel_x, v_rel_y;

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

    void RotatingFrameCartesianKalmanFilter::processEgoRobotVelsAndAccs(const ros::Time & t_update)
    {
        /*
        // Printing original dt values from intermediate odom measurements
        ROS_INFO_STREAM("   t_0 - t_last_update difference:" << (ego_rbt_vels[0].header.stamp - t_last_update).toSec() << " sec");

        for (int i = 0; i < (ego_rbt_vels.size() - 1); i++)
        {
            ROS_INFO_STREAM("   t_" << (i+1) << " - t_" << i << " difference: " << (ego_rbt_vels[i + 1].header.stamp - ego_rbt_vels[i].header.stamp).toSec() << " sec");
        }

        ROS_INFO_STREAM("   t_update" << " - t_" << (ego_rbt_vels.size() - 1) << " difference:" << (t_update - ego_rbt_vels[ego_rbt_vels.size() - 1].header.stamp).toSec() << " sec");
        */

        // Tweaking ego robot velocities/acceleration to make sure that updates:
        //      1. Are never negative (backwards in time)
        //      2. Always start from time of last update
        //      3. Always at end of time of incoming laser scan measurement

        // Erasing odometry measurements that are from *before* the last update 
        while (!ego_rbt_vels.empty() && t_last_update > ego_rbt_vels[0].header.stamp)
        {
            ego_rbt_vels.erase(ego_rbt_vels.begin());
            ego_rbt_accs.erase(ego_rbt_accs.begin());
        }

        // Inserting placeholder odometry to represent the time of the last update
        ego_rbt_vels.insert(ego_rbt_vels.begin(), last_ego_rbt_vel);
        ego_rbt_vels[0].header.stamp = t_last_update;

        ego_rbt_accs.insert(ego_rbt_accs.begin(), last_ego_rbt_acc);
        ego_rbt_accs[0].header.stamp = t_last_update;

        // Erasing odometry measurements that occur *after* the incoming laser scan was received
        while (!ego_rbt_vels.empty() && t_update < ego_rbt_vels[ego_rbt_vels.size() - 1].header.stamp)
        {
            ego_rbt_vels.erase(ego_rbt_vels.end() - 1);
            ego_rbt_accs.erase(ego_rbt_accs.end() - 1);
        }

        // Inserting placeholder odometry to represent the time that the incoming laser scan was received
        ego_rbt_vels.push_back(ego_rbt_vels[ego_rbt_vels.size() - 1]);
        ego_rbt_vels[ego_rbt_vels.size() - 1].header.stamp = t_update;

        for (int i = 0; i < (ego_rbt_vels.size() - 1); i++)
        {
            float dt = (ego_rbt_vels[i + 1].header.stamp - ego_rbt_vels[i].header.stamp).toSec();
            
            if (print)
                ROS_INFO_STREAM("   t_" << (i+1) << " - t_" << i << " difference: " << dt << " sec");
            
            if (dt < 0)
            {
                ROS_INFO_STREAM("   ERROR IN TIMESTEP CALCULATION, SHOULD NOT BE NEGATIVE");
            }
        }
    }

    void RotatingFrameCartesianKalmanFilter::isolateGapDynamics() {
        Eigen::Vector4f cartesian_state = getState();
        
        // fixing position (otherwise can get bugs)
        cartesian_state[0] = x_tilde_[0];
        cartesian_state[1] = x_tilde_[1];

        // update cartesian
        cartesian_state[2] += last_ego_rbt_vel.twist.linear.x;
        cartesian_state[3] += last_ego_rbt_vel.twist.linear.y;
        frozen_x = cartesian_state;

        //std::cout << "modified cartesian state: " << frozen_x[0] << ", " << frozen_x[1] << ", " << frozen_x[2] << ", " << frozen_x[3] << std::endl;
    }

    void RotatingFrameCartesianKalmanFilter::set_rewind_state() {
        rewind_x = frozen_x;
    }

    void RotatingFrameCartesianKalmanFilter::rewind_propagate(float rew_dt) {
        Eigen::Matrix<float, 4, 1> new_rewind_x;     
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

    void RotatingFrameCartesianKalmanFilter::frozen_state_propagate(float froz_dt) {
        Eigen::Matrix<float, 4, 1> new_frozen_x;     
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
    

    Eigen::Matrix<float, 4, 1> RotatingFrameCartesianKalmanFilter::integrate() 
    {
        if (print) ROS_INFO_STREAM("    [integrate()]");
        Eigen::Matrix<float, 4, 1> x_intermediate = x_hat_kmin1_plus_;
        Eigen::Matrix<float, 4, 1> new_x = x_hat_kmin1_plus_;

        for (int i = 0; i < (ego_rbt_vels.size() - 1); i++) 
        {
            if (print) ROS_INFO_STREAM("        intermediate step " << i);
            
            float dt = (ego_rbt_vels[i + 1].header.stamp - ego_rbt_vels[i].header.stamp).toSec();
            float ang_vel_ego = ego_rbt_vels[i].twist.angular.z;
            
            if (print) ROS_INFO_STREAM("        ang_vel_ego: " << ang_vel_ego);

            float p_dot_x = (x_intermediate[2] + ang_vel_ego*x_intermediate[1]);
            float p_dot_y = (x_intermediate[3] - ang_vel_ego*x_intermediate[0]);
            if (print) ROS_INFO_STREAM("        p_dot_x: " << p_dot_x << ", p_dot_y: " << p_dot_y);

            float vdot_x_body = ego_rbt_accs[i].twist.linear.x;
            float vdot_y_body = ego_rbt_accs[i].twist.linear.y;
            if (print) ROS_INFO_STREAM("        vdot_x_body: " << vdot_x_body << ", vdot_y_body: " << vdot_y_body);

            float v_dot_x = (x_intermediate[3]*ang_vel_ego - vdot_x_body);
            float v_dot_y = (-x_intermediate[2]*ang_vel_ego - vdot_y_body);
            if (print) ROS_INFO_STREAM("        v_dot_x: " << v_dot_x << ", v_dot_y: " << v_dot_y);

            new_x << x_intermediate[0] + p_dot_x*dt, // r_x
                     x_intermediate[1] + p_dot_y*dt, // r_y
                     x_intermediate[2] + v_dot_x*dt, // v_x
                     x_intermediate[3] + v_dot_y*dt; // v_y
            
            x_intermediate = new_x;
            if (print) ROS_INFO_STREAM("        x_intermediate: " << x_intermediate[0] << ", " << x_intermediate[1] << ", " << x_intermediate[2] << ", " << x_intermediate[3]);
        
        }

        return x_intermediate;
    }

    void RotatingFrameCartesianKalmanFilter::linearize(int idx) 
    {
        float dt = (ego_rbt_vels[idx + 1].header.stamp - ego_rbt_vels[idx].header.stamp).toSec();    
        float ang_vel_ego = ego_rbt_vels[idx].twist.angular.z;
        
        A_ << 0.0, ang_vel_ego, 1.0, 0.0,
             -ang_vel_ego, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0, ang_vel_ego,
             0.0, 0.0, -ang_vel_ego, 0.0;

        STM_ = (A_*dt).exp();
    }

    void RotatingFrameCartesianKalmanFilter::discretizeQ(int idx) 
    {

        // ROS_INFO_STREAM("VxVx: " << cfg_->gap_est.Q_VxVx << ", VyVy: " << cfg_->gap_est.Q_VyVy);
        float dt = (ego_rbt_vels[idx + 1].header.stamp - ego_rbt_vels[idx].header.stamp).toSec();

        Q_1 = Q_k_;
        Q_2 = A_ * Q_1 + Q_1 * A_.transpose();
        Q_3 = A_ * Q_2 + Q_2 * A_.transpose();

        dQ_ = (Q_1 * dt) + (Q_2 * dt * dt / 2.0) + (Q_3 * dt * dt * dt / 6.0);
    }

    void RotatingFrameCartesianKalmanFilter::update(Eigen::Matrix<float, 2, 1> range_bearing_measurement, 
                                    const std::vector<geometry_msgs::TwistStamped> & ego_rbt_vels_copied, 
                                    const std::vector<geometry_msgs::TwistStamped> & ego_rbt_accs_copied, 
                                    bool _print,
                                    const std::vector<geometry_msgs::Pose> & _agent_odoms,
                                    const std::vector<geometry_msgs::Vector3Stamped> & _agent_vels,
                                    const ros::Time & t_update) {
        
        agent_odoms = _agent_odoms;
        agent_vels = _agent_vels;
        print = _print;

        // acceleration and velocity come in wrt robot frame
        ego_rbt_vels = ego_rbt_vels_copied;
        ego_rbt_accs = ego_rbt_accs_copied;
        // last_ego_rbt_vel = _current_rbt_vel;
        // last_ego_rbt_acc = _current_rbt_acc;

        // dt = scan_dt;
        // life_time += dt;

        // inter_dt = (dt / ego_rbt_vels.size());

        if (ego_rbt_vels.size() == 0 || ego_rbt_accs.size() == 0)
            return;

        if (ego_rbt_vels.size() != ego_rbt_accs.size())
        {
            if (print) ROS_INFO_STREAM("    ego_rbt_vels is of size " << ego_rbt_vels.size() << " while ego_rbt_accs is of size " << ego_rbt_accs.size());
            return;
        }


        if (print) {
            ROS_INFO_STREAM("    update for model " << get_index()); // << ", life_time: " << life_time << ", dt: " << dt << ", inter_dt: " << inter_dt);
            ROS_INFO_STREAM("    x_hat_kmin1_plus_: " << x_hat_kmin1_plus_[0] << ", " << x_hat_kmin1_plus_[1] << ", " << x_hat_kmin1_plus_[2] << ", " << x_hat_kmin1_plus_[3]);
            ROS_INFO_STREAM("    current_rbt_vel, x_lin: " << last_ego_rbt_vel.twist.linear.x << ", y_lin: " << last_ego_rbt_vel.twist.linear.y << ", z_ang: " << last_ego_rbt_vel.twist.angular.z);
        }

        processEgoRobotVelsAndAccs(t_update);

        // get_ego_rbt_vels_accs();

        x_tilde_ << range_bearing_measurement[0]*std::cos(range_bearing_measurement[1]),
                   range_bearing_measurement[0]*std::sin(range_bearing_measurement[1]);
        
        if (print) {
            ROS_INFO_STREAM("    linear ego vel: " << last_ego_rbt_vel.twist.linear.x << ", " << last_ego_rbt_vel.twist.linear.y << ", angular ego vel: " << last_ego_rbt_vel.twist.angular.z);
            ROS_INFO_STREAM("    linear ego acceleration: " << last_ego_rbt_acc.twist.linear.x << ", " << last_ego_rbt_acc.twist.linear.y << ", angular ego acc: " << last_ego_rbt_acc.twist.angular.z);
        }
        

        x_ground_truth = update_ground_truth_cartesian_state();

        x_hat_k_minus_ = integrate();

        
        if (print) ROS_INFO_STREAM("    x_hat_k_minus_: " << x_hat_k_minus_[0] << ", " << x_hat_k_minus_[1] << ", " 
                                                         << x_hat_k_minus_[2] << ", " << x_hat_k_minus_[3]);
        

        P_intermediate = P_kmin1_plus_;
        new_P = P_kmin1_plus_;
        for (int i = 0; i < (ego_rbt_vels.size() - 1); i++) 
        {
            linearize(i);

            discretizeQ(i);

            new_P = STM_ * P_intermediate * STM_.transpose() + dQ_;

            P_intermediate = new_P;
        }
        P_k_minus_ = new_P;
        
        /*
        if (print) {
            ROS_INFO_STREAM("P_k_minus_: " << P_k_minus_(0, 0) << ", " << P_k_minus_(0, 1) << ", " << P_k_minus_(0, 2) << ", " << P_k_minus_(0, 3));
            ROS_INFO_STREAM("           " << P_k_minus_(1, 0) << ", " << P_k_minus_(1, 1) << ", " << P_k_minus_(1, 2) << ", " << P_k_minus_(1, 3));
            ROS_INFO_STREAM("           " << P_k_minus_(2, 0) << ", " << P_k_minus_(2, 1) << ", " << P_k_minus_(2, 2) << ", " << P_k_minus_(2, 3));
            ROS_INFO_STREAM("           " << P_k_minus_(3, 0) << ", " << P_k_minus_(3, 1) << ", " << P_k_minus_(3, 2) << ", " << P_k_minus_(3, 3));     
        }
        */

        if (print) {
            ROS_INFO_STREAM("    x_tilde_: " << x_tilde_[0] << ", " << x_tilde_[1]);
        }
        
        innovation_ = x_tilde_ - H_*x_hat_k_minus_;
        x_hat_k_plus_ = x_hat_k_minus_ + G_k_*innovation_;
        residual_ = x_tilde_ - H_*x_hat_k_plus_;
        
        float sensor_noise_factor = R_scalar * range_bearing_measurement[0];
        R_k_ << sensor_noise_factor, 0.0,
               0.0, sensor_noise_factor;

        // if (print) ROS_INFO_STREAM("1");

        // ROS_INFO_STREAM("Rxx: " << cfg_->gap_est.R_xx << ", Ryy: " << cfg_->gap_est.R_yy);
        // R_k_ << cfg_->gap_est.R_xx, 0.0,
        //        0.0, cfg_->gap_est.R_yy;

        tmp_mat = H_*P_k_minus_*H_transpose_ + R_k_;

        G_k_ = P_k_minus_ * H_transpose_ * tmp_mat.inverse();

        P_k_plus_ = (eyes - G_k_*H_)*P_k_minus_;
    
        // if (print) ROS_INFO_STREAM("2");

        x_hat_kmin1_plus_ = x_hat_k_plus_;
        P_kmin1_plus_ = P_k_plus_;
        t_last_update = t_update;

        // if (print) ROS_INFO_STREAM("3");

        // if (print)
        // {
        //     ROS_INFO_STREAM("ego_rbt_vels size: " << ego_rbt_vels.size());
        //     ROS_INFO_STREAM("ego_rbt_accs size: " << ego_rbt_accs.size());
        // }

        
        if (ego_rbt_vels.size() > 0)
            last_ego_rbt_vel = ego_rbt_vels.back();
        
        if (ego_rbt_accs.size() > 0)
            last_ego_rbt_acc = ego_rbt_accs.back();

        // if (print) ROS_INFO_STREAM("4");

        if (print) 
        {
            /*
            ROS_INFO_STREAM("G_k_: " << G_k_(0, 0) << ", " << G_k_(0, 1));
            ROS_INFO_STREAM("     " << G_k_(1, 0) << ", " << G_k_(1, 1));
            ROS_INFO_STREAM("     " << G_k_(2, 0) << ", " << G_k_(2, 1));
            ROS_INFO_STREAM("     " << G_k_(3, 0) << ", " << G_k_(3, 1));
            */

            ROS_INFO_STREAM("    x_hat_k_plus_: " << x_hat_k_plus_[0] << ", " << x_hat_k_plus_[1] << ", " << x_hat_k_plus_[2] << ", " << x_hat_k_plus_[3]);       
            ROS_INFO_STREAM("    x_GT: " << x_ground_truth[0] << ", " << x_ground_truth[1] << ", " << x_ground_truth[2] << ", " << x_ground_truth[3]);
            ROS_INFO_STREAM("    -----------");
        }

        return;
    }    

    void RotatingFrameCartesianKalmanFilter::plot_models() {
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

        
        if (print) {
            ROS_INFO_STREAM("updating ground truth cartesian state");
            // ROS_INFO_STREAM("x_tilde_: " << x_tilde_[0] << ", " << x_tilde_[1]);
        }
        

        return_x[0] = x_tilde_[0];
        return_x[1] = x_tilde_[1];
        x_ground_truth_gap_only[0] = x_tilde_[0];
        x_ground_truth_gap_only[1] = x_tilde_[1];
        
        float robot_i_odom_dist;
        float min_dist = std::numeric_limits<float>::infinity();
        int min_idx = -1;
        for (int i = 0; i < agent_odoms.size(); i++) {
            robot_i_odom_dist = sqrt(pow(agent_odoms[i].position.x - x_hat_kmin1_plus_[0], 2) + 
                                     pow(agent_odoms[i].position.y - x_hat_kmin1_plus_[1], 2));
            
            if (robot_i_odom_dist < min_dist) {
                min_dist = robot_i_odom_dist;
                min_idx = i;
            }
        }
        
        
        if (print) {
            if (agent_odoms.size() > 0) {
                ROS_INFO_STREAM("closest odom: " << agent_odoms[min_idx].position.x << ", " << agent_odoms[min_idx].position.y);
            }
        }
        
        float min_dist_thresh = 0.4;
        if (min_dist < min_dist_thresh) {
            
            
            if (print) {
                ROS_INFO_STREAM("attaching to odom");
            }
            
            
            // x_tilde_[0] = agent_odoms[min_idx].position.x;
            // x_tilde_[1] = agent_odoms[min_idx].position.y;
            // return_x[0] = x_tilde_[0];
            // return_x[1] = x_tilde_[1];
            return_x[2] = agent_vels[min_idx].vector.x - last_ego_rbt_vel.twist.linear.x;
            return_x[3] = agent_vels[min_idx].vector.y - last_ego_rbt_vel.twist.linear.y;

            x_ground_truth_gap_only[2] = agent_vels[min_idx].vector.x;
            x_ground_truth_gap_only[3] = agent_vels[min_idx].vector.y;
        } else {
            
            if (print) {
                ROS_INFO_STREAM("attaching to nothing");
            }
            
            return_x[2] = 0.0 - last_ego_rbt_vel.twist.linear.x;
            return_x[3] = 0.0 - last_ego_rbt_vel.twist.linear.y;

            x_ground_truth_gap_only[2] = 0.0;
            x_ground_truth_gap_only[3] = 0.0;            
        }

        return return_x;
    }

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::getState() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4f return_x = (perfect) ? x_ground_truth : x_hat_k_plus_;
        
        /*
        if (life_time < 0.25)
        {
            // not trusting early measurements, just treating gap as static
            return_x[2] = 0.0 - last_ego_rbt_vel.twist.linear.x;
            return_x[3] = 0.0 - last_ego_rbt_vel.twist.linear.y;
        }
        */

        return return_x;
    }

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::getTrueState() {
        // x state:
        // [r_x, r_y, v_x, v_y]

        return x_ground_truth;
    }

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::getGapState() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4f return_x = frozen_x;
        return return_x;
    }

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::get_rewind_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4f return_x = rewind_x;
        return return_x;
    }

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::get_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4f mp_state;
        Eigen::Vector4f cart_state = getState();
        mp_state << 1.0 / sqrt(pow(cart_state[0], 2) + pow(cart_state[1], 2)),
                    std::atan2(cart_state[1], cart_state[0]),
                    (cart_state[0]*cart_state[2] + cart_state[1]*cart_state[3]) / (pow(cart_state[0], 2) + pow(cart_state[1], 2)),
                    (cart_state[0]*cart_state[3] - cart_state[1]*cart_state[2]) / (pow(cart_state[0], 2) + pow(cart_state[1], 2));
        return mp_state;
    }

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::get_rewind_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4f rewind_mp_state;
        Eigen::Vector4f rewind_cart_state = get_rewind_cartesian_state();
        rewind_mp_state << 1.0 / sqrt(pow(rewind_cart_state[0], 2) + pow(rewind_cart_state[1], 2)),
                           std::atan2(rewind_cart_state[1], rewind_cart_state[0]),
                           (rewind_cart_state[0]*rewind_cart_state[2] + rewind_cart_state[1]*rewind_cart_state[3]) / (pow(rewind_cart_state[0], 2) + pow(rewind_cart_state[1], 2)),
                           (rewind_cart_state[0]*rewind_cart_state[3] - rewind_cart_state[1]*rewind_cart_state[2]) / (pow(rewind_cart_state[0], 2) + pow(rewind_cart_state[1], 2));
        return rewind_mp_state;
    }
   

    Eigen::Vector4f RotatingFrameCartesianKalmanFilter::get_frozen_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4f frozen_mp_state;
        Eigen::Vector4f frozen_cart_state = getGapState();
        frozen_mp_state << 1.0 / sqrt(pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2)),
                           std::atan2(frozen_cart_state[1], frozen_cart_state[0]),
                           (frozen_cart_state[0]*frozen_cart_state[2] + frozen_cart_state[1]*frozen_cart_state[3]) / (pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2)),
                           (frozen_cart_state[0]*frozen_cart_state[3] - frozen_cart_state[1]*frozen_cart_state[2]) / (pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2));
        return frozen_mp_state;
    }

    geometry_msgs::TwistStamped RotatingFrameCartesianKalmanFilter::getRobotVel() {
        return last_ego_rbt_vel;
    }

    int RotatingFrameCartesianKalmanFilter::get_index() {
        return index;
    }

    Eigen::Vector2f RotatingFrameCartesianKalmanFilter::get_x_tilde() {
        return x_tilde_;
    }
}