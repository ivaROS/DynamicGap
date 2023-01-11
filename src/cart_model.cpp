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
#include <dynamic_gap/cart_model.h>
// #include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <limits>
// #include <sstream>
// #include <unsupported/Eigen/MatrixFunctions>
// #include "/home/masselmeier/Desktop/Research/vcpkg/installed/x64-linux/include/matplotlibcpp.h"
// namespace plt = matplotlibcpp;

namespace dynamic_gap {
    cart_model::cart_model(std::string _side, int _index, double init_r, double init_beta, geometry_msgs::Twist _current_rbt_vel) {
        side = _side;
        index = _index;
        initialize(init_r, init_beta, _current_rbt_vel);
    }

    void cart_model::initialize(double init_r, double init_beta, geometry_msgs::Twist _current_rbt_vel) {
        // OBSERVATION MATRIX
        H << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0;
        H_transpose = H.transpose();

        // MEASUREMENT NOISE
        R_k << 0.1, 0.0,
               0.0, 0.1;

        // PROCESS NOISE        
        Q_k << 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.3, 0.0,
             0.0, 0.0, 0.0, 0.3;
        
        // COVARIANCE MATRIX
        // covariance/uncertainty of state variables (r_x, r_y, v_x, v_y)
        // larger P_0 helps with GT values that are non-zero
        // larger P_0 gives more weight to measurements (behaves like Q)
        P_kmin1_plus << 0.01, 0.0, 0.0, 0.0,
                        0.0, 0.01, 0.0, 0.0,
                        0.0, 0.0, 0.1, 0.0,
                        0.0, 0.0, 0.0, 0.1;
        P_k_minus = P_kmin1_plus;
        P_k_plus = P_kmin1_plus;

        current_rbt_vel = _current_rbt_vel;
        current_rbt_acc = geometry_msgs::TwistStamped();

        double v_rel_x = -current_rbt_vel.linear.x;
        double v_rel_y = -current_rbt_vel.linear.y;
        std::vector<double> measurement{init_r * std::cos(init_beta), 
                                        init_r * std::sin(init_beta), 
                                        v_rel_x, 
                                        v_rel_y};
        
        x_tilde << measurement[0], measurement[1];
        x_hat_kmin1_plus << measurement[0], measurement[1], v_rel_x, v_rel_y;
        x_hat_k_minus = x_hat_kmin1_plus; 
        x_hat_k_plus = x_hat_kmin1_plus;
        x_ground_truth << measurement[0], measurement[1], 0.0, 0.0;
        x_ground_truth_gap_only << measurement[0], measurement[1], v_rel_x, v_rel_y;

        G_k << 1.0, 1.0,
             1.0, 1.0,
             1.0, 1.0,
             1.0, 1.0;

        dt = 0.0;

        A << 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0;
        STM = A;

        frozen_x << 0.0, 0.0, 0.0, 0.0;

        initialized = true;
        life_time = 0.0;
        life_time_threshold = 7.5;
        eyes = Eigen::MatrixXd::Identity(4,4);
        inverted_tmp_mat << 0.0, 0.0, 0.0, 0.0;

        alpha_Q = 0.9;
        alpha_R = 0.9;
        plot_dir = "/home/masselmeier/catkin_ws/src/DynamicGap/estimator_plots/";   
        perfect = false;
        plotted = false;
        plot = true;
    }

    void cart_model::freeze_robot_vel() {
        Eigen::Vector4d cartesian_state = get_cartesian_state();
        
        // fixing position (otherwise can get bugs)
        cartesian_state[0] = x_tilde[0];
        cartesian_state[1] = x_tilde[1];

        // update cartesian
        cartesian_state[2] += current_rbt_vel.linear.x;
        cartesian_state[3] += current_rbt_vel.linear.y;
        frozen_x = cartesian_state;

        //std::cout << "modified cartesian state: " << frozen_x[0] << ", " << frozen_x[1] << ", " << frozen_x[2] << ", " << frozen_x[3] << std::endl;
    }

    void cart_model::set_rewind_state() {
        rewind_x = frozen_x;
    }

    void cart_model::rewind_propagate(double rew_dt) {
        Eigen::Matrix<double, 4, 1> new_rewind_x;     
        new_rewind_x << 0.0, 0.0, 0.0, 0.0;

        Eigen::Vector2d frozen_linear_acc_ego(0.0, 0.0);

        Eigen::Vector2d frozen_linear_vel_ego(0.0, 0.0); 
        double frozen_ang_vel_ego = 0.0;

        double vdot_x_body = frozen_linear_acc_ego[0];
        double vdot_y_body = frozen_linear_acc_ego[1];

        // discrete euler update of state (ignoring rbt acceleration, set as 0)
        new_rewind_x[0] = rewind_x[0] + (rewind_x[2] + rewind_x[1]*frozen_ang_vel_ego)*rew_dt;
        new_rewind_x[1] = rewind_x[1] + (rewind_x[3] - rewind_x[0]*frozen_ang_vel_ego)*rew_dt;
        new_rewind_x[2] = rewind_x[2] + (rewind_x[3]*frozen_ang_vel_ego - vdot_x_body)*rew_dt;
        new_rewind_x[3] = rewind_x[3] + (-rewind_x[2]*frozen_ang_vel_ego - vdot_y_body)*rew_dt;
        rewind_x = new_rewind_x; 
    }

    void cart_model::frozen_state_propagate(double froz_dt) {
        Eigen::Matrix<double, 4, 1> new_frozen_x;     
        new_frozen_x << 0.0, 0.0, 0.0, 0.0;

        Eigen::Vector2d frozen_linear_acc_ego(0.0, 0.0);

        Eigen::Vector2d frozen_linear_vel_ego(0.0, 0.0); 
        double frozen_ang_vel_ego = 0.0;

        double vdot_x_body = frozen_linear_acc_ego[0];
        double vdot_y_body = frozen_linear_acc_ego[1];

        // discrete euler update of state (ignoring rbt acceleration, set as 0)
        new_frozen_x[0] = frozen_x[0] + (frozen_x[2] + frozen_x[1]*frozen_ang_vel_ego)*froz_dt;
        new_frozen_x[1] = frozen_x[1] + (frozen_x[3] - frozen_x[0]*frozen_ang_vel_ego)*froz_dt;
        new_frozen_x[2] = frozen_x[2] + (frozen_x[3]*frozen_ang_vel_ego - vdot_x_body)*froz_dt;
        new_frozen_x[3] = frozen_x[3] + (-frozen_x[2]*frozen_ang_vel_ego - vdot_y_body)*froz_dt;
        frozen_x = new_frozen_x; 
    }
    

    Eigen::Matrix<double, 4, 1> cart_model::integrate() {
        // ROS_INFO_STREAM("INTEGRATING");
        Eigen::Matrix<double, 4, 1> x_intermediate = x_hat_kmin1_plus;
        Eigen::Matrix<double, 4, 1> new_x = x_hat_kmin1_plus;

        for (int i = 0; i < intermediate_vels.size(); i++) {
            if (print) { ROS_INFO_STREAM("intermediate step " << i); }
            
            double ang_vel_ego = intermediate_vels[i].angular.z;
            
            if (print) { ROS_INFO_STREAM("ang_vel_ego: " << ang_vel_ego);}

            double p_dot_x = (x_intermediate[2] + ang_vel_ego*x_intermediate[1]);
            double p_dot_y = (x_intermediate[3] - ang_vel_ego*x_intermediate[0]);
            if (print) { ROS_INFO_STREAM("p_dot_x: " << p_dot_x << ", p_dot_y: " << p_dot_y);}

            double vdot_x_body = intermediate_accs[i].twist.linear.x;
            double vdot_y_body = intermediate_accs[i].twist.linear.y;
            if (print) { ROS_INFO_STREAM("vdot_x_body: " << vdot_x_body << ", vdot_y_body: " << vdot_y_body);}

            double v_dot_x = (x_intermediate[3]*ang_vel_ego - vdot_x_body);
            double v_dot_y = (-x_intermediate[2]*ang_vel_ego - vdot_y_body);
            if (print) { ROS_INFO_STREAM("v_dot_x: " << v_dot_x << ", v_dot_y: " << v_dot_y); }

            new_x << x_intermediate[0] + p_dot_x*inter_dt, // r_x
                     x_intermediate[1] + p_dot_y*inter_dt, // r_y
                     x_intermediate[2] + v_dot_x*inter_dt, // v_x
                     x_intermediate[3] + v_dot_y*inter_dt; // v_y
            
            x_intermediate = new_x;
            if (print) { ROS_INFO_STREAM("x_intermediate: " << x_intermediate[0] << ", " << x_intermediate[1] << ", " << x_intermediate[2] << ", " << x_intermediate[3]); }
        
        }

        return x_intermediate;
    }

    void cart_model::linearize(int idx) {

        double ang_vel_ego = intermediate_vels[idx].angular.z;
        
        A << 0.0, ang_vel_ego, 1.0, 0.0,
             -ang_vel_ego, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0, ang_vel_ego,
             0.0, 0.0, -ang_vel_ego, 0.0;

        STM = (A*inter_dt).exp();
    }

    void cart_model::discretizeQ() {
        Q_1 = Q_k;
        Q_2 = A * Q_1 + Q_1 * A.transpose();
        Q_3 = A * Q_2 + Q_2 * A.transpose();

        dQ = (Q_1 * inter_dt) + (Q_2 * inter_dt * inter_dt / 2.0) + (Q_3 * inter_dt * inter_dt * inter_dt / 6.0);
    }

    void cart_model::kf_update_loop(Eigen::Matrix<double, 2, 1> range_bearing_measurement, 
                                    geometry_msgs::Twist _current_rbt_vel, 
                                    geometry_msgs::TwistStamped _current_rbt_acc, 
                                    std::vector<geometry_msgs::Twist> _intermediate_vels,
                                    std::vector<geometry_msgs::TwistStamped> _intermediate_accs,
                                    bool _print,
                                    std::vector<geometry_msgs::Pose> _agent_odoms,
                                    std::vector<geometry_msgs::Vector3Stamped> _agent_vels,
                                    double scan_dt) {
        
        agent_odoms = _agent_odoms;
        agent_vels = _agent_vels;
        print = _print;

        // acceleration and velocity come in wrt robot frame
        intermediate_vels = _intermediate_vels;
        intermediate_accs = _intermediate_accs;
        current_rbt_vel = _current_rbt_vel;
        current_rbt_acc = _current_rbt_acc;

        dt = scan_dt;
        life_time += dt;

        inter_dt = (dt / intermediate_vels.size());

        if (print) {
            ROS_INFO_STREAM("update for model " << get_index() << ", life_time: " << life_time << ", dt: " << dt << ", inter_dt: " << inter_dt);
            ROS_INFO_STREAM("x_hat_kmin1_plus: " << x_hat_kmin1_plus[0] << ", " << x_hat_kmin1_plus[1] << ", " << x_hat_kmin1_plus[2] << ", " << x_hat_kmin1_plus[3]);
            ROS_INFO_STREAM("current_rbt_vel, x_lin: " << _current_rbt_vel.linear.x << ", y_lin: " << _current_rbt_vel.linear.y << ", z_ang: " << _current_rbt_vel.angular.z);
        }

        // get_intermediate_vels_accs();

        x_tilde << range_bearing_measurement[0]*std::cos(range_bearing_measurement[1]),
                   range_bearing_measurement[0]*std::sin(range_bearing_measurement[1]);
        
        if (print) {
            ROS_INFO_STREAM("linear ego vel: " << current_rbt_vel.linear.x << ", " << current_rbt_vel.linear.y << ", angular ego vel: " << current_rbt_vel.angular.z);
            ROS_INFO_STREAM("linear ego acceleration: " << current_rbt_acc.twist.linear.x << ", " << current_rbt_acc.twist.linear.y << ", angular ego acc: " << current_rbt_acc.twist.angular.z);
        }
        

        x_ground_truth = update_ground_truth_cartesian_state();

        if (print) { ROS_INFO_STREAM("INTEGRATING"); }
        x_hat_k_minus = integrate();

        
        if (print) {
            ROS_INFO_STREAM("x_hat_k_minus: " << x_hat_k_minus[0] << ", " << x_hat_k_minus[1] << ", " << x_hat_k_minus[2] << ", " << x_hat_k_minus[3]);
        }
        

        P_intermediate = P_kmin1_plus;
        for (int i = 0; i < intermediate_vels.size(); i++) {
            linearize(i);

            discretizeQ();

            new_P = STM * P_intermediate * STM.transpose() + dQ;

            P_intermediate = new_P;
        }
        P_k_minus = new_P;
        
        /*
        if (print) {
            ROS_INFO_STREAM("P_k_minus: " << P_k_minus(0, 0) << ", " << P_k_minus(0, 1) << ", " << P_k_minus(0, 2) << ", " << P_k_minus(0, 3));
            ROS_INFO_STREAM("           " << P_k_minus(1, 0) << ", " << P_k_minus(1, 1) << ", " << P_k_minus(1, 2) << ", " << P_k_minus(1, 3));
            ROS_INFO_STREAM("           " << P_k_minus(2, 0) << ", " << P_k_minus(2, 1) << ", " << P_k_minus(2, 2) << ", " << P_k_minus(2, 3));
            ROS_INFO_STREAM("           " << P_k_minus(3, 0) << ", " << P_k_minus(3, 1) << ", " << P_k_minus(3, 2) << ", " << P_k_minus(3, 3));     
        }
        */

        if (print) {
            ROS_INFO_STREAM("x_tilde: " << x_tilde[0] << ", " << x_tilde[1]);
        }
        
        innovation = x_tilde - H*x_hat_k_minus;
        x_hat_k_plus = x_hat_k_minus + G_k*innovation;
        residual = x_tilde - H*x_hat_k_plus;
        
        double sensor_noise_factor = 0.01 * range_bearing_measurement[0];
        R_k << sensor_noise_factor, 0.0,
               0.0, sensor_noise_factor;

        // new_R = (alpha_R * R_k) + (1.0 - alpha_R)*(residual * residual.transpose() + H*P_k_minus*H_transpose);
        // R_k = new_R;

        tmp_mat = H*P_k_minus*H_transpose + R_k;

        G_k = P_k_minus * H_transpose * tmp_mat.inverse();

        P_k_plus = (eyes - G_k*H)*P_k_minus;
        
        // new_Q = (alpha_Q * Q_k) + (1.0 - alpha_Q) * (G_k * residual * residual.transpose() * G_k.transpose());
        // Q_k = new_Q;

        
        if (print) {
            /*
            ROS_INFO_STREAM("G_k: " << G_k(0, 0) << ", " << G_k(0, 1));
            ROS_INFO_STREAM("     " << G_k(1, 0) << ", " << G_k(1, 1));
            ROS_INFO_STREAM("     " << G_k(2, 0) << ", " << G_k(2, 1));
            ROS_INFO_STREAM("     " << G_k(3, 0) << ", " << G_k(3, 1));
            */

            ROS_INFO_STREAM("x_hat_k_plus: " << x_hat_k_plus[0] << ", " << x_hat_k_plus[1] << ", " << x_hat_k_plus[2] << ", " << x_hat_k_plus[3]);       
            ROS_INFO_STREAM("x_GT: " << x_ground_truth[0] << ", " << x_ground_truth[1] << ", " << x_ground_truth[2] << ", " << x_ground_truth[3]);
            ROS_INFO_STREAM("-----------");
        }
        
        x_hat_kmin1_plus = x_hat_k_plus;
        P_kmin1_plus = P_k_plus;
        return;
    }    

    /*
    void cart_model::get_intermediate_vels_accs() {
        
        // need this so vector is not empty
        if (intermediate_vels.size() == 0) {
            intermediate_vels.push_back(current_rbt_vel); // just adding most recent value
        }
        
        if (print) { 
            ROS_INFO_STREAM("intermediate velocities:");
            for (int i = 0; i < intermediate_vels.size(); i++) {
                ROS_INFO_STREAM("v_x: " << intermediate_vels[i].linear.x << ", v_y: " << intermediate_vels[i].linear.y << ", v_ang: " << intermediate_vels[i].angular.z); 
            }
        }

        if (intermediate_accs.size() == 0) { 
            intermediate_accs.push_back(current_rbt_acc); // just adding most recent value
        }

        if (print) { 
            ROS_INFO_STREAM("intermediate accelerations:");
            for (int i = 0; i < intermediate_accs.size(); i++) {
                ROS_INFO_STREAM("v_x: " << intermediate_accs[i].twist.linear.x << ", v_y: " << intermediate_accs[i].twist.linear.y << ", v_ang: " << intermediate_accs[i].twist.angular.z);
            }
        }


        int int_vels_size = intermediate_vels.size();
        int int_accs_size = intermediate_accs.size();
        if (int_vels_size != int_accs_size) {
            // ROS_INFO_STREAM("int_vels_size: " << int_vels_size << ", int_accs_size: " << int_accs_size);
            if (int_vels_size > int_accs_size) {
                // pad accels
                for (int i = 0; i < (int_vels_size - int_accs_size); i++) {
                    // ROS_INFO_STREAM("padding accel " << pad_acc.twist.linear.x << ", " << pad_acc.twist.linear.y << ", " << pad_acc.twist.angular.z);
                    intermediate_accs.push_back(current_rbt_acc);
                }
            } else {
                // pad vels
                geometry_msgs::Twist pad_vel = intermediate_vels[intermediate_vels.size() - 1];
                for (int i = 0; i < (int_accs_size - int_vels_size); i++) {
                    // ROS_INFO_STREAM("padding with vel " << pad_vel.linear.x << ", " << pad_vel.linear.y << ", " << pad_vel.angular.z);
                    intermediate_vels.push_back(current_rbt_vel);
                }
            }
        }

        if (print) {
            ROS_INFO_STREAM("revised intermediate velocities:");
            for (int i = 0; i < intermediate_vels.size(); i++) {
                ROS_INFO_STREAM("v_x: " << intermediate_vels[i].linear.x << ", v_y: " << intermediate_vels[i].linear.y << ", v_ang: " << intermediate_vels[i].angular.z);             
            }

            ROS_INFO_STREAM("revised intermediate accelerations:");
            for (int i = 0; i < intermediate_accs.size(); i++) {
                ROS_INFO_STREAM("v_x: " << intermediate_accs[i].twist.linear.x << ", v_y: " << intermediate_accs[i].twist.linear.y << ", v_ang: " << intermediate_accs[i].twist.angular.z);            
            }
        }    
    }
    */

    void cart_model::plot_models() {
        /*
        if (plot && !plotted && print) {
            if (life_time <= life_time_threshold) {
                std::vector<double> rel_state{x_hat_k_plus[0], x_hat_k_plus[1], x_hat_k_plus[2], x_hat_k_plus[3]};
                std::vector<double> gap_only_state{x_hat_k_plus[0], x_hat_k_plus[1], x_hat_k_plus[2] + v_ego[0], x_hat_k_plus[3] + v_ego[1]};
     
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
    void cart_model::plot_states() {
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

    Eigen::Vector4d cart_model::update_ground_truth_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d return_x = x_ground_truth;

        
        if (print) {
            ROS_INFO_STREAM("updating ground truth cartesian state");
            // ROS_INFO_STREAM("x_tilde: " << x_tilde[0] << ", " << x_tilde[1]);
        }
        

        return_x[0] = x_tilde[0];
        return_x[1] = x_tilde[1];
        x_ground_truth_gap_only[0] = x_tilde[0];
        x_ground_truth_gap_only[1] = x_tilde[1];
        
        double robot_i_odom_dist;
        double min_dist = std::numeric_limits<double>::infinity();
        int min_idx = -1;
        for (int i = 0; i < agent_odoms.size(); i++) {
            robot_i_odom_dist = sqrt(pow(agent_odoms[i].position.x - x_hat_kmin1_plus[0], 2) + 
                                     pow(agent_odoms[i].position.y - x_hat_kmin1_plus[1], 2));
            
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
        
        

        double min_dist_thresh = 0.4;
        if (min_dist < min_dist_thresh) {
            
            
            if (print) {
                ROS_INFO_STREAM("attaching to odom");
            }
            
            
            // x_tilde[0] = agent_odoms[min_idx].position.x;
            // x_tilde[1] = agent_odoms[min_idx].position.y;
            // return_x[0] = x_tilde[0];
            // return_x[1] = x_tilde[1];
            return_x[2] = agent_vels[min_idx].vector.x - current_rbt_vel.linear.x;
            return_x[3] = agent_vels[min_idx].vector.y - current_rbt_vel.linear.y;

            x_ground_truth_gap_only[2] = agent_vels[min_idx].vector.x;
            x_ground_truth_gap_only[3] = agent_vels[min_idx].vector.y;
        } else {
            
            if (print) {
                ROS_INFO_STREAM("attaching to nothing");
            }
            
            return_x[2] = 0.0 - current_rbt_vel.linear.x;
            return_x[3] = 0.0 - current_rbt_vel.linear.y;

            x_ground_truth_gap_only[2] = 0.0;
            x_ground_truth_gap_only[3] = 0.0;            
        }

        return return_x;
    }

    Eigen::Vector4d cart_model::get_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d return_x = (perfect) ? x_ground_truth : x_hat_k_plus;

        return return_x;
    }

    Eigen::Vector4d cart_model::get_GT_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]

        return x_ground_truth;
    }

    Eigen::Vector4d cart_model::get_frozen_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d return_x = frozen_x;
        return return_x;
    }

    Eigen::Vector4d cart_model::get_rewind_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d return_x = rewind_x;
        return return_x;
    }

    Eigen::Vector4d cart_model::get_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4d mp_state;
        Eigen::Vector4d cart_state = get_cartesian_state();
        mp_state << 1.0 / sqrt(pow(cart_state[0], 2) + pow(cart_state[1], 2)),
                    std::atan2(cart_state[1], cart_state[0]),
                    (cart_state[0]*cart_state[2] + cart_state[1]*cart_state[3]) / (pow(cart_state[0], 2) + pow(cart_state[1], 2)),
                    (cart_state[0]*cart_state[3] - cart_state[1]*cart_state[2]) / (pow(cart_state[0], 2) + pow(cart_state[1], 2));
        return mp_state;
    }

    Eigen::Vector4d cart_model::get_rewind_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4d rewind_mp_state;
        Eigen::Vector4d rewind_cart_state = get_rewind_cartesian_state();
        rewind_mp_state << 1.0 / sqrt(pow(rewind_cart_state[0], 2) + pow(rewind_cart_state[1], 2)),
                           std::atan2(rewind_cart_state[1], rewind_cart_state[0]),
                           (rewind_cart_state[0]*rewind_cart_state[2] + rewind_cart_state[1]*rewind_cart_state[3]) / (pow(rewind_cart_state[0], 2) + pow(rewind_cart_state[1], 2)),
                           (rewind_cart_state[0]*rewind_cart_state[3] - rewind_cart_state[1]*rewind_cart_state[2]) / (pow(rewind_cart_state[0], 2) + pow(rewind_cart_state[1], 2));
        return rewind_mp_state;
    }
   

    Eigen::Vector4d cart_model::get_frozen_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4d frozen_mp_state;
        Eigen::Vector4d frozen_cart_state = get_frozen_cartesian_state();
        frozen_mp_state << 1.0 / sqrt(pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2)),
                           std::atan2(frozen_cart_state[1], frozen_cart_state[0]),
                           (frozen_cart_state[0]*frozen_cart_state[2] + frozen_cart_state[1]*frozen_cart_state[3]) / (pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2)),
                           (frozen_cart_state[0]*frozen_cart_state[3] - frozen_cart_state[1]*frozen_cart_state[2]) / (pow(frozen_cart_state[0], 2) + pow(frozen_cart_state[1], 2));
        return frozen_mp_state;
    }

    geometry_msgs::Twist cart_model::get_v_ego() {
        return current_rbt_vel;
    }

    void cart_model::set_side(std::string _side) {
        side = _side;
    }
    
    std::string cart_model::get_side() {
        return side;
    }

    int cart_model::get_index() {
        return index;
    }

    Eigen::Vector2d cart_model::get_x_tilde() {
        return x_tilde;
    }
}