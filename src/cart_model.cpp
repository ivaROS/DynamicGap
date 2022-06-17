#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <turtlesim/Spawn.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <dynamic_gap/cart_model.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <limits>
#include <sstream>
#include "/home/masselmeier3/Desktop/Research/vcpkg/installed/x64-linux/include/matplotlibcpp.h"

using namespace Eigen;
namespace plt = matplotlibcpp;

namespace dynamic_gap {
    cart_model::cart_model(std::string _side, int _index, double init_r, double init_beta, Matrix<double, 1, 3> v_ego) {
        side = _side;
        index = _index;
        initialize(init_r, init_beta, v_ego);
    }

    //cart_model::cart_model(const dynamic_gap::cart_model &model) {
    //    x = model.x;
    // }

    void cart_model::initialize(double init_r, double init_beta, Matrix<double, 1, 3> _v_ego) {
        // std::cout << "initializing with init_r: " << init_r << ", init_beta: " << init_beta << std::endl;
        // OBSERVATION MATRIX
        H << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0;
        H_transpose = H.transpose();
        // MEASUREMENT NOISE
        // Bigger R: better performance with static things (weighting robot's motion more)
        // I think 0.1 is roughly the minimum we can do. Otherwise, measurements get really noisy
        R << 0.10, 0.0,
             0.0, 0.10;

        // PROCESS NOISE
        // Bigger Q: Better with dynamic things (weighting measurements more)

        Q << 0.075, 0.0, 0.0, 0.0,
             0.0, 0.075, 0.0, 0.0,
             0.0, 0.0, 0.15, 0.0,
             0.0, 0.0, 0.0, 0.15;
        
        // Q *= 0.001;
        dQ = Q;

        // covariance/uncertainty of state variables (r_x, r_y, v_x, v_y)
        // larger P_0 helps with GT values that are non-zero
        // larger P_0 gives more weight to measurements (behaves like Q)
        // Could initialize off-diagonal terms, not sure if helps
        P << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 2.5, 0.0,
             0.0, 0.0, 0.0, 2.5;

        double v_rel_x = -_v_ego[0];
        double v_rel_y = -_v_ego[1];
        std::vector<double> measurement{init_r * std::cos(init_beta), 
                                        init_r * std::sin(init_beta), 
                                        v_rel_x, 
                                        v_rel_y};
        
        x << measurement[0], measurement[1], 0.0, 0.0;

        G << 1.0, 1.0,
             1.0, 1.0,
             1.0, 1.0,
             1.0, 1.0;

        t0 = ros::Time::now().toSec();
        t = ros::Time::now().toSec();
        dt = t - t0;
        v_ego << 0.0, 0.0, 0.0;
        a_ego << 0.0, 0.0, 0.0;

        A << 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0;
        Ad = A;
        Ad_transpose = A;

        frozen_x << 0.0, 0.0, 0.0, 0.0;

        extended_origin_x << 0.0, 0.0, 0.0, 0.0;
        initialized = true;
        life_time = 0.0;
        life_time_threshold = 3.0;
        eyes_state = MatrixXd::Identity(4,4);
        new_P = eyes_state;
        inverted_tmp_mat << 0.0, 0.0, 0.0, 0.0;
        x_update << 0.0, 0.0, 0.0, 0.0;
        std::vector<double> state{life_time, x[0], x[1], x[2], x[3]};
        std::vector<double> ego_accels{a_ego[0], a_ego[1], a_ego[2]};
        std::vector<double> ego_vels{v_ego[0], v_ego[1], v_ego[2]};
        previous_states.push_back(state);
        previous_measurements.push_back(measurement);
        previous_ego_accels.push_back(ego_accels);
        previous_ego_vels.push_back(ego_vels);
        plot_dir = "/home/masselmeier3/Desktop/Research/cart_model_plots/";   
    }

    void cart_model::copy_model() {
        // std::cout << "in freeze_robot_vel" << std::endl;
        Eigen::Vector4d cartesian_state = get_cartesian_state();
        // std::cout << "original cartesian state: " << cartesian_state[0] << ", " << cartesian_state[1] << ", " << cartesian_state[2] << ", " << cartesian_state[3] << std::endl;
        //std::cout << "original MP state. r: " << y[0] << ", beta: " << std::atan2(y[1], y[2]) << ", rdot/r: " << y[3] << ", betadot: " << y[4] << std::endl;
        // std::cout << "v_ego: " << v_ego[0] << ", " << v_ego[1] << std::endl;
        
        copied_x = cartesian_state; // << cartesian_state[0], cartesian_state[1], cartesian_state[2], cartesian_state[3];
        //std::cout << "modified cartesian state: " << frozen_x[0] << ", " << frozen_x[1] << ", " << frozen_x[2] << ", " << frozen_x[3] << std::endl;
    }

    void cart_model::copy_model_propagate(double dt) {
        Matrix<double, 4, 1> new_copied_x;
        new_copied_x << 0.0, 0.0, 0.0, 0.0;

        double ang_vel_ego = v_ego[2];
        double vdot_x_body = a_ego[0] + v_ego[1]*ang_vel_ego;
        double vdot_y_body = a_ego[1] - v_ego[0]*ang_vel_ego;

        new_copied_x[0] = copied_x[0] + (copied_x[2] + copied_x[1]*ang_vel_ego)*dt; // r_x
        new_copied_x[1] = copied_x[1] + (copied_x[3] - copied_x[0]*ang_vel_ego)*dt; // r_y
        new_copied_x[2] = copied_x[2] + (copied_x[3]*ang_vel_ego - vdot_x_body)*dt; // v_x
        new_copied_x[3] = copied_x[3] + (-copied_x[2]*ang_vel_ego - vdot_y_body)*dt; // v_y
        copied_x = new_copied_x;
    }

    void cart_model::freeze_robot_vel() {
        // std::cout << "in freeze_robot_vel" << std::endl;
        Eigen::Vector4d cartesian_state = get_cartesian_state();
        // std::cout << "original cartesian state: " << cartesian_state[0] << ", " << cartesian_state[1] << ", " << cartesian_state[2] << ", " << cartesian_state[3] << std::endl;
        //std::cout << "original MP state. r: " << y[0] << ", beta: " << std::atan2(y[1], y[2]) << ", rdot/r: " << y[3] << ", betadot: " << y[4] << std::endl;
        // std::cout << "v_ego: " << v_ego[0] << ", " << v_ego[1] << std::endl;
        
        // update cartesian
        cartesian_state[2] += v_ego[0];
        cartesian_state[3] += v_ego[1];
        frozen_x = cartesian_state; // << cartesian_state[0], cartesian_state[1], cartesian_state[2], cartesian_state[3];
        //std::cout << "modified cartesian state: " << frozen_x[0] << ", " << frozen_x[1] << ", " << frozen_x[2] << ", " << frozen_x[3] << std::endl;
    }

    void cart_model::frozen_state_propagate(double dt) {
        Matrix<double, 4, 1> new_frozen_x;     
        new_frozen_x << 0.0, 0.0, 0.0, 0.0;

        Eigen::Vector2d frozen_linear_acc_ego(0.0, 0.0);

        Eigen::Vector2d frozen_linear_vel_ego(0.0, 0.0); 
        double frozen_ang_vel_ego = 0.0;

        double vdot_x_body = frozen_linear_acc_ego[0] + frozen_linear_vel_ego[1]*frozen_ang_vel_ego;
        double vdot_y_body = frozen_linear_acc_ego[1] - frozen_linear_vel_ego[0]*frozen_ang_vel_ego;

        // std::cout << "frozen_y stepping from 1/r: " << frozen_y[0] << ", beta: " << std::atan2(frozen_y[1], frozen_y[2]) << ", rdot/r: " << frozen_y[3] << ", betadot: " << frozen_y[4] << " to ";
        // discrete euler update of state (ignoring rbt acceleration, set as 0)
        new_frozen_x[0] = frozen_x[0] + (frozen_x[2] + frozen_x[1]*frozen_ang_vel_ego)*dt;
        new_frozen_x[1] = frozen_x[1] + (frozen_x[3] - frozen_x[0]*frozen_ang_vel_ego)*dt;
        new_frozen_x[2] = frozen_x[2] + (frozen_x[3]*frozen_ang_vel_ego - vdot_x_body)*dt;
        new_frozen_x[3] = frozen_x[3] + (-frozen_x[2]*frozen_ang_vel_ego - vdot_y_body)*dt;
        frozen_x = new_frozen_x; // is this ok? do we need a deep copy?
        // std::cout << " 1/r: " << frozen_y[0] << ", beta: " << std::atan2(frozen_y[1], frozen_y[2]) << ", rdot/r: " << frozen_y[3] << ", betadot: " << frozen_y[4] << std::endl;
    }
    

    void cart_model::integrate() {
        double ang_vel_ego = (v_ego[2] + (v_ego[2] - a_ego[2]*dt)) / 2.0;
        double vdot_x_body = a_ego[0] + v_ego[1]*ang_vel_ego;
        double vdot_y_body = a_ego[1] - v_ego[0]*ang_vel_ego;

        double p_dot_x = (x[2] + x[1]*ang_vel_ego);
        double p_dot_y = (x[3] - x[0]*ang_vel_ego);
        double v_dot_x = (x[3]*ang_vel_ego - vdot_x_body);
        double v_dot_y = (-x[2]*ang_vel_ego - vdot_y_body);

        Matrix<double, 4, 1> new_x;
        new_x << x[0] + p_dot_x*dt + v_dot_x*dt*dt, // r_x
                 x[1] + p_dot_y*dt + v_dot_y*dt*dt, // r_y
                 x[2] + v_dot_x*dt, // v_x
                 x[3] + v_dot_y*dt; // v_y
        
        x = new_x;
    }

    void cart_model::linearize() {
        double ang_vel_ego = (v_ego[2] + (v_ego[2] - a_ego[2]*dt)) / 2.0;
        
        A << 0.0, ang_vel_ego, 1.0, ang_vel_ego*dt,
             -ang_vel_ego, 0.0, -ang_vel_ego*dt, 1.0,
             0.0, 0.0, 0.0, ang_vel_ego,
             0.0, 0.0, -ang_vel_ego, 0.0;

        Ad << 1.0, ang_vel_ego*dt, dt, 0.5 * ang_vel_ego * dt * dt,
              -ang_vel_ego*dt, 1.0, -0.5 * ang_vel_ego * dt * dt, dt,
              0.0, 0.0, 1.0, ang_vel_ego*dt,
              0.0, 0.0, -ang_vel_ego*dt, 1.0;
        /*
        A << 0.0, ang_vel_ego, 1.0, 0.0,
             -ang_vel_ego, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0, ang_vel_ego,
             0.0, 0.0, -ang_vel_ego, 0.0;

        Ad << 1.0, ang_vel_ego*dt, dt, 0.0,
              -ang_vel_ego*dt, 1.0, 0.0, dt,
              0.0, 0.0, 1.0, ang_vel_ego*dt,
              0.0, 0.0, -ang_vel_ego*dt, 1.0;
        */
        //Ad_transpose << 1.0, -ang_vel_ego*dt, 0.0, 0.0,
        //                ang_vel_ego*dt, 1.0, 0.0, 0.0,
        //                dt, 0.0, 1.0, -ang_vel_ego*dt,
        //                0.0, dt, ang_vel_ego*dt, 1.0;
        // std::cout << "Ad in linearize: " << Ad << std::endl;
    }

    // this does give off-diagonal terms to Q, so init diagonal Q is fine
    void cart_model::discretizeQ() {
        dQ = Q * dt;

        Matrix<double, 4, 4> A_dQ_transpose = (A * dQ).transpose();
        Matrix<double, 4, 4> M2 = 0.5 * dt * (A_dQ_transpose + A * dQ);
        Matrix<double, 4, 4> M3 = 0.3333 * dt * dt * A_dQ_transpose;

        dQ = dQ + M2 + M3;
        //ROS_INFO_STREAM("dQ+1: " << dQ(0, 0) << ", " << dQ(0, 1) << ", " << dQ(0, 2) << ", " << dQ(0, 3));
        //    ROS_INFO_STREAM(dQ(1, 0) << ", " << dQ(1, 1) << ", " << dQ(1, 2) << ", " << dQ(1, 3));
        //    ROS_INFO_STREAM(dQ(2, 0) << ", " << dQ(2, 1) << ", " << dQ(2, 2) << ", " << dQ(2, 3));
        //    ROS_INFO_STREAM(dQ(3, 0) << ", " << dQ(3, 1) << ", " << dQ(3, 2) << ", " << dQ(3, 3));            
        //    ROS_INFO_STREAM("-----------");
    }

    void cart_model::kf_update_loop(Matrix<double, 2, 1> range_bearing_measurement, 
                                    Matrix<double, 1, 3> _a_ego, Matrix<double, 1, 3> _v_ego, 
                                    bool print, bool _bridge_model,
                                    std::vector<geometry_msgs::Pose> _agent_odoms,
                                    std::vector<geometry_msgs::Vector3Stamped> _agent_vels) {
        bridge_model = _bridge_model;
        agent_odoms = _agent_odoms;
        agent_vels = _agent_vels;
                
        t = ros::Time::now().toSec();
        dt = t - t0;
        life_time += dt;
        //std::cout << "model lifetime: " << life_time << std::endl;
        // acceleration comes in wrt robot frame
        a_ego = _a_ego;
        v_ego = _v_ego;
        //  Eigen::Vector4d cart_state = get_cartesian_state();
        //std::cout << "acceleration" << std::endl;
        //std::cout<< "integrating" << std::endl;
        //if (get_initialized()) {
        //    set_initialized(false);
        //    return;
        //} 
        
        if (print) {
            ROS_INFO_STREAM("update for model " << get_index());
            ROS_INFO_STREAM("x_i: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);
            ROS_INFO_STREAM("linear ego vel: " << v_ego[0] << ", " << v_ego[1] << ", angular ego vel: " << v_ego[2]);
            ROS_INFO_STREAM("linear ego acceleration: " << a_ego[0] << ", " << a_ego[1] << ", angular ego acc: " << a_ego[2]);
        }
        //check_time1 = ros::Time::now().toSec(); 
        integrate();
        //std::cout << "integrate time elapsed: " << ros::Time::now().toSec() - check_time1 << std::endl;
        // cart_state = get_cartesian_state();
        if (print) {
            ROS_INFO_STREAM("x_i+1_prime: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);
        }
        // cart_state = get_cartesian_state();
        //std::cout << "x_i bar: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;
        
        // std::cout << "x after integration" << x << std::endl;
        // std::cout<< "linearizing" << std::endl;
        //check_time1 = ros::Time::now().toSec(); 
        linearize();
        //std::cout << "linearize time elapsed: " << ros::Time::now().toSec() - check_time1 << std::endl;
        
        // std::cout<< "discretizing Q" << std::endl;
        //check_time1 = ros::Time::now().toSec(); 
        discretizeQ();
        //std::cout << "discretizeQ time elapsed: " << ros::Time::now().toSec() - check_time1 << std::endl;

        //std::cout<< "estimating covariance matrix" << std::endl;
        //std::cout << "Ad: " << Ad << std::endl;
        //std::cout << "initial P: " << P << std::endl;
        //Matrix<double, 4, 4> Ad_transpose = ;
        // std::cout << "Ad_transpose: " << Ad_transpose << std::endl;
        //std::cout << "dQ: " << dQ << std::endl;

        //check_time1 = ros::Time::now().toSec(); 
        new_P = Ad * P * Ad.transpose() + dQ;
        //std::cout << "Ad transpose time elapsed: " << ros::Time::now().toSec() - check_time1 << std::endl;

        // std::cout << "new_P: " << new_P << std::endl;
        P = new_P;
        // std::cout << "P: " << P << std::endl;

        //std::cout<< "updating Kalman gain" << std::endl;

        //std::cout << "H: " << H << std::endl;
        // std::cout << "H_transpose: " << H_transpose << std::endl;
        //std::cout << "R: " << R << std::endl;
        tmp_mat = H*P*H_transpose + R; //  * (10.0 - 9.0*std::exp(-life_time))
        // std::cout << "tmp_mat: " << tmp_mat << std::endl;
        //double det = 1.0 / (tmp_mat.coeff(0,0)*tmp_mat.coeff(1,1) - tmp_mat.coeff(0,1)*tmp_mat.coeff(1, 0));
        //
        //inverted_tmp_mat << det*tmp_mat.coeff(1,1), det*-tmp_mat.coeff(0,1),
        //                    det*-tmp_mat.coeff(1, 0), det*tmp_mat.coeff(0,0);

        //check_time1 = ros::Time::now().toSec(); 
        inverted_tmp_mat = tmp_mat.inverse();
        //std::cout << "Inversion time elapsed: " << ros::Time::now().toSec() - check_time1 << std::endl;
         
        // std::cout << "tmp_mat inverse: " << inverted_tmp_mat << std::endl;
        //Matrix<double, 4, 2> P_H_prod = ;
        //std::cout << "P_H_prod: " << P_H_prod << std::endl;
        //std::cout << "inverted tmp mat: " << inverted_tmp_mat << std::endl;
        G = P * H_transpose * inverted_tmp_mat;
        // std::cout << "G: " << G << std::endl;
        //std::cout << "error: " << y_tilde - H*y << std::endl;
        //std::cout<< "updating state" << std::endl;

        x_tilde << range_bearing_measurement[0]*std::cos(range_bearing_measurement[1]),
                   range_bearing_measurement[0]*std::sin(range_bearing_measurement[1]);
        if (print) {
            ROS_INFO_STREAM("x_tilde: " << x_tilde[0] << ", " << x_tilde[1]);
        }

        // std::cout << "P: " << P << std::endl;
        x_update = G*(x_tilde - H*x);
        // std::cout << "actual update to x: " << x_update_mat << std::endl;
        x += x_update;

        // cart_state = get_cartesian_state();
        if (print) {
            ROS_INFO_STREAM("x_i+1: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3]);
            //ROS_INFO_STREAM("P_i+1: " << P(0, 0) << ", " << P(0, 1) << ", " << P(0, 2) << ", " << P(0, 3));
            //ROS_INFO_STREAM(P(1, 0) << ", " << P(1, 1) << ", " << P(1, 2) << ", " << P(1, 3));
            //ROS_INFO_STREAM(P(2, 0) << ", " << P(2, 1) << ", " << P(2, 2) << ", " << P(2, 3));
            //ROS_INFO_STREAM(P(3, 0) << ", " << P(3, 1) << ", " << P(3, 2) << ", " << P(3, 3));            
            ROS_INFO_STREAM("-----------");
        }
        //std::cout<< "updating covariance matrix" << std::endl;
        P = (eyes_state - G*H)*P;
        // std::cout << "P after update: " << P << std::endl;
        t0 = t;
        
        /*
        if (life_time <= 15.0 && !plotted) {
            std::vector<double> state{life_time, x[0], x[1], x[2], x[3]};
            std::vector<double> ground_truths{x_tilde[0], x_tilde[1], 0.0, 0.0};

            double robot0_odom_dist = sqrt(pow(robot0_odom.position.x - x[0], 2) + pow(robot0_odom.position.y - x[1], 2));
            double robot1_odom_dist = sqrt(pow(robot1_odom.position.x - x[0], 2) + pow(robot1_odom.position.y - x[1], 2));
        
            //ROS_INFO_STREAM("distance: " << robot0_odom_dist);

            if (robot0_odom_dist < 0.3) {
                ground_truths[2] = robot0_vel.vector.x - v_ego[0];
                ground_truths[3] = robot0_vel.vector.y - v_ego[1];
            } else if (robot1_odom_dist < 0.3) {
                ground_truths[2] = robot1_vel.vector.x - v_ego[0];
                ground_truths[3] = robot1_vel.vector.y - v_ego[1];  
            }
                          
            std::vector<double> ego_vels{v_ego[0], v_ego[1], v_ego[2]};
            std::vector<double> ego_accels{a_ego[0], a_ego[1], a_ego[2]};
        
            previous_states.push_back(state);
            previous_measurements.push_back(ground_truths);
            previous_ego_accels.push_back(ego_accels);
            previous_ego_vels.push_back(ego_vels);            
        }

        if (life_time > 15.0 && !plotted) {
            plot_states();
        }
        */
    }

    void cart_model::plot_states() {
        //std::cout << "in plot states" << std::endl;
        int n = previous_states.size();
        std::vector<double> t(n), r_xs(n), r_ys(n), v_xs(n), v_ys(n), 
                            r_xs_GT(n), r_ys_GT(n), v_xs_GT(n), v_ys_GT(n),
                            v_ego_angs(n), a_ego_angs(n);
        for(int i=0; i < previous_states.size(); i++) {
            t.at(i) = previous_states[i][0];
            r_xs.at(i) = previous_states[i][1];
            r_ys.at(i) = previous_states[i][2];
            v_xs.at(i) = previous_states[i][3];
            v_ys.at(i) = previous_states[i][4];
            r_xs_GT.at(i) = previous_measurements[i][0];
            r_ys_GT.at(i) = previous_measurements[i][1];
            v_xs_GT.at(i) = previous_measurements[i][2];
            v_ys_GT.at(i) = previous_measurements[i][3];
            v_ego_angs.at(i) = previous_ego_vels[i][2];
            a_ego_angs.at(i) = previous_ego_accels[i][2];
        }

        //std::cout << "position plot" << std::endl;
        // Set the size of output image to 1200x780 pixels
        //plt::subplot(1, 2, 1);
        plt::figure_size(1200, 780);
        // Plot line from given x and y data. Color is selected automatically.
        plt::scatter(t, r_xs_GT, 25.0, {{"label", "r_x (GT)"}});
        plt::scatter(t, r_ys_GT, 25.0, {{"label", "r_y (GT)"}});
        plt::scatter(t, r_xs, 25.0, {{"label", "r_x"}});
        plt::scatter(t, r_ys, 25.0, {{"label", "r_y"}});
        plt::xlim(0, 15);
        plt::legend();
        plt::save(plot_dir + std::to_string(index) + "_positions.png");
        plt::close();

        //std::cout << "velocity plot" << std::endl;
        plt::figure_size(1200, 780);
        plt::scatter(t, v_xs_GT, 25.0, {{"label", "v_x (GT)"}});
        plt::scatter(t, v_ys_GT, 25.0, {{"label", "v_y (GT)"}});
        plt::scatter(t, v_xs, 25.0, {{"label", "v_x"}});
        plt::scatter(t, v_ys, 25.0, {{"label", "v_y"}});
        //plt::scatter(t, v_ego_angs, 25.0, {{"label", "v_ego_ang"}});
        //plt::scatter(t, a_ego_angs, 25.0, {{"label", "a_ego_ang"}});
        plt::xlim(0, 15);
        plt::legend();
        plt::save(plot_dir + std::to_string(index) + "_velocities.png");
        plt::close();
        plotted = true;
    }
    

    Eigen::Vector4d cart_model::get_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d return_x = x;

        // ROS_INFO_STREAM("robot0_odom: " << robot0_odom.position.x << ", " << robot0_odom.position.y);
        //ROS_INFO_STREAM("x,y: " << x[0] << ", " << x[1]);
        
        // double robot0_odom_dist = sqrt(pow(robot0_odom.position.x - x[0], 2) + pow(robot0_odom.position.y - x[1], 2));
        // double robot1_odom_dist = sqrt(pow(robot1_odom.position.x - x[0], 2) + pow(robot1_odom.position.y - x[1], 2));
        
        //ROS_INFO_STREAM("distance: " << robot0_odom_dist);
        
        double robot_i_odom_dist;
        double min_dist = 100;
        int min_idx;
        for (int i = 0; i < agent_odoms.size(); i++) {
            robot_i_odom_dist = sqrt(pow(agent_odoms[i].position.x - x[0], 2) + pow(agent_odoms[i].position.y - x[1], 2));
            if (robot_i_odom_dist < min_dist) {
                min_dist = robot_i_odom_dist;
                min_idx = i;
            }
        }
        
        if (min_dist < 0.4) {
            // ROS_INFO_STREAM("fixing velocity");
            return_x[2] = agent_vels[min_idx].vector.x - v_ego[0];
            return_x[3] = agent_vels[min_idx].vector.y - v_ego[1];
        } else {
            return_x[2] = -v_ego[0];
            return_x[3] = -v_ego[1];
        }

        /*
        if (bridge_model) {
            return_x[2] = -v_ego[0];
            return_x[3] = -v_ego[1];
        }
        */

        return return_x;
    }

    Eigen::Vector4d cart_model::get_frozen_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d return_x;
        if (life_time > life_time_threshold) {
            return_x = frozen_x;
        } else {
            return_x << x[0], x[1], 0.0, 0.0;
        }
        return frozen_x;
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

    Matrix<double, 3, 1> cart_model::get_v_ego() {
        return v_ego;
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

    void cart_model::set_initialized(bool _initialized) {
        initialized = _initialized;
    }

    bool cart_model::get_initialized() {
        return initialized;
    }
}