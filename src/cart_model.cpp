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

using namespace Eigen;

namespace dynamic_gap {
    cart_model::cart_model(std::string _side, int _index, double init_r, double init_beta, Matrix<double, 1, 3> v_ego) {
        side = _side;
        index = _index;
        initialize(init_r, init_beta, v_ego);
    }

    cart_model::cart_model(const dynamic_gap::cart_model &model) {
        x = model.x;
    }

    // copy constructor
    // copies what
    // state really only thing that matters
    // won't be using covariance matrix or anything

    cart_model::~cart_model() {}

    void cart_model::initialize(double init_r, double init_beta, Matrix<double, 1, 3> _v_ego) {
        // std::cout << "initializing with init_r: " << init_r << ", init_beta: " << init_beta << std::endl;
        // OBSERVATION MATRIX
        H << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0;

        // MEASUREMENT NOISE
        R << 0.04, 0.0,
             0.0, 0.04;

        // PROCESS NOISE
        Q << 0.01, 0.0, 0.0, 0.0,
             0.0, 0.01, 0.0, 0.0,
             0.0, 0.0, 0.15, 0.0,
             0.0, 0.0, 0.0, 0.15;

        double v_rel_x = -_v_ego[0];
        double v_rel_y = -_v_ego[1];
        x << init_r * std::cos(init_beta),
             init_r * std::sin(init_beta),
             0.0,
             0.0;

        P << 0.01, 0.0, 0.0, 0.0,
             0.0, 0.01, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;

        G << 1.0, 1.0,
             1.0, 1.0,
             1.0, 1.0,
             1.0, 1.0;

        t0 = ros::Time::now().toSec();
        t = ros::Time::now().toSec();
        dt = t - t0;
        accel << 0.0, 0.0, 0.0;
        v_ego << 0.0, 0.0, 0.0;

        A << 0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0;

        Ad << 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0;

        dQ << 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0;

        frozen_x << 0.0, 0.0, 0.0, 0.0;

        extended_origin_x << 0.0, 0.0, 0.0, 0.0;
        omega_rbt_prev = 0.0;
        linear_acc_ego << 0.0, 0.0;
        linear_vel_ego << 0.0, 0.0;
        ang_vel_ego = 0.0;
    }

    void cart_model::copy_model()) {
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
        new_x << 0.0, 0.0, 0.0, 0.0;

        double vdot_x_body = linear_acc_ego[0] + linear_vel_ego[1]*ang_vel_ego;
        double vdot_y_body = linear_acc_ego[1] - linear_vel_ego[0]*ang_vel_ego;

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
        if (!freeze) {
            frozen_linear_vel_ego << linear_vel_ego[0], linear_vel_ego[1];
            frozen_ang_vel_ego = ang_vel_ego;
        }

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
        Matrix<double, 4, 1> new_x;
        new_x << 0.0, 0.0, 0.0, 0.0;

        double vdot_x_body = linear_acc_ego[0] + linear_vel_ego[1]*ang_vel_ego;
        double vdot_y_body = linear_acc_ego[1] - linear_vel_ego[0]*ang_vel_ego;

        new_x[0] = x[0] + (x[2] + x[1]*ang_vel_ego)*dt; // r_x
        new_x[1] = x[1] + (x[3] - x[0]*ang_vel_ego)*dt; // r_y
        new_x[2] = x[2] + (x[3]*ang_vel_ego - vdot_x_body)*dt; // v_x
        new_x[3] = x[3] + (-x[2]*ang_vel_ego - vdot_y_body)*dt; // v_y
        x = new_x;
    }

    void cart_model::linearize() {
        A << 0.0, ang_vel_ego, 1.0, 0.0,
             -ang_vel_ego, 0.0, 0.0, 1.0,
             0.0, 0.0, 0.0, ang_vel_ego,
             0.0, 0.0, -ang_vel_ego, 0.0;

        Ad << 1.0, ang_vel_ego*dt, dt, 0.0,
              -ang_vel_ego*dt, 1.0, 0.0, dt,
              0.0, 0.0, 1.0, ang_vel_ego*dt,
              0.0, 0.0, -ang_vel_ego*dt, 1.0;
        // std::cout << "Ad in linearize: " << Ad << std::endl;
    }

    void cart_model::discretizeQ() {
        dQ = Q * dt;

        Matrix<double, 4, 4> M2 = 0.5 * dt * ((A * dQ).transpose() + A * dQ);
        Matrix<double, 4, 4> M3 = 0.3333 * dt * dt * (A * dQ).transpose();

        dQ = dQ + M2 + M3;
    }

    void cart_model::kf_update_loop(Matrix<double, 2, 1> range_bearing_measurement, Matrix<double, 1, 3> _a_ego, Matrix<double, 1, 3> _v_ego, std::string gap_type) {
        t = ros::Time::now().toSec();
        dt = t - t0;
        // acceleration comes in wrt robot frame
        linear_acc_ego << _a_ego[0], _a_ego[1];
        linear_vel_ego << _v_ego[0], _v_ego[1];
        ang_vel_ego = _v_ego[2];
        
        v_ego = _v_ego;
        //  Eigen::Vector4d cart_state = get_cartesian_state();
        //std::cout << "acceleration" << std::endl;
        //std::cout << "v_ego: " << v_ego[0] << ", " << v_ego[1] << ", " << v_ego[2] << std::endl;
        //std::cout << "a_ego: " << _a_ego[0] << ", " << _a_ego[1] << ", " << _a_ego[2] << std::endl;
        //std::cout<< "integrating" << std::endl;
        if (gap_type == "simplified") {
            std::cout << "linear ego vel: " << linear_vel_ego[0] << ", " << linear_vel_ego[1] << ", angular ego vel: " << ang_vel_ego << std::endl;
            std::cout << "linear ego acceleration: " << linear_acc_ego[0] << ", " << linear_acc_ego[1] << std::endl;
            std::cout << "x_i: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << std::endl;
        }
        integrate();
        // cart_state = get_cartesian_state();
        if (gap_type == "simplified") {
            std::cout << "x_i+1_prime: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << std::endl;
        }
        // cart_state = get_cartesian_state();
        //std::cout << "x_i bar: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;
        
        // std::cout << "x after integration" << x << std::endl;
        // std::cout<< "linearizing" << std::endl;
        linearize();
        // std::cout<< "discretizing Q" << std::endl;
        discretizeQ();

        //std::cout<< "estimating covariance matrix" << std::endl;
        //std::cout << "Ad: " << Ad << std::endl;
        //std::cout << "initial P: " << P << std::endl;
        Matrix<double, 4, 4> Ad_transpose = Ad.transpose();
        // std::cout << "Ad_transpose: " << Ad_transpose << std::endl;
        //std::cout << "dQ: " << dQ << std::endl;

        Matrix<double, 4, 4> new_P = Ad * P * Ad_transpose + dQ;
        // std::cout << "new_P: " << new_P << std::endl;
        P = new_P;
        // std::cout << "P: " << P << std::endl;

        //std::cout<< "updating Kalman gain" << std::endl;

        //std::cout << "H: " << H << std::endl;
        Matrix<double, 4, 2> H_transpose = H.transpose();
        // std::cout << "H_transpose: " << H_transpose << std::endl;
        //std::cout << "R: " << R << std::endl;
        tmp_mat = H*P*H_transpose + R;
        // std::cout << "tmp_mat: " << tmp_mat << std::endl;
        Matrix<double, 2, 2> inverted_tmp_mat = tmp_mat.inverse();
        // std::cout << "tmp_mat inverse: " << inverted_tmp_mat << std::endl;
        Matrix<double, 4, 2> P_H_prod = P * H_transpose;
        //std::cout << "P_H_prod: " << P_H_prod << std::endl;
        //std::cout << "inverted tmp mat: " << inverted_tmp_mat << std::endl;
        G = P_H_prod * inverted_tmp_mat;
        // std::cout << "G: " << G << std::endl;
        //std::cout << "error: " << y_tilde - H*y << std::endl;
        //std::cout<< "updating state" << std::endl;

        x_tilde << range_bearing_measurement[0]*std::cos(range_bearing_measurement[1]),
                   range_bearing_measurement[0]*std::sin(range_bearing_measurement[1]);
        if (gap_type == "simplified") {
            std::cout << "x_tilde: " << x_tilde[0] << ", " << x_tilde[1] << std::endl;
        }

        // std::cout << "P: " << P << std::endl;
        Matrix<double, 4, 1> x_update_mat = G*(x_tilde - H*x);
        // std::cout << "actual update to x: " << x_update_mat << std::endl;
        x = x + x_update_mat;

        // cart_state = get_cartesian_state();
        if (gap_type == "simplified") {
            std::cout << "x_i+1: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << std::endl;
            std::cout << "-----------" << std::endl;
        }
        //std::cout<< "updating covariance matrix" << std::endl;
        P = (MatrixXd::Identity(4,4) - G*H)*P;
        // std::cout << "P after update: " << P << std::endl;
        t0 = t;
        omega_rbt_prev = _v_ego[2];
    }

    Eigen::Vector4d cart_model::get_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        return x;
    }

    Eigen::Vector4d cart_model::get_frozen_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        return frozen_x;
    }

    Eigen::Vector4d cart_model::get_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4d mp_state;
        mp_state << 1.0 / sqrt(pow(x[0], 2) + pow(x[1], 2)),
                    std::atan2(x[1], x[0]),
                    (x[0]*x[2] + x[1]*x[3]) / (pow(x[0], 2) + pow(x[1], 2)),
                    (x[0]*x[3] - x[1]*x[2]) / (pow(x[0], 2) + pow(x[1], 2));
        return mp_state;
    }

    Eigen::Vector4d cart_model::get_frozen_modified_polar_state() {
        // y state:
        // [1/r, beta, rdot/r, betadot]
        Eigen::Vector4d frozen_mp_state;
        frozen_mp_state << 1.0 / sqrt(pow(frozen_x[0], 2) + pow(frozen_x[1], 2)),
                           std::atan2(frozen_x[1], frozen_x[0]),
                           (frozen_x[0]*frozen_x[2] + frozen_x[1]*frozen_x[3]) / (pow(frozen_x[0], 2) + pow(frozen_x[1], 2)),
                           (frozen_x[0]*frozen_x[3] - frozen_x[1]*frozen_x[2]) / (pow(frozen_x[0], 2) + pow(frozen_x[1], 2));
        return frozen_mp_state;
    }

    Matrix<double, 4, 1> cart_model::get_state() {
        return x;
    }

    Matrix<double, 4, 1> cart_model::get_copy_state() {
        return copied_x;
    }

    Matrix<double, 4, 1> cart_model::get_frozen_state() {
        return frozen_x;
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

}