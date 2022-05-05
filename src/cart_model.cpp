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
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0;

        // MEASUREMENT NOISE
        R << 0.00001, 0.0,
             0.0, 0.00001;

        // PROCESS NOISE
        Q << 0.000001, 0.0, 0.0, 0.0,
            0.0, 0.000001, 0.0, 0.0,
            0.0, 0.0, 0.000001, 0.0,
            0.0, 0.0, 0.0, 0.000001;

        double v_rel_x = -_v_ego[0];
        double v_rel_y = -_v_ego[1];
        x << init_r * std::cos(init_beta),
             init_r * std::sin(init_beta);

        P << 10.0e-4, 0.0, 0.0, 0.0,
                0.0, 10.0e-4, 0.0, 0.0,
                0.0, 0.0, 10.0e-4, 0.0,
                0.0, 0.0, 0.0, 10.0e-2;
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
    }

    void cart_model::freeze_robot_vel() {
        std::cout << "in freeze_robot_vel" << std::endl;
        Eigen::Vector4d cartesian_state = get_cartesian_state();
        std::cout << "original cartesian state: " << cartesian_state[0] << ", " << cartesian_state[1] << ", " << cartesian_state[2] << ", " << cartesian_state[3] << std::endl;
        //std::cout << "original MP state. r: " << y[0] << ", beta: " << std::atan2(y[1], y[2]) << ", rdot/r: " << y[3] << ", betadot: " << y[4] << std::endl;
        std::cout << "v_ego: " << v_ego[0] << ", " << v_ego[1] << std::endl;
        
        // update cartesian
        cartesian_state[2] += v_ego[0];
        cartesian_state[3] += v_ego[1];
        
        frozen_x = cartesian_state; // << cartesian_state[0], cartesian_state[1], cartesian_state[2], cartesian_state[3];
        //std::cout << "modified cartesian state: " << frozen_x[0] << ", " << frozen_x[1] << ", " << frozen_x[2] << ", " << frozen_x[3] << std::endl;
    }

    void cart_model::frozen_state_propagate(double dt) {
        Matrix<double, 4, 1> new_frozen_x;     
        new_frozen_x << 0.0, 0.0, 0.0, 0.0;
        // std::cout << "frozen_y stepping from 1/r: " << frozen_y[0] << ", beta: " << std::atan2(frozen_y[1], frozen_y[2]) << ", rdot/r: " << frozen_y[3] << ", betadot: " << frozen_y[4] << " to ";
        // discrete euler update of state (ignoring rbt acceleration, set as 0)
        new_frozen_x[0] = frozen_x[0] + (frozen_x[2])*dt;
        new_frozen_x[1] = frozen_x[1] + (frozen_x[3])*dt;
        new_frozen_x[2] = frozen_x[2];
        new_frozen_x[3] = frozen_x[3];
        frozen_x = new_frozen_x; // is this ok? do we need a deep copy?
        // std::cout << " 1/r: " << frozen_y[0] << ", beta: " << std::atan2(frozen_y[1], frozen_y[2]) << ", rdot/r: " << frozen_y[3] << ", betadot: " << frozen_y[4] << std::endl;
    }
    

    void cart_model::integrate() {
        t = ros::Time::now().toSec();
        dt = t - t0; // 0.01
        //std::cout << "t0: " << t0 << ", t: " << t << std::endl;
        // std::cout << "a: " << a[0] << ", " << a[1] << ", dt: " << dt << std::endl;
        Matrix<double, 4, 1> new_x;
        new_x << 0.0, 0.0, 0.0, 0.0;

        new_x[0] = x[0] + (x[2])*dt + 0.5*accel[0]*dt*dt; // r_x
        new_x[1] = x[1] + (x[3])*dt + 0.5*accel[1]*dt*dt; // r_y
        new_x[2] = x[2] + accel[0]*dt; // v_x
        new_x[3] = x[3] + accel[1]*dt; // v_y
        x = new_x;
    }

    void cart_model::linearize() {
        A << 0.0, 0.0, 1.0, 0.0;
             0.0, 0.0, 0.0, 1.0;
             0.0, 0.0, 0.0, 0.0;
             0.0, 0.0, 0.0, 0.0;

        Ad << 1.0, 0.0, dt, 0.0;
             0.0, 1.0, 0.0, dt;
             0.0, 0.0, 1.0, 0.0;
             0.0, 0.0, 0.0, 1.0;
        // std::cout << "Ad in linearize: " << Ad << std::endl;
    }

    void cart_model::discretizeQ() {
        dQ = Q * dt;

        Matrix<double, 4, 4> M2 = 0.5 * dt * ((A * dQ).transpose() + A * dQ);
        Matrix<double, 4, 4> M3 = 0.3333 * dt * dt * (A * dQ).transpose();

        dQ = dQ + M2 + M3;
    }

    void cart_model::kf_update_loop(Matrix<double, 2, 1> x_tilde, Matrix<double, 1, 3> _a_ego, Matrix<double, 1, 3> _v_ego) {
        // acceleration comes in wrt robot frame
        accel = -1 * _a_ego; // negative because a = a_target - a_ego, but we assume a_target = 0
        v_ego = _v_ego;
        Eigen::Vector4d cart_state = get_cartesian_state();
        std::cout << "x_i: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;
        //std::cout << "acceleration" << std::endl;
        std::cout << "v_ego: " << v_ego[0] << ", " << v_ego[1] << ", " << v_ego[2] << std::endl;
        std::cout << "a_ego: " << _a_ego[0] << ", " << _a_ego[1] << ", " << _a_ego[2] << std::endl;
        //std::cout<< "integrating" << std::endl;
        integrate();
        cart_state = get_cartesian_state();
        std::cout << "x_i+1_prime: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;
        // cart_state = get_cartesian_state();
        //std::cout << "x_i bar: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;
        std::cout << "x_tilde: " << x_tilde[0] << ", " << x_tilde[1] << std::endl;

        //std::cout << "y after integration" << y << std::endl;
        //std::cout<< "linearizing" << std::endl;
        linearize();
        // std::cout<< "discretizing Q" << std::endl;
        discretizeQ();

        //std::cout<< "estimating covariance matrix" << std::endl;
        //std::cout << "Ad: " << Ad << std::endl;
        //std::cout << "initial P: " << P << std::endl;
        Matrix<double, 4, 4> Ad_transpose = Ad.transpose();
        //std::cout << "Ad_transpose: " << Ad_transpose << std::endl;
        //std::cout << "dQ: " << dQ << std::endl;

        Matrix<double, 4, 4> new_P = Ad * P * Ad_transpose + dQ;
        //std::cout << "new_P: " << new_P << std::endl;
        P = new_P;
        // std::cout << "P: " << P << std::endl;

        //std::cout<< "updating Kalman gain" << std::endl;

        //std::cout << "H: " << H << std::endl;
        Matrix<double, 4, 2> H_transpose = H.transpose();
        //std::cout << "H_transpose: " << H_transpose << std::endl;
        //std::cout << "R: " << R << std::endl;
        tmp_mat = H*P*H_transpose + R;
        //std::cout << "tmp_mat: " << tmp_mat << std::endl;
        Matrix<double, 2, 2> inverted_tmp_mat = tmp_mat.inverse();
        //std::cout << "tmp_mat inverse: " << inverted_tmp_mat << std::endl;
        Matrix<double, 4, 2> P_H_prod = P * H_transpose;
        //std::cout << "P_H_prod: " << P_H_prod << std::endl;
        //std::cout << "inverted tmp mat: " << inverted_tmp_mat << std::endl;
        G = P_H_prod * inverted_tmp_mat;
        //std::cout << "G: " << G << std::endl;
        //std::cout << "error: " << y_tilde - H*y << std::endl;
        //std::cout<< "updating state" << std::endl;
        Matrix<double, 4, 1> x_update_mat = G*(x_tilde - H*x);
        //std::cout << "actual update to y: " << y_update_mat << std::endl;
        x = x + x_update_mat;

        cart_state = get_cartesian_state();
        std::cout << "x_i+1: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;

        //std::cout<< "updating covariance matrix" << std::endl;
        P = (MatrixXd::Identity(4,4) - G*H)*P;
        // std::cout << "P after update: " << P << std::endl;
        t0 = t;
        //std::cout << "" << std::endl;
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

    Matrix<double, 4, 1> cart_model::get_state() {
        return x;
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