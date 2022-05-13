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
#include <dynamic_gap/mp_model.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <limits>
#include <sstream>

using namespace Eigen;

namespace dynamic_gap {
    MP_model::MP_model(std::string _side, int _index, double init_r, double init_beta, Matrix<double, 1, 3> v_ego) {
        side = _side;
        index = _index;
        initialize(init_r, init_beta, v_ego);
    }

    MP_model::MP_model(const dynamic_gap::MP_model &model) {
        y = model.y;
        x = get_cartesian_state();
    }

    // copy constructor
    // copies what
    // state really only thing that matters
    // won't be using covariance matrix or anything

    MP_model::~MP_model() {}

    void MP_model::initialize(double init_r, double init_beta, Matrix<double, 1, 3> _v_ego) {
        // std::cout << "initializing with init_r: " << init_r << ", init_beta: " << init_beta << std::endl;
        // OBSERVATION MATRIX
        H << 1.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, 0.0;
        // MEASUREMENT NOISE
        R << 0.00001, 0.0, 0.0,
             0.0, 0.00001, 0.0,
             0.0, 0.0, 0.00001;
        // PROCESS NOISE
        
        Q << 0.005, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.005, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.005, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.05, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.05;
        

        double v_rel_x = -_v_ego[0];
        double v_rel_y = -_v_ego[1];
        y << 1.0 / init_r, 
                std::sin(init_beta), 
                std::cos(init_beta), 
                (init_r*std::cos(init_beta)*v_rel_x + init_r*std::sin(init_beta)*v_rel_y)/ pow(init_r, 2), 
                (init_r*std::cos(init_beta)*v_rel_y - init_r*std::sin(init_beta)*v_rel_x)/ pow(init_r, 2);
        x = get_cartesian_state();
        P << 10.0e-4, 0.0, 0.0, 0.0, 0.0,
                0.0, 10.0e-4, 0.0, 0.0, 0.0,
                0.0, 0.0, 10.0e-4, 0.0, 0.0,
                0.0, 0.0, 0.0, 10.0e-2, 0.0,
                0.0, 0.0, 0.0, 0.0, 10.0e-2;
        G << 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0;

        t0 = ros::Time::now().toSec();
        t = ros::Time::now().toSec();
        dt = t - t0;
        accel << 0.0, 0.0, 0.0;
        v_ego << 0.0, 0.0, 0.0;

        A << 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0;
        Ad << 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0;
        dQ << 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0;


        frozen_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        frozen_x << 0.0, 0.0, 0.0, 0.0;

        extended_origin_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        extended_origin_x << 0.0, 0.0, 0.0, 0.0;
    }

    void MP_model::inflate_model(float inf_x_pos, float inf_y_pos) {
        Matrix<double, 4, 1> cartesian_state = get_cartesian_state();
        // std::cout << "in inflate model, setting x from: (" << cartesian_state[0] << ", " << cartesian_state[1] << ", " << cartesian_state[2] << ", " << cartesian_state[3] << ") to ";
        cartesian_state[0] = inf_x_pos;
        cartesian_state[1] = inf_y_pos;
        //std::cout << "(" << cartesian_state[0] << ", " << cartesian_state[1] << ", " << cartesian_state[2] << ", " << cartesian_state[3] << ")" << std::endl;
        
        double new_r = std::sqrt(pow(cartesian_state[0], 2) + pow(cartesian_state[1], 2));
        double new_beta = std::atan2(cartesian_state[1], cartesian_state[0]);
        double new_rdot_over_r = (cartesian_state[0]*cartesian_state[2] + cartesian_state[1]*cartesian_state[3]) / (pow(cartesian_state[0],2) + pow(cartesian_state[1], 2));
        double new_betadot = (cartesian_state[0]*cartesian_state[3] - cartesian_state[1]*cartesian_state[2]) / (pow(cartesian_state[0],2) + pow(cartesian_state[1], 2));
        
        // Matrix<double, 5, 1> new_y;
        y << (1.0 / new_r), std::sin(new_beta), std::cos(new_beta), new_rdot_over_r, new_betadot;
    }
    
    void MP_model::extend_model_origin(Eigen::Vector2f qB) {
        Matrix<double, 4, 1> cartesian_state = get_cartesian_state();
        // std::cout << "extending model from: " << extended_origin_x[0] << ", " << extended_origin_x[1] << ", " << extended_origin_x[2] << ", " << extended_origin_x[3] << " to ";
        cartesian_state[0] -= qB[0];
        cartesian_state[1] -= qB[1];
        // std::cout << extended_origin_x[0] << ", " << extended_origin_x[1] << ", " << extended_origin_x[2] << ", " << extended_origin_x[3] << std::endl;
        
        double new_r = std::sqrt(pow(cartesian_state[0], 2) + pow(cartesian_state[1], 2));
        double new_beta = std::atan2(cartesian_state[1], cartesian_state[0]);
        double new_rdot_over_r = (cartesian_state[0]*cartesian_state[2] + cartesian_state[1]*cartesian_state[3]) / (pow(cartesian_state[0],2) + pow(cartesian_state[1], 2));
        double new_betadot = (cartesian_state[0]*cartesian_state[3] - cartesian_state[1]*cartesian_state[2]) / (pow(cartesian_state[0],2) + pow(cartesian_state[1], 2));
        
        // Matrix<double, 5, 1> new_y;
        y << (1.0 / new_r), std::sin(new_beta), std::cos(new_beta), new_rdot_over_r, new_betadot;
    }
    
    void MP_model::freeze_robot_vel() {
        std::cout << "in freeze_robot_vel" << std::endl;
        Eigen::Vector4d cartesian_state = get_cartesian_state();
        std::cout << "original cartesian state: " << cartesian_state[0] << ", " << cartesian_state[1] << ", " << cartesian_state[2] << ", " << cartesian_state[3] << std::endl;
        //std::cout << "original MP state. r: " << y[0] << ", beta: " << std::atan2(y[1], y[2]) << ", rdot/r: " << y[3] << ", betadot: " << y[4] << std::endl;
        std::cout << "v_ego: " << v_ego[0] << ", " << v_ego[1] << std::endl;
        
        // update cartesian
        cartesian_state[2] += v_ego[0];
        cartesian_state[3] += v_ego[1];
        
        // recalculate polar
        double new_rdot_over_r = (cartesian_state[0]*cartesian_state[2] + cartesian_state[1]*cartesian_state[3]) / (pow(cartesian_state[0],2) + pow(cartesian_state[1], 2));
        double new_betadot = (cartesian_state[0]*cartesian_state[3] - cartesian_state[1]*cartesian_state[2]) / (pow(cartesian_state[0],2) + pow(cartesian_state[1], 2));
        // frozen_x = cartesian_state; // << cartesian_state[0], cartesian_state[1], cartesian_state[2], cartesian_state[3];
        frozen_y << y(0), y(1), y(2), new_rdot_over_r, new_betadot; // get_state(); // <<    
        //std::cout << "modified cartesian state: " << frozen_x[0] << ", " << frozen_x[1] << ", " << frozen_x[2] << ", " << frozen_x[3] << std::endl;
        //std::cout << "modified MP state. r: " << frozen_y[0] << ", beta: " << std::atan2(frozen_y[1], frozen_y[2]) << ", rdot/r: " << frozen_y[3] << ", betadot: " << frozen_y[4] << std::endl;
    }

    void MP_model::frozen_state_propagate(double dt) {
        Matrix<double, 5, 1> new_frozen_y;     
        new_frozen_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        // std::cout << "frozen_y stepping from 1/r: " << frozen_y[0] << ", beta: " << std::atan2(frozen_y[1], frozen_y[2]) << ", rdot/r: " << frozen_y[3] << ", betadot: " << frozen_y[4] << " to ";
        // discrete euler update of state (ignoring rbt acceleration, set as 0)
        new_frozen_y[0] = frozen_y[0] + (-frozen_y[3]*frozen_y[0])*dt;
        new_frozen_y[1] = frozen_y[1] + frozen_y[2]*frozen_y[4]*dt;
        new_frozen_y[2] = frozen_y[2] + (-frozen_y[1]*frozen_y[4])*dt;

        new_frozen_y[1] /= std::sqrt(pow(new_frozen_y[1], 2) + pow(new_frozen_y[2],2));
        new_frozen_y[2] /= std::sqrt(pow(new_frozen_y[1], 2) + pow(new_frozen_y[2],2));
        
        new_frozen_y[3] = frozen_y[3] + (frozen_y[4]*frozen_y[4] - frozen_y[3]*frozen_y[3]) * dt;
        new_frozen_y[4] = frozen_y[4] + (-2 * frozen_y[3]*frozen_y[4])*dt;
        frozen_y = new_frozen_y; // is this ok? do we need a deep copy?
        // std::cout << " 1/r: " << frozen_y[0] << ", beta: " << std::atan2(frozen_y[1], frozen_y[2]) << ", rdot/r: " << frozen_y[3] << ", betadot: " << frozen_y[4] << std::endl;
    }
    

    void MP_model::integrate() {
        t = ros::Time::now().toSec();
        dt = t - t0; // 0.01
        //std::cout << "t0: " << t0 << ", t: " << t << std::endl;
        // std::cout << "a: " << a[0] << ", " << a[1] << ", dt: " << dt << std::endl;
        double a_r = accel[0]*y[2] + accel[1]*y[1]; // ax*cos(beta) + ay*sin(beta)
        double a_beta = -accel[0]*y[1] + accel[1]*y[2]; // -ax*sin(beta) + ay*cos(beta)
        //std::cout << "a_r: " << a_r << ", a_beta " << a_beta << std::endl;
        Matrix<double, 5, 1> new_y;
        new_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        // discrete euler update of state
        // 1/r, sin(beta), cos(beta), rdot/r, betadot 
        // double omega_rbt = v_ego[2];
        // double alpha_rel = accel[2];
        new_y[0] = y[0] + (-y[3]*y[0])*dt; // 1/r
        new_y[1] = y[1] + y[2]*y[4]*dt; // sin(beta)
        new_y[2] = y[2] + -y[1]*y[4]*dt; // cos(beta)
        new_y[1] /= std::sqrt(pow(new_y[1], 2) + pow(new_y[2], 2));
        new_y[2] /= std::sqrt(pow(new_y[1], 2) + pow(new_y[2], 2));
        new_y[3] = y[3] + (y[4]*y[4] - y[3]*y[3] + y[0]* a_r) * dt; // rdot/r
        new_y[4] = y[4] + (-2 * y[3]*y[4] + y[0]*a_beta)*dt; // betadot
        y = new_y;
    }

    void MP_model::linearize() {
        double a_r = accel[0]*y[2] + accel[1]*y[1]; // ax*cos(beta) + ay*sin(beta)
        double a_beta = -accel[0]*y[1] + accel[1]*y[2]; // -ax*sin(beta) + ay*cos(beta)
        //std::cout << "a_r: " << a_r << std::endl;
        //std::cout << "a_beta: " << a_beta << std::endl;
        //std::cout << "y in linearize: " << y << std::endl;
        A << -y[3], 0.0, 0.0, -y[0], 0.0,
                 0.0, 0.0, y[4], 0.0, y[2],
                 0.0, -y[4], 0.0, 0.0, -y[1],
                 a_r, y[0]*accel[1], y[0]*accel[0], -2*y[3], 2*y[4],
                 a_beta, -y[0]*accel[0], y[0]*accel[1], -2*y[4], -2*y[3];

        Ad << 1.0 - y[3]*dt, 0.0, 0.0, -y[0]*dt, 0.0,
                 0.0, 1.0, y[4]*dt, 0.0, y[2]*dt,
                 0.0, -y[4]*dt, 1.0, 0.0, -y[1]*dt,
                 a_r*dt, y[0]*accel[1]*dt, y[0]*accel[0]*dt, 1 - 2*y[3]*dt, 2*y[4]*dt,
                 a_beta*dt, -y[0]*accel[0]*dt, y[0]*accel[1]*dt, -2*y[4]*dt, 1 - 2*y[3]*dt;
        // std::cout << "Ad in linearize: " << Ad << std::endl;
    }

    void MP_model::discretizeQ() {
        dQ = Q * dt;

        Matrix<double, 5, 5> M2 = 0.5 * dt * ((A * dQ).transpose() + A * dQ);
        Matrix<double, 5, 5> M3 = 0.3333 * dt * dt * (A * dQ).transpose();

        dQ = dQ + M2 + M3;
    }

    void MP_model::kf_update_loop(Matrix<double, 3, 1> y_tilde, Matrix<double, 1, 3> _a_ego, Matrix<double, 1, 3> _v_ego) {
        // acceleration comes in wrt robot frame
        accel = -1 * _a_ego; // negative because a = a_target - a_ego, but we assume a_target = 0
        v_ego = _v_ego;
        Eigen::Vector4d cart_state = get_cartesian_state();
        std::cout << "y_i:" << y[0] << ", " << y[1] << ", " << y[2] << ", " << y[3] << ", " << y[4] << ". x_i: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;
        //std::cout << "acceleration" << std::endl;
        std::cout << "v_ego: " << v_ego[0] << ", " << v_ego[1] << ", " << v_ego[2] << std::endl;
        std::cout << "a_ego: " << _a_ego[0] << ", " << _a_ego[1] << ", " << _a_ego[2] << std::endl;
        //std::cout<< "integrating" << std::endl;
        integrate();
        cart_state = get_cartesian_state();
        std::cout << "y_i+1_prime: " << y[0] << ", " << y[1] << ", " << y[2] << ", " << y[3] << ", " << y[4] << ". x_i+1_prime: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;
        // cart_state = get_cartesian_state();
        //std::cout << "y_i bar:" << y[0] << ", " << y[1] << ", " << y[2] << ", " << y[3] << ", " << y[4] << std::endl;
        //std::cout << "x_i bar: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;
        std::cout << "y_tilde: " << y_tilde[0] << ", " << y_tilde[1] << ", " << y_tilde[2] << ". x_tilde: " << (1.0 / y_tilde[0])*y_tilde[2] << ", " << (1.0 / y_tilde[0])*y_tilde[1] << std::endl;

        //std::cout << "y after integration" << y << std::endl;
        //std::cout<< "linearizing" << std::endl;
        linearize();
        // std::cout<< "discretizing Q" << std::endl;
        discretizeQ();

        //std::cout<< "estimating covariance matrix" << std::endl;
        //std::cout << "Ad: " << Ad << std::endl;
        //std::cout << "initial P: " << P << std::endl;
        Matrix<double, 5, 5> Ad_transpose = Ad.transpose();
        //std::cout << "Ad_transpose: " << Ad_transpose << std::endl;
        //std::cout << "dQ: " << dQ << std::endl;

        Matrix<double, 5, 5> new_P = Ad * P * Ad_transpose + dQ;
        //std::cout << "new_P: " << new_P << std::endl;
        P = new_P;
        // std::cout << "P: " << P << std::endl;

        //std::cout<< "updating Kalman gain" << std::endl;

        //std::cout << "H: " << H << std::endl;
        Matrix<double, 5, 3> H_transpose = H.transpose();
        //std::cout << "H_transpose: " << H_transpose << std::endl;
        //std::cout << "R: " << R << std::endl;
        tmp_mat = H*P*H_transpose + R;
        //std::cout << "tmp_mat: " << tmp_mat << std::endl;
        Matrix<double,3,3> inverted_tmp_mat = tmp_mat.inverse();
        //std::cout << "tmp_mat inverse: " << inverted_tmp_mat << std::endl;
        Matrix<double,5,3> P_H_prod = P * H_transpose;
        //std::cout << "P_H_prod: " << P_H_prod << std::endl;
        //std::cout << "inverted tmp mat: " << inverted_tmp_mat << std::endl;
        G = P_H_prod * inverted_tmp_mat;
        //std::cout << "G: " << G << std::endl;
        //std::cout << "error: " << y_tilde - H*y << std::endl;
        //std::cout<< "updating state" << std::endl;
        Matrix<double, 5, 1> y_update_mat = G*(y_tilde - H*y);
        //std::cout << "actual update to y: " << y_update_mat << std::endl;
        y = y + y_update_mat;

        y[1] /= std::sqrt(pow(y[1], 2) + pow(y[2], 2));
        y[2] /= std::sqrt(pow(y[1], 2) + pow(y[2], 2));

        cart_state = get_cartesian_state();
        // std::cout << "y_i+1:" << y[0] << ", " << y[1] << ", " << y[2] << ", " << y[3] << ", " << y[4] << ". x_i+1: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;

        //std::cout<< "updating covariance matrix" << std::endl;
        P = (MatrixXd::Identity(5,5) - G*H)*P;
        // std::cout << "P after update: " << P << std::endl;
        t0 = t;
        //std::cout << "" << std::endl;
    }

    Eigen::Vector4d MP_model::get_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]

        // y state:
        // [1/r, sin(beta), cos(beta), rdot/r, betadot]
        Eigen::Vector4d x(0.0, 0.0, 0.0, 0.0);
        x(0) = (1 / y(0)) * y(2); // r * cos(beta)
        x(1) = (1 / y(0)) * y(1); // r * sin(beta)
        x(2) = (1 / y(0)) * (y(3) * y(2) - y(4)*y(1)); // 
        x(3) = (1 / y(0)) * (y(4) * y(2) + y(3)*y(1));
        return x;
    }

    Matrix<double, 5, 1> MP_model::cartesian_to_polar_state() {
        Matrix<double, 5, 1> polar_y;
        polar_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        polar_y(0) = 1.0 / std::sqrt(pow(x[0], 2) + pow(x[1], 2));
        double beta = std::atan2(x[1], x[0]);
        polar_y(1) = std::sin(beta);
        polar_y(2) = std::cos(beta);
        polar_y(3) = (x[0]*x[2] + x[1]*x[3]) / (pow(x[0],2) + pow(x[1], 2));
        polar_y(4) = (x[0]*x[3] - x[1]*x[2]) / (pow(x[0],2) + pow(x[1], 2));
        return polar_y;
    }

    Eigen::Vector4d MP_model::get_frozen_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d frozen_x(0.0, 0.0, 0.0, 0.0);
        frozen_x(0) = (1 / frozen_y(0)) * frozen_y(2); // r * cos(beta)
        frozen_x(1) = (1 / frozen_y(0)) * frozen_y(1); // r * sin(beta)
        frozen_x(2) = (1 / frozen_y(0)) * (frozen_y(2)*frozen_y(3) - frozen_y(4)*frozen_y(1));
        frozen_x(3) = (1 / frozen_y(0)) * (frozen_y(2)*frozen_y(4) + frozen_y(1)*frozen_y(3));
        return frozen_x;
    }

    void MP_model::set_init_state(double r, double beta) {
        y << 1.0 / r, 
                std::sin(beta), 
                std::cos(beta), 
                0.0, 
                0.0;
    }

    Matrix<double, 5, 1> MP_model::get_state() {
        return y;
    }

    Matrix<double, 5, 1> MP_model::get_frozen_state() {
        return frozen_y;
    }

    Matrix<double, 3, 1> MP_model::get_v_ego() {
        return v_ego;
    }

    void MP_model::set_side(std::string _side) {
        side = _side;
    }
    
    std::string MP_model::get_side() {
        return side;
    }

    int MP_model::get_index() {
        return index;
    }

}