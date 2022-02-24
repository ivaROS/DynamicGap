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
    MP_model::MP_model(std::string _side, int _index, double init_r, double init_beta) {
        side = _side;
        index = _index;
        initialize(init_r, init_beta);
    }

    MP_model::~MP_model() {}

    void MP_model::initialize(double init_r, double init_beta) {
        // OBSERVATION MATRIX
        H << 1.0, 0.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0, 0.0;
        // MEASUREMENT NOISE
        R << 0.0001, 0.0, 0.0,
             0.0, 0.0001, 0.0,
             0.0, 0.0, 0.0001;
        // PROCESS NOISE
        Q << 0.001, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.001, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.001, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.001, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.001;
        y << 1.0 / init_r, 
                std::sin(init_beta), 
                std::cos(init_beta), 
                0.0, 
                0.0;
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
        a << 0.0, 0.0;
        v_ego << 0.0, 0.0;

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
    }

    void MP_model::freeze_robot_vel() {
        Eigen::Vector4d cartesian_state = get_cartesian_state();
        //std::cout << "original cartesian state: " << cartesian_state[0] << ", " << cartesian_state[1] << ", " << cartesian_state[2] << ", " << cartesian_state[3] << std::endl;
        //std::cout << "original MP state. r: " << y[0] << ", beta: " << std::atan2(y[1], y[2]) << ", rdot/r: " << y[3] << ", betadot: " << y[4] << std::endl;
        // std::cout << "v_ego: " << v_ego[0] << ", " << v_ego[1] << std::endl;
        // update cartesian
        cartesian_state[2] += v_ego[0];
        cartesian_state[3] += v_ego[1];
        // recalculate polar
        double new_rdot_over_r = (cartesian_state[0]*cartesian_state[2] + cartesian_state[1]*cartesian_state[3]) / (pow(cartesian_state[0],2) + pow(cartesian_state[1], 2));
        double new_betadot = (cartesian_state[0]*cartesian_state[3] - cartesian_state[1]*cartesian_state[2]) / (pow(cartesian_state[0],2) + pow(cartesian_state[1], 2));
        frozen_x << cartesian_state[0], cartesian_state[1], cartesian_state[2], cartesian_state[3];
        frozen_y << y(0), y(1), y(2), new_rdot_over_r, new_betadot;           
        //std::cout << "modified cartesian state: " << frozen_x[0] << ", " << frozen_x[1] << ", " << frozen_x[2] << ", " << frozen_x[3] << std::endl;
        //std::cout << "modified MP state. r: " << frozen_y[0] << ", beta: " << std::atan2(frozen_y[1], frozen_y[2]) << ", rdot/r: " << frozen_y[3] << ", betadot: " << frozen_y[4] << std::endl;
    }

    void MP_model::frozen_state_propagate(double dt) {
        Matrix<double, 1, 5> new_frozen_y;     
        new_frozen_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        // std::cout << "frozen_y stepping from 1/r: " << frozen_y[0] << ", beta: " << std::atan2(frozen_y[1], frozen_y[2]) << ", rdot/r: " << frozen_y[3] << ", betadot: " << frozen_y[4] << " to ";
        // discrete euler update of state (ignoring rbt acceleration, set as 0)
        new_frozen_y[0] = frozen_y[0] + (-frozen_y[3]*frozen_y[0])*dt;
        new_frozen_y[1] = frozen_y[1] + frozen_y[2]*frozen_y[4]*dt;
        new_frozen_y[2] = frozen_y[2] + (-frozen_y[1]*frozen_y[4])*dt;
        new_frozen_y[3] = frozen_y[3] + (-frozen_y[4]*frozen_y[4] - frozen_y[3]*frozen_y[3]) * dt;
        new_frozen_y[4] = frozen_y[4] + (2 * frozen_y[3]*frozen_y[4])*dt;
        frozen_y = new_frozen_y; // is this ok? do we need a deep copy?
        // std::cout << " 1/r: " << frozen_y[0] << ", beta: " << std::atan2(frozen_y[1], frozen_y[2]) << ", rdot/r: " << frozen_y[3] << ", betadot: " << frozen_y[4] << std::endl;
    }


    void MP_model::integrate() {
        t = ros::Time::now().toSec();
        dt = t - t0; // 0.01
        //std::cout << "t0: " << t0 << ", t: " << t << std::endl;
        std::cout << "dt: " << dt << std::endl;
        double a_r = - a[0]*y[1] + a[1]*y[2];
        double a_beta = -a[0]*y[2] - a[1]*y[1];
        //std::cout << "a_r: " << a_r << ", a_beta " << a_beta << std::endl;
        Matrix<double, 1, 5> new_y;
        new_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        // discrete euler update of state
        // 1/r, sin(beta), cos(beta), rdot/r, betadot 
        new_y[0] = y[0] + (-y[3]*y[0])*dt;
        new_y[1] = y[1] + y[2]*y[4]*dt;
        new_y[2] = y[2] + (-y[1]*y[4])*dt;
        new_y[3] = y[3] + (-y[4]*y[4] - y[3]*y[3] + y[0]* a_r) * dt;
        new_y[4] = y[4] + (2 * y[3]*y[4] + y[0]*a_beta)*dt;
        y = new_y; // is this ok? do we need a deep copy?
    }

    void MP_model::linearize() {
        double a_r = - a[0]*y[1] + a[1]*y[2];
        double a_beta = -a[0]*y[2] - a[1]*y[1];

        A(0) = -y[3], 0.0, 0.0, -y[0], 0.0;
        A(1) = 0.0, 0.0, y[4], 0.0, y[2];
        A(2) = 0.0, -y[4], 0.0, 0.0, -y[1];
        A(3) = a_r, -y[0]*a[0], y[0]*a[1], -2*y[3], -2*y[4];
        A(4) = a_beta, -y[0]*a[1], -y[0]*a[0], 2*y[4], 2*y[3];

        Ad(0) = 1.0 - y[3]*dt, 0.0, 0.0, -y[0]*dt, 0.0;
        Ad(1) = 0.0, 1.0, y[4]*dt, 0.0, y[2]*dt;
        Ad(2) = 0.0, -y[4]*dt, 1.0, 0.0, -y[1]*dt;
        Ad(3) = a_r*dt, -y[0]*a[0]*dt, y[0]*a[1]*dt, 1 - 2*y[3]*dt, -2*y[4]*dt;
        Ad(4) = a_beta*dt, -y[0]*a[1]*dt, -y[0]*a[0]*dt, 2*y[4]*dt, 1 + 2*y[3]*dt;
        // std::cout << "Ad: " << Ad << std::endl;
    }

    void MP_model::discretizeQ() {
        dQ = Q * dt;

        Matrix<double, 5, 5> M2 = 0.5 * dt * ((A * dQ).transpose() + A * dQ);
        Matrix<double, 5, 5> M3 = 0.3333 * dt * dt * (A * dQ).transpose();

        dQ = dQ + M2 + M3;
        std::cout << "dQ: " << dQ << std::endl;
    }

    void MP_model::kf_update_loop(Matrix<double, 3, 1> y_tilde, Matrix<double, 1, 2> _a_ego, Matrix<double, 1, 2> _v_ego) {
        // acceleration comes in wrt robot frame
        a = -1 * _a_ego; // negative because a = a_target - a_ego, but we assume a_target = 0
        v_ego = _v_ego;
        Eigen::Vector4d cart_state = get_cartesian_state();
        std::cout << "MP state at start:" << y[0] << ", " << y[1] << ", " << y[2] << ", " << y[3] << ", " << y[4] << std::endl;
        std::cout << "cartesian state at start: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;
        //std::cout << "acceleration" << std::endl;
        // std::cout << "a_ego: " << _a_ego[0] << ", " << _a_ego[1] << std::endl;
        std::cout << "acceleration: " << a[0] << ", " << a[1] << std::endl;
        
        //std::cout<< "integrating" << std::endl;
        integrate();
        cart_state = get_cartesian_state();
        std::cout << "MP state after integrating:" << y[0] << ", " << y[1] << ", " << y[2] << ", " << y[3] << ", " << y[4] << std::endl;
        std::cout << "cartesian state after integrating : " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;
        std::cout << "MP observation: " << y_tilde[0] << ", " << y_tilde[1] << ", " << y_tilde[2] << std::endl;
        std::cout << "cartesian observation: " << (1.0 / y_tilde[0])*-y_tilde[1] << ", " << (1.0 / y_tilde[0])*y_tilde[2] << std::endl;

        //std::cout << "y after integration" << y << std::endl;
        //std::cout<< "linearizing" << std::endl;
        linearize();
        //std::cout<< "discretizing Q" << std::endl;
        discretizeQ();

        //std::cout<< "estimating covariance matrix" << std::endl;
        P = Ad * P * Ad.transpose() + dQ;
        //std::cout << "P after estimate: " << P << std::endl;
        //std::cout<< "updating Kalman gain" << std::endl;

        tmp_mat = H*P*H.transpose() + R;
        Matrix<double,5,3> P_H_prod = P * H.transpose();
        //std::cout << "P_H_prod: " << P_H_prod << std::endl;
        Matrix<double,3,3> inverted_tmp_mat = tmp_mat.inverse();
        //std::cout << "inverted tmp mat: " << inverted_tmp_mat << std::endl;
        G = P_H_prod * inverted_tmp_mat;
        std::cout << "G after update: " << G << std::endl;
        //std::cout<< "updating state" << std::endl;
        Matrix<double, 5, 1> y_update_mat = G*(y_tilde - H*y);
        std::cout << "actual update to y: " << y_update_mat << std::endl;
        y = y + y_update_mat;
        cart_state = get_cartesian_state();
        std::cout << "MP state after update:" << y[0] << ", " << y[1] << ", " << y[2] << ", " << y[3] << ", " << y[4] << std::endl;
        std::cout << "cartesian state after update: " << cart_state[0] << ", " << cart_state[1] << ", " << cart_state[2] << ", " << cart_state[3] << std::endl;

        //std::cout<< "updating covariance matrix" << std::endl;
        P = (MatrixXd::Identity(5,5) - G*H)*P;
        std::cout << "P after update: " << P << std::endl;
        t0 = t;
    }

    Eigen::Vector4d MP_model::get_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d x(0.0, 0.0, 0.0, 0.0);
        x(0) = -(1 / y(0)) * y(1);
        x(1) = (1 / y(0)) * y(2);
        x(2) = (1 / y(0)) * (-y(3) * y(1) - y(4)*y(2));
        x(3) = (1 / y(0)) * (y(3) * y(2) - y(4)*y(1));
        return x;
    }

    Eigen::Vector4d MP_model::get_frozen_cartesian_state() {
        // x state:
        // [r_x, r_y, v_x, v_y]
        Eigen::Vector4d frozen_x(0.0, 0.0, 0.0, 0.0);
        frozen_x(0) = -(1 / frozen_y(0)) * frozen_y(1);
        frozen_x(1) = (1 / frozen_y(0)) * frozen_y(2);
        frozen_x(2) = (1 / frozen_y(0)) * (-frozen_y(3) * frozen_y(1) - frozen_y(4)*frozen_y(2));
        frozen_x(3) = (1 / frozen_y(0)) * (frozen_y(3) * frozen_y(2) - frozen_y(4)*frozen_y(1));
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

    Matrix<double, 2, 1> MP_model::get_v_ego() {
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