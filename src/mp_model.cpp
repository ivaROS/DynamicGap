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
    MP_model::MP_model(std::string _side, int _index) {
        // std::string robot_name = frame.substr(0, frame.length() - 8);
        // std::cout << "robot_name " << robot_name << std::endl;
        H << 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0;
        R << 0.00001, 0.0, 0.0,
            0.0, 0.00001, 0.0,
            0.0, 0.0, 0.00001;
        Q << 0.005, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.005, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.005, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.005, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.005;
        y << 0.0, 
                0.0, 
                0.0, 
                0.0, 
                0.0;
        P << 10.0e-6, 0.0, 0.0, 0.0, 0.0,
                0.0, 10.0e-4, 0.0, 0.0, 0.0,
                0.0, 0.0, 10.0e-4, 0.0, 0.0,
                0.0, 0.0, 0.0, 10.0e-4, 0.0,
                0.0, 0.0, 0.0, 0.0, 10.0e-3;
        G << 1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0,
                1.0, 1.0, 1.0;

        t0 = ros::Time::now().toSec();
        t = ros::Time::now().toSec();
        dt = t - t0;
        /*
        acc_t0 = ros::Time::now().toSec();
        acc_t = ros::Time::now().toSec();
        acc_T = t- t0;
        */
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
        side = _side;
        index = _index;
        //ros::NodeHandle n;
        // std::cout << "topic name: " << robot_name + "/cmd_vel" << std::endl;
        // acc_sub = n.subscribe(robot_name + "/cmd_vel", 100, &MP_model::cmd_velCB, this);
    }

    /*
    MP_model::MP_model(const dynamic_gap::MP_model& model) {
        // std::string robot_name = frame.substr(0, frame.length() - 8);
        // std::cout << "robot_name " << robot_name << std::endl;
        H = model.H;
        R = model.R;
        Q = model.Q;
        y = model.y;
        P = model.P;
        G = model.G;

        t0 = model.t0;
        t = model.t;
        dt = model.dt;
        
        acc_t0 = ros::Time::now().toSec();
        acc_t = ros::Time::now().toSec();
        acc_T = t- t0;
        
        a = model.a;
        v_ego = model.v_ego;

        A = model.A;
        Ad = model.Ad;
        dQ = model.dQ;

        frozen_y = model.frozen_y;

    }
    */

    MP_model::~MP_model() {}

    /*
    void MP_model::cmd_velCB(const geometry_msgs::Twist::ConstPtr& msg) {
        acc_t = ros::Time::now().toSec();
        acc_T = 0.01; // acc_t - acc_t0;
        // std::cout << "msg linear x: " << msg->linear.x << " msg linear y: " << msg->linear.y << std::endl;
        //std::cout << "previous_twist[0]: " << previous_twist[0] << " previous_twist[1]: " << previous_twist[1] << std::endl;
        a[0] = (msg->linear.x - previous_twist[0]) / acc_T;
        a[1] = (msg->linear.y - previous_twist[1]) / acc_T;
        //std::cout << "in imuCB: " << a[0] << ", " << a[1] << std::endl;
        //std::cout << "acc_t: " << acc_t << ", acc_t0: " << acc_t0 << " acc_T: " << acc_T << std::endl;
        previous_twist[0] = msg->linear.x;
        previous_twist[1] = msg->linear.y;
        acc_t0 = acc_t;
        //a[0] = msg->linear_acceleration.x;
        //a[1] = msg->linear_acceleration.y;
    }
    */

    void MP_model::freeze_robot_vel() {
        Eigen::Vector4d cartesian_state = get_cartesian_state();
        //std::cout << "original cartesian state: " << cartesian_state[0] << ", " << cartesian_state[1] << ", " << cartesian_state[2] << ", " << cartesian_state[3] << std::endl;
        //std::cout << "original MP state. r: " << y[0] << ", beta: " << std::atan2(y[1], y[2]) << ", rdot/r: " << y[3] << ", betadot: " << y[4] << std::endl;
        //std::cout << "v_ego: " << v_ego[0] << ", " << v_ego[1] << std::endl;
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
        new_frozen_y[3] = frozen_y[3] + (frozen_y[4]*frozen_y[4] - frozen_y[3]*frozen_y[3]) * dt;
        new_frozen_y[4] = frozen_y[4] + (-2 * frozen_y[3]*frozen_y[4])*dt;
        frozen_y = new_frozen_y; // is this ok? do we need a deep copy?
        // std::cout << " 1/r: " << frozen_y[0] << ", beta: " << std::atan2(frozen_y[1], frozen_y[2]) << ", rdot/r: " << frozen_y[3] << ", betadot: " << frozen_y[4] << std::endl;
    }


    void MP_model::integrate() {
        t = ros::Time::now().toSec();
        dt = t - t0; // 0.01
        //std::cout << "t0: " << t0 << ", t: " << t << std::endl;
        //std::cout << "T: " << T << std::endl;
        double a_r = a[1]*y[2] - a[0]*y[1];
        double a_beta = -a[0]*y[2] - a[1]*y[1];
        //std::cout << "a_r: " << a_r << ", a_beta " << a_beta << std::endl;
        Matrix<double, 1, 5> new_y;
        new_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        // discrete euler update of state
        new_y[0] = y[0] + (-y[3]*y[0])*dt;
        new_y[1] = y[1] + y[2]*y[4]*dt;
        new_y[2] = y[2] + (-y[1]*y[4])*dt;
        new_y[3] = y[3] + (y[4]*y[4] - y[3]*y[3] - y[0]* a_r) * dt;
        new_y[4] = y[4] + (-2 * y[3]*y[4] - y[0]*a_beta)*dt;
        y = new_y; // is this ok? do we need a deep copy?
    }

    void MP_model::linearize() {
        double a_r = a[1]*y[2] - a[0]*y[1];
        double a_beta = -a[0]*y[2] - a[1]*y[1];

        A(0) = -y[3], 0.0, 0.0, -y[0], 0.0;
        A(1) = 0.0, 0.0, y[4], 0.0, y[2];
        A(2) = 0.0, -y[4], 0.0, 0.0, -y[1];
        A(3) = -a_r, y[0]*a[0], -y[0]*a[1], -2*y[3], 2*y[4];
        A(4) = -a_beta, y[0]*a[1], y[0]*a[0], -2*y[4], -2*y[3];

        Ad(0) = 1.0 - y[3]*dt, 0.0, 0.0, -y[0]*dt, 0.0;
        Ad(1) = 0.0, 1.0, y[4]*dt, 0.0, y[2]*dt;
        Ad(2) = 0.0, -y[4]*dt, 1.0, 0.0, -y[1]*dt;
        Ad(3) = -a_r*dt, y[0]*a[0]*dt, -y[0]*a[1]*dt, 1 - 2*y[3]*dt, 2*y[4]*dt;
        Ad(4) = -a_beta*dt, y[0]*a[0]*dt, y[0]*a[0]*dt, -2*y[4]*dt, 1 - 2*y[3]*dt;
        // std::cout << "Ad: " << Ad << std::endl;
    }

    void MP_model::discretizeQ() {
        dQ = Q * dt;

        Matrix<double, 5, 5> M2 = 0.5 * dt * ((A * dQ).transpose() + A * dQ);
        Matrix<double, 5, 5> M3 = 0.3333 * dt * dt * (A * dQ).transpose();

        dQ = dQ + M2 + M3;
        // std::cout << "dQ: " << dQ << std::endl;
    }

    void MP_model::kf_update_loop(Matrix<double, 3, 1> y_tilde, Matrix<double, 1, 2> _a_ego, Matrix<double, 1, 2> _v_ego) {
        // acceleration comes in wrt robot frame
        a = -_a_ego; // negative because a = a_target - a_ego, but we assume a_target = 0
        v_ego = _v_ego;
        std::cout << "y at start: " << y[0] << ", " << y[1] << ", " << y[2] << ", " << y[3] << ", " << y[4] << std::endl;
        std::cout << "y_tilde: " << y_tilde[0] << ", " << y_tilde[1] << ", " << y_tilde[2] << std::endl;
        //std::cout << "acceleration" << std::endl;
        //std::cout << "acceleration: " << a[0] << ", " << a[1] << std::endl;
        
        //std::cout<< "integrating" << std::endl;
        integrate();
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
        Matrix<double,3,3> inverted_tmp_mat = tmp_mat.inverse();
        //std::cout << "inverted tmp mat: " << inverted_tmp_mat << std::endl;
        G = P * H.transpose() * inverted_tmp_mat;
        //std::cout << "G after update: " << G << std::endl;
        //std::cout<< "updating state" << std::endl;
        y = y + G*(y_tilde - H*y);
        std::cout << "y after update" << y[0] << ", " << y[1] << ", " << y[2] << ", " << y[3] << ", " << y[4] << std::endl;
        //std::cout<< "updating covariance matrix" << std::endl;
        P = (MatrixXd::Identity(5,5) - G*H)*P;
        //std::cout << "P after update: " << P << std::endl;
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

/*
int main(int argc, char **argv) {
    ros::init(argc, argv, "mp_kf");

    ros::NodeHandle n;

    MP_model model = MP_model();

    std:: cout << "in mp kf main" << std::endl;

    ros::Subscriber imu_sub = n.subscribe("/imu", 100, &MP_model::imuCB, &model);
    ros::Rate loop_rate(10);

    while (ros::ok()) {

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}
*/