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
    MP_model::MP_model(std::string frame) {
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
        y << 1.0, 
                1.0, 
                1.0, 
                1.0, 
                1.0;
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
        T = t - t0;
        /*
        acc_t0 = ros::Time::now().toSec();
        acc_t = ros::Time::now().toSec();
        acc_T = t- t0;
        */
        a << 0.0, 0.0;

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
        //ros::NodeHandle n;
        // std::cout << "topic name: " << robot_name + "/cmd_vel" << std::endl;
        // acc_sub = n.subscribe(robot_name + "/cmd_vel", 100, &MP_model::cmd_velCB, this);
    }

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

    void MP_model::integrate() {
        t = ros::Time::now().toSec();
        T = t - t0; // 0.01
        //std::cout << "t0: " << t0 << ", t: " << t << std::endl;
        //std::cout << "T: " << T << std::endl;
        double a_r = a[1]*y[2] - a[0]*y[1];
        double a_beta = -a[0]*y[2] - a[1]*y[1];
        //std::cout << "a_r: " << a_r << ", a_beta " << a_beta << std::endl;
        Matrix<double, 1, 5> new_y;
        new_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        // discrete euler update of state
        new_y[0] = y[0] + (-y[3]*y[0])*T;
        new_y[1] = y[1] + y[2]*y[4]*T;
        new_y[2] = y[2] + (-y[1]*y[4])*T;
        new_y[3] = y[3] + (y[4]*y[4] - y[3]*y[3] - y[0]* a_r) * T;
        new_y[4] = y[4] + (-2 * y[3]*y[4] - y[0]*a_beta)*T;
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

        Ad(0) = 1.0 - y[3]*T, 0.0, 0.0, -y[0]*T, 0.0;
        Ad(1) = 0.0, 1.0, y[4]*T, 0.0, y[2]*T;
        Ad(2) = 0.0, -y[4]*T, 1.0, 0.0, -y[1]*T;
        Ad(3) = -a_r*T, y[0]*a[0]*T, -y[0]*a[1]*T, 1 - 2*y[3]*T, 2*y[4]*T;
        Ad(4) = -a_beta*T, y[0]*a[0]*T, y[0]*a[0]*T, -2*y[4]*T, 1 - 2*y[3]*T;
        // std::cout << "Ad: " << Ad << std::endl;
    }

    void MP_model::discretizeQ() {
        dQ = Q * T;

        Matrix<double, 5, 5> M2 = 0.5 * T * ((A * dQ).transpose() + A * dQ);
        Matrix<double, 5, 5> M3 = 0.3333 * T * T * (A * dQ).transpose();

        dQ = dQ + M2 + M3;
        // std::cout << "dQ: " << dQ << std::endl;
    }

    void MP_model::kf_update_loop(Matrix<double, 3, 1> y_tilde, Matrix<double, 1, 2> a_odom) {
        // acceleration comes in in world frame
        a = a_odom;
        //std::cout << "y at start: " << y[0] << ", " << y[1] << ", " << y[2] << ", " << y[3] << ", " << y[4] << std::endl;
        //std::cout << "y_tilde: " << y_tilde[0] << ", " << y_tilde[1] << ", " << y_tilde[2] << std::endl;
        //std::cout << "acceleration" << std::endl;
        // std::cout << a[0] << ", " << a[1] << std::endl;
        
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
        //std::cout << "y after update" << y[0] << ", " << y[1] << ", " << y[2] << ", " << y[3] << ", " << y[4] << std::endl;
        //std::cout<< "updating covariance matrix" << std::endl;
        P = (MatrixXd::Identity(5,5) - G*H)*P;
        //std::cout << "P after update: " << P << std::endl;
        t0 = t;
    }

    Matrix<double, 5, 1> MP_model::get_state() {
        return y;
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