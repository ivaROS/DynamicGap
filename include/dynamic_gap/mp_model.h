#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>


using namespace Eigen;

namespace dynamic_gap {
    class MP_model {
        private:
            int n;
            Matrix<double, 3, 5> H; // observation matrix
            Matrix3d R; // measurement noise matrix
            Matrix<double, 5, 5> Q; // covariance noise matrix
            Matrix<double, 5, 5> dQ; // discretized covariance noise matrix

            Matrix<double, 3, 3> tmp_mat; //  place holder for inverse

            Matrix<double, 5, 1> y; // modified polar coordinates state
            Matrix<double, 5, 1> frozen_y; // for simulating situation where robot is not moving
            Matrix<double, 4, 1> frozen_x;
            Matrix<double, 5, 5> P; // covariance matrix
            Matrix<double, 5, 3> G; // kalman gain

            double t0;
            double t;
            double dt;

            //float acc_t0;
            //float acc_t;
            //float acc_T;

            Matrix<double, 1, 2> a;
            Matrix<double, 1, 2> v_ego;
            // Matrix<float, 3, 1> y_tilde;

            Matrix<double, 5, 5> A;
            Matrix<double, 5, 5> Ad;

            std::string side;
            int index;
            //ros::Subscriber acc_sub;
            //Matrix<float, 1, 2> previous_twist;

        public:

            MP_model(std::string, int);

            ~MP_model();

            //void cmd_velCB(const geometry_msgs::Twist::ConstPtr&); // imu callback to get acceleration

            Matrix<double, 5, 1> get_state();
            Matrix<double, 5, 1> get_frozen_state();
            Eigen::Vector4d get_cartesian_state();
            Matrix<double, 2, 1> get_v_ego();
            void integrate();
            void linearize();
            void discretizeQ();

            void frozen_state_propagate(double dt);
            void freeze_robot_vel();
            void kf_update_loop(Matrix<double, 3, 1> y_tilde, Matrix<double, 1, 2> a_ego, Matrix<double, 1, 2> v_ego);
            void set_side(std::string _side);
            std::string get_side();
            int get_index();

    };
}