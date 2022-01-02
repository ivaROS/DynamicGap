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
            Matrix<float, 3, 5> H; // observation matrix
            Matrix3f R; // measurement noise matrix
            Matrix<float, 5, 5> Q; // covariance noise matrix
            Matrix<float, 5, 5> dQ; // discretized covariance noise matrix

            Matrix<float, 5, 1> y; // modified polar coordinates state
            Matrix<float, 5, 5> P; // covariance matrix
            Matrix<float, 5, 3> G; // kalman gain

            float t0;
            float t;
            float T;

            //float acc_t0;
            //float acc_t;
            //float acc_T;

            Matrix<float, 1, 2> a;
            // Matrix<float, 3, 1> y_tilde;

            Matrix<float, 5, 5> A;
            Matrix<float, 5, 5> Ad;
            //ros::Subscriber acc_sub;
            //Matrix<float, 1, 2> previous_twist;

        public:
            MP_model(std::string frame);

            ~MP_model();

            //void cmd_velCB(const geometry_msgs::Twist::ConstPtr&); // imu callback to get acceleration

            Matrix<float, 5, 1> get_state();
            void integrate();
            void linearize();
            void discretizeQ();

            void kf_update_loop(Matrix<float, 3, 1> y_tilde, Matrix<float, 1, 2> a_rbt);
    };
}