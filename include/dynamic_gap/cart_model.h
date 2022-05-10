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
    class cart_model {
        private:
            int n;
            Matrix<double, 2, 4> H; // observation matrix
            Matrix2d R; // measurement noise matrix
            Matrix<double, 4, 4> Q; // covariance noise matrix
            Matrix<double, 4, 4> dQ; // discretized covariance noise matrix

            Matrix<double, 2, 2> tmp_mat; //  place holder for inverse

            Matrix<double, 4, 1> x; // cartesian state
            Matrix<double, 4, 1> frozen_x;
            Matrix<double, 4, 1> extended_origin_x;
            Matrix<double, 4, 4> P; // covariance matrix
            Matrix<double, 4, 2> G; // kalman gain
            Matrix<double, 2, 1> x_tilde;

            double t0;
            double t;
            double dt;

            Matrix<double, 1, 3> accel;
            Matrix<double, 1, 3> v_ego;

            Matrix<double, 4, 4> A;
            Matrix<double, 4, 4> Ad;

            std::string side;
            int index;
            double omega_rbt_prev;

            Matrix<double, 2, 1> linear_acc_ego;
            Matrix<double, 2, 1> linear_vel_ego;
            double ang_vel_ego;


        public:

            cart_model(std::string, int, double, double, Matrix<double, 1, 3>);
            cart_model(const dynamic_gap::cart_model &model);

            void initialize(double, double, Matrix<double, 1, 3>);

            ~cart_model();

            Matrix<double, 4, 1> get_state();
            Matrix<double, 4, 1> get_frozen_state();
            Eigen::Vector4d get_cartesian_state();
            Eigen::Vector4d get_frozen_cartesian_state();
            Eigen::Vector4d get_modified_polar_state();
            Eigen::Vector4d get_frozen_modified_polar_state();

            Matrix<double, 3, 1> get_v_ego();
            void integrate();
            void linearize();
            void discretizeQ();

            void frozen_state_propagate(double dt);
            void freeze_robot_vel();
            void kf_update_loop(Matrix<double, 2, 1> range_bearing_measurement, Matrix<double, 1, 3> a_ego, Matrix<double, 1, 3> v_ego);
            void set_side(std::string _side);
            std::string get_side();
            int get_index();
            void inflate_model(float x, float y);
    };
}