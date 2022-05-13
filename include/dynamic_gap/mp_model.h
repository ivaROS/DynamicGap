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
            Matrix<double, 4, 1> x; // cartesian state
            Matrix<double, 5, 1> frozen_y; // for simulating situation where robot is not moving
            Matrix<double, 4, 1> frozen_x;
            Matrix<double, 5, 1> extended_origin_y; // for extending origin of model in radiallyExtendGap
            Matrix<double, 4, 1> extended_origin_x;
            Matrix<double, 5, 5> P; // covariance matrix
            Matrix<double, 5, 3> G; // kalman gain

            double t0;
            double t;
            double dt;

            Matrix<double, 1, 3> accel;
            Matrix<double, 1, 3> v_ego;
            // Matrix<float, 3, 1> y_tilde;

            Matrix<double, 5, 5> A;
            Matrix<double, 5, 5> Ad;

            std::string side;
            int index;


        public:

            MP_model(std::string, int, double, double, Matrix<double, 1, 3>);
            MP_model(const dynamic_gap::MP_model &model);

            void initialize(double, double, Matrix<double, 1, 3>);

            ~MP_model();


            void set_init_state(double r, double beta);
            Matrix<double, 5, 1> get_state();
            Matrix<double, 5, 1> get_frozen_state();
            Eigen::Vector4d get_cartesian_state();
            Eigen::Vector4d get_frozen_cartesian_state();
            void extend_model_origin(Eigen::Vector2f qB);

            Matrix<double, 3, 1> get_v_ego();
            void integrate();
            void linearize();
            void discretizeQ();
            Matrix<double, 5, 1> cartesian_to_polar_state();

            void frozen_state_propagate(double dt);
            void freeze_robot_vel();
            void kf_update_loop(Matrix<double, 3, 1> y_tilde, Matrix<double, 1, 3> a_ego, Matrix<double, 1, 3> v_ego);
            void set_side(std::string _side);
            std::string get_side();
            int get_index();
            void inflate_model(float x, float y);

    };
}