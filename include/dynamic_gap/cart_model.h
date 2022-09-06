#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <random>


using namespace Eigen;

namespace dynamic_gap {
    class cart_model {
        private:
            int n;
            Matrix<double, 2, 4> H; // observation matrix
            Matrix<double, 4, 2> H_transpose;
            Matrix2d R_k, new_R; // measurement noise matrix
            Matrix<double, 4, 4> Q_k, Q_1, Q_2, Q_3, new_Q; // covariance noise matrix
            Matrix<double, 4, 4> dQ; // discretized covariance noise matrix

            Matrix<double, 2, 2> tmp_mat; //  place holder for inverse

            Matrix<double, 4, 1> x_hat_kmin1_plus, x_hat_k_minus, x_hat_k_plus, 
                                 new_x, x_ground_truth, x_ground_truth_gap_only, frozen_x, rewind_x;
            Matrix<double, 4, 4> P_kmin1_plus, P_k_minus, P_k_plus, P_intermediate, new_P; // covariance matrix
            Matrix<double, 4, 2> G_k; // kalman gain
            Matrix<double, 2, 1> x_tilde, innovation, residual;

            double t_min1;
            double t;
            double dt, inter_dt;
            double alpha_Q, alpha_R;

            Matrix<double, 1, 3> a_ego, v_ego;

            Matrix<double, 4, 4> A,  STM;
            std::string side;
            int index;

            bool initialized;
            double life_time, start_time;

            std::vector< std::vector<double>> previous_states, previous_measurements, previous_measurements_gap_only,
                                              previous_ego_accels, previous_ego_vels, previous_times,
                                              previous_gap_only_states, vel_euler_derivatives;
            double life_time_threshold;
            Matrix<double, 4, 4> eyes;
            Matrix<double, 2, 2> inverted_tmp_mat;
            std::string plot_dir;

            std::vector<geometry_msgs::Pose> agent_odoms;
            std::vector<geometry_msgs::Vector3Stamped> agent_vels;

            bool perfect;
            bool print;
            bool plot;
            bool plotted;
            std::vector<double> prev_euler_deriv;

            std::vector<geometry_msgs::Twist> intermediate_vels;
            std::vector<geometry_msgs::TwistStamped> intermediate_accs;

        public:

            cart_model(std::string, int, double, double, Matrix<double, 1, 3>);

            void initialize(double, double, Matrix<double, 1, 3>);

            ~cart_model() {};

            Eigen::Vector4d update_ground_truth_cartesian_state();
            Eigen::Vector4d get_cartesian_state();
            Eigen::Vector4d get_GT_cartesian_state();

            Eigen::Vector4d get_frozen_cartesian_state();
            Eigen::Vector4d get_rewind_cartesian_state();
            Eigen::Vector4d get_modified_polar_state();
            Eigen::Vector4d get_frozen_modified_polar_state();
            Eigen::Vector4d get_rewind_modified_polar_state();

            Matrix<double, 3, 1> get_v_ego();
            Matrix<double, 4, 1> integrate();
            void linearize(int i);
            void discretizeQ();

            void frozen_state_propagate(double dt);
            void rewind_propagate(double dt);
            void freeze_robot_vel();
            void set_rewind_state();

            void kf_update_loop(Matrix<double, 2, 1> range_bearing_measurement, 
                                std::vector<geometry_msgs::TwistStamped> intermediate_accs, 
                                std::vector<geometry_msgs::Twist> intermediate_vels, 
                                bool print,
                                std::vector<geometry_msgs::Pose> _agent_odoms,
                                std::vector<geometry_msgs::Vector3Stamped> _agent_vels,
                                double scan_dt);
            void set_side(std::string _side);
            std::string get_side();
            int get_index();
            void inflate_model(float x, float y);

            void set_initialized(bool _initialized);
            bool get_initialized();
            void plot_states();
    };
}