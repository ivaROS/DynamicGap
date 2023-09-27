#include <ros/ros.h>
#include <dynamic_gap/dynamicgap_config.h>
// #include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
// #include <sensor_msgs/Imu.h>
// #include <tf2_ros/buffer.h>
// #include <sensor_msgs/LaserScan.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <dynamic_gap/estimation/estimator.h>

// #include <random>


// using namespace Eigen;

namespace dynamic_gap {
    class rot_frame_kf : public Estimator {
        private:
            void processEgoRobotVelsAndAccs(const ros::Time & t_update);
            
            Eigen::Matrix4d Q_1, Q_2, Q_3; // covariance noise matrix
            double R_scalar, Q_scalar;

            Eigen::Matrix2d tmp_mat; //  place holder for inverse

            Eigen::Vector4d new_x, x_ground_truth, x_ground_truth_gap_only, frozen_x, rewind_x;
            Eigen::Matrix4d P_intermediate, new_P; // covariance matrix

            std::string side;
            int index;

            bool initialized;
            double life_time, start_time;

            std::vector< std::vector<double>> previous_states, previous_measurements, previous_measurements_gap_only,
                                              previous_ego_accels, previous_ego_vels, previous_times,
                                              previous_gap_only_states, vel_euler_derivatives;
            double life_time_threshold;
            Eigen::Matrix4d eyes;
            std::string plot_dir;

            std::vector<geometry_msgs::Pose> agent_odoms;
            std::vector<geometry_msgs::Vector3Stamped> agent_vels;

            bool perfect;
            bool print;
            bool plot;
            bool plotted;
            std::vector<double> prev_euler_deriv;

            ros::Time t_last_update;
            std::vector<geometry_msgs::TwistStamped> ego_rbt_vels;
            std::vector<geometry_msgs::TwistStamped> ego_rbt_accs;        
            geometry_msgs::TwistStamped last_ego_rbt_vel;
            geometry_msgs::TwistStamped last_ego_rbt_acc;

        public:

            rot_frame_kf(std::string, int, double, double,const ros::Time & t_update,
                        const geometry_msgs::TwistStamped & last_ego_rbt_vel,
                        const geometry_msgs::TwistStamped & last_ego_rbt_acc);

            void initialize(double, double, const ros::Time & t_update,
                            const geometry_msgs::TwistStamped & last_ego_rbt_vel,
                            const geometry_msgs::TwistStamped & last_ego_rbt_acc);

            Eigen::Vector4d update_ground_truth_cartesian_state();
            Eigen::Vector4d get_cartesian_state();
            Eigen::Vector4d get_GT_cartesian_state();

            Eigen::Vector4d get_frozen_cartesian_state();
            Eigen::Vector4d get_rewind_cartesian_state();
            Eigen::Vector4d get_modified_polar_state();
            Eigen::Vector4d get_frozen_modified_polar_state();
            Eigen::Vector4d get_rewind_modified_polar_state();

            Eigen::Vector2d get_x_tilde();

            geometry_msgs::TwistStamped get_v_ego();
            Eigen::Matrix<double, 4, 1> integrate();
            void linearize(int idx);
            void discretizeQ(int idx);

            void frozen_state_propagate(double dt);
            void rewind_propagate(double dt);
            void freeze_robot_vel();
            void set_rewind_state();

            void update(Eigen::Matrix<double, 2, 1> range_bearing_measurement, 
                                const std::vector<geometry_msgs::TwistStamped> & ego_rbt_vels_copied, 
                                const std::vector<geometry_msgs::TwistStamped> & ego_rbt_accs_copied, 
                                bool print,
                                std::vector<geometry_msgs::Pose> _agent_odoms,
                                std::vector<geometry_msgs::Vector3Stamped> _agent_vels,
                                const ros::Time & t_kf_update);

            void set_side(std::string _side);
            std::string get_side();
            int get_index();
            void inflate_model(float x, float y);

            void set_initialized(bool _initialized);
            bool get_initialized();
            void plot_states();
            void get_intermediate_vels_accs();
            void plot_models();
    };
}