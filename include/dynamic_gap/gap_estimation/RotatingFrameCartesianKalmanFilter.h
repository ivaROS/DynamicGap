#pragma once

#include <ros/ros.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
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

#include <dynamic_gap/gap_estimation/Estimator.h>

// #include <random>


// using namespace Eigen;

namespace dynamic_gap 
{
    class RotatingFrameCartesianKalmanFilter : public Estimator 
    {
        private:
            void processEgoRobotVelsAndAccs(const ros::Time & t_update);
            
            Eigen::Matrix4f Q_1, Q_2, Q_3; // covariance noise matrix
            float R_scalar, Q_scalar;

            Eigen::Matrix2f tmp_mat; //  place holder for inverse

            Eigen::Vector4f x_ground_truth, x_ground_truth_gap_only, frozen_x, rewind_x;
            Eigen::Matrix4f P_intermediate, new_P; // covariance matrix

            std::string side;
            int index;

            float life_time, start_time;

            std::vector< std::vector<float>> previous_states, previous_measurements, previous_measurements_gap_only,
                                              previous_ego_accels, previous_ego_vels, previous_times,
                                              previous_gap_only_states, vel_euler_derivatives;
            float life_time_threshold;
            Eigen::Matrix4f eyes;
            std::string plot_dir;

            std::vector<geometry_msgs::Pose> agent_odoms;
            std::vector<geometry_msgs::Vector3Stamped> agent_vels;

            bool perfect;
            bool print;
            bool plot;
            bool plotted;

            ros::Time t_last_update;
            std::vector<geometry_msgs::TwistStamped> ego_rbt_vels;
            std::vector<geometry_msgs::TwistStamped> ego_rbt_accs;        
            geometry_msgs::TwistStamped last_ego_rbt_vel;
            geometry_msgs::TwistStamped last_ego_rbt_acc;

        public:

            RotatingFrameCartesianKalmanFilter(std::string, int, float, float,const ros::Time & t_update,
                        const geometry_msgs::TwistStamped & last_ego_rbt_vel,
                        const geometry_msgs::TwistStamped & last_ego_rbt_acc);

            void initialize(float, float, const ros::Time & t_update,
                            const geometry_msgs::TwistStamped & last_ego_rbt_vel,
                            const geometry_msgs::TwistStamped & last_ego_rbt_acc);

            Eigen::Vector4f update_ground_truth_cartesian_state();
            Eigen::Vector4f getState();
            Eigen::Vector4f getTrueState();
            
            Eigen::Vector4f getGapState();
            Eigen::Vector4f get_rewind_cartesian_state();
            Eigen::Vector4f get_modified_polar_state();
            Eigen::Vector4f get_frozen_modified_polar_state();
            Eigen::Vector4f get_rewind_modified_polar_state();

            Eigen::Vector2f get_x_tilde();

            geometry_msgs::TwistStamped getRobotVel();
            Eigen::Matrix<float, 4, 1> integrate();
            void linearize(int idx);
            void discretizeQ(int idx);

            void frozen_state_propagate(float dt);
            void rewind_propagate(float dt);
            void isolateGapDynamics();
            void set_rewind_state();

            void update(Eigen::Matrix<float, 2, 1> range_bearing_measurement, 
                        const std::vector<geometry_msgs::TwistStamped> & ego_rbt_vels_copied, 
                        const std::vector<geometry_msgs::TwistStamped> & ego_rbt_accs_copied, 
                        bool print,
                        const std::vector<geometry_msgs::Pose> & _agent_odoms,
                        const std::vector<geometry_msgs::Vector3Stamped> & _agent_vels,
                        const ros::Time & t_kf_update);

            int get_index();
            void inflate_model(float x, float y);

            void set_initialized(bool _initialized);
            bool get_initialized();
            void plot_states();
            void get_intermediate_vels_accs();
            void plot_models();
    };
}