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
    class StaticEstimator : public Estimator 
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

            std::vector<geometry_msgs::Pose> agentPoses_;
            std::vector<geometry_msgs::Vector3Stamped> agentVels_;

            bool perfect;
            bool print;
            bool plot;
            bool plotted;

            ros::Time t_last_update;
            std::vector<geometry_msgs::TwistStamped> intermediateRbtVels_;
            std::vector<geometry_msgs::TwistStamped> intermediateRbtAccs_;        
            geometry_msgs::TwistStamped lastRbtVel_;
            geometry_msgs::TwistStamped lastRbtAcc_;

        public:

            StaticEstimator(std::string, int, float, float,const ros::Time & t_update,
                        const geometry_msgs::TwistStamped & lastRbtVel,
                        const geometry_msgs::TwistStamped & lastRbtAcc);

            void initialize(float, float, const ros::Time & t_update,
                            const geometry_msgs::TwistStamped & lastRbtVel,
                            const geometry_msgs::TwistStamped & lastRbtAcc);

            Eigen::Vector4f update_ground_truth_cartesian_state();
            Eigen::Vector4f getState();
            Eigen::Vector4f getTrueState();

            Eigen::Vector4f getGapState();
            Eigen::Vector4f getRewindGapState();
            // Eigen::Vector4f get_modified_polar_state();
            // Eigen::Vector4f get_frozen_modified_polar_state();
            // Eigen::Vector4f get_rewind_modified_polar_state();

            Eigen::Vector2f get_x_tilde();

            geometry_msgs::TwistStamped getRobotVel();
            Eigen::Matrix<float, 4, 1> integrate();
            void linearize(int idx);
            void discretizeQ(int idx);

            void gapStatePropagate(float dt);
            void rewindPropagate(float dt);
            void isolateGapDynamics();
            void setRewindState();

            void update(Eigen::Matrix<float, 2, 1> range_bearing_measurement, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs, 
                                bool print,
                                const std::vector<geometry_msgs::Pose> & agentPoses,
                                const std::vector<geometry_msgs::Vector3Stamped> & agentVels,
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