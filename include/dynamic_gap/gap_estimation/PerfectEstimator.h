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
    class PerfectEstimator : public Estimator 
    {
        private:
            void processEgoRobotVelsAndAccs(const ros::Time & t_update);
            
            Eigen::Matrix4f Q_1, Q_2, Q_3; // covariance noise matrix
            float R_scalar, Q_scalar;

            Eigen::Matrix2f tmp_mat; //  place holder for inverse

            Eigen::Vector4f frozen_x, rewind_x;
            Eigen::Matrix4f P_intermediate, new_P; // covariance matrix

            std::vector< std::vector<float>> previous_states, previous_measurements, previous_measurements_gap_only,
                                              previous_ego_accels, previous_ego_vels, previous_times,
                                              previous_gap_only_states, vel_euler_derivatives;
            Eigen::Matrix4f eyes;
            std::string plot_dir;

            std::vector<geometry_msgs::Pose> agentPoses_;
            std::vector<geometry_msgs::Vector3Stamped> agentVels_;

            ros::Time t_last_update;
            std::vector<geometry_msgs::TwistStamped> intermediateRbtVels_;
            std::vector<geometry_msgs::TwistStamped> intermediateRbtAccs_;        
            geometry_msgs::TwistStamped lastRbtVel_;
            geometry_msgs::TwistStamped lastRbtAcc_;

        public:

            PerfectEstimator();

            void initialize(const std::string & side, const int & modelID, 
                            const float & gapPtX, const float & gapPtY,
                            const ros::Time & t_update, const geometry_msgs::TwistStamped & lastRbtVel,
                            const geometry_msgs::TwistStamped & lastRbtAcc);
            void transfer(const Estimator & placeholder);

            Eigen::Vector4f update_ground_truth_cartesian_state();
            Eigen::Vector4f getState();
            // Eigen::Vector4f getTrueState();

            Eigen::Vector4f getGapState();
            Eigen::Vector4f getRewindGapState();
            // Eigen::Vector4f get_modified_polar_state();
            // Eigen::Vector4f get_frozen_modified_polar_state();
            // Eigen::Vector4f get_rewind_modified_polar_state();

            Eigen::Vector2f get_x_tilde();

            geometry_msgs::TwistStamped getRobotVel();
            Eigen::Vector4f integrate();
            void linearize(const int & idx);
            void discretizeQ(const int & idx);

            void gapStatePropagate(const float & dt);
            void rewindPropagate(const float & dt);
            void isolateGapDynamics();
            void setRewindState();

            void update(const Eigen::Vector2f & measurement, 
                        const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                        const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs, 
                        const std::vector<geometry_msgs::Pose> & agentPoses,
                        const std::vector<geometry_msgs::Vector3Stamped> & agentVels,
                        const ros::Time & t_kf_update);
            int getID();

    };
}