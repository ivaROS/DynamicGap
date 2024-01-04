#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace dynamic_gap 
{
    class Estimator 
    {
        public:
            Eigen::Matrix<float, 2, 4> H_; // observation matrix
            Eigen::Matrix<float, 4, 2> H_transpose_;
            float R_scalar = 0.0, Q_scalar = 0.0;
            Eigen::Matrix2f R_k_; // measurement noise matrix
            Eigen::Matrix4f Q_k_; // covariance noise matrix
            Eigen::Matrix4f dQ_; // discretized covariance noise matrix

            Eigen::Vector4f x_hat_kmin1_plus_, x_hat_k_minus_, x_hat_k_plus_; // states before and after updates and integrations
            Eigen::Matrix4f P_kmin1_plus_, P_k_minus_, P_k_plus_; // covariance matrices before and after updates and integrations
            
            Eigen::Matrix<float, 4, 2> G_k_; // kalman gain
            Eigen::Vector2f x_tilde_, innovation_, residual_;

            Eigen::Matrix4f A_, STM_;
            
            int modelID_ = 0;
            std::string side_ = "";

            std::vector<geometry_msgs::TwistStamped> intermediateRbtVels_;
            std::vector<geometry_msgs::TwistStamped> intermediateRbtAccs_;        
            geometry_msgs::TwistStamped lastRbtVel_;
            geometry_msgs::TwistStamped lastRbtAcc_;    

            ros::Time t_last_update;

            Eigen::Matrix4f eyes;

            virtual void linearize(int idx) = 0;
            virtual void discretizeQ(int idx) = 0;
            virtual Eigen::Vector4f integrate() = 0;

            virtual void initialize(const std::string & side, const int & modelID, 
                                    const float & gapPtX, const float & gapPtY,
                                    const ros::Time & t_update, const geometry_msgs::TwistStamped & lastRbtVel,
                                    const geometry_msgs::TwistStamped & lastRbtAcc) = 0;
            // virtual void transfer(const int & placeholder) = 0;
            virtual void transfer(const Estimator & placeholder) = 0;

            virtual Eigen::Vector4f getState() = 0;
            virtual Eigen::Vector4f getTrueState() = 0;
            virtual geometry_msgs::TwistStamped getRobotVel() = 0;
            virtual void isolateGapDynamics() = 0;
            virtual Eigen::Vector4f getGapState() = 0;
            // virtual Eigen::Vector4f get_frozen_modified_polar_state() = 0;
            // virtual Eigen::Vector4f get_rewind_modified_polar_state() = 0;
            virtual Eigen::Vector4f getRewindGapState() = 0;
            virtual void setRewindState() = 0;

            virtual void gapStatePropagate(float dt) = 0;
            virtual void rewindPropagate(float dt) = 0;

            virtual Eigen::Vector2f get_x_tilde() = 0;
            virtual int getID() = 0;

            virtual void update(const Eigen::Vector2f & measurement, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs, 
                                const std::vector<geometry_msgs::Pose> & agentPoses,
                                const std::vector<geometry_msgs::Vector3Stamped> & agentVels,
                                const ros::Time & t_kf_update) = 0;
    };
}