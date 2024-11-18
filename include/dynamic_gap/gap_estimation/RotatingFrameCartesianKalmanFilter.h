#pragma once

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <dynamic_gap/config/DynamicGapConfig.h>
// #include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
// #include <sensor_msgs/Imu.h>
// #include <tf2_ros/buffer.h>
// #include <sensor_msgs/LaserScan.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <limits>

#include <dynamic_gap/gap_estimation/Estimator.h>

// #include <random>


// using namespace Eigen;

namespace dynamic_gap 
{
    /**
    * \brief Extended Kalman filter model that uses a constant velocity rotating Cartesian frame dynamics model
    */
    class RotatingFrameCartesianKalmanFilter : public Estimator 
    {
        private:
            Eigen::Matrix<float, 2, 4> H_; /**< Observation matrix */
            Eigen::Matrix<float, 4, 2> H_transpose_; /**< Transposed observation matrix */

            Eigen::Matrix2f R_k_; /**<  Measurement noise matrix */
            Eigen::Matrix4f Q_k_; /**< Covariance noise matrix */
            Eigen::Matrix4f dQ_; /**< Discretized covariance noise matrix */

            Eigen::Matrix4f Q_1_; /**< 1st order approximation of discretized covariance noise matrix */
            Eigen::Matrix4f Q_2_; /**< 2nd order approximation of discretized covariance noise matrix */ 
            Eigen::Matrix4f Q_3_; /**< 3rd order approximation of discretized covariance noise matrix */

            float R_scalar = 0.0; /**< Scalar value used to populate R matrix*/
            float Q_scalar = 0.0; /**< Scalar value used to populate Q matrix*/

            Eigen::Matrix4f A_; /**< Continuous form of autonomous dynamics term */
            Eigen::Matrix4f STM_; /**< Discrete form of autonomous dynamics term */

            Eigen::Matrix4f eyes; /**< 4x4 identity matrix */

            Eigen::Vector2f innovation_; /**< innovation term from Kalman filter update loop */
            Eigen::Vector2f residual_; /**< residual term from Kalman filter update loop */

            Eigen::Matrix2f tmp_mat; /**< place holder for inverse calculations */

            // Eigen::Vector4f frozen_x, rewind_x;
            Eigen::Matrix4f P_intermediate; /**< placeholding variable for covariance matrix during updates */
            Eigen::Matrix4f new_P; /**< placeholding variable for covariance matrix during updates */

            double lifetimeThreshold_ = 3.0; /**< Threshold in seconds that gap model must exist for before we trust and use state */

        public:

            RotatingFrameCartesianKalmanFilter();

            void initialize(const std::string & side, const int & modelID, 
                            const float & gapPtX, const float & gapPtY,
                            const ros::Time & t_update, const geometry_msgs::TwistStamped & lastRbtVel,
                            const geometry_msgs::TwistStamped & lastRbtAcc);
            void transfer(const Estimator & placeholder);

            void update(const Eigen::Vector2f & measurement,
                        const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                        const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs, 
                        const std::vector<geometry_msgs::Pose> & agentPoses,
                        const std::vector<geometry_msgs::Vector3Stamped> & agentVels,
                        const ros::Time & t_kf_update);

            /**
            * \brief Helper function for integrating estimator state forward in time
            * \return Propagated estimator state
            */     
            Eigen::Vector4f integrate();

            /**
            * \brief Helper function for linearizing nonlinear estimator dynamics
            * \param idx index of intermediate update to linearize for 
            */                 
            void linearize(const int & idx);

            /**
            * \brief Helper function for discretizing continuous form of covariance noise matrix
            * \param idx index of intermediate update to discretize for 
            */                 
            void discretizeQ(const int & idx); 

            /**
            * \brief Getter function for relative estimator state
            * \return relative (gap-robot) estimator state
            */     
            Eigen::Vector4f getState();
     };
}