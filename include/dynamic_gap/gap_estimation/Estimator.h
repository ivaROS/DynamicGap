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
            int modelID_ = 0; /**< model ID */
            std::string side_ = ""; /**< Gap side for model (left or right) */

            // Must be stored here for transfering models
            Eigen::Vector4f x_hat_kmin1_plus_; /**< State at step k-1 after correction */
            Eigen::Vector4f x_hat_k_minus_; /**< State at step k before correction */
            Eigen::Vector4f x_hat_k_plus_; /**< State at step k after correction */
            Eigen::Matrix4f P_kmin1_plus_; /**< Covariance matrix at step k-1 after correction */
            Eigen::Matrix4f P_k_minus_; /**< Covariance matrix at step k before correction */
            Eigen::Matrix4f P_k_plus_; /**< Covariance matrix at step k after correction */
            
            Eigen::Vector4f frozen_x; /**< Non-relative gap-only state */
            Eigen::Vector4f rewind_x;/**< Rewound non-relative gap-only state */

            Eigen::Matrix<float, 4, 2> G_k_; /**< Kalman gain at step k */
            Eigen::Vector2f xTilde_; /**< Current sensor measurement */

            std::vector<geometry_msgs::TwistStamped> intermediateRbtVels_; /**< sequence of ego-robot velocities received since last model update */
            std::vector<geometry_msgs::TwistStamped> intermediateRbtAccs_; /**< sequence of ego-robot accelerations received since last model update */     
            geometry_msgs::TwistStamped lastRbtVel_; /**< most recent ego-robot velocity from last model update */
            geometry_msgs::TwistStamped lastRbtAcc_; /**< most recent ego-robot acceleration from last model update */   

            ros::Time tLastUpdate_; /**< time of last model update */

            /**
            * \brief Virtual function for initializing estimator, must be overridden by desired model class
            *
            * \param side Gap side for model (left or right)
            * \param modelID ID for model
            * \param gapPtX x-coordinate of initial sensor measurement for model
            * \param gapPtY y-coordinate of initial sensor measurement for model
            * \param tUpdate time of current model update
            * \param lastRbtVel most recent ego-robot velocity from last model update
            * \param lastRbtAcc most recent ego-robot acceleration from last model update
            * \return N/A
            */
            virtual void initialize(const std::string & side, const int & modelID, 
                                    const float & gapPtX, const float & gapPtY,
                                    const ros::Time & tUpdate, const geometry_msgs::TwistStamped & lastRbtVel,
                                    const geometry_msgs::TwistStamped & lastRbtAcc) = 0;
            
            /**
            * \brief Virtual function for transferring incoming model to this model
            *
            * \param incomingModel incoming model from which we must transfer information to this model
            * \return N/A
            */            
            virtual void transfer(const Estimator & incomingModel) = 0;

            /**
            * \brief Virtual function for updating estimator based on new sensor measurements
            *
            * \param measurement new sensor measurement
		    * \param intermediateRbtVels sequence of ego-robot velocities received since last model update
		    * \param intermediateRbtAccs sequence of ego-robot accelerations received since last model update            
            * \param agentPoses poses of all agents in environment (ground truth information used for certain estimator classes)
            * \param agentVels velocities of all agents in environment (ground truth information used for certain estimator classes)
            * \param tUpdate time of current model update
            * \return N/A
            */
            virtual void update(const Eigen::Vector2f & measurement, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs, 
                                const std::vector<geometry_msgs::Pose> & agentPoses,
                                const std::vector<geometry_msgs::Vector3Stamped> & agentVels,
                                const ros::Time & tUpdate) = 0;

            /**
            * \brief Getter function for relative estimator state
            *
            * \return relative (gap-robot) estimator state
            */                 
            Eigen::Vector4f getState() { return x_hat_k_plus_; };

            /**
            * \brief Getter function for non-relative estimator state
            *
            * \return non-relative (gap) estimator state
            */               
            Eigen::Vector4f getGapState() { return frozen_x; };

            /**
            * \brief Getter function for rewound non-relative estimator state
            *
            * \return rewound non-relative (gap) estimator state
            */  
            Eigen::Vector4f getRewindGapState() { return rewind_x; };

            /**
            * \brief Getter function for robot velocity
            *
            * \return robot velocity
            */ 
            geometry_msgs::TwistStamped getRobotVel() { return lastRbtVel_; };

            /**
            * \brief Getter function for model ID
            *
            * \return robot velocity
            */ 
            int getID() { return modelID_; };

            /**
            * \brief Getter function for sensor measurement
            *
            * \return sensor measurement
            */ 
            Eigen::Vector2f getXTilde() { return xTilde_; };

            /**
            * \brief Function for subtracting off ego-robot velocity to obtain gap-only dynamical state
            *
            * \return N/A
            */ 
            void isolateGapDynamics()
            {
                frozen_x = getState();
        
                // fixing position (otherwise can get bugs)
                frozen_x[0] = xTilde_[0];
                frozen_x[1] = xTilde_[1];

                // update cartesian
                frozen_x[2] += lastRbtVel_.twist.linear.x;
                frozen_x[3] += lastRbtVel_.twist.linear.y;
            }

            /**
            * \brief Function for initializing rewound gap-only state
            *
            * \return N/A
            */ 
            void setRewindState()
            {
                rewind_x = frozen_x;
            }

            /**
            * \brief Function for propagating gap-only state forward in time
            *
            * \param froz_dt timestep to propagate forward
            * \return N/A
            */ 
            void gapStatePropagate(const float & froz_dt)
            {
                Eigen::Vector4f new_frozen_x;     
                new_frozen_x << 0.0, 0.0, 0.0, 0.0;

                Eigen::Vector2f frozen_linear_acc_ego(0.0, 0.0);

                Eigen::Vector2f frozen_linear_vel_ego(0.0, 0.0); 
                float frozen_ang_vel_ego = 0.0;

                float vdot_x_body = frozen_linear_acc_ego[0];
                float vdot_y_body = frozen_linear_acc_ego[1];

                // discrete euler update of state (ignoring rbt acceleration, set as 0)
                new_frozen_x[0] = frozen_x[0] + (frozen_x[2] + frozen_x[1]*frozen_ang_vel_ego)*froz_dt;
                new_frozen_x[1] = frozen_x[1] + (frozen_x[3] - frozen_x[0]*frozen_ang_vel_ego)*froz_dt;
                new_frozen_x[2] = frozen_x[2] + (frozen_x[3]*frozen_ang_vel_ego - vdot_x_body)*froz_dt;
                new_frozen_x[3] = frozen_x[3] + (-frozen_x[2]*frozen_ang_vel_ego - vdot_y_body)*froz_dt;
                frozen_x = new_frozen_x;                 
            }

            /**
            * \brief Function for propagating gap-only state backward in time
            *
            * \param rew_dt timestep to propagate backward
            * \return N/A
            */ 
            void rewindPropagate(const float & rew_dt)
            {
                Eigen::Vector4f new_rewind_x;     
                new_rewind_x << 0.0, 0.0, 0.0, 0.0;

                Eigen::Vector2f frozen_linear_acc_ego(0.0, 0.0);

                Eigen::Vector2f frozen_linear_vel_ego(0.0, 0.0); 
                float frozen_ang_vel_ego = 0.0;

                float vdot_x_body = frozen_linear_acc_ego[0];
                float vdot_y_body = frozen_linear_acc_ego[1];

                // discrete euler update of state (ignoring rbt acceleration, set as 0)
                new_rewind_x[0] = rewind_x[0] + (rewind_x[2] + rewind_x[1]*frozen_ang_vel_ego)*rew_dt;
                new_rewind_x[1] = rewind_x[1] + (rewind_x[3] - rewind_x[0]*frozen_ang_vel_ego)*rew_dt;
                new_rewind_x[2] = rewind_x[2] + (rewind_x[3]*frozen_ang_vel_ego - vdot_x_body)*rew_dt;
                new_rewind_x[3] = rewind_x[3] + (-rewind_x[2]*frozen_ang_vel_ego - vdot_y_body)*rew_dt;
                rewind_x = new_rewind_x; 
            }

    };
}