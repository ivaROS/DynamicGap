#pragma once

#include <ros/ros.h>

#include <map>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace dynamic_gap 
{
    /**
    * \brief Base class for gap point estimator
    */
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
            
            Eigen::Vector4f xFrozen_; /**< Non-relative gap-only state */
            Eigen::Vector4f xRewind_;/**< Rewound non-relative gap-only state */

            Eigen::Matrix<float, 4, 2> G_k_; /**< Kalman gain at step k */
            Eigen::Vector2f xTilde_; /**< Current sensor measurement */

            std::vector<geometry_msgs::TwistStamped> intermediateRbtVels_; /**< sequence of ego-robot velocities received since last model update */
            std::vector<geometry_msgs::TwistStamped> intermediateRbtAccs_; /**< sequence of ego-robot accelerations received since last model update */     
            geometry_msgs::TwistStamped lastRbtVel_; /**< most recent ego-robot velocity from last model update */
            geometry_msgs::TwistStamped lastRbtAcc_; /**< most recent ego-robot acceleration from last model update */   

            ros::Time tStart_; /**< time of model initialization */
            ros::Time tLastUpdate_; /**< time of last model update */

            bool manip_ = false;
            Eigen::Vector2f manipPosition;

            /**
            * \brief Virtual function for initializing estimator, must be overridden by desired model class
            * \param side Gap side for model (left or right)
            * \param modelID ID for model
            * \param gapPtX x-coordinate of initial sensor measurement for model
            * \param gapPtY y-coordinate of initial sensor measurement for model
            * \param tUpdate time of current model update
            * \param lastRbtVel most recent ego-robot velocity from last model update
            * \param lastRbtAcc most recent ego-robot acceleration from last model update
            */
            virtual void initialize(const std::string & side, const int & modelID, 
                                    const float & gapPtX, const float & gapPtY,
                                    const ros::Time & tUpdate, const geometry_msgs::TwistStamped & lastRbtVel,
                                    const geometry_msgs::TwistStamped & lastRbtAcc) = 0;
            
            /**
            * \brief Virtual function for transferring incoming model to this model
            * \param incomingModel incoming model from which we must transfer information to this model
            */            
            virtual void transfer(const Estimator & incomingModel) = 0;

            /**
            * \brief Virtual function for updating estimator based on new sensor measurements
            * \param measurement new sensor measurement
		    * \param intermediateRbtVels sequence of ego-robot velocities received since last model update
		    * \param intermediateRbtAccs sequence of ego-robot accelerations received since last model update            
            * \param agentPoses poses of all agents in environment (ground truth information used for certain estimator classes)
            * \param agentVels velocities of all agents in environment (ground truth information used for certain estimator classes)
            * \param tUpdate time of current model update
            */
            virtual void update(const Eigen::Vector2f & measurement, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtVels, 
                                const std::vector<geometry_msgs::TwistStamped> & intermediateRbtAccs, 
                                const std::map<std::string, geometry_msgs::Pose> & agentPoses,
                                const std::map<std::string, geometry_msgs::Vector3Stamped> & agentVels,
                                const ros::Time & tUpdate) = 0;

            /**
            * \brief Sequence intermediate robot velocities and accelerations so that intermediate model
            * updates start from time of last update and end at time of incoming sensor measurement
            * \param tUpdate time of current model update
            */
            void processEgoRobotVelsAndAccs(const ros::Time & tUpdate)
            {
                /*                */

                // Printing original dt values from intermediate odom measurements
                ROS_INFO_STREAM_NAMED("GapEstimation", "   t_0 - tLastUpdate_ difference:" << (intermediateRbtVels_[0].header.stamp - tLastUpdate_).toSec() << " sec");

                for (int i = 0; i < (intermediateRbtVels_.size() - 1); i++)
                {
                    ROS_INFO_STREAM_NAMED("GapEstimation", "   t_" << (i+1) << " - t_" << i << " difference: " << (intermediateRbtVels_[i + 1].header.stamp - intermediateRbtVels_[i].header.stamp).toSec() << " sec");
                }

                ROS_INFO_STREAM_NAMED("GapEstimation", "   tUpdate" << " - t_" << (intermediateRbtVels_.size() - 1) << " difference:" << (tUpdate - intermediateRbtVels_[intermediateRbtVels_.size() - 1].header.stamp).toSec() << " sec");

                // Tweaking ego robot velocities/acceleration to make sure that updates:
                //      1. Are never negative (backwards in time)
                //      2. Always start from time of last update
                //      3. Always at end at time of incoming laser scan measurement

                // Erasing odometry measurements that are from *before* the last update 
                while (!intermediateRbtVels_.empty() && tLastUpdate_ > intermediateRbtVels_[0].header.stamp)
                {
                    intermediateRbtVels_.erase(intermediateRbtVels_.begin());
                    intermediateRbtAccs_.erase(intermediateRbtAccs_.begin());
                }

                // Inserting placeholder odometry to represent the time of the last update
                intermediateRbtVels_.insert(intermediateRbtVels_.begin(), lastRbtVel_);
                intermediateRbtVels_[0].header.stamp = tLastUpdate_;

                intermediateRbtAccs_.insert(intermediateRbtAccs_.begin(), lastRbtAcc_);
                intermediateRbtAccs_[0].header.stamp = tLastUpdate_;

                // Erasing odometry measurements that occur *after* the incoming laser scan was received
                while (!intermediateRbtVels_.empty() && tUpdate < intermediateRbtVels_[intermediateRbtVels_.size() - 1].header.stamp)
                {
                    intermediateRbtVels_.erase(intermediateRbtVels_.end() - 1);
                    intermediateRbtAccs_.erase(intermediateRbtAccs_.end() - 1);
                }

                // Inserting placeholder odometry to represent the time that the incoming laser scan was received
                intermediateRbtVels_.push_back(intermediateRbtVels_.back());
                intermediateRbtVels_.back().header.stamp = tUpdate;

                intermediateRbtAccs_.push_back(intermediateRbtAccs_.back());
                intermediateRbtAccs_.back().header.stamp = tUpdate;

                for (int i = 0; i < (intermediateRbtVels_.size() - 1); i++)
                {
                    float dt = (intermediateRbtVels_[i + 1].header.stamp - intermediateRbtVels_[i].header.stamp).toSec();
                    
                    // ROS_INFO_STREAM_NAMED("GapEstimation", "   t_" << (i+1) << " - t_" << i << " difference: " << dt << " sec");
                    
                    ROS_WARN_STREAM_COND_NAMED(dt < 0, "GapEstimation", "ERROR IN TIMESTEP CALCULATION, SHOULD NOT BE NEGATIVE");

                }

                ROS_INFO_STREAM("intermediateRbtVels_: ");
                for (int i = 0; i < intermediateRbtVels_.size(); i++)
                {
                    ROS_INFO_STREAM("   " << i << ": ");                
                    ROS_INFO_STREAM("       header.stamp: " << intermediateRbtVels_[i].header.stamp);
                    ROS_INFO_STREAM("       header.frame_id: " << intermediateRbtVels_[i].header.frame_id);
                    ROS_INFO_STREAM("       linear: ");
                    ROS_INFO_STREAM("           x: " << intermediateRbtVels_[i].twist.linear.x);
                    ROS_INFO_STREAM("           y: " << intermediateRbtVels_[i].twist.linear.y);                 
                }

                ROS_INFO_STREAM("intermediateRbtAccs_: ");
                for (int i = 0; i < intermediateRbtAccs_.size(); i++)
                {
                    ROS_INFO_STREAM("   " << i << ": ");                
                    ROS_INFO_STREAM("       header.stamp: " << intermediateRbtAccs_[i].header.stamp);
                    ROS_INFO_STREAM("       header.frame_id: " << intermediateRbtAccs_[i].header.frame_id);
                    ROS_INFO_STREAM("       linear: ");
                    ROS_INFO_STREAM("           x: " << intermediateRbtAccs_[i].twist.linear.x);
                    ROS_INFO_STREAM("           y: " << intermediateRbtAccs_[i].twist.linear.y);
                }
            }

            /**
            * \brief Virtual getter function for relative estimator state
            * \return relative (gap-robot) estimator state
            */                 
            virtual Eigen::Vector4f getState() = 0;

            /**
            * \brief Getter function for non-relative estimator state. Should be run AFTER isolateGapDynamics().
            * \return non-relative (gap) estimator state
            */               
            Eigen::Vector4f getGapState() { return xFrozen_; };

            /**
            * \brief Getter function for non-relative estimator position. Should be run AFTER isolateGapDynamics().
            * \return non-relative (gap) estimator position
            */               
            Eigen::Vector2f getGapPosition() { return xFrozen_.head(2); };

            /**
            * \brief Getter function for non-relative estimator bearing. Should be run AFTER isolateGapDynamics().
            * \return non-relative (gap) estimator bearing
            */               
            float getGapBearing() { Eigen::Vector2f gapPosition = xFrozen_.head(2); 
                                    float gapTheta = std::atan2(gapPosition[1], gapPosition[0]);
                                    return gapTheta; };

            /**
            * \brief Getter function for non-relative estimator velocity. Should be run AFTER isolateGapDynamics().
            * \return non-relative (gap) estimator velocity
            */               
            Eigen::Vector2f getGapVelocity() { return xFrozen_.tail(2); };

            /**
            * \brief Getter function for rewound non-relative estimator state
            * \return rewound non-relative (gap) estimator state
            */  
            Eigen::Vector4f getRewindGapState() { return xRewind_; };

            /**
            * \brief Getter function for robot velocity
            * \return robot velocity
            */ 
            geometry_msgs::TwistStamped getRobotVel() { return lastRbtVel_; };

            /**
            * \brief Getter function for model ID
            * \return robot velocity
            */ 
            int getID() { return modelID_; };

            /**
            * \brief Getter function for sensor measurement
            * \return sensor measurement
            */ 
            Eigen::Vector2f getXTilde() { return xTilde_; };

            /**
            * \brief Function for subtracting off ego-robot velocity to obtain gap-only dynamical state
            */ 
            void isolateGapDynamics()
            {
                xFrozen_ = getState();
        
                // fixing position (otherwise can get bugs)
                xFrozen_[0] = xTilde_[0];
                xFrozen_[1] = xTilde_[1];

                // update cartesian
                xFrozen_[2] += lastRbtVel_.twist.linear.x;
                xFrozen_[3] += lastRbtVel_.twist.linear.y;
            }

            /**
            * \brief Function for initializing rewound gap-only state
            */ 
            void setRewindState()
            {
                xRewind_ = xFrozen_;
            }

            /**
            * \brief Function for propagating gap-only state forward in time
            * \param dt timestep to propagate forward
            */ 
            void gapStatePropagate(const float & dt)
            {
                Eigen::Vector4f xFrozenProp_;     
                xFrozenProp_ << 0.0, 0.0, 0.0, 0.0;

                Eigen::Vector2f egoLinVel(0.0, 0.0); 
                float egoAngVel = 0.0;
                Eigen::Vector2f egoLinAcc(0.0, 0.0); // "frozen" robot
                Eigen::Vector2f gapLinAcc(0.0, 0.0); // constant velocity assumption

                Eigen::Vector2f relLinAcc = gapLinAcc - egoLinAcc;

                // discrete euler update of state (ignoring rbt acceleration, set as 0)
                xFrozenProp_[0] = xFrozen_[0] + (xFrozen_[2] + xFrozen_[1]*egoAngVel)*dt;
                xFrozenProp_[1] = xFrozen_[1] + (xFrozen_[3] - xFrozen_[0]*egoAngVel)*dt;
                xFrozenProp_[2] = xFrozen_[2] + (relLinAcc[0] + xFrozen_[3]*egoAngVel)*dt;
                xFrozenProp_[3] = xFrozen_[3] + (relLinAcc[1] - xFrozen_[2]*egoAngVel)*dt;
                xFrozen_ = xFrozenProp_;                 
            }

            /**
            * \brief Function for propagating gap-only state backward in time
            * \param dt timestep to propagate backward
            */ 
            void rewindPropagate(const float & dt)
            {
                Eigen::Vector4f xRewindProp_;     
                xRewindProp_ << 0.0, 0.0, 0.0, 0.0;

                Eigen::Vector2f egoLinVel(0.0, 0.0); 
                float egoAngVel = 0.0;
                Eigen::Vector2f egoLinAcc(0.0, 0.0); // "frozen" robot
                Eigen::Vector2f gapLinAcc(0.0, 0.0); // constant velocity assumption

                Eigen::Vector2f relLinAcc = gapLinAcc - egoLinAcc;

                // discrete euler update of state (ignoring rbt acceleration, set as 0)
                xRewindProp_[0] = xRewind_[0] + (xRewind_[2] + xRewind_[1]*egoAngVel)*dt;
                xRewindProp_[1] = xRewind_[1] + (xRewind_[3] - xRewind_[0]*egoAngVel)*dt;
                xRewindProp_[2] = xRewind_[2] + (relLinAcc[0] + xRewind_[3]*egoAngVel)*dt;
                xRewindProp_[3] = xRewind_[3] + (relLinAcc[1] - xRewind_[2]*egoAngVel)*dt;
                xRewind_ = xRewindProp_; 
            }

            void setManip() { manip_ = true; }
            void setNewPosition(const float & newTheta, const float & newRange) 
            { 
                manipPosition << newRange * std::cos(newTheta), 
                                 newRange * std::sin(newTheta); 
            }

    };
}