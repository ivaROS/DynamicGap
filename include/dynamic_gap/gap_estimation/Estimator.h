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
                                const std::vector<geometry_msgs::Pose> & agentPoses,
                                const std::vector<geometry_msgs::Vector3Stamped> & agentVels,
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
                // ROS_INFO_STREAM_NAMED("GapEstimation", "   t_0 - tLastUpdate_ difference:" << (intermediateRbtVels_[0].header.stamp - tLastUpdate_).toSec() << " sec");

                // for (int i = 0; i < (intermediateRbtVels_.size() - 1); i++)
                // {
                //     ROS_INFO_STREAM_NAMED("GapEstimation", "   t_" << (i+1) << " - t_" << i << " difference: " << (intermediateRbtVels_[i + 1].header.stamp - intermediateRbtVels_[i].header.stamp).toSec() << " sec");
                // }

                // ROS_INFO_STREAM_NAMED("GapEstimation", "   tUpdate" << " - t_" << (intermediateRbtVels_.size() - 1) << " difference:" << (tUpdate - intermediateRbtVels_[intermediateRbtVels_.size() - 1].header.stamp).toSec() << " sec");

                // Tweaking ego robot velocities/acceleration to make sure that updates:
                //      1. Are never negative (backwards in time)
                //      2. Always start from time of last update
                //      3. Always at end at time of incoming laser scan measurement

                // Erasing odometry measurements that are from *before* the last update 
                while (!intermediateRbtVels_.empty() && tLastUpdate_ > intermediateRbtVels_[0].header.stamp)
                    intermediateRbtVels_.erase(intermediateRbtVels_.begin());

                while (!intermediateRbtAccs_.empty() && tLastUpdate_ > intermediateRbtAccs_[0].header.stamp)
                    intermediateRbtAccs_.erase(intermediateRbtAccs_.begin());

                // Inserting placeholder odometry to represent the time of the last update
                // intermediateRbtVels_.insert(intermediateRbtVels_.begin(), lastRbtVel_);
                // intermediateRbtVels_[0].header.stamp = tLastUpdate_;
                bool velValueAtScanTime = false;
                for (int i = 0; i < intermediateRbtVels_.size(); i++)
                {
                    if (intermediateRbtVels_.at(i).header.stamp == tLastUpdate_)
                    {
                        velValueAtScanTime = true;
                        break;
                    }
                }
                if (!velValueAtScanTime)
                {
                    intermediateRbtVels_.insert(intermediateRbtVels_.begin(), lastRbtVel_);
                    intermediateRbtVels_[0].header.stamp = tLastUpdate_;
                }

                // intermediateRbtAccs_.insert(intermediateRbtAccs_.begin(), lastRbtAcc_);
                // intermediateRbtAccs_[0].header.stamp = tLastUpdate_;
                bool accValueAtScanTime = false;
                for (int i = 0; i < intermediateRbtAccs_.size(); i++)
                {
                    if (intermediateRbtAccs_.at(i).header.stamp == tLastUpdate_)
                    {
                        accValueAtScanTime = true;
                        break;
                    }
                }
                if (!accValueAtScanTime)
                {
                    intermediateRbtAccs_.insert(intermediateRbtAccs_.begin(), lastRbtAcc_);
                    intermediateRbtAccs_[0].header.stamp = tLastUpdate_;
                }

                // Erasing odometry measurements that occur *after* the incoming laser scan was received
                while (!intermediateRbtVels_.empty() && tUpdate < intermediateRbtVels_[intermediateRbtVels_.size() - 1].header.stamp)
                    intermediateRbtVels_.erase(intermediateRbtVels_.end() - 1);

                while (!intermediateRbtAccs_.empty() && tUpdate < intermediateRbtAccs_[intermediateRbtAccs_.size() - 1].header.stamp)
                    intermediateRbtAccs_.erase(intermediateRbtAccs_.end() - 1);

                // Inserting placeholder odometry to represent the time that the incoming laser scan was received
                bool lastVelValueAtScanTime = false;
                for (int i = 0; i < intermediateRbtVels_.size(); i++)
                {
                    if (intermediateRbtVels_.at(i).header.stamp == tUpdate)
                    {
                        lastVelValueAtScanTime = true;
                        break;
                    }
                }
                if (!lastVelValueAtScanTime)
                {
                    if (!intermediateRbtVels_.empty())
                        intermediateRbtVels_.push_back(intermediateRbtVels_.back());
                    else
                        intermediateRbtVels_.push_back(lastRbtVel_);

                    intermediateRbtVels_.back().header.stamp = tUpdate;
                }
                
                bool lastAccValueAtScanTime = false;
                for (int i = 0; i < intermediateRbtAccs_.size(); i++)
                {
                    if (intermediateRbtAccs_.at(i).header.stamp == tUpdate)
                    {
                        lastAccValueAtScanTime = true;
                        break;
                    }
                }
                if (!lastAccValueAtScanTime)
                {
                    if (!intermediateRbtAccs_.empty())
                        intermediateRbtAccs_.push_back(intermediateRbtAccs_.back());
                    else
                        intermediateRbtAccs_.push_back(lastRbtAcc_);

                    intermediateRbtAccs_.back().header.stamp = tUpdate;
                }

                // Interpolating to make sure that vels and accs share all of the same time steps
                // std::vector<geometry_msgs::TwistStamped> testIntermediateRbtVels_ = intermediateRbtVels_;
                // std::vector<geometry_msgs::TwistStamped> testIntermediateRbtAccs_ = intermediateRbtAccs_;

                // ROS_INFO_STREAM("incoming testIntermediateRbtVels_: ");
                // for (int i = 0; i < testIntermediateRbtVels_.size(); i++)
                // {
                //     ROS_INFO_STREAM("   " << i << ": ");                
                //     ROS_INFO_STREAM("       header.stamp: " << testIntermediateRbtVels_[i].header.stamp);
                //     ROS_INFO_STREAM("       linear: ");
                //     ROS_INFO_STREAM("           x: " << testIntermediateRbtVels_[i].twist.linear.x);
                //     ROS_INFO_STREAM("           y: " << testIntermediateRbtVels_[i].twist.linear.y);                 
                // }

                // ROS_INFO_STREAM("incoming testIntermediateRbtAccs_: ");
                // for (int i = 0; i < testIntermediateRbtAccs_.size(); i++)
                // {
                //     ROS_INFO_STREAM("   " << i << ": ");                
                //     ROS_INFO_STREAM("       header.stamp: " << testIntermediateRbtAccs_[i].header.stamp);
                //     ROS_INFO_STREAM("       linear: ");
                //     ROS_INFO_STREAM("           x: " << testIntermediateRbtAccs_[i].twist.linear.x);
                //     ROS_INFO_STREAM("           y: " << testIntermediateRbtAccs_[i].twist.linear.y);
                // }

                if (intermediateRbtVels_.size() > 0 && intermediateRbtAccs_.size() > 0)
                    interpolateIntermediateValues(intermediateRbtVels_, intermediateRbtAccs_);
                
                if (intermediateRbtAccs_.size() > 0 && intermediateRbtVels_.size() > 0)
                    interpolateIntermediateValues(intermediateRbtAccs_, intermediateRbtVels_);

                // ROS_INFO_STREAM("outgoing testIntermediateRbtVels_: ");
                // for (int i = 0; i < testIntermediateRbtVels_.size(); i++)
                // {
                //     ROS_INFO_STREAM("   " << i << ": ");                
                //     ROS_INFO_STREAM("       header.stamp: " << testIntermediateRbtVels_[i].header.stamp);
                //     ROS_INFO_STREAM("       linear: ");
                //     ROS_INFO_STREAM("           x: " << testIntermediateRbtVels_[i].twist.linear.x);
                //     ROS_INFO_STREAM("           y: " << testIntermediateRbtVels_[i].twist.linear.y);                 
                // }

                // ROS_INFO_STREAM("outgoing testIntermediateRbtAccs_: ");
                // for (int i = 0; i < testIntermediateRbtAccs_.size(); i++)
                // {
                //     ROS_INFO_STREAM("   " << i << ": ");                
                //     ROS_INFO_STREAM("       header.stamp: " << testIntermediateRbtAccs_[i].header.stamp);
                //     ROS_INFO_STREAM("       linear: ");
                //     ROS_INFO_STREAM("           x: " << testIntermediateRbtAccs_[i].twist.linear.x);
                //     ROS_INFO_STREAM("           y: " << testIntermediateRbtAccs_[i].twist.linear.y);
                // }

                for (int i = 0; i < (intermediateRbtVels_.size() - 1); i++)
                {
                    float dt = (intermediateRbtVels_[i + 1].header.stamp - intermediateRbtVels_[i].header.stamp).toSec();
                    
                    // ROS_INFO_STREAM_NAMED("GapEstimation", "   t_" << (i+1) << " - t_" << i << " difference: " << dt << " sec");
                    
                    ROS_WARN_STREAM_COND_NAMED(dt < 0, "GapEstimation", "ERROR IN TIMESTEP CALCULATION, SHOULD NOT BE NEGATIVE");

                }

                // ROS_INFO_STREAM("intermediateRbtVels_: ");
                // for (int i = 0; i < intermediateRbtVels_.size(); i++)
                // {
                //     ROS_INFO_STREAM("   " << i << ": ");                
                //     ROS_INFO_STREAM("       header.stamp: " << intermediateRbtVels_[i].header.stamp);
                //     ROS_INFO_STREAM("       linear: ");
                //     ROS_INFO_STREAM("           x: " << intermediateRbtVels_[i].twist.linear.x);
                //     ROS_INFO_STREAM("           y: " << intermediateRbtVels_[i].twist.linear.y);                 
                // }

                // ROS_INFO_STREAM("intermediateRbtAccs_: ");
                // for (int i = 0; i < intermediateRbtAccs_.size(); i++)
                // {
                //     ROS_INFO_STREAM("   " << i << ": ");                
                //     ROS_INFO_STREAM("       header.stamp: " << intermediateRbtAccs_[i].header.stamp);
                //     ROS_INFO_STREAM("       linear: ");
                //     ROS_INFO_STREAM("           x: " << intermediateRbtAccs_[i].twist.linear.x);
                //     ROS_INFO_STREAM("           y: " << intermediateRbtAccs_[i].twist.linear.y);
                // }
            }

            void interpolateIntermediateValues(std::vector<geometry_msgs::TwistStamped> & vectorI,
                                               const std::vector<geometry_msgs::TwistStamped> & vectorJ)
            {
                // filling in velocities first
                for (int j = 0; j < vectorJ.size(); j++)
                {
                    // see if acceleration's timestamp is present
                    int timeJLowerBoundIthIdx = -1; // largest timestamp in i smaller than j
                    int timeJUpperBoundIthIdx = -1; // smallest timestamp in i bigger than j

                    int timeJIthIndex = -1;
                    for (int i = 0; i < vectorI.size(); i++)
                    {
                        if (vectorJ[j].header.stamp > 
                            vectorI[i].header.stamp)
                            timeJLowerBoundIthIdx = i;

                        if (timeJUpperBoundIthIdx < 0 && vectorJ[j].header.stamp < 
                                                         vectorI[i].header.stamp)
                            timeJUpperBoundIthIdx = i;

                        if (vectorJ[j].header.stamp == 
                            vectorI[i].header.stamp)
                        {
                            timeJIthIndex = i;
                            break;
                        }
                    }

                    if (timeJIthIndex > 0) // acceleration's timestamp is present, we don't need to do anything
                    {
                        // ROS_INFO_STREAM("       timeJIthIndex: " << timeJIthIndex);
                        continue;
                    } else
                    {
                        // ROS_INFO_STREAM("       timeJLowerBoundIthIdx: " << timeJLowerBoundIthIdx);
                        // ROS_INFO_STREAM("       timeJUpperBoundIthIdx: " << timeJUpperBoundIthIdx);

                        if (timeJLowerBoundIthIdx == -1 && timeJUpperBoundIthIdx == -1) // lower bound is -1 and upper bound is -1
                        {
                            //      only if one array is empty, should not happen
                            // ROS_INFO_STREAM("lower and upper bounds are -1, should not happen");
                        } else if (timeJLowerBoundIthIdx == -1 && timeJUpperBoundIthIdx >= 0) // lower bound is -1 and upper bound is >=0
                        {
                            // nothing in i is smaller than j, just re-add first element of i
                            vectorI.insert(vectorI.begin(), vectorI.at(0));
                            vectorI.at(0).header.stamp = vectorJ.at(j).header.stamp;
                        } else if (timeJLowerBoundIthIdx >= 0 && timeJUpperBoundIthIdx == -1) // lower bound is >=0 and upper bound is -1
                        {
                            // nothing in i is bigger than j, just re-add last element of i
                            vectorI.insert(vectorI.end(), vectorI.at(vectorI.size() - 1));
                            vectorI.at(vectorI.size() - 1).header.stamp = vectorJ.at(j).header.stamp;
                        } else if (timeJLowerBoundIthIdx >= 0 && timeJUpperBoundIthIdx >= 0) // lower bound is >=0 and upper bound is >=0
                        {
                            geometry_msgs::TwistStamped interpTwist;
                            interpTwist.header.frame_id = vectorJ.at(j).header.frame_id;
                            interpTwist.header.stamp = vectorJ.at(j).header.stamp;

                            float linear_x_diff = (vectorI.at(timeJUpperBoundIthIdx).twist.linear.x - vectorI.at(timeJLowerBoundIthIdx).twist.linear.x);
                            float linear_y_diff = (vectorI.at(timeJUpperBoundIthIdx).twist.linear.y - vectorI.at(timeJLowerBoundIthIdx).twist.linear.y);
                            float angular_z_diff = (vectorI.at(timeJUpperBoundIthIdx).twist.angular.z - vectorI.at(timeJLowerBoundIthIdx).twist.angular.z);

                            float time_diff_num = (interpTwist.header.stamp - vectorI.at(timeJLowerBoundIthIdx).header.stamp).toSec();
                            float time_diff_denom = (vectorI.at(timeJUpperBoundIthIdx).header.stamp - vectorI.at(timeJLowerBoundIthIdx).header.stamp).toSec();

                            interpTwist.twist.linear.x = vectorI.at(timeJLowerBoundIthIdx).twist.linear.x +
                                                            linear_x_diff * time_diff_num / time_diff_denom; 
                            interpTwist.twist.linear.y = vectorI.at(timeJLowerBoundIthIdx).twist.linear.y +
                                                            linear_y_diff * time_diff_num / time_diff_denom; 
                            interpTwist.twist.angular.z = vectorI.at(timeJLowerBoundIthIdx).twist.angular.z +
                                                            angular_z_diff * time_diff_num / time_diff_denom;

                            vectorI.insert(vectorI.begin() + timeJUpperBoundIthIdx, interpTwist);
                        }
                    }
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