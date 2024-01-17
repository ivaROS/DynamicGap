#pragma once

#include <ros/ros.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
// #include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include "dynamic_gap/TrajPlan.h"
#include <dynamic_gap/trajectory_generation/GapTrajectoryGenerator.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

namespace dynamic_gap 
{
    class TrajectoryController 
    {
        public:

            TrajectoryController(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);

            /**
            * \brief receive new laser scan and update member variable accordingly
            * \param scan new laser scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);
            
            /**
            * \brief Control law for pure obstacle avoidance
            * \return command velocity for robot
            */
            geometry_msgs::Twist obstacleAvoidanceControlLaw(); // const sensor_msgs::LaserScan & scan
            
            /**
            * \brief Control law for manual robot operation
            * \return command velocity for robot
            */
            geometry_msgs::Twist manualControlLaw();

            /**
            * \brief Control law for trajectory tracking
            * \param current current robot pose
            * \param desired desired robot pose
            * \param currentPeakSplineVel peak spline velocity for current trajectory we are trying to track
            * \return command velocity for robot
            */
            geometry_msgs::Twist controlLaw(const geometry_msgs::Pose & current, 
                                            const geometry_msgs::Pose & desired,
                                            const geometry_msgs::TwistStamped & currentPeakSplineVel); // const sensor_msgs::LaserScan & scan, 

            /**
            * \brief Apply post-processing steps to command velocity including robot kinematic limits
            * along with last-resort safety modules such as projection operator or CBF
            * \param rawCmdVel raw command velocity
            * \param rbtPoseInSensorFrame robot pose in sensor frame
            * \param currGapLeftPtModel current gap's estimator for left gap point
            * \param currGapRightPtModel current gap's estimator for right gap point
            * \param currRbtVel current robot velocity
            * \param currRbtAcc current robot acceleration
            * \return processed command velocity
            */
            geometry_msgs::Twist processCmdVel(const geometry_msgs::Twist & rawCmdVel,
                                                const geometry_msgs::PoseStamped & rbtPoseInSensorFrame, 
                                                const dynamic_gap::Estimator * currGapLeftPtModel,
                                                const dynamic_gap::Estimator * currGapRightPtModel, 
                                                const geometry_msgs::TwistStamped & currRbtVel, 
                                                const geometry_msgs::TwistStamped & currRbtAcc); // const sensor_msgs::LaserScan & scan, 
            
            /**
            * \brief Extract pose within target trajectory that we should track
            * \param currPose current robot pose
            * \param localTrajectory selected local trajectory to track
            * \return index along local trajectory for which pose the robot should drive towards
            */
            int extractTargetPoseIdx(const geometry_msgs::Pose & currPose, 
                                     const geometry_msgs::PoseArray & localTrajectory);
            
        private:
            /**
            * \brief build complex matrix to represent current robot 2D pose
            * \param x current robot x-position
            * \param y current robot y-position
            * \param theta current robot orientation
            * \return complex matrix for current robot 2D pose
            */
            Eigen::Matrix2cf getComplexMatrix(const float & x, 
                                              const float & y, 
                                              const float & theta);
            // float dist2Pose(const float & theta, const float & dist, const geometry_msgs::Pose & pose);

            /**
            * \brief Helper function for clipping velocities to maximum allowed velocities
            * \param velLinXFeedback feedback linear command velocity in x direction
            * \param velLinYFeedback feedback linear command velocity in y direction
            * \param velAngFeedback feedback angular command velocity
            */
            void clipRobotVelocity(float & velLinXFeedback, float & velLinYFeedback, float & velAngFeedback);


            void runProjectionOperator(const geometry_msgs::PoseStamped & rbtPoseInSensorFrame,
                                        Eigen::Vector2f & cmdVelFeedback,
                                        float & Psi, Eigen::Vector2f & dPsiDx,
                                        float & velLinXSafe, float & velLinYSafe,
                                        float & minDistTheta, float & minDist);

            Eigen::Vector3f calculateProjectionOperator(const Eigen::Vector2f & closestScanPtToRobot);

            void visualizeProjectionOperator(const float & weightedVelLinXSafe, const float & weightedVelLinYSafe);

            void runBearingRateCBF(const Eigen::Vector4f & state, 
                                    const Eigen::Vector4f & rightGapPtState,
                                    const Eigen::Vector4f & leftGapPtState,
                                    const Eigen::Vector2f & currRbtAcc,
                                    float & velLinXSafe, float & velLinYSafe, float & PsiCBF);

            float rightGapSideCBF(const Eigen::Vector4f & state);
            Eigen::Vector4f rightGapSideCBFDerivative(const Eigen::Vector4f & state);
            float leftGapSideCBF(const Eigen::Vector4f & state);
            Eigen::Vector4f leftGapSideCBFDerivative(const Eigen::Vector4f & state);


            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */
            const DynamicGapConfig * cfg_ = NULL; /**< Planner hyperparameter config list */

            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */
            ros::Publisher projOpPublisher_;

            float k_fb_theta_;

            float distanceThresh_;

            float manualVelX_;
            float manualVelY_;
            float manualVelAng_;

            float manualVelLinIncrement_;
            float manualVelAngIncrement_;
    };
}