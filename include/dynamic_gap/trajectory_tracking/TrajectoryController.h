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
    /**
    * \brief Class responsible for (a) tracking selected trajectory and 
    * (b) enacting last resort safety modules
    */
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
            * \return command velocity for robot
            */
            geometry_msgs::Twist constantVelocityControlLaw(const geometry_msgs::Pose & current, 
                                                            const geometry_msgs::Pose & desired); // const sensor_msgs::LaserScan & scan, 

            /**
            * \brief Control law for trajectory tracking
            * \param current current robot pose
            * \param desired desired robot pose
            * \return command velocity for robot
            */
            geometry_msgs::Twist controlLaw(const geometry_msgs::Pose & current, 
                                            const geometry_msgs::Pose & desired); // const sensor_msgs::LaserScan & scan, 

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
            void clipRobotVelocity(float & velLinXFeedback, 
                                   float & velLinYFeedback, 
                                   float & velAngFeedback);

            /**
            * \brief Function for running projection operator module on command velocity
            * \param rbtPoseInSensorFrame robot pose in sensor frame
            * \param cmdVelFeedback feedback command velocity
            * \param Psi projection operator function value
            * \param dPsiDx projection operator gradient values
            * \param velLinXSafe safe command velocity in x direction
            * \param velLinYSafe safe command velocity in y direction
            * \param minDistTheta orientation of minimum distance scan point
            * \param minDist range of minimum distance scan point
            */
            void runProjectionOperator(const geometry_msgs::PoseStamped & rbtPoseInSensorFrame,
                                        Eigen::Vector2f & cmdVelFeedback,
                                        float & Psi, 
                                        Eigen::Vector2f & dPsiDx,
                                        float & velLinXSafe, 
                                        float & velLinYSafe,
                                        float & minDistTheta, 
                                        float & minDist);

            /**
            * \brief Function for calculating projection operator
            * \param closestScanPtToRobot minimum distance scan point
            * \return projection operator function and gradient values (Psi and dPsiDx)
            */
            Eigen::Vector3f calculateProjectionOperator(const Eigen::Vector2f & closestScanPtToRobot);

            /**
            * \brief Function for visualizing projection operator output in RViz
            * \param weightedVelLinXSafe weighted safe command velocity in x direction
            * \param weightedVelLinYSafe weighted safe command velocity in y direction
            */
            void visualizeProjectionOperator(const float & weightedVelLinXSafe, 
                                             const float & weightedVelLinYSafe);

            /**
            * \brief Function for running bearing rate CBF on command velocity
            * \param state current robot state (position and velocity)
            * \param leftGapPtState current left gap point state
            * \param rightGapPtState current right gap point state
            * \param currRbtAcc current robot acceleration
            * \param velLinXSafe safe command velocity in x direction
            * \param velLinYSafe safe command velocity in y direction
            * \param PsiCBF CBF value            
            */
            void runBearingRateCBF(const Eigen::Vector4f & state, 
                                    const Eigen::Vector4f & leftGapPtState,
                                    const Eigen::Vector4f & rightGapPtState,
                                    const Eigen::Vector2f & currRbtAcc,
                                    float & velLinXSafe, 
                                    float & velLinYSafe, 
                                    float & PsiCBF);

            /**
            * \brief Function for running left gap side bearing CBF
            * \param state current robot state (position and velocity)
            * \return left gap side bearing CBF value
            */
            float leftGapSideCBF(const Eigen::Vector4f & state);

            /**
            * \brief Function for running left gap side bearing CBF gradient
            * \param state current robot state (position and velocity)
            * \return left gap side bearing CBF gradient
            */            
            Eigen::Vector4f leftGapSideCBFDerivative(const Eigen::Vector4f & state);

            /**
            * \brief Function for running right gap side bearing CBF
            * \param state current robot state (position and velocity)
            * \return right gap side bearing CBF value
            */            
            float rightGapSideCBF(const Eigen::Vector4f & state);

            /**
            * \brief Function for running right gap side bearing CBF gradient
            * \param state current robot state (position and velocity)
            * \return right gap side bearing CBF gradient
            */  
            Eigen::Vector4f rightGapSideCBFDerivative(const Eigen::Vector4f & state);

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */
            const DynamicGapConfig * cfg_ = NULL; /**< Planner hyperparameter config list */

            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */
            ros::Publisher projOpPublisher_; /**< Projection operator publisher */

            float KFeedbackTheta_; /**< Proportional feedback gain for robot theta */

            float manualVelX_; /**< Linear command velocity in x-direction for manual control */
            float manualVelY_; /**< Linear command velocity in y-direction for manual control */
            float manualVelAng_; /**< Angular command velocity in yaw direction for manual control */

            float manualVelLinIncrement_; /**< Linear command velocity increment for manual control */
            float manualVelAngIncrement_; /**< Angular command velocity increment for manual control */
    };
}