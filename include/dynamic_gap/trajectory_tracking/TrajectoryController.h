#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>

#include <thread>
// #include <chrono>

namespace dynamic_gap 
{
    /**
    * \brief Class responsible for (a) tracking selected trajectory and 
    *                              (b) enacting last resort safety modules
    */
    class TrajectoryController 
    {
        public:

            TrajectoryController(ros::NodeHandle& nh, const DynamicGapConfig& cfg);

            void updateParams(const ControlParameters & ctrlParams);

            /**
            * \brief receive new laser scan and update member variable accordingly
            * \param scan new laser scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);
            
            /**
            * \brief Control law for pure obstacle avoidance
            * \return command velocity for robot
            */
            geometry_msgs::Twist obstacleAvoidanceControlLaw();

            /**
            * \brief Control law for pure obstacle avoidance
            * \return command velocity for robot
            */
            geometry_msgs::Twist obstacleAvoidanceControlLawNonHolonomic();
            
            /**
            * \brief Control law for manual robot operation
            * \return command velocity for robot
            */
            geometry_msgs::Twist manualControlLawKeyboard();

            /**
            * \brief Control law for manual robot operation
            * \return command velocity for robot
            */
            geometry_msgs::Twist manualControlLawReconfig();

            // geometry_msgs::Twist manualControlLawPrescribed(const geometry_msgs::Pose & current);

            /**
            * \brief Control law for trajectory tracking
            * \param current current robot pose
            * \param desired desired robot pose
            * \return command velocity for robot
            */
            geometry_msgs::Twist constantVelocityControlLaw(const geometry_msgs::Pose & current, 
                                                            const geometry_msgs::Pose & desired,
                                                            const float & desiredSpeed);

            /**
            * \brief Control law for trajectory tracking
            * \param current current robot pose
            * \param desired desired robot pose
            * \return command velocity for robot
            */
            geometry_msgs::Twist constantVelocityControlLawNonHolonomicLookahead(const geometry_msgs::Pose & current, 
                                                                        const geometry_msgs::Pose & desired,
                                                                        const float & desiredSpeed);                                                            

            /**
            * \brief Apply post-processing steps to command velocity including robot kinematic limits
            * along with last-resort safety modules such as projection operator or CBF
            * \param rawCmdVel raw command velocity
            * \param rbtPoseInSensorFrame robot pose in sensor frame
            * \param currRbtVel current robot velocity
            * \param currRbtAcc current robot acceleration
            * \return processed command velocity
            */
            geometry_msgs::Twist processCmdVel(const geometry_msgs::Twist & rawCmdVel,
                                                const geometry_msgs::PoseStamped & rbtPoseInSensorFrame);
                                                         
            /**
            * \brief Apply post-processing steps to command velocity including robot kinematic limits
            * along with last-resort safety modules such as projection operator or CBF
            * \param rawCmdVel raw command velocity
            * \param rbtPoseInSensorFrame robot pose in sensor frame
            * \param currRbtVel current robot velocity
            * \param currRbtAcc current robot acceleration
            * \return processed command velocity
            */
            geometry_msgs::Twist processCmdVelNonHolonomic(const geometry_msgs::Pose & currentPoseOdomFrame,
                                                            const geometry_msgs::Pose & desiredPoseOdomFrame,
                                                            const geometry_msgs::Twist & rawCmdVel,
                                                            const geometry_msgs::PoseStamped & rbtPoseInSensorFrame);

            geometry_msgs::Twist cbf_processCmdVelNonHolonomic(const geometry_msgs::Pose & currentPoseOdomFrame,
                                                                            const geometry_msgs::Pose & desiredPoseOdomFrame,
                                                                            const geometry_msgs::Twist & nonholoCmdVel,
                                                                            const geometry_msgs::PoseStamped & rbtPoseInSensorFrame,
                                                                            Eigen::Vector2f humanVel,
                                                                            Eigen::Vector2f relativeGapPos,
                                                                            Eigen::Vector2f trajPos,
                                                                            Eigen::Vector2f robotVel);

            float DPCBF(Eigen::Vector2f relativeVel,
                            Eigen::Vector2f relativeGapPos,
                            Eigen::Vector2f trajPos,
                            Eigen::Vector2f robotVel);
   
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
                                        // float & Psi, 
                                        // Eigen::Vector2f & dPsiDx,
                                        float & velLinXSafe, 
                                        float & velLinYSafe,
                                        float & minDistTheta, 
                                        float & minDist);

            /**
            * \brief Function for calculating projection operator
            * \param closestScanPtToRobot minimum distance scan point
            * \return projection operator function and gradient values (Psi and dPsiDx)
            */
            void calculateProjectionOperator(const Eigen::Vector2f & closestScanPtToRobot,
                                                float & Psi, Eigen::Vector2f & dPsiDx);

            /**
            * \brief Function for visualizing projection operator output in RViz
            * \param weightedVelLinXSafe weighted safe command velocity in x direction
            * \param weightedVelLinYSafe weighted safe command velocity in y direction
            * \param minRangeTheta theta at which minimum range occurs
            * \param minRange minimum range in scan
            */
            void visualizeProjectionOperator(const float & weightedVelLinXSafe, 
                                             const float & weightedVelLinYSafe,
                                             const float & minRangeTheta, 
                                                const float & minRange);

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */
            const DynamicGapConfig * cfg_ = NULL; /**< Planner hyperparameter config list */

            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */
            ros::Publisher projOpPublisher_; /**< Projection operator publisher */

            float KFeedbackTheta_; /**< Proportional feedback gain for robot theta */

            float l_; /**< Lookahead distance for nonholonomic control */

            float manualVelX_; /**< Linear command velocity in x-direction for manual control */
            float manualVelY_; /**< Linear command velocity in y-direction for manual control */
            float manualVelAng_; /**< Angular command velocity in yaw direction for manual control */

            float manualVelLinIncrement_; /**< Linear command velocity increment for manual control */
            float manualVelAngIncrement_; /**< Angular command velocity increment for manual control */

            ControlParameters ctrlParams_; /**< Control parameters for gap control */

            geometry_msgs::Pose prevDesired_; /**< Previous desired robot pose */
            ros::Time prevDesiredTime_; /**< Previous desired robot pose time */
            float lambda_; /**< lambda value for CBF */
            float lambdaInit_; /**< initial lambda value for CBF */
            float epsilon_; /**< epsilon value for CBF */
            float cp_ = 0.0; /**< proportional coefficient value for CBF */
            float clambda_ = 0.0; /**< lambda coefficient value for CBF */

            ros::Publisher propagatedPointsPublisher_; /**< Publisher for propagated points */
            // std::chrono::steady_clock::time_point startTime_; /**< Start time */

            ros::NodeHandle nh_;
            ros::Publisher vrel_pub_; /**< used for visualizing vrel */
            ros::Publisher vrel_safe_pub_;
            int vrel_marker_id_ = 0;

            ros::Publisher dbg_marker_pub_; // visualization for debugging cbf line of sighte rotation
            std::string dbg_frame_ = "map";   // or odom/base_link
            bool dbg_dpcbf_ = true;
            float dbg_scale_ = 0.5f;          // meters for unit vectors

           void publishVrelArrow(const Eigen::Vector2f& origin_xy,
                      const Eigen::Vector2f& vrel_xy,
                      const std::string& frame_id,
                      const std::string& ns,
                      ros::Publisher& pub,
                      float r = 1.0f,
                      float g = 0.2f,
                      float b = 0.2f,
                      float a = 1.0f);

                };



}