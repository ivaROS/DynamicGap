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
            geometry_msgs::Twist obstacleAvoidanceControlLaw(const sensor_msgs::LaserScan &);
            geometry_msgs::Twist manualControlLaw();
            geometry_msgs::Twist controlLaw(const geometry_msgs::Pose & current, 
                                            const geometry_msgs::Pose & desired,
                                            const sensor_msgs::LaserScan & inflated_egocircle, 
                                            const geometry_msgs::TwistStamped & currentPeakSplineVel_);
            geometry_msgs::Twist processCmdVel(const geometry_msgs::Twist & rawCmdVel,
                                                const sensor_msgs::LaserScan & scan, 
                                                const geometry_msgs::PoseStamped & rbtPoseInSensorFrame, 
                                                const dynamic_gap::Estimator * currGapRightPtModel, 
                                                const dynamic_gap::Estimator * currGapLeftPtModel,
                                                const geometry_msgs::TwistStamped & currRbtVel, 
                                                const geometry_msgs::TwistStamped & currRbtAcc);
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);
            int targetPoseIdx(const geometry_msgs::Pose & currPose, const geometry_msgs::PoseArray & localTrajectory);
            // dynamic_gap::TrajPlan trajGen(geometry_msgs::PoseArray);
            
        private:
            // Eigen::Matrix2cf getComplexMatrix(float, float, float, float);
            Eigen::Matrix2cf getComplexMatrix(float x, float y, float theta);
            float dist2Pose(float theta, float dist, geometry_msgs::Pose pose);

            float polDist(float l1, float t1, float l2, float t2);

            bool leqThres(const float dist);
            bool geqThres(const float dist);

            void runProjectionOperator(const sensor_msgs::LaserScan & scan, 
                                        const geometry_msgs::PoseStamped & rbtPoseInSensorFrame,
                                        Eigen::Vector2f & cmdVelFeedback,
                                        float & Psi, Eigen::Vector2f & dPsiDx,
                                        float & velLinXSafe, float & velLinYSafe,
                                        float & minDistTheta, float & minDist);
            void runBearingRateCBF(const Eigen::Vector4f & state, 
                                    const Eigen::Vector4f & rightGapPtState,
                                    const Eigen::Vector4f & leftGapPtState,
                                    const Eigen::Vector2f & currRbtAcc,
                                    float & velLinXSafe, float & velLinYSafe, float & PsiCBF);
            void clipRobotVelocity(float & velLinXFeedback, float & velLinYFeedback, float & velAngFeedback);
            void visualizeProjectionOperator(float weightedVelLinXSafe, float weightedVelLinYSafe);


            float rightGapSideCBF(const Eigen::Vector4f & state);
            Eigen::Vector4f rightGapSideCBFDerivative(const Eigen::Vector4f & state);
            float leftGapSideCBF(const Eigen::Vector4f & state);
            Eigen::Vector4f leftGapSideCBFDerivative(const Eigen::Vector4f & state);

            Eigen::Vector3f calculateProjectionOperator(const Eigen::Vector2f & closestScanPtToRobot);

            const DynamicGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_;
            boost::mutex egocircleLock_;
            ros::Publisher projOpPublisher_;

            // bool holonomic;
            // bool full_fov;
            // bool projection_operator;
            // float kFeedbackX_;
            // float k_fb_y_;
            float k_fb_theta_;
            // float k_po_x_;
            // float k_po_theta_;            
            // float k_CBF_;
            // int cmd_counter_;

            float distanceThresh_;

            float manualVelX_, manualVelY_, manualVelAng_;
            float manualVelLinIncrement_, manualVelAngIncrement_;
    };
}