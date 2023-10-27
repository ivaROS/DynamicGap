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
            geometry_msgs::Twist processCmdVel(const geometry_msgs::Twist & raw_cmd_vel,
                                                const sensor_msgs::LaserScan & inflated_egocircle, 
                                                const geometry_msgs::PoseStamped & rbt_in_cam_lc, 
                                                const dynamic_gap::Estimator * curr_rightGapPtModel, 
                                                const dynamic_gap::Estimator * curr_leftGapPtModel,
                                                const geometry_msgs::TwistStamped & current_rbt_vel, 
                                                const geometry_msgs::TwistStamped & rbt_accel);
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg);
            int targetPoseIdx(const geometry_msgs::Pose & curr_pose, const geometry_msgs::PoseArray & poseArray);
            // dynamic_gap::TrajPlan trajGen(geometry_msgs::PoseArray);
            
        private:
            Eigen::Matrix2cf getComplexMatrix(float, float, float, float);
            Eigen::Matrix2cf getComplexMatrix(float, float, float);
            float dist2Pose(float theta, float dist, geometry_msgs::Pose pose);

            std::vector<geometry_msgs::Point> findLocalLine(int idx);
            float polDist(float l1, float t1, float l2, float t2);

            bool leqThres(const float dist);
            bool geqThres(const float dist);

            void runProjectionOperator(const sensor_msgs::LaserScan & inflated_egocircle, 
                                         const geometry_msgs::PoseStamped & rbt_in_cam_lc,
                                         Eigen::Vector2f & cmd_vel_fb, float & Psi, Eigen::Vector2f & Psi_der,
                                         float & cmd_vel_x_safe, float & cmd_vel_y_safe,
                                         float & min_dist_ang, float & min_dist);
            void runBearingRateCBF(const Eigen::Vector4f & state, 
                                    const Eigen::Vector4f & rightGapPtState,
                                    const Eigen::Vector4f & leftGapPtState,
                                    const Eigen::Vector2f & currRbtAcc,
                                    float & cmd_vel_x_safe, float & cmd_vel_y_safe, 
                                    float & Psi_CBF);
            void clipRobotVelocity(float & v_lin_x_fb, float & v_lin_y_fb, float & v_ang_fb);
            void visualizeProjectionOperator(float weighted_cmd_vel_x_safe, float weighted_cmd_vel_y_safe);


            float rightGapSideCBF(const Eigen::Vector4f & state);
            Eigen::Vector4f rightGapSideCBFDerivative(const Eigen::Vector4f & state);
            float leftGapSideCBF(const Eigen::Vector4f & state);
            Eigen::Vector4f leftGapSideCBFDerivative(const Eigen::Vector4f & state);

            Eigen::Vector3f calculateProjectionOperator(float min_diff_x, float min_diff_y);

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