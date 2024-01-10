#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include "tf/transform_datatypes.h"
// #include <tf/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

namespace dynamic_gap
{
    class GoalSelector
    {
        public: 
            GoalSelector(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            // GoalSelector& operator=(GoalSelector other) {cfg_ = other.cfg_; return *this; };
            // GoalSelector(const GoalSelector &t) {cfg_ = t.cfg_;};

            // Map Frame
            void updateGlobalPathMapFrame(const std::vector<geometry_msgs::PoseStamped> & globalPath);
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);
            void generateGlobalPathLocalWaypoint(const geometry_msgs::TransformStamped & map2rbt);
            geometry_msgs::PoseStamped getGlobalPathLocalWaypointOdomFrame(const geometry_msgs::TransformStamped & rbt2odom);
            geometry_msgs::PoseStamped getGlobalPathLocalWaypointRobotFrame() { return globalPathLocalWaypointRobotFrame_; };
            std::vector<geometry_msgs::PoseStamped> getGlobalPathOdomFrame();
            std::vector<geometry_msgs::PoseStamped> getVisibleGlobalPlanSnippetRobotFrame(const geometry_msgs::TransformStamped & map2rbt);


        private:
            const DynamicGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr_;
            std::vector<geometry_msgs::PoseStamped> globalPlanMapFrame_;
            geometry_msgs::PoseStamped globalPathLocalWaypointRobotFrame_; // Robot Frame
            boost::mutex goalSelectMutex_;
            boost::mutex scanMutex_;
            boost::mutex globalPlanMutex_;

            // If distance to robot is within
            bool isNotWithin(const float dist);

            // Pose to robot, when all in rbt frames
            float poseNorm(const geometry_msgs::PoseStamped & pose);
            float calculateScanDistsAtPlanIndices(const geometry_msgs::PoseStamped & pose);
            int poseIdxInScan(const geometry_msgs::PoseStamped & pose);
            float getPoseOrientation(const geometry_msgs::PoseStamped & pose);

    };
}