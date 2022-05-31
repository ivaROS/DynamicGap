#ifndef GOAL_SELECT_H
#define GOAL_SELECT_H

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/gap.h>
#include <dynamic_gap/dynamicgap_config.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

namespace dynamic_gap
{
    class GoalSelector{
        public: 
            GoalSelector() {};
            ~GoalSelector() {};

            GoalSelector(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            GoalSelector& operator=(GoalSelector other) {cfg_ = other.cfg_;};
            GoalSelector(const GoalSelector &t) {cfg_ = t.cfg_;};

            // Map Frame
            bool setGoal(const std::vector<geometry_msgs::PoseStamped> &);
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);
            void updateLocalGoal(geometry_msgs::TransformStamped map2rbt);
            geometry_msgs::PoseStamped getCurrentLocalGoal(geometry_msgs::TransformStamped rbt2odom);
            geometry_msgs::PoseStamped rbtFrameLocalGoal() {return local_goal;};
            std::vector<geometry_msgs::PoseStamped> getOdomGlobalPlan();
            std::vector<geometry_msgs::PoseStamped> getRelevantGlobalPlan(geometry_msgs::TransformStamped);


        private:
            const DynamicGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser;
            std::vector<geometry_msgs::PoseStamped> global_plan;
            std::vector<geometry_msgs::PoseStamped> mod_plan;
            geometry_msgs::PoseStamped local_goal; // Robot Frame
            boost::mutex goal_select_mutex;
            boost::mutex lscan_mutex;
            boost::mutex gplan_mutex;

            double threshold = 3;

            // If distance to robot is within
            bool isNotWithin(const double dist);
            // Pose to robot, when all in rbt frames
            double dist2rbt(geometry_msgs::PoseStamped);
            double scanDistsAtPlanIndices(geometry_msgs::PoseStamped pose, sensor_msgs::LaserScan stored_scan_msgs);
            int PoseIndexInSensorMsg(geometry_msgs::PoseStamped pose);
            double getPoseOrientation(geometry_msgs::PoseStamped);
            bool VisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose);
            bool NoTVisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose);

    };
}

#endif