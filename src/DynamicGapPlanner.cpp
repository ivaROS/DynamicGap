#include <ros/ros.h>
#include <dynamic_gap/DynamicGapPlanner.h>
// #include <dynamic_gap/utils/Gap.h>
#include <pluginlib/class_list_macros.h>

// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

// #include <boost/numeric/odeint.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>

// using namespace boost::numeric::odeint;
namespace pl = std::placeholders;

PLUGINLIB_EXPORT_CLASS(dynamic_gap::DynamicGapPlanner, nav_core::BaseLocalPlanner)

namespace dynamic_gap 
{
    DynamicGapPlanner::DynamicGapPlanner(){}

    DynamicGapPlanner::~DynamicGapPlanner()
    {
        ROS_INFO_STREAM_NAMED("Planner", "Planner terminated");
    }

    bool DynamicGapPlanner::isGoalReached()
    {
        return planner_.isGoalReached();
    }

    bool DynamicGapPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame)
    {
        ROS_INFO_STREAM_NAMED("Planner", "DynamicGap: setPlan");
        return planner_.setGoal(globalPlanMapFrame);
    }

    void DynamicGapPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // ROS_INFO_STREAM_NAMED("Planner", "DynamicGap: initialize");
        plannerName_ = name;
        pnh_ = ros::NodeHandle("~/" + plannerName_); // pnh: planner node handle?
        planner_.initialize(pnh_);
        std::string robot_name = "/robot" + std::to_string(planner_.getCurrentAgentCount());

        laserSub_ = pnh_.subscribe(robot_name + "/mod_laser_0", 5, &Planner::laserScanCB, &planner_);
        // staticLaserSub_ = pnh_.subscribe("/static_point_scan", 1, &Planner::staticLaserScanCB, &planner);
        
        // Linking the robot pose and acceleration subscribers because these messages are published 
        // essentially at the same time in STDR
        rbtPoseSub_.subscribe(pnh_, robot_name + "/odom", 10);
        rbtAccSub_.subscribe(pnh_, robot_name + "/acc", 10);
        sync_.reset(new Sync(MySyncPolicy(10), rbtPoseSub_, rbtAccSub_));
        sync_->registerCallback(boost::bind(&Planner::jointPoseAccCB, &planner_, _1, _2));
    }

    bool DynamicGapPlanner::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
    {
        if (!planner_.initialized())
        {
            // ros::NodeHandle pnh("~/" + plannerName_);
            planner_.initialize(pnh_);
            ROS_WARN_STREAM("computeVelocity called before initializing planner");
        }

        geometry_msgs::PoseArray localTrajectory = planner_.runPlanningLoop();

        cmd_vel = planner_.ctrlGeneration(localTrajectory);

        return planner_.recordAndCheckVel(cmd_vel);
    }

    void DynamicGapPlanner::reset()
    {
        planner_.reset();
        return;
    }

}