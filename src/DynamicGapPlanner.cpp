#include <dynamic_gap/DynamicGapPlanner.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(dynamic_gap::DynamicGapPlanner, nav_core::BaseLocalPlanner)

namespace dynamic_gap 
{
    bool DynamicGapPlanner::isGoalReached()
    {
        return planner_.isGoalReached();
    }

    bool DynamicGapPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame)
    {
        ROS_INFO_STREAM_NAMED("Planner", "DynamicGap: setPlan");
        return planner_.setGoal(globalPlanMapFrame);
    }

    // This function signature must be used in order for planner to succeed as a move_base plugin,
    // but we will not really use the parameters
    void DynamicGapPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap)
    {
        // ROS_INFO_STREAM_NAMED("Planner", "DynamicGap: initialize");
        plannerName_ = name;
        nh_ = ros::NodeHandle("~/" + plannerName_); // pnh: planner node handle?
        planner_.initialize(nh_);
        std::string robot_name = "/robot" + std::to_string(planner_.getCurrentAgentCount());

        laserSub_ = nh_.subscribe(robot_name + "/mod_laser_0", 5, &Planner::laserScanCB, &planner_);
        
        // Linking the robot pose and acceleration subscribers because these messages are published 
        // essentially at the same time in STDR
        rbtPoseSub_.subscribe(nh_, robot_name + "/odom", 10);
        rbtAccSub_.subscribe(nh_, robot_name + "/acc", 10);
        sync_.reset(new CustomSynchronizer(rbtPoseAndAccSyncPolicy(10), rbtPoseSub_, rbtAccSub_));
        sync_->registerCallback(boost::bind(&Planner::jointPoseAccCB, &planner_, _1, _2));

        for (int i = 0; i < planner_.getCurrentAgentCount(); i++)
        {
            agentPoseSubs_.push_back(nh_.subscribe("/robot" + std::to_string(i) + "/odom", 5, &Planner::agentOdomCB, &planner_));
        }

    }

    bool DynamicGapPlanner::computeVelocityCommands(geometry_msgs::Twist & cmdVel)
    {
        if (!planner_.initialized())
        {
            // ros::NodeHandle pnh("~/" + plannerName_);
            planner_.initialize(nh_);
            ROS_WARN_STREAM("computeVelocity called before initializing planner");
        }

        dynamic_gap::Trajectory localTrajectory = planner_.runPlanningLoop();

        cmdVel = planner_.ctrlGeneration(localTrajectory.getPathOdomFrame());

        bool cmdVelCheck = planner_.recordAndCheckVel(cmdVel);

        // bool cmdVelCheck = true;
        // cmdVel.linear.x = 0.5;
        // cmdVel.linear.y = 0.0;
        // cmdVel.angular.z = 0.0;

        return cmdVelCheck;
    }

    void DynamicGapPlanner::reset()
    {
        planner_.reset();
        return;
    }

}