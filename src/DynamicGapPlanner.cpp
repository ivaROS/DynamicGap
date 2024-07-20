#include <dynamic_gap/DynamicGapPlanner.h>
#include <pluginlib/class_list_macros.h>


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
        ROS_INFO_STREAM("[DynamicGapPlanner::isGoalReached()]");

        return 0;
        // return planner_.isGoalReached();
    }

    bool DynamicGapPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame)
    {
        ROS_INFO_STREAM("[DynamicGapPlanner::setPlan()]");

        // 0: fail, 1: success
        return 1;
        // return planner_.setGoal(globalPlanMapFrame);
    }

    // This function signature must be used in order for planner to succeed as a move_base plugin,
    // but we will not really use the parameters
    void DynamicGapPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_INFO_STREAM("[DynamicGapPlanner::initialize()]");
        nh_ = ros::NodeHandle("~/" + name); // pnh: planner node handle?
        planner_.initialize(nh_);
        // std::string robot_name = "/robot" + std::to_string(planner_.getCurrentAgentCount());

        // laserSub_ = nh_.subscribe(robot_name + "/mod_laser_0", 5, &Planner::laserScanCB, &planner_);
        
        // // Linking the robot pose and acceleration subscribers because these messages are published 
        // // essentially at the same time in STDR
        // rbtPoseSub_.subscribe(nh_, robot_name + "/odom", 10);
        // rbtAccSub_.subscribe(nh_, robot_name + "/acc", 10);
        // sync_.reset(new CustomSynchronizer(rbtPoseAndAccSyncPolicy(10), rbtPoseSub_, rbtAccSub_));
        // sync_->registerCallback(boost::bind(&Planner::jointPoseAccCB, &planner_, _1, _2));

        // for (int i = 0; i < planner_.getCurrentAgentCount(); i++)
        // {
        //     agentPoseSubs_.push_back(nh_.subscribe("/robot" + std::to_string(i) + "/odom", 5, &Planner::agentOdomCB, &planner_));
        // }

        return;
    }

    uint32_t DynamicGapPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                        const geometry_msgs::TwistStamped& velocity,
                                                        geometry_msgs::TwistStamped &cmd_vel,
                                                        std::string &message)
    {
        ROS_INFO_STREAM("[DynamicGapPlanner::computeVelocityCommands(long)]");

        return 0;

        // if (!planner_.initialized())
        // {
        //     planner_.initialize(nh_);
        //     ROS_WARN_STREAM("computeVelocity called before initializing planner");
        // }

        // dynamic_gap::Trajectory localTrajectory = planner_.runPlanningLoop();

        // geometry_msgs::Twist cmdVelNoStamp = planner_.ctrlGeneration(localTrajectory.getPathOdomFrame());

        // cmd_vel.twist = cmdVelNoStamp;

        // return planner_.recordAndCheckVel(cmdVelNoStamp);
    }

    bool DynamicGapPlanner::computeVelocityCommands(geometry_msgs::Twist & cmdVel)
    {
        cmdVel.linear.x = 0.5;

        return 1;

        // ROS_INFO_STREAM("[DynamicGapPlanner::computeVelocityCommands(short)]");

        // std::string dummy_message;
        // geometry_msgs::PoseStamped dummy_pose;
        // geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;

        // bool outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);

        // cmdVel = cmd_vel_stamped.twist;

        // // TODO: just hardcoding this now, need to revise
        // bool success = 0;

        // return success;
    }
}