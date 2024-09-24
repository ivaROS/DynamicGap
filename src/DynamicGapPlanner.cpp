#include <dynamic_gap/DynamicGapPlanner.h>

// MBF return codes
#include <mbf_msgs/ExePathResult.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dynamic_gap::DynamicGapPlanner, nav_core::BaseLocalPlanner)

namespace dynamic_gap 
{
    void DynamicGapPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // ROS_INFO_STREAM("[DynamicGapPlanner::initialize()]");
        // do NOT set nodehandle to any string
        // nh_ = ros::NodeHandle(); // "~/" + name pnh: planner node handle?

        name_ = name;
        planner_.initialize(name_);

        return;
    }

    bool DynamicGapPlanner::computeVelocityCommands(geometry_msgs::Twist & cmdVel)
    {
        // ROS_INFO_STREAM("[DynamicGapPlanner::computeVelocityCommands(short)]");

        std::string dummy_message;
        geometry_msgs::PoseStamped dummy_pose;
        geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;

        bool outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);

        cmdVel = cmd_vel_stamped.twist;

        ROS_INFO_STREAM("computeVelocityCommands cmdVel: " << cmdVel);

        // TODO: just hardcoding this now, need to revise
        bool success = 1;

        return success;
    }

    uint32_t DynamicGapPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                        const geometry_msgs::TwistStamped& velocity,
                                                        geometry_msgs::TwistStamped &cmd_vel,
                                                        std::string &message)
    {
        // ROS_INFO_STREAM("[DynamicGapPlanner::computeVelocityCommands(long)]");

        if (!planner_.initialized())
        {
            planner_.initialize(name_); // nh_
            ROS_WARN_STREAM("computeVelocity called before initializing planner");
        }

        planner_.setReachedGlobalGoal(false);

        dynamic_gap::Trajectory localTrajectory;
        int trajFlag; 
        planner_.runPlanningLoop(localTrajectory, trajFlag);

        if (planner_.isGoalReached())
        {
            cmd_vel.twist = geometry_msgs::Twist();
            return mbf_msgs::ExePathResult::SUCCESS;
        }

        geometry_msgs::Twist cmdVelNoStamp = planner_.ctrlGeneration(localTrajectory.getPathOdomFrame(), trajFlag);

        cmd_vel.twist = cmdVelNoStamp;

        bool acceptedCmdVel = planner_.recordAndCheckVel(cmdVelNoStamp);

        /*
        *         SUCCESS           = 0
        *         1..9 are reserved as plugin specific non-error results
        *         FAILURE           = 100  # Unspecified failure, only used for old, non-mfb_core based plugins
        *         CANCELED          = 101
        *         NO_VALID_CMD      = 102
        *         PAT_EXCEEDED      = 103
        *         COLLISION         = 104
        *         OSCILLATION       = 105
        *         ROBOT_STUCK       = 106
        *         MISSED_GOAL       = 107
        *         MISSED_PATH       = 108
        *         BLOCKED_GOAL      = 109
        *         BLOCKED_PATH      = 110
        *         INVALID_PATH      = 111
        *         TF_ERROR          = 112
        *         NOT_INITIALIZED   = 113
        *         INVALID_PLUGIN    = 114
        *         INTERNAL_ERROR    = 115
        *         OUT_OF_MAP        = 116  # The start and / or the goal are outside the map
        *         MAP_ERROR         = 117  # The map is not running properly
        *         STOPPED           = 118  # The controller execution has been stopped rigorously
        */
        if (acceptedCmdVel)
            return mbf_msgs::ExePathResult::SUCCESS;
        else
            return mbf_msgs::ExePathResult::FAILURE;
    }

    bool DynamicGapPlanner::isGoalReached()
    {
        // ROS_INFO_STREAM("[DynamicGapPlanner::isGoalReached()]");

        return planner_.isGoalReached();
    }

    bool DynamicGapPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame)
    {
        // ROS_INFO_STREAM("[DynamicGapPlanner::setPlan()]");

        if (!planner_.initialized())
        {
            return false;
        } else
        {
            return planner_.setPlan(globalPlanMapFrame);
        }
        // 0: fail, 1: success
        // return 1;
    }
}