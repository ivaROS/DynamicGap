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

        ros::NodeHandle nh("~/" + name);

        ros::NodeHandle estimation_nh(nh, "estimation");
        dynamic_estimation_recfg_ = boost::make_shared<dynamic_reconfigure::Server<EstimationParametersConfig>>(estimation_nh);
        dynamic_reconfigure::Server<EstimationParametersConfig>::CallbackType estimation_cb =
        boost::bind(&DynamicGapPlanner::reconfigureEstimationCallback, this, _1, _2);
        dynamic_estimation_recfg_->setCallback(estimation_cb);

        ros::NodeHandle control_nh(nh, "control");
        dynamic_control_recfg_ = boost::make_shared<dynamic_reconfigure::Server<ControlParametersConfig>>(control_nh);
        dynamic_reconfigure::Server<ControlParametersConfig>::CallbackType control_cb =
        boost::bind(&DynamicGapPlanner::reconfigureControlCallback, this, _1, _2);
        dynamic_control_recfg_->setCallback(control_cb);

        return;
    }

    void DynamicGapPlanner::reconfigureControlCallback(ControlParametersConfig &config, uint32_t level)
    {
        ctrlParams_.linear_vel_x_ = config.linear_vel_x;
        ctrlParams_.linear_vel_y_ = config.linear_vel_y;
        ctrlParams_.angular_vel_z_ = config.angular_vel_z;
    }


    void DynamicGapPlanner::reconfigureEstimationCallback(EstimationParametersConfig &config, uint32_t level)
    {
        estParams_.Q_ = config.Q;
        estParams_.R_ = config.R;
    }


    bool DynamicGapPlanner::computeVelocityCommands(geometry_msgs::Twist & cmdVel)
    {
        // ROS_INFO_STREAM("[DynamicGapPlanner::computeVelocityCommands(short)]");

        std::string dummy_message;
        geometry_msgs::PoseStamped dummy_pose;
        geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;

        bool outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);

        cmdVel = cmd_vel_stamped.twist;

        ROS_INFO_STREAM_NAMED("DynamicGapPlanner", "computeVelocityCommands cmdVel: ");
        ROS_INFO_STREAM_NAMED("DynamicGapPlanner", "                linear: ");
        ROS_INFO_STREAM_NAMED("DynamicGapPlanner", "                  x: " << cmdVel.linear.x << ", y: " << cmdVel.linear.y << ", z: " << cmdVel.linear.z);
        ROS_INFO_STREAM_NAMED("DynamicGapPlanner", "                angular: ");
        ROS_INFO_STREAM_NAMED("DynamicGapPlanner", "                  x: " << cmdVel.angular.x << ", y: " << cmdVel.angular.y << ", z: " << cmdVel.angular.z);

        // TODO: just hardcoding this now, need to revise
        bool success = 1;

        return success;
    }

    uint32_t DynamicGapPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                        const geometry_msgs::TwistStamped& velocity,
                                                        geometry_msgs::TwistStamped &cmd_vel,
                                                        std::string &message)
    {
        boost::mutex::scoped_lock cfg_lock(config_mutex_);

        // ROS_INFO_STREAM("[DynamicGapPlanner::computeVelocityCommands(long)]");

        if (!planner_.initialized())
        {
            planner_.initialize(name_); // nh_
            ROS_WARN_STREAM_NAMED("DynamicGapPlanner", "computeVelocity called before initializing planner");
        }

        planner_.setReachedGlobalGoal(false);

        planner_.setParams(estParams_, ctrlParams_);

        Trajectory localTrajectory;
        int trajFlag; 
        planner_.runPlanningLoop(localTrajectory, trajFlag);

        if (planner_.isGoalReached())
        {
            cmd_vel.twist = geometry_msgs::Twist();
            return mbf_msgs::ExePathResult::SUCCESS;
        }

        geometry_msgs::Twist cmdVelNoStamp = planner_.ctrlGeneration(localTrajectory.getPathOdomFrame(), trajFlag);

        cmd_vel.twist = cmdVelNoStamp;

        bool acceptedCmdVel = planner_.recordAndCheckVel(cmdVelNoStamp, trajFlag);

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