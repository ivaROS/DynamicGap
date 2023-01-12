#include <ros/ros.h>
#include <dynamic_gap/dynamic_gap.h>
#include <dynamic_gap/gap.h>
#include <pluginlib/class_list_macros.h>

// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>

// using namespace boost::numeric::odeint;
namespace pl = std::placeholders;

#define JANKY_PID false
#define NEAR_IDENTITY true

PLUGINLIB_EXPORT_CLASS(dynamic_gap::DynamicGapPlanner, nav_core::BaseLocalPlanner)

namespace dynamic_gap 
{
    DynamicGapPlanner::DynamicGapPlanner(){}

    DynamicGapPlanner::~DynamicGapPlanner()
    {
        ROS_INFO_STREAM("Planner terminated");
    }

    bool DynamicGapPlanner::isGoalReached()
    {
        return planner.isGoalReached();
    }

    bool DynamicGapPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> & plan)
    {
        ROS_INFO_STREAM("DynamicGap: setPlan");
        return planner.setGoal(plan);
    }

    void DynamicGapPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // ROS_INFO_STREAM("DynamicGap: initialize");
        planner_name = name;
        // pnh: planner node handle?
        ros::NodeHandle pnh("~/" + planner_name);
        planner.initialize(pnh);

        laser_sub = pnh.subscribe("/point_scan", 5, &Planner::laserScanCB, &planner);
        
        std::string robot_name = "/robot" + std::to_string(planner.get_num_obsts());
        // static_laser_sub = pnh.subscribe("/static_point_scan", 1, &Planner::staticLaserScanCB, &planner);
        
        // queue needs to be 3 to not skip any messages
        // pose_sub = pnh.subscribe("/odom", 3, &Planner::poseCB, &planner);
        // rbt_accel_sub = nh.subscribe(robot_name + "/acc", 3, &Planner::robotAccCB, &planner);

        odom_sub.subscribe(nh, "/odom", 3);
        acc_sub.subscribe(nh, robot_name + "/acc", 3);
        sync_.reset(new Sync(MySyncPolicy(10), odom_sub, acc_sub));
        sync_->registerCallback(boost::bind(&Planner::jointPoseAccCB, &planner, _1, _2));

        for (int i = 0; i < planner.get_num_obsts(); i++) {
            ros::Subscriber temp_odom_sub = nh.subscribe("/robot" + std::to_string(i) + "/odom", 3, &Planner::agentOdomCB, &planner);
            agent_odom_subscribers.push_back(temp_odom_sub);
        }

        initialized = true;

        // Setup dynamic reconfigure
        dynamic_recfg_server = boost::make_shared<dynamic_reconfigure::Server <dynamic_gap::dgConfig> > (pnh);
        f = boost::bind(&dynamic_gap::Planner::rcfgCallback, &planner, _1, _2);
        dynamic_recfg_server->setCallback(f);
    }

    bool DynamicGapPlanner::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
    {
        if (!planner.initialized())
        {
            ros::NodeHandle pnh("~/" + planner_name);
            planner.initialize(pnh);
            ROS_WARN_STREAM("computeVelocity called before initializing planner");
        }

        auto final_traj = planner.getPlanTrajectory();

        cmd_vel = planner.ctrlGeneration(final_traj);

        return planner.recordAndCheckVel(cmd_vel);
    }

    void DynamicGapPlanner::reset()
    {
        planner.reset();
        return;
    }

}