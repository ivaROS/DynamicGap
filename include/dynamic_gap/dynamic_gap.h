#ifndef DYNAMIC_GAP_H
#define DYNAMIC_GAP_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
// #include <sensor_msgs/LaserScan.h>
// #include <std_msgs/Header.h>
#include <navfn/navfn_ros.h>
#include <boost/shared_ptr.hpp>

// #include <geometry_msgs/PoseArray.h>

// #include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
// #include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/numeric/odeint.hpp>

#include <dynamic_gap/planner.h>
#include <dynamic_gap/gap.h>
// #include <dynamic_gap/helper.h>
#include <dynamic_gap/dgConfig.h>

#include <dynamic_reconfigure/server.h>

namespace dynamic_gap {

    class DynamicGapPlanner : public nav_core::BaseLocalPlanner 
    {
        public: 

            DynamicGapPlanner();

            ~DynamicGapPlanner();

            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

            bool isGoalReached();

            bool setPlan(const std::vector<geometry_msgs::PoseStamped> & plan);

            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

            void reset();

        private:
            // dynamic_gap::dgConfig loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

            dynamic_gap::Planner planner;
            std::string planner_name;
            ros::NodeHandle nh, pnh;

            ros::Subscriber laser_sub, static_laser_sub;
            ros::Subscriber pose_sub;
            ros::Subscriber rbt_accel_sub;      
            std::vector<ros::Subscriber> agent_odom_subscribers;

            message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
            message_filters::Subscriber<geometry_msgs::TwistStamped> acc_sub;

            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::TwistStamped> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> Sync;
            boost::shared_ptr<Sync> sync_;
            
            bool initialized = false;

            boost::shared_ptr<dynamic_reconfigure::Server<dynamic_gap::dgConfig> > dynamic_recfg_server;
            dynamic_reconfigure::Server<dynamic_gap::dgConfig>::CallbackType f;
    };
}

#endif 
