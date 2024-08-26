#pragma once

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

// #include <navfn/navfn_ros.h>
// #include <boost/shared_ptr.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_gap/Planner.h>

namespace dynamic_gap 
{
    /**
    * \brief Class that we will write as our local path planner 
    * plugin for the move_base package
    */
    class DynamicGapPlanner : public nav_core::BaseLocalPlanner 
    {
        public: 

            /**
            * \brief Use local path planner to compute next command velocity
            * \param cmdVel command velocity to update
            * \return boolean for if command velocity was successfully computed
            */
            bool computeVelocityCommands(geometry_msgs::Twist & cmdVel);

            /**
            * \brief Check if global goal has been reached by robot
            * \return boolean for if global goal has been reached or not
            */
            bool isGoalReached();

            /**
            * \brief update current global plan in map frame
            * \param globalPlanMapFrame incoming global plan in map frame to update with
            * \return boolean for if global plan update succeeded
            */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame);

            /**
            * \brief initialize local path planner plugin
            * \param name name of the local path planner
            * \param tf ROS transform buffer
            * \param costmap incoming local 2D costmap
            */
            void initialize(std::string name, 
                            tf2_ros::Buffer * tf, 
                            costmap_2d::Costmap2DROS * costmap);

            /**
            * Function for resetting planner in case of planning failure
            */
            void reset();

        private:
            dynamic_gap::Planner planner_; /**< Local path planner object */
            std::string plannerName_; /**< Local path planner name */
            ros::NodeHandle nh_; /**< ROS node handle for local path planner */

            ros::Subscriber laserSub_; /**< Subscriber to incoming laser scan */

            std::vector<ros::Subscriber> agentPoseSubs_; /**< Subscribers for agent poses */

            message_filters::Subscriber<nav_msgs::Odometry> rbtPoseSub_; /**< Subscriber to incoming robot pose */
            message_filters::Subscriber<geometry_msgs::TwistStamped> rbtAccSub_; /**< Subscriber to incoming robot acceleration */

            typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, geometry_msgs::TwistStamped> rbtPoseAndAccSyncPolicy; /**< Custom synchronization policy for robot pose and acceleration messages */
            typedef message_filters::Synchronizer<rbtPoseAndAccSyncPolicy> CustomSynchronizer; /**< Custom synchronizer for robot pose and acceleration messages */
            boost::shared_ptr<CustomSynchronizer> sync_; /**< Shared pointer to custom synchronizer */
    };
}