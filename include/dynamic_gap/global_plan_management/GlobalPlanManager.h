#pragma once

#include <ros/ros.h>
#include <math.h>
#include <vector>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>

namespace dynamic_gap
{
    /**
    * \brief Class responsible for parsing incoming global plans to select local waypoints along the global plan
    * that we aim to reach through our local planning
    */
    class GlobalPlanManager
    {
        public: 
            GlobalPlanManager(const dynamic_gap::DynamicGapConfig& cfg) { cfg_ = &cfg; }

            /**
            * \brief parse global path to obtain local waypoint along global path
            * that we will try to move towards with our local path
            * \param map2rbt transformation from map frame to robot frame
            */
            void generateGlobalPathLocalWaypoint(const geometry_msgs::TransformStamped & map2rbt);
            
            /**
            * \brief return local waypoint along global path in odometry frame
            * \param rbt2odom transformation from robot frame and odometry frame
            * \return local waypoint along global path in odometry frame
            */            
            geometry_msgs::PoseStamped getGlobalPathLocalWaypointOdomFrame(const geometry_msgs::TransformStamped & rbt2odom);

            /**
            * \brief return local waypoint along global path in robot frame
            * \return local waypoint along global path in robot frame
            */  
            geometry_msgs::PoseStamped getGlobalPathLocalWaypointRobotFrame() { return globalPathLocalWaypointRobotFrame_; };
            
            /**
            * \brief return global path in odometry frame
            * \return global path in odometry frame
            */  
            std::vector<geometry_msgs::PoseStamped> getGlobalPathOdomFrame();

            /**
            * \brief extract portion of global plan (in robot frame) that lies within the current laser scan
            * \param map2rbt transformation from map frame to robot frame
            * \return visible portion of global plan in robot frame
            */  
            std::vector<geometry_msgs::PoseStamped> getVisibleGlobalPlanSnippetRobotFrame(const geometry_msgs::TransformStamped & map2rbt);

            /**
            * \brief receive new global plan in map frame and update member variable accordingly
            * \param globalPlanMapFrame new global plan in map frame
            */
            void updateGlobalPathMapFrame(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame);

            /**
            * \brief receive new laser scan and update member variable accordingly
            * \param scan new laser scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);

        private:
            /**
            * \brief helper function for search that checks if distance is less than or equal to zero
            * \param dist queried distance
            * \return boolean for if dist <= 0 
            */
            bool isNegative(const float dist); // can not pass in const reference for some reason

            /**
            * \brief helper function for returning norm of 2D position vector of pose
            * \param pose queried pose
            * \return pose norm 
            */            
            float poseNorm(const geometry_msgs::PoseStamped & pose);

            /**
            * \brief helper function for returning orientation of 2D position vector of pose
            * \param pose queried pose
            * \return pose orientation
            */     
            float getPoseOrientation(const geometry_msgs::PoseStamped & pose);

            /**
            * \brief get range of current scan along bearing of passed in pose
            * \param pose queried pose
            * \return scan range at pose bearing
            */            
            float calculateScanRangesAtPlanIndices(const geometry_msgs::PoseStamped & pose);
            
            /**
            * \brief get idx of current scan along bearing of passed in pose
            * \param pose queried pose
            * \return scan idx at pose bearing
            */                
            int poseIdxInScan(const geometry_msgs::PoseStamped & pose);

            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */

            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */

            boost::mutex goalSelectMutex_; /**< mutex locking thread for goal selection updates */
            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */
            boost::mutex globalPlanMutex_; /**< mutex locking thread for updating current global plan */

            std::vector<geometry_msgs::PoseStamped> globalPlanMapFrame_; /**< Current global plan in map frame */
            geometry_msgs::PoseStamped globalPathLocalWaypointRobotFrame_; /**< Current local waypoint along global plan in robot frame */
    };
}