#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Trajectory.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <vector>
#include <numeric>
// #include <map>
// #include <visualization_msgs/MarkerArray.h>
// #include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
// #include <omp.h>
#include <boost/thread/mutex.hpp>
// #include "tf/transform_datatypes.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace dynamic_gap
{
    /**
    * \brief Class responsible for scoring candidate trajectory according to
    * trajectory's proximity to local environment and global path's local waypoint
    */
    class TrajectoryScorer
    {
        public:
            TrajectoryScorer(ros::NodeHandle & nh, const dynamic_gap::DynamicGapConfig& cfg);
            // TrajectoryScorer& operator=(TrajectoryScorer other) {cfg_ = other.cfg_; return *this;};
            // TrajectoryScorer(const TrajectoryScorer &t) {cfg_ = t.cfg_;};
            
            /**
            * \brief receive new laser scan and update member variable accordingly
            * \param scan new laser scan
            */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);
            
            // void updateStaticEgoCircle(const sensor_msgs::LaserScan & staticScan);
            
            /**
            * \brief Helper function for transforming global path local waypoint into robot frame
            * \param globalPathLocalWaypointOdomFrame Current local waypoint along global plan in robot frame
            * \param odom2rbt transformation from odom frame to robot frame
            */
            void transformGlobalPathLocalWaypointToRbtFrame(const geometry_msgs::PoseStamped & globalPathLocalWaypointOdomFrame, 
                                                            const geometry_msgs::TransformStamped & odom2rbt);

            /**
            * \brief Function for evaluating pose-wise scores along candidate trajectory
            * \param traj candidate trajectory to score
            * \param futureScans set of propagated laser scans that we will use to evaluate each trajectory pose
            * \return vector of pose-wise scores along candidate trajecory
            */
            std::vector<float> scoreTrajectory(const dynamic_gap::Trajectory & traj,
                                                const std::vector<sensor_msgs::LaserScan> & futureScans);
            
        private:
            /**
            * \brief function for evaluating terminal waypoint cost for candidate trajectory
            * \param pose final pose in candidate trajectory to check against terminal waypoint
            * \return terminal waypoint cost for candidate trajectory
            */
            float terminalGoalCost(const geometry_msgs::Pose & pose);

            /**
            * \brief function for evaluating intermediate cost of pose for candidate trajectory (in static environment)
            * \param pose pose within candidate trajectory to evaluate
            * \return intermediate cost of pose
            */
            float scorePose(const geometry_msgs::Pose & pose);
            
            /**
            * \brief function for calculating intermediate trajectory cost (in static environment)
            * \param rbtToScanDist minimum distance from robot pose to current scan
            * \return intermediate cost of pose
            */
            float chapterScore(const float & rbtToScanDist);

            /**
            * \brief function for evaluating intermediate cost of pose for candidate trajectory (in dynamic environment)
            * \param pose pose in candidate trajectory to calculate distance with            
            * \param theta theta value of queried laser scan point
            * \param range range value of queried laser scan point
            * \return intermediate cost of pose            
            */
            float dynamicScorePose(const geometry_msgs::Pose & pose, const float & theta, const float & range);

            /**
            * \brief function for calculating intermediate trajectory cost (in dynamic environment)
            * \param rbtToScanDist minimum distance from robot pose to current scan
            * \return intermediate cost of pose
            */            
            float dynamicChapterScore(const float & rbtToScanDist);

            boost::mutex globalPlanMutex_; /**< mutex locking thread for updating current global plan */
            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */
            
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */
            const DynamicGapConfig * cfg_ = NULL; /**< Planner hyperparameter config list */

            // sensor_msgs::LaserScan staticScan_;
            geometry_msgs::PoseStamped globalPathLocalWaypointRobotFrame_; /**< Current local waypoint along global plan in robot frame */
    };
}