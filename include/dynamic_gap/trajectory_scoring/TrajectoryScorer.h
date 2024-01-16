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
    * Class responsible for scoring candidate trajectory according to
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
            
            /**
            * \brief Function for propagating agents forward in time and populating propagated laser scan
            * \param t_i current time step
            * \param t_iplus1 next time step
            * \param propagatedAgents set of estimated agents to propagate forward in time
            * \param dynamicLaserScan propagated laser scan to populate
            */
            void recoverDynamicEgoCircle(const float & t_i, 
                                        const float & t_iplus1, 
                                        std::vector<Eigen::Vector4f> & propagatedAgents,
                                        sensor_msgs::LaserScan & dynamicLaserScan);

        private:
            /**
            * \brief function for evaluating terminal waypoint cost for candidate trajectory
            * \param pose final pose in candidate trajectory to check against terminal waypoint
            * \return terminal waypoint cost for candidate trajectory
            */
            float terminalGoalCost(const geometry_msgs::Pose & pose);
            
            /**
            * \brief function for calculating distance between trajectory pose and (range, theta) pair in laser scan
            * \param theta theta value of queried laser scan point
            * \param range range value of queried laser scan point
            * \param pose pose in candidate trajectory to calculate distance with
            * \return distance from trajectory pose to (range, theta) pair in laser scan
            */
            float dist2Pose(const float & theta, 
                            const float & range, 
                            const geometry_msgs::Pose & pose);

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

            /**
            * \brief function for publishing propagated laser scans
            * \param dynamicLaserScan propagated laser scan to publish in RViz
            */
            void visualizePropagatedEgocircle(const sensor_msgs::LaserScan & dynamicLaserScan);

            /**
            * \brief function for sorting through current set of agents and pruning those outside of current laser scan
            * \param agentPoses set of current agents in environment
            * \return sorted and pruned set of current agents in environment
            */
            std::vector<Eigen::Vector4f> sortAndPrune(const std::vector<Eigen::Vector4f> & agentPoses);

            boost::mutex globalPlanMutex_; /**< mutex locking thread for updating current global plan */
            boost::mutex scanMutex_; /**< mutex locking thread for updating current scan */
            
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_; /**< Current laser scan */
            const DynamicGapConfig * cfg_ = NULL; /**< Planner hyperparameter config list */

            // sensor_msgs::LaserScan staticScan_;
            geometry_msgs::PoseStamped globalPathLocalWaypointRobotFrame_; /**< Current local waypoint along global plan in robot frame */

            ros::Publisher propagatedEgocirclePublisher_; /**< Publisher for propagated egocircle */
    };
}