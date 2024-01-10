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
    class TrajectoryScorer
    {
        public:
        TrajectoryScorer(ros::NodeHandle & nh, const dynamic_gap::DynamicGapConfig& cfg);
        // TrajectoryScorer& operator=(TrajectoryScorer other) {cfg_ = other.cfg_; return *this;};
        // TrajectoryScorer(const TrajectoryScorer &t) {cfg_ = t.cfg_;};
        
        void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);
        void updateStaticEgoCircle(const sensor_msgs::LaserScan & staticScan);
        void transformGlobalPathLocalWaypointToRbtFrame(const geometry_msgs::PoseStamped & globalPathLocalWaypointOdomFrame, 
                                                        const geometry_msgs::TransformStamped & odom2rbt);

        std::vector<float> scoreTrajectory(const dynamic_gap::Trajectory & traj,
                                            const std::vector<dynamic_gap::Gap *> & rawGaps,
                                            const std::vector<sensor_msgs::LaserScan> & futureScans);
        
        void recoverDynamicEgoCircle(const float & t_i, 
                                     const float & t_iplus1, 
                                     std::vector<Eigen::Vector4f> & propagatedAgents,
                                     sensor_msgs::LaserScan & dynamicLaserScan,
                                     const bool & print);
        

        private:
            // int signum(float dy);
            float terminalGoalCost(const geometry_msgs::Pose & pose);
            
            float dist2Pose(const float & theta, 
                            const float & dist, 
                            const geometry_msgs::Pose & pose);

            float scorePose(const geometry_msgs::Pose & pose);
            float chapterScore(const float & d);

            float dynamicScorePose(const geometry_msgs::Pose & pose, const float & theta, const float & range);
            float dynamicChapterScore(const float & d);

            void visualizePropagatedEgocircle(const sensor_msgs::LaserScan & dynamicLaserScan);

            std::vector< std::vector<float> > sortAndPrune(const std::vector<Eigen::Vector4f> & agentPoses);            

            const DynamicGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_;
            sensor_msgs::LaserScan staticScan_;
            geometry_msgs::PoseStamped globalPathLocalWaypointRobotFrame_;
            boost::mutex globalPlanMutex_, egocircleMutex_;
            ros::Publisher propagatedEgocirclePublisher_;
    };
}