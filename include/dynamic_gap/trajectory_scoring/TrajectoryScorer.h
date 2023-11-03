#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
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
        TrajectoryScorer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
        TrajectoryScorer& operator=(TrajectoryScorer other) {cfg_ = other.cfg_; return *this;};
        TrajectoryScorer(const TrajectoryScorer &t) {cfg_ = t.cfg_;};
        
        void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan);
        void updateStaticEgoCircle(const sensor_msgs::LaserScan & staticScan);
        void transformGlobalPathLocalWaypointToRbtFrame(const geometry_msgs::PoseStamped & globalPathLocalWaypointOdomFrame, 
                                                        const geometry_msgs::TransformStamped & odom2rbt);

        // std::vector<float> scoreGaps();
        
        // Full Scoring
        // std::vector<float> scoreTrajectories(std::vector<geometry_msgs::PoseArray>);
        // geometry_msgs::PoseStamped getLocalGoal() {return globalPathLocalWaypointRobotFrame_; }; // in robot frame
        
        std::vector<float> scoreTrajectory(const geometry_msgs::PoseArray & path, 
                                                         const std::vector<float> & pathTiming, 
                                                         const std::vector<dynamic_gap::Gap> & rawGaps,
                                                         const std::vector<sensor_msgs::LaserScan> & futureScans);
        
        void recoverDynamicEgoCircle(float t_i, float t_iplus1, 
                                    std::vector<Eigen::Matrix<float, 4, 1> > & propagatedAgents,
                                    sensor_msgs::LaserScan & dynamicLaserScan,
                                    bool print);
        

        private:
            // int signum(float dy);
            float terminalGoalCost(const geometry_msgs::Pose & pose);
            
            float dist2Pose(float theta, float dist, const geometry_msgs::Pose & pose);

            float scorePose(const geometry_msgs::Pose & pose);
            float chapterScore(float d);

            // int dynamicGetMinDistIndex(const geometry_msgs::Pose & pose, 
            //                             const sensor_msgs::LaserScan & dynamicLaserScan, 
            //                             bool print);

            float dynamicScorePose(const geometry_msgs::Pose & pose, float theta, float range);
            float dynamicChapterScore(float d);

            void visualizePropagatedEgocircle(const sensor_msgs::LaserScan & dynamicLaserScan);

            std::vector< std::vector<float> > sortAndPrune(const std::vector<Eigen::Matrix<float, 4, 1> > & agentPoses);            

            const DynamicGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_;
            sensor_msgs::LaserScan staticScan_;
            // std::vector<dynamic_gap::Gap> gaps;
            geometry_msgs::PoseStamped globalPathLocalWaypointRobotFrame_;
            boost::mutex globalPlanMutex_, egocircleMutex_;


            // int search_idx = -1;

            ros::Publisher propagatedEgocirclePublisher_;
    };
}