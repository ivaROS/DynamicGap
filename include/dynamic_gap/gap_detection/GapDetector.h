#pragma once

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_gap/utils/Gap.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>
#include <dynamic_gap/config/DynamicGapConfig.h>

namespace dynamic_gap
{
	class GapDetector
    {
        public:
            GapDetector(const DynamicGapConfig& cfg) { cfg_ = &cfg; }

            GapDetector& operator=(GapDetector other) {cfg_ = other.cfg_; return *this; }

            GapDetector(const GapDetector &t) {cfg_ = t.cfg_;}

            std::vector<dynamic_gap::Gap> gapDetection(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
                                                        geometry_msgs::PoseStamped final_goal_rbt);
        
            std::vector<dynamic_gap::Gap> gapSimplification(const std::vector<dynamic_gap::Gap> & raw_gaps);     

        private:
            bool isFinite(float scan_dist);

            bool sweptGapStartedOrEnded(float scan_dist_i, float scan_dist_imin1);

            bool sweptGapSizeCheck(const dynamic_gap::Gap & detected_gap);

            bool radialGapSizeCheck(float scan_dist_i, float scan_dist_imin1, 
                                    float gap_angle);

            bool bridgeCondition(const std::vector<dynamic_gap::Gap> & raw_gaps);

            bool isGlobalGoalWithinGap(geometry_msgs::PoseStamped final_goal_rbt, 
                                        int & final_goal_idx);
            
            bool mergeSweptGapCondition(const dynamic_gap::Gap & raw_gap, 
                                        const std::vector<dynamic_gap::Gap> & simplified_gaps);

            int checkSimplifiedGapsMergeability(const dynamic_gap::Gap & raw_gap, 
                                                const std::vector<dynamic_gap::Gap> & simplified_gaps);

            void addGapForGlobalGoal(int final_goal_idx, std::vector<dynamic_gap::Gap> & raw_gaps);   

            sensor_msgs::LaserScan scan_;
            const DynamicGapConfig* cfg_;
            float minScanDist_, maxScanDist_;
            float halfScanRayCount_;
            int fullScanRayCount_; 

    };
}