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

            std::vector<dynamic_gap::Gap *> gapDetection(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr, 
                                                            geometry_msgs::PoseStamped globalGoalRbtFrame);
        
            std::vector<dynamic_gap::Gap *> gapSimplification(const std::vector<dynamic_gap::Gap *> & rawGaps);     

        private:
            bool isFinite(float rayDist);

            bool sweptGapStartedOrEnded(float currRayDist, float prevRayDist);

            bool sweptGapSizeCheck(dynamic_gap::Gap * gap);

            bool radialGapSizeCheck(float currRayDist, float prevRayDist, float gapAngle);

            bool bridgeCondition(const std::vector<dynamic_gap::Gap *> & rawGaps);

            bool isGlobalGoalWithinGap(const geometry_msgs::PoseStamped & globalGoalRbtFrame,
                                            int & globalGoalScanIdx);

            void addGapForGlobalGoal(int globalGoalScanIdx, std::vector<dynamic_gap::Gap *> & rawGaps);   

            bool mergeSweptGapCondition(dynamic_gap::Gap * rawGap, 
                                        const std::vector<dynamic_gap::Gap *> & simplifiedGaps);

            int checkSimplifiedGapsMergeability(dynamic_gap::Gap * rawGap, 
                                                const std::vector<dynamic_gap::Gap *> & simplifiedGaps);

            sensor_msgs::LaserScan scan_;
            const DynamicGapConfig* cfg_;
            float minScanDist_, maxScanDist_;
            float halfScanRayCount_;
            int fullScanRayCount_; 

    };
}