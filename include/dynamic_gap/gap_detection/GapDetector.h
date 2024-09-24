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
    /** 
    * \brief Class responsible for detecting raw set of gaps from incoming laser scan
    * and simplifying raw set of gaps into simplified set of gaps.
    */
	class GapDetector
    {
        public:
            /** 
            * \brief Constructor with planner config 

            * \param cfg config file for planner parameters
            */
            GapDetector(const DynamicGapConfig& cfg) { cfg_ = &cfg; }

            /**
            * \brief Detect raw set of gaps from incoming laser scan.
            * 
            * \param scanPtr pointer to incoming laser scan
            * \param globalGoalRbtFrame global goal pose in robot frame
            * \return raw set of gaps
            */
            std::vector<dynamic_gap::Gap *> gapDetection(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr, 
                                                        const geometry_msgs::PoseStamped & globalGoalRbtFrame);
        
            /**
            * \brief Condense raw set of gaps into a smaller set of simplified gaps more amenable for navigation.
            * 
            * \param rawGaps set of raw gaps
            * \return set of simplified gaps
            */        
            std::vector<dynamic_gap::Gap *> gapSimplification(const std::vector<dynamic_gap::Gap *> & rawGaps);     

        private:
            /**
            * \brief Check if scan range registers an object (finite range value)
            * 
            * \param range incoming scan range value
            * \return boolean if range is finite or not
            */        
            bool isFinite(const float & range);

            /**
            * \brief Determining if swept gap has 
            * either started (finite scan --> infinite scan)
            * or ended (infinite scan --> finite scan)
            * 
            * \param currRange current scan range value
            * \param prevRange previous scan range value
            * \return boolean if a swept gap has either started or ended at current scan index
            */      
            bool sweptGapStartedOrEnded(const float & currRange, 
                                        const float & prevRange);

            /**
            * \brief Checking if gap is large enough to be classified as a swept gap
            
            * Checking if gap is either very large, 
            * or if robot can fit within gap (precondition to swept gap)
            * \param gap queried gap
            * \return boolean if gap should be classified as swept
            */
            bool sweptGapSizeCheck(dynamic_gap::Gap * gap);

            /**
            * \brief Checking if gap should be classified as radial 
            
            * Checking if robot can fit between 
            * consecutive scan points (precondition to radial gap)
            * \param currRange current scan range value
            * \param prevRange previous scan range value
            * \param gapAngle angle between consecutive scan points
            * \return boolean if gap should be classified as radial 
            */
            bool radialGapSizeCheck(const float & currRange, 
                                    const float & prevRange, 
                                    const float & gapAngle);

            /**
            * \brief Checking if first and last raw gaps should be merged together
            * 
            * \param rawGaps raw set of gaps
            * \return boolean if first and last raw gaps should be merged together
            */
            bool bridgeCondition(const std::vector<dynamic_gap::Gap *> & rawGaps);

            /**
            * \brief Check if a raw swept gap should be merged into a simplified swept gap
            * or if should exist on its own.
            * 
            * \param rawGap queried raw swept gap
            * \param simplifiedGaps existing set of simplified gaps
            * \return boolean for if raw gap should be merged or not
            */
            bool mergeSweptGapCondition(dynamic_gap::Gap * rawGap, 
                                        const std::vector<dynamic_gap::Gap *> & simplifiedGaps);

            /**
            * \brief Iterate backwards through simplified gaps to see if/where
            * a raw radial gap should be merged 
            *
            * \param rawGap queried raw radial gap
            * \param simplifiedGaps existing set of simplified gaps
            * \return index within simplified gaps that should be merged
            */
            int checkSimplifiedGapsMergeability(dynamic_gap::Gap * rawGap, 
                                                const std::vector<dynamic_gap::Gap *> & simplifiedGaps);

            sensor_msgs::LaserScan scan_; /**< Current laser scan */
            const DynamicGapConfig* cfg_ = NULL; /**< Planner hyperparameter config list */
            float minScanDist_ = 0.0; /**< Minimum distance within current laser scan */
            float maxScanDist_ = 0.0; /**< Maximum distance within current laser scan */
            float halfScanRayCount_ = 0.0; /**< Half of number of rays within scan (float) */
            int fullScanRayCount_ = 0; /**< Number of rays within scan (int) */

    };
}