#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <vector>
// #include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

namespace dynamic_gap 
{
    class GapFeasibilityChecker 
    {
        public: 
            GapFeasibilityChecker(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg;};
            GapFeasibilityChecker& operator=(GapFeasibilityChecker & other) {cfg_ = other.cfg_; return *this; };
            GapFeasibilityChecker(const GapFeasibilityChecker &t) {cfg_ = t.cfg_;};

            bool indivGapFeasibilityCheck(dynamic_gap::Gap * gap);
            // void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_);
            
        private:
            float gapSplinecheck(dynamic_gap::Gap * gap, 
                                    dynamic_gap::Estimator* leftGapPtModel, 
                                    dynamic_gap::Estimator* rightGapPtModel);
            float generateCrossedGapTerminalPoints(float t, dynamic_gap::Gap * gap, 
                                                    dynamic_gap::Estimator* leftGapPtModel, 
                                                    dynamic_gap::Estimator* rightGapPtModel);
            void generateTerminalPoints(dynamic_gap::Gap * gap, 
                                        const Eigen::Vector4f & leftCrossPt,
                                        const Eigen::Vector4f & rightCrossPt);
                                        // float terminal_thetaLeft, float terminal_reciprocal_range_left, 
                                        // float terminal_thetaRight, float terminal_reciprocal_range_right);
            float indivGapFindCrossingPoint(dynamic_gap::Gap * gap, 
                                            Eigen::Vector2f & gap_crossing_point, 
                                            dynamic_gap::Estimator* leftGapPtModel, 
                                            dynamic_gap::Estimator* rightGapPtModel);

            // boost::shared_ptr<sensor_msgs::LaserScan const> msg;
            // float scan_angle_min_, scan_angle_increment_;
            const DynamicGapConfig* cfg_;
            // int num_of_scan;
            // boost::mutex egolock;

    };
}