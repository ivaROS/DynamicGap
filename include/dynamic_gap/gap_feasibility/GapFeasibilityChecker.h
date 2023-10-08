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

namespace dynamic_gap {
    class GapFeasibilityChecker {
        public: 
            GapFeasibilityChecker(){};
            ~GapFeasibilityChecker(){};

            GapFeasibilityChecker(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg;};
            GapFeasibilityChecker& operator=(GapFeasibilityChecker & other) {cfg_ = other.cfg_; return *this; };
            GapFeasibilityChecker(const GapFeasibilityChecker &t) {cfg_ = t.cfg_;};

            bool indivGapFeasibilityCheck(dynamic_gap::Gap& gap);
            double gapSplinecheck(dynamic_gap::Gap & gap, dynamic_gap::Estimator*, dynamic_gap::Estimator*);
            double indivGapFindCrossingPoint(dynamic_gap::Gap & gap, Eigen::Vector2f& gap_crossing_point, dynamic_gap::Estimator*, dynamic_gap::Estimator*);
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_);
            void generateTerminalPoints(dynamic_gap::Gap & gap, double terminal_beta_left, double terminal_reciprocal_range_left, 
                                                                double terminal_beta_right, double terminal_reciprocal_range_right);
        private:
            boost::shared_ptr<sensor_msgs::LaserScan const> msg;
            const DynamicGapConfig* cfg_;
            int num_of_scan;
            boost::mutex egolock;
            double generateCrossedGapTerminalPoints(double t, dynamic_gap::Gap & gap, dynamic_gap::Estimator* left_model, dynamic_gap::Estimator* right_model);

    };
}