#ifndef GAP_FEASIBILITY_H
#define GAP_FEASIBILITY_H


#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/gap.h>
#include <dynamic_gap/dynamicgap_config.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
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
            GapFeasibilityChecker& operator=(GapFeasibilityChecker & other) {cfg_ = other.cfg_;};
            GapFeasibilityChecker(const GapFeasibilityChecker &t) {cfg_ = t.cfg_;};


            bool indivGapFeasibilityCheck(dynamic_gap::Gap& gap);
            bool feasibilityCheck(dynamic_gap::Gap& gap, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model);
            double gapSplinecheck(dynamic_gap::Gap & gap, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model);
            double indivGapFindCrossingPoint(dynamic_gap::Gap & gap, Eigen::Vector2f& gap_crossing_point, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model);
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_);

        private:
            boost::shared_ptr<sensor_msgs::LaserScan const> msg;
            const DynamicGapConfig* cfg_;
            int num_of_scan;
            boost::mutex egolock;
    };
}

#endif