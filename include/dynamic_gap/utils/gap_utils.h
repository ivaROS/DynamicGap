#pragma once

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_gap/utils/Gap.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>
#include <dynamic_gap/config/DynamicGapConfig.h>

namespace dynamic_gap {
    class GapUtils 
    {
        public: 
            GapUtils(const DynamicGapConfig& cfg) { cfg_ = &cfg; }

            GapUtils& operator=(GapUtils other) {cfg_ = other.cfg_; return *this; }

            GapUtils(const GapUtils &t) {cfg_ = t.cfg_;}

            std::vector<Eigen::Matrix<double, 4, 1> > getCurrAgents();

            sensor_msgs::LaserScan staticDynamicScanSeparation(std::vector<dynamic_gap::Gap> observed_gaps, 
                                                                boost::shared_ptr<sensor_msgs::LaserScan const> msg,
                                                                bool print);

        private:
            const DynamicGapConfig* cfg_;
            std::vector<Eigen::Matrix<double, 4, 1> > curr_agents;

    };


}