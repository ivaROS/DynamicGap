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
    Eigen::Vector2d pol2car(Eigen::Vector2d polar_vector);
    double atanThetaWrap(double theta); 
    float atanThetaWrap(float theta);
    double getLeftToRightAngle(Eigen::Vector2d left_norm_vect, Eigen::Vector2d right_norm_vect);
    float getLeftToRightAngle(Eigen::Vector2f, Eigen::Vector2f, bool wrap);

    class Utils 
    {
        public: 
            Utils(const DynamicGapConfig& cfg) { cfg_ = &cfg; }

            Utils& operator=(Utils other) {cfg_ = other.cfg_; return *this; }

            Utils(const Utils &t) {cfg_ = t.cfg_;}
            std::vector<Eigen::Matrix<double, 4, 1> > getCurrAgents();

            sensor_msgs::LaserScan staticDynamicScanSeparation(const std::vector<dynamic_gap::Gap> & observed_gaps, 
                                                                boost::shared_ptr<sensor_msgs::LaserScan const> msg,
                                                                bool print);

        private:
            const DynamicGapConfig* cfg_;
            std::vector<Eigen::Matrix<double, 4, 1> > curr_agents;
    };


}