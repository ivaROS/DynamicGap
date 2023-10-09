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
    Eigen::Vector2f pol2car(Eigen::Vector2f polar_vector);
    float atanThetaWrap(float theta); 
    // float atanThetaWrap(float theta);
    float getLeftToRightAngle(Eigen::Vector2f left_norm_vect, Eigen::Vector2f right_norm_vect);
    float getLeftToRightAngle(Eigen::Vector2f, Eigen::Vector2f, bool wrap);

    float idx2theta(const int idx);
    int theta2idx(const float theta);

    static int half_num_scan = 256;
    static float angle_increment = 0.0122959f;

    class Utils 
    {
        public: 
            Utils(const DynamicGapConfig& cfg) { cfg_ = &cfg; half_num_scan = cfg_->rbt.half_num_scan; }

            Utils& operator=(Utils other) {cfg_ = other.cfg_; return *this; }

            Utils(const Utils &t) {cfg_ = t.cfg_;}
            std::vector<Eigen::Matrix<float, 4, 1> > getCurrAgents();

            sensor_msgs::LaserScan staticDynamicScanSeparation(const std::vector<dynamic_gap::Gap> & observed_gaps, 
                                                                boost::shared_ptr<sensor_msgs::LaserScan const> msg,
                                                                bool print);

        private:
            const DynamicGapConfig* cfg_;
            std::vector<Eigen::Matrix<float, 4, 1> > curr_agents;
    };


}