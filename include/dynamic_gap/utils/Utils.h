#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <sensor_msgs/LaserScan.h>
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

    float getGapDist(Eigen::Vector4f gapState);
    float getGapBearing(Eigen::Vector4f gapState);
    float getGapBearingRateOfChange(Eigen::Vector4f gapState);

    static int half_num_scan = 256;
    static float angle_increment = (2*M_PI) / 511; //  0.0122959f;
}