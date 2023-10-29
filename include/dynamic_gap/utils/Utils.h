#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/shared_ptr.hpp>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <tf/tf.h>

namespace dynamic_gap 
{
    Eigen::Vector2f pol2car(const Eigen::Vector2f & polar_vector);
    float atanThetaWrap(float theta); 
    // float atanThetaWrap(float theta);
    float getLeftToRightAngle(const Eigen::Vector2f & left_norm_vect, const Eigen::Vector2f & right_norm_vect);
    float getLeftToRightAngle(const Eigen::Vector2f & left_norm_vect, const Eigen::Vector2f & right_norm_vect, bool wrap);

    float idx2theta(const int idx);
    int theta2idx(const float theta);

    float getGapDist(const Eigen::Vector4f & gapState);
    float getGapBearing(const Eigen::Vector4f & gapState);
    float getGapBearingRateOfChange(const Eigen::Vector4f & gapState);

    // float lawOfCosinesDistance()

    float quaternionToYaw(const tf::Quaternion & quat);

    int subtract_wrap(int a, int b);
    bool isGapLocalGoalWithin(int goal_idx, int idx_lower, int idx_upper, int full_scan);
    int signum(float value);

    static int half_num_scan = 256;
    static float angle_increment = (2*M_PI) / 511; //  0.0122959f;
}