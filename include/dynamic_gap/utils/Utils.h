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
    float getLeftToRightAngle(const Eigen::Vector2f & leftVect, const Eigen::Vector2f & rightVect);
    float getLeftToRightAngle(const Eigen::Vector2f & leftVect, const Eigen::Vector2f & rightVect, bool wrap);

    float idx2theta(const int idx);
    int theta2idx(const float theta);

    float getGapDist(const Eigen::Vector4f & gapState);
    float getGapBearing(const Eigen::Vector4f & gapState);
    float getGapBearingRateOfChange(const Eigen::Vector4f & gapState);

    // float lawOfCosinesDistance()

    float quaternionToYaw(const tf::Quaternion & quat);

    int subtract_wrap(int a, int b);
    bool isGlobalPathLocalWaypointWithinGapAngle(int goalIdx, int lowerIdx, int upperIdx);
    int signum(float value);

    static int half_num_scan = 256;
    static float angle_increment = (2*M_PI) / 511; //  0.0122959f;

    static Eigen::Matrix2f Rpi2 = (Eigen::Matrix2f() << 0.0, -1.0, 1.0, 0.0).finished();
    // Rpi2 << 0, -1, 1,0; 


    //  {{0.0, -1.0}, {1.0, 0.0}}; // PI/2 counter clockwise
    static Eigen::MatrixXf Rnegpi2 = (Eigen::Matrix2f() << 0.0, 1.0, -1.0, 0.0).finished();       //  = Rpi2;                // PI/2 clockwise;
    // Rnegpi2 = -Rpi2;
    // , Rnegpi2;
    // Rpi2 << 0, -1, 1,0; 
    // 
}