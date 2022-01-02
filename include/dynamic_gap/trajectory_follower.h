#ifndef TRAJ_FOL_H
#define TRAJ_FOL_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <dynamic_gap/gap.h>

namespace dynamic_gap {
    float P2EDistance(float r1, float r2, float the1, float the2);
    Eigen::Matrix2cd getComplexMatrix(double x, double y, double quat_w, double quat_z);

    Eigen::Matrix2cd getComplexMatrix(double x, double y, double theta);
    
    geometry_msgs::Twist controlLaw(geometry_msgs::Pose current, nav_msgs::Odometry desired, 
    								const sensor_msgs::LaserScan& inflated_egocircle, 
    								bool holonomic = false, bool projection_operator = false,
    								double k_turn_ = 0.5, double k_drive_x_ = 0.5, double k_drive_y_ = 3.5, double k_po_ = 1.,
    								float v_ang_const = 0, float v_lin_x_const = 0.4, float v_lin_y_const = 0.4,
    								float r_min = 0.5, float r_norm = 1., float r_norm_offset = 0.5);

    geometry_msgs::Twist controlLaw(geometry_msgs::Pose current,
        nav_msgs::Odometry desired,
        double k_turn_ = 0.5,
        double k_drive_x_ = 0.5,
        double k_drive_y_ = 3.5);

}

#endif