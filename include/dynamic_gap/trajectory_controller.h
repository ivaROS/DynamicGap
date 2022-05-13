#ifndef TRAJ_CONT_H
#define TRAJ_CONT_H

#include <ros/ros.h>
#include <dynamic_gap/dynamicgap_config.h>
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
#include "dynamic_gap/TrajPlan.h"
#include <dynamic_gap/gap_trajectory_generator.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

namespace dynamic_gap {
    class TrajectoryController {
        public:

            TrajectoryController(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            geometry_msgs::Twist controlLaw(geometry_msgs::Pose, nav_msgs::Odometry, sensor_msgs::LaserScan, geometry_msgs::PoseStamped);
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg);
            int targetPoseIdx(geometry_msgs::Pose curr_pose, dynamic_gap::TrajPlan ref_pose);
            dynamic_gap::TrajPlan trajGen(geometry_msgs::PoseArray);

        private:
            Eigen::Matrix2cd getComplexMatrix(double, double, double, double);
            Eigen::Matrix2cd getComplexMatrix(double, double, double);
            double dist2Pose(float theta, float dist, geometry_msgs::Pose pose);

            std::vector<geometry_msgs::Point> findLocalLine(int idx);
            double polDist(float l1, float t1, float l2, float t2);

            bool leqThres(const double dist);
            bool geqThres(const double dist);


            Eigen::Vector2d car2pol(Eigen::Vector2d a);
            Eigen::Vector2d pol2car(Eigen::Vector2d a);
            Eigen::Vector3d projection_method(float min_diff_x, float min_diff_y);

            double thres;
            const DynamicGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> msg_;
            boost::mutex egocircle_l;
            ros::Publisher projection_viz;
            ros::Time last_time;
    };
}

#endif