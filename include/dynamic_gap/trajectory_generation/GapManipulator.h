#pragma once

#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/trajectory_scoring/TrajectoryScorer.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

namespace dynamic_gap 
{
    class GapManipulator 
    {
        public: 
            GapManipulator(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg;};
            GapManipulator& operator=(GapManipulator & other) {cfg_ = other.cfg_; return *this; };
            GapManipulator(const GapManipulator &t) {cfg_ = t.cfg_;};

            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);
            void updateStaticEgoCircle(const sensor_msgs::LaserScan &);
            void updateDynamicEgoCircle(dynamic_gap::Gap * gap,
                                        const std::vector<sensor_msgs::LaserScan> &);

            void setGapWaypoint(dynamic_gap::Gap * gap, const geometry_msgs::PoseStamped & localgoal, bool initial); //, sensor_msgs::LaserScan const dynamic_laser_scan);
            void setTerminalGapWaypoint(dynamic_gap::Gap * gap, const geometry_msgs::PoseStamped & localgoal);
            
            void reduceGap(dynamic_gap::Gap * gap, const geometry_msgs::PoseStamped & localGoal, bool initial); //), sensor_msgs::LaserScan const);
            void convertRadialGap(dynamic_gap::Gap * gap, bool initial); //, sensor_msgs::LaserScan const);
            void radialExtendGap(dynamic_gap::Gap * gap, bool initial); //, sensor_msgs::LaserScan const);
            void inflateGapSides(dynamic_gap::Gap * gap, bool initial);

        private:
            bool checkWaypointVisibility(const Eigen::Vector2f & leftPt, const Eigen::Vector2f & rightPt,
                                                const Eigen::Vector2f & globalPathLocalWaypoint);
            float setBiasedGapGoalTheta(float leftTheta, float rightTheta, float globalPathLocalWaypointTheta,
                                        float leftToRightAngle, float rightToGoalAngle, float leftToGoalAngle);
            boost::shared_ptr<sensor_msgs::LaserScan const> scan_;
            sensor_msgs::LaserScan staticScan_, dynamicScan_;
            const DynamicGapConfig* cfg_;
            // int num_of_scan;
            // int half_num_scan;
            // float angle_min;
            // float angle_increment; 
            boost::mutex egolock;

    };
}