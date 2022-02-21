#ifndef GAP_MOD_H
#define GAP_MOD_H


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
    class GapManipulator {
        public: 
            GapManipulator(){};
            ~GapManipulator(){};

            GapManipulator(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg;};
            GapManipulator& operator=(GapManipulator & other) {cfg_ = other.cfg_;};
            GapManipulator(const GapManipulator &t) {cfg_ = t.cfg_;};

            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);

            void setValidSliceWaypoint(dynamic_gap::Gap&, geometry_msgs::PoseStamped);
            void reduceGap(dynamic_gap::Gap&, geometry_msgs::PoseStamped);
            void convertAxialGap(dynamic_gap::Gap&);
            void radialExtendGap(dynamic_gap::Gap&);
            bool indivGapFeasibilityCheck(dynamic_gap::Gap&);
            std::vector<dynamic_gap::Gap> feasibilityCheck(std::vector<dynamic_gap::Gap>& manip_set);
            std::vector<double> determineLeftRightModels(dynamic_gap::Gap& selectedGap, Eigen::Vector2f pg);

            void setGapGoal(dynamic_gap::Gap&, geometry_msgs::PoseStamped);

        private:
            boost::shared_ptr<sensor_msgs::LaserScan const> msg;
            const DynamicGapConfig* cfg_;
            int num_of_scan;
            boost::mutex egolock;

            Eigen::Vector2f car2pol(Eigen::Vector2f);
            Eigen::Vector2f pol2car(Eigen::Vector2f);
            Eigen::Vector2f pTheta(float, float, Eigen::Vector2f, Eigen::Vector2f);
            bool checkGoalVisibility(geometry_msgs::PoseStamped);
            bool checkGoalWithinGap(dynamic_gap::Gap& gap, double gap_goal_idx);
            bool feasibilityCheckHelper(dynamic_gap::Gap& gap, double left_betadot_check, double right_betadot_check, double gap_angle);
            void setSweptValues(dynamic_gap::Gap& gap, double left_betadot_check, double right_betadot_check, double left_ori, double right_ori);




    };
}

#endif