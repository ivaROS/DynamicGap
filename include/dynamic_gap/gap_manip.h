#ifndef GAP_MOD_H
#define GAP_MOD_H


#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/gap.h>
#include <dynamic_gap/dynamicgap_config.h>
#include <dynamic_gap/trajectory_scoring.h>
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
            void updateStaticEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);
            void updateDynamicEgoCircle(std::vector<dynamic_gap::Gap>, 
                                        dynamic_gap::Gap&,
                                        std::vector<geometry_msgs::Pose> &, 
                                        std::vector<geometry_msgs::Vector3Stamped>,
                                        dynamic_gap::TrajectoryArbiter *);

            void setGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal, bool initial); //, sensor_msgs::LaserScan const dynamic_laser_scan);
            void setTerminalGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal);
            void reduceGap(dynamic_gap::Gap&, geometry_msgs::PoseStamped, bool); //), sensor_msgs::LaserScan const);
            void convertAxialGap(dynamic_gap::Gap&, bool); //, sensor_msgs::LaserScan const);
            void radialExtendGap(dynamic_gap::Gap&, bool); //, sensor_msgs::LaserScan const);
            void inflateGapSides(dynamic_gap::Gap& gap, bool initial);
            bool indivGapFeasibilityCheck(dynamic_gap::Gap&);
            double indivGapFindCrossingPoint(dynamic_gap::Gap& gap, Eigen::Vector2f& gap_crossing_point, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model);
            std::vector<double> determineLeftRightModels(dynamic_gap::Gap& selectedGap, Eigen::Vector2f pg);

            void setGapGoal(dynamic_gap::Gap&, geometry_msgs::PoseStamped);

        private:
            boost::shared_ptr<sensor_msgs::LaserScan const> msg, static_msg;
            sensor_msgs::LaserScan dynamic_scan;
            const DynamicGapConfig* cfg_;
            int num_of_scan;
            boost::mutex egolock;

            Eigen::Vector2f car2pol(Eigen::Vector2f);
            Eigen::Vector2f pol2car(Eigen::Vector2f);
            Eigen::Vector2f pTheta(float, float, Eigen::Vector2f, Eigen::Vector2f);
            bool checkGoalVisibility(geometry_msgs::PoseStamped,  sensor_msgs::LaserScan const scan);
            bool checkGoalWithinGapAngleRange(dynamic_gap::Gap& gap, double gap_goal_idx, float lidx, float ridx);
            bool feasibilityCheck(dynamic_gap::Gap& gap, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model, double gap_angle);
            double gapSplinecheck(dynamic_gap::Gap& gap, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model);
            void setGapGoalTimeBased(dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model, dynamic_gap::Gap& gap,  geometry_msgs::PoseStamped localgoal);
    };
}

#endif