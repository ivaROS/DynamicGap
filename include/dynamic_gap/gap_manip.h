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
            void updateStaticEgoCircle(sensor_msgs::LaserScan);
            void updateDynamicEgoCircle(dynamic_gap::Gap&,
                                        std::vector<sensor_msgs::LaserScan>);

            void setGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal, bool initial); //, sensor_msgs::LaserScan const dynamic_laser_scan);
            void setTerminalGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal);
            void reduceGap(dynamic_gap::Gap&, geometry_msgs::PoseStamped, bool); //), sensor_msgs::LaserScan const);
            void convertRadialGap(dynamic_gap::Gap&, bool); //, sensor_msgs::LaserScan const);
            void radialExtendGap(dynamic_gap::Gap&, bool); //, sensor_msgs::LaserScan const);
            void inflateGapSides(dynamic_gap::Gap& gap, bool initial);
            bool indivGapFeasibilityCheck(dynamic_gap::Gap&);
            double indivGapFindCrossingPoint(dynamic_gap::Gap&, Eigen::Vector2f&, dynamic_gap::rot_frame_kf*, dynamic_gap::rot_frame_kf*);
            std::vector<double> determineLeftRightModels(dynamic_gap::Gap& selectedGap, Eigen::Vector2f pg);

            void setGapGoal(dynamic_gap::Gap&, geometry_msgs::PoseStamped);

        private:
            boost::shared_ptr<sensor_msgs::LaserScan const> msg;
            sensor_msgs::LaserScan static_scan, dynamic_scan;
            const DynamicGapConfig* cfg_;
            int num_of_scan;
            int half_num_scan;
            double angle_min;
            double angle_increment; 
            boost::mutex egolock;

            float atanThetaWrap(float theta);
            float getLeftToRightAngle(Eigen::Vector2f, Eigen::Vector2f, bool);
            Eigen::Vector2f car2pol(Eigen::Vector2f);
            Eigen::Vector2f pol2car(Eigen::Vector2f);
            Eigen::Vector2f pTheta(float, float, Eigen::Vector2f, Eigen::Vector2f);
            bool checkGoalVisibility(geometry_msgs::PoseStamped, float theta_r, float theta_l, float rdist, float ldist, sensor_msgs::LaserScan const scan);
            bool checkGoalWithinGapAngleRange(dynamic_gap::Gap& gap, double gap_goal_idx, float lidx, float ridx);
            bool feasibilityCheck(dynamic_gap::Gap& gap, dynamic_gap::rot_frame_kf*, dynamic_gap::rot_frame_kf*);
            double gapSplinecheck(dynamic_gap::Gap& gap, dynamic_gap::rot_frame_kf*, dynamic_gap::rot_frame_kf*);
            void setGapGoalTimeBased(dynamic_gap::rot_frame_kf*, dynamic_gap::rot_frame_kf*, dynamic_gap::Gap&,  geometry_msgs::PoseStamped);
    };
}

#endif