#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include <ros/ros.h>
#include <boost/numeric/odeint.hpp>
#include <ros/console.h>
// #include <traj_generator.h>
// #include <turtlebot_trajectory_generator/near_identity.h>
#include <geometry_msgs/PoseArray.h>
#include <dynamic_gap/helper.h>
#include <ros/ros.h>
#include <math.h>
#include <dynamic_gap/gap.h>
#include <dynamic_gap/dynamicgap_config.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/transform_datatypes.h"
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

namespace dynamic_gap {

    class TrajectoryGenerator {
        public:
            TrajectoryGenerator(){};
            ~TrajectoryGenerator(){};

            TrajectoryGenerator(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg;};
            TrajectoryGenerator& operator=(TrajectoryGenerator & other) {cfg_ = other.cfg_;};
            TrajectoryGenerator(const TrajectoryGenerator &t) {cfg_ = t.cfg_;};

            virtual std::tuple<geometry_msgs::PoseArray, std::vector<double>, std::vector<double>> generateTrajectory(dynamic_gap::Gap, geometry_msgs::PoseStamped, geometry_msgs::Twist curr_vel) = 0;
            virtual std::vector<geometry_msgs::PoseArray> generateTrajectory(std::vector<dynamic_gap::Gap>) = 0;
        protected:
            const DynamicGapConfig* cfg_;
    };

    class GapTrajGenerator : public TrajectoryGenerator {
        using TrajectoryGenerator::TrajectoryGenerator;
        public:
            void updateTF(geometry_msgs::TransformStamped tf) {planning2odom = tf;};
            std::tuple<geometry_msgs::PoseArray, std::vector<double>, std::vector<double>> generateTrajectory(dynamic_gap::Gap, geometry_msgs::PoseStamped, geometry_msgs::Twist curr_vel);
            std::vector<geometry_msgs::PoseArray> generateTrajectory(std::vector<dynamic_gap::Gap>);
            geometry_msgs::PoseArray transformBackTrajectory(geometry_msgs::PoseArray, geometry_msgs::TransformStamped);
            std::tuple<geometry_msgs::PoseArray, std::vector<double>, std::vector<double>> forwardPassTrajectory(std::tuple<geometry_msgs::PoseArray, std::vector<double>, std::vector<double>>);
            void determineLeftRightModels(Matrix<double, 5, 1>&, Matrix<double, 5, 1>&, dynamic_gap::Gap&, double);

        private: 
            geometry_msgs::TransformStamped planning2odom;

    };
}

#endif