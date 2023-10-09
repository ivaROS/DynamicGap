#pragma once

#include <ros/ros.h>
#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>

// #include <ros/console.h>
// #include <traj_generator.h>
// #include <turtlebot_trajectory_generator/near_identity.h>
#include <math.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/trajectory_generation/trajectory_synthesis_methods.h>
// #include <vector>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
// #include "tf/transform_datatypes.h"
// #include <sensor_msgs/LaserScan.h>
#include "OsqpEigen/OsqpEigen.h"

namespace dynamic_gap {

    class GapTrajectoryGenerator
    {
        public:
            GapTrajectoryGenerator(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {cfg_ = &cfg; };
            void updateTF(geometry_msgs::TransformStamped tf) {planning2odom = tf;};
            
            std::tuple<geometry_msgs::PoseArray, std::vector<float>> generateTrajectory(const dynamic_gap::Gap&, geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, bool);
            // std::vector<geometry_msgs::PoseArray> generateTrajectory(std::vector<dynamic_gap::Gap>);
            geometry_msgs::PoseArray transformBackTrajectory(const geometry_msgs::PoseArray &, 
                                                             const geometry_msgs::TransformStamped &);
            std::tuple<geometry_msgs::PoseArray, std::vector<float>> forwardPassTrajectory(const std::tuple<geometry_msgs::PoseArray, std::vector<float>> & return_tuple);

        private: 
            void initializeSolver(OsqpEigen::Solver & solver, int Kplus1, const Eigen::MatrixXf & A);

            Eigen::VectorXf arclength_sample_bezier(Eigen::Vector2f pt_origin, Eigen::Vector2f pt_0, Eigen::Vector2f pt_1, float num_curve_points, float & des_dist_interval);        
            void buildBezierCurve(dynamic_gap::Gap& selectedGap, Eigen::MatrixXf & left_curve, Eigen::MatrixXf & right_curve, Eigen::MatrixXf & all_curve_pts, 
                                Eigen::MatrixXf & left_curve_vel, Eigen::MatrixXf & right_curve_vel,
                                Eigen::MatrixXf & left_curve_inward_norm, Eigen::MatrixXf & right_curve_inward_norm, 
                                Eigen::MatrixXf & all_inward_norms, Eigen::MatrixXf & left_right_centers, Eigen::MatrixXf & all_centers,
                                Eigen::Vector2f nonrel_left_vel, Eigen::Vector2f nonrel_right_vel, Eigen::Vector2f nom_vel,
                                Eigen::Vector2f left_pt_0, Eigen::Vector2f left_pt_1, Eigen::Vector2f right_pt_0, Eigen::Vector2f right_pt_1, 
                                Eigen::Vector2f gap_radial_extension, Eigen::Vector2f goal_pt_1, float & left_weight, float & right_weight, 
                                float num_curve_points, 
                                int & true_left_num_rge_points, int & true_right_num_rge_points, Eigen::Vector2f init_rbt_pos,
                                Eigen::Vector2f left_bezier_origin, Eigen::Vector2f right_bezier_origin);
            void setConstraintMatrix(Eigen::MatrixXf &A, 
                                     int N, 
                                     int Kplus1, 
                                     const Eigen::MatrixXf & all_curve_pts, 
                                     const Eigen::MatrixXf & all_inward_norms, 
                                     const Eigen::MatrixXf & all_centers);


            geometry_msgs::TransformStamped planning2odom;       
            int num_curve_points;
            const DynamicGapConfig* cfg_;

    };
}