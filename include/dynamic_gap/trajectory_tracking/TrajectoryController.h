#pragma once

#include <ros/ros.h>
#include <dynamic_gap/config/DynamicGapConfig.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
// #include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <dynamic_gap/utils/Gap.h>
#include <dynamic_gap/utils/Utils.h>
#include "dynamic_gap/TrajPlan.h"
#include <dynamic_gap/trajectory_generation/GapTrajectoryGenerator.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

namespace dynamic_gap 
{
    class TrajectoryController 
    {
        public:

            TrajectoryController(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            geometry_msgs::Twist obstacleAvoidanceControlLaw(const sensor_msgs::LaserScan &);
            geometry_msgs::Twist manualControlLaw();
            geometry_msgs::Twist controlLaw(const geometry_msgs::Pose & current, 
                                            const nav_msgs::Odometry & desired,
                                            const sensor_msgs::LaserScan & inflated_egocircle, 
                                            const geometry_msgs::TwistStamped & currentPeakSplineVel_);
            geometry_msgs::Twist processCmdVel(const geometry_msgs::Twist & raw_cmd_vel,
                                                const sensor_msgs::LaserScan & inflated_egocircle, 
                                                const geometry_msgs::PoseStamped & rbt_in_cam_lc, 
                                                const dynamic_gap::Estimator * curr_rightGapPtModel, 
                                                const dynamic_gap::Estimator * curr_leftGapPtModel,
                                                const geometry_msgs::TwistStamped & current_rbt_vel, 
                                                const geometry_msgs::TwistStamped & rbt_accel);
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg);
            int targetPoseIdx(geometry_msgs::Pose curr_pose, dynamic_gap::TrajPlan ref_pose);
            dynamic_gap::TrajPlan trajGen(geometry_msgs::PoseArray);
            
        private:
            Eigen::Matrix2cf getComplexMatrix(float, float, float, float);
            Eigen::Matrix2cf getComplexMatrix(float, float, float);
            float dist2Pose(float theta, float dist, geometry_msgs::Pose pose);

            std::vector<geometry_msgs::Point> findLocalLine(int idx);
            float polDist(float l1, float t1, float l2, float t2);

            bool leqThres(const float dist);
            bool geqThres(const float dist);

            void run_projection_operator(const sensor_msgs::LaserScan & inflated_egocircle, 
                                         const geometry_msgs::PoseStamped & rbt_in_cam_lc,
                                         Eigen::Vector2f cmd_vel_fb, Eigen::Vector2f & Psi_der,
                                         float & Psi, float & cmd_vel_x_safe, float & cmd_vel_y_safe,
                                         float & min_dist_ang, float & min_dist);
            void run_bearing_rate_barrier_function(Eigen::Vector4f state, 
                                                   Eigen::Vector4f left_rel_model, 
                                                   Eigen::Vector4f right_rel_model,
                                                   Eigen::Vector2f rbt_accel, 
                                                   float & cmd_vel_x_safe, float & cmd_vel_y_safe, 
                                                   float & Psi_CBF);
            void clip_command_velocities(float & v_lin_x_fb, float & v_lin_y_fb, float & v_ang_fb);
            void visualize_projection_operator(float weighted_cmd_vel_x_safe, float weighted_cmd_vel_y_safe, ros::Publisher projection_viz);


            float cbf_right(Eigen::Vector4f);
            Eigen::Vector4f cbf_partials_right(Eigen::Vector4f);
            float cbf_left(Eigen::Vector4f);
            Eigen::Vector4f cbf_partials_left(Eigen::Vector4f);

            Eigen::Vector3f projection_method(float min_diff_x, float min_diff_y);

            float thres;
            const DynamicGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> msg_;
            boost::mutex egocircle_l;
            ros::Publisher projection_viz;

            bool holonomic;
            bool full_fov;
            bool projection_operator;
            float k_fb_theta_;
            float k_fb_x_;
            float k_fb_y_;
            float k_po_x_;
            float k_po_theta_;            
            float k_CBF_;
            int cmd_counter_;

            float manual_linear_interval;
            float manual_angular_interval;
            float manual_v_x;
            float manual_v_y;
            float manual_v_ang;
    };
}