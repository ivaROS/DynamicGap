#ifndef TRAJ_CONT_H
#define TRAJ_CONT_H

#include <ros/ros.h>
#include <dynamic_gap/dynamicgap_config.h>
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
#include <dynamic_gap/gap.h>
#include "dynamic_gap/TrajPlan.h"
#include <dynamic_gap/gap_trajectory_generator.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>

namespace dynamic_gap {
    class TrajectoryController {
        public:

            TrajectoryController(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg);
            geometry_msgs::Twist obstacleAvoidanceControlLaw(sensor_msgs::LaserScan);
            geometry_msgs::Twist manualControlLaw();
            geometry_msgs::Twist controlLaw(geometry_msgs::Pose current, nav_msgs::Odometry desired,
                                            sensor_msgs::LaserScan inflated_egocircle, geometry_msgs::PoseStamped rbt_in_cam_lc,
                                            geometry_msgs::Twist current_rbt_vel, geometry_msgs::TwistStamped rbt_accel,
                                            dynamic_gap::cart_model * curr_right_model, dynamic_gap::cart_model * curr_left_model,
                                            double curr_peak_velocity_x, double curr_peak_velocity_y);
            geometry_msgs::Twist processCmdVel(geometry_msgs::Twist raw_cmd_vel,
                        sensor_msgs::LaserScan inflated_egocircle, geometry_msgs::PoseStamped rbt_in_cam_lc, 
                        dynamic_gap::cart_model * curr_right_model, dynamic_gap::cart_model * curr_left_model,
                        geometry_msgs::Twist current_rbt_vel, geometry_msgs::TwistStamped rbt_accel);
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
            void run_projection_operator(sensor_msgs::LaserScan inflated_egocircle,  geometry_msgs::PoseStamped rbt_in_cam_lc,
                                         Eigen::Vector2d cmd_vel_fb, Eigen::Vector2d & Psi_der,
                                         double & Psi, float & cmd_vel_x_safe, float & cmd_vel_y_safe,
                                         float & min_dist_ang, float & min_dist);
            void run_bearing_rate_barrier_function(Eigen::Vector4d state, 
                                                   Eigen::Vector4d left_rel_model, Eigen::Vector4d right_rel_model,
                                                   Eigen::Vector2d rbt_accel, float & cmd_vel_x_safe, float & cmd_vel_y_safe, double & Psi_CBF);
            void clip_command_velocities(double & v_lin_x_fb, double & v_lin_y_fb, double & v_ang_fb);
            void visualize_projection_operator(double weighted_cmd_vel_x_safe, double weighted_cmd_vel_y_safe, ros::Publisher projection_viz);


            double cbf_right(Eigen::Vector4d);
            Eigen::Vector4d cbf_partials_right(Eigen::Vector4d);
            double cbf_left(Eigen::Vector4d);
            Eigen::Vector4d cbf_partials_left(Eigen::Vector4d);

            Eigen::Vector3d projection_method(float min_diff_x, float min_diff_y);

            double thres;
            const DynamicGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> msg_;
            boost::mutex egocircle_l;
            ros::Publisher projection_viz;
            ros::Time last_time;

            bool holonomic;
            bool full_fov;
            bool projection_operator;
            double k_fb_theta_;
            double k_fb_x_;
            double k_fb_y_;
            double k_po_x_;
            double k_po_theta_;            
            double k_CBF_;
    };
}

#endif