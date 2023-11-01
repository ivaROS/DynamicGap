#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

namespace dynamic_gap 
{
    class DynamicGapConfig 
    {
        // DEFAULT VALUES, CAN BE OVERRIDEN THROUGH ROS PARAMETERS
        public:
            std::string map_frame_id = "map";
            std::string odom_frame_id = "odom";
            std::string robot_frame_id = "base_link";
            std::string sensor_frame_id = "camera_link";

            
            struct GapVisualization 
            {
                int min_resoln = 2;
                // bool fig_gen = true;
                // float viz_jitter = 0.05;
                // bool debug_viz = true;
            } gap_viz;
            

            struct Robot 
            {
                float r_inscr = 0.2;
            } rbt;

            struct Environment
            {
                int num_obsts = 0; // needs to be updated from launch file
            } env;

            struct Scan
            {
                float angle_min = -M_PI;
                float angle_max = M_PI;
                int half_scan = 256;
                float half_scan_f = 256.;
                int full_scan = 512;
                float full_scan_f = 512.;
                float angle_increment = (2 * M_PI) / (full_scan_f - 1);
                float range_max = 4.99;
                float range_min = 0.0;
            } scan;

            struct PlanningMode 
            {
                bool projection_inflated = false;
                bool holonomic = true;
                bool full_fov = true;
                bool projection_operator = false;
                bool far_feasible = true;
                int num_feasi_check = 20;
                int halt_size = 5;
                bool egocircle_prop_cheat = true;
                bool dynamic_feasibility_check = true;
            } planning;            

            struct ManualControl 
            {
                bool man_ctrl = false;
                float man_x = 0.0;
                float man_y = 0.0;
                float man_theta = 0.0;
            } man;

            struct Goal 
            {
                float goal_tolerance = 0.2;
                float waypoint_tolerance = 0.1;
            } goal;

            struct Debug 
            {
                bool gap_detection_debug_log = false;
                bool gap_simplification_debug_log = false;
                bool raw_gaps_debug_log = false;
                bool static_scan_separation_debug_log = false;
                bool simplified_gaps_debug_log = false;
                bool future_scan_propagation_debug_log = false;
                bool feasibility_debug_log = true;
                bool manipulation_debug_log = false;
                bool traj_debug_log = true;
                bool control_debug_log = true;
            } debug;

            struct GapAssociation 
            {
                float assoc_thresh = 0.15;
            } gap_assoc;           

            struct GapManipulation 
            {
                float epsilon1 = 0.20;
                float epsilon2 = 0.30;
                // float rot_ratio = 1.5;
                float reduction_threshold = M_PI;
                float reduction_target = M_PI;
                int max_idx_diff = 256;
                bool radial_extend = true;
                bool radial_convert = true;
            } gap_manip;

            struct Trajectory 
            {
                bool synthesized_frame = true;
                float scale = 1.0;
                float integrate_maxt = 5.0;
                float integrate_stept = 0.5;
                float max_pose_pen_dist = 0.5;
                float cobs = -1.0;
                float pose_exp_weight = 5.0;
                float inf_ratio = 1.21;
                float terminal_weight = 10.0;
                float waypoint_ratio = 1.5;
                int num_curve_points = 20;
                int num_extended_gap_origin_points = 6;
            } traj;            

            struct ControlParams 
            {
                float k_fb_x = 0.5;
                float k_fb_y = 0.5;
                float k_fb_theta = 0.5;
                int ctrl_ahead_pose = 2;
                float vx_absmax = 0.5;
                float vy_absmax = 0.5;
                float vang_absmax = 1.5;
                float ax_absmax = 3.0;
                float ay_absmax = 3.0;
                float aang_absmax = 3.0;
            } control;
            
            struct ProjectionParam 
            {
                float k_po_x = 1.0;
                float k_po_theta = 1.0;

                float r_min = 0.35;
                float r_norm = 1.0;
                float r_norm_offset = 0.5;
                float cbf_param = 0.1;                
                float k_CBF = 1.0;

                bool line = false;
            } projection;

        void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);
        void updateParamFromScan(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr);
    };
}