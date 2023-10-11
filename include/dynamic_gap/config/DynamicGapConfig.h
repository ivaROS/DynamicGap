#pragma once

// #include <ros/console.h>
#include <ros/ros.h>
// #include <dynamic_gap/dgConfig.h>
// #include <Eigen/Core>
#include <boost/thread/mutex.hpp>

namespace dynamic_gap {
    class DynamicGapConfig {
        public:
            std::string map_frame_id;
            std::string odom_frame_id;
            std::string robot_frame_id;
            std::string sensor_frame_id;

            struct GapVisualization 
            {
                int min_resoln;
                bool fig_gen;
                float viz_jitter;
                bool debug_viz;
            } gap_viz;

            struct Robot 
            {
                float r_inscr;
                int num_obsts;
                float max_range;
                int half_num_scan;
            } rbt;

            struct PlanningMode 
            {
                bool projection_inflated;
                bool planning_inflated;
                bool holonomic;
                bool full_fov;
                bool projection_operator;
                bool far_feasible;
                int num_feasi_check;
                int halt_size;
                bool egocircle_prop_cheat;
            } planning;            

            struct ManualControl 
            {
                bool man_ctrl;
                float man_x;
                float man_y;
                float man_theta;
            } man;

            struct Goal 
            {
                float goal_tolerance;
                float waypoint_tolerance;
            } goal;

            struct Debug 
            {
                bool raw_gaps_debug_log;
                bool static_scan_separation_debug_log;
                bool simplified_gaps_debug_log;
                bool feasibility_debug_log;
                bool manipulation_debug_log;
                bool traj_debug_log;
                bool control_debug_log;
            } debug;

            struct GapAssociation 
            {
                float assoc_thresh;
            } gap_assoc;           

            struct GapEstimation 
            {
                float R_scalar;
                float Q_scalar;
            } gap_est;

            struct GapManipulation 
            {
                float epsilon1;
                float epsilon2;
                float rot_ratio;
                float reduction_threshold;
                float reduction_target;
                int max_idx_diff;
                bool radial_extend;
                bool radial_convert;
            } gap_manip;

            struct Trajectory 
            {
                bool synthesized_frame;
                float scale;
                float integrate_maxt;
                float integrate_stept;
                float max_pose_pen_dist;
                float cobs;
                float pose_exp_weight;
                float inf_ratio;
                float terminal_weight;
                float waypoint_ratio;
                int num_curve_points;
                int num_qB_points;
            } traj;            

            struct ControlParams 
            {
                float k_fb_x;
                float k_fb_y;
                float k_fb_theta;
                int ctrl_ahead_pose;
                float vx_absmax;
                float vy_absmax;
                float vang_absmax;
                float ax_absmax;
                float ay_absmax;
                float aang_absmax;
            } control;
            
            struct ProjectionParam 
            {
                float k_po_x;
                float k_po_theta;

                float r_min;
                float r_norm;
                float r_norm_offset;
                float cbf_param;                
                float k_CBF;

                bool line;
            } projection;

        DynamicGapConfig() {
            map_frame_id = "map";
            odom_frame_id = "odom";
            robot_frame_id = "base_link";
            sensor_frame_id = "camera_link";

            gap_viz.min_resoln = 2;
            gap_viz.fig_gen = true;
            gap_viz.viz_jitter = 0.05;
            gap_viz.debug_viz = true;

            rbt.r_inscr = 0.2;
            rbt.num_obsts = 0;
            rbt.max_range = 4.99;

            planning.projection_inflated = false;
            planning.planning_inflated = false;
            planning.holonomic = true;
            planning.full_fov = true;
            planning.projection_operator = false;
            planning.num_feasi_check = 20;
            planning.far_feasible = true;
            planning.halt_size = 5;
            planning.egocircle_prop_cheat = false;

            man.man_ctrl = false;
            man.man_x = 0;
            man.man_y = 0;
            man.man_theta = 0;

            goal.goal_tolerance = 0.2;
            goal.waypoint_tolerance = 0.1;

            debug.raw_gaps_debug_log = false;
            debug.static_scan_separation_debug_log = false;
            debug.simplified_gaps_debug_log = false;
            debug.feasibility_debug_log = false;
            debug.manipulation_debug_log = false;
            debug.traj_debug_log = true;
            debug.control_debug_log = true;             

            gap_assoc.assoc_thresh = 0.15;

            gap_est.R_scalar = 0.1;
            gap_est.Q_scalar = 0.5;

            gap_manip.epsilon1 = 0.18;
            gap_manip.epsilon2 = 0.18;
            gap_manip.rot_ratio = 1.5;
            gap_manip.reduction_threshold = M_PI;
            gap_manip.reduction_target = M_PI;
            gap_manip.max_idx_diff = 256;
            gap_manip.radial_extend = true;
            gap_manip.radial_convert = true;

            traj.synthesized_frame = true;
            traj.scale = 1;
            traj.integrate_maxt = 5;
            traj.integrate_stept = 0.50;
            traj.max_pose_pen_dist = 0.5;
            traj.cobs = -1.0;
            traj.pose_exp_weight = 5;
            traj.inf_ratio = 1.21;
            traj.terminal_weight = 10;
            traj.waypoint_ratio = 1.5;
            traj.num_curve_points = 20;
            traj.num_qB_points = 6;     
            
            control.k_fb_x = 0.5;
            control.k_fb_y = 0.5;
            control.k_fb_theta = 0.5;
            control.ctrl_ahead_pose = 2;
            control.vx_absmax = 0.5;
            control.vy_absmax = 0.5;
            control.vang_absmax = 1.5;
            control.ax_absmax = 3.0;
            control.ay_absmax = 3.0;
            control.aang_absmax = 3.0;

            projection.k_po_x = 1.0;
            projection.k_po_theta = 1.0;
            projection.r_min = 0.35;
            projection.r_norm = 1.0;
            projection.r_norm_offset = 0.5;
            projection.cbf_param = 0.1;
            projection.k_CBF = 1.0;
            projection.line = false;
        }

        void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

        // void reconfigure(dgConfig& cfg);

        boost::mutex & configMutex() {return config_mutex;}

        private: 
            boost::mutex config_mutex; 
    };
}