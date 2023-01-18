#ifndef DG_CONFIG_H
#define DG_CONFIG_H

// #include <ros/console.h>
#include <ros/ros.h>
#include <dynamic_gap/dgConfig.h>
// #include <Eigen/Core>
#include <boost/thread/mutex.hpp>

namespace dynamic_gap {
    class DynamicGapConfig {
        public:
            std::string map_frame_id;
            std::string odom_frame_id;
            std::string robot_frame_id;
            std::string sensor_frame_id;

            struct GapVisualization {
                int min_resoln;
                bool fig_gen;
                double viz_jitter;
                bool debug_viz;
            } gap_viz;

            struct Robot {
                float r_inscr;
                int num_obsts;
                double max_range;
            } rbt;

            struct PlanningMode {
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

            struct ManualControl {
                bool man_ctrl;
                float man_x;
                float man_y;
                float man_theta;
            } man;

            struct Goal {
                double goal_tolerance;
                double waypoint_tolerance;
            } goal;

            struct Debug {
                bool raw_gaps_debug_log;
                bool static_scan_separation_debug_log;
                bool simplified_gaps_debug_log;
                bool feasibility_debug_log;
                bool manipulation_debug_log;
                bool traj_debug_log;
                bool control_debug_log;
            } debug;

            struct GapAssociation {
                double assoc_thresh;
            } gap_assoc;           

            struct GapEstimation {
                double R_scalar;
                double Q_scalar;
            } gap_est;

            struct GapManipulation {
                double epsilon1;
                double epsilon2;
                double rot_ratio;
                double reduction_threshold;
                double reduction_target;
                int max_idx_diff;
                bool radial_extend;
                bool radial_convert;
            } gap_manip;

            struct Trajectory {
                bool synthesized_frame;
                double scale;
                double integrate_maxt;
                double integrate_stept;
                double max_pose_pen_dist;
                double cobs;
                double pose_exp_weight;
                double inf_ratio;
                double terminal_weight;
                double waypoint_ratio;
                int num_curve_points;
                int num_qB_points;
            } traj;            

            struct ControlParams {
                double k_fb_x;
                double k_fb_y;
                double k_fb_theta;
                int ctrl_ahead_pose;
                double vx_absmax;
                double vy_absmax;
                double vang_absmax;
                double ax_absmax;
                double ay_absmax;
                double aang_absmax;
            } control;
            
            struct ProjectionParam {
                double k_po_x;
                double k_po_theta;

                double r_min;
                double r_norm;
                double r_norm_offset;
                double cbf_param;                
                double k_CBF;

                bool line;
            } projection;

        DynamicGapConfig() {
            map_frame_id = "map";
            odom_frame_id = "odom";
            robot_frame_id = "base_link";
            sensor_frame_id = "camera_link";

            gap_viz.min_resoln = 1;
            gap_viz.fig_gen = true;
            gap_viz.viz_jitter = 0.1;
            gap_viz.debug_viz = true;

            rbt.r_inscr = 0.2;
            rbt.num_obsts = 0;
            rbt.max_range = 4.99;

            planning.projection_inflated = false;
            planning.planning_inflated = false;
            planning.holonomic = false;
            planning.full_fov = false;
            planning.projection_operator = false;
            planning.num_feasi_check = 10;
            planning.far_feasible = false;
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
            debug.traj_debug_log = false;
            debug.control_debug_log = false;             

            gap_assoc.assoc_thresh = 0.25;

            gap_est.R_scalar = 0.01;
            gap_est.Q_scalar = 0.5;

            gap_manip.epsilon1 = 0.18;
            gap_manip.epsilon2 = 0.18;
            gap_manip.rot_ratio = 1.5;
            gap_manip.reduction_threshold = M_PI;
            gap_manip.reduction_target = M_PI;
            gap_manip.radial_extend = true;
            gap_manip.radial_convert = true;

            traj.synthesized_frame = true;
            traj.scale = 1;
            traj.integrate_maxt = 5;
            traj.integrate_stept = 0.50;
            traj.max_pose_pen_dist = 0.3;
            traj.cobs = -1.0;
            traj.pose_exp_weight = 5;
            traj.inf_ratio = 1.21;
            traj.terminal_weight = 10;
            traj.waypoint_ratio = 1.5;
            traj.num_curve_points = 40;
            traj.num_qB_points = 3;     
            
            control.k_fb_x = 3.5;
            control.k_fb_y = 3.5;
            control.k_fb_theta = 0.5;
            control.ctrl_ahead_pose = 2;
            control.vx_absmax = 0.5;
            control.vy_absmax = 0.5;
            control.vang_absmax = 0.5;
            control.ax_absmax = 0.5;
            control.ay_absmax = 0.5;
            control.aang_absmax = 0.5;

            projection.k_po_x = 0.8;
            projection.k_po_theta = 1;
            projection.r_min = 0.35;
            projection.r_norm = 1.0;
            projection.r_norm_offset = 0.5;
            projection.cbf_param = 0.1;
            projection.k_CBF = 1.0;
            projection.line = false;
        }

        void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

        void reconfigure(dgConfig& cfg);

        boost::mutex & configMutex() {return config_mutex;}

        private: 
            boost::mutex config_mutex; 
    };
}

#endif