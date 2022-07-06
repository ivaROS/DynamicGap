#ifndef DG_CONFIG_H
#define DG_CONFIG_H

#include <ros/console.h>
#include <ros/ros.h>
#include <dynamic_gap/dgConfig.h>
#include <Eigen/Core>
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
                bool close_gap_vis;
                bool follow_the_gap_vis;
                bool fig_gen;
                double viz_jitter;
                bool debug_viz;
            } gap_viz;

            struct GapManipulation {
                double gap_diff;
                double epsilon2;
                double epsilon1;
                double sigma;
                double reduction_threshold;
                double reduction_target;
                int max_idx_diff;
                bool radial_extend;
                bool axial_convert;
                double rot_ratio;
                double cbf_param;
                double cbf_r_min;
                double K_des;
                double K_acc;
            } gap_manip;

            struct GapAssociation {
                double assoc_thresh;
            } gap_assoc;

            struct ControlParams {
                double k_drive_x;
                double k_drive_y;
                double k_turn;
                double v_ang_const;
                double v_lin_x_const;
                double v_lin_y_const;
                int ctrl_ahead_pose;
                double vx_absmax;
                double vy_absmax;
                double vang_absmax;
            } control;
            
            struct ProjectionParam {
                double k_po;
                double r_min;
                double r_norm;
                double r_norm_offset;
                double k_po_turn;
                double k_CBF;
            } projection;

            struct Waypoint {
                int global_plan_lookup_increment;
                double global_plan_change_tolerance;
            } waypoint;

            struct PlanningMode {
                bool feasi_inflated;  
                bool projection_inflated;
                bool planning_inflated;
                bool holonomic;
                bool full_fov;
                bool projection_operator;
                bool niGen_s;
                bool far_feasible;
                int num_feasi_check;
                int halt_size;
            } planning;

            struct Goal {
                double goal_tolerance;
                double waypoint_tolerance;
            } goal;

            struct Trajectory {
                bool synthesized_frame;
                double scale;
                double integrate_maxt;
                double integrate_stept;
                double rmax;
                double cobs;
                double w;
                double inf_ratio;
                double terminal_weight;
                double waypoint_ratio;
            } traj;

            struct Robot {
                float r_inscr;
                int num_obsts;
            } rbt;

            struct ManualControl {
                bool man_ctrl;
                float man_x;
                float man_y;
                float man_theta;
                bool line;
            } man;

        DynamicGapConfig() {
            map_frame_id = "map";
            odom_frame_id = "odom";
            robot_frame_id = "base_link";
            sensor_frame_id = "camera_link";

            gap_viz.min_resoln = 1;
            gap_viz.close_gap_vis = false;
            gap_viz.follow_the_gap_vis = false;
            gap_viz.fig_gen = false;
            gap_viz.viz_jitter = 0.1;
            gap_viz.debug_viz = true;

            gap_assoc.assoc_thresh = 0.5;

            gap_manip.gap_diff = 0.1;
            gap_manip.epsilon1 = 0.18;
            gap_manip.epsilon2 = 0.18;
            gap_manip.sigma = 10.0;
            gap_manip.rot_ratio = 1.5;
            gap_manip.reduction_threshold = M_PI;
            gap_manip.reduction_target = M_PI;
            gap_manip.radial_extend = false;
            gap_manip.axial_convert = true;
            gap_manip.cbf_param = 0.1;
            gap_manip.K_des = 0.5;
            gap_manip.K_acc = 3.0;
            
            control.k_drive_x = 3.5;
            control.k_drive_y = 3.5;
            control.k_turn = 0.5;
            control.v_ang_const = 0;
            control.v_lin_x_const = 0;
            control.v_lin_y_const = 0;
            control.ctrl_ahead_pose = 2;
            control.vx_absmax = 0.5;
            control.vy_absmax = 0.5;
            control.vang_absmax = 0.5;

            projection.k_po = 0.8;
            projection.k_po_turn = 1;
            projection.r_min = 0.75;
            projection.r_norm = 1.25;
            projection.r_norm_offset = 0.5;

            waypoint.global_plan_lookup_increment = 75;
            waypoint.global_plan_change_tolerance = 0.1;

            planning.feasi_inflated = false;
            planning.projection_inflated = false;
            planning.planning_inflated = false;
            planning.holonomic = false;
            planning.full_fov = false;
            planning.projection_operator = true;
            planning.niGen_s = false;
            planning.num_feasi_check = 10;
            planning.far_feasible = false;
            planning.halt_size = 5;

            goal.goal_tolerance = 0.2;
            goal.waypoint_tolerance = 0.1;
            
            traj.synthesized_frame = true;
            traj.scale = 1;
            traj.integrate_maxt = 5;
            traj.integrate_stept = 0.05;
            traj.rmax = 0.3;
            traj.cobs = -1;
            traj.w = 3;
            traj.inf_ratio = 1.25;
            traj.terminal_weight = 10;
            traj.waypoint_ratio = 1.5;

            man.man_ctrl = false;
            man.man_x = 0;
            man.man_y = 0;
            man.man_theta = 0;
            man.line = false;

            rbt.r_inscr = 0.2;
            rbt.num_obsts = 0;
        }

        void loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

        void reconfigure(dgConfig& cfg);

        boost::mutex & configMutex() {return config_mutex;}

        private: 
            boost::mutex config_mutex; 
    };
}

#endif