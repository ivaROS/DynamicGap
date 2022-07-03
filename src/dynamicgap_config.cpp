#include <dynamic_gap/dynamicgap_config.h>

namespace dynamic_gap {
    void DynamicGapConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
    {
        // format: key, value, default value
        nh.param("map_frame_id", map_frame_id, map_frame_id);
        nh.param("odom_frame_id", odom_frame_id, odom_frame_id);
        nh.param("robot_frame_id", robot_frame_id, robot_frame_id);
        nh.param("sensor_frame_id", sensor_frame_id, sensor_frame_id);

        // Gap Visualization
        nh.param("min_resoln", gap_viz.min_resoln, gap_viz.min_resoln);
        nh.param("close_gap", gap_viz.close_gap_vis, gap_viz.close_gap_vis);
        nh.param("follow_the_gap", gap_viz.follow_the_gap_vis, gap_viz.follow_the_gap_vis);
        nh.param("fig_gen", gap_viz.fig_gen, gap_viz.fig_gen);
        nh.param("viz_jitter", gap_viz.viz_jitter, gap_viz.viz_jitter);
        nh.param("debug_viz", gap_viz.debug_viz, gap_viz.debug_viz);

        // Gap Association
        nh.param("assoc_thresh", gap_assoc.assoc_thresh, gap_assoc.assoc_thresh);

        // Gap Manipulation
        nh.param("gap_diff", gap_manip.gap_diff, gap_manip.gap_diff);
        nh.param("epsilon2", gap_manip.epsilon2, gap_manip.epsilon2);
        nh.param("epsilon1", gap_manip.epsilon1, gap_manip.epsilon1);
        nh.param("sigma", gap_manip.sigma, gap_manip.sigma);
        nh.param("rot_ratio", gap_manip.rot_ratio, gap_manip.rot_ratio);
        nh.param("reduction_threshold", gap_manip.reduction_threshold, gap_manip.reduction_threshold);
        nh.param("reduction_target", gap_manip.reduction_target, gap_manip.reduction_target);        
        nh.param("max_idx_diff", gap_manip.max_idx_diff, gap_manip.max_idx_diff);
        nh.param("radial_extend", gap_manip.radial_extend, gap_manip.radial_extend);
        nh.param("axial_convert", gap_manip.axial_convert, gap_manip.axial_convert);
        nh.param("cbf_param", gap_manip.cbf_param, gap_manip.cbf_param);
        nh.param("cbf_r_min", gap_manip.cbf_r_min, gap_manip.cbf_r_min);
        nh.param("K_des", gap_manip.K_des, gap_manip.K_des);
        nh.param("K_acc", gap_manip.K_acc, gap_manip.K_acc);

        // Control Params
        nh.param("k_drive_x",control.k_drive_x, control.k_drive_x);
        nh.param("k_drive_y",control.k_drive_y, control.k_drive_y);
        nh.param("k_turn",control.k_turn, control.k_turn);
        nh.param("v_ang_const",control.v_ang_const, control.v_ang_const);
        nh.param("v_lin_x_const",control.v_lin_x_const, control.v_lin_x_const);
        nh.param("v_lin_y_const",control.v_lin_y_const, control.v_lin_y_const);
        nh.param("ctrl_ahead_pose",control.ctrl_ahead_pose, control.ctrl_ahead_pose);

        nh.param("vx_absmax",control.vx_absmax, control.vx_absmax);
        nh.param("vy_absmax",control.vy_absmax, control.vy_absmax);
        nh.param("vang_absmax", control.vang_absmax, control.vang_absmax);

        // Projection Params
        nh.param("k_po", projection.k_po, projection.k_po);
        nh.param("k_po_turn", projection.k_po, projection.k_po);
        nh.param("r_min", projection.r_min, projection.r_min);
        nh.param("r_norm", projection.r_norm, projection.r_norm);
        nh.param("r_norm_offset", projection.r_norm_offset, projection.r_norm_offset);

        // Waypoint Params
        nh.param("global_plan_lookup_increment", waypoint.global_plan_lookup_increment, waypoint.global_plan_lookup_increment);
        nh.param("global_plan_change_tolerance", waypoint.global_plan_change_tolerance, waypoint.global_plan_change_tolerance);

        // Goal Param
        nh.param("goal_tolerance", goal.goal_tolerance, goal.goal_tolerance);
        nh.param("waypoint_tolerance", goal.waypoint_tolerance, goal.waypoint_tolerance);

        // General Planning Mode Params
        nh.param("feasi_inflated", planning.feasi_inflated, planning.feasi_inflated);
        nh.param("projection_inflated", planning.projection_inflated, planning.projection_inflated);
        nh.param("planning_inflated", planning.planning_inflated, planning.planning_inflated);
        nh.param("holonomic", planning.holonomic, planning.holonomic);
        nh.param("full_fov", planning.full_fov, planning.full_fov);
        nh.param("projection_operator", planning.projection_operator, planning.projection_operator);
        nh.param("niGen_s", planning.niGen_s, planning.niGen_s);
        nh.param("num_feasi_check", planning.num_feasi_check, planning.num_feasi_check);
        nh.param("num_feasi_check", planning.far_feasible, planning.far_feasible);

        // Trajectory
        nh.param("synthesized_frame", traj.synthesized_frame, traj.synthesized_frame);
        nh.param("scale", traj.scale, traj.scale);
        nh.param("integrate_maxt", traj.integrate_maxt, traj.integrate_maxt);
        nh.param("integrate_stept", traj.integrate_stept, traj.integrate_stept);
        nh.param("rmax", traj.rmax, traj.rmax);
        nh.param("inf_ratio", traj.inf_ratio, traj.inf_ratio);
        nh.param("terminal_weight", traj.terminal_weight, traj.terminal_weight);
        nh.param("waypoint_ratio", traj.waypoint_ratio, traj.waypoint_ratio);
        
        // Robot
        nh.param("r_inscr", rbt.r_inscr, rbt.r_inscr);
        nh.param("num_obsts", rbt.num_obsts, rbt.num_obsts);

    }

    void DynamicGapConfig::reconfigure(dgConfig& cfg)
    {
        // This locks the lock within this function
        boost::mutex::scoped_lock lock(config_mutex);
        gap_viz.min_resoln = cfg.min_resoln;
        gap_viz.close_gap_vis = cfg.close_gap_vis;
        gap_viz.follow_the_gap_vis = cfg.follow_the_gap_vis;
        gap_viz.fig_gen = cfg.fig_gen;
        gap_viz.viz_jitter = cfg.viz_jitter;
        gap_viz.debug_viz = cfg.debug_viz;

        // Gap Association
        gap_assoc.assoc_thresh = cfg.assoc_thresh;

        // Gap Manipulation
        gap_manip.gap_diff = cfg.gap_diff;
        gap_manip.epsilon2 = cfg.epsilon2;
        gap_manip.epsilon1 = cfg.epsilon1;
        gap_manip.sigma = cfg.sigma;
        gap_manip.rot_ratio = cfg.rot_ratio;
        gap_manip.reduction_threshold = cfg.reduction_threshold;
        gap_manip.reduction_target = cfg.reduction_target;        
        gap_manip.max_idx_diff = cfg.max_idx_diff;
        gap_manip.radial_extend = cfg.radial_extend;
        gap_manip.axial_convert = cfg.axial_convert;
        gap_manip.cbf_param = cfg.cbf_param;
        gap_manip.K_des = cfg.K_des;

        // Control Params
        control.k_drive_x = cfg.k_drive_x;
        control.k_drive_y = cfg.k_drive_y;
        control.k_turn = cfg.k_turn;
        control.v_ang_const = cfg.v_ang_const;
        control.v_lin_x_const = cfg.v_lin_x_const;
        control.v_lin_y_const = cfg.v_lin_y_const;
        control.ctrl_ahead_pose = cfg.ctrl_ahead_pose;
        control.vx_absmax = cfg.vx_absmax;
        control.vy_absmax = cfg.vy_absmax;
        control.vang_absmax = cfg.vang_absmax;

        // Projection Params
        projection.k_po = cfg.k_po;
        projection.k_po_turn = cfg.k_po_turn;
        projection.k_CBF = cfg.k_CBF;
        projection.r_min = cfg.r_min;
        projection.r_norm = cfg.r_norm;
        projection.r_norm_offset = cfg.r_norm_offset;

        // Waypoint Params
        waypoint.global_plan_lookup_increment = cfg.global_plan_lookup_increment;
        waypoint.global_plan_change_tolerance = cfg.global_plan_change_tolerance;

        // Goal Param
        goal.goal_tolerance = cfg.goal_tolerance;
        goal.waypoint_tolerance = cfg.waypoint_tolerance;

        // General Planning Mode Params
        planning.feasi_inflated = cfg.feasi_inflated;
        planning.projection_inflated = cfg.projection_inflated;
        planning.planning_inflated = cfg.planning_inflated;
        planning.holonomic = cfg.holonomic;
        planning.full_fov = cfg.full_fov;
        planning.projection_operator = cfg.projection_operator;
        planning.niGen_s = cfg.niGen_s;
        planning.num_feasi_check = cfg.num_feasi_check;
        planning.far_feasible = cfg.far_feasible;

        traj.synthesized_frame = cfg.synthesized_frame;
        traj.scale = cfg.scale;
        traj.integrate_maxt = cfg.integrate_maxt;
        traj.integrate_stept = cfg.integrate_stept;
        traj.rmax = cfg.rmax;
        traj.inf_ratio = cfg.inf_ratio;
        traj.terminal_weight = cfg.terminal_weight;
        traj.waypoint_ratio = cfg.waypoint_ratio;

        man.man_ctrl = cfg.man_ctrl;
        man.man_x = cfg.man_x;
        man.man_y = cfg.man_y;
        man.man_theta = cfg.man_theta;
        man.line = cfg.line;

        rbt.r_inscr = cfg.r_inscr;
        rbt.num_obsts = cfg.num_obsts;
    }



}