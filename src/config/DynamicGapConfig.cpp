#include <dynamic_gap/config/DynamicGapConfig.h>

namespace dynamic_gap 
{
    void DynamicGapConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
    {
        // format: key, value, default value
        nh.param("map_frame_id", map_frame_id, map_frame_id);
        nh.param("odom_frame_id", odom_frame_id, odom_frame_id);
        nh.param("robot_frame_id", robot_frame_id, robot_frame_id);
        nh.param("sensor_frame_id", sensor_frame_id, sensor_frame_id);

        // Gap Visualization
        nh.param("min_resoln", gap_viz.min_resoln, gap_viz.min_resoln);
        nh.param("fig_gen", gap_viz.fig_gen, gap_viz.fig_gen);
        nh.param("viz_jitter", gap_viz.viz_jitter, gap_viz.viz_jitter);
        nh.param("debug_viz", gap_viz.debug_viz, gap_viz.debug_viz);

        // Robot
        nh.param("r_inscr", rbt.r_inscr, rbt.r_inscr);
        nh.param("num_obsts", rbt.num_obsts, rbt.num_obsts);
        nh.param("max_range", rbt.max_range, rbt.max_range);

        // Planning Information
        nh.param("projection_inflated", planning.projection_inflated, planning.projection_inflated);
        nh.param("holonomic", planning.holonomic, planning.holonomic);
        nh.param("full_fov", planning.full_fov, planning.full_fov);
        nh.param("projection_operator", planning.projection_operator, planning.projection_operator);
        nh.param("num_feasi_check", planning.num_feasi_check, planning.num_feasi_check);
        nh.param("far_feasible", planning.far_feasible, planning.far_feasible);
        nh.param("egocircle_prop_cheat", planning.egocircle_prop_cheat, planning.egocircle_prop_cheat);

        // Manual Control
        nh.param("man_ctrl", man.man_ctrl, man.man_ctrl);
        nh.param("man_x", man.man_x, man.man_x);
        nh.param("man_y", man.man_y, man.man_y);
        nh.param("man_theta", man.man_theta, man.man_theta);

        // Goal Param
        nh.param("goal_tolerance", goal.goal_tolerance, goal.goal_tolerance);
        nh.param("waypoint_tolerance", goal.waypoint_tolerance, goal.waypoint_tolerance);

        // Debug
        // nh.param("raw_gaps_debug_log", debug.raw_gaps_debug_log, debug.raw_gaps_debug_log);
        // nh.param("static_scan_separation_debug_log", debug.static_scan_separation_debug_log, debug.static_scan_separation_debug_log);
        // nh.param("simplified_gaps_debug_log", debug.simplified_gaps_debug_log, debug.simplified_gaps_debug_log);
        // nh.param("future_scan_propagation_debug_log", debug.future_scan_propagation_debug_log, debug.future_scan_propagation_debug_log);      
        // nh.param("feasibility_debug_log", debug.feasibility_debug_log, debug.feasibility_debug_log);
        // nh.param("manipulation_debug_log", debug.manipulation_debug_log, debug.manipulation_debug_log);
        // nh.param("traj_debug_log", debug.traj_debug_log, debug.traj_debug_log);
        // nh.param("control_debug_log", debug.control_debug_log, debug.control_debug_log);             

        // Gap Association
        nh.param("assoc_thresh", gap_assoc.assoc_thresh, gap_assoc.assoc_thresh);

        // Gap Manipulation
        nh.param("epsilon2", gap_manip.epsilon2, gap_manip.epsilon2);
        nh.param("epsilon1", gap_manip.epsilon1, gap_manip.epsilon1);
        nh.param("rot_ratio", gap_manip.rot_ratio, gap_manip.rot_ratio);
        nh.param("reduction_threshold", gap_manip.reduction_threshold, gap_manip.reduction_threshold);
        nh.param("reduction_target", gap_manip.reduction_target, gap_manip.reduction_target);        
        nh.param("max_idx_diff", gap_manip.max_idx_diff, gap_manip.max_idx_diff);
        nh.param("radial_extend", gap_manip.radial_extend, gap_manip.radial_extend);
        nh.param("radial_convert", gap_manip.radial_convert, gap_manip.radial_convert);

        // Control Params
        nh.param("k_fb_x",control.k_fb_x, control.k_fb_x);
        nh.param("k_fb_y",control.k_fb_y, control.k_fb_y);
        nh.param("k_fb_theta",control.k_fb_theta, control.k_fb_theta);
        nh.param("ctrl_ahead_pose",control.ctrl_ahead_pose, control.ctrl_ahead_pose);
        nh.param("vx_absmax",control.vx_absmax, control.vx_absmax);
        nh.param("vy_absmax",control.vy_absmax, control.vy_absmax);
        nh.param("vang_absmax", control.vang_absmax, control.vang_absmax);
        nh.param("ax_absmax",control.ax_absmax, control.ax_absmax);
        nh.param("ay_absmax",control.ay_absmax, control.ay_absmax);
        nh.param("aang_absmax", control.aang_absmax, control.aang_absmax);

        // Projection Params
        nh.param("k_po_x", projection.k_po_x, projection.k_po_x);
        nh.param("k_po_theta", projection.k_po_theta, projection.k_po_theta);
        nh.param("r_min", projection.r_min, projection.r_min);
        nh.param("r_norm", projection.r_norm, projection.r_norm);
        nh.param("r_norm_offset", projection.r_norm_offset, projection.r_norm_offset);
        nh.param("k_CBF", projection.k_CBF, projection.k_CBF);
        nh.param("cbf_param", projection.cbf_param, projection.cbf_param);

        // Trajectory
        nh.param("synthesized_frame", traj.synthesized_frame, traj.synthesized_frame);
        nh.param("integrate_maxt", traj.integrate_maxt, traj.integrate_maxt);
        nh.param("integrate_stept", traj.integrate_stept, traj.integrate_stept);
        nh.param("max_pose_pen_dist", traj.max_pose_pen_dist, traj.max_pose_pen_dist);
        nh.param("terminal_weight", traj.terminal_weight, traj.terminal_weight);
        nh.param("waypoint_ratio", traj.waypoint_ratio, traj.waypoint_ratio);
        nh.param("num_curve_points", traj.num_curve_points, traj.num_curve_points);
        nh.param("num_qB_points", traj.num_qB_points, traj.num_qB_points);
    }
}