#include <dynamic_gap/config/DynamicGapConfig.h>

namespace dynamic_gap 
{
    void DynamicGapConfig::loadRosParamFromNodeHandle(const ros::NodeHandle& nh)
    {
        // Would need to do a bit of processing
        //      Read in params:
        //          model
        //          
        //      set config parameters in this class using model

        std::string model;
        nh.param("/model", model, model);

        if (model == "rto")
        {
            // format: key, value, default value
            odom_frame_id = model + "/odom";
            robot_frame_id = model + "/base_link";
            sensor_frame_id = model + "/hokuyo_link";

            
            odom_topic = model + "/odom";
            imu_topic = model + "/imu";
            scan_topic = model + "/scan";

            // nh.param("map_frame_id", map_frame_id, map_frame_id);
            // nh.param("odom_frame_id", odom_frame_id, odom_frame_id);
            // nh.param("robot_frame_id", robot_frame_id, robot_frame_id);
            // nh.param("sensor_frame_id", sensor_frame_id, sensor_frame_id);

            // Gap Visualization
            // nh.param("min_resoln", gap_viz.min_resoln, gap_viz.min_resoln);
            // nh.param("fig_gen", gap_viz.fig_gen, gap_viz.fig_gen);
            // nh.param("viz_jitter", gap_viz.viz_jitter, gap_viz.viz_jitter);
            // nh.param("debug_viz", gap_viz.debug_viz, gap_viz.debug_viz);

            // Robot
            nh.param("robot_radius", rbt.r_inscr, rbt.r_inscr);
            nh.param("max_vel_x",rbt.vx_absmax, rbt.vx_absmax);
            nh.param("max_vel_y",rbt.vy_absmax, rbt.vy_absmax);
            nh.param("max_vel_theta", rbt.vang_absmax, rbt.vang_absmax);
            nh.param("acc_lim_x",rbt.ax_absmax, rbt.ax_absmax);
            nh.param("acc_lim_x",rbt.ay_absmax, rbt.ay_absmax);
            nh.param("acc_lim_theta", rbt.aang_absmax, rbt.aang_absmax);

            // Environment
            // nh.param("num_agents", env.num_agents, env.num_agents);

            // Scan
            nh.param("max_range", scan.range_max, scan.range_max);

            // Planning Information
            nh.param("holonomic", planning.holonomic, planning.holonomic);
            nh.param("projection_operator", planning.projection_operator, planning.projection_operator);
            nh.param("egocircle_prop_cheat", planning.egocircle_prop_cheat, planning.egocircle_prop_cheat);
            nh.param("gap_feasibility_check", planning.gap_feasibility_check, planning.gap_feasibility_check);
            nh.param("perfect_gap_models", planning.perfect_gap_models, planning.perfect_gap_models);
            nh.param("future_scan_propagation", planning.future_scan_propagation, planning.future_scan_propagation);

            // Manual Control
            nh.param("man_ctrl", man.man_ctrl, man.man_ctrl);

            // Goal Param
            nh.param("xy_goal_tolerance", goal.goal_tolerance, goal.goal_tolerance);
            nh.param("xy_waypoint_tolerance", goal.waypoint_tolerance, goal.waypoint_tolerance);

            // Gap Detection
            nh.param("max_idx_diff", gap_det.max_idx_diff, gap_det.max_idx_diff);

            // Gap Association
            nh.param("assoc_thresh", gap_assoc.assoc_thresh, gap_assoc.assoc_thresh);

            // Gap Manipulation
            nh.param("epsilon1", gap_manip.epsilon1, gap_manip.epsilon1);
            nh.param("epsilon2", gap_manip.epsilon2, gap_manip.epsilon2);
            // nh.param("rot_ratio", gap_manip.rot_ratio, gap_manip.rot_ratio);
            nh.param("reduction_threshold", gap_manip.reduction_threshold, gap_manip.reduction_threshold);
            nh.param("radial_extend", gap_manip.radial_extend, gap_manip.radial_extend);
            nh.param("radial_convert", gap_manip.radial_convert, gap_manip.radial_convert);

            // Trajectory
            // nh.param("synthesized_frame", traj.synthesized_frame, traj.synthesized_frame);
            nh.param("integrate_maxt", traj.integrate_maxt, traj.integrate_maxt);
            nh.param("integrate_stept", traj.integrate_stept, traj.integrate_stept);
            nh.param("max_pose_pen_dist", traj.max_pose_pen_dist, traj.max_pose_pen_dist);
            nh.param("c_obs", traj.c_obs, traj.c_obs);
            nh.param("pose_exp_weight", traj.pose_exp_weight, traj.pose_exp_weight);
            nh.param("inf_ratio", traj.inf_ratio, traj.inf_ratio);
            nh.param("terminal_weight", traj.terminal_weight, traj.terminal_weight);
            nh.param("num_curve_points", traj.num_curve_points, traj.num_curve_points);

            // Control Params
            nh.param("k_fb_x",control.k_fb_x, control.k_fb_x);
            nh.param("k_fb_y",control.k_fb_y, control.k_fb_y);
            nh.param("k_fb_theta",control.k_fb_theta, control.k_fb_theta);
            nh.param("ctrl_ahead_pose",control.ctrl_ahead_pose, control.ctrl_ahead_pose);

            // Projection Params
            nh.param("k_po_x", projection.k_po_x, projection.k_po_x);
            nh.param("k_po_theta", projection.k_po_theta, projection.k_po_theta);
            nh.param("r_unity", projection.r_unity, projection.r_unity);
            nh.param("r_zero", projection.r_zero, projection.r_zero);
            nh.param("k_CBF", projection.k_CBF, projection.k_CBF);
            nh.param("cbf_param", projection.cbf_param, projection.cbf_param);
        } else
        {
            throw std::runtime_error("Model " + model + " not implemented!");
        }
    }

    void DynamicGapConfig::updateParamFromScan(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr)
    {
        sensor_msgs::LaserScan incomingScan = *scanPtr.get();
        scan.angle_min = incomingScan.angle_min;
        scan.angle_max = incomingScan.angle_max;
        scan.full_scan = incomingScan.ranges.size();
        scan.full_scan_f = float(scan.full_scan);
        scan.half_scan = scan.full_scan / 2;
        scan.half_scan_f = float(scan.half_scan);        
        scan.angle_increment = (2 * M_PI) / (scan.full_scan_f - 1);
        // scan.range_max = 4.99; // this is the maximum possible range, not the max range within a particular scan
        // scan.range_min = 0.0;
    }

}