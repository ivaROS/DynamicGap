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

        // Robot
        nh.param("r_inscr", rbt.r_inscr, rbt.r_inscr);
        nh.param("vx_absmax",rbt.vx_absmax, rbt.vx_absmax);
        nh.param("vy_absmax",rbt.vy_absmax, rbt.vy_absmax);
        nh.param("vang_absmax", rbt.vang_absmax, rbt.vang_absmax);
        nh.param("ax_absmax",rbt.ax_absmax, rbt.ax_absmax);
        nh.param("ay_absmax",rbt.ay_absmax, rbt.ay_absmax);
        nh.param("aang_absmax", rbt.aang_absmax, rbt.aang_absmax);

        // Environment
        nh.param("num_agents", env.num_agents, env.num_agents);

        // Scan
        nh.param("max_range", scan.range_max, scan.range_max);

        // Planning Information
        nh.param("projection_operator", planning.projection_operator, planning.projection_operator);

        // Manual Control
        nh.param("man_ctrl", man.man_ctrl, man.man_ctrl);

        // Goal Param
        nh.param("goal_tolerance", goal.goal_tolerance, goal.goal_tolerance);
        nh.param("waypoint_tolerance", goal.waypoint_tolerance, goal.waypoint_tolerance);

        // Gap Association
        nh.param("assoc_thresh", gap_assoc.assoc_thresh, gap_assoc.assoc_thresh);

        // Gap Manipulation
        nh.param("radial_extend", gap_manip.radial_extend, gap_manip.radial_extend);

        // Trajectory
        nh.param("integrate_maxt", traj.integrate_maxt, traj.integrate_maxt);
        nh.param("integrate_stept", traj.integrate_stept, traj.integrate_stept);
        nh.param("max_pose_to_scan_dist", traj.max_pose_to_scan_dist, traj.max_pose_to_scan_dist);
        nh.param("Q", traj.Q, traj.Q);
        nh.param("Q_f", traj.Q_f, traj.Q_f);

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
        scan.range_max = 4.99; // this is the maximum possible range, not the max range within a particular scan
        // scan.range_min = 0.0;
    }

}