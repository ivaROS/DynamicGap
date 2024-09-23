#include <dynamic_gap/config/DynamicGapConfig.h>

namespace dynamic_gap 
{
    void DynamicGapConfig::loadRosParamFromNodeHandle(const std::string & name)
    {
        ros::NodeHandle nh("~/" + name);

        ROS_INFO_STREAM("Setting nh to: " << "~/" << name);

        std::string model;
        nh.param("/model", model, model); // Must write as "/model" with leading slash

        if (model == "rto")
        {
            // format: key, value, default value
            odom_frame_id = model + "/odom";
            robot_frame_id = model + "/base_link";
            sensor_frame_id = model + "/hokuyo_link";
            
            odom_topic = "odom"; // model + "/odom";
            acc_topic = "acc"; // model + "/acc";
            scan_topic = "scan"; // model + "/scan";

            // Robot
            nh.param("robot_radius", rbt.r_inscr, rbt.r_inscr);
            nh.param("max_vel_x",rbt.vx_absmax, rbt.vx_absmax);
            nh.param("max_vel_y",rbt.vy_absmax, rbt.vy_absmax);
            nh.param("max_vel_theta", rbt.vang_absmax, rbt.vang_absmax);

            // Scan
            nh.param("max_range", scan.range_max, scan.range_max);

            // Planning Information
            nh.param("projection_operator", planning.projection_operator, planning.projection_operator);
            nh.param("egocircle_prop_cheat", planning.egocircle_prop_cheat, planning.egocircle_prop_cheat);
            nh.param("heading", planning.heading, planning.heading);
            ROS_INFO_STREAM("       setting heading to " << planning.heading);
            nh.param("gap_feasibility_check", planning.gap_feasibility_check, planning.gap_feasibility_check);
            nh.param("perfect_gap_models", planning.perfect_gap_models, planning.perfect_gap_models);
            nh.param("future_scan_propagation", planning.future_scan_propagation, planning.future_scan_propagation);

            // Manual Control
            nh.param("man_ctrl", ctrl.man_ctrl, ctrl.man_ctrl);

            // Goal Param
            nh.param("xy_goal_tolerance", goal.goal_tolerance, goal.goal_tolerance);
            nh.param("yaw_goal_tolerance", goal.yaw_goal_tolerance, goal.yaw_goal_tolerance);
            nh.param("xy_waypoint_tolerance", goal.waypoint_tolerance, goal.waypoint_tolerance);

            // Gap Association
            nh.param("assoc_thresh", gap_assoc.assoc_thresh, gap_assoc.assoc_thresh);

            // Gap Manipulation
            nh.param("epsilon1", gap_manip.epsilon1, gap_manip.epsilon1);
            nh.param("epsilon2", gap_manip.epsilon2, gap_manip.epsilon2);
            nh.param("radial_extend", gap_manip.radial_extend, gap_manip.radial_extend);

            // Trajectory
            nh.param("integrate_maxt", traj.integrate_maxt, traj.integrate_maxt);
            nh.param("integrate_stept", traj.integrate_stept, traj.integrate_stept);
            nh.param("max_pose_to_scan_dist", traj.max_pose_to_scan_dist, traj.max_pose_to_scan_dist);
            nh.param("inf_ratio", traj.inf_ratio, traj.inf_ratio);
            nh.param("Q", traj.Q, traj.Q);            
            nh.param("Q_f", traj.Q_f, traj.Q_f);

            // Control Params
            nh.param("k_fb_x",control.k_fb_x, control.k_fb_x);
            nh.param("k_fb_y",control.k_fb_y, control.k_fb_y);
            nh.param("k_fb_theta",control.k_fb_theta, control.k_fb_theta);
            nh.param("ctrl_ahead_pose",control.ctrl_ahead_pose, control.ctrl_ahead_pose);

            // Projection Params
            nh.param("k_po_x", projection.k_po_x, projection.k_po_x);
            nh.param("r_unity", projection.r_unity, projection.r_unity);
            nh.param("r_zero", projection.r_zero, projection.r_zero);
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

        scan.range_max = incomingScan.range_max; // this is the maximum detectable range, not the max range within a particular scan
        scan.range_min = incomingScan.range_min;
    }

}