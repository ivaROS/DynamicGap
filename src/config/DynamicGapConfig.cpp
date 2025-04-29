#include <dynamic_gap/config/DynamicGapConfig.h>

namespace dynamic_gap 
{
    void DynamicGapConfig::loadRosParamFromNodeHandle(const std::string & name)
    {
        ros::NodeHandle nh("~/" + name);

        ROS_INFO_STREAM_NAMED("Parameters", "Setting nh to: " << "~/" << name);

        std::string model;
        // nh.param("/model", model, model); // Must write as "/model" with leading slash
        ros_throw_param_load(nh, "/model", model); // Must write as "/model" with leading slash

        if (model == "rto")
        {
            ROS_INFO_STREAM_NAMED("Parameters", "Setting model to: " << model);

            ROS_INFO_STREAM_NAMED("Parameters", "map_frame_id is: " << map_frame_id);

            odom_frame_id = model + "/odom";
            ROS_INFO_STREAM_NAMED("Parameters", "Setting odom_frame_id to: " << odom_frame_id);

            robot_frame_id = model + "/base_link";
            ROS_INFO_STREAM_NAMED("Parameters", "Setting robot_frame_id to: " << robot_frame_id);

            sensor_frame_id = model + "/hokuyo_link";
            ROS_INFO_STREAM_NAMED("Parameters", "Setting sensor_frame_id to: " << sensor_frame_id);

            odom_topic = "odom"; // model + "/odom";
            ROS_INFO_STREAM_NAMED("Parameters", "Setting odom_topic to: " << odom_topic);

            acc_topic = "acc"; // model + "/acc";
            ROS_INFO_STREAM_NAMED("Parameters", "Setting acc_topic to: " << acc_topic);
            
            scan_topic = "scan"; // model + "/scan";
            ROS_INFO_STREAM_NAMED("Parameters", "Setting scan_topic to: " << scan_topic);

            ROS_INFO_STREAM_NAMED("Parameters", "ped_topic is: " << ped_topic);

            // Robot
            // nh.param("robot_radius", rbt.r_inscr, rbt.r_inscr);
            ros_throw_param_load(nh, "robot_radius", rbt.r_inscr);

            // nh.param("max_vel_x",rbt.vx_absmax, rbt.vx_absmax);
            
            ros_throw_param_load(nh, "max_vel_x", rbt.vx_absmax);
            
            // nh.param("max_vel_y",rbt.vy_absmax, rbt.vy_absmax);
            ros_throw_param_load(nh, "max_vel_y", rbt.vy_absmax);

            // nh.param("max_vel_theta", rbt.vang_absmax, rbt.vang_absmax);
            ros_throw_param_load(nh, "max_vel_theta", rbt.vang_absmax);

            // Scan
            // nh.param("max_range", scan.range_max, scan.range_max);
            ROS_INFO_STREAM_NAMED("Parameters", "angle_min is: " << scan.angle_min);
            ROS_INFO_STREAM_NAMED("Parameters", "angle_max is: " << scan.angle_max);
            ROS_INFO_STREAM_NAMED("Parameters", "full_scan is: " << scan.full_scan);
            ROS_INFO_STREAM_NAMED("Parameters", "full_scan_f is: " << scan.full_scan_f);
            ROS_INFO_STREAM_NAMED("Parameters", "half_scan is: " << scan.half_scan);
            ROS_INFO_STREAM_NAMED("Parameters", "half_scan_f is: " << scan.half_scan_f);
            ROS_INFO_STREAM_NAMED("Parameters", "angle_increment is: " << scan.angle_increment);
            ROS_INFO_STREAM_NAMED("Parameters", "range_min is: " << scan.range_min);
            ros_throw_param_load(nh, "max_range", scan.range_max);

            // Planning Information
            ROS_INFO_STREAM_NAMED("Parameters", "gap_prop is: " << planning.gap_prop);
            ROS_INFO_STREAM_NAMED("Parameters", "pursuit_guidance_method is: " << planning.pursuit_guidance_method);
            // nh.param("holonomic", planning.holonomic, planning.holonomic);
            ros_throw_param_load(nh, "holonomic", planning.holonomic);
            // nh.param("future_scan_propagation", planning.future_scan_propagation, planning.future_scan_propagation);
            ros_throw_param_load(nh, "future_scan_propagation", planning.future_scan_propagation);
            // nh.param("egocircle_prop_cheat", planning.egocircle_prop_cheat, planning.egocircle_prop_cheat);
            ros_throw_param_load(nh, "egocircle_prop_cheat", planning.egocircle_prop_cheat);
            // nh.param("projection_operator", planning.projection_operator, planning.projection_operator);
            ros_throw_param_load(nh, "projection_operator", planning.projection_operator);
            // nh.param("gap_feasibility_check", planning.gap_feasibility_check, planning.gap_feasibility_check);
            ros_throw_param_load(nh, "gap_feasibility_check", planning.gap_feasibility_check);
            nh.param("perfect_gap_models", planning.perfect_gap_models, planning.perfect_gap_models);
            ros_throw_param_load(nh, "perfect_gap_models", planning.perfect_gap_models);


            // nh.param("heading", planning.heading, planning.heading);
            // ROS_INFO_STREAM_NAMED("Planner", "       setting heading to " << planning.heading);

            // Manual Control
            // nh.param("man_ctrl", ctrl.man_ctrl, ctrl.man_ctrl);
            ROS_INFO_STREAM_NAMED("Parameters", "man_ctrl is: " << ctrl.man_ctrl);
            ROS_INFO_STREAM_NAMED("Parameters", "mpc_ctrl is: " << ctrl.mpc_ctrl);
            ROS_INFO_STREAM_NAMED("Parameters", "feedback_ctrl is: " << ctrl.feedback_ctrl);

            // Goal Param
            // nh.param("xy_goal_tolerance", goal.xy_global_goal_tolerance, goal.xy_global_goal_tolerance);
            ros_throw_param_load(nh, "xy_goal_tolerance", goal.xy_global_goal_tolerance);
            // nh.param("yaw_goal_tolerance", goal.yaw_global_goal_tolerance, goal.yaw_global_goal_tolerance);
            ros_throw_param_load(nh, "yaw_goal_tolerance", goal.yaw_global_goal_tolerance);
            // nh.param("xy_waypoint_tolerance", goal.xy_waypoint_tolerance, goal.xy_waypoint_tolerance);
            ros_throw_param_load(nh, "xy_waypoint_tolerance", goal.xy_waypoint_tolerance);

            // Gap Association
            // nh.param("assoc_thresh", gap_assoc.assoc_thresh, gap_assoc.assoc_thresh);
            ros_throw_param_load(nh, "assoc_thresh", gap_assoc.assoc_thresh);

            // Gap Manipulation
            // nh.param("rgc_angle", gap_manip.rgc_angle, gap_manip.rgc_angle);
            ros_throw_param_load(nh, "rgc_angle", gap_manip.rgc_angle);
            // nh.param("radial_extend", gap_manip.radial_extend, gap_manip.radial_extend);

            // Trajectory
            // nh.param("integrate_maxt", traj.integrate_maxt, traj.integrate_maxt);
            ros_throw_param_load(nh, "integrate_maxt", traj.integrate_maxt);

            // nh.param("integrate_stept", traj.integrate_stept, traj.integrate_stept);
            ros_throw_param_load(nh, "integrate_stept", traj.integrate_stept);

            // nh.param("max_pose_to_scan_dist", traj.max_pose_to_scan_dist, traj.max_pose_to_scan_dist);
            ros_throw_param_load(nh, "max_pose_to_scan_dist", traj.max_pose_to_scan_dist);

            // nh.param("Q", traj.Q, traj.Q);            
            ros_throw_param_load(nh, "Q", traj.Q);

            ros_throw_param_load(nh, "pen_exp_weight", traj.pen_exp_weight);

            // nh.param("inf_ratio", traj.inf_ratio, traj.inf_ratio);
            ros_throw_param_load(nh, "inf_ratio", traj.inf_ratio);

            // nh.param("Q_f", traj.Q_f, traj.Q_f);
            ros_throw_param_load(nh, "Q_f", traj.Q_f);

            // Control Params
            // nh.param("k_fb_x",control.k_fb_x, control.k_fb_x);
            // ros_throw_param_load(nh, "k_fb_x", control.k_fb_x);

            // nh.param("k_fb_y",control.k_fb_y, control.k_fb_y);
            // ros_throw_param_load(nh, "k_fb_y", control.k_fb_y);

            // nh.param("k_fb_theta",control.k_fb_theta, control.k_fb_theta);
            // ros_throw_param_load(nh, "k_fb_theta", control.k_fb_theta);

            // nh.param("ctrl_ahead_pose",control.ctrl_ahead_pose, control.ctrl_ahead_pose);
            ros_throw_param_load(nh, "ctrl_ahead_pose", control.ctrl_ahead_pose);

            // Projection Params
            // nh.param("k_po_x", projection.k_po_x, projection.k_po_x);
            ros_throw_param_load(nh, "k_po_x", projection.k_po_x);

            // nh.param("r_unity", projection.r_unity, projection.r_unity);
            ros_throw_param_load(nh, "r_unity", projection.r_unity);

            // nh.param("r_zero", projection.r_zero, projection.r_zero);
            ros_throw_param_load(nh, "r_zero", projection.r_zero);
        } else
        {
            throw std::runtime_error("Model " + model + " not implemented!");
        }
    }

    void DynamicGapConfig::updateParamFromScan(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr)
    {
        ROS_INFO_STREAM_ONCE_NAMED("Parameters", "[updateParamFromScan]: ");

        sensor_msgs::LaserScan incomingScan = *scanPtr.get();
        scan.angle_min = incomingScan.angle_min;
        scan.angle_max = incomingScan.angle_max;
        scan.full_scan = incomingScan.ranges.size();
        scan.full_scan_f = float(scan.full_scan);
        scan.half_scan = 0.5 * scan.full_scan;
        scan.half_scan_f = float(scan.half_scan);        
        scan.angle_increment = (2 * M_PI) / (scan.full_scan_f - 1);

        scan.range_max = incomingScan.range_max; // maximum detectable range, not max range within a particular scan
        scan.range_min = incomingScan.range_min; // minimum detectable range, not min range within a particular scan
    
        ROS_INFO_STREAM_ONCE_NAMED("Parameters", "  Setting scan.angle_min to: " << scan.angle_min);
        ROS_INFO_STREAM_ONCE_NAMED("Parameters", "  Setting scan.angle_max to: " << scan.angle_max);
        ROS_INFO_STREAM_ONCE_NAMED("Parameters", "  Setting scan.full_scan to: " << scan.full_scan);
        ROS_INFO_STREAM_ONCE_NAMED("Parameters", "  Setting scan.full_scan_f to: " << scan.full_scan_f);
        ROS_INFO_STREAM_ONCE_NAMED("Parameters", "  Setting scan.half_scan to: " << scan.half_scan);
        ROS_INFO_STREAM_ONCE_NAMED("Parameters", "  Setting scan.half_scan_f to: " << scan.half_scan_f);
        ROS_INFO_STREAM_ONCE_NAMED("Parameters", "  Setting scan.angle_increment to: " << scan.angle_increment);
        ROS_INFO_STREAM_ONCE_NAMED("Parameters", "  Setting scan.range_max to: " << scan.range_max);
        ROS_INFO_STREAM_ONCE_NAMED("Parameters", "  Setting scan.range_min to: " << scan.range_min);
    }

}