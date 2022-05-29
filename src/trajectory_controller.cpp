#include <dynamic_gap/trajectory_controller.h>

namespace dynamic_gap{
    TrajectoryController::TrajectoryController(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg) {
        projection_viz = nh.advertise<visualization_msgs::Marker>("po_dir", 10);
        cfg_ = & cfg;
        thres = 0.1;
        last_time = ros::Time::now();
    }

    void TrajectoryController::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        boost::mutex::scoped_lock lock(egocircle_l);
        msg_ = msg;
    }

    std::vector<geometry_msgs::Point> TrajectoryController::findLocalLine(int idx) {
        // get egocircle measurement
        auto egocircle = *msg_.get();
        std::vector<double> dist(egocircle.ranges.size());

        if (!msg_) {
            return std::vector<geometry_msgs::Point>(0);
        }
        
        if (egocircle.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect findLocalLine");
        }


        // iterating through egocircle
        for (int i = 1; i < dist.size(); i++) {
            // current distance/idx
            float l1 = egocircle.ranges.at(i);
            float t1 = float(i) * egocircle.angle_increment + egocircle.angle_min;
            // prior distance/idx
            float l2 = egocircle.ranges.at(i - 1);
            float t2 = float(i - 1) * egocircle.angle_increment + egocircle.angle_min;
            
            // if current distance is big, set dist to big
            if (l1 > 2.9) {
                dist.at(i) = 10;
            } else {
                // get distance between two indices
                dist.at(i) = polDist(l1, t1, l2, t2);
            } 
        }

        dist.at(0) = polDist(egocircle.ranges.at(0), egocircle.angle_min, egocircle.ranges.at(511), float(511) * egocircle.angle_increment + egocircle.angle_min);

        // searching forward for big enough distances
        auto result_fwd = std::find_if(dist.begin() + idx, dist.end(), 
            std::bind1st(std::mem_fun(&TrajectoryController::geqThres), this));

        // searching backward for big enough distances
        auto res_rev = std::find_if(dist.rbegin() + (dist.size() - idx), dist.rend(),
            std::bind1st(std::mem_fun(&TrajectoryController::geqThres), this));
        

        if (res_rev == dist.rend()) {
            return std::vector<geometry_msgs::Point>(0);
        }

        // get index for big enough distance
        int idx_fwd = std::distance(dist.begin(), std::prev(result_fwd));
        int idx_rev = std::distance(res_rev, dist.rend());

        // are indices valid
        int min_idx_range = 0;
        int max_idx_range = int(egocircle.ranges.size() - 1);
        if (idx_fwd < min_idx_range || idx_fwd > max_idx_range || idx_rev < min_idx_range || idx_rev > max_idx_range) {
            return std::vector<geometry_msgs::Point>(0);
        }
        
        float dist_fwd = egocircle.ranges.at(idx_fwd);
        float dist_rev = egocircle.ranges.at(idx_rev);
        float dist_cent = egocircle.ranges.at(idx);

        double angle_fwd = double(idx_fwd) * egocircle.angle_increment + egocircle.angle_min;
        double angle_rev = double(idx_rev) * egocircle.angle_increment + egocircle.angle_min;
        
        if (idx_fwd < idx || idx_rev > idx) {
            return std::vector<geometry_msgs::Point>(0);
        }

        Eigen::Vector2d fwd_pol(dist_fwd, angle_fwd);
        Eigen::Vector2d rev_pol(dist_rev, angle_rev);
        Eigen::Vector2d cent_pol(dist_cent, double(idx) * egocircle.angle_increment + egocircle.angle_min);
        Eigen::Vector2d fwd_car = pol2car(fwd_pol);
        Eigen::Vector2d rev_car = pol2car(rev_pol);
        Eigen::Vector2d cent_car = pol2car(cent_pol);

        Eigen::Vector2d pf;
        Eigen::Vector2d pr;

        if (dist_cent < dist_fwd && dist_cent < dist_rev) {
            // ROS_INFO_STREAM("Non line");
            Eigen::Vector2d a = cent_car - fwd_car;
            Eigen::Vector2d b = rev_car - fwd_car;
            Eigen::Vector2d a1 = (a.dot(b / b.norm())) * (b / b.norm());
            Eigen::Vector2d a2 = a - a1;
            pf = fwd_car + a2;
            pr = rev_car + a2;
        } else {
            pf = fwd_car;
            pr = rev_car;
        }

        geometry_msgs::Point lower_point;
        lower_point.x = pf(0);
        lower_point.y = pf(1);
        lower_point.z = 3;
        geometry_msgs::Point upper_point;
        upper_point.x = pr(0);
        upper_point.y = pr(1);
        upper_point.z = 3;
        // if form convex hull
        std::vector<geometry_msgs::Point> retArr(0);
        retArr.push_back(lower_point);
        retArr.push_back(upper_point);
        return retArr;
    }

    bool TrajectoryController::leqThres(const double dist) {
        return dist <= thres;
    }

    bool TrajectoryController::geqThres(const double dist) {
        return dist >= thres;
    }

    double TrajectoryController::polDist(float l1, float t1, float l2, float t2) {
        return abs(double(pow(l1, 2) + pow(l2, 2) - 2 * l1 * l2 * std::cos(t1 - t2)));
    }

    geometry_msgs::Twist TrajectoryController::controlLaw(
        geometry_msgs::Pose current, nav_msgs::Odometry desired,
        sensor_msgs::LaserScan inflated_egocircle, geometry_msgs::PoseStamped rbt_in_cam_lc
    ) {
        // Setup Vars
        boost::mutex::scoped_lock lock(egocircle_l);
        bool holonomic = cfg_->planning.holonomic;
        bool full_fov = cfg_->planning.full_fov;
        bool projection_operator = cfg_->planning.projection_operator;
        double k_turn_ = cfg_->control.k_turn;
        if (holonomic && full_fov) k_turn_ = 0.8;
        double k_drive_x_ = cfg_->control.k_drive_x;
        double k_drive_y_ = cfg_->control.k_drive_y;
        double k_po_ = cfg_->projection.k_po;
        float v_ang_const = cfg_->control.v_ang_const;
        float v_lin_x_const = cfg_->control.v_lin_x_const;
        float v_lin_y_const = cfg_->control.v_lin_y_const;
        float r_min = cfg_->projection.r_min;
        float r_norm = cfg_->projection.r_norm;
        float r_norm_offset = cfg_->projection.r_norm_offset; 
        float k_po_turn_ = cfg_->projection.k_po_turn;

        // auto inflated_egocircle = *msg_.get();
        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Point position = current.position;
        geometry_msgs::Quaternion orientation = current.orientation;

        // obtain roll, pitch, and yaw of current orientation (I think we're only using yaw)
        tf::Quaternion q_c(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
        tf::Matrix3x3 m_c(q_c);
        double c_roll, c_pitch, c_yaw;
        m_c.getRPY(c_roll, c_pitch, c_yaw);

        // get current x,y,theta
        Eigen::Matrix2cd g_curr = getComplexMatrix(position.x, position.y, c_yaw);

        // obtaining RPY of desired orientation
        position = desired.pose.pose.position;
        orientation = desired.pose.pose.orientation;
        tf::Quaternion q_d(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
        tf::Matrix3x3 m_d(q_d);
        double d_roll, d_pitch, d_yaw;
        m_d.getRPY(d_roll, d_pitch, d_yaw);

        // get desired x,y,theta
        Eigen::Matrix2cd g_des = getComplexMatrix(position.x, position.y, d_yaw);

        // get x,y,theta error
        Eigen::Matrix2cd g_error = g_curr.inverse() * g_des;
        float theta_error = std::arg(g_error(0, 0));
        float x_error = g_error.real()(0, 1);
        float y_error = g_error.imag()(0, 1);


        float cmd_vel_x_safe = 0;
        float cmd_vel_y_safe = 0;
        double v_ang_fb = 0;
        double v_lin_x_fb = 0;
        double v_lin_y_fb = 0;

        // obtain feedback velocities
        if (cfg_->man.man_ctrl) {
            ROS_INFO_STREAM("Manual Control");
            v_ang_fb = cfg_->man.man_theta;
            v_lin_x_fb = cfg_->man.man_x;
            v_lin_y_fb = cfg_->man.man_y;
        } else {
            v_ang_fb = theta_error * k_turn_;
            v_lin_x_fb = x_error * k_drive_x_;
            v_lin_y_fb = y_error * k_drive_y_;
        }

        ROS_INFO_STREAM("Feedback command velocities, v_x: " << v_lin_x_fb << ", v_y: " << v_lin_y_fb << ", v_ang: " << v_ang_fb);

        float min_dist_ang = 0;
        float min_dist = 0;

        float min_diff_x = 0;
        float min_diff_y = 0;

        // ROS_INFO_STREAM(rbt_in_cam_lc.pose);
        
        Eigen::Vector3d comp;
        double PO_dot_prod_check;
        Eigen::Vector2d Psi_der;
        double Psi;
        Eigen::Vector2d cmd_vel_fb(v_lin_x_fb, v_lin_y_fb);


        if (inflated_egocircle.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect controlLaw");
        }

        // applies PO
        if(projection_operator)
        {
            std::vector<double> min_dist_arr(inflated_egocircle.ranges.size());
            for (int i = 0; i < min_dist_arr.size(); i++) {
                float angle = i * inflated_egocircle.angle_increment - M_PI;
                float dist = inflated_egocircle.ranges.at(i);
                min_dist_arr.at(i) = dist2Pose(angle, dist, rbt_in_cam_lc.pose);
            }
            int min_idx = std::min_element( min_dist_arr.begin(), min_dist_arr.end() ) - min_dist_arr.begin();

            // ROS_INFO_STREAM("Local Line Start");
            std::vector<geometry_msgs::Point> vec = findLocalLine(min_idx);
            // ROS_INFO_STREAM("Local Line End");
            /*
            if (vec.size() > 1) {
                // Visualization and Recenter
                vec.at(0).x -= rbt_in_cam_lc.pose.position.x;
                vec.at(0).y -= rbt_in_cam_lc.pose.position.y;
                vec.at(1).x -= rbt_in_cam_lc.pose.position.x;
                vec.at(1).y -= rbt_in_cam_lc.pose.position.y;

                std::vector<std_msgs::ColorRGBA> color;
                std_msgs::ColorRGBA std_color;
                std_color.a = 1;
                std_color.r = 1;
                std_color.g = 0.1;
                std_color.b = 0.1;
                color.push_back(std_color);
                color.push_back(std_color);
                visualization_msgs::Marker line_viz;
                line_viz.header.frame_id = cfg_->robot_frame_id;
                line_viz.type = visualization_msgs::Marker::LINE_STRIP;
                line_viz.action = visualization_msgs::Marker::ADD;
                line_viz.points = vec;
                line_viz.colors = color;
                line_viz.scale.x = 0.01;
                line_viz.scale.y = 0.1;
                line_viz.scale.z = 0.1;
                line_viz.id = 10;
                projection_viz.publish(line_viz);
            }
            */

            // ROS_DEBUG_STREAM("Elapsed: " << (ros::Time::now() - last_time).toSec());
            last_time = ros::Time::now();
            float r_max = r_norm + r_norm_offset;
            min_dist = (float) min_dist_arr.at(min_idx);
            min_dist = min_dist >= r_max ? r_max : min_dist;
            if (min_dist <= 0) ROS_INFO_STREAM("Min dist <= 0, : " << min_dist);
            min_dist = min_dist <= 0 ? 0.01 : min_dist;
            // min_dist -= cfg_->rbt.r_inscr / 2;

            // find minimum ego circle dist
            min_dist_ang = (float)(min_idx) * inflated_egocircle.angle_increment + inflated_egocircle.angle_min;
            float min_x = min_dist * cos(min_dist_ang) - rbt_in_cam_lc.pose.position.x;
            float min_y = min_dist * sin(min_dist_ang) - rbt_in_cam_lc.pose.position.y;
            min_dist = sqrt(pow(min_x, 2) + pow(min_y, 2));

            ROS_INFO_STREAM("min_x: " << min_x << ", min_y: " << min_y);

            if (cfg_->man.line && vec.size() > 0) {
                // Dist to 
                Eigen::Vector2d pt1(vec.at(0).x, vec.at(0).y);
                Eigen::Vector2d pt2(vec.at(1).x, vec.at(1).y);
                Eigen::Vector2d rbt(0, 0);
                Eigen::Vector2d a = rbt - pt1;
                Eigen::Vector2d b = pt2 - pt1;
                Eigen::Vector2d c = rbt - pt2;
                
                if (a.dot(b) < 0) {
                    // Dist to pt1
                    // ROS_INFO_STREAM("Pt1");
                    min_diff_x = - pt1(0);
                    min_diff_y = - pt1(1);
                    comp = projection_method(min_diff_x, min_diff_y);
                    Psi_der = Eigen::Vector2d(comp(0), comp(1));
                    PO_dot_prod_check = cmd_vel_fb.dot(Psi_der);
                } else if (c.dot(-b) < 0) {
                    min_diff_x = - pt2(0);
                    min_diff_y = - pt2(1);
                    comp = projection_method(min_diff_x, min_diff_y);
                    Psi_der = Eigen::Vector2d(comp(0), comp(1));
                    PO_dot_prod_check = cmd_vel_fb.dot(Psi_der);
                } else {
                    double line_dist = (pt1(0) * pt2(1) - pt2(0) * pt1(1)) / (pt1 - pt2).norm();
                    double sign;
                    sign = line_dist < 0 ? -1 : 1;
                    line_dist *= sign;
                    
                    double line_si = (r_min / line_dist - r_min / r_norm) / (1. - r_min / r_norm);
                    double line_Psi_der_base = r_min / (pow(line_dist, 2) * (r_min / r_norm - 1));
                    double line_Psi_der_x = - (pt2(1) - pt1(1)) / (pt1 - pt2).norm() * line_Psi_der_base;
                    double line_Psi_der_y = - (pt1(0) - pt2(0)) / (pt1 - pt2).norm() * line_Psi_der_base;
                    Eigen::Vector2d der(line_Psi_der_x, line_Psi_der_y);
                    der /= der.norm();
                    comp = Eigen::Vector3d(der(0), der(1), line_si);
                    Psi_der = der;
                    Psi_der(1);// /= 3;
                    PO_dot_prod_check = cmd_vel_fb.dot(Psi_der);
                }

            } else {
                min_diff_x = - min_x;
                min_diff_y = - min_y;
                comp = projection_method(min_diff_x, min_diff_y);
                Psi_der = Eigen::Vector2d(comp(0), comp(1));
                Psi_der(1);// /= 3; // deriv wrt y?
                PO_dot_prod_check = cmd_vel_fb.dot(Psi_der);
            }
            ROS_INFO_STREAM("Psi_der: " << Psi_der[0] << ", " << Psi_der[1]);

            Psi = comp(2);

            ROS_INFO_STREAM("Psi: " << Psi << ", dot product check: " << PO_dot_prod_check);
            if(Psi >= 0 && PO_dot_prod_check <= 0)
            {
                cmd_vel_x_safe = Psi * PO_dot_prod_check * - comp(0);
                cmd_vel_y_safe = Psi * PO_dot_prod_check * - comp(1);
                ROS_INFO_STREAM("cmd_vel_safe: " << cmd_vel_x_safe << ", " << cmd_vel_y_safe);
            }

            // cmd_vel_safe
            if (cmd_vel_x_safe != 0 || cmd_vel_y_safe != 0) {
                visualization_msgs::Marker res;
                res.header.frame_id = cfg_->robot_frame_id;
                res.type = visualization_msgs::Marker::ARROW;
                res.action = visualization_msgs::Marker::ADD;
                res.pose.position.x = 0;
                res.pose.position.y = 0;
                res.pose.position.z = 1;
                double dir = std::atan2(cmd_vel_y_safe, cmd_vel_x_safe);
                tf2::Quaternion dir_quat;
                dir_quat.setRPY(0, 0, dir);
                res.pose.orientation = tf2::toMsg(dir_quat);

                res.scale.x = sqrt(pow(cmd_vel_y_safe, 2) + pow(cmd_vel_x_safe, 2));
                res.scale.y = 0.01;  
                res.scale.z = 0.01;
                
                res.color.a = 1;
                res.color.r = 0.9;
                res.color.g = 0.9;
                res.color.b = 0.9;
                res.id = 0;
                res.lifetime = ros::Duration(0.1);
                projection_viz.publish(res);
            }

            /*
            // Min Direction
            res.header.frame_id = cfg_->sensor_frame_id;
            res.scale.x = 1;
            dir_quat.setRPY(0, 0, min_dist_ang);
            res.pose.orientation = tf2::toMsg(dir_quat);
            res.id = 1;
            res.color.a = 0.5;
            res.pose.position.z = 0.9;
            projection_viz.publish(res);
            */
            /*
            res.header.frame_id = cfg_->robot_frame_id;
            res.type = visualization_msgs::Marker::SPHERE;
            res.action = visualization_msgs::Marker::ADD;
            res.pose.position.x = -min_diff_x;
            res.pose.position.y = -min_diff_y;
            res.pose.position.z = 1;
            res.scale.x = 0.1;
            res.scale.y = 0.1;
            res.scale.z = 0.1;
            res.id = 2;
            // res.color.a = 0.5;
            // res.pose.position.z = 0.9;
            projection_viz.publish(res);
            */
        } else {
            ROS_DEBUG_STREAM_THROTTLE(10, "Projection operator off");
        }

        // Make sure no ejection from gap. Max question: x does not always point into gap. 
        cmd_vel_x_safe = std::min(cmd_vel_x_safe, float(0));

        if(holonomic)
        {
            v_ang_fb = v_ang_fb + v_ang_const;
            v_lin_x_fb = abs(theta_error) > M_PI / 3 ? 0 : v_lin_x_fb + v_lin_x_const + k_po_ * cmd_vel_x_safe;
            v_lin_y_fb = abs(theta_error) > M_PI / 3 ? 0 : v_lin_y_fb + v_lin_y_const + k_po_ * cmd_vel_y_safe;

            if(v_lin_x_fb < 0)
                v_lin_x_fb = 0;

            ROS_INFO_STREAM("ultimate command velocity, v_x:" << v_lin_x_fb << ", v_y: " << v_lin_y_fb << ", v_ang: " << v_ang_fb);
        }
        else
        {
            v_ang_fb = v_ang_fb + v_lin_y_fb + k_po_turn_ * cmd_vel_y_safe + v_ang_const;
            v_lin_x_fb = v_lin_x_fb + v_lin_x_const + k_po_ * cmd_vel_x_safe;

            if (projection_operator && min_dist_ang > - M_PI / 4 && min_dist_ang < M_PI / 4 && min_dist < cfg_->rbt.r_inscr)
            {
                v_lin_x_fb = 0;
                v_ang_fb *= 2;
            }

            v_lin_y_fb = 0;

            if(v_lin_x_fb < 0)
                v_lin_x_fb = 0;
        }

        cmd_vel.linear.x = std::max(-cfg_->control.vx_absmax, std::min(cfg_->control.vx_absmax, v_lin_x_fb));
        cmd_vel.linear.y = std::max(-cfg_->control.vy_absmax, std::min(cfg_->control.vy_absmax, v_lin_y_fb));
        cmd_vel.angular.z = std::max(-cfg_->control.vang_absmax, std::min(cfg_->control.vang_absmax, v_ang_fb));
        return cmd_vel;
    }

    Eigen::Vector3d TrajectoryController::projection_method(float min_diff_x, float min_diff_y) {
        float r_min = cfg_->projection.r_min;
        float r_norm = cfg_->projection.r_norm;

        float min_dist = sqrt(pow(min_diff_x, 2) + pow(min_diff_y, 2));
        float Psi = (r_min / min_dist - r_min / r_norm) / (1.0 - r_min / r_norm);
        float base_const = pow(min_dist, 3) * (r_min - r_norm);
        float up_const = r_min * r_norm;
        float Psi_der_x = up_const * - min_diff_x / base_const;
        float Psi_der_y = up_const * - min_diff_y / base_const;

        float norm_Psi_der = sqrt(pow(Psi_der_x, 2) + pow(Psi_der_y, 2));
        float norm_Psi_der_x = Psi_der_x / norm_Psi_der;
        float norm_Psi_der_y = Psi_der_y / norm_Psi_der;
        return Eigen::Vector3d(norm_Psi_der_x, norm_Psi_der_y, Psi);
    }

    Eigen::Matrix2cd TrajectoryController::getComplexMatrix(
        double x, double y, double quat_w, double quat_z)
    {
        std::complex<double> phase(quat_w, quat_z);
        phase = phase * phase;

        Eigen::Matrix2cd g(2, 2);
        //g.real()(0,0) = phase.real();
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        //g.imag()(0,0) = phase.imag();
        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }

    Eigen::Matrix2cd TrajectoryController::getComplexMatrix(
        double x, double y, double theta)
    {
        std::complex<double> phase(std::cos(theta), std::sin(theta));

        Eigen::Matrix2cd g(2, 2);
        //g.real()(0,0) = phase.real();
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        //g.imag()(0,0) = phase.imag();
        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }


    int TrajectoryController::targetPoseIdx(geometry_msgs::Pose curr_pose, dynamic_gap::TrajPlan ref_pose) {
        // Find pose right ahead
        std::vector<double> pose_diff(ref_pose.poses.size());
        // ROS_INFO_STREAM("Ref_pose length: " << ref_pose.poses.size());

        // obtain distance from entire ref traj and current pose
        for (int i = 0; i < pose_diff.size(); i++) // i will always be positive, so this is fine
        {
            pose_diff[i] = sqrt(pow(curr_pose.position.x - ref_pose.poses[i].position.x, 2) + 
                                pow(curr_pose.position.y - ref_pose.poses[i].position.y, 2)) + 
                                0.5 * (1 - (   curr_pose.orientation.x * ref_pose.poses[i].orientation.x + 
                                        curr_pose.orientation.y * ref_pose.poses[i].orientation.y +
                                        curr_pose.orientation.z * ref_pose.poses[i].orientation.z +
                                        curr_pose.orientation.w * ref_pose.poses[i].orientation.w)
                                );
        }

        // find pose in ref traj with smallest difference
        auto min_element_iter = std::min_element(pose_diff.begin(), pose_diff.end());
        // go n steps ahead of pose with smallest difference
        int target_pose = std::distance(pose_diff.begin(), min_element_iter) + cfg_->control.ctrl_ahead_pose;
        return std::min(target_pose, int(ref_pose.poses.size() - 1));
    }


    dynamic_gap::TrajPlan TrajectoryController::trajGen(geometry_msgs::PoseArray orig_traj)
    {
        dynamic_gap::TrajPlan traj;
        traj.header.frame_id = cfg_->odom_frame_id;
        for(size_t i = 0; i < orig_traj.poses.size(); i++)
        {
            geometry_msgs::Pose ni_pose = orig_traj.poses[i];
            geometry_msgs::Twist ni_twist;
            traj.poses.push_back(ni_pose);
            traj.twist.push_back(ni_twist);
        }
        return traj;
    }

    double TrajectoryController::dist2Pose(float theta, float dist, geometry_msgs::Pose pose) {
        float x = dist * std::cos(theta);
        float y = dist * std::sin(theta);
        return sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2));
    }


    Eigen::Vector2d TrajectoryController::car2pol(Eigen::Vector2d a) {
        return Eigen::Vector2d(a.norm(), float(std::atan2(a(1), a(0))));
    }

    Eigen::Vector2d TrajectoryController::pol2car(Eigen::Vector2d a) {
        return Eigen::Vector2d(cos(a(1)) * a(0), sin(a(1)) * a(0));
    }


}