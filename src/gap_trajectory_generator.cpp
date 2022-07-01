 #include <dynamic_gap/gap_trajectory_generator.h>

namespace dynamic_gap{
    std::tuple<geometry_msgs::PoseArray, std::vector<double>> GapTrajGenerator::generateTrajectory(
                                                    dynamic_gap::Gap& selectedGap, 
                                                    geometry_msgs::PoseStamped curr_pose, 
                                                    geometry_msgs::Twist curr_vel,
                                                    bool run_g2g) {
        try {        
            // return geometry_msgs::PoseArray();
            geometry_msgs::PoseArray posearr;
            std::vector<double> timearr;
            posearr.header.stamp = ros::Time::now();
            double start_time = ros::Time::now().toSec();
            double coefs = cfg_->traj.scale;
            write_trajectory corder(posearr, cfg_->robot_frame_id, coefs, timearr);
            posearr.header.frame_id = cfg_->traj.synthesized_frame ? cfg_->sensor_frame_id : cfg_->robot_frame_id;

            // ROS_INFO_STREAM("coefs:" << coefs);
            // flipping models here to be from robot's POV
            // dynamic_gap::cart_model* left_model = selectedGap.right_model;
            // dynamic_gap::cart_model* right_model = selectedGap.left_model;

            // Matrix<double, 4, 1> left_model_state = left_model->get_modified_polar_state();
            // Matrix<double, 4, 1> right_model_state = right_model->get_modified_polar_state();

            // curr_pose is in sensor frame, gaps are in robot frame?, curr_vel is in robot frame
            Eigen::Vector4d ego_x(curr_pose.pose.position.x + 1e-5, curr_pose.pose.position.y + 1e-6,
                                    curr_vel.linear.x, curr_vel.linear.y);
            /*
            state_type x = {                 
                            left_model_state[0], left_model_state[1], left_model_state[2], left_model_state[3], left_model_state[4],
                            right_model_state[0], right_model_state[1], right_model_state[2], right_model_state[3], right_model_state[4]}; 
            */

            // get gap points in cartesian
            float x1, x2, y1, y2;
            float half_num_scan = selectedGap.half_scan;
            x1 = (selectedGap.convex.convex_ldist) * cos(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
            y1 = (selectedGap.convex.convex_ldist) * sin(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
            x2 = (selectedGap.convex.convex_rdist) * cos(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);
            y2 = (selectedGap.convex.convex_rdist) * sin(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);

            float term_x1, term_x2, term_y1, term_y2;
            term_x1 = (selectedGap.convex.terminal_ldist) * cos(-((float) half_num_scan - selectedGap.convex.terminal_lidx) / half_num_scan * M_PI);
            term_y1 = (selectedGap.convex.terminal_ldist) * sin(-((float) half_num_scan - selectedGap.convex.terminal_lidx) / half_num_scan * M_PI);
            term_x2 = (selectedGap.convex.terminal_rdist) * cos(-((float) half_num_scan - selectedGap.convex.terminal_ridx) / half_num_scan * M_PI);
            term_y2 = (selectedGap.convex.terminal_rdist) * sin(-((float) half_num_scan - selectedGap.convex.terminal_ridx) / half_num_scan * M_PI);

            // std::cout << "is gap axial: " << selectedGap.isAxial() << std::endl;
            //std::cout << "original initial robot pos: (" << ego_x[0] << ", " << ego_x[1] << ")" << std::endl;
            //std::cout << "original inital robot velocity: " << ego_x[2] << ", " << ego_x[3] << ")" << std::endl;
            //std::cout << "original initial goal: (" << selectedGap.goal.x << ", " << selectedGap.goal.y << ")" << std::endl; 
            //std::cout << "original terminal goal: (" << selectedGap.terminal_goal.x << ", " << selectedGap.terminal_goal.y << ")" << std::endl; 
            //std::cout << "original initial left point: (" << x2 << ", " << y2 << "), original initial right point: (" << x1 << ", " << y1 << ")" << std::endl; 
            //std::cout << "original terminal left point: (" << term_x2 << ", " << term_y2 << "), original terminal right point: (" << term_x1 << ", " << term_y1 << ")" << std::endl;
            
            //std::cout << "starting left model state: " << left_model_state[0] << ", " <<  left_model_state[1] << ", " <<  left_model_state[2] << ", " <<  left_model_state[3] << ", " <<  left_model_state[4] << std::endl;
            //std::cout << "starting right model state: " << right_model_state[0] << ", " <<  right_model_state[1] << ", " <<  right_model_state[2] << ", " <<  right_model_state[3] << ", " <<  right_model_state[4] << std::endl;
            //int rbt_idx = int((std::atan2(ego_x[1],ego_x[0]) - (-M_PI)) / (M_PI / selectedGap.half_scan));
            //int left_idx = int((std::atan2(y1,x1) - (-M_PI)) / (M_PI / selectedGap.half_scan));
            //int right_idx = int((std::atan2(y2,x2) - (-M_PI)) / (M_PI / selectedGap.half_scan));
            //std::cout << "original rbt index: " << rbt_idx << ", original left index: " << left_idx << ", original right index: " << right_idx << std::endl;

            if (run_g2g) { //   || selectedGap.goal.goalwithin
                state_type x = {ego_x[0], ego_x[1], ego_x[2], ego_x[3],
                            0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0,0.0,
                            selectedGap.goal.x * coefs, selectedGap.goal.y * coefs};
                // ROS_INFO_STREAM("Goal to Goal");
                g2g inte_g2g(selectedGap.goal.x * coefs, selectedGap.goal.y * coefs,
                             selectedGap.terminal_goal.x * coefs, selectedGap.terminal_goal.y * coefs,
                             selectedGap.gap_lifespan, cfg_->control.vx_absmax);
                boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                inte_g2g, x, 0.0,
                cfg_->traj.integrate_maxt,
                cfg_->traj.integrate_stept,
                corder);
                std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
                return return_tuple;
            }

            Eigen::Vector2f qB = selectedGap.terminal_qB; // (selectedGap.qB + selectedGap.terminal_qB) / 2.0;
            // ROS_INFO_STREAM("qB: " << qB[0] << ", " << qB[1]);

            if (selectedGap.mode.convex) {
                selectedGap.goal.x -= qB(0);
                selectedGap.goal.y -= qB(1);
                selectedGap.terminal_goal.x -= qB(0);
                selectedGap.terminal_goal.y -= qB(1);

                x1 -= qB(0);
                x2 -= qB(0);
                y1 -= qB(1);
                y2 -= qB(1);
                term_x1 -= qB(0);
                term_x2 -= qB(0);
                term_y1 -= qB(1);
                term_y2 -= qB(1);
        
                ego_x[0] -= qB(0);
                ego_x[1] -= qB(1);
            }

            // point 1 is right from robot POV
            // point 2 is left from robot POV
            float x_left, y_left, x_right, y_right;
            x_left = x2*coefs;
            y_left = y2*coefs;
            x_right = x1*coefs;
            y_right = y1*coefs;
            float term_x_left, term_y_left, term_x_right, term_y_right;
            term_x_left = term_x2*coefs;
            term_y_left = term_y2*coefs;
            term_x_right = term_x1*coefs;
            term_y_right = term_y1*coefs;

            double initial_goal_x, initial_goal_y, terminal_goal_x, terminal_goal_y;
            initial_goal_x = selectedGap.goal.x * coefs;
            initial_goal_y = selectedGap.goal.y * coefs;
            terminal_goal_x = selectedGap.terminal_goal.x * coefs;
            terminal_goal_y = selectedGap.terminal_goal.y * coefs;

            double goal_vel_x = (terminal_goal_x - initial_goal_x) / selectedGap.gap_lifespan; // absolute velocity (not relative to robot)
            double goal_vel_y = (terminal_goal_y - initial_goal_y) / selectedGap.gap_lifespan;

            ROS_INFO_STREAM("actual initial robot pos: (" << ego_x[0] << ", " << ego_x[1] << ")");
            ROS_INFO_STREAM("actual inital robot velocity: " << ego_x[2] << ", " << ego_x[3] << ")");
            ROS_INFO_STREAM("actual initial left point: (" << x_left << ", " << y_left << "), actual initial right point: (" << x_right << ", " << y_right << ")"); 
            ROS_INFO_STREAM("actual terminal left point: (" << term_x_left << ", " << term_y_left << "), actual terminal right point: (" << term_x_right << ", " << term_y_right << ")");
            ROS_INFO_STREAM("actual initial goal: (" << initial_goal_x << ", " << initial_goal_y << ")"); 
            ROS_INFO_STREAM("actual terminal goal: (" << terminal_goal_x << ", " << terminal_goal_y << ")"); 

            double left_vel_x = ((term_x_left - x_left) / selectedGap.gap_lifespan);
            double left_vel_y = ((term_y_left - y_left) / selectedGap.gap_lifespan);

            double right_vel_x = ((term_x_right - x_right) / selectedGap.gap_lifespan);
            double right_vel_y = ((term_y_right - y_right) / selectedGap.gap_lifespan);

            Eigen::Vector4d manip_left_cart_state(x_left - ego_x[0], 
                                                y_left - ego_x[1],
                                                left_vel_x - curr_vel.linear.x,
                                                left_vel_y - curr_vel.linear.y);
            Eigen::Vector4d manip_right_cart_state(x_right - ego_x[0],
                                                y_right - ego_x[1],
                                                right_vel_x - curr_vel.linear.x,
                                                right_vel_y - curr_vel.linear.y);
            //std::cout << "manipulated left cartesian model: " << manip_left_cart_state[0] << ", " << manip_left_cart_state[1] << ", " << manip_left_cart_state[2] << ", " << manip_left_cart_state[3] << std::endl;
            //std::cout << "manipulated right cartesian model: " << manip_right_cart_state[0] << ", " << manip_right_cart_state[1] << ", " << manip_right_cart_state[2] << ", " << manip_right_cart_state[3] << std::endl;

            Matrix<double, 5, 1> manip_left_polar_state = cartesian_to_polar(manip_left_cart_state);
            Matrix<double, 5, 1> manip_right_polar_state = cartesian_to_polar(manip_right_cart_state);

            //std::cout << "manipulated left polar model: " << manip_left_polar_state[0] << ", " << std::atan2(manip_left_polar_state[1], manip_left_polar_state[2]) << ", " << manip_left_polar_state[3] << ", " << manip_left_polar_state[4] << std::endl;
            //std::cout << "manipulated right polar model: " << manip_right_polar_state[0] << ", " << std::atan2(manip_right_polar_state[1], manip_right_polar_state[2]) << ", " << manip_right_polar_state[3] << ", " << manip_right_polar_state[4] << std::endl;

            state_type x = {ego_x[0], 
                            ego_x[1],
                            ego_x[2],
                            ego_x[3],
                            x_left,
                            y_left,
                            left_vel_x,
                            left_vel_y,
                            x_right,
                            y_right,
                            right_vel_x,
                            right_vel_y,
                            initial_goal_x,
                            initial_goal_y,
                            goal_vel_x,
                            goal_vel_y};
            
            // or if model is invalid?
            //bool invalid_models = left_model_state[0] < 0.01 || right_model_state[0] < 0.01;
            if (selectedGap.goal.discard || selectedGap.terminal_goal.discard) {
                ROS_INFO_STREAM("discarding gap");
                std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
                return return_tuple;
            }
            
            double a_lin_max = 1.5;      
            /*
            polar_gap_field polar_gap_field_inte(x_right, x_left, y_right, y_left,
                                initial_goal_x, initial_goal_y,
                                selectedGap.mode.agc, selectedGap.pivoted_left, selectedGap.isAxial(),
                                cfg_->gap_manip.sigma, x[0], x[1], cfg_->gap_manip.K_acc,  
                                cfg_->control.vx_absmax, a_lin_max);
            */

                        
            Eigen::Vector2d init_rbt_pos(x[0], x[1]);
            Eigen::Vector2d left_pt_0(x_left, y_left);
            Eigen::Vector2d left_pt_1(term_x_left, term_y_left);
            Eigen::Vector2d right_pt_0(x_right, y_right);
            Eigen::Vector2d right_pt_1(term_x_right, term_y_right);
            Eigen::Vector2d nonrel_left_vel(left_vel_x, left_vel_y);
            Eigen::Vector2d nonrel_right_vel(right_vel_x, right_vel_y);
            Eigen::Vector2d rel_left_vel(manip_left_cart_state[2], manip_left_cart_state[3]);
            Eigen::Vector2d rel_right_vel(manip_right_cart_state[2], manip_right_cart_state[3]);

            Eigen::Vector2d nom_vel(cfg_->control.vx_absmax, cfg_->control.vy_absmax);
            Eigen::Vector2d goal_pt_1(terminal_goal_x, terminal_goal_y);
            Eigen::Vector2d gap_origin(0.0, 0.0);
            reachable_gap_APF reachable_gap_APF_inte(init_rbt_pos, gap_origin, left_pt_0, left_pt_1,
                                                    right_pt_0, right_pt_1,
                                                    nonrel_left_vel, nonrel_right_vel,
                                                    nom_vel, goal_pt_1,
                                                    cfg_->gap_manip.sigma, cfg_->gap_manip.K_acc,
                                                    cfg_->control.vx_absmax, a_lin_max);   
            

            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
                                                    reachable_gap_APF_inte, x, 0.0, selectedGap.gap_lifespan, 
                                                    cfg_->traj.integrate_stept, corder);

            if (selectedGap.mode.convex) {
                for (auto & p : posearr.poses) {
                    p.position.x += qB(0);
                    p.position.y += qB(1);
                }
            }

            std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
            ROS_INFO_STREAM("generateTrajectory time elapsed: " << ros::Time::now().toSec() - start_time);
            return return_tuple;
            
        } catch (...) {
            ROS_FATAL_STREAM("integrator");
        }

    }

    Matrix<double, 5, 1> GapTrajGenerator::cartesian_to_polar(Eigen::Vector4d x) {
        Matrix<double, 5, 1> polar_y;
        polar_y << 0.0, 0.0, 0.0, 0.0, 0.0;
        polar_y(0) = 1.0 / std::sqrt(pow(x[0], 2) + pow(x[1], 2));
        double beta = std::atan2(x[1], x[0]);
        polar_y(1) = std::sin(beta);
        polar_y(2) = std::cos(beta);
        polar_y(3) = (x[0]*x[2] + x[1]*x[3]) / (pow(x[0],2) + pow(x[1], 2)); // rdot/r
        polar_y(4) = (x[0]*x[3] - x[1]*x[2]) / (pow(x[0],2) + pow(x[1], 2)); // betadot
        return polar_y;
    }
        

    // If i try to delete this DGap breaks
    [[deprecated("Use single trajectory generation")]]
    std::vector<geometry_msgs::PoseArray> GapTrajGenerator::generateTrajectory(std::vector<dynamic_gap::Gap> gapset) {
        std::vector<geometry_msgs::PoseArray> traj_set(gapset.size());
        return traj_set;
    }

    // Return in Odom frame (used for ctrl)
    geometry_msgs::PoseArray GapTrajGenerator::transformBackTrajectory(
        geometry_msgs::PoseArray posearr,
        geometry_msgs::TransformStamped planning2odom)
    {
        geometry_msgs::PoseArray retarr;
        geometry_msgs::PoseStamped outplaceholder;
        outplaceholder.header.frame_id = cfg_->odom_frame_id;
        geometry_msgs::PoseStamped inplaceholder;
        inplaceholder.header.frame_id = cfg_->robot_frame_id;
        for (const auto pose : posearr.poses)
        {
            inplaceholder.pose = pose;
            tf2::doTransform(inplaceholder, outplaceholder, planning2odom);
            retarr.poses.push_back(outplaceholder.pose);
        }
        retarr.header.frame_id = cfg_->odom_frame_id;
        retarr.header.stamp = ros::Time::now();
        // ROS_WARN_STREAM("leaving transform back with length: " << retarr.poses.size());
        return retarr;
    }

    std::tuple<geometry_msgs::PoseArray, std::vector<double>> GapTrajGenerator::forwardPassTrajectory(std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple)
    {
        geometry_msgs::PoseArray pose_arr = std::get<0>(return_tuple);
        std::vector<double> time_arr = std::get<1>(return_tuple);
        Eigen::Quaternionf q;
        geometry_msgs::Pose old_pose;
        old_pose.position.x = 0;
        old_pose.position.y = 0;
        old_pose.position.z = 0;
        old_pose.orientation.x = 0;
        old_pose.orientation.y = 0;
        old_pose.orientation.z = 0;
        old_pose.orientation.w = 1;
        geometry_msgs::Pose new_pose;
        double dx, dy, result;
        // std::cout << "entering at : " << pose_arr.poses.size() << std::endl;
        //std::cout << "starting pose: " << posearr.poses[0].position.x << ", " << posearr.poses[0].position.y << std::endl; 
        //std::cout << "final pose: " << posearr.poses[posearr.poses.size() - 1].position.x << ", " << posearr.poses[posearr.poses.size() - 1].position.y << std::endl;
        /*
        if (pose_arr.poses.size() > 1) {
            double total_dx = pose_arr.poses[0].position.x - pose_arr.poses[pose_arr.poses.size() - 1].position.x;
            double total_dy = pose_arr.poses[0].position.y - pose_arr.poses[pose_arr.poses.size() - 1].position.y;
            double total_dist = sqrt(pow(total_dx, 2) + pow(total_dy, 2));
            ROS_WARN_STREAM("total distance: " << total_dist);
        }
        */
        std::vector<geometry_msgs::Pose> shortened;
        std::vector<double> shortened_time_arr;
        shortened.push_back(old_pose);
        shortened_time_arr.push_back(0.0);
        double threshold = 0.1;
        // ROS_INFO_STREAM("pose[0]: " << pose_arr.poses[0].position.x << ", " << pose_arr.poses[0].position.y);
        for (int i = 1; i < pose_arr.poses.size(); i++) {
            auto pose = pose_arr.poses[i];
            dx = pose.position.x - shortened.back().position.x;
            dy = pose.position.y - shortened.back().position.y;
            result = sqrt(pow(dx, 2) + pow(dy, 2));
            if (result > threshold) {
                // ROS_INFO_STREAM("result " << i << " kept at " << result);
                shortened.push_back(pose);
                shortened_time_arr.push_back(time_arr[i]);

            } else {
                // ROS_INFO_STREAM("result " << i << " cut at " << result);
            }
        }
        // std::cout << "leaving at : " << shortened.size() << std::endl;
        pose_arr.poses = shortened;

        // Fix rotation
        for (int idx = 1; idx < pose_arr.poses.size(); idx++)
        {
            new_pose = pose_arr.poses[idx];
            old_pose = pose_arr.poses[idx - 1];
            dx = new_pose.position.x - old_pose.position.x;
            dy = new_pose.position.y - old_pose.position.y;
            result = std::atan2(dy, dx);
            q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(result, Eigen::Vector3f::UnitZ());
            q.normalize();
            pose_arr.poses[idx - 1].orientation.x = q.x();
            pose_arr.poses[idx - 1].orientation.y = q.y();
            pose_arr.poses[idx - 1].orientation.z = q.z();
            pose_arr.poses[idx - 1].orientation.w = q.w();
        }
        pose_arr.poses.pop_back();
        shortened_time_arr.pop_back();

        std::tuple<geometry_msgs::PoseArray, std::vector<double>> shortened_tuple(pose_arr, shortened_time_arr);
        return shortened_tuple;
    }

}