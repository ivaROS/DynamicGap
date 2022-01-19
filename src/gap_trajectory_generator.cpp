#include <dynamic_gap/gap_trajectory_generator.h>

namespace dynamic_gap{
    geometry_msgs::PoseArray GapTrajGenerator::generateTrajectory(dynamic_gap::Gap selectedGap, geometry_msgs::PoseStamped curr_pose, geometry_msgs::Twist curr_vel) {
        // return geometry_msgs::PoseArray();
        geometry_msgs::PoseArray posearr;
        posearr.header.stamp = ros::Time::now();
        
        double coefs = cfg_->traj.scale;
        write_trajectory corder(posearr, cfg_->robot_frame_id, coefs);
        posearr.header.frame_id = cfg_->traj.synthesized_frame ? cfg_->sensor_frame_id : cfg_->robot_frame_id;

        // flipping it here because the values appear to be wrong in the integration
        Matrix<double, 5, 1> left_model_state;
        Matrix<double, 5, 1> right_model_state;
        Matrix<double, 5, 1> model_one = selectedGap.left_model->get_state();
        //std::cout << "nominal left model: " << model_one[0] << ", " << model_one[1] << ", " << model_one[2] << std::endl;
        Matrix<double, 5, 1> model_two = selectedGap.right_model->get_state();
        //std::cout << "nominal right model: " << model_two[0] << ", " << model_two[1] << ", " << model_two[2] << std::endl;
        
        // are these the correct angles to check?
        double angle_one = atan2(model_one[1], model_one[2]);
        double angle_two = atan2(model_two[1], model_two[2]);
        double angle_goal = atan2(-selectedGap.goal.x*coefs, selectedGap.goal.y*coefs);
        // if all three are positive or negative
        //std::cout << "angle_one: " << angle_one << ", angle_two: " << angle_two << ", angle_goal: " << angle_goal << std::endl;
        if ((angle_one > 0 && angle_two > 0 && angle_goal > 0) ||
            (angle_one < 0 && angle_two < 0 && angle_goal < 0)) {
            if (angle_one > angle_two) {
                left_model_state = model_one;
                right_model_state = model_two;
            } else {
                left_model_state = model_two;
                right_model_state = model_one;
            }
        } else if ((angle_one > 0 && angle_two > 0) ||
                   (angle_one < 0 && angle_two < 0)) {
            if (angle_one > angle_two) {
                right_model_state = model_one;
                left_model_state = model_two;
            } else {
                right_model_state = model_two;
                left_model_state = model_one;
            }
        } else if (angle_one > 0 && angle_goal > 0 && angle_two < 0) {
            if (angle_goal < angle_one) {
                left_model_state = model_one;
                right_model_state = model_two;
            } else {
                left_model_state = model_two;
                right_model_state = model_one;
            }
        } else if (angle_two > 0 && angle_goal > 0 && angle_one < 0) {
            if (angle_goal < angle_two) {
                left_model_state = model_two;
                right_model_state = model_one;
            } else {
                left_model_state = model_one;
                right_model_state = model_two;
            }
        } else if (angle_one < 0 && angle_goal < 0 && angle_two > 0) {
            if (angle_goal > angle_one) {
                right_model_state = model_one;
                left_model_state = model_two;
            } else {
                left_model_state = model_one;
                right_model_state = model_two;
            }
        } else if (angle_two < 0 && angle_goal < 0 && angle_one > 0) {
            if (angle_goal > angle_two) {
                right_model_state = model_two;
                left_model_state = model_one;
            } else {
                left_model_state = model_two;
                right_model_state = model_one;
            }
        }

        
        // curr_pose is in sensor frame
        // gaps are in robot frame?
        // curr_vel is in robot frame
        state_type x = {curr_pose.pose.position.x + 1e-5, 
                        curr_pose.pose.position.y + 1e-6,
                        curr_vel.linear.x,
                        curr_vel.linear.y,
                        left_model_state[0],
                        left_model_state[1],
                        left_model_state[2],
                        left_model_state[3],
                        left_model_state[4],
                        right_model_state[0],
                        right_model_state[1],
                        right_model_state[2],
                        right_model_state[3],
                        right_model_state[4]}; 
        
        // get gap points in cartesian
        float x1, x2, y1, y2;
        float half_num_scan = selectedGap.half_scan;
        x1 = (selectedGap.convex.convex_ldist) * cos(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (selectedGap.convex.convex_ldist) * sin(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
        x2 = (selectedGap.convex.convex_rdist) * cos(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (selectedGap.convex.convex_rdist) * sin(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);

        std::cout << "starting trajectory generation" << std::endl;
        std::cout << "x1, y1: (" << x1 << ", " << y1 << "), x2,y2: (" << x2 << ", " << y2 << ")" << std::endl; 
        std::cout << "local goal: " << selectedGap.goal.x << ", " << selectedGap.goal.y << std::endl;
        
        /*
        if (model_one[0] == left_model_state[0]) {
            std::cout << "no flip " << std::endl;
        } else {
            std::cout << "flipped" << std::endl;
        }
        */
        if (selectedGap.goal.discard) {
            //std::cout << "discarding gap" << std::endl;
            return posearr;
        }

        double gap_angle = (selectedGap._right_idx - selectedGap._left_idx) * M_PI / selectedGap.half_scan;

        /*
        if (selectedGap.goal.goalwithin) {
            
            std::cout << "starting go to goal" << std::endl;
            //ROS_WARN_STREAM("Goal to Goal");
            // std::cout << "gap angle: " << gap_angle << std::endl;
            std::cout << "p1: " << x1*coefs << ", " << y1*coefs << " p2: " << x2*coefs << ", " << y2*coefs << std::endl;
            std::cout << "local goal: " << selectedGap.goal.x*coefs << ", " << selectedGap.goal.y*coefs << std::endl;
            std::cout << "initial x: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << std::endl;
            std::cout << "p1/goal dot: " << selectedGap.goal.x*coefs*x1*coefs + selectedGap.goal.y*coefs*y1*coefs << std::endl;
            std::cout << "p2/goal dot: " << selectedGap.goal.x*coefs*x2*coefs + selectedGap.goal.y*coefs*y2*coefs << std::endl;
            //std::cout << "left model: " << x[4] << ", " << x[5] << ", " << x[6] << ", " << x[7] << ", " << x[8] << std::endl;
            //std::cout << "right model: " << x[9] << ", " << x[10] << ", " << x[11] << ", " << x[12] << ", " << x[13] << std::endl;
            //std::cout << "left cbf: " << x[8] / x[4] << ", right cbf: " << x[13] / x[9] << std::endl;
            
            g2g inte_g2g(
                selectedGap.goal.x * coefs,
                selectedGap.goal.y * coefs);
            // here, inte_g2g are the dynamics,  corder is observer?
            // system, state, start_time, end_time, dt, observer
            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
            inte_g2g, x, 0.0,
            cfg_->traj.integrate_maxt,
            cfg_->traj.integrate_stept,
            corder);
            return posearr;
        }
        */

        if (selectedGap.mode.convex) {
            // std::cout << "convex, subtracting: " << - selectedGap.qB(0) << ", " << - selectedGap.qB(1) << std::endl;
            x = {- selectedGap.qB(0) - 1e-6, 
                 - selectedGap.qB(1) + 1e-6,
                 curr_vel.linear.x,
                 curr_vel.linear.y,
                 left_model_state[0],
                 left_model_state[1],
                 left_model_state[2],
                 left_model_state[3],
                 left_model_state[4],
                 right_model_state[0],
                 right_model_state[1],
                 right_model_state[2],
                 right_model_state[3],
                 right_model_state[4]};
            x1 -= selectedGap.qB(0);
            x2 -= selectedGap.qB(0);
            y1 -= selectedGap.qB(1);
            y2 -= selectedGap.qB(1);
            selectedGap.goal.x -= selectedGap.qB(0);
            selectedGap.goal.y -= selectedGap.qB(1);

        }
        
        // std::cout << "<<<<<<<<<<<<<<<<<<<<starting polar field>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
        // std::cout << "coefs: " << coefs << std::endl;
        // std::cout << "local goal: " << selectedGap.goal.x*coefs << ", " << selectedGap.goal.y*coefs << std::endl;
        //std::cout << "initial x: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << std::endl;
        //std::cout << "left gap point: " << x1*coefs << ", " << y1*coefs << " right gap point: " << x2*coefs << ", " << y2*coefs << std::endl;
        //std::cout << "p1/goal dot: " << selectedGap.goal.x*coefs*x1*coefs + selectedGap.goal.y*coefs*y1*coefs << std::endl;
        //std::cout << "p2/goal dot: " << selectedGap.goal.x*coefs*x2*coefs + selectedGap.goal.y*coefs*y2*coefs << std::endl;
        /*
        polar_gap_field inte(x1 * coefs, x2 * coefs,
                            y1 * coefs, y2 * coefs,
                            selectedGap.goal.x * coefs,
                            selectedGap.goal.y * coefs,
                            selectedGap.getLeftObs(),
                            selectedGap.getRightObs(),
                            selectedGap.isAxial(),
                            cfg_->gap_manip.sigma);
        */
        
        double orig_beta_left = atan2(left_model_state(1), left_model_state(2));
        double orig_beta_right = atan2(right_model_state(1), right_model_state(2));
        
        clf_cbf clf_cbf_dyn(x1 * coefs, x2 * coefs,
                            y1 * coefs, y2 * coefs,
                            selectedGap.isAxial(),
                            selectedGap.goal.x * coefs,
                            selectedGap.goal.y * coefs,
                            cfg_->gap_manip.K_des,
                            cfg_->gap_manip.cbf_param,
                            cfg_->gap_manip.cbf_r_min,
                            cfg_->gap_manip.K_acc,
                            orig_beta_left,
                            orig_beta_right);
        
        // std::cout << "p1: " << x1*coefs << ", " << y1*coefs << " p2: " << x2*coefs << ", " << y2*coefs << std::endl;
        //std::cout << "starting left model state: " << left_model_state[0] << ", " <<  left_model_state[1] << ", " <<  left_model_state[2] << ", " <<  left_model_state[3] << ", " <<  left_model_state[4] << std::endl;
        //std::cout << "starting right model state: " << right_model_state[0] << ", " <<  right_model_state[1] << ", " <<  right_model_state[2] << ", " <<  right_model_state[3] << ", " <<  right_model_state[4] << std::endl;
        
        //std::cout << "starting clf cbf" << std::endl;
        //std::cout << "local goal: " << selectedGap.goal.x*coefs << ", " << selectedGap.goal.y*coefs << std::endl;
        //std::cout << "initial x: " << x[0] << ", " << x[1] << ", " << x[2] << ", " << x[3] << std::endl;
        //std::cout << "left model: " << x[4] << ", " << x[5] << ", " << x[6] << ", " << x[7] << ", " << x[8] << std::endl;
        //std::cout << "right model: " << x[9] << ", " << x[10] << ", " << x[11] << ", " << x[12] << ", " << x[13] << std::endl;
        //std::cout << "left cbf: " << x[8] / x[4] << ", right cbf: " << x[13] / x[9] << std::endl;
        
        boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
            clf_cbf_dyn, x, 0.0,
            cfg_->traj.integrate_maxt,
            cfg_->traj.integrate_stept, corder);

        //std::cout << "starting pose: " << posearr.poses[0].position.x << ", " << posearr.poses[0].position.y << std::endl; 
        //std::cout << "final pose: " << posearr.poses[posearr.poses.size() - 1].position.x << ", " << posearr.poses[posearr.poses.size() - 1].position.y << std::endl;
        //ROS_WARN_STREAM("CLF CBF");
        //ROS_WARN_STREAM("start: " << posearr.poses[0].position.x << ", " << posearr.poses[0].position.y << ", goal " << selectedGap.goal.x*coefs << ", " << selectedGap.goal.y*coefs << ", finish " << posearr.poses[posearr.poses.size() - 1].position.x << ", " << posearr.poses[posearr.poses.size() - 1].position.y << ", length: " << posearr.poses.size());
        if (selectedGap.mode.convex) {
            for (auto & p : posearr.poses) {
                p.position.x += selectedGap.qB(0);
                p.position.y += selectedGap.qB(1);
            }
        }

        return posearr;
    }

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

    geometry_msgs::PoseArray GapTrajGenerator::forwardPassTrajectory(geometry_msgs::PoseArray pose_arr)
    {
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
        // ROS_WARN_STREAM("entering at : " << pose_arr.poses.size());
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
        shortened.push_back(old_pose);
        for (auto pose : pose_arr.poses)
        {
            dx = pose.position.x - shortened.back().position.x;
            dy = pose.position.y - shortened.back().position.y;
            result = sqrt(pow(dx, 2) + pow(dy, 2));
            if (result > 0.05) {
                //ROS_WARN_STREAM("result kept at " << result);
                shortened.push_back(pose);
            } else {
                //ROS_WARN_STREAM("result cut at " << result);
            }
        }
        // ROS_WARN_STREAM("leaving at : " << shortened.size());
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

        return pose_arr;
    }

}