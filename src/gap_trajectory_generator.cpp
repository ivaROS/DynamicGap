#include <dynamic_gap/gap_trajectory_generator.h>

namespace dynamic_gap{
    std::tuple<geometry_msgs::PoseArray, std::vector<double>> GapTrajGenerator::generateTrajectory(dynamic_gap::Gap& selectedGap, geometry_msgs::PoseStamped curr_pose, geometry_msgs::Twist curr_vel) {
        // return geometry_msgs::PoseArray();
        geometry_msgs::PoseArray posearr;
        std::vector<double> timearr;
        posearr.header.stamp = ros::Time::now();
        
        double coefs = cfg_->traj.scale;
        write_trajectory corder(posearr, cfg_->robot_frame_id, coefs, timearr);
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
        double angle_goal = atan2(selectedGap.goal.y*coefs, selectedGap.goal.x*coefs);
        // std::cout << "picking gap sides" << std::endl;
        // seems to only do 2,7,9
        determineLeftRightModels(left_model_state, right_model_state, selectedGap, angle_goal);

        double beta_local_goal; 
        beta_local_goal = atan2(selectedGap.goal.y*coefs, selectedGap.goal.x*coefs);
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
                        right_model_state[4],
                        sin(beta_local_goal),
                        cos(beta_local_goal)}; 
        
        auto left_ori = std::atan2(left_model_state[1], left_model_state[2]);
        auto right_ori = std::atan2(right_model_state[1], right_model_state[2]);
        // std::cout << "left ori: " << left_ori << "< right ori: " << right_ori << std::endl;
        
        // if expanding: just do CLF

        // get gap points in cartesian
        float x1, x2, y1, y2;
        float half_num_scan = selectedGap.half_scan;
        x1 = (selectedGap.convex.convex_ldist) * cos(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (selectedGap.convex.convex_ldist) * sin(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
        x2 = (selectedGap.convex.convex_rdist) * cos(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (selectedGap.convex.convex_rdist) * sin(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);

        //std::cout << "original starting goal: (" << selectedGap.goal.x << ", " << selectedGap.goal.y << ")" << std::endl; 
        //std::cout << "original points x1, y1: (" << x1 << ", " << y1 << "), x2,y2: (" << x2 << ", " << y2 << ")" << std::endl; 
        //std::cout << "starting left model state: " << left_model_state[0] << ", " <<  left_model_state[1] << ", " <<  left_model_state[2] << ", " <<  left_model_state[3] << ", " <<  left_model_state[4] << std::endl;
        //std::cout << "starting right model state: " << right_model_state[0] << ", " <<  right_model_state[1] << ", " <<  right_model_state[2] << ", " <<  right_model_state[3] << ", " <<  right_model_state[4] << std::endl;
        
        if (left_model_state[4] > 0 && right_model_state[4] < 0) {
            std::cout << "go to goal trajectory generated" << std::endl;
            std::cout << "start: " << x[0] << ", " << x[1] << ", goal: " << selectedGap.goal.x * coefs << ", " << selectedGap.goal.y * coefs << std::endl;
            g2g inte_g2g(
                selectedGap.goal.x * coefs,
                selectedGap.goal.y * coefs,
                cfg_->gap_manip.K_des,
                cfg_->gap_manip.K_acc);
            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
            inte_g2g, x, 0.0,
            selectedGap.gap_lifespan,
            cfg_->traj.integrate_stept,
            corder);
            std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
            return return_tuple;
        }
        
        if (selectedGap.mode.convex) {
            // std::cout << "convex" << std::endl;
            selectedGap.goal.x -= selectedGap.qB(0);
            selectedGap.goal.y -= selectedGap.qB(1);
            beta_local_goal = atan2(selectedGap.goal.y, selectedGap.goal.x);
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
                 right_model_state[4],
                 sin(beta_local_goal),
                 cos(beta_local_goal)};
            x1 -= selectedGap.qB(0);
            x2 -= selectedGap.qB(0);
            y1 -= selectedGap.qB(1);
            y2 -= selectedGap.qB(1);
            //std::cout << "x1, y1 actually: (" << x1 << ", " << y1 << "), x2,y2: (" << x2 << ", " << y2 << ")" << std::endl; 
        }


        double local_goal_dist = std::sqrt(pow(selectedGap.goal.x*coefs, 2) + pow(selectedGap.goal.y*coefs, 2));
        //std::cout << "rbt start: " << x[0] << ", " << x[1] << std::endl;
        //std::cout << "starting goal: (" << selectedGap.goal.x * coefs << ", " << selectedGap.goal.y * coefs << ")" << std::endl; 
        //std::cout << "p1: " << x1*coefs << ", " << y1*coefs << " p2: " << x2*coefs << ", " << y2*coefs << std::endl;
            
        // TODO: make sure that this is always less than 180, make sure convex (BROKEN)
        double gap_angle;
        if (right_ori > left_ori) {
            gap_angle = right_ori - left_ori;
        } else {
            gap_angle = left_ori - right_ori;
        }        
        // std::cout << "gap angle: " << gap_angle << std::endl;
        
        //std::cout << "gap is either static or closing, CLF/CBF trajectory generated" << std::endl;
        clf_cbf clf_cbf_dyn(selectedGap.isAxial(),
                            cfg_->gap_manip.K_des,
                            cfg_->gap_manip.cbf_param,
                            cfg_->gap_manip.K_acc,
                            local_goal_dist,
                            x[8],
                            x[13],
                            cfg_->control.vx_absmax,
                            cfg_->control.vy_absmax,
                            x[0],
                            x[1]);
        
        // or if model is invalid?
        //bool invalid_models = left_model_state[0] < 0.01 || right_model_state[0] < 0.01;
        if (selectedGap.goal.discard) {
            std::cout << "discarding gap" << std::endl;
            std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
            return return_tuple;
        }
        
        std::cout << "CLF/CBF trajectory generated" << std::endl;
        std::cout << "start: " << x[0] << ", " << x[1] << ", goal: " << local_goal_dist*x[15] << ", " << local_goal_dist*x[14] << std::endl;
            
        // boost::numeric::odeint::max_step_checker m_checker = boost::numeric::odeint::max_step_checker(10);

        boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
            clf_cbf_dyn, x, 0.0,
            selectedGap.gap_lifespan,
            cfg_->traj.integrate_stept, corder);

        //ROS_WARN_STREAM("CLF CBF");
        //ROS_WARN_STREAM("start: " << posearr.poses[0].position.x << ", " << posearr.poses[0].position.y << ", goal " << selectedGap.goal.x*coefs << ", " << selectedGap.goal.y*coefs << ", finish " << posearr.poses[posearr.poses.size() - 1].position.x << ", " << posearr.poses[posearr.poses.size() - 1].position.y << ", length: " << posearr.poses.size());
        if (selectedGap.mode.convex) {
            for (auto & p : posearr.poses) {
                p.position.x += selectedGap.qB(0);
                p.position.y += selectedGap.qB(1);
            }
        }
        //std::cout << "starting pose: " << posearr.poses[0].position.x << ", " << posearr.poses[0].position.y << std::endl; 
        //std::cout << "final pose: " << posearr.poses[posearr.poses.size() - 1].position.x << ", " << posearr.poses[posearr.poses.size() - 1].position.y << std::endl;
        
        std::tuple<geometry_msgs::PoseArray, std::vector<double>> return_tuple(posearr, timearr);
        return return_tuple;
    }

    [[deprecated("Use single trajectory generation")]]
    std::vector<geometry_msgs::PoseArray> GapTrajGenerator::generateTrajectory(std::vector<dynamic_gap::Gap> gapset) {
        std::vector<geometry_msgs::PoseArray> traj_set(gapset.size());
        return traj_set;
    }


    void GapTrajGenerator::determineLeftRightModels(Matrix<double, 5, 1>& left_model_state, Matrix<double, 5, 1>& right_model_state, dynamic_gap::Gap& selectedGap, double angle_goal) {
        Matrix<double, 5, 1> model_one = selectedGap.left_model->get_state();
        //std::cout << "nominal left model: " << model_one[0] << ", " << model_one[1] << ", " << model_one[2] << std::endl;
        Matrix<double, 5, 1> model_two = selectedGap.right_model->get_state();
        //std::cout << "nominal right model: " << model_two[0] << ", " << model_two[1] << ", " << model_two[2] << std::endl;
        
        // are these the correct angles to check?
        double angle_one = atan2(model_one[1], model_one[2]);
        double angle_two = atan2(model_two[1], model_two[2]);
        // if all three are positive or negative
        //std::cout << "angle_one: " << angle_one << ", angle_two: " << angle_two << ", angle_goal: " << angle_goal << std::endl;
        //std::cout << "picking gap sides" << std::endl;
        // seems to only do 2,7,9
        if ((angle_one > 0 && angle_two > 0 && angle_goal > 0) ||
            (angle_one < 0 && angle_two < 0 && angle_goal < 0)) {
            if (angle_one > angle_two) {
                //std::cout << "1" << std::endl;
                left_model_state = model_one;
                right_model_state = model_two;
            } else {
                //std::cout << "2" << std::endl;
                left_model_state = model_two;
                right_model_state = model_one;
            }
        } else if ((angle_one > 0 && angle_two > 0) || (angle_one < 0 && angle_two < 0)) {
            if (angle_one > angle_two) {
                //std::cout << "3" << std::endl;
                right_model_state = model_one;
                left_model_state = model_two;
            } else {
                //std::cout << "4" << std::endl;
                right_model_state = model_two;
                left_model_state = model_one;
            }
        } else if (angle_one > 0 && angle_goal > 0 && angle_two < 0) {
            if (angle_goal < angle_one) {
                //std::cout << "5" << std::endl;
                left_model_state = model_one;
                right_model_state = model_two;
            } else {
                //std::cout << "6" << std::endl;
                left_model_state = model_two;
                right_model_state = model_one;
            }
        } else if (angle_two > 0 && angle_goal > 0 && angle_one < 0) {
            if (angle_goal < angle_two) {
                //std::cout << "7" << std::endl;
                left_model_state = model_two;
                right_model_state = model_one;
            } else {
                //std::cout << "8" << std::endl;
                left_model_state = model_one;
                right_model_state = model_two;
            }
        } else if (angle_one < 0 && angle_goal < 0 && angle_two > 0) {
            if (angle_goal > angle_one) {
                //std::cout << "9" << std::endl;
                right_model_state = model_one;
                left_model_state = model_two;
            } else {
                //std::cout << "10" << std::endl;
                left_model_state = model_one;
                right_model_state = model_two;
            }
        } else if (angle_two < 0 && angle_goal < 0 && angle_one > 0) {
            if (angle_goal > angle_two) {
                //std::cout << "11" << std::endl;
                right_model_state = model_two;
                left_model_state = model_one;
            } else {
                //std::cout << "12" << std::endl;
                left_model_state = model_two;
                right_model_state = model_one;
            }
        }
        return;
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
        for (int i = 1; i < pose_arr.poses.size(); i++) {
            auto pose = pose_arr.poses[i];
            dx = pose.position.x - shortened.back().position.x;
            dy = pose.position.y - shortened.back().position.y;
            result = sqrt(pow(dx, 2) + pow(dy, 2));
            if (result > 0.05) {
                //ROS_WARN_STREAM("result kept at " << result);
                // std::cout << "keeping: " << pose.position.x << ", " << pose.position.y << std::endl;
                shortened.push_back(pose);
                shortened_time_arr.push_back(time_arr[i]);

            } else {
                //ROS_WARN_STREAM("result cut at " << result);
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