
#include <dynamic_gap/gap_feasibility.h>

namespace dynamic_gap {

    void GapFeasibilityChecker::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egolock);
        msg = msg_;
        num_of_scan = (int)(msg.get()->ranges.size());
    }

    bool GapFeasibilityChecker::indivGapFeasibilityCheck(dynamic_gap::Gap& gap) {
        dynamic_gap::cart_model* left_model = gap.left_model;
        dynamic_gap::cart_model* right_model = gap.right_model;

        left_model->freeze_robot_vel();
        right_model->freeze_robot_vel();

        Matrix<double, 4, 1> frozen_left_model_state = left_model->get_frozen_modified_polar_state();
        Matrix<double, 4, 1> frozen_right_model_state = right_model->get_frozen_modified_polar_state();

        double frozen_left_betadot = frozen_left_model_state[3];
        double frozen_right_betadot = frozen_right_model_state[3];

        double min_betadot = std::min(frozen_left_betadot, frozen_right_betadot);
        double subtracted_left_betadot = frozen_left_betadot - min_betadot;
        double subtracted_right_betadot = frozen_right_betadot - min_betadot;

        // ROS_INFO_STREAM("frozen left betadot: " << frozen_left_betadot);
        // ROS_INFO_STREAM("frozen right betadot: " << frozen_right_betadot);

        // double start_time = ros::Time::now().toSec();
        double crossing_time = gapSplinecheck(gap, left_model, right_model);
        // ROS_INFO_STREAM("gapSplinecheck time elapsed:" << ros::Time::now().toSec() - start_time);


        bool feasible = false;
        if (gap.artificial) {
            feasible = true;
            gap.gap_lifespan = cfg_->traj.integrate_maxt;
            gap.setTerminalPoints(gap.LIdx(), gap.LDist(), gap.RIdx(), gap.RDist());
            gap.setCategory("static");
        } else if (subtracted_left_betadot > 0) {
            // expanding
            feasible = true;
            gap.gap_lifespan = cfg_->traj.integrate_maxt;
            gap.setCategory("expanding");
        } else if (subtracted_left_betadot == 0 && subtracted_right_betadot == 0) {
            // static
            feasible = true;
            gap.gap_lifespan = cfg_->traj.integrate_maxt;
            gap.setCategory("static");
        } else {
            // closing
            gap.setCategory("closing"); 
            if (crossing_time >= 0) {
                feasible = true;
                gap.gap_lifespan = crossing_time;
            }
        }

        ROS_INFO_STREAM("gap is feasible: " << feasible);
        return feasible;
    }
    

    double GapFeasibilityChecker::gapSplinecheck(dynamic_gap::Gap & gap, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model) {

        Eigen::Vector2f crossing_pt(0.0, 0.0);
        double start_time = ros::Time::now().toSec();
        double crossing_time = indivGapFindCrossingPoint(gap, crossing_pt, left_model, right_model);
        // ROS_INFO_STREAM("indivGapFindCrossingPoint time elapsed:" << ros::Time::now().toSec() - start_time);

        Eigen::Vector2f starting_pos(0.0, 0.0);
        Eigen::Vector2f starting_vel(left_model->get_v_ego()[0], left_model->get_v_ego()[1]);
        
        Eigen::Vector2f ending_vel(0.0, 0.0);
        
        if (crossing_pt.norm() > 0) {
            ending_vel << starting_vel.norm() * crossing_pt[0] / crossing_pt.norm(), starting_vel.norm() * crossing_pt[1] / crossing_pt.norm();
        } 
        
        // ROS_INFO_STREAM("starting x: " << starting_pos[0] << ", " << starting_pos[1] << ", " << starting_vel[0] << ", " << starting_vel[1]);
        // ROS_INFO_STREAM("ending x: " << crossing_pt[0] << ", " << crossing_pt[1] << ", ending_vel: " << ending_vel[0] << ", " << ending_vel[1]);
        
        start_time = ros::Time::now().toSec();
        Eigen::MatrixXf A_spline = MatrixXf::Random(4,4);
        Eigen::VectorXf b_spline = VectorXf::Random(4);
        A_spline << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    1.0, crossing_time, pow(crossing_time,2), pow(crossing_time,3),
                    0.0, 1.0, 2*crossing_time, 3*pow(crossing_time,2);
        b_spline << starting_pos[0], starting_vel[0], crossing_pt[0], ending_vel[0];
        gap.spline_x_coefs = A_spline.partialPivLu().solve(b_spline);

        // std::cout << "x coeffs: " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ", " << coeffs[3] << std::endl;
        double peak_velocity_time = crossing_time/2.0;
        double peak_velocity_x = 3*gap.spline_x_coefs[3]*pow(peak_velocity_time, 2) + 
                                 2*gap.spline_x_coefs[2]*peak_velocity_time + 
                                 gap.spline_x_coefs[1];
        
        b_spline << starting_pos[1], starting_vel[1], crossing_pt[1], ending_vel[1];

        gap.spline_y_coefs = A_spline.partialPivLu().solve(b_spline);
        //std::cout << "y coeffs: " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ", " << coeffs[3] << std::endl;
        double peak_velocity_y = 3*gap.spline_y_coefs[3]*pow(peak_velocity_time, 2) + 
                                 2*gap.spline_y_coefs[2]*peak_velocity_time + 
                                 gap.spline_y_coefs[1];
        // ROS_INFO_STREAM("spline build time elapsed:" << ros::Time::now().toSec() - start_time);

        // ROS_INFO_STREAM("peak velocity: " << peak_velocity_x << ", " << peak_velocity_y);
        gap.peak_velocity_x = peak_velocity_x;
        gap.peak_velocity_y = peak_velocity_y;
        
        if (std::max(std::abs(peak_velocity_x), std::abs(peak_velocity_y)) <= cfg_->control.vx_absmax) {
            return crossing_time;
        } else {
            return -1.0;
        }
    }
 
    double GapFeasibilityChecker::indivGapFindCrossingPoint(dynamic_gap::Gap & gap, Eigen::Vector2f& gap_crossing_point, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model) {
        //std::cout << "determining crossing point" << std::endl;
        auto egocircle = *msg.get();

        double x_r, x_l, y_r, y_l;

        double beta_left = double(gap.LIdx() - gap.half_scan) * M_PI / gap.half_scan;
        double beta_right = double(gap.RIdx() - gap.half_scan) * M_PI / gap.half_scan;
        x_l = gap.LDist() * cos(beta_left);
        y_l = gap.LDist() * sin(beta_left);
        x_r = gap.RDist() * cos(beta_right);
        y_r = gap.RDist() * sin(beta_right);
       
        Eigen::Vector2d left_bearing_vect(x_l / gap.LDist(), y_l / gap.LDist());
        Eigen::Vector2d right_bearing_vect(x_r / gap.RDist(), y_r / gap.RDist());

        double L_to_R_angle = getLeftToRightAngle(left_bearing_vect, right_bearing_vect);
        Eigen::Vector2d prev_right_bearing_vect = right_bearing_vect;

        double beta_center = (beta_left - (L_to_R_angle / 2.0));

        double prev_beta_left = beta_left;
        double prev_beta_right = beta_right;
        double prev_L_to_R_angle = L_to_R_angle;
        Eigen::Vector2d prev_left_bearing_vect = left_bearing_vect;
        

        Eigen::Vector2d central_bearing_vect(std::cos(beta_center), std::sin(beta_center));
        
        //std::cout << "initial beta left: (" << left_bearing_vect[0] << ", " << left_bearing_vect[1] << "), initial beta right: (" << right_bearing_vect[0] << ", " << right_bearing_vect[1] << "), initial beta center: (" << central_bearing_vect[0] << ", " << central_bearing_vect[1] << ")" << std::endl;
        
        Matrix<double, 4, 1> left_frozen_mp_state = left_model->get_frozen_modified_polar_state();        
        Matrix<double, 4, 1> right_frozen_mp_state = right_model->get_frozen_modified_polar_state();
        Matrix<double, 4, 1> left_frozen_cartesian_state = left_model->get_frozen_cartesian_state();
        Matrix<double, 4, 1> right_frozen_cartesian_state = right_model->get_frozen_cartesian_state();

        // ROS_INFO_STREAM("gap category: " << gap.getCategory());
        // ROS_INFO_STREAM("starting frozen cartesian left: " << left_frozen_cartesian_state[0] << ", " << left_frozen_cartesian_state[1] << ", " << left_frozen_cartesian_state[2] << ", " << left_frozen_cartesian_state[3]); 
        // ROS_INFO_STREAM("starting frozen cartesian right: " << right_frozen_cartesian_state[0] << ", " << right_frozen_cartesian_state[1] << ", " << right_frozen_cartesian_state[2] << ", " << right_frozen_cartesian_state[3]);

        // double beta_left, beta_right, beta_center;
       
        double left_central_dot, right_central_dot;
        bool first_cross = true;
        bool bearing_crossing_check, range_closing_check;    
        int idx_left, idx_right;
        double min_dist;

        Matrix<double, 4, 1> prev_left_frozen_mp_state = left_frozen_mp_state;        
        Matrix<double, 4, 1> prev_right_frozen_mp_state = right_frozen_mp_state;        
        Matrix<double, 2, 1> prev_central_bearing_vect = central_bearing_vect;

        Eigen::Vector2d left_cross_pt, right_cross_pt, diff_pt;
        for (double t = cfg_->traj.integrate_stept; t < cfg_->traj.integrate_maxt; t += cfg_->traj.integrate_stept) {
            left_model->frozen_state_propagate(cfg_->traj.integrate_stept);
            right_model->frozen_state_propagate(cfg_->traj.integrate_stept);
            // ROS_INFO_STREAM("t: " << t);
            left_frozen_mp_state = left_model->get_frozen_modified_polar_state();
            right_frozen_mp_state = right_model->get_frozen_modified_polar_state();
            
            
            beta_left = left_frozen_mp_state[1];
            beta_right = right_frozen_mp_state[1];
            // idx_left = (beta_left - egocircle.angle_min) / egocircle.angle_increment;
            // idx_right = (beta_right - egocircle.angle_min) / egocircle.angle_increment;
            // ROS_INFO_STREAM("beta_left: " << beta_left << ", beta_right: " << beta_right);
            // ROS_INFO_STREAM("idx_left: " << idx_left << ", idx_right: " << idx_right);
            left_bearing_vect << std::cos(beta_left), std::sin(beta_left);
            right_bearing_vect << std::cos(beta_right), std::sin(beta_right);
            L_to_R_angle = getLeftToRightAngle(left_bearing_vect, right_bearing_vect);
            beta_center = (beta_left - 0.5 * L_to_R_angle);

            prev_beta_left = prev_left_frozen_mp_state[1];
            prev_beta_right = prev_right_frozen_mp_state[1];
            prev_left_bearing_vect << std::cos(prev_beta_left), std::sin(prev_beta_left);
            prev_right_bearing_vect << std::cos(prev_beta_right), std::sin(prev_beta_right);
            prev_L_to_R_angle = getLeftToRightAngle(prev_left_bearing_vect, prev_right_bearing_vect);            

            // ROS_INFO_STREAM("prev_L_to_R_angle: " << prev_L_to_R_angle << ", L_to_R_angle: " << L_to_R_angle);

            central_bearing_vect << std::cos(beta_center), std::sin(beta_center);
        
            left_central_dot = left_bearing_vect.dot(prev_central_bearing_vect);
            right_central_dot = right_bearing_vect.dot(prev_central_bearing_vect);
            bearing_crossing_check = left_central_dot > 0.0 && right_central_dot > 0.0;

            // checking for bearing crossing conditions for closing and crossing gaps
            if (L_to_R_angle > M_PI && bearing_crossing_check) {
                ROS_INFO_STREAM("bearing cross at " << t);
                // CLOSING GAP CHECK
                left_cross_pt << (1.0 / prev_left_frozen_mp_state[0])*std::cos(prev_left_frozen_mp_state[1]), 
                                    (1.0 / prev_left_frozen_mp_state[0])*std::sin(prev_left_frozen_mp_state[1]);
                right_cross_pt << (1.0 / prev_right_frozen_mp_state[0])*std::cos(prev_right_frozen_mp_state[1]), 
                                    (1.0 / prev_right_frozen_mp_state[0])*std::sin(prev_right_frozen_mp_state[1]);
                diff_pt = (left_cross_pt - right_cross_pt);
                // ROS_INFO_STREAM("diff_pt: " << diff_pt << ", diff_pt.norm: " << diff_pt.norm());

                range_closing_check = (left_cross_pt - right_cross_pt).norm()  < 2*cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; 
                // IF POINTS ARE SUFFICIENTLY CLOSE TOGETHER, GAP HAS CLOSED
                if (range_closing_check) {
                    
                    if (left_cross_pt.norm() < right_cross_pt.norm()) {
                        gap_crossing_point << right_cross_pt[0], right_cross_pt[1];
                    } else {
                        gap_crossing_point << left_cross_pt[0], left_cross_pt[1];
                    }

                    gap_crossing_point += 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * (gap_crossing_point / gap_crossing_point.norm());
                    gap.setClosingPoint(gap_crossing_point[0], gap_crossing_point[1]);
                    double ending_time = generateCrossedGapTerminalPoints(t, gap, left_model, right_model);
                    ROS_INFO_STREAM("considering gap closed at " << ending_time); 

                    gap.gap_closed = true;
                    return ending_time;
                } else {
                    if (first_cross) {
                        double mid_x = (left_cross_pt[0] + right_cross_pt[0]) / 2;
                        double mid_y = (left_cross_pt[1] + right_cross_pt[1]) / 2;
                        //  ROS_INFO_STREAM("gap crosses but does not close at " << t << ", left point at: " << left_cross_pt[0] << ", " << left_cross_pt[1] << ", right point at " << right_cross_pt[0] << ", " << right_cross_pt[1]); 

                        gap.setCrossingPoint(mid_x, mid_y);
                        first_cross = false;

                        double ending_time = generateCrossedGapTerminalPoints(t, gap, left_model, right_model);
                        ROS_INFO_STREAM("considering gap crossed at " << ending_time); 

                        gap.gap_crossed = true;
                    }
                }
            } else if (prev_L_to_R_angle > (3*M_PI / 4) && L_to_R_angle < (M_PI / 4)) {
                // checking for corner case of gap crossing behind the robot
                left_cross_pt << (1.0 / prev_left_frozen_mp_state[0])*std::cos(prev_left_frozen_mp_state[1]), 
                                 (1.0 / prev_left_frozen_mp_state[0])*std::sin(prev_left_frozen_mp_state[1]);
                right_cross_pt << (1.0 / prev_right_frozen_mp_state[0])*std::cos(prev_right_frozen_mp_state[1]), 
                                  (1.0 / prev_right_frozen_mp_state[0])*std::sin(prev_right_frozen_mp_state[1]);
                ROS_INFO_STREAM("crossing from behind, terminal points at: (" << left_cross_pt[0] << ", " << left_cross_pt[1] << "), (" << right_cross_pt[0] << ", " << right_cross_pt[1] << ")");
                generateTerminalPoints(gap, prev_left_frozen_mp_state[1], prev_left_frozen_mp_state[0], prev_right_frozen_mp_state[1], prev_right_frozen_mp_state[0]);
                gap.gap_crossed_behind = true;
            }
            
            prev_left_frozen_mp_state = left_frozen_mp_state;
            prev_right_frozen_mp_state = right_frozen_mp_state;
            prev_central_bearing_vect = central_bearing_vect;
        }

        if (!gap.gap_crossed && !gap.gap_closed && !gap.gap_crossed_behind) {
            left_frozen_mp_state = left_model->get_frozen_modified_polar_state();
            right_frozen_mp_state = right_model->get_frozen_modified_polar_state();
            left_cross_pt << (1.0 / left_frozen_mp_state[0])*std::cos(left_frozen_mp_state[1]), (1.0 / left_frozen_mp_state[0])*std::sin(left_frozen_mp_state[1]);
            right_cross_pt << (1.0 / right_frozen_mp_state[0])*std::cos(right_frozen_mp_state[1]), (1.0 / right_frozen_mp_state[0])*std::sin(right_frozen_mp_state[1]);
            ROS_INFO_STREAM("no close, terminal points at: (" << left_cross_pt[0] << ", " << left_cross_pt[1] << "), (" << right_cross_pt[0] << ", " << right_cross_pt[1] << ")");

            generateTerminalPoints(gap, left_frozen_mp_state[1], left_frozen_mp_state[0], right_frozen_mp_state[1], right_frozen_mp_state[0]);
        }

        return cfg_->traj.integrate_maxt;
    }

    double GapFeasibilityChecker::generateCrossedGapTerminalPoints(double t, dynamic_gap::Gap & gap, dynamic_gap::cart_model* left_model, dynamic_gap::cart_model* right_model) {
        
        Matrix<double, 4, 1> left_rewind_mp_state = left_model->get_rewind_modified_polar_state();        
        Matrix<double, 4, 1> right_rewind_mp_state = right_model->get_rewind_modified_polar_state();
        Eigen::Vector2d left_cross_pt, right_cross_pt, left_bearing_vect, right_bearing_vect;

        // instantiate model rewind states
        left_model->set_rewind_state();
        right_model->set_rewind_state();
        // do rewind_propagate

        double beta_left, beta_right, range_left, range_right, r_min, L_to_R_angle;
        // REWINDING THE GAP FROM ITS CROSSED CONFIGURATION UNTIL THE GAP IS SUFFICIENTLY OPEN
        for (double t_rew = t; t_rew >= 0.0; t_rew -= cfg_->traj.integrate_stept) {
            left_model->rewind_propagate(-1 * cfg_->traj.integrate_stept); // resetting model we used before, not good
            right_model->rewind_propagate(-1 * cfg_->traj.integrate_stept);
            // ROS_INFO_STREAM("t_rew: " << t_rew);
            left_rewind_mp_state = left_model->get_rewind_modified_polar_state();
            right_rewind_mp_state = right_model->get_rewind_modified_polar_state();
            beta_left = left_rewind_mp_state[1];
            beta_right = right_rewind_mp_state[1]; 
            left_bearing_vect << std::cos(beta_left), std::sin(beta_left);
            right_bearing_vect << std::cos(beta_right), std::sin(beta_right);
            L_to_R_angle = getLeftToRightAngle(left_bearing_vect, right_bearing_vect);
            
            range_left = (1.0 / left_rewind_mp_state[0]);
            range_right = (1.0 / right_rewind_mp_state[0]);
            r_min = std::min(range_left, range_right);

            // if gap is sufficiently open
            // option 1: arc-length:
            // first condition, make sure slightly open gap is convex
            if (t_rew == 0 || r_min * L_to_R_angle > 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) {
                auto egocircle = *msg.get();        
                
                double wrapped_beta_left = atanThetaWrap(beta_left);
                double init_left_idx = (wrapped_beta_left - egocircle.angle_min) / egocircle.angle_increment;
                int left_idx = (int) std::floor(init_left_idx);

                double wrapped_term_beta_right = atanThetaWrap(beta_right);
                double init_right_idx = (wrapped_term_beta_right - egocircle.angle_min) / egocircle.angle_increment;
                int right_idx = (int) std::floor(init_right_idx);
                left_cross_pt << range_left*std::cos(wrapped_beta_left), 
                                range_left*std::sin(wrapped_beta_left);
                right_cross_pt << range_right*std::cos(wrapped_term_beta_right), 
                                range_right*std::sin(wrapped_term_beta_right);
                ROS_INFO_STREAM("terminal points at time " << t_rew << ", left: (" << left_cross_pt[0] << ", " << left_cross_pt[1] << "), right: (" << right_cross_pt[0] << ", " << right_cross_pt[1]);
                generateTerminalPoints(gap, wrapped_beta_left, left_rewind_mp_state[0], wrapped_term_beta_right, right_rewind_mp_state[0]);
                return t_rew;
            }
            
        }

        // gap.setTerminalPoints(float(gap.LIdx()), gap.LDist(), float(gap.RIdx()), gap.RDist());
        // return 0.0;

        // if falls out at t=0, just set terminal points equal to initial points? 
        // Lifespan would be 0, so would be infeasible anyways

        // should never fall out of this?
    }



    // THIS IS CALCULATE WITH LEFT AND RIGHT VECTORS FROM THE ROBOT'S POV
    double GapFeasibilityChecker::getLeftToRightAngle(Eigen::Vector2d left_norm_vect, Eigen::Vector2d right_norm_vect) {
        double determinant = left_norm_vect[1]*right_norm_vect[0] - left_norm_vect[0]*right_norm_vect[1];
        double dot_product = left_norm_vect[0]*right_norm_vect[0] + left_norm_vect[1]*right_norm_vect[1];

        double left_to_right_angle = std::atan2(determinant, dot_product);
        
        if (left_to_right_angle < 0) {
            left_to_right_angle += 2*M_PI; 
        }

        return left_to_right_angle;
    }

    double GapFeasibilityChecker::atanThetaWrap(double theta) {
        double new_theta = theta;
        while (new_theta <= -M_PI) {
            new_theta += 2*M_PI;
            ROS_INFO_STREAM("wrapping theta: " << theta << " to new_theta: " << new_theta);
        } 
        
        while (new_theta >= M_PI) {
            new_theta -= 2*M_PI;
            ROS_INFO_STREAM("wrapping theta: " << theta << " to new_theta: " << new_theta);
        }

        return new_theta;
    }

    void GapFeasibilityChecker::generateTerminalPoints(dynamic_gap::Gap & gap, double terminal_beta_left, double terminal_reciprocal_range_left, 
                                                                                 double terminal_beta_right, double terminal_reciprocal_range_right) {
        auto egocircle = *msg.get();        
        
        double wrapped_term_beta_left = atanThetaWrap(terminal_beta_left);
        float init_left_idx = (terminal_beta_left - egocircle.angle_min) / egocircle.angle_increment;
        int left_idx = (int) std::floor(init_left_idx);
        float left_dist = (1.0 / terminal_reciprocal_range_left);

        double wrapped_term_beta_right = atanThetaWrap(terminal_beta_right);
        float init_right_idx = (terminal_beta_right - egocircle.angle_min) / egocircle.angle_increment;
        int right_idx = (int) std::floor(init_right_idx);
        float right_dist = (1.0 / terminal_reciprocal_range_right);
        // if (left_idx == right_idx) right_idx++;

        gap.setTerminalPoints(left_idx, left_dist, right_idx, right_dist);
    }

}
