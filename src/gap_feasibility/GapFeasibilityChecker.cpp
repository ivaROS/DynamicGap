
#include <dynamic_gap/gap_feasibility/GapFeasibilityChecker.h>

namespace dynamic_gap {

    void GapFeasibilityChecker::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) 
    {
        boost::mutex::scoped_lock lock(egolock);
        msg = msg_;
        auto scan = *msg.get();
        num_of_scan = (int)(scan.ranges.size());
        scan_angle_increment_ = scan.angle_increment;
        scan_angle_min_ = scan.angle_min;
    }

    bool GapFeasibilityChecker::indivGapFeasibilityCheck(dynamic_gap::Gap& gap) 
    {
        if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("        [indivGapFeasibilityCheck()]");
        dynamic_gap::Estimator* left_model = gap.left_model;
        dynamic_gap::Estimator* right_model = gap.right_model;

        // int tmp1 = 0;
        // int tmp2 = 1;

        // int test = tmp2 / tmp1; // no catch

        // std::vector<float> testVector(10);
        // testVector.at(-1);

        left_model->isolateGapDynamics();
        right_model->isolateGapDynamics();

        Eigen::Vector4f frozen_left_model_state = left_model->getGapState(); // left_model->get_frozen_modified_polar_state();
        Eigen::Vector4f frozen_right_model_state = right_model->getGapState(); // right_model->get_frozen_modified_polar_state();

        float frozen_left_betadot = getGapBearingRateOfChange(frozen_left_model_state); // frozen_left_model_state[3];
        float frozen_right_betadot = getGapBearingRateOfChange(frozen_right_model_state); // frozen_right_model_state[3];

        float min_betadot = std::min(frozen_left_betadot, frozen_right_betadot);
        float subtracted_left_betadot = frozen_left_betadot - min_betadot;
        float subtracted_right_betadot = frozen_right_betadot - min_betadot;

        // ROS_INFO_STREAM("frozen left betadot: " << frozen_left_betadot);
        // ROS_INFO_STREAM("frozen right betadot: " << frozen_right_betadot);

        // float start_time = ros::Time::now().toSec();
        float crossing_time = gapSplinecheck(gap, left_model, right_model);
        // ROS_INFO_STREAM("gapSplinecheck time elapsed:" << ros::Time::now().toSec() - start_time);


        bool feasible = false;
        if (gap.artificial) 
        {
            feasible = true;
            gap.gap_lifespan = cfg_->traj.integrate_maxt;
            gap.setTerminalPoints(gap.LIdx(), gap.LDist(), gap.RIdx(), gap.RDist());
            gap.setCategory("static");
        } else if (subtracted_left_betadot > 0) 
        {
            // expanding
            feasible = true;
            gap.gap_lifespan = cfg_->traj.integrate_maxt;
            gap.setCategory("expanding");
        } else if (subtracted_left_betadot == 0 && subtracted_right_betadot == 0) 
        {
            // static
            feasible = true;
            gap.gap_lifespan = cfg_->traj.integrate_maxt;
            gap.setCategory("static");
        } else 
        {
            // closing
            gap.setCategory("closing"); 
            if (crossing_time >= 0) 
            {
                feasible = true;
                gap.gap_lifespan = crossing_time;
            }
        }

        if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("            gap is feasible: " << feasible);

        return feasible;
    }
    

    float GapFeasibilityChecker::gapSplinecheck(dynamic_gap::Gap & gap, dynamic_gap::Estimator* left_model, dynamic_gap::Estimator* right_model) 
    {
        if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("            [indivGapFeasibilityCheck()]");        
        Eigen::Vector2f crossing_pt(0.0, 0.0);
        float start_time = ros::Time::now().toSec();
        float crossing_time = indivGapFindCrossingPoint(gap, crossing_pt, left_model, right_model);
        // ROS_INFO_STREAM("indivGapFindCrossingPoint time elapsed:" << ros::Time::now().toSec() - start_time);

        Eigen::Vector2f starting_pos(0.0, 0.0);
        Eigen::Vector2f starting_vel(left_model->getRobotVel().twist.linear.x, left_model->getRobotVel().twist.linear.y);
        
        Eigen::Vector2f ending_vel(0.0, 0.0);
        
        if (crossing_pt.norm() > 0) {
            ending_vel << starting_vel.norm() * crossing_pt[0] / crossing_pt.norm(), starting_vel.norm() * crossing_pt[1] / crossing_pt.norm();
        } 
        
        // ROS_INFO_STREAM("starting x: " << starting_pos[0] << ", " << starting_pos[1] << ", " << starting_vel[0] << ", " << starting_vel[1]);
        // ROS_INFO_STREAM("ending x: " << crossing_pt[0] << ", " << crossing_pt[1] << ", ending_vel: " << ending_vel[0] << ", " << ending_vel[1]);
        
        start_time = ros::Time::now().toSec();
        Eigen::MatrixXf A_spline = Eigen::MatrixXf::Random(4,4);
        Eigen::VectorXf b_spline = Eigen::VectorXf::Random(4);
        A_spline << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    1.0, crossing_time, pow(crossing_time,2), pow(crossing_time,3),
                    0.0, 1.0, 2*crossing_time, 3*pow(crossing_time,2);
        b_spline << starting_pos[0], starting_vel[0], crossing_pt[0], ending_vel[0];
        gap.spline_x_coefs = A_spline.partialPivLu().solve(b_spline);

        // std::cout << "x coeffs: " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ", " << coeffs[3] << std::endl;
        float peak_velocity_time = crossing_time/2.0;
        float peak_velocity_x = 3*gap.spline_x_coefs[3]*pow(peak_velocity_time, 2) + 
                                 2*gap.spline_x_coefs[2]*peak_velocity_time + 
                                 gap.spline_x_coefs[1];
        
        b_spline << starting_pos[1], starting_vel[1], crossing_pt[1], ending_vel[1];

        gap.spline_y_coefs = A_spline.partialPivLu().solve(b_spline);
        //std::cout << "y coeffs: " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ", " << coeffs[3] << std::endl;
        float peak_velocity_y = 3*gap.spline_y_coefs[3]*pow(peak_velocity_time, 2) + 
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
 
    float GapFeasibilityChecker::indivGapFindCrossingPoint(dynamic_gap::Gap & gap, Eigen::Vector2f& gap_crossing_point, 
                                                            dynamic_gap::Estimator* left_model, dynamic_gap::Estimator* right_model) 
    {
        if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("                [indivGapFindCrossingPoint()]");
        // auto egocircle = *msg.get();

        float x_r, x_l, y_r, y_l;

        float beta_left = idx2theta(gap.LIdx());
        float beta_right = idx2theta(gap.RIdx());
        x_l = gap.LDist() * cos(beta_left);
        y_l = gap.LDist() * sin(beta_left);
        x_r = gap.RDist() * cos(beta_right);
        y_r = gap.RDist() * sin(beta_right);
       
        Eigen::Vector2f leftBearingVect(x_l / gap.LDist(), y_l / gap.LDist());
        Eigen::Vector2f rightBearingVect(x_r / gap.RDist(), y_r / gap.RDist());

        float L_to_R_angle = getLeftToRightAngle(leftBearingVect, rightBearingVect);
        
        Eigen::Vector2f prevLeftBearingVect = leftBearingVect;        
        Eigen::Vector2f prevRightBearingVect = rightBearingVect;

        float beta_center = (beta_left - (L_to_R_angle / 2.0));

        // float prev_beta_left = beta_left;
        // float prev_beta_right = beta_right;
        float prev_L_to_R_angle = L_to_R_angle;
        

        Eigen::Vector2f central_bearing_vect(std::cos(beta_center), std::sin(beta_center));
        
        //std::cout << "initial beta left: (" << leftBearingVect[0] << ", " << leftBearingVect[1] << "), initial beta right: (" << rightBearingVect[0] << ", " << rightBearingVect[1] << "), initial beta center: (" << central_bearing_vect[0] << ", " << central_bearing_vect[1] << ")" << std::endl;
        
        // Eigen::Vector4f left_frozen_mp_state = left_model->get_frozen_modified_polar_state();        
        // Eigen::Vector4f right_frozen_mp_state = right_model->get_frozen_modified_polar_state();
        Eigen::Vector4f leftGapState = left_model->getGapState();
        Eigen::Vector4f rightGapState = right_model->getGapState();

        // ROS_INFO_STREAM("gap category: " << gap.getCategory());
        // ROS_INFO_STREAM("starting frozen cartesian left: " << leftGapState[0] << ", " << leftGapState[1] << ", " << leftGapState[2] << ", " << leftGapState[3]); 
        // ROS_INFO_STREAM("starting frozen cartesian right: " << rightGapState[0] << ", " << rightGapState[1] << ", " << rightGapState[2] << ", " << rightGapState[3]);

        // float beta_left, beta_right, beta_center;
       
        float left_central_dot, right_central_dot;
        bool first_cross = true;
        bool bearing_crossing_check, range_closing_check;    

        bool left_opening = (getGapBearingRateOfChange(leftGapState) > 0.0);
        bool right_opening = (getGapBearingRateOfChange(rightGapState) < 0.0);
        Eigen::Vector4f prevLeftGapState = leftGapState;
        Eigen::Vector4f prevRightGapState = rightGapState;
        // Eigen::Vector4f prev_left_frozen_mp_state = left_frozen_mp_state;        
        // Eigen::Vector4f prev_right_frozen_mp_state = right_frozen_mp_state;        
        Eigen::Vector2f prev_central_bearing_vect = central_bearing_vect;

        Eigen::Vector2f leftCrossPt, rightCrossPt;
        bool left_range_less_radius, right_range_less_radius, gap_collided_w_rbt;
        for (float t = cfg_->traj.integrate_stept; t < cfg_->traj.integrate_maxt; t += cfg_->traj.integrate_stept) 
        {
            // checking to see if left point is reachable
            if (!(left_opening && (getGapDist(leftGapState) < cfg_->control.vx_absmax * t))) 
            {
                // ROS_INFO_STREAM("propagating left");
                left_model->gapStatePropagate(cfg_->traj.integrate_stept);
            }

            // checking to see if right point is reachable
            if (!(right_opening && (getGapDist(rightGapState) < cfg_->control.vx_absmax * t))) 
            {
                // ROS_INFO_STREAM("propagating right");
                right_model->gapStatePropagate(cfg_->traj.integrate_stept);
            }
            // ROS_INFO_STREAM("t: " << t);
            // left_frozen_mp_state = left_model->get_frozen_modified_polar_state();
            // right_frozen_mp_state = right_model->get_frozen_modified_polar_state();
            leftGapState = left_model->getGapState();
            rightGapState = right_model->getGapState();

            left_range_less_radius = getGapDist(leftGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            right_range_less_radius = getGapDist(rightGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            gap_collided_w_rbt = (left_range_less_radius || right_range_less_radius);

            if (gap_collided_w_rbt) 
            {
                gap_crossing_point << 0.0, 0.0;
                gap.setClosingPoint(gap_crossing_point[0], gap_crossing_point[1]);
                gap.gap_closed = true;

                return t;
            }

            beta_left = getGapBearing(leftGapState);
            beta_right = getGapBearing(rightGapState);
            // ROS_INFO_STREAM("beta_left: " << beta_left << ", beta_right: " << beta_right);
            leftBearingVect = leftGapState.head(2) / getGapDist(leftGapState); // << std::cos(beta_left), std::sin(beta_left);
            rightBearingVect = rightGapState.head(2) / getGapDist(rightGapState); // << std::cos(beta_right), std::sin(beta_right);
            L_to_R_angle = getLeftToRightAngle(leftBearingVect, rightBearingVect);
            beta_center = (beta_left - 0.5 * L_to_R_angle);

            // prev_beta_left = getGapBearing(prevLeftGapState); // prev_left_frozen_mp_state[1];
            // prev_beta_right = getGapBearing(prevRightGapState); // prev_right_frozen_mp_state[1];
            prevLeftBearingVect = prevLeftGapState.head(2) / getGapDist(prevLeftGapState); // << std::cos(prev_beta_left), std::sin(prev_beta_left);
            prevRightBearingVect = prevRightGapState.head(2) / getGapDist(prevRightGapState); // << std::cos(prev_beta_right), std::sin(prev_beta_right);
            prev_L_to_R_angle = getLeftToRightAngle(prevLeftBearingVect, prevRightBearingVect);            

            // ROS_INFO_STREAM("prev_L_to_R_angle: " << prev_L_to_R_angle << ", L_to_R_angle: " << L_to_R_angle);

            central_bearing_vect << std::cos(beta_center), std::sin(beta_center);
        
            left_central_dot = leftBearingVect.dot(prev_central_bearing_vect);
            right_central_dot = rightBearingVect.dot(prev_central_bearing_vect);
            bearing_crossing_check = left_central_dot > 0.0 && right_central_dot > 0.0;

            // checking for bearing crossing conditions for closing and crossing gaps
            if (L_to_R_angle > M_PI && bearing_crossing_check) 
            {
                if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("                    bearing cross at " << t);
                // CLOSING GAP CHECK
                leftCrossPt = prevLeftGapState.head(2);
                                 // (1.0 / prev_left_frozen_mp_state[0])*std::cos(prev_left_frozen_mp_state[1]), 
                                 //   (1.0 / prev_left_frozen_mp_state[0])*std::sin(prev_left_frozen_mp_state[1]);
                rightCrossPt << prevRightGapState.head(2);
                                 // (1.0 / prev_right_frozen_mp_state[0])*std::cos(prev_right_frozen_mp_state[1]), 
                                 //   (1.0 / prev_right_frozen_mp_state[0])*std::sin(prev_right_frozen_mp_state[1]);

                range_closing_check = (leftCrossPt - rightCrossPt).norm()  < 2*cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; 
                // IF POINTS ARE SUFFICIENTLY CLOSE TOGETHER, GAP HAS CLOSED
                if (range_closing_check) 
                {    
                    if (leftCrossPt.norm() < rightCrossPt.norm())
                        gap_crossing_point << rightCrossPt[0], rightCrossPt[1];
                    else
                        gap_crossing_point << leftCrossPt[0], leftCrossPt[1];

                    gap_crossing_point += 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * (gap_crossing_point / gap_crossing_point.norm());
                    gap.setClosingPoint(gap_crossing_point[0], gap_crossing_point[1]);
                    float ending_time = generateCrossedGapTerminalPoints(t, gap, left_model, right_model);
                    if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("                    considering gap closed at " << ending_time); 

                    gap.gap_closed = true;
                    return ending_time;
                } else
                {
                    if (first_cross) 
                    {
                        float mid_x = (leftCrossPt[0] + rightCrossPt[0]) / 2;
                        float mid_y = (leftCrossPt[1] + rightCrossPt[1]) / 2;
                        //  ROS_INFO_STREAM("gap crosses but does not close at " << t << ", left point at: " << leftCrossPt[0] << ", " << leftCrossPt[1] << ", right point at " << rightCrossPt[0] << ", " << rightCrossPt[1]); 

                        gap.setCrossingPoint(mid_x, mid_y);
                        first_cross = false;

                        float ending_time = generateCrossedGapTerminalPoints(t, gap, left_model, right_model);
                        if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("                    considering gap crossed at " << ending_time); 

                        gap.gap_crossed = true;
                    }
                }
            } else if (prev_L_to_R_angle > (3*M_PI / 4) && L_to_R_angle < (M_PI / 4)) 
            {
                // checking for case of gap crossing behind the robot
                // leftCrossPt = prevLeftGapState.head(2);
                                 // (1.0 / prev_left_frozen_mp_state[0])*std::cos(prev_left_frozen_mp_state[1]), 
                                 // (1.0 / prev_left_frozen_mp_state[0])*std::sin(prev_left_frozen_mp_state[1]);
                // rightCrossPt << prevRightGapState.head(2);                
                                 // (1.0 / prev_right_frozen_mp_state[0])*std::cos(prev_right_frozen_mp_state[1]), 
                                 // (1.0 / prev_right_frozen_mp_state[0])*std::sin(prev_right_frozen_mp_state[1]);
                if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("                    crossing from behind, terminal points at: (" << 
                                prevLeftGapState[0] << ", " << prevLeftGapState[1] << "), (" << 
                                prevRightGapState[0] << ", " << prevRightGapState[1] << ")");
                generateTerminalPoints(gap, prevLeftGapState, prevRightGapState); // prev_left_frozen_mp_state[1], prev_left_frozen_mp_state[0], prev_right_frozen_mp_state[1], prev_right_frozen_mp_state[0]);
                gap.gap_crossed_behind = true;
            }
            
            prevLeftGapState = leftGapState;
            prevRightGapState = rightGapState;
            // prev_left_frozen_mp_state = left_frozen_mp_state;
            // prev_right_frozen_mp_state = right_frozen_mp_state;
            prev_central_bearing_vect = central_bearing_vect;
        }

        if (!gap.gap_crossed && !gap.gap_closed && !gap.gap_crossed_behind) 
        {
            // left_frozen_mp_state = left_model->get_frozen_modified_polar_state();
            // right_frozen_mp_state = right_model->get_frozen_modified_polar_state();
            // leftCrossPt = leftGapState.head(2);
            // rightCrossPt << rightGapState.head(2);                

            // leftCrossPt << (1.0 / left_frozen_mp_state[0])*std::cos(left_frozen_mp_state[1]), (1.0 / left_frozen_mp_state[0])*std::sin(left_frozen_mp_state[1]);
            // rightCrossPt << (1.0 / right_frozen_mp_state[0])*std::cos(right_frozen_mp_state[1]), (1.0 / right_frozen_mp_state[0])*std::sin(right_frozen_mp_state[1]);
            if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("                    no close, terminal points at: (" << 
                    leftGapState[0] << ", " << leftGapState[1] << "), (" << 
                    rightGapState[0] << ", " << rightGapState[1] << ")");

            generateTerminalPoints(gap, leftGapState, rightGapState); // left_frozen_mp_state[1], left_frozen_mp_state[0], right_frozen_mp_state[1], right_frozen_mp_state[0]);
        }

        return cfg_->traj.integrate_maxt;
    }

    float GapFeasibilityChecker::generateCrossedGapTerminalPoints(float t, dynamic_gap::Gap & gap, 
                                                                    dynamic_gap::Estimator* left_model, 
                                                                    dynamic_gap::Estimator* right_model) 
    {    
        // Eigen::Vector4f left_rewind_mp_state = left_model->get_rewind_modified_polar_state();        
        // Eigen::Vector4f right_rewind_mp_state = right_model->get_rewind_modified_polar_state();
        Eigen::Vector4f rewindLeftGapState = left_model->getGapState();
        Eigen::Vector4f rewindRightGapState = right_model->getGapState();        

        Eigen::Vector2f leftCrossPt, rightCrossPt; // , leftBearingVect, rightBearingVect;

        // instantiate model rewind states
        left_model->setRewindState();
        right_model->setRewindState();
        // do rewindPropagate
        // auto egocircle = *msg.get();        

        // float beta_left, beta_right, range_left, range_right, r_min, L_to_R_angle;
        // REWINDING THE GAP FROM ITS CROSSED CONFIGURATION UNTIL THE GAP IS SUFFICIENTLY OPEN
        for (float t_rew = t; t_rew >= 0.0; t_rew -= cfg_->traj.integrate_stept) 
        {
            left_model->rewindPropagate(-1 * cfg_->traj.integrate_stept); // resetting model we used before, not good
            right_model->rewindPropagate(-1 * cfg_->traj.integrate_stept);
            // ROS_INFO_STREAM("t_rew: " << t_rew);
            // left_rewind_mp_state = left_model->get_rewind_modified_polar_state();
            // right_rewind_mp_state = right_model->get_rewind_modified_polar_state();
            rewindLeftGapState = left_model->getRewindGapState();
            rewindRightGapState = right_model->getRewindGapState();  
            // beta_left = getGapBearing(rewindLeftGapState); // left_rewind_mp_state[1];
            // beta_right = getGapBearing(rewindRightGapState); // right_rewind_mp_state[1]; 
            // leftBearingVect << std::cos(beta_left), std::sin(beta_left);
            // rightBearingVect << std::cos(beta_right), std::sin(beta_right);
            // L_to_R_angle = getLeftToRightAngle(leftBearingVect, rightBearingVect);
            
            // range_left = getGapDist(rewindLeftGapState); // (1.0 / left_rewind_mp_state[0]);
            // range_right = getGapDist(rewindRightGapState); // (1.0 / right_rewind_mp_state[0]);
            // r_min = std::min(range_left, range_right);

            // // float wrapped_beta_left = atanThetaWrap(beta_left);
            // float init_left_idx = (wrapped_beta_left - scan_angle_min_) / scan_angle_increment_;
            // int left_idx = (int) std::floor(init_left_idx);

            // // float wrapped_term_beta_right = atanThetaWrap(beta_right);
            // float init_right_idx = (wrapped_term_beta_right - scan_angle_min_) / scan_angle_increment_;
            // int right_idx = (int) std::floor(init_right_idx);

            leftCrossPt = rewindLeftGapState.head(2);
                            // range_left*std::cos(wrapped_beta_left), 
                            // range_left*std::sin(wrapped_beta_left);
            rightCrossPt = rewindRightGapState.head(2);
                            // range_right*std::cos(wrapped_term_beta_right), 
                            // range_right*std::sin(wrapped_term_beta_right);            

            // if gap is sufficiently open
            // option 1: arc-length:
            if (t_rew == 0 || (leftCrossPt - rightCrossPt).norm() >  2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) // r_min * L_to_R_angle > 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio
            { 
                if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("terminal points at time " << t_rew << ", left: (" << leftCrossPt[0] << ", " << leftCrossPt[1] << "), right: (" << rightCrossPt[0] << ", " << rightCrossPt[1]);
                generateTerminalPoints(gap, rewindLeftGapState, rewindRightGapState);
                                        // wrapped_beta_left, left_rewind_mp_state[0], 
                                        // wrapped_term_beta_right, right_rewind_mp_state[0]);
                return t_rew;
            }
            
        }

        // gap.setTerminalPoints(float(gap.LIdx()), gap.LDist(), float(gap.RIdx()), gap.RDist());
        // return 0.0;

        // if falls out at t=0, just set terminal points equal to initial points? 
        // Lifespan would be 0, so would be infeasible anyways

        // should never fall out of this?
        return 0.0;
    }

    void GapFeasibilityChecker::generateTerminalPoints(dynamic_gap::Gap & gap, 
                                                        Eigen::Vector4f leftCrossPt,
                                                        Eigen::Vector4f rightCrossPt)
                                                        // float terminal_beta_left, float terminal_reciprocal_range_left, 
                                                        // float terminal_beta_right, float terminal_reciprocal_range_right) 
    {
        // auto egocircle = *msg.get();        
        
        float beta_left = getGapBearing(leftCrossPt);
        int left_idx = theta2idx(beta_left);
        float left_dist = getGapDist(leftCrossPt);

        // float wrapped_term_beta_left = atanThetaWrap(getGapBearing(leftCrossPt));
        // float init_left_idx = (wrapped_term_beta_left - scan_angle_min_) / scan_angle_increment_;
        // int left_idx = (int) std::floor(init_left_idx);
        // float left_dist = getGapDist(leftCrossPt); // (1.0 / terminal_reciprocal_range_left);

        // float wrapped_term_beta_right = atanThetaWrap(getGapBearing(rightCrossPt));
        // float init_right_idx = (wrapped_term_beta_right - scan_angle_min_) / scan_angle_increment_;
        // int right_idx = (int) std::floor(init_right_idx);
        // float right_dist = getGapDist(rightCrossPt); // (1.0 / terminal_reciprocal_range_right);
        // if (left_idx == right_idx) right_idx++;

        float beta_right = getGapBearing(rightCrossPt);
        int right_idx = theta2idx(beta_right);
        float right_dist = getGapDist(rightCrossPt);

        if (cfg_->debug.feasibility_debug_log) 
        {
            ROS_INFO_STREAM("            setting terminal points to, left: (" << 
                                        leftCrossPt[0] << ", " << leftCrossPt[1] << "), right: (" << 
                                        rightCrossPt[0] << ", " << rightCrossPt[1] << ")");
        }
        gap.setTerminalPoints(left_idx, left_dist, right_idx, right_dist);
    }

}
