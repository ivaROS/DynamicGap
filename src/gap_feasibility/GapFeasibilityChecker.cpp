
#include <dynamic_gap/gap_feasibility/GapFeasibilityChecker.h>

namespace dynamic_gap 
{
    /*
    void GapFeasibilityChecker::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) 
    {
        boost::mutex::scoped_lock lock(egolock);
        msg = msg_;
        auto scan = *msg.get();
        num_of_scan = (int)(scan.ranges.size());
        scan_angle_increment_ = scan.angle_increment;
        scan_angle_min_ = scan.angle_min;
    }
    */

    bool GapFeasibilityChecker::indivGapFeasibilityCheck(dynamic_gap::Gap& gap) 
    {
        if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("        [indivGapFeasibilityCheck()]");
        dynamic_gap::Estimator* leftGapPtModel = gap.leftGapPtModel;
        dynamic_gap::Estimator* rightGapPtModel = gap.rightGapPtModel;

        // int tmp1 = 0;
        // int tmp2 = 1;

        // int test = tmp2 / tmp1; // no catch

        // std::vector<float> testVector(10);
        // testVector.at(-1);

        leftGapPtModel->isolateGapDynamics();
        rightGapPtModel->isolateGapDynamics();

        Eigen::Vector4f leftGapState = leftGapPtModel->getGapState(); // leftGapPtModel->get_frozen_modified_polar_state();
        Eigen::Vector4f rightGapState = rightGapPtModel->getGapState(); // rightGapPtModel->get_frozen_modified_polar_state();

        float leftGapBearingRate = getGapBearingRateOfChange(leftGapState); // leftGapState[3];
        float rightGapBearingRate = getGapBearingRateOfChange(rightGapState); // rightGapState[3];

        float minGapBearingRate = std::min(leftGapBearingRate, rightGapBearingRate);
        float subLeftGapBearingRate = leftGapBearingRate - minGapBearingRate;
        float subRightGapBearingRate = rightGapBearingRate - minGapBearingRate;

        // ROS_INFO_STREAM("frozen left betadot: " << leftGapBearingRateOfChange);
        // ROS_INFO_STREAM("frozen right betadot: " << rightGapBearingRateOfChange);

        // float start_time = ros::Time::now().toSec();
        float crossing_time = gapSplinecheck(gap, leftGapPtModel, rightGapPtModel);
        // ROS_INFO_STREAM("gapSplinecheck time elapsed:" << ros::Time::now().toSec() - start_time);


        bool feasible = false;
        if (gap.artificial) 
        {
            feasible = true;
            gap.gap_lifespan = cfg_->traj.integrate_maxt;
            gap.setTerminalPoints(gap.LIdx(), gap.LDist(), gap.RIdx(), gap.RDist());
            gap.setCategory("static");
        } else if (subLeftGapBearingRate > 0) 
        {
            // expanding
            feasible = true;
            gap.gap_lifespan = cfg_->traj.integrate_maxt;
            gap.setCategory("expanding");
        } else if (subLeftGapBearingRate == 0 && subRightGapBearingRate == 0) 
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
    

    float GapFeasibilityChecker::gapSplinecheck(dynamic_gap::Gap & gap, dynamic_gap::Estimator* leftGapPtModel, dynamic_gap::Estimator* rightGapPtModel) 
    {
        if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("            [indivGapFeasibilityCheck()]");        
        Eigen::Vector2f crossing_pt(0.0, 0.0);
        // float start_time = ros::Time::now().toSec();
        float gapCrossingTime = indivGapFindCrossingPoint(gap, crossing_pt, leftGapPtModel, rightGapPtModel);
        // ROS_INFO_STREAM("indivGapFindCrossingPoint time elapsed:" << ros::Time::now().toSec() - start_time);

        Eigen::Vector2f starting_pos(0.0, 0.0);
        Eigen::Vector2f starting_vel(leftGapPtModel->getRobotVel().twist.linear.x, leftGapPtModel->getRobotVel().twist.linear.y);
        
        Eigen::Vector2f ending_vel(0.0, 0.0);
        
        if (crossing_pt.norm() > 0) {
            ending_vel << starting_vel.norm() * crossing_pt[0] / crossing_pt.norm(), starting_vel.norm() * crossing_pt[1] / crossing_pt.norm();
        } 
        
        // ROS_INFO_STREAM("starting x: " << starting_pos[0] << ", " << starting_pos[1] << ", " << starting_vel[0] << ", " << starting_vel[1]);
        // ROS_INFO_STREAM("ending x: " << crossing_pt[0] << ", " << crossing_pt[1] << ", ending_vel: " << ending_vel[0] << ", " << ending_vel[1]);
        
        // start_time = ros::Time::now().toSec();
        Eigen::MatrixXf A_spline = Eigen::MatrixXf::Random(4,4);
        Eigen::VectorXf b_spline = Eigen::VectorXf::Random(4);
        A_spline << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    1.0, gapCrossingTime, pow(gapCrossingTime,2), pow(gapCrossingTime,3),
                    0.0, 1.0, 2*gapCrossingTime, 3*pow(gapCrossingTime,2);
        b_spline << starting_pos[0], starting_vel[0], crossing_pt[0], ending_vel[0];
        gap.spline_x_coefs = A_spline.partialPivLu().solve(b_spline);

        // std::cout << "x coeffs: " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ", " << coeffs[3] << std::endl;
        float peak_velocity_time = gapCrossingTime/2.0;
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
            return gapCrossingTime;
        } else {
            return -1.0;
        }
    }
 
    float GapFeasibilityChecker::indivGapFindCrossingPoint(dynamic_gap::Gap & gap, 
                                                           Eigen::Vector2f& gapCrossingPt, 
                                                           dynamic_gap::Estimator* leftGapPtModel, 
                                                           dynamic_gap::Estimator* rightGapPtModel) 
    {
        if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("                [indivGapFindCrossingPoint()]");
        // auto egocircle = *msg.get();

        float xLeft, yLeft, xRight, yRight;

        float thetaLeft = idx2theta(gap.LIdx());
        float thetaRight = idx2theta(gap.RIdx());
        xLeft = gap.LDist() * cos(thetaLeft);
        yLeft = gap.LDist() * sin(thetaLeft);
        xRight = gap.RDist() * cos(thetaRight);
        yRight = gap.RDist() * sin(thetaRight);
       
        Eigen::Vector2f leftBearingVect(xLeft / gap.LDist(), yLeft / gap.LDist());
        Eigen::Vector2f rightBearingVect(xRight / gap.RDist(), yRight / gap.RDist());

        float leftToRightAngle = getLeftToRightAngle(leftBearingVect, rightBearingVect);
        
        Eigen::Vector2f prevLeftBearingVect = leftBearingVect;        
        Eigen::Vector2f prevRightBearingVect = rightBearingVect;

        float thetaCenter = (thetaLeft - (leftToRightAngle / 2.0));

        // float prev_thetaLeft = thetaLeft;
        // float prev_thetaRight = thetaRight;
        float prevLeftToRightAngle = leftToRightAngle;
        

        Eigen::Vector2f centralBearingVect(std::cos(thetaCenter), std::sin(thetaCenter));
        
        //std::cout << "initial beta left: (" << leftBearingVect[0] << ", " << leftBearingVect[1] << "), initial beta right: (" << rightBearingVect[0] << ", " << rightBearingVect[1] << "), initial beta center: (" << centralBearingVect[0] << ", " << centralBearingVect[1] << ")" << std::endl;
        
        // Eigen::Vector4f left_frozen_mp_state = leftGapPtModel->get_frozen_modified_polar_state();        
        // Eigen::Vector4f right_frozen_mp_state = rightGapPtModel->get_frozen_modified_polar_state();
        Eigen::Vector4f leftGapState = leftGapPtModel->getGapState();
        Eigen::Vector4f rightGapState = rightGapPtModel->getGapState();

        // ROS_INFO_STREAM("gap category: " << gap.getCategory());
        // ROS_INFO_STREAM("starting frozen cartesian left: " << leftGapState[0] << ", " << leftGapState[1] << ", " << leftGapState[2] << ", " << leftGapState[3]); 
        // ROS_INFO_STREAM("starting frozen cartesian right: " << rightGapState[0] << ", " << rightGapState[1] << ", " << rightGapState[2] << ", " << rightGapState[3]);

        // float thetaLeft, thetaRight, thetaCenter;
       
        float left_central_dot, right_central_dot;
        bool first_cross = true;
        bool bearing_crossing_check, range_closing_check;    

        bool left_opening = (getGapBearingRateOfChange(leftGapState) > 0.0);
        bool right_opening = (getGapBearingRateOfChange(rightGapState) < 0.0);
        Eigen::Vector4f prevLeftGapState = leftGapState;
        Eigen::Vector4f prevRightGapState = rightGapState;
        // Eigen::Vector4f prev_left_frozen_mp_state = left_frozen_mp_state;        
        // Eigen::Vector4f prev_right_frozen_mp_state = right_frozen_mp_state;        
        Eigen::Vector2f prevCentralBearingVect = centralBearingVect;

        Eigen::Vector2f leftCrossPt, rightCrossPt;
        bool leftGapPtCollision, rightGapPtCollision, collision;
        for (float t = cfg_->traj.integrate_stept; t < cfg_->traj.integrate_maxt; t += cfg_->traj.integrate_stept) 
        {
            // checking to see if left point is reachable
            if (!(left_opening && (getGapDist(leftGapState) < cfg_->control.vx_absmax * t))) 
            {
                // ROS_INFO_STREAM("propagating left");
                leftGapPtModel->gapStatePropagate(cfg_->traj.integrate_stept);
            }

            // checking to see if right point is reachable
            if (!(right_opening && (getGapDist(rightGapState) < cfg_->control.vx_absmax * t))) 
            {
                // ROS_INFO_STREAM("propagating right");
                rightGapPtModel->gapStatePropagate(cfg_->traj.integrate_stept);
            }
            // ROS_INFO_STREAM("t: " << t);
            // left_frozen_mp_state = leftGapPtModel->get_frozen_modified_polar_state();
            // right_frozen_mp_state = rightGapPtModel->get_frozen_modified_polar_state();
            leftGapState = leftGapPtModel->getGapState();
            rightGapState = rightGapPtModel->getGapState();

            leftGapPtCollision = getGapDist(leftGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            rightGapPtCollision = getGapDist(rightGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            collision = (leftGapPtCollision || rightGapPtCollision);

            if (collision) 
            {
                gapCrossingPt << 0.0, 0.0;
                gap.setClosingPoint(gapCrossingPt[0], gapCrossingPt[1]);
                gap.closed_ = true;

                return t;
            }

            thetaLeft = getGapBearing(leftGapState);
            thetaRight = getGapBearing(rightGapState);
            // ROS_INFO_STREAM("thetaLeft: " << thetaLeft << ", thetaRight: " << thetaRight);
            leftBearingVect = leftGapState.head(2) / getGapDist(leftGapState); // << std::cos(thetaLeft), std::sin(thetaLeft);
            rightBearingVect = rightGapState.head(2) / getGapDist(rightGapState); // << std::cos(thetaRight), std::sin(thetaRight);
            leftToRightAngle = getLeftToRightAngle(leftBearingVect, rightBearingVect);
            thetaCenter = (thetaLeft - 0.5 * leftToRightAngle);

            // prev_thetaLeft = getGapBearing(prevLeftGapState); // prev_left_frozen_mp_state[1];
            // prev_thetaRight = getGapBearing(prevRightGapState); // prev_right_frozen_mp_state[1];
            prevLeftBearingVect = prevLeftGapState.head(2) / getGapDist(prevLeftGapState); // << std::cos(prev_thetaLeft), std::sin(prev_thetaLeft);
            prevRightBearingVect = prevRightGapState.head(2) / getGapDist(prevRightGapState); // << std::cos(prev_thetaRight), std::sin(prev_thetaRight);
            prevLeftToRightAngle = getLeftToRightAngle(prevLeftBearingVect, prevRightBearingVect);            

            // ROS_INFO_STREAM("prevLeftToRightAngle: " << prevLeftToRightAngle << ", leftToRightAngle: " << leftToRightAngle);

            centralBearingVect << std::cos(thetaCenter), std::sin(thetaCenter);
        
            left_central_dot = leftBearingVect.dot(prevCentralBearingVect);
            right_central_dot = rightBearingVect.dot(prevCentralBearingVect);
            bearing_crossing_check = left_central_dot > 0.0 && right_central_dot > 0.0;

            // checking for bearing crossing conditions for closing and crossing gaps
            if (leftToRightAngle > M_PI && bearing_crossing_check) 
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
                        gapCrossingPt << rightCrossPt[0], rightCrossPt[1];
                    else
                        gapCrossingPt << leftCrossPt[0], leftCrossPt[1];

                    gapCrossingPt += 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * (gapCrossingPt / gapCrossingPt.norm());
                    gap.setClosingPoint(gapCrossingPt[0], gapCrossingPt[1]);
                    float ending_time = generateCrossedGapTerminalPoints(t, gap, leftGapPtModel, rightGapPtModel);
                    if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("                    considering gap closed at " << ending_time); 

                    gap.closed_ = true;
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

                        float ending_time = generateCrossedGapTerminalPoints(t, gap, leftGapPtModel, rightGapPtModel);
                        if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("                    considering gap crossed at " << ending_time); 

                        gap.crossed_ = true;
                    }
                }
            } else if (prevLeftToRightAngle > (3*M_PI / 4) && leftToRightAngle < (M_PI / 4)) 
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
                gap.crossedBehind_ = true;
            }
            
            prevLeftGapState = leftGapState;
            prevRightGapState = rightGapState;
            // prev_left_frozen_mp_state = left_frozen_mp_state;
            // prev_right_frozen_mp_state = right_frozen_mp_state;
            prevCentralBearingVect = centralBearingVect;
        }

        if (!gap.crossed_ && !gap.closed_ && !gap.crossedBehind_) 
        {
            // left_frozen_mp_state = leftGapPtModel->get_frozen_modified_polar_state();
            // right_frozen_mp_state = rightGapPtModel->get_frozen_modified_polar_state();
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
                                                                    dynamic_gap::Estimator* leftGapPtModel, 
                                                                    dynamic_gap::Estimator* rightGapPtModel) 
    {    
        // Eigen::Vector4f left_rewind_mp_state = leftGapPtModel->get_rewind_modified_polar_state();        
        // Eigen::Vector4f right_rewind_mp_state = rightGapPtModel->get_rewind_modified_polar_state();
        Eigen::Vector4f rewindLeftGapState = leftGapPtModel->getGapState();
        Eigen::Vector4f rewindRightGapState = rightGapPtModel->getGapState();        

        Eigen::Vector2f leftCrossPt, rightCrossPt; // , leftBearingVect, rightBearingVect;

        // instantiate model rewind states
        leftGapPtModel->setRewindState();
        rightGapPtModel->setRewindState();
        // do rewindPropagate
        // auto egocircle = *msg.get();        

        // float thetaLeft, thetaRight, range_left, range_right, r_min, leftToRightAngle;
        // REWINDING THE GAP FROM ITS CROSSED CONFIGURATION UNTIL THE GAP IS SUFFICIENTLY OPEN
        for (float t_rew = t; t_rew >= 0.0; t_rew -= cfg_->traj.integrate_stept) 
        {
            leftGapPtModel->rewindPropagate(-1 * cfg_->traj.integrate_stept); // resetting model we used before, not good
            rightGapPtModel->rewindPropagate(-1 * cfg_->traj.integrate_stept);
            // ROS_INFO_STREAM("t_rew: " << t_rew);
            // left_rewind_mp_state = leftGapPtModel->get_rewind_modified_polar_state();
            // right_rewind_mp_state = rightGapPtModel->get_rewind_modified_polar_state();
            rewindLeftGapState = leftGapPtModel->getRewindGapState();
            rewindRightGapState = rightGapPtModel->getRewindGapState();  
            // thetaLeft = getGapBearing(rewindLeftGapState); // left_rewind_mp_state[1];
            // thetaRight = getGapBearing(rewindRightGapState); // right_rewind_mp_state[1]; 
            // leftBearingVect << std::cos(thetaLeft), std::sin(thetaLeft);
            // rightBearingVect << std::cos(thetaRight), std::sin(thetaRight);
            // leftToRightAngle = getLeftToRightAngle(leftBearingVect, rightBearingVect);
            
            // range_left = getGapDist(rewindLeftGapState); // (1.0 / left_rewind_mp_state[0]);
            // range_right = getGapDist(rewindRightGapState); // (1.0 / right_rewind_mp_state[0]);
            // r_min = std::min(range_left, range_right);

            // // float wrapped_thetaLeft = atanThetaWrap(thetaLeft);
            // float init_left_idx = (wrapped_thetaLeft - scan_angle_min_) / scan_angle_increment_;
            // int left_idx = (int) std::floor(init_left_idx);

            // // float wrapped_term_thetaRight = atanThetaWrap(thetaRight);
            // float init_right_idx = (wrapped_term_thetaRight - scan_angle_min_) / scan_angle_increment_;
            // int right_idx = (int) std::floor(init_right_idx);

            leftCrossPt = rewindLeftGapState.head(2);
                            // range_left*std::cos(wrapped_thetaLeft), 
                            // range_left*std::sin(wrapped_thetaLeft);
            rightCrossPt = rewindRightGapState.head(2);
                            // range_right*std::cos(wrapped_term_thetaRight), 
                            // range_right*std::sin(wrapped_term_thetaRight);            

            // if gap is sufficiently open
            // option 1: arc-length:
            if (t_rew == 0 || (leftCrossPt - rightCrossPt).norm() >  2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) // r_min * leftToRightAngle > 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio
            { 
                if (cfg_->debug.feasibility_debug_log) ROS_INFO_STREAM("terminal points at time " << t_rew << ", left: (" << leftCrossPt[0] << ", " << leftCrossPt[1] << "), right: (" << rightCrossPt[0] << ", " << rightCrossPt[1]);
                generateTerminalPoints(gap, rewindLeftGapState, rewindRightGapState);
                                        // wrapped_thetaLeft, left_rewind_mp_state[0], 
                                        // wrapped_term_thetaRight, right_rewind_mp_state[0]);
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
                                                        // float terminal_thetaLeft, float terminal_reciprocal_range_left, 
                                                        // float terminal_thetaRight, float terminal_reciprocal_range_right) 
    {
        // auto egocircle = *msg.get();        
        
        float thetaLeft = getGapBearing(leftCrossPt);
        int left_idx = theta2idx(thetaLeft);
        float left_dist = getGapDist(leftCrossPt);

        // float wrapped_term_thetaLeft = atanThetaWrap(getGapBearing(leftCrossPt));
        // float init_left_idx = (wrapped_term_thetaLeft - scan_angle_min_) / scan_angle_increment_;
        // int left_idx = (int) std::floor(init_left_idx);
        // float left_dist = getGapDist(leftCrossPt); // (1.0 / terminal_reciprocal_range_left);

        // float wrapped_term_thetaRight = atanThetaWrap(getGapBearing(rightCrossPt));
        // float init_right_idx = (wrapped_term_thetaRight - scan_angle_min_) / scan_angle_increment_;
        // int right_idx = (int) std::floor(init_right_idx);
        // float right_dist = getGapDist(rightCrossPt); // (1.0 / terminal_reciprocal_range_right);
        // if (left_idx == right_idx) right_idx++;

        float thetaRight = getGapBearing(rightCrossPt);
        int right_idx = theta2idx(thetaRight);
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
