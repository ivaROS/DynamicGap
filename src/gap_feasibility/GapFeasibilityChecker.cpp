
#include <dynamic_gap/gap_feasibility/GapFeasibilityChecker.h>

namespace dynamic_gap 
{
    void GapFeasibilityChecker::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        scan_ = scan;
    }

    void GapFeasibilityChecker::propagateGapPoints(dynamic_gap::Gap * gap) 
    {
        ROS_INFO_STREAM("                [propagateGapPoints()]");

        Eigen::Vector2f crossingPt(0.0, 0.0);

        gap->leftGapPtModel_->isolateGapDynamics();
        gap->rightGapPtModel_->isolateGapDynamics();

        float thetaLeft = idx2theta(gap->LIdx());
        float thetaRight = idx2theta(gap->RIdx());

        Eigen::Vector2f leftBearingVect(cos(thetaLeft), sin(thetaLeft)); 
        Eigen::Vector2f rightBearingVect(cos(thetaRight), sin(thetaRight));

        float leftToRightAngle = getSweptLeftToRightAngle(leftBearingVect, rightBearingVect);
        
        Eigen::Vector2f prevLeftBearingVect = leftBearingVect;        
        Eigen::Vector2f prevRightBearingVect = rightBearingVect;

        float thetaCenter = (thetaLeft - (leftToRightAngle / 2.0));

        float prevLeftToRightAngle = leftToRightAngle;
        
        Eigen::Vector2f centralBearingVect(std::cos(thetaCenter), std::sin(thetaCenter));
        
        //std::cout << "initial beta left: (" << leftBearingVect[0] << ", " << leftBearingVect[1] << "), initial beta right: (" << rightBearingVect[0] << ", " << rightBearingVect[1] << "), initial beta center: (" << centralBearingVect[0] << ", " << centralBearingVect[1] << ")" << std::endl;
        
        Eigen::Vector4f leftGapState = gap->leftGapPtModel_->getGapState();
        Eigen::Vector4f rightGapState = gap->rightGapPtModel_->getGapState();

        // ROS_INFO_STREAM("gap category: " << gap->getCategory());
        ROS_INFO_STREAM("starting frozen cartesian left: " << leftGapState[0] << ", " << leftGapState[1] << ", " << leftGapState[2] << ", " << leftGapState[3]); 
        ROS_INFO_STREAM("starting frozen cartesian right: " << rightGapState[0] << ", " << rightGapState[1] << ", " << rightGapState[2] << ", " << rightGapState[3]);
       
        float leftBearingDotCentBearing = 0.0, rightBearingDotCentBearing = 0.0;
        bool gapHasCrossed = false;
        bool bearingsCrossed = false, crossedGapPtsDistCheck = false;    

        bool leftSideOpening = (getGapBearingRateOfChange(leftGapState) > 0.0);
        bool rightSideOpening = (getGapBearingRateOfChange(rightGapState) < 0.0);
        Eigen::Vector4f prevLeftGapState = leftGapState;
        Eigen::Vector4f prevRightGapState = rightGapState;       
        Eigen::Vector2f prevCentralBearingVect = centralBearingVect;

        Eigen::Vector2f leftCrossPt(0.0, 0.0);
        Eigen::Vector2f rightCrossPt(0.0, 0.0);
        bool leftGapPtCollision = false, rightGapPtCollision = false, collision = false;
        float gapLifespan = 0.0;
        for (float t = cfg_->traj.integrate_stept; t < cfg_->traj.integrate_maxt; t += cfg_->traj.integrate_stept) 
        {
            // ROS_INFO_STREAM("                       t: " << t);

            // propagate left point
            gap->leftGapPtModel_->gapStatePropagate(cfg_->traj.integrate_stept);

            // propagate right point
            gap->rightGapPtModel_->gapStatePropagate(cfg_->traj.integrate_stept);

            leftGapState = gap->leftGapPtModel_->getGapState();
            rightGapState = gap->rightGapPtModel_->getGapState();

            // ROS_INFO_STREAM("                       leftGapState: " << leftGapState.transpose());
            // ROS_INFO_STREAM("                       rightGapState: " << rightGapState.transpose());

            ////////////////////////////////
            // END CONDITION 0: COLLISION //
            ////////////////////////////////
            
            // leftGapPtCollision = getGapRange(leftGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            // rightGapPtCollision = getGapRange(rightGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            // collision = (leftGapPtCollision || rightGapPtCollision);

            // if (collision) 
            // {
            //     ROS_INFO_STREAM("                    end condition 0 (collision) at " << t);
            //     if (!gapHasCrossed)
            //     {
            //         generateTerminalPoints(gap, leftGapState, rightGapState);
            //         gap->setGapLifespan(t);
            //         gap->end_condition = 0;

            //         ROS_INFO_STREAM("                    setting gap lifespan to " << gap->gapLifespan_); 
            //     }

            //     return;
            // }

            ///////////////////////////////////
            // END CONDITION 1: GAP CROSSING //
            ///////////////////////////////////
            
            thetaLeft = getGapBearing(leftGapState);
            thetaRight = getGapBearing(rightGapState);
            // ROS_INFO_STREAM("thetaLeft: " << thetaLeft << ", thetaRight: " << thetaRight);
            leftBearingVect = leftGapState.head(2) / getGapRange(leftGapState); // << std::cos(thetaLeft), std::sin(thetaLeft);
            rightBearingVect = rightGapState.head(2) / getGapRange(rightGapState); // << std::cos(thetaRight), std::sin(thetaRight);
            leftToRightAngle = getSweptLeftToRightAngle(leftBearingVect, rightBearingVect);
            thetaCenter = (thetaLeft - 0.5 * leftToRightAngle);

            centralBearingVect << std::cos(thetaCenter), std::sin(thetaCenter);
        
            leftBearingDotCentBearing = leftBearingVect.dot(prevCentralBearingVect);
            rightBearingDotCentBearing = rightBearingVect.dot(prevCentralBearingVect);
            bearingsCrossed = leftBearingDotCentBearing > 0.0 && rightBearingDotCentBearing > 0.0;

            // checking for bearing crossing conditions for closing and crossing gaps
            if (leftToRightAngle > M_PI && bearingsCrossed) 
            {
                ROS_INFO_STREAM("                    end condition 1 (crossing) at " << t);

                gapLifespan = rewindGapPoints(t, gap);
                gap->setGapLifespan(gapLifespan);
                gap->end_condition = 1;

                ROS_INFO_STREAM("                    setting gap lifespan to " << gap->gapLifespan_); 

                return;
            }
            
            //////////////////////////////////////
            // END CONDITION 2: GAP OVERLAPPING //
            //////////////////////////////////////

            // ROS_INFO_STREAM("prevLeftToRightAngle: " << prevLeftToRightAngle << ", leftToRightAngle: " << leftToRightAngle);            
            
            if (leftToRightAngle < M_PI && leftBearingDotCentBearing < 0.0 && rightBearingDotCentBearing < 0.0) 
            {
                // checking for case of gap crossing behind the robot
                ROS_INFO_STREAM("                    end condition 2 (overlapping) at " << t);

                gap->setGapLifespan(t - cfg_->traj.integrate_stept);
                gap->end_condition = 2;

                ROS_INFO_STREAM("                    setting gap lifespan to " << gap->gapLifespan_); 

                return;
            }
            
            prevLeftGapState = leftGapState;
            prevRightGapState = rightGapState;
            prevCentralBearingVect = centralBearingVect;
        }

        ////////////////////////////////
        // END CONDITION 3: TIMED OUT //
        ////////////////////////////////

        ROS_INFO_STREAM("                    end condition 3 (time out) at " << cfg_->traj.integrate_maxt);

        gap->setGapLifespan(cfg_->traj.integrate_maxt);
        gap->end_condition = 3;
        ROS_INFO_STREAM("                    setting gap lifespan to " << gap->gapLifespan_); 

        return;
    }

    float GapFeasibilityChecker::rewindGapPoints(const float & t, dynamic_gap::Gap * gap) 
    {    
        // ROS_INFO_STREAM("                   [rewindGapPoints()]");

        Eigen::Vector4f rewindLeftGapState = gap->leftGapPtModel_->getGapState();
        Eigen::Vector4f rewindRightGapState = gap->rightGapPtModel_->getGapState();        

        Eigen::Vector2f leftRewindPt, rightRewindPt, leftBearingVect, rightBearingVect;
        float leftToRightAngle = 0.0;

        // instantiate model rewind states
        gap->leftGapPtModel_->setRewindState();
        gap->rightGapPtModel_->setRewindState();
        // do rewindPropagate

        bool leftSideOpening = (getGapBearingRateOfChange(rewindLeftGapState) > 0.0);
        bool rightSideOpening = (getGapBearingRateOfChange(rewindRightGapState) < 0.0);

        // REWINDING THE GAP FROM ITS CROSSED CONFIGURATION UNTIL THE GAP IS SUFFICIENTLY OPEN
        for (float tRewind = (t - cfg_->traj.integrate_stept); tRewind >= 0.0; tRewind -= cfg_->traj.integrate_stept) 
        {
            // ROS_INFO_STREAM("                       tRewind: " << tRewind);

            // Rewind left gap point
            gap->leftGapPtModel_->rewindPropagate(-1 * cfg_->traj.integrate_stept); // resetting model we used before, not good
            
            // Rewind right gap point
            gap->rightGapPtModel_->rewindPropagate(-1 * cfg_->traj.integrate_stept);

            rewindLeftGapState = gap->leftGapPtModel_->getRewindGapState();
            rewindRightGapState = gap->rightGapPtModel_->getRewindGapState();  

            leftRewindPt = rewindLeftGapState.head(2);
            rightRewindPt = rewindRightGapState.head(2);           

            // ROS_INFO_STREAM("                           rewindLeftGapState: " << rewindLeftGapState.transpose());
            // ROS_INFO_STREAM("                           rewindRightGapState: " << rewindRightGapState.transpose());


            leftBearingVect = rewindLeftGapState.head(2) / getGapRange(rewindLeftGapState);
            rightBearingVect = rewindRightGapState.head(2) / getGapRange(rewindRightGapState);

            leftToRightAngle = getSweptLeftToRightAngle(leftBearingVect, rightBearingVect);

            float r_min = std::min( leftRewindPt.norm(), rightRewindPt.norm() );
            Eigen::Vector2f rewindLeftGapStateStar = r_min * leftBearingVect;
            Eigen::Vector2f rewindRightGapStateStar = r_min * rightBearingVect;

            // if gap is sufficiently open
            // option 1: arc-length:
            if (tRewind == 0 || 
                (leftToRightAngle < M_PI &&
                 (rewindLeftGapStateStar - rewindRightGapStateStar).norm() >  2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio))
            { 
                // ROS_INFO_STREAM("                    terminal points at time " << t_rew << ", left: (" << leftCrossPt[0] << ", " << leftCrossPt[1] << "), right: (" << rightCrossPt[0] << ", " << rightCrossPt[1]);
                return tRewind;
            }
            
        }

        // if falls out at t=0, just set terminal points equal to initial points? 
        // Lifespan would be 0, so would be infeasible anyways

        // should never fall out of this?
        return 0.0;
    }

    bool GapFeasibilityChecker::pursuitGuidanceAnalysis(dynamic_gap::Gap * gap)
    {
        // check what method we are using
        if (cfg_->planning.pursuit_guidance_method == 0)
        {
            // pure pursuit
            return purePursuitFeasibilityCheck(gap);
        } else
        {
            // parallel navigation
            return parallelNavigationFeasibilityCheck(gap);
        }
    }
    
    bool GapFeasibilityChecker::purePursuitFeasibilityCheck(dynamic_gap::Gap * gap)
    {
        ROS_INFO_STREAM("                [purePursuitFeasibilityCheck()]"); 
        // calculate intercept angle and intercept time using center point (gap goal)

        throw std::runtime_error("Should not be using pure pursuit");

        return false;

        /*

        gap->leftGapPtModel_->isolateGapDynamics();
        gap->rightGapPtModel_->isolateGapDynamics();

        Eigen::Vector4f leftGapState = gap->leftGapPtModel_->getGapState();
        Eigen::Vector4f rightGapState = gap->rightGapPtModel_->getGapState();

        ROS_INFO_STREAM("                       leftGapState: " << leftGapState.transpose()); 
        ROS_INFO_STREAM("                       rightGapState: " << rightGapState.transpose()); 
    
        float t_intercept_left = 0.0;
        purePursuitHelper(leftGapState.head(2), 
                            leftGapState.tail(2), 
                            cfg_->rbt.vx_absmax,
                            t_intercept_left);

        ROS_INFO_STREAM("                       t_intercept_left: " << t_intercept_left); 

        float t_intercept_right = 0.0;
        purePursuitHelper(rightGapState.head(2), 
                            rightGapState.tail(2), 
                            cfg_->rbt.vx_absmax,
                            t_intercept_right);

        ROS_INFO_STREAM("                       t_intercept_right: " << t_intercept_right); 

        // set target position to gap goal
        Eigen::Vector2f p_target(gap->goal.x_, gap->goal.y_);

        // set target velocity to mean of left and right gap points
        Eigen::Vector2f v_target = (leftGapState.tail(2) + rightGapState.tail(2)) / 2.;

        float speed_robot = cfg_->rbt.vx_absmax;
        float K = epsilonDivide(speed_robot, v_target.norm()); // just set to one dimensional norm

        // if K <= 1, reject
        if (K <= 1)
            return false;

        float t_intercept_goal = 0.0;
        float gamma_intercept_goal = 0.0;
        purePursuitHelper(p_target, 
                            v_target, 
                            cfg_->rbt.vx_absmax,
                            t_intercept_goal);

        ROS_INFO_STREAM("                       t_intercept_goal: " << t_intercept_goal); 


        if (!gap->rgc_ && 
            (gap->end_condition == 0 || gap->end_condition == 1) && 
            t_intercept_goal > gap->gapLifespan_)
        {
            ROS_INFO_STREAM("                    gap is not feasible! t_intercept: " << t_intercept_goal << ", gap lifespan: " << gap->gapLifespan_); 
            return false;
        } else
        {
            ROS_INFO_STREAM("                    gap is feasible! t_intercept: " << t_intercept_goal << ", gap lifespan: " << gap->gapLifespan_); 
            // set t_intercept
            gap->t_intercept = t_intercept_goal;

            Eigen::Vector2f terminalGoal = p_target + v_target * gap->t_intercept;
            gap->setTerminalGoal(terminalGoal);

            return true;            
        }
        */
    }

    void GapFeasibilityChecker::purePursuitHelper(const Eigen::Vector2f & p_target, 
                                                    const Eigen::Vector2f & v_target, 
                                                    const float speed_robot,
                                                    float & t_intercept)
    {
        float lambda = std::atan2(p_target[1], p_target[0]);
        float gamma = std::atan2(v_target[1], v_target[0]);

        if (v_target.norm() < eps) // static gap point
        {
            ROS_INFO_STREAM("                           static gap point"); 

            t_intercept = p_target.norm() / speed_robot;
        } else
        {
            ROS_INFO_STREAM("                           dynamic gap point"); 

            float K = epsilonDivide(speed_robot, v_target.norm()); // just set to one dimensional norm

            ROS_INFO_STREAM("                               K: " << K); 

            // assert(K > 1);

            Eigen::Vector2f n_lambda(std::cos(lambda), std::sin(lambda));
            Eigen::Vector2f n_gamma(std::cos(gamma), std::sin(gamma));

            float theta = getSignedLeftToRightAngle(n_gamma, n_lambda);

            t_intercept = (p_target.norm() / v_target.norm()) * (epsilonDivide(K + std::cos(theta), (K * K - 1)));
        }
    }

    bool GapFeasibilityChecker::parallelNavigationFeasibilityCheck(dynamic_gap::Gap * gap)
    {
        ROS_INFO_STREAM("                [parallelNavigationFeasibilityCheck()]"); 

        gap->leftGapPtModel_->isolateGapDynamics();
        gap->rightGapPtModel_->isolateGapDynamics();

        Eigen::Vector4f leftGapState = gap->leftGapPtModel_->getGapState();
        Eigen::Vector4f rightGapState = gap->rightGapPtModel_->getGapState();

        // Eigen::Vector2f leftGapPtPos = leftGapState.head(2);
        Eigen::Vector2f leftGapPtVel = leftGapState.tail(2);

        // Eigen::Vector2f rightGapPtPos = rightGapState.head(2);
        Eigen::Vector2f rightGapPtVel = rightGapState.tail(2);

        // ROS_INFO_STREAM("                       leftGapState: " << leftGapState.transpose()); 

        // ROS_INFO_STREAM("                       rightGapState: " << rightGapState.transpose()); 

        ///////////////////////////////
        // Evaluating left gap point //
        ///////////////////////////////
        
        float t_intercept_left = 0.0;
        float gamma_intercept_left = 0.0;
        parallelNavigationHelper(gap->getManipulatedLPosition(), 
                                    leftGapPtVel, 
                                    cfg_->rbt.vx_absmax,
                                    t_intercept_left, 
                                    gamma_intercept_left);

        ROS_INFO_STREAM("                       t_intercept_left: " << t_intercept_left); 
        ROS_INFO_STREAM("                       gamma_intercept_left: " << gamma_intercept_left); 

        gap->t_intercept_left = t_intercept_left;
        gap->gamma_intercept_left = gamma_intercept_left;

        ///////////////////////////////
        // Evaluating right gap point //
        ///////////////////////////////

        float t_intercept_right = 0.0;
        float gamma_intercept_right = 0.0;
        parallelNavigationHelper(gap->getManipulatedRPosition(), 
                                    rightGapPtVel, 
                                    cfg_->rbt.vx_absmax,
                                    t_intercept_right, 
                                    gamma_intercept_right);

        ROS_INFO_STREAM("                       t_intercept_right: " << t_intercept_right); 
        ROS_INFO_STREAM("                       gamma_intercept_right: " << gamma_intercept_right); 

        gap->t_intercept_right = t_intercept_right;
        gap->gamma_intercept_right = gamma_intercept_right;

        // set target position to gap goal
        Eigen::Vector2f p_target(gap->goal.x_, gap->goal.y_);

        // set target velocity to mean of left and right gap points
        Eigen::Vector2f v_target(gap->goal.vx_, gap->goal.vy_);

        ROS_INFO_STREAM("                       p_target: " << p_target.transpose()); 
        ROS_INFO_STREAM("                       v_target: " << v_target.transpose()); 

        float t_intercept_goal = 0.0;
        float gamma_intercept_goal = 0.0;
        parallelNavigationHelper(p_target, 
                                    v_target, 
                                    cfg_->rbt.vx_absmax,
                                    t_intercept_goal, 
                                    gamma_intercept_goal);

        ROS_INFO_STREAM("                       t_intercept_goal: " << t_intercept_goal); 
        ROS_INFO_STREAM("                       gamma_intercept_goal: " << gamma_intercept_goal); 

        if (isnan(t_intercept_goal) || isnan(gamma_intercept_goal)) // can happen if K < 1
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "                    gap is not feasible! t_intercept: " << t_intercept_goal << ", gap lifespan: " << gap->gapLifespan_); 
            return false;
        } else if (!gap->rgc_ && 
                    gap->end_condition == 1 && 
                    t_intercept_goal > gap->gapLifespan_)
        {
            ROS_INFO_STREAM("                    gap is not feasible!");
            ROS_INFO_STREAM("                       t_intercept_goal: " << t_intercept_goal);
            ROS_INFO_STREAM("                       gap lifespan: " << gap->gapLifespan_); 
            ROS_INFO_STREAM("                       gap->end_condition: " << gap->end_condition); 

            return false;
        } else
        {
            ROS_INFO_STREAM("                    gap is feasible! t_intercept_left: " 
                                                                << t_intercept_left << 
                                                                ", t_intercept_right: " << 
                                                                t_intercept_right << 
                                                                ", t_intercept_goal: " << 
                                                                t_intercept_goal << 
                                                                ", gap lifespan: " << gap->gapLifespan_); 
            
            // How to handle situation where t_intercept is negative?

            Eigen::Vector2f terminalGoal;
            if (t_intercept_goal < 0) //      Can happen when K < 0
            {
                terminalGoal = p_target;
            } else if (t_intercept_goal > gap->gapLifespan_) //      Can happen when ego-robot not fast enough
            {
                terminalGoal = p_target + v_target * gap->gapLifespan_;
            } else
            {
                terminalGoal = p_target + v_target * t_intercept_goal;
            }

            // clip at scan
            float terminalGoalTheta = std::atan2(terminalGoal[1], terminalGoal[0]);
            int terminalGoalScanIdx = theta2idx(terminalGoalTheta);

            // if terminal goal lives beyond scan
            if (scan_->ranges.at(terminalGoalScanIdx) < (terminalGoal.norm() + cfg_->traj.max_pose_to_scan_dist))
            {
                float newTerminalGoalRange = scan_->ranges.at(terminalGoalScanIdx) - cfg_->traj.max_pose_to_scan_dist;
                terminalGoal << newTerminalGoalRange * std::cos(terminalGoalTheta),
                                newTerminalGoalRange * std::sin(terminalGoalTheta);
            }

            // set gamma_rbt
            gap->t_intercept_goal = t_intercept_goal;
            gap->gamma_intercept_goal = terminalGoalTheta;

            gap->setTerminalGoal(terminalGoal);

            return true;            
        }

    }

    void GapFeasibilityChecker::parallelNavigationHelper(const Eigen::Vector2f & p_target_init, 
                                                            const Eigen::Vector2f & v_target_init, 
                                                            const float speed_robot,
                                                            float & t_intercept, 
                                                            float & gamma_intercept)
    {
        ROS_INFO_STREAM("                       [parallelNavigationHelper()]"); 

        // float eps = 0.00001;

        float scaling_factor = 0.99;

        Eigen::Vector2f p_target = p_target_init;
        Eigen::Vector2f v_target = v_target_init;

        // t_intercept = std::numeric_limits<float>::quiet_NaN();
        // gamma_intercept = std::numeric_limits<float>::quiet_NaN();

        // while ( isnan(t_intercept) )
        // {
        float lambda = std::atan2(p_target[1], p_target[0]);
        float gamma = std::atan2(v_target[1], v_target[0]);

        if (v_target.norm() < eps) // static gap point
        {
            ROS_INFO_STREAM("                           static gap point"); 

            t_intercept = p_target.norm() / speed_robot;
            gamma_intercept = lambda;
        } else // moving gap point
        {
            ROS_INFO_STREAM("                           dynamic gap point");

            float K = epsilonDivide(speed_robot, v_target.norm()); // just set to one dimensional norm

            if (K < 0)
            {
                ROS_INFO_STREAM("                       something has gone very wrong");
            }

            ROS_INFO_STREAM("                           K: " << K);

            Eigen::Vector2f n_lambda(std::cos(lambda), std::sin(lambda));
            Eigen::Vector2f n_gamma(std::cos(gamma), std::sin(gamma));

            ROS_INFO_STREAM("                           n_lambda: " << n_lambda.transpose());
            ROS_INFO_STREAM("                           n_gamma: " << n_gamma.transpose());

            float theta = getSignedLeftToRightAngle(n_gamma, n_lambda);

            ROS_INFO_STREAM("                           theta: " << theta);

            float delta = std::asin( epsilonDivide(std::sin(theta), K));

            ROS_INFO_STREAM("                           delta: " << delta);

            t_intercept = (p_target.norm() / v_target.norm()) * (epsilonDivide(1.0, (K * std::cos(delta) - std::cos(theta))));

            gamma_intercept = lambda + delta;

            // if ( isnan(t_intercept) )
            // {
            //     ROS_INFO_STREAM("                               t_intercept was nan! scaling down: ");
            //     p_target *= scaling_factor; // iteratively lessen and lessen until we can intercept
            //     v_target *= scaling_factor; // iteratively lessen and lessen until we can intercept
            // }
        }
        // }

        // if (isnan(t_intercept))
        // {
        //     ROS_WARN_STREAM_NAMED("GapFeasibility", "                           t_interecept is nan!"); 
        // }

        // if (isnan(gamma_intercept))
        // {
        //     ROS_WARN_STREAM_NAMED("GapFeasibility", "                           gamma_intercept is nan!"); 
        // }

        return;
    }

}
