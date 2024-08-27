
#include <dynamic_gap/gap_feasibility/GapFeasibilityChecker.h>

namespace dynamic_gap 
{
    void GapFeasibilityChecker::propagateGapPoints(dynamic_gap::Gap * gap) 
    {
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                [propagateGapPoints()]");

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

        // ROS_INFO_STREAM_NAMED("GapFeasibility", "gap category: " << gap->getCategory());
        ROS_INFO_STREAM_NAMED("GapFeasibility", "starting frozen cartesian left: " << leftGapState[0] << ", " << leftGapState[1] << ", " << leftGapState[2] << ", " << leftGapState[3]); 
        ROS_INFO_STREAM_NAMED("GapFeasibility", "starting frozen cartesian right: " << rightGapState[0] << ", " << rightGapState[1] << ", " << rightGapState[2] << ", " << rightGapState[3]);
       
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
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t: " << t);

            // propagate left point
            gap->leftGapPtModel_->gapStatePropagate(cfg_->traj.integrate_stept);

            // propagate right point
            gap->rightGapPtModel_->gapStatePropagate(cfg_->traj.integrate_stept);

            leftGapState = gap->leftGapPtModel_->getGapState();
            rightGapState = gap->rightGapPtModel_->getGapState();

            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                       leftGapState: " << leftGapState.transpose());
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                       rightGapState: " << rightGapState.transpose());

            ////////////////////////////////
            // END CONDITION 0: COLLISION //
            ////////////////////////////////
            
            leftGapPtCollision = getGapRange(leftGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            rightGapPtCollision = getGapRange(rightGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            collision = (leftGapPtCollision || rightGapPtCollision);

            if (collision) 
            {
                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    end condition 0 (collision) at " << t);
                if (!gapHasCrossed)
                {
                    generateTerminalPoints(gap, leftGapState, rightGapState);
                    gap->setGapLifespan(t);
                    gap->end_condition = 0;

                    ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting gap lifespan to " << gap->gapLifespan_); 
                }

                return;
            }

            ///////////////////////////////////
            // END CONDITION 1: GAP CROSSING //
            ///////////////////////////////////
            
            thetaLeft = getGapBearing(leftGapState);
            thetaRight = getGapBearing(rightGapState);
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "thetaLeft: " << thetaLeft << ", thetaRight: " << thetaRight);
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
                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    end condition 1 (crossing) at " << t);

                gapLifespan = generateCrossedGapTerminalPoints(t, gap);
                gap->setGapLifespan(gapLifespan);
                gap->end_condition = 1;

                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting gap lifespan to " << gap->gapLifespan_); 

                return;
            }
            
            //////////////////////////////////////
            // END CONDITION 2: GAP OVERLAPPING //
            //////////////////////////////////////

            prevLeftBearingVect = prevLeftGapState.head(2) / getGapRange(prevLeftGapState); // << std::cos(prev_thetaLeft), std::sin(prev_thetaLeft);
            prevRightBearingVect = prevRightGapState.head(2) / getGapRange(prevRightGapState); // << std::cos(prev_thetaRight), std::sin(prev_thetaRight);
            prevLeftToRightAngle = getSweptLeftToRightAngle(prevLeftBearingVect, prevRightBearingVect);            

            // ROS_INFO_STREAM_NAMED("GapFeasibility", "prevLeftToRightAngle: " << prevLeftToRightAngle << ", leftToRightAngle: " << leftToRightAngle);            
            
            if (prevLeftToRightAngle > (3*M_PI / 4) && leftToRightAngle < (M_PI / 4)) 
            {
                // checking for case of gap crossing behind the robot
                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    end condition 2 (overlapping) at " << t);

                generateTerminalPoints(gap, prevLeftGapState, prevRightGapState); // prev_left_frozen_mp_state[1], prev_left_frozen_mp_state[0], prev_right_frozen_mp_state[1], prev_right_frozen_mp_state[0]);
                gap->setGapLifespan(t);
                gap->end_condition = 2;

                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting gap lifespan to " << gap->gapLifespan_); 

                return;
            }
            
            prevLeftGapState = leftGapState;
            prevRightGapState = rightGapState;
            prevCentralBearingVect = centralBearingVect;
        }

        ////////////////////////////////
        // END CONDITION 3: TIMED OUT //
        ////////////////////////////////

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                    end condition 3 (time out) at " << cfg_->traj.integrate_maxt);


        generateTerminalPoints(gap, leftGapState, rightGapState);
        gap->setGapLifespan(cfg_->traj.integrate_maxt);
        gap->end_condition = 3;
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting gap lifespan to " << gap->gapLifespan_); 

        return;
    }

    float GapFeasibilityChecker::generateCrossedGapTerminalPoints(const float & t, dynamic_gap::Gap * gap) 
    {    
        // ROS_INFO_STREAM_NAMED("GapFeasibility", "                   [generateCrossedGapTerminalPoints()]");

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
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                       tRewind: " << tRewind);

            // Rewind left gap point
            gap->leftGapPtModel_->rewindPropagate(-1 * cfg_->traj.integrate_stept); // resetting model we used before, not good
            
            // Rewind right gap point
            gap->rightGapPtModel_->rewindPropagate(-1 * cfg_->traj.integrate_stept);

            rewindLeftGapState = gap->leftGapPtModel_->getRewindGapState();
            rewindRightGapState = gap->rightGapPtModel_->getRewindGapState();  


            leftRewindPt = rewindLeftGapState.head(2);
            rightRewindPt = rewindRightGapState.head(2);           

            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                           rewindLeftGapState: " << rewindLeftGapState.transpose());
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                           rewindRightGapState: " << rewindRightGapState.transpose());


            leftBearingVect = rewindLeftGapState.head(2) / getGapRange(rewindLeftGapState);
            rightBearingVect = rewindRightGapState.head(2) / getGapRange(rewindRightGapState);

            leftToRightAngle = getSweptLeftToRightAngle(leftBearingVect, rightBearingVect);

            // if gap is sufficiently open
            // option 1: arc-length:
            if (tRewind == 0 || 
                (leftToRightAngle < M_PI &&
                 (leftRewindPt - rightRewindPt).norm() >  2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio))
            { 
                // ROS_INFO_STREAM_NAMED("GapFeasibility", "                    terminal points at time " << t_rew << ", left: (" << leftCrossPt[0] << ", " << leftCrossPt[1] << "), right: (" << rightCrossPt[0] << ", " << rightCrossPt[1]);
                generateTerminalPoints(gap, rewindLeftGapState, rewindRightGapState);
                return tRewind;
            }
            
        }

        // if falls out at t=0, just set terminal points equal to initial points? 
        // Lifespan would be 0, so would be infeasible anyways

        // should never fall out of this?
        return 0.0;
    }

    void GapFeasibilityChecker::generateTerminalPoints(dynamic_gap::Gap * gap, 
                                                        const Eigen::Vector4f & leftCrossPt,
                                                        const Eigen::Vector4f & rightCrossPt)
    {        
        float thetaLeft = getGapBearing(leftCrossPt);
        int idxLeft = theta2idx(thetaLeft);
        float rangeLeft = getGapRange(leftCrossPt);

        float thetaRight = getGapBearing(rightCrossPt);
        int idxRight = theta2idx(thetaRight);
        float rangeRight = getGapRange(rightCrossPt);

        gap->setTerminalPoints(idxLeft, rangeLeft, idxRight, rangeRight);

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting terminal points (polar) to, left: (" << 
                                                idxLeft << ", " << rangeLeft << "), right: (" << 
                                                idxRight << ", " << rangeRight << ")");


        ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting terminal points (cart) to, left: (" << 
                                                leftCrossPt[0] << ", " << leftCrossPt[1] << "), right: (" << 
                                                rightCrossPt[0] << ", " << rightCrossPt[1] << ")");
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
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                [purePursuitFeasibilityCheck()]"); 
        // calculate intercept angle and intercept time using center point (gap goal)

        gap->leftGapPtModel_->isolateGapDynamics();
        gap->rightGapPtModel_->isolateGapDynamics();

        Eigen::Vector4f leftGapState = gap->leftGapPtModel_->getGapState();
        Eigen::Vector4f rightGapState = gap->rightGapPtModel_->getGapState();

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       leftGapState: " << leftGapState.transpose()); 
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       rightGapState: " << rightGapState.transpose()); 
    
        float t_intercept_left = 0.0;
        purePursuitHelper(leftGapState.head(2), 
                            leftGapState.tail(2), 
                            cfg_->rbt.vx_absmax,
                            t_intercept_left);

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_left: " << t_intercept_left); 

        float t_intercept_right = 0.0;
        purePursuitHelper(rightGapState.head(2), 
                            rightGapState.tail(2), 
                            cfg_->rbt.vx_absmax,
                            t_intercept_right);

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_right: " << t_intercept_right); 

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

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_goal: " << t_intercept_goal); 


        if ((gap->end_condition == 0 || gap->end_condition == 1)
            && t_intercept_goal > gap->gapLifespan_)
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "                    gap is not feasible! t_intercept: " << t_intercept_goal << ", gap lifespan: " << gap->gapLifespan_); 
            return false;
        } else
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "                    gap is feasible! t_intercept: " << t_intercept_goal << ", gap lifespan: " << gap->gapLifespan_); 
            // set t_intercept
            gap->t_intercept = t_intercept_goal;

            Eigen::Vector2f terminalGoal = p_target + v_target * gap->t_intercept;
            gap->setTerminalGoal(terminalGoal);

            return true;            
        }
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
            ROS_INFO_STREAM_NAMED("GapFeasibility", "                           static gap point"); 

            t_intercept = p_target.norm() / speed_robot;
        } else
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "                           dynamic gap point"); 

            float K = epsilonDivide(speed_robot, v_target.norm()); // just set to one dimensional norm

            ROS_INFO_STREAM_NAMED("GapFeasibility", "                               K: " << K); 

            assert(K > 1);

            Eigen::Vector2f n_lambda(std::cos(lambda), std::sin(lambda));
            Eigen::Vector2f n_gamma(std::cos(gamma), std::sin(gamma));

            float theta = getSignedLeftToRightAngle(n_gamma, n_lambda);

            t_intercept = (p_target.norm() / v_target.norm()) * (epsilonDivide(K + std::cos(theta), (K * K - 1)));
        }
    }

    bool GapFeasibilityChecker::parallelNavigationFeasibilityCheck(dynamic_gap::Gap * gap)
    {
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                [parallelNavigationFeasibilityCheck()]"); 
        // calculate intercept angle and intercept time using center point (gap goal)

        gap->leftGapPtModel_->isolateGapDynamics();
        gap->rightGapPtModel_->isolateGapDynamics();

        Eigen::Vector4f leftGapState = gap->leftGapPtModel_->getGapState();
        Eigen::Vector4f rightGapState = gap->rightGapPtModel_->getGapState();

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       leftGapState: " << leftGapState.transpose()); 
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       rightGapState: " << rightGapState.transpose()); 

        float t_intercept_left = 0.0;
        float gamma_intercept_left = 0.0;
        parallelNavigationHelper(leftGapState.head(2), 
                                    leftGapState.tail(2), 
                                    cfg_->rbt.vx_absmax,
                                    t_intercept_left, 
                                    gamma_intercept_left);

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_left: " << t_intercept_left); 
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gamma_intercept_left: " << gamma_intercept_left); 

        float t_intercept_right = 0.0;
        float gamma_intercept_right = 0.0;
        parallelNavigationHelper(rightGapState.head(2), 
                                    rightGapState.tail(2), 
                                    cfg_->rbt.vx_absmax,
                                    t_intercept_right, 
                                    gamma_intercept_right);

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_right: " << t_intercept_right); 
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gamma_intercept_right: " << gamma_intercept_right); 

        // set target position to gap goal
        Eigen::Vector2f p_target(gap->goal.x_, gap->goal.y_);

        // set target velocity to mean of left and right gap points
        Eigen::Vector2f v_target = (leftGapState.tail(2) + rightGapState.tail(2)) / 2.;

        float t_intercept_goal = 0.0;
        float gamma_intercept_goal = 0.0;
        parallelNavigationHelper(p_target, 
                                    v_target, 
                                    cfg_->rbt.vx_absmax,
                                    t_intercept_goal, 
                                    gamma_intercept_goal);

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       t_intercept_goal: " << t_intercept_goal); 
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gamma_intercept_goal: " << gamma_intercept_goal); 

        if (isnan(t_intercept_goal) || isnan(gamma_intercept_goal)) // can happen if K < 1
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "                    gap is not feasible! t_intercept: " << t_intercept_goal << ", gap lifespan: " << gap->gapLifespan_); 
            return false;
        } else if ((gap->end_condition == 0 || gap->end_condition == 1)
            && t_intercept_goal > gap->gapLifespan_)
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "                    gap is not feasible! t_intercept: " << t_intercept_goal << ", gap lifespan: " << gap->gapLifespan_); 
            return false;
        } else
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "                    gap is feasible! t_intercept: " << t_intercept_goal << ", gap lifespan: " << gap->gapLifespan_); 
            // set t_intercept
            gap->t_intercept = t_intercept_goal;
            // set gamma_rbt
            gap->gamma_intercept = gamma_intercept_goal;

            // set gamma left
            gap->gamma_intercept_left = gamma_intercept_left;

            // set gamma right
            gap->gamma_intercept_right = gamma_intercept_right;

            Eigen::Vector2f terminalGoal = p_target + v_target * gap->t_intercept;
            gap->setTerminalGoal(terminalGoal);

            return true;            
        }
    }

    void GapFeasibilityChecker::parallelNavigationHelper(const Eigen::Vector2f & p_target, 
                                                            const Eigen::Vector2f & v_target, 
                                                            const float speed_robot,
                                                            float & t_intercept, 
                                                            float & gamma_intercept)
    {
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                       [parallelNavigationHelper()]"); 

        // float eps = 0.00001;

        float lambda = std::atan2(p_target[1], p_target[0]);
        float gamma = std::atan2(v_target[1], v_target[0]);

        if (v_target.norm() < eps) // static gap point
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "                           static gap point"); 

            t_intercept = p_target.norm() / speed_robot;
            gamma_intercept = lambda;
        } else // moving gap point
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "                           dynamic gap point");

            float K = epsilonDivide(speed_robot, v_target.norm()); // just set to one dimensional norm

            assert(K > 0);

            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                           K: " << K);

            Eigen::Vector2f n_lambda(std::cos(lambda), std::sin(lambda));
            Eigen::Vector2f n_gamma(std::cos(gamma), std::sin(gamma));

            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                           n_lambda: " << n_lambda.transpose());
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                           n_gamma: " << n_gamma.transpose());

            float theta = getSignedLeftToRightAngle(n_gamma, n_lambda);

            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                           theta: " << theta);

            float delta = std::asin( epsilonDivide(std::sin(theta), K));

            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                           delta: " << delta);

            t_intercept = (p_target.norm() / v_target.norm()) * (epsilonDivide(1.0, (K * std::cos(delta) - std::cos(theta))));

            gamma_intercept = lambda + delta;
        
        }

        if (isnan(t_intercept))
        {
            ROS_WARN_STREAM_NAMED("GapFeasibility", "                           t_interecept is nan!"); 
        }

        if (isnan(gamma_intercept))
        {
            ROS_WARN_STREAM_NAMED("GapFeasibility", "                           gamma_intercept is nan!"); 
        }

        return;
    }

}
