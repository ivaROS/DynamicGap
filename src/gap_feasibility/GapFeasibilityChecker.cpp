
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
        // ROS_INFO_STREAM_NAMED("GapFeasibility", "starting frozen cartesian left: " << leftGapState[0] << ", " << leftGapState[1] << ", " << leftGapState[2] << ", " << leftGapState[3]); 
        // ROS_INFO_STREAM_NAMED("GapFeasibility", "starting frozen cartesian right: " << rightGapState[0] << ", " << rightGapState[1] << ", " << rightGapState[2] << ", " << rightGapState[3]);
       
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

            // checking to see if left point is reachable
            if (!(leftSideOpening && (getGapRange(leftGapState) < cfg_->rbt.vx_absmax * t))) 
            {
                // ROS_INFO_STREAM_NAMED("GapFeasibility", "propagating left");
                gap->leftGapPtModel_->gapStatePropagate(cfg_->traj.integrate_stept);
            }

            // checking to see if right point is reachable
            if (!(rightSideOpening && (getGapRange(rightGapState) < cfg_->rbt.vx_absmax * t))) 
            {
                // ROS_INFO_STREAM_NAMED("GapFeasibility", "propagating right");
                gap->rightGapPtModel_->gapStatePropagate(cfg_->traj.integrate_stept);
            }
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "t: " << t);

            leftGapState = gap->leftGapPtModel_->getGapState();
            rightGapState = gap->rightGapPtModel_->getGapState();

            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                       leftGapState: " << leftGapState.transpose());
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "                       rightGapState: " << rightGapState.transpose());

            ////////////////////////////////
            // END CONDITION 1: COLLISION //
            ////////////////////////////////
            
            leftGapPtCollision = getGapRange(leftGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            rightGapPtCollision = getGapRange(rightGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            collision = (leftGapPtCollision || rightGapPtCollision);

            if (collision) 
            {
                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    collision at " << t);
                if (!gapHasCrossed)
                {
                    gapLifespan = generateCrossedGapTerminalPoints(t, gap);
                    gap->setGapLifespan(gapLifespan);

                    ROS_INFO_STREAM_NAMED("GapFeasibility", "                    considering gap crossed at " << gapLifespan); 
                }

                return;
            }

            ///////////////////////////////////
            // END CONDITION 2: GAP CROSSING //
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
                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    bearing cross at " << t);
                // CLOSING GAP CHECK
                leftCrossPt = prevLeftGapState.head(2);
                rightCrossPt << prevRightGapState.head(2);

                crossedGapPtsDistCheck = (leftCrossPt - rightCrossPt).norm()  < 2*cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; 
                // IF POINTS ARE SUFFICIENTLY CLOSE TOGETHER, GAP HAS CLOSED
                if (crossedGapPtsDistCheck) 
                {    
                    gapLifespan = generateCrossedGapTerminalPoints(t, gap);
                    gap->setGapLifespan(gapLifespan);

                    ROS_INFO_STREAM_NAMED("GapFeasibility", "                    considering gap shut at " << gapLifespan); 

                    return;
                } else
                {
                    gapLifespan = generateCrossedGapTerminalPoints(t, gap);
                    gap->setGapLifespan(gapLifespan);

                    ROS_INFO_STREAM_NAMED("GapFeasibility", "                    considering gap crossed at " << gapLifespan); 

                    return;

                }
            }
            
            //////////////////////////////////////
            // END CONDITION 3: GAP OVERLAPPING //
            //////////////////////////////////////

            prevLeftBearingVect = prevLeftGapState.head(2) / getGapRange(prevLeftGapState); // << std::cos(prev_thetaLeft), std::sin(prev_thetaLeft);
            prevRightBearingVect = prevRightGapState.head(2) / getGapRange(prevRightGapState); // << std::cos(prev_thetaRight), std::sin(prev_thetaRight);
            prevLeftToRightAngle = getSweptLeftToRightAngle(prevLeftBearingVect, prevRightBearingVect);            

            // ROS_INFO_STREAM_NAMED("GapFeasibility", "prevLeftToRightAngle: " << prevLeftToRightAngle << ", leftToRightAngle: " << leftToRightAngle);            
            
            if (prevLeftToRightAngle > (3*M_PI / 4) && leftToRightAngle < (M_PI / 4)) 
            {
                // checking for case of gap crossing behind the robot
                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    crossing from behind, terminal points at: (" << 
                                prevLeftGapState[0] << ", " << prevLeftGapState[1] << "), (" << 
                                prevRightGapState[0] << ", " << prevRightGapState[1] << ")");
                generateTerminalPoints(gap, prevLeftGapState, prevRightGapState); // prev_left_frozen_mp_state[1], prev_left_frozen_mp_state[0], prev_right_frozen_mp_state[1], prev_right_frozen_mp_state[0]);
                gap->setGapLifespan(t);

                return;
            }
            
            prevLeftGapState = leftGapState;
            prevRightGapState = rightGapState;
            prevCentralBearingVect = centralBearingVect;
        }

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                    no close, terminal points at: (" << 
                                                leftGapState[0] << ", " << leftGapState[1] << "), (" << 
                                                rightGapState[0] << ", " << rightGapState[1] << ")");

        generateTerminalPoints(gap, leftGapState, rightGapState);
        gap->setGapLifespan(cfg_->traj.integrate_maxt);
        
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

            if (!(leftSideOpening && (getGapRange(rewindLeftGapState) < cfg_->rbt.vx_absmax * t))) 
                gap->leftGapPtModel_->rewindPropagate(-1 * cfg_->traj.integrate_stept); // resetting model we used before, not good
            
            if (!(rightSideOpening && (getGapRange(rewindRightGapState) < cfg_->rbt.vx_absmax * t))) 
                gap->rightGapPtModel_->rewindPropagate(-1 * cfg_->traj.integrate_stept);
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "t_rew: " << t_rew);

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

}
