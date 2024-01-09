
#include <dynamic_gap/gap_feasibility/GapFeasibilityChecker.h>

namespace dynamic_gap 
{
    bool GapFeasibilityChecker::indivGapFeasibilityCheck(dynamic_gap::Gap * gap) 
    {
        ROS_INFO_STREAM_NAMED("GapFeasibility", "        [indivGapFeasibilityCheck()]");
        dynamic_gap::Estimator* leftGapPtModel = gap->leftGapPtModel_;
        dynamic_gap::Estimator* rightGapPtModel = gap->rightGapPtModel_;

        leftGapPtModel->isolateGapDynamics();
        rightGapPtModel->isolateGapDynamics();

        Eigen::Vector4f leftGapState = leftGapPtModel->getGapState(); 
        Eigen::Vector4f rightGapState = rightGapPtModel->getGapState();

        float leftGapBearingRate = getGapBearingRateOfChange(leftGapState); 
        float rightGapBearingRate = getGapBearingRateOfChange(rightGapState); 

        float minGapBearingRate = std::min(leftGapBearingRate, rightGapBearingRate);
        float subLeftGapBearingRate = leftGapBearingRate - minGapBearingRate;
        float subRightGapBearingRate = rightGapBearingRate - minGapBearingRate;

        // ROS_INFO_STREAM_NAMED("GapFeasibility", "frozen left betadot: " << leftGapBearingRateOfChange);
        // ROS_INFO_STREAM_NAMED("GapFeasibility", "frozen right betadot: " << rightGapBearingRateOfChange);

        float crossing_time = gapSplinecheck(gap, leftGapPtModel, rightGapPtModel);

        bool feasible = false;
        if (gap->artificial_) 
        {
            feasible = true;
            gap->gapLifespan_ = cfg_->traj.integrate_maxt;
            gap->setTerminalPoints(gap->LIdx(), gap->LDist(), gap->RIdx(), gap->RDist());
            gap->setCategory("static");
        } else if (subLeftGapBearingRate > 0) 
        {
            // expanding
            feasible = true;
            gap->gapLifespan_ = cfg_->traj.integrate_maxt;
            gap->setCategory("expanding");
        } else if (subLeftGapBearingRate == 0 && subRightGapBearingRate == 0) 
        {
            // static
            feasible = true;
            gap->gapLifespan_ = cfg_->traj.integrate_maxt;
            gap->setCategory("static");
        } else 
        {
            // closing
            gap->setCategory("closing"); 
            if (crossing_time >= 0) 
            {
                feasible = true;
                gap->gapLifespan_ = crossing_time;
            }
        }

        ROS_INFO_STREAM_NAMED("GapFeasibility", "            gap is feasible: " << feasible);

        return feasible;
    }
    

    float GapFeasibilityChecker::gapSplinecheck(dynamic_gap::Gap * gap, 
                                                dynamic_gap::Estimator* leftGapPtModel, 
                                                dynamic_gap::Estimator* rightGapPtModel) 
    {
        ROS_INFO_STREAM_NAMED("GapFeasibility", "            [gapSplinecheck()]");        
        Eigen::Vector2f crossingPt(0.0, 0.0);
        float gapCrossingTime = indivGapFindCrossingPoint(gap, crossingPt, leftGapPtModel, rightGapPtModel);

        Eigen::Vector2f splineInitialPos(0.0, 0.0);
        Eigen::Vector2f splineInitialVel(leftGapPtModel->getRobotVel().twist.linear.x, leftGapPtModel->getRobotVel().twist.linear.y);
        
        Eigen::Vector2f splineTerminalVel(0.0, 0.0);
        
        splineTerminalVel = splineInitialVel.norm() * unitNorm(crossingPt);
            //  << splineInitialVel.norm() * crossingPt[0] / crossingPt.norm(), 
            //                    splineInitialVel.norm() * crossingPt[1] / crossingPt.norm();
        
        // ROS_INFO_STREAM_NAMED("GapFeasibility", "starting x: " << splineInitialPos[0] << ", " << splineInitialPos[1] << ", " << splineInitialVel[0] << ", " << splineInitialVel[1]);
        // ROS_INFO_STREAM_NAMED("GapFeasibility", "ending x: " << crossingPt[0] << ", " << crossingPt[1] << ", splineTerminalVel: " << splineTerminalVel[0] << ", " << splineTerminalVel[1]);
        
        Eigen::MatrixXf splineAMat = Eigen::MatrixXf::Random(4,4);
        Eigen::VectorXf splineBVec = Eigen::VectorXf::Random(4);
        splineAMat << 1.0, 0.0, 0.0, 0.0,
                      0.0, 1.0, 0.0, 0.0,
                      1.0, gapCrossingTime, pow(gapCrossingTime,2), pow(gapCrossingTime,3),
                      0.0, 1.0, 2*gapCrossingTime, 3*pow(gapCrossingTime,2);
        splineBVec << splineInitialPos[0], splineInitialVel[0], crossingPt[0], splineTerminalVel[0];
        gap->splineXCoefs_ = splineAMat.partialPivLu().solve(splineBVec);

        // std::cout << "x coeffs: " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ", " << coeffs[3] << std::endl;
        float slinePeakVelTime = gapCrossingTime/2.0;
        float peakSplineVelX = 3*gap->splineXCoefs_[3]*pow(slinePeakVelTime, 2) + 
                                 2*gap->splineXCoefs_[2]*slinePeakVelTime + 
                                 gap->splineXCoefs_[1];
        
        splineBVec << splineInitialPos[1], splineInitialVel[1], crossingPt[1], splineTerminalVel[1];

        gap->splineYCoefs_ = splineAMat.partialPivLu().solve(splineBVec);
        //std::cout << "y coeffs: " << coeffs[0] << ", " << coeffs[1] << ", " << coeffs[2] << ", " << coeffs[3] << std::endl;
        float peakSplineVelY = 3*gap->splineYCoefs_[3]*pow(slinePeakVelTime, 2) + 
                                 2*gap->splineYCoefs_[2]*slinePeakVelTime + 
                                 gap->splineYCoefs_[1];

        // ROS_INFO_STREAM_NAMED("GapFeasibility", "peak velocity: " << peakSplineVelX << ", " << peakSplineVelY);
        gap->peakVelX_ = peakSplineVelX;
        gap->peakVelY_ = peakSplineVelY;
        
        if (std::max(std::abs(peakSplineVelX), std::abs(peakSplineVelY)) <= cfg_->control.vx_absmax)
            return gapCrossingTime;
        else
            return -1.0; // communicating infeasibility
    }
 
    float GapFeasibilityChecker::indivGapFindCrossingPoint(dynamic_gap::Gap * gap, 
                                                           Eigen::Vector2f& gapCrossingPt, 
                                                           dynamic_gap::Estimator* leftGapPtModel, 
                                                           dynamic_gap::Estimator* rightGapPtModel) 
    {
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                [indivGapFindCrossingPoint()]");

        float thetaLeft = idx2theta(gap->LIdx());
        float thetaRight = idx2theta(gap->RIdx());

        Eigen::Vector2f leftBearingVect(cos(thetaLeft), sin(thetaLeft)); 
        Eigen::Vector2f rightBearingVect(cos(thetaRight), sin(thetaRight));

        float leftToRightAngle = getLeftToRightAngle(leftBearingVect, rightBearingVect);
        
        Eigen::Vector2f prevLeftBearingVect = leftBearingVect;        
        Eigen::Vector2f prevRightBearingVect = rightBearingVect;

        float thetaCenter = (thetaLeft - (leftToRightAngle / 2.0));

        float prevLeftToRightAngle = leftToRightAngle;
        
        Eigen::Vector2f centralBearingVect(std::cos(thetaCenter), std::sin(thetaCenter));
        
        //std::cout << "initial beta left: (" << leftBearingVect[0] << ", " << leftBearingVect[1] << "), initial beta right: (" << rightBearingVect[0] << ", " << rightBearingVect[1] << "), initial beta center: (" << centralBearingVect[0] << ", " << centralBearingVect[1] << ")" << std::endl;
        
        Eigen::Vector4f leftGapState = leftGapPtModel->getGapState();
        Eigen::Vector4f rightGapState = rightGapPtModel->getGapState();

        // ROS_INFO_STREAM_NAMED("GapFeasibility", "gap category: " << gap->getCategory());
        // ROS_INFO_STREAM_NAMED("GapFeasibility", "starting frozen cartesian left: " << leftGapState[0] << ", " << leftGapState[1] << ", " << leftGapState[2] << ", " << leftGapState[3]); 
        // ROS_INFO_STREAM_NAMED("GapFeasibility", "starting frozen cartesian right: " << rightGapState[0] << ", " << rightGapState[1] << ", " << rightGapState[2] << ", " << rightGapState[3]);
       
        float leftBearingDotCentBearing = 0.0, rightBearingDotCentBearing = 0.0;
        bool gapHasCrossed = true;
        bool bearingsCrossed = false, crossedGapPtsDistCheck = false;    

        bool leftSideOpening = (getGapBearingRateOfChange(leftGapState) > 0.0);
        bool rightSideOpening = (getGapBearingRateOfChange(rightGapState) < 0.0);
        Eigen::Vector4f prevLeftGapState = leftGapState;
        Eigen::Vector4f prevRightGapState = rightGapState;       
        Eigen::Vector2f prevCentralBearingVect = centralBearingVect;

        Eigen::Vector2f leftCrossPt(0.0, 0.0);
        Eigen::Vector2f rightCrossPt(0.0, 0.0);
        bool leftGapPtCollision = false, rightGapPtCollision = false, collision = false;
        for (float t = cfg_->traj.integrate_stept; t < cfg_->traj.integrate_maxt; t += cfg_->traj.integrate_stept) 
        {
            // checking to see if left point is reachable
            if (!(leftSideOpening && (getGapDist(leftGapState) < cfg_->control.vx_absmax * t))) 
            {
                // ROS_INFO_STREAM_NAMED("GapFeasibility", "propagating left");
                leftGapPtModel->gapStatePropagate(cfg_->traj.integrate_stept);
            }

            // checking to see if right point is reachable
            if (!(rightSideOpening && (getGapDist(rightGapState) < cfg_->control.vx_absmax * t))) 
            {
                // ROS_INFO_STREAM_NAMED("GapFeasibility", "propagating right");
                rightGapPtModel->gapStatePropagate(cfg_->traj.integrate_stept);
            }
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "t: " << t);

            leftGapState = leftGapPtModel->getGapState();
            rightGapState = rightGapPtModel->getGapState();

            leftGapPtCollision = getGapDist(leftGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            rightGapPtCollision = getGapDist(rightGapState) < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            collision = (leftGapPtCollision || rightGapPtCollision);

            if (collision) 
            {
                gapCrossingPt << 0.0, 0.0;
                gap->setClosingPoint(gapCrossingPt[0], gapCrossingPt[1]);
                gap->closed_ = true;

                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    collision at " << t);
                // , terminal points at: (" << 
                //                 prevLeftGapState[0] << ", " << prevLeftGapState[1] << "), (" << 
                //                 prevRightGapState[0] << ", " << prevRightGapState[1] << ")");
                generateTerminalPoints(gap, prevLeftGapState, prevRightGapState); // prev_left_frozen_mp_state[1], prev_left_frozen_mp_state[0], prev_right_frozen_mp_state[1], prev_right_frozen_mp_state[0]);
                return t;
            }

            thetaLeft = getGapBearing(leftGapState);
            thetaRight = getGapBearing(rightGapState);
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "thetaLeft: " << thetaLeft << ", thetaRight: " << thetaRight);
            leftBearingVect = leftGapState.head(2) / getGapDist(leftGapState); // << std::cos(thetaLeft), std::sin(thetaLeft);
            rightBearingVect = rightGapState.head(2) / getGapDist(rightGapState); // << std::cos(thetaRight), std::sin(thetaRight);
            leftToRightAngle = getLeftToRightAngle(leftBearingVect, rightBearingVect);
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
                    if (leftCrossPt.norm() < rightCrossPt.norm())
                        gapCrossingPt << rightCrossPt[0], rightCrossPt[1];
                    else
                        gapCrossingPt << leftCrossPt[0], leftCrossPt[1];

                    gapCrossingPt += 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * unitNorm(gapCrossingPt);;
                    gap->setClosingPoint(gapCrossingPt[0], gapCrossingPt[1]);
                    float gapLifespan = generateCrossedGapTerminalPoints(t, gap, leftGapPtModel, rightGapPtModel);

                    ROS_INFO_STREAM_NAMED("GapFeasibility", "                    considering gap closed at " << gapLifespan); 

                    gap->closed_ = true;
                    return gapLifespan;
                } else
                {
                    if (gapHasCrossed) 
                    {
                        float mid_x = (leftCrossPt[0] + rightCrossPt[0]) / 2;
                        float mid_y = (leftCrossPt[1] + rightCrossPt[1]) / 2;
                        //  ROS_INFO_STREAM_NAMED("GapFeasibility", "gap crosses but does not close at " << t << ", left point at: " << leftCrossPt[0] << ", " << leftCrossPt[1] << ", right point at " << rightCrossPt[0] << ", " << rightCrossPt[1]); 

                        gap->setCrossingPoint(mid_x, mid_y);
                        gapHasCrossed = false;

                        float gapLifespan = generateCrossedGapTerminalPoints(t, gap, leftGapPtModel, rightGapPtModel);

                        ROS_INFO_STREAM_NAMED("GapFeasibility", "                    considering gap crossed at " << gapLifespan); 

                        gap->crossed_ = true;
                    }
                }
            }
            
            prevLeftBearingVect = prevLeftGapState.head(2) / getGapDist(prevLeftGapState); // << std::cos(prev_thetaLeft), std::sin(prev_thetaLeft);
            prevRightBearingVect = prevRightGapState.head(2) / getGapDist(prevRightGapState); // << std::cos(prev_thetaRight), std::sin(prev_thetaRight);
            prevLeftToRightAngle = getLeftToRightAngle(prevLeftBearingVect, prevRightBearingVect);            

            // ROS_INFO_STREAM_NAMED("GapFeasibility", "prevLeftToRightAngle: " << prevLeftToRightAngle << ", leftToRightAngle: " << leftToRightAngle);            
            
            if (prevLeftToRightAngle > (3*M_PI / 4) && leftToRightAngle < (M_PI / 4)) 
            {
                // checking for case of gap crossing behind the robot
                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    crossing from behind, terminal points at: (" << 
                                prevLeftGapState[0] << ", " << prevLeftGapState[1] << "), (" << 
                                prevRightGapState[0] << ", " << prevRightGapState[1] << ")");
                generateTerminalPoints(gap, prevLeftGapState, prevRightGapState); // prev_left_frozen_mp_state[1], prev_left_frozen_mp_state[0], prev_right_frozen_mp_state[1], prev_right_frozen_mp_state[0]);
                gap->crossedBehind_ = true;
            }
            
            prevLeftGapState = leftGapState;
            prevRightGapState = rightGapState;
            prevCentralBearingVect = centralBearingVect;
        }

        if (!gap->crossed_ && !gap->closed_ && !gap->crossedBehind_) 
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "                    no close, terminal points at: (" << 
                                                    leftGapState[0] << ", " << leftGapState[1] << "), (" << 
                                                    rightGapState[0] << ", " << rightGapState[1] << ")");

            generateTerminalPoints(gap, leftGapState, rightGapState);
        }

        return cfg_->traj.integrate_maxt;
    }

    float GapFeasibilityChecker::generateCrossedGapTerminalPoints(const float & t, dynamic_gap::Gap * gap, 
                                                                    dynamic_gap::Estimator* leftGapPtModel, 
                                                                    dynamic_gap::Estimator* rightGapPtModel) 
    {    
        Eigen::Vector4f rewindLeftGapState = leftGapPtModel->getGapState();
        Eigen::Vector4f rewindRightGapState = rightGapPtModel->getGapState();        

        Eigen::Vector2f leftCrossPt(0.0, 0.0);
        Eigen::Vector2f rightCrossPt(0.0, 0.0); 

        // instantiate model rewind states
        leftGapPtModel->setRewindState();
        rightGapPtModel->setRewindState();
        // do rewindPropagate

        // REWINDING THE GAP FROM ITS CROSSED CONFIGURATION UNTIL THE GAP IS SUFFICIENTLY OPEN
        for (float tRewind = t; tRewind >= 0.0; tRewind -= cfg_->traj.integrate_stept) 
        {
            leftGapPtModel->rewindPropagate(-1 * cfg_->traj.integrate_stept); // resetting model we used before, not good
            rightGapPtModel->rewindPropagate(-1 * cfg_->traj.integrate_stept);
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "t_rew: " << t_rew);

            rewindLeftGapState = leftGapPtModel->getRewindGapState();
            rewindRightGapState = rightGapPtModel->getRewindGapState();  

            leftCrossPt = rewindLeftGapState.head(2);
            rightCrossPt = rewindRightGapState.head(2);           

            // if gap is sufficiently open
            // option 1: arc-length:
            if (tRewind == 0 || (leftCrossPt - rightCrossPt).norm() >  2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) // r_min * leftToRightAngle > 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio
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
        float rangeLeft = getGapDist(leftCrossPt);

        float thetaRight = getGapBearing(rightCrossPt);
        int idxRight = theta2idx(thetaRight);
        float rangeRight = getGapDist(rightCrossPt);

        ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting terminal points to, left: (" << 
                                                leftCrossPt[0] << ", " << leftCrossPt[1] << "), right: (" << 
                                                rightCrossPt[0] << ", " << rightCrossPt[1] << ")");

        gap->setTerminalPoints(idxLeft, rangeLeft, idxRight, rangeRight);
    }

}
