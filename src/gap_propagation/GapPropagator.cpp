
#include <dynamic_gap/gap_propagation/GapPropagator.h>

namespace dynamic_gap 
{
    void GapPropagator::propagateGapPoints(Gap * gap) 
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

        // ROS_INFO_STREAM("gap category: " << gap->getCategory());
        ROS_INFO_STREAM_NAMED("GapFeasibility", "       Starting frozen cartesian left: " << leftGapState[0] << ", " << leftGapState[1] << ", " << leftGapState[2] << ", " << leftGapState[3]); 
        ROS_INFO_STREAM_NAMED("GapFeasibility", "       Starting frozen cartesian right: " << rightGapState[0] << ", " << rightGapState[1] << ", " << rightGapState[2] << ", " << rightGapState[3]);
       
        float leftBearingDotCentBearing = 0.0, rightBearingDotCentBearing = 0.0;
        bool gapHasCrossed = false;
        bool bearingsCrossed = false;
        bool crossedGapPtsDistCheck = false;    

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
            
            // leftGapPtCollision = leftGapState.head(2).normalized() < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            // rightGapPtCollision = rightGapState.head(2).normalized() < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            // collision = (leftGapPtCollision || rightGapPtCollision);

            // if (collision) 
            // {
            //     ROS_INFO_STREAM_NAMED("GapFeasibility", "                    end condition 0 (collision) at " << t);
            //     if (!gapHasCrossed)
            //     {
            //         generateTerminalPoints(gap, leftGapState, rightGapState);
            //         gap->setGapLifespan(t);
            //         gap->end_condition = 0;

            //         ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting gap lifespan to " << gap->gapLifespan_); 
            //     }

            //     return;
            // }

            ///////////////////////////////////
            // END CONDITION 1: GAP CROSSING //
            ///////////////////////////////////
            
            thetaLeft = gap->leftGapPtModel_->getGapBearing();
            thetaRight = gap->rightGapPtModel_->getGapBearing();
            // ROS_INFO_STREAM_NAMED("GapFeasibility", "thetaLeft: " << thetaLeft << ", thetaRight: " << thetaRight);
            leftBearingVect = leftGapState.head(2).normalized();
            rightBearingVect = rightGapState.head(2).normalized();
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

                gapLifespan = rewindGapPoints(t, gap);
                gap->setGapLifespan(gapLifespan);
                gap->end_condition = 1;

                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting gap lifespan to " << gap->gapLifespan_); 

                return;
            }
            
            //////////////////////////////////////
            // END CONDITION 2: GAP OVERLAPPING //
            //////////////////////////////////////

            // ROS_INFO_STREAM("prevLeftToRightAngle: " << prevLeftToRightAngle << ", leftToRightAngle: " << leftToRightAngle);            
            
            if (leftToRightAngle < M_PI && leftBearingDotCentBearing < 0.0 && rightBearingDotCentBearing < 0.0) 
            {
                // checking for case of gap crossing behind the robot
                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    end condition 2 (overlapping) at " << t);

                gap->setGapLifespan(t - cfg_->traj.integrate_stept);
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

        gap->setGapLifespan(cfg_->traj.integrate_maxt);
        gap->end_condition = 3;
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting gap lifespan to " << gap->gapLifespan_); 

        return;
    }

    void GapPropagator::propagateGapPointsV2(const std::vector<Gap *> & gaps) 
    {
        ROS_INFO_STREAM_NAMED("GapFeasibility", "   [GapPropagator::propagateGapPointsV2()]");

        gapPoints_.clear();

        // 1. Turn gaps into gap points
        convertGapsToGapPoints(gaps);

        // 2. Sort gap points by bearing
        std::sort(gapPoints_.begin(), gapPoints_.end(), GapPointComparator());

        // 3. Assign Ungap IDs to gap points
        assignUnGapIDsToGapPoints();

        // 4. Propagate gap points loop
        for (float t = cfg_->traj.integrate_stept; t < cfg_->traj.integrate_maxt; t += cfg_->traj.integrate_stept) 
        {
            ROS_INFO_STREAM_NAMED("GapFeasibility", "       t: " << t);

            // 5. Propagate points
            for (int i = 0; i < gapPoints_.size(); i++)
            {
                gapPoints_.at(i)->propagate(cfg_->traj.integrate_stept);
            }

            // 6. Re-sort by bearing
            std::sort(gapPoints_.begin(), gapPoints_.end(), GapPointComparator());

            ROS_INFO_STREAM_NAMED("GapFeasibility", "       Gap points: ");
            for (int i = 0; i < gapPoints_.size(); i++)
            {
                ROS_INFO_STREAM_NAMED("GapFeasibility", "           gap point " << i);
                ROS_INFO_STREAM_NAMED("GapFeasibility", "               scan idx: " << gapPoints_.at(i)->getScanIdx());
                ROS_INFO_STREAM_NAMED("GapFeasibility", "               model state: " << gapPoints_.at(i)->getModel()->getGapState().transpose());
                ROS_INFO_STREAM_NAMED("GapFeasibility", "               model ID: " << gapPoints_.at(i)->getModel()->getID());
                ROS_INFO_STREAM_NAMED("GapFeasibility", "               ungapID: " << gapPoints_.at(i)->getUngapID());
                ROS_INFO_STREAM_NAMED("GapFeasibility", "               is left: " << gapPoints_.at(i)->isLeft());
                ROS_INFO_STREAM_NAMED("GapFeasibility", "               is right: " << gapPoints_.at(i)->isRight());
            }

            // 7. Generate gaps
            ROS_INFO_STREAM_NAMED("GapFeasibility", "       Generating gaps ...");
            for (int i = 0; i < gapPoints_.size(); i++) // outer loop through points
            {
                ROS_INFO_STREAM_NAMED("GapFeasibility", "           looping for outer gap point " << i);

                GapPoint * gapPtI = gapPoints_.at(i);

                if (gapPtI->isAssignedToGap()) // skip if point is already assigned to a gap
                {
                    ROS_INFO_STREAM_NAMED("GapFeasibility", "               gap point is already assigned to a gap, continuing...");
                    continue;
                }

                if (gapPtI->isRight()) // checking for open gap (right to left)
                {
                    ROS_INFO_STREAM_NAMED("GapFeasibility", "               gap point is right, looping for open gap");
                    for (int adder = 1; adder < gapPoints_.size(); adder++) // inner loop through points
                    {
                        int j = (i + adder) % gapPoints_.size();
                        ROS_INFO_STREAM_NAMED("GapFeasibility", "                   looping for inner gap point " << j);
                     
                        GapPoint * gapPtJ = gapPoints_.at(j);

                        if (gapPtJ->isAssignedToGap()) // skip if point is already assigned to a gap
                        {
                            ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gap point is already assigned to a gap, continuing...");
                            continue;
                        }

                        if (gapPtJ->isRight())
                        {
                            ROS_INFO_STREAM_NAMED("GapFeasibility", "                       found a better right gap point, breaking...");
                            break;
                        } else
                        {
                            ROS_INFO_STREAM_NAMED("GapFeasibility", "                       creating an open gap from pt " << i << " to pt " << j);
                            // Create an open gap
                            gapPtI->assignToGap();
                            gapPtJ->assignToGap();

                            // Create a gap

                            break;
                        }
                    }
                } else if (gapPtI->isLeft()) // checking for reversed gap
                {
                    ROS_INFO_STREAM_NAMED("GapFeasibility", "               gap point is right, looping for reversed gap");
                    for (int adder = 1; adder < gapPoints_.size(); adder++) // inner loop through points
                    {                        
                        int j = (i + adder) % gapPoints_.size();
                        ROS_INFO_STREAM_NAMED("GapFeasibility", "                   looping for inner gap point " << j);
                     
                        GapPoint * gapPtJ = gapPoints_.at(j);

                        if (gapPtJ->isAssignedToGap()) // skip if point is already assigned to a gap
                        {
                            ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gap point is already assigned to a gap, continuing...");                        
                            continue;
                        }

                        if (gapPtJ->isLeft())
                        {
                            ROS_INFO_STREAM_NAMED("GapFeasibility", "                       found a better right gap point, breaking...");
                            break;
                        } else
                        {
                            if (gapPtJ->getUngapID() != gapPtI->getUngapID())
                            {
                                ROS_INFO_STREAM_NAMED("GapFeasibility", "                       creating a reversed gap from pt " << i << " to pt " << j);                         
                                // Create a reversed gap
                                gapPtI->assignToGap();
                                gapPtJ->assignToGap();

                                // Create a gap

                                break;
                            } else
                            {
                                ROS_INFO_STREAM_NAMED("GapFeasibility", "                       gap points form an ungap, continuing...");
                            }
                        }
                    }
                } else
                {
                    ROS_WARN_STREAM_NAMED("GapFeasibility", "        gap point is not left or right");
                }
            }
        }

        // delete gap points
        for (GapPoint * gapPt : gapPoints_)
        {
            delete gapPt;
        }

        return;
    }

    void GapPropagator::convertGapsToGapPoints(const std::vector<Gap *> & gaps)
    {
        for (const Gap * gap : gaps)
        {
            // right
            gap->rightGapPtModel_->isolateGapDynamics();
            float rightGapPtTheta = gap->rightGapPtModel_->getGapBearing();
            int rightGapPtIdx = theta2idx(rightGapPtTheta);

            if (rightGapPtIdx >= 0 && rightGapPtIdx < cfg_->scan.full_scan)
                gapPoints_.push_back(new GapPoint(gap->rightGapPtModel_, rightGapPtIdx, false));
            else
                ROS_WARN_STREAM_NAMED("GapFeasibility", "        right gap pt idx out of bounds");

            // left
            gap->leftGapPtModel_->isolateGapDynamics();
            float leftGapPtTheta = gap->leftGapPtModel_->getGapBearing();
            int leftGapPtIdx = theta2idx(leftGapPtTheta);

            if (leftGapPtIdx >= 0 && leftGapPtIdx < cfg_->scan.full_scan)
                gapPoints_.push_back(new GapPoint(gap->leftGapPtModel_, leftGapPtIdx, true));
            else
                ROS_WARN_STREAM_NAMED("GapFeasibility", "        left gap pt idx out of bounds");
            }
    }

    void GapPropagator::assignUnGapIDsToGapPoints()
    {
        for (int i = 0; i < gapPoints_.size(); i++)
        {
            Eigen::Vector4f ptIState = gapPoints_.at(i)->getModel()->getGapState();

            int nextIdx = (i + 1) % gapPoints_.size();
            Eigen::Vector4f ptJState = gapPoints_.at(nextIdx)->getModel()->getGapState();

            if (isUngap(ptIState, ptJState))
            {
                gapPoints_.at(i)->setUngapID(i);
                gapPoints_.at(nextIdx)->setUngapID(i);
            }
        }
    }

    bool GapPropagator::isUngap(const Eigen::Vector4f & ptIState, const Eigen::Vector4f & ptJState)
    {
        Eigen::Vector2f ptIPos = ptIState.head(2);
        Eigen::Vector2f ptJPos = ptJState.head(2);

        Eigen::Vector2f ptIVel = ptIState.tail(2);
        Eigen::Vector2f ptJVel = ptJState.tail(2);

        // check if distance between points is less than 4 * r_inscr * inf_ratio
        bool distCheck = (ptIPos - ptJPos).norm() < 4 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

        // // check if speed of points is greater than 0.10
        bool speedCheck = (ptIVel.norm() >= 0.10 && ptJVel.norm() >= 0.10);

        // // run angle check on LHS and RHS model velocities
        float vectorProj = ptIVel.dot(ptJVel) / (ptIVel.norm() * ptJVel.norm() + eps);
        bool angleCheck = (vectorProj > 0.0);

        return (distCheck && speedCheck && angleCheck);
    }

    ////////
    // v1 //
    ////////

    float GapPropagator::rewindGapPoints(const float & t, Gap * gap) 
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


            leftBearingVect = leftRewindPt.normalized();
            rightBearingVect = rightRewindPt.normalized();

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
}
