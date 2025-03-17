
#include <dynamic_gap/gap_propagation/GapPropagator.h>

namespace dynamic_gap 
{
    void GapPropagator::propagateGapPoints(Gap * gap) 
    {
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                [propagateGapPoints()]");

        Eigen::Vector2f crossingPt(0.0, 0.0);

        gap->getLeftGapPt()->getModel()->isolateGapDynamics();
        gap->getRightGapPt()->getModel()->isolateGapDynamics();

        float thetaLeft = idx2theta(gap->LIdx());
        float thetaRight = idx2theta(gap->RIdx());

        Eigen::Vector2f leftBearingVect(cos(thetaLeft), sin(thetaLeft)); 
        Eigen::Vector2f rightBearingVect(cos(thetaRight), sin(thetaRight));

        float leftToRightAngle = getSweptLeftToRightAngle(leftBearingVect, rightBearingVect);
        
        Eigen::Vector2f prevLeftBearingVect = leftBearingVect;        
        Eigen::Vector2f prevRightBearingVect = rightBearingVect;

        float thetaCenter = (thetaLeft - (0.5 * leftToRightAngle));

        float prevLeftToRightAngle = leftToRightAngle;
        
        Eigen::Vector2f centralBearingVect(std::cos(thetaCenter), std::sin(thetaCenter));
        
        //std::cout << "initial beta left: (" << leftBearingVect[0] << ", " << leftBearingVect[1] << "), initial beta right: (" << rightBearingVect[0] << ", " << rightBearingVect[1] << "), initial beta center: (" << centralBearingVect[0] << ", " << centralBearingVect[1] << ")" << std::endl;
        
        Eigen::Vector4f leftGapState = gap->getLeftGapPt()->getModel()->getGapState();
        Eigen::Vector4f rightGapState = gap->getRightGapPt()->getModel()->getGapState();

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
            gap->getLeftGapPt()->getModel()->gapStatePropagate(cfg_->traj.integrate_stept);

            // propagate right point
            gap->getRightGapPt()->getModel()->gapStatePropagate(cfg_->traj.integrate_stept);

            leftGapState = gap->getLeftGapPt()->getModel()->getGapState();
            rightGapState = gap->getRightGapPt()->getModel()->getGapState();

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
            //         gap->setEndCondition(COLLISION);

            //         ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting gap lifespan to " << gap->getGapLifespan()); 
            //     }

            //     return;
            // }

            ///////////////////////////////////
            // END CONDITION 1: GAP CROSSING //
            ///////////////////////////////////
            
            thetaLeft = gap->getLeftGapPt()->getModel()->getGapBearing();
            thetaRight = gap->getRightGapPt()->getModel()->getGapBearing();
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
                gap->setEndCondition(SHUT);

                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting gap lifespan to " << gap->getGapLifespan()); 

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
                gap->setEndCondition(OVERLAPPED);

                ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting gap lifespan to " << gap->getGapLifespan()); 

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
        gap->setEndCondition(TIMED_OUT);
        ROS_INFO_STREAM_NAMED("GapFeasibility", "                    setting gap lifespan to " << gap->getGapLifespan()); 

        return;
    }

    void GapPropagator::propagateGapPointsV2(const std::vector<Gap *> & gaps) 
    {
        ROS_INFO_STREAM_NAMED("GapPropagator", "   [GapPropagator::propagateGapPointsV2()]");

        // deep copy of gaps
        std::vector<Gap *> propagatedGaps;
        for (Gap * gap : gaps)
            propagatedGaps.push_back(new Gap(*gap));

        // create gap tubes
        gapTubes_.resize(propagatedGaps.size());
        for (int i = 0; i < propagatedGaps.size(); i++)
            gapTubes_.at(i) = new GapTube(propagatedGaps.at(i));

        ROS_INFO_STREAM_NAMED("GapPropagator", "       Gaps at time " << 0.0 << ": ");
        for (int i = 0; i < gaps.size(); i++)
        {
            Gap * gap = gaps.at(i);
            ROS_INFO_STREAM_NAMED("GapPropagator", "           gap " << i << ": ");
            ROS_INFO_STREAM_NAMED("GapPropagator", "                left point:" << gap->getLPosition().transpose());
            ROS_INFO_STREAM_NAMED("GapAssociator", "		        left ID: (" << gap->getLeftGapPt()->getModel()->getID() << ")");                        
            ROS_INFO_STREAM_NAMED("GapPropagator", "                right point:" << gap->getRPosition().transpose());
            ROS_INFO_STREAM_NAMED("GapAssociator", "				right ID: (" << gap->getRightGapPt()->getModel()->getID() << ")");            
        }

        // 1. Turn gaps into gap points
        convertGapsToGapPoints(gaps); // CONSTRUCTING PROPAGATED GAP POINTS

        // 2. Sort gap points by bearing
        std::sort(propagatedGapPoints_.begin(), propagatedGapPoints_.end(), PropagatedGapPointComparator());

        // 3. Assign Ungap IDs to gap points
        assignUnGapIDsToGapPoints();

        // 4. Propagate gap points loop
        std::vector<Gap *> previousGaps = propagatedGaps; // SHALLOW COPY
        std::vector<Gap *> currentGaps;

        float t_i = 0.0, t_iplus1 = 0.0;
        int numSteps = int(cfg_->traj.integrate_maxt/cfg_->traj.integrate_stept) + 1;
        for (int futureTimeIdx = 1; futureTimeIdx < numSteps; futureTimeIdx++) 
        {        
            t_iplus1 = t_i + cfg_->traj.integrate_stept;        

            ROS_INFO_STREAM_NAMED("GapPropagator", "        t: " << t_iplus1);

            // 5. Propagate points
            for (int i = 0; i < propagatedGapPoints_.size(); i++)
            {
                propagatedGapPoints_.at(i)->propagate(cfg_->traj.integrate_stept);
            }

            // 6. Re-sort by bearing
            std::sort(propagatedGapPoints_.begin(), propagatedGapPoints_.end(), PropagatedGapPointComparator());

            // ROS_INFO_STREAM_NAMED("GapPropagator", "       Gap points: ");
            // for (int i = 0; i < propagatedGapPoints_.size(); i++)
            // {
            //     ROS_INFO_STREAM_NAMED("GapPropagator", "           gap point " << i);
            //     // ROS_INFO_STREAM_NAMED("GapPropagator", "               scan idx: " << propagatedGapPoints_.at(i)->getScanIdx());
            //     ROS_INFO_STREAM_NAMED("GapPropagator", "               model state: " << propagatedGapPoints_.at(i)->getModel()->getGapState().transpose());
            //     ROS_INFO_STREAM_NAMED("GapPropagator", "               model ID: " << propagatedGapPoints_.at(i)->getModel()->getID());
            //     ROS_INFO_STREAM_NAMED("GapPropagator", "               ungapID: " << propagatedGapPoints_.at(i)->getUngapID());
            //     ROS_INFO_STREAM_NAMED("GapPropagator", "               is left: " << propagatedGapPoints_.at(i)->isLeft());
            //     ROS_INFO_STREAM_NAMED("GapPropagator", "               is right: " << propagatedGapPoints_.at(i)->isRight());
            // }

            // 7. Generate gaps
            // ROS_INFO_STREAM_NAMED("GapPropagator", "       Generating gaps ...");
            getGaps(currentGaps, t_iplus1);

            // 8. Perform gap association
            runGapAssociation(currentGaps, previousGaps, t_iplus1);

            // 9. Delete gaps we don't need

            for (Gap * gap : previousGaps)
            {
                // if (gap->getSafeToDelete())
                // {
                    delete gap;
                // }
            }
            previousGaps.clear();
            previousGaps = currentGaps;

            t_i = t_iplus1;
        }

        for (Gap * gap : previousGaps)
        {
            // if (gap->getSafeToDelete())
            // {
                delete gap;
            // }
        }


        // print out gap tubes


        //////////////
        // CLEAN UP //
        //////////////

        // delete gap tubes
        // all gaps in here should be intact still, all other gaps should have gotten deleted
        for (GapTube * tube : gapTubes_)
        {
            delete tube;
        }
        gapTubes_.clear();

        // delete gap points
        for (PropagatedGapPoint * propagatedGapPt : propagatedGapPoints_)
        {
            delete propagatedGapPt;
        }
        propagatedGapPoints_.clear();

        return;
    }

    void GapPropagator::runGapAssociation(const std::vector<Gap *> & currentGaps, 
                                            const std::vector<Gap *> & previousGaps,
                                            const float & t_iplus1)
    {
        distMatrix_ = gapAssociator_->populateDistMatrix(currentGaps, previousGaps);
        assocation_ = gapAssociator_->associate(distMatrix_);
        gapAssociator_->assignGaps(assocation_, distMatrix_, currentGaps, previousGaps, gapTubes_, t_iplus1);
    }

    void GapPropagator::getGaps(std::vector<Gap *> & currentGaps,
                                const float & t_iplus1)
    {
        ROS_INFO_STREAM_NAMED("GapPropagator", "            [getGaps]");

        currentGaps.clear();

        for (int i = 0; i < propagatedGapPoints_.size(); i++) // outer loop through points
        {
            // ROS_INFO_STREAM_NAMED("GapPropagator", "           looping for outer gap point " << i);

            PropagatedGapPoint * propagatedGapPtI = propagatedGapPoints_.at(i);

            if (propagatedGapPtI->isAssignedToGap()) // skip if point is already assigned to a gap
            {
                // ROS_INFO_STREAM_NAMED("GapPropagator", "               gap point is already assigned to a gap, continuing...");
                continue;
            }

            if (propagatedGapPtI->isRight()) // checking for open gap (right to left)
            {
                // ROS_INFO_STREAM_NAMED("GapPropagator", "               gap point is right, looping for open gap");
                for (int adder = 1; adder < propagatedGapPoints_.size(); adder++) // inner loop through points
                {
                    int j = (i + adder) % propagatedGapPoints_.size();
                    // ROS_INFO_STREAM_NAMED("GapPropagator", "                   looping for inner gap point " << j);
                 
                    PropagatedGapPoint * propagatedGapPtJ = propagatedGapPoints_.at(j);

                    if (propagatedGapPtJ->isAssignedToGap()) // skip if point is already assigned to a gap
                    {
                        // ROS_INFO_STREAM_NAMED("GapPropagator", "                       gap point is already assigned to a gap, continuing...");
                        continue;
                    }

                    if (propagatedGapPtJ->isRight())
                    {
                        // ROS_INFO_STREAM_NAMED("GapPropagator", "                       found a better right gap point, breaking...");
                        break;
                    } else
                    {
                        // ROS_INFO_STREAM_NAMED("GapPropagator", "                       creating an open gap from pt " << i << " to pt " << j);
                        // Create an open gap
                        propagatedGapPtI->assignToGap();
                        propagatedGapPtJ->assignToGap();

                        // Create a gap
                        currentGaps.push_back(new Gap(propagatedGapPtI->getFrame(),
                                                        *propagatedGapPtJ, *propagatedGapPtI, t_iplus1, true));

                        break;
                    }
                }
            } else if (propagatedGapPtI->isLeft()) // checking for reversed gap
            {
                // ROS_INFO_STREAM_NAMED("GapPropagator", "               gap point is right, looping for reversed gap");
                for (int adder = 1; adder < propagatedGapPoints_.size(); adder++) // inner loop through points
                {                        
                    int j = (i + adder) % propagatedGapPoints_.size();
                    // ROS_INFO_STREAM_NAMED("GapPropagator", "                   looping for inner gap point " << j);
                 
                    PropagatedGapPoint * propagatedGapPtJ = propagatedGapPoints_.at(j);

                    if (propagatedGapPtJ->isAssignedToGap()) // skip if point is already assigned to a gap
                    {
                        // ROS_INFO_STREAM_NAMED("GapPropagator", "                       gap point is already assigned to a gap, continuing...");                        
                        continue;
                    }

                    if (propagatedGapPtJ->isLeft())
                    {
                        // ROS_INFO_STREAM_NAMED("GapPropagator", "                       found a better right gap point, breaking...");
                        break;
                    } else
                    {
                        if (propagatedGapPtJ->getUngapID() != propagatedGapPtI->getUngapID())
                        {
                            // ROS_INFO_STREAM_NAMED("GapPropagator", "                       creating a reversed gap from pt " << i << " to pt " << j);                         
                            // Create a reversed gap
                            propagatedGapPtI->assignToGap();
                            propagatedGapPtJ->assignToGap();

                            // Create a gap
                            currentGaps.push_back(new Gap(propagatedGapPtI->getFrame(),
                                                    *propagatedGapPtI, *propagatedGapPtJ, t_iplus1, false));

                            break;
                        } else
                        {
                            // ROS_INFO_STREAM_NAMED("GapPropagator", "                       gap points form an ungap, continuing...");
                        }
                    }
                }
            } else
            {
                // ROS_WARN_STREAM_NAMED("GapPropagator", "        gap point is not left or right");
            }
        }

        ROS_INFO_STREAM_NAMED("GapPropagator", "                Gaps: ");
        for (int i = 0; i < currentGaps.size(); i++)
        {
            Gap * gap = currentGaps.at(i);
            // gap->setSafeToDelete();
            ROS_INFO_STREAM_NAMED("GapPropagator", "                    gap " << i << ": ");
            ROS_INFO_STREAM_NAMED("GapPropagator", "                        left point:" << gap->getLPosition().transpose());
            ROS_INFO_STREAM_NAMED("GapAssociator", "		                left ID: (" << gap->getLeftGapPt()->getModel()->getID() << ")");                        
            ROS_INFO_STREAM_NAMED("GapPropagator", "                        right point:" << gap->getRPosition().transpose());
            ROS_INFO_STREAM_NAMED("GapAssociator", "				        right ID: (" << gap->getRightGapPt()->getModel()->getID() << ")");
            ROS_INFO_STREAM_NAMED("GapPropagator", "                        gap available: " << gap->isAvailable());
        }        
    }

    void GapPropagator::convertGapsToGapPoints(const std::vector<Gap *> & gaps)
    {
        for (const Gap * gap : gaps)
        {
            // right
            gap->getRightGapPt()->getModel()->isolateGapDynamics();
            // float rightGapPtTheta = gap->getRightGapPt()->getModel()->getGapBearing();
            // int rightGapPtIdx = theta2idx(rightGapPtTheta);
            // int rightGapPtIdx = gap->RIdx();

            // if (rightGapPtIdx >= 0 && rightGapPtIdx < cfg_->scan.full_scan)
            propagatedGapPoints_.push_back(new PropagatedGapPoint(gap->getRightGapPt()->getModel(), gap->getFrame(), false));
            // else
                // ROS_WARN_STREAM_NAMED("GapPropagator", "        right gap pt idx out of bounds");

            // left
            gap->getLeftGapPt()->getModel()->isolateGapDynamics();
            // float leftGapPtTheta = gap->getLeftGapPt()->getModel()->getGapBearing();
            // int leftGapPtIdx = theta2idx(leftGapPtTheta);
            // int leftGapPtIdx = gap->LIdx();

            // if (leftGapPtIdx >= 0 && leftGapPtIdx < cfg_->scan.full_scan)
                propagatedGapPoints_.push_back(new PropagatedGapPoint(gap->getLeftGapPt()->getModel(), gap->getFrame(), true));
            // else
                // ROS_WARN_STREAM_NAMED("GapPropagator", "        left gap pt idx out of bounds");
        }
    }

    void GapPropagator::assignUnGapIDsToGapPoints()
    {
        for (int i = 0; i < propagatedGapPoints_.size(); i++)
        {
            Eigen::Vector4f ptIState = propagatedGapPoints_.at(i)->getModel()->getGapState();

            int nextIdx = (i + 1) % propagatedGapPoints_.size();
            Eigen::Vector4f ptJState = propagatedGapPoints_.at(nextIdx)->getModel()->getGapState();

            if (isUngap(ptIState, ptJState))
            {
                propagatedGapPoints_.at(i)->setUngapID(i);
                propagatedGapPoints_.at(nextIdx)->setUngapID(i);

                // set gap point states
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

        Eigen::Vector4f rewindLeftGapState = gap->getLeftGapPt()->getModel()->getGapState();
        Eigen::Vector4f rewindRightGapState = gap->getRightGapPt()->getModel()->getGapState();        

        Eigen::Vector2f leftRewindPt, rightRewindPt, leftBearingVect, rightBearingVect;
        float leftToRightAngle = 0.0;

        // instantiate model rewind states
        gap->getLeftGapPt()->getModel()->setRewindState();
        gap->getRightGapPt()->getModel()->setRewindState();
        // do rewindPropagate

        bool leftSideOpening = (getGapBearingRateOfChange(rewindLeftGapState) > 0.0);
        bool rightSideOpening = (getGapBearingRateOfChange(rewindRightGapState) < 0.0);

        // REWINDING THE GAP FROM ITS CROSSED CONFIGURATION UNTIL THE GAP IS SUFFICIENTLY OPEN
        for (float tRewind = (t - cfg_->traj.integrate_stept); tRewind >= 0.0; tRewind -= cfg_->traj.integrate_stept) 
        {
            // ROS_INFO_STREAM("                       tRewind: " << tRewind);

            // Rewind left gap point
            gap->getLeftGapPt()->getModel()->rewindPropagate(-1 * cfg_->traj.integrate_stept); // resetting model we used before, not good
            
            // Rewind right gap point
            gap->getRightGapPt()->getModel()->rewindPropagate(-1 * cfg_->traj.integrate_stept);

            rewindLeftGapState = gap->getLeftGapPt()->getModel()->getRewindGapState();
            rewindRightGapState = gap->getRightGapPt()->getModel()->getRewindGapState();  

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
