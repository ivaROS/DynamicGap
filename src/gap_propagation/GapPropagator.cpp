
#include <dynamic_gap/gap_propagation/GapPropagator.h>

namespace dynamic_gap 
{
    void GapPropagator::propagateGapPointsV2(const std::vector<Gap *> & manipulatedGaps,
                                            std::vector<GapTube *> & gapTubes) 
    {
        ROS_INFO_STREAM_NAMED("GapPropagator", "   [GapPropagator::propagateGapPointsV2()]");

        ROS_INFO_STREAM_NAMED("GapPropagator", "       (Manipulated) Gaps at time " << 0.0 << ": ");
        for (int i = 0; i < manipulatedGaps.size(); i++)
        {
            Gap * manipulatedGap = manipulatedGaps.at(i);

            manipulatedGap->getLeftGapPt()->getModel()->isolateGapDynamics();
            manipulatedGap->getLeftGapPt()->getModel()->isolateManipGapDynamics();
            manipulatedGap->getRightGapPt()->getModel()->isolateGapDynamics();
            manipulatedGap->getRightGapPt()->getModel()->isolateManipGapDynamics();

            ROS_INFO_STREAM_NAMED("GapPropagator", "           gap " << i << ": ");
            ROS_INFO_STREAM_NAMED("GapPropagator", "                left model:");
            ROS_INFO_STREAM_NAMED("GapPropagator", "		            ID: " << manipulatedGap->getLeftGapPt()->getModel()->getID());                        
            ROS_INFO_STREAM_NAMED("GapPropagator", "		            gap state: " << manipulatedGap->getLeftGapPt()->getModel()->getGapState().transpose());                        
            ROS_INFO_STREAM_NAMED("GapPropagator", "		            manip gap state: " << manipulatedGap->getLeftGapPt()->getModel()->getManipGapState().transpose());                        
            ROS_INFO_STREAM_NAMED("GapPropagator", "                left point:");
            ROS_INFO_STREAM_NAMED("GapPropagator", "                    orig: " << manipulatedGap->getLPosition().transpose());
            ROS_INFO_STREAM_NAMED("GapPropagator", "                    manip: " << manipulatedGap->getManipulatedLPosition().transpose());
            ROS_INFO_STREAM_NAMED("GapPropagator", "                left vel: ");
            ROS_INFO_STREAM_NAMED("GapPropagator", "                    orig: " << manipulatedGap->getLVelocity().transpose());
            ROS_INFO_STREAM_NAMED("GapPropagator", "                    manip: " << manipulatedGap->getManipulatedLVelocity().transpose());
            ROS_INFO_STREAM_NAMED("GapPropagator", "                right model:");
            ROS_INFO_STREAM_NAMED("GapPropagator", "		            ID: " << manipulatedGap->getRightGapPt()->getModel()->getID());                        
            ROS_INFO_STREAM_NAMED("GapPropagator", "		            gap state: " << manipulatedGap->getRightGapPt()->getModel()->getGapState().transpose());                        
            ROS_INFO_STREAM_NAMED("GapPropagator", "		            manip gap state: " << manipulatedGap->getRightGapPt()->getModel()->getManipGapState().transpose());                                                
            ROS_INFO_STREAM_NAMED("GapPropagator", "                right point:");
            ROS_INFO_STREAM_NAMED("GapPropagator", "                    orig:" << manipulatedGap->getRPosition().transpose());
            ROS_INFO_STREAM_NAMED("GapPropagator", "                    manip:" << manipulatedGap->getManipulatedRPosition().transpose());
            ROS_INFO_STREAM_NAMED("GapPropagator", "                right vel: ");
            ROS_INFO_STREAM_NAMED("GapPropagator", "                    orig: " << manipulatedGap->getRVelocity().transpose());
            ROS_INFO_STREAM_NAMED("GapPropagator", "                    manip: " << manipulatedGap->getManipulatedRVelocity().transpose());
        }

        // 1. Turn gaps into gap points
        convertGapsToGapPoints(manipulatedGaps); // CONSTRUCTING PROPAGATED GAP POINTS

        // 2. Sort gap points by bearing
        std::sort(propagatedGapPoints_.begin(), propagatedGapPoints_.end(), PropagatedGapPointComparator());

        std::vector<Gap *> currentGaps;
        float t_i = 0.0, t_iplus1 = 0.0;

        // create set of gaps at t=0.0
        // std::vector<Gap *> propagatedGaps;
        // for (Gap * manipulatedGap : manipulatedGaps)
        //     propagatedGaps.push_back(new Gap(*manipulatedGap));

        getGaps(currentGaps, t_iplus1);

        // create gap tubes
        gapTubes.resize(currentGaps.size());
        for (int i = 0; i < currentGaps.size(); i++)
            gapTubes.at(i) = new GapTube(currentGaps.at(i));

        // 4. Propagate gap points loop
        std::vector<Gap *> previousGaps = currentGaps; // SHALLOW COPY

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
            bool weird = runGapAssociation(currentGaps, previousGaps, t_iplus1, gapTubes);

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

            if (weird)
            {
                ROS_INFO_STREAM_NAMED("GapPropagator", "       Weird gap association, breaking out of loop");
                ROS_WARN_STREAM_NAMED("GapPropagator", "       Weird gap association, breaking out of loop");
                break;
            }
        }

        for (Gap * gap : previousGaps)
        {
            // if (gap->getSafeToDelete())
            // {
                delete gap;
            // }
        }


        // print out gap tubes
        for (int i = 0; i < gapTubes.size(); i++)
        {
            ROS_INFO_STREAM_NAMED("GapPropagator", "       Gap tube " << i << " (size " << gapTubes.at(i)->size() << "): ");
            gapTubes.at(i)->print();
        }

        //////////////
        // CLEAN UP //
        //////////////

        // delete gap points
        for (PropagatedGapPoint * propagatedGapPt : propagatedGapPoints_)
        {
            delete propagatedGapPt;
        }
        propagatedGapPoints_.clear();

        return;
    }

    bool GapPropagator::runGapAssociation(const std::vector<Gap *> & currentGaps, 
                                            const std::vector<Gap *> & previousGaps,
                                            const float & t_iplus1,
                                            std::vector<GapTube *> & gapTubes)
    {
        distMatrix_ = gapAssociator_->populateDistMatrix(currentGaps, previousGaps);
        assocation_ = gapAssociator_->associate(distMatrix_);
        bool weird = gapAssociator_->assignGaps(assocation_, distMatrix_, currentGaps, previousGaps, gapTubes, t_iplus1);

        return weird;
    }

    void GapPropagator::getGaps(std::vector<Gap *> & currentGaps,
                                const float & t_iplus1)
    {
        ROS_INFO_STREAM_NAMED("GapPropagator", "            [getGaps]");

        currentGaps.clear();

        // We might need to start with a right point
        int startRightIdx = 0;
        for (int i = 0; i < propagatedGapPoints_.size(); i++)
        {
            if (propagatedGapPoints_.at(i)->isRight())
            {
                startRightIdx = i;
                break;
            }
        }

        int counter = 0;
        int i = startRightIdx;

        while (counter < propagatedGapPoints_.size())
        {
        // for (int i = ; i < propagatedGapPoints_.size(); i++) // outer loop through points
        // {
            PropagatedGapPoint * propagatedGapPtI = propagatedGapPoints_.at(i);
            ROS_INFO_STREAM_NAMED("GapPropagator", "           looping for outer gap point " << i << " (ID " << propagatedGapPtI->getModel()->getID() << ")");

            if (propagatedGapPtI->isAssignedToGap()) // skip if point is already assigned to a gap
            {
                ROS_INFO_STREAM_NAMED("GapPropagator", "               gap point is already assigned to a gap, continuing...");

                i = (i + 1) % propagatedGapPoints_.size();
                counter++;
                continue;
            }

            if (propagatedGapPtI->isRight()) // checking for open gap (right to left)
            {
                ROS_INFO_STREAM_NAMED("GapPropagator", "               gap point is right, looping for open gap");
                for (int adder = 1; adder < propagatedGapPoints_.size(); adder++) // inner loop through points
                {
                    int j = (i + adder) % propagatedGapPoints_.size();                 
                    PropagatedGapPoint * propagatedGapPtJ = propagatedGapPoints_.at(j);
                    ROS_INFO_STREAM_NAMED("GapPropagator", "                   looping for inner gap point " << j << " (ID " << propagatedGapPtJ->getModel()->getID() << ")");

                    if (propagatedGapPtJ->isAssignedToGap()) // skip if point is already assigned to a gap
                    {
                        ROS_INFO_STREAM_NAMED("GapPropagator", "                       gap point is already assigned to a gap, continuing...");
                        continue;
                    }

                    if (propagatedGapPtJ->isRight())
                    {
                        ROS_INFO_STREAM_NAMED("GapPropagator", "                       found a better right gap point, breaking...");
                        break;
                    } else
                    {
                        ROS_INFO_STREAM_NAMED("GapPropagator", "                       creating an open gap from pt " << i << " to pt " << j);
                        // Create an open gap
                        propagatedGapPtI->assignToGap();
                        propagatedGapPtJ->assignToGap();

                        // Create a gap
                        currentGaps.push_back(new Gap(propagatedGapPtI->getFrame(),
                                                        *propagatedGapPtJ, *propagatedGapPtI, t_iplus1, true)); // (j, i): current left/right

                        break;
                    }
                }
            } else if (propagatedGapPtI->isLeft()) // checking for reversed gap
            {
                ROS_INFO_STREAM_NAMED("GapPropagator", "               gap point is right, looping for reversed gap");
                for (int adder = 1; adder < propagatedGapPoints_.size(); adder++) // inner loop through points
                {                        
                    int j = (i + adder) % propagatedGapPoints_.size();                 
                    PropagatedGapPoint * propagatedGapPtJ = propagatedGapPoints_.at(j);
                    ROS_INFO_STREAM_NAMED("GapPropagator", "                   looping for inner gap point " << j << " (ID " << propagatedGapPtJ->getModel()->getID() << ")");

                    if (propagatedGapPtJ->isAssignedToGap()) // skip if point is already assigned to a gap
                    {
                        ROS_INFO_STREAM_NAMED("GapPropagator", "                       gap point is already assigned to a gap, continuing...");                        
                        continue;
                    }

                    if (propagatedGapPtJ->isLeft())
                    {
                        ROS_INFO_STREAM_NAMED("GapPropagator", "                       found a better right gap point, breaking...");
                        break;
                    } else
                    {
                        if (propagatedGapPtJ->getUngapID() >= 0 && propagatedGapPtI->getUngapID() >= 0 &&
                            propagatedGapPtJ->getUngapID() == propagatedGapPtI->getUngapID())
                        {
                            ROS_INFO_STREAM_NAMED("GapPropagator", "                       gap points form an ungap (j ID: " << propagatedGapPtJ->getUngapID() << ", i ID: " << propagatedGapPtI->getUngapID() << "), continuing...");
                        } else
                        {
                            ROS_INFO_STREAM_NAMED("GapPropagator", "                       creating a reversed gap from pt " << i << " to pt " << j);                         
                            // Create a reversed gap
                            propagatedGapPtI->assignToGap();
                            propagatedGapPtJ->assignToGap();

                            // Create a gap
                            currentGaps.push_back(new Gap(propagatedGapPtI->getFrame(),
                                                            *propagatedGapPtJ, *propagatedGapPtI, t_iplus1, false)); // (j, i): current left/right

                            break;                            
                        }
                    }
                }
            } else
            {
                ROS_WARN_STREAM_NAMED("GapPropagator", "        gap point is not left or right");
            }

            i = (i + 1) % propagatedGapPoints_.size();
            counter++;
        }

        ROS_INFO_STREAM_NAMED("GapPropagator", "                Gaps: ");
        for (int i = 0; i < currentGaps.size(); i++)
        {
            Gap * gap = currentGaps.at(i);
            // gap->setSafeToDelete();
            ROS_INFO_STREAM_NAMED("GapPropagator", "                    gap " << i << ": ");
            ROS_INFO_STREAM_NAMED("GapPropagator", "                        manip left point:" << gap->getManipulatedLPosition().transpose());
            ROS_INFO_STREAM_NAMED("GapPropagator", "                        manip left vel:" << gap->getManipulatedLVelocity().transpose());
            ROS_INFO_STREAM_NAMED("GapPropagator", "		                left ID: (" << gap->getLeftGapPt()->getModel()->getID() << ")");                        
            ROS_INFO_STREAM_NAMED("GapPropagator", "                        manip right point:" << gap->getManipulatedRPosition().transpose());
            ROS_INFO_STREAM_NAMED("GapPropagator", "                        manip right vel:" << gap->getManipulatedRVelocity().transpose());
            ROS_INFO_STREAM_NAMED("GapPropagator", "				        right ID: (" << gap->getRightGapPt()->getModel()->getID() << ")");
            ROS_INFO_STREAM_NAMED("GapPropagator", "                        gap available: " << gap->isAvailable());
        }        
    }

    void GapPropagator::convertGapsToGapPoints(const std::vector<Gap *> & gaps)
    {
        for (const Gap * gap : gaps)
        {
            // right
            // gap->getRightGapPt()->getModel()->isolateGapDynamics();
            // gap->getRightGapPt()->getModel()->isolateManipGapDynamics();
            // float rightGapPtTheta = gap->getRightGapPt()->getModel()->getGapBearing();
            // int rightGapPtIdx = theta2idx(rightGapPtTheta);
            // int rightGapPtIdx = gap->RIdx();

            // if (rightGapPtIdx >= 0 && rightGapPtIdx < cfg_->scan.full_scan)
            propagatedGapPoints_.push_back(new PropagatedGapPoint(gap->getRightGapPt()->getModel(), 
                                                                    gap->getFrame(), 
                                                                    gap->getRightGapPt()->getUngapID(),        
                                                                    false));
            // else
                // ROS_WARN_STREAM_NAMED("GapPropagator", "        right gap pt idx out of bounds");

            // left
            // gap->getLeftGapPt()->getModel()->isolateGapDynamics();
            // gap->getLeftGapPt()->getModel()->isolateManipGapDynamics();
            // float leftGapPtTheta = gap->getLeftGapPt()->getModel()->getGapBearing();
            // int leftGapPtIdx = theta2idx(leftGapPtTheta);
            // int leftGapPtIdx = gap->LIdx();

            // if (leftGapPtIdx >= 0 && leftGapPtIdx < cfg_->scan.full_scan)
            propagatedGapPoints_.push_back(new PropagatedGapPoint(gap->getLeftGapPt()->getModel(), 
                                                                    gap->getFrame(), 
                                                                    gap->getLeftGapPt()->getUngapID(),
                                                                    true));
            // else
                // ROS_WARN_STREAM_NAMED("GapPropagator", "        left gap pt idx out of bounds");
        }
    }

    // void GapPropagator::assignUnGapIDsToGapPoints()
    // {
    //     for (int i = 0; i < propagatedGapPoints_.size(); i++)
    //     {
    //         Eigen::Vector4f ptIState = propagatedGapPoints_.at(i)->getModel()->getGapState();

    //         int nextIdx = (i + 1) % propagatedGapPoints_.size();
    //         Eigen::Vector4f ptJState = propagatedGapPoints_.at(nextIdx)->getModel()->getGapState();

    //         if (isUngap(ptIState, ptJState))
    //         {
    //             propagatedGapPoints_.at(i)->setUngapID(i);
    //             propagatedGapPoints_.at(nextIdx)->setUngapID(i);

    //             // set gap point states
    //         }
    //     }
    // }

    // bool GapPropagator::isUngap(const Eigen::Vector4f & ptIState, const Eigen::Vector4f & ptJState)
    // {
    //     Eigen::Vector2f ptIPos = ptIState.head(2);
    //     Eigen::Vector2f ptJPos = ptJState.head(2);

    //     Eigen::Vector2f ptIVel = ptIState.tail(2);
    //     Eigen::Vector2f ptJVel = ptJState.tail(2);

    //     // check if distance between points is less than 4 * r_inscr * inf_ratio
    //     bool distCheck = (ptIPos - ptJPos).norm() < 4 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

    //     // // check if speed of points is greater than 0.10
    //     bool speedCheck = (ptIVel.norm() >= 0.10 && ptJVel.norm() >= 0.10);

    //     // // run angle check on LHS and RHS model velocities
    //     float vectorProj = ptIVel.dot(ptJVel) / (ptIVel.norm() * ptJVel.norm() + eps);
    //     bool angleCheck = (vectorProj > 0.0);

    //     return (distCheck && speedCheck && angleCheck);
    // }

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
