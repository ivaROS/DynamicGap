#include <dynamic_gap/trajectory_generation/GapGoalPlacer.h>

namespace dynamic_gap
{
    //////////////////////////////////////////////////////////////////////
    // Constructor moved out of the header because it now needs a node
    // handle for the goal-skew marker publisher. Same pattern as
    // GapPropagator::GapPropagator(nh, cfg).
    //////////////////////////////////////////////////////////////////////

    GapGoalPlacer::GapGoalPlacer(ros::NodeHandle & nh, const DynamicGapConfig & cfg)
    {
        cfg_ = &cfg;

        goalSkewMarkerPublisher_ =
            nh.advertise<visualization_msgs::MarkerArray>("goal_placement_skew", 10);

        ROS_INFO_STREAM_NAMED("GoalPlacementSkew",
            "GapGoalPlacer goal placement skew enabled: " << useGoalPlacementSkew_
            << ", lookahead T: " << goalSkewLookaheadTime_ << " s"
            << ", max delta: " << goalSkewMaxDelta_
            << ", k limits: [" << goalSkewMinFraction_
            << ", " << goalSkewMaxFraction_ << "]");
    }

    //////////////////////////////////////////////////////////////////////
    // Call once per planning cycle, at the top of
    // Planner::gapGoalPlacementV2(), BEFORE the gap tube loop. Without it
    // the markers from previous cycles accumulate in RViz.
    //////////////////////////////////////////////////////////////////////

    void GapGoalPlacer::resetGoalSkewMarkers()
    {
        goalSkewMarkerIdx_ = 0;

        if (!publishGoalSkewMarkers_)
            return;

        visualization_msgs::MarkerArray clearMarkerArray;
        visualization_msgs::Marker clearMarker;

        clearMarker.action = visualization_msgs::Marker::DELETEALL;
        clearMarkerArray.markers.push_back(clearMarker);

        goalSkewMarkerPublisher_.publish(clearMarkerArray);
    }

    void GapGoalPlacer::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    // void GapGoalPlacer::setGapGoal(Gap * gap, 
    //                                 const geometry_msgs::PoseStamped & globalPathLocalWaypointRobotFrame, 
    //                                 const geometry_msgs::PoseStamped & globalGoalRobotFrame) 
    // {
    //     ROS_INFO_STREAM_NAMED("GapGoalPlacer", "    [setGapGoal()]");

    //     int leftIdx = gap->manipLeftIdx();
    //     int rightIdx = gap->manipRightIdx();
    //     float leftRange = gap->manipLeftRange();
    //     float rightRange = gap->manipRightRange();

    //     float leftTheta = idx2theta(leftIdx);
    //     float rightTheta = idx2theta(rightIdx);

    //     float xLeft = (leftRange) * cos(leftTheta);
    //     float yLeft = (leftRange) * sin(leftTheta);
    //     float xRight = (rightRange) * cos(rightTheta);
    //     float yRight = (rightRange) * sin(rightTheta);

    //     Eigen::Vector2f leftPt(xLeft, yLeft);
    //     Eigen::Vector2f rightPt(xRight, yRight);

    //     gap->getLeftGapPt()->getModel()->isolateGapDynamics();
    //     gap->getRightGapPt()->getModel()->isolateGapDynamics();

    //     Eigen::Vector4f leftGapState = gap->getLeftGapPt()->getModel()->getGapState();
    //     Eigen::Vector4f rightGapState = gap->getRightGapPt()->getModel()->getGapState();

    //     ROS_INFO_STREAM_NAMED("GapGoalPlacer", "        gap polar points, left: (" << leftIdx << ", " << leftRange << ") , right: (" << rightIdx << ", " << rightRange << ")");
    //     ROS_INFO_STREAM_NAMED("GapGoalPlacer", "        gap cart points, left: (" << xLeft << ", " << yLeft << ") , right: (" << xRight << ", " << yRight << ")");

    //     float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);

    //     Eigen::Vector2f globalGoalRobotFrameVector(globalGoalRobotFrame.pose.position.x, 
    //                             globalGoalRobotFrame.pose.position.y);

    //     float globalGoalTheta = std::atan2(globalGoalRobotFrameVector[1], globalGoalRobotFrameVector[0]);
    //     float globalGoalIdx = theta2idx(globalGoalTheta); // std::floor(goal_orientation*half_num_scan/M_PI + half_num_scan);

    //     // ROS_INFO_STREAM("        global goal idx: " << globalGoalIdx << 
    //             // ", global goal: (" << globalGoalRobotFrameVector[0] << 
    //             //                  ", " << globalGoalRobotFrameVector[1] << ")");


    //     // Check if global goal is within current scan
    //     //      - previously, we also checked if global goal was within *gap*,
    //     //        but in a corridor or corner, the global goal will not always be contained
    //     //        within one of our gaps. Therefore, we will perform a lazy check
    //     //        to enable the planner to run g2g if the global goal is within the scan,
    //     //        and then we can evaluate whether or not the path is fine later
    //     if (checkWaypointVisibility(leftPt, rightPt, globalGoalRobotFrameVector))
    //     {                
    //         // all we will do is mark it for later so we can run g2g policy on global goal.
    //         // Still set mid point for pursuit guidance policy and feasibility check
    //         gap->setGlobalGoalWithin();

    //         ROS_INFO_STREAM_NAMED("GapGoalPlacer", "        global goal within gap");
    //         ROS_INFO_STREAM_NAMED("GapGoalPlacer", "            goal: " << globalGoalRobotFrameVector[0] << 
    //                                                 ", " << globalGoalRobotFrameVector[1]);

    //     }

    //     if (leftToRightAngle < M_PI) // M_PI / 2,  M_PI / 4
    //     {
    //         ROS_INFO_STREAM_NAMED("GapGoalPlacer", "        Option 1: gap mid point");

    //         float centerTheta = leftTheta - (0.5 * leftToRightAngle);
    //         float centerRange = 0.5 * (leftRange + rightRange);
    //         Eigen::Vector2f centerPt(centerRange * std::cos(centerTheta),
    //                                     centerRange * std::sin(centerTheta));
    //         Eigen::Vector2f centerVel = 0.5 * (leftGapState.tail(2) + rightGapState.tail(2));

    //         ROS_INFO_STREAM_NAMED("GapGoalPlacer", "            original goal: " << centerPt[0] << ", " << centerPt[1]);                 

    //         Eigen::Vector2f gapGoalRadialOffset = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * centerPt.normalized();

    //         Eigen::Vector2f inflatedCenterPt = centerPt + gapGoalRadialOffset;

    //         ROS_INFO_STREAM_NAMED("GapGoalPlacer", "            inflated goal: " << inflatedCenterPt[0] << ", " << inflatedCenterPt[1]);                 

    //         gap->getGoal()->setOrigGoalPos(inflatedCenterPt);
    //         gap->getGoal()->setOrigGoalVel(centerVel);
    //         // gap->setGoal(inflatedCenterPt);
    //         // gap->setGoalVel(centerVel);
    //     } else
    //     {
    //         ROS_INFO_STREAM_NAMED("GapGoalPlacer", "        Option 2: global path local waypoint biased");

    //         Eigen::Vector2f globalPathLocalWaypointRobotFrameVector(globalPathLocalWaypointRobotFrame.pose.position.x, 
    //                                                 globalPathLocalWaypointRobotFrame.pose.position.y);
    //         float globalPathLocalWaypointTheta = std::atan2(globalPathLocalWaypointRobotFrameVector[1], globalPathLocalWaypointRobotFrameVector[0]);

    //         float leftToWaypointAngle = getSweptLeftToRightAngle(leftPt, globalPathLocalWaypointRobotFrameVector);
    //         float rightToWaypointAngle = getSweptLeftToRightAngle(rightPt, globalPathLocalWaypointRobotFrameVector);

    //         float biasedGapGoalTheta = setBiasedGapGoalTheta(leftTheta, rightTheta, globalPathLocalWaypointTheta,
    //                                         leftToRightAngle, leftToWaypointAngle, rightToWaypointAngle);
    //         Eigen::Vector2f biasedGapGoalUnitNorm(std::cos(biasedGapGoalTheta), std::sin(biasedGapGoalTheta));

    //         float leftToGapGoalAngle = getSweptLeftToRightAngle(leftPt, biasedGapGoalUnitNorm); 

    //         // float biasedGapGoalIdx = theta2idx(biasedGapGoalTheta); // std::floor(biasedGapGoalTheta*half_num_scan/M_PI + half_num_scan);

    //         float biasedGapGoalDist = leftRange + (rightRange - leftRange) * leftToGapGoalAngle / leftToRightAngle;
    //         Eigen::Vector2f biasedGapGoal(biasedGapGoalDist * cos(biasedGapGoalTheta), biasedGapGoalDist * sin(biasedGapGoalTheta));
    //         Eigen::Vector2f biasedGapVel = leftGapState.tail(2) + (rightGapState.tail(2) - leftGapState.tail(2)) * leftToGapGoalAngle / leftToRightAngle;

    //         ROS_INFO_STREAM_NAMED("GapGoalPlacer", "            original goal: " << biasedGapGoal[0] << ", " << biasedGapGoal[1]);                 

    //         Eigen::Vector2f gapGoalRadialOffset = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * biasedGapGoal.normalized();

    //         Eigen::Vector2f inflatedBiasedGapGoal = biasedGapGoal + gapGoalRadialOffset;

    //         ROS_INFO_STREAM_NAMED("GapGoalPlacer", "            inflated goal: " << inflatedBiasedGapGoal[0] << ", " << inflatedBiasedGapGoal[1]);                 

    //         gap->getGoal()->setOrigGoalPos(inflatedBiasedGapGoal);
    //         gap->getGoal()->setOrigGoalVel(biasedGapVel);
    //         // gap->setGoal(inflatedBiasedGapGoal);
    //         // gap->setGoalVel(biasedGapVel);
    //     }      
    // }

    void GapGoalPlacer::setGapGoalFromPriorV2(Gap * gap,
                                                Gap * priorGap)
    {
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "          [setGapGoalFromPriorV2()]");

        int leftIdx = gap->manipLeftIdx();
        int rightIdx = gap->manipRightIdx();
        float leftRange = gap->manipLeftRange();
        float rightRange = gap->manipRightRange();

        // gap->getLeftGapPt()->getModel()->isolateManipGapDynamics();
        // gap->getRightGapPt()->getModel()->isolateManipGapDynamics();

        // Eigen::Vector4f leftManipGapState = gap->getLeftGapPt()->getModel()->getManipGapState();
        // Eigen::Vector4f rightManipGapState = gap->getRightGapPt()->getModel()->getManipGapState();

        // Eigen::Vector2f leftPt = leftManipGapState.head(2);
        // Eigen::Vector2f leftVel = leftManipGapState.tail(2);
        // Eigen::Vector2f rightPt = rightManipGapState.head(2);
        // Eigen::Vector2f rightVel = rightManipGapState.tail(2);

        Eigen::Vector2f leftPt = gap->getManipulatedLPosition();
        Eigen::Vector2f leftVel = gap->getManipulatedLVelocity();
        Eigen::Vector2f rightPt = gap->getManipulatedRPosition();
        Eigen::Vector2f rightVel = gap->getManipulatedRVelocity();

        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              gap polar points, left: (" << leftIdx << ", " << leftRange << ") , right: (" << rightIdx << ", " << rightRange << ")");
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              gap cart points, left: (" << leftPt.transpose() << ") , right: (" << rightPt.transpose() << ")");
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              gap cart vels, left: (" << leftVel.transpose() << ") , right: (" << rightVel.transpose() << ")");

        Eigen::Vector2f priorGapGoalPos = priorGap->getGoal()->getOrigGoalPos();
        Eigen::Vector2f priorGapGoalVel = priorGap->getGoal()->getOrigGoalVel();
        float priorGapLifespan = priorGap->getGapLifespan();

        Eigen::Vector2f gapGoalPos = priorGapGoalPos + priorGapGoalVel * priorGapLifespan;
        Eigen::Vector2f gapGoalVel(0.0, 0.0);

        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              priorGapGoalPos: (" << priorGapGoalPos.transpose() << ")");
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              priorGapGoalVel: (" << priorGapGoalVel.transpose() << ")");
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              priorGapLifespan: " << priorGapLifespan);

        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              gapGoalPos: (" << gapGoalPos.transpose() << ")");
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              gapGoalVel: (" << gapGoalVel.transpose() << ")");

        gap->getGoal()->setOrigGoalPos( gapGoalPos );
        gap->getGoal()->setOrigGoalVel( gapGoalVel );
    }

    void GapGoalPlacer::setGapGoalFromNextV2(Gap * gap,
                                                Gap * nextGap)
    {
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "          [setGapGoalFromNextV2()]");

        int leftIdx = nextGap->manipLeftIdx();
        int rightIdx = nextGap->manipRightIdx();
        float leftRange = nextGap->manipLeftRange();
        float rightRange = nextGap->manipRightRange();

        float leftTheta = idx2theta(leftIdx);
        float rightTheta = idx2theta(rightIdx);

        Eigen::Vector2f leftPt = nextGap->getManipulatedLPosition();
        Eigen::Vector2f leftVel = nextGap->getManipulatedLVelocity();
        Eigen::Vector2f rightPt = nextGap->getManipulatedRPosition();
        Eigen::Vector2f rightVel = nextGap->getManipulatedRVelocity();

        float leftToRightAngle = getSignedLeftToRightAngle(leftPt, rightPt); // we want this to be able to be negative

        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              nextGap polar points, left: (" << leftIdx << ", " << leftRange << ") , right: (" << rightIdx << ", " << rightRange << ")");
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              nextGap cart points, left: (" << leftPt.transpose() << ") , right: (" << rightPt.transpose() << ")");
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              nextGap cart vels, left: (" << leftVel.transpose() << ") , right: (" << rightVel.transpose() << ")");

        float centerTheta = leftTheta - (0.5 * leftToRightAngle);
        float minRange = std::min(leftRange, rightRange);

        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              centerTheta: " << centerTheta);
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              minRange: " << minRange);

        Eigen::Vector2f gapGoalPos = minRange * Eigen::Vector2f(std::cos(centerTheta), std::sin(centerTheta));

        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              gapGoalPos: (" << gapGoalPos.transpose() << ")");

        Eigen::Vector2f gapGoalRadialOffset = - 2.0 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * gapGoalPos.normalized();

        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              gapGoalRadialOffset: (" << gapGoalRadialOffset.transpose() << ")");

        Eigen::Vector2f inflatedGapGoalPos = gapGoalPos + gapGoalRadialOffset;

        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              inflatedGapGoalPos: (" << inflatedGapGoalPos.transpose() << ")");

        Eigen::Vector2f gapGoalVel(0.0, 0.0);

        gap->getGoal()->setOrigGoalPos( inflatedGapGoalPos );
        gap->getGoal()->setOrigGoalVel( gapGoalVel );
    }

    void GapGoalPlacer::setGapGoalV2(Gap * gap, 
                                        const geometry_msgs::PoseStamped & globalPathLocalWaypointRobotFrame, 
                                        const geometry_msgs::PoseStamped & globalGoalRobotFrame) 
    {
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "          [setGapGoalV2()]");

        int leftIdx = gap->manipLeftIdx();
        int rightIdx = gap->manipRightIdx();
        float leftRange = gap->manipLeftRange();
        float rightRange = gap->manipRightRange();

        float leftTheta = idx2theta(leftIdx);
        float rightTheta = idx2theta(rightIdx);

        // gap->getLeftGapPt()->getModel()->isolateManipGapDynamics();
        // gap->getRightGapPt()->getModel()->isolateManipGapDynamics();

        // Eigen::Vector4f leftManipGapState = gap->getLeftGapPt()->getModel()->getManipGapState();
        // Eigen::Vector4f rightManipGapState = gap->getRightGapPt()->getModel()->getManipGapState();

        Eigen::Vector2f leftPt = gap->getManipulatedLPosition();
        Eigen::Vector2f leftVel = gap->getManipulatedLVelocity();
        Eigen::Vector2f rightPt = gap->getManipulatedRPosition();
        Eigen::Vector2f rightVel = gap->getManipulatedRVelocity();

        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              gap polar points, left: (" << leftIdx << ", " << leftRange << ") , right: (" << rightIdx << ", " << rightRange << ")");
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              gap cart points, left: (" << leftPt.transpose() << ") , right: (" << rightPt.transpose() << ")");
        ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              gap cart vels, left: (" << leftVel.transpose() << ") , right: (" << rightVel.transpose() << ")");

        float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);

        Eigen::Vector2f globalGoalRobotFrameVector(globalGoalRobotFrame.pose.position.x, 
                                                    globalGoalRobotFrame.pose.position.y);

        float globalGoalTheta = std::atan2(globalGoalRobotFrameVector[1], globalGoalRobotFrameVector[0]);
        float globalGoalIdx = theta2idx(globalGoalTheta); // std::floor(goal_orientation*half_num_scan/M_PI + half_num_scan);

        // ROS_INFO_STREAM("        global goal idx: " << globalGoalIdx << 
                // ", global goal: (" << globalGoalRobotFrameVector[0] << 
                //                  ", " << globalGoalRobotFrameVector[1] << ")");


        // Check if global goal is within current scan
        //      - previously, we also checked if global goal was within *gap*,
        //        but in a corridor or corner, the global goal will not always be contained
        //        within one of our gaps. Therefore, we will perform a lazy check
        //        to enable the planner to run g2g if the global goal is within the scan,
        //        and then we can evaluate whether or not the path is fine later
        if (checkWaypointVisibility(leftPt, rightPt, globalGoalRobotFrameVector))
        {                
            // all we will do is mark it for later so we can run g2g policy on global goal.
            // Still set mid point for pursuit guidance policy and feasibility check
            gap->setGlobalGoalWithin();

            ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              global goal within gap");
            ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              goal: " << globalGoalRobotFrameVector[0] << 
                                                                        ", " << globalGoalRobotFrameVector[1]);

        }

        if (leftToRightAngle < M_PI) // M_PI / 2,  M_PI / 4
        {
           ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              Option 1: gap mid point");

            //////////////////////////////////////////////////////////
            //                  GOAL PLACEMENT SKEW                 //
            //////////////////////////////////////////////////////////

            const float kBase = 0.5f;

            float closingRateDiff = 0.0f;
            float skewGapWidth = -1.0f;
            float dKApplied = 0.0f;

            if (useGoalPlacementSkew_)
            {
                const float dK = computeGoalSkewDelta(gap, closingRateDiff, skewGapWidth);
                const float kSkewed = clampGoalFraction(kBase + dK);

                dKApplied = kSkewed - kBase;
            }

            const float kGoal = kBase + dKApplied;

            ROS_INFO_STREAM_NAMED("GoalPlacementSkew",
                "              [Option 1] kBase: " << kBase
                << ", dKApplied: " << dKApplied
                << ", kGoal: " << kGoal);

            // Base (unskewed) point. Used for the RViz markers only.
            // With the skew disabled this is bit-for-bit the original goal:
            //   leftRange + (rightRange - leftRange) * 0.5 == 0.5 * (leftRange + rightRange)
            const float baseTheta = leftTheta - (kBase * leftToRightAngle);
            const float baseRange = leftRange + (rightRange - leftRange) * kBase;

            const Eigen::Vector2f baseCenterPt(baseRange * std::cos(baseTheta),
                                                baseRange * std::sin(baseTheta));

            // The skew is applied as an ANGULAR OFFSET on the base angle. Do not
            // rebuild the angle from kGoal -- that would depend on the swept-angle
            // wrapping convention of getSweptLeftToRightAngle().
            float centerTheta = baseTheta - (dKApplied * leftToRightAngle);
            float centerRange = leftRange + (rightRange - leftRange) * kGoal;

            Eigen::Vector2f centerPt(centerRange * std::cos(centerTheta),
                                        centerRange * std::sin(centerTheta));

            // leftVel and rightVel come from getManipulatedLVelocity(), which is
            // always (0, 0) for a gap point. centerVel is therefore zero, exactly
            // as before this change. That is deliberate: making the goal velocity
            // non-zero would change the feasibility check at the same time as the
            // skew. See implementation_plan.md section 2.
            Eigen::Vector2f centerVel = leftVel + (rightVel - leftVel) * kGoal;

            if (publishGoalSkewMarkers_)
                publishGoalSkewMarkers(gap, baseCenterPt, centerPt);

            ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              original goal: " << centerPt[0] << ", " << centerPt[1]);                 

            Eigen::Vector2f gapGoalRadialOffset = 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * centerPt.normalized();
            Eigen::Vector2f inflatedCenterPt = centerPt + gapGoalRadialOffset;

            // check if goal is beyond scan
            int centerIdx = theta2idx(centerTheta);
            float centerRangeScan = scan_->ranges.at(centerIdx);
            float inflatedCenterRangeScan = centerRangeScan - 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; 
            if (inflatedCenterRangeScan < inflatedCenterPt.norm())
            {
                inflatedCenterPt = inflatedCenterRangeScan * centerPt.normalized();
            }


            Eigen::Vector2f scaledCenterVel = centerVel * (inflatedCenterPt.norm() / centerPt.norm()); // scale by r?

            gap->getGoal()->setOrigGoalPos(inflatedCenterPt);
            gap->getGoal()->setOrigGoalVel(scaledCenterVel); 
            ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              inflated goal: " << inflatedCenterPt[0] << ", " << inflatedCenterPt[1]);                 

        } else
        {
            ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              Option 2: global path local waypoint biased");

            Eigen::Vector2f globalPathLocalWaypointRobotFrameVector(globalPathLocalWaypointRobotFrame.pose.position.x, 
                                                    globalPathLocalWaypointRobotFrame.pose.position.y);
            float globalPathLocalWaypointTheta = std::atan2(globalPathLocalWaypointRobotFrameVector[1], globalPathLocalWaypointRobotFrameVector[0]);

            float leftToWaypointAngle = getSweptLeftToRightAngle(leftPt, globalPathLocalWaypointRobotFrameVector);
            float rightToWaypointAngle = getSweptLeftToRightAngle(rightPt, globalPathLocalWaypointRobotFrameVector);

            float biasedGapGoalTheta = setBiasedGapGoalTheta(leftTheta, rightTheta, globalPathLocalWaypointTheta,
                                            leftToRightAngle, leftToWaypointAngle, rightToWaypointAngle);
            Eigen::Vector2f biasedGapGoalUnitNorm(std::cos(biasedGapGoalTheta), std::sin(biasedGapGoalTheta));

            float leftToGapGoalAngle = getSweptLeftToRightAngle(leftPt, biasedGapGoalUnitNorm);

            //////////////////////////////////////////////////////////
            //                  GOAL PLACEMENT SKEW                 //
            //////////////////////////////////////////////////////////
            //
            // Option 2 is the same interpolation as Option 1 with a different
            // base fraction. Here the base fraction comes from the global-goal
            // bias instead of being pinned at 0.5.
            //
            //////////////////////////////////////////////////////////

            const float kBase = (std::fabs(leftToRightAngle) > 1e-6f)
                                    ? (leftToGapGoalAngle / leftToRightAngle)
                                    : 0.5f;

            float closingRateDiff = 0.0f;
            float skewGapWidth = -1.0f;
            float dKApplied = 0.0f;

            if (useGoalPlacementSkew_)
            {
                const float dK = computeGoalSkewDelta(gap, closingRateDiff, skewGapWidth);
                const float kSkewed = clampGoalFraction(kBase + dK);

                dKApplied = kSkewed - kBase;
            }

            const float kGoal = kBase + dKApplied;

            ROS_INFO_STREAM_NAMED("GoalPlacementSkew",
                "              [Option 2] kBase: " << kBase
                << ", dKApplied: " << dKApplied
                << ", kGoal: " << kGoal);

            // Base (unskewed) goal. Used for the RViz markers only.
            const float baseGapGoalDist = leftRange + (rightRange - leftRange) * kBase;

            const Eigen::Vector2f baseGapGoal(baseGapGoalDist * std::cos(biasedGapGoalTheta),
                                                baseGapGoalDist * std::sin(biasedGapGoalTheta));

            // Angular offset on the angle setBiasedGapGoalTheta() already returned.
            const float skewedGapGoalTheta =
                biasedGapGoalTheta - (dKApplied * leftToRightAngle);

            float biasedGapGoalDist = leftRange + (rightRange - leftRange) * kGoal;

            Eigen::Vector2f biasedGapGoal(biasedGapGoalDist * std::cos(skewedGapGoalTheta),
                                            biasedGapGoalDist * std::sin(skewedGapGoalTheta));

            Eigen::Vector2f biasedGapVel = leftVel + (rightVel - leftVel) * kGoal;

            if (publishGoalSkewMarkers_)
                publishGoalSkewMarkers(gap, baseGapGoal, biasedGapGoal);

            ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              original goal: " << biasedGapGoal[0] << ", " << biasedGapGoal[1]);                 

            Eigen::Vector2f gapGoalRadialOffset = 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * biasedGapGoal.normalized();

            Eigen::Vector2f inflatedBiasedGapGoal = biasedGapGoal + gapGoalRadialOffset;

            // check if goal is beyond scan
            // skewedGapGoalTheta, not biasedGapGoalTheta -- the goal has moved
            int biasedGapGoalIdx = theta2idx(skewedGapGoalTheta);
            float biasedGapGoalRangeScan = scan_->ranges.at(biasedGapGoalIdx);
            float inflatedBiasedGapGoalRangeScan = biasedGapGoalRangeScan - 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; 
            if (inflatedBiasedGapGoalRangeScan < inflatedBiasedGapGoal.norm())
            {
                inflatedBiasedGapGoal = inflatedBiasedGapGoalRangeScan * biasedGapGoal.normalized();
            }

            Eigen::Vector2f scaledBiasedVel = biasedGapVel * (inflatedBiasedGapGoal.norm() / biasedGapGoal.norm()); // scale by r?

            ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              inflated biased gap goal: " << inflatedBiasedGapGoal[0] << ", " << inflatedBiasedGapGoal[1]);                 

            gap->getGoal()->setOrigGoalPos(inflatedBiasedGapGoal);
            gap->getGoal()->setOrigGoalVel(scaledBiasedVel);
        }      
    }
    
    float GapGoalPlacer::setBiasedGapGoalTheta(const float & leftTheta, const float & rightTheta, const float & globalGoalTheta,
                                                const float & leftToRightAngle, const float & leftToWaypointAngle,  const float & rightToWaypointAngle)
    {
        float biasedGapGoalTheta = 0.0;
        if (leftTheta > rightTheta) // gap is not behind robot
        { 
            biasedGapGoalTheta = std::min(leftTheta, std::max(rightTheta, globalGoalTheta));
        } else // gap is behind
        { 
            if (0 < leftToWaypointAngle && leftToWaypointAngle < leftToRightAngle)
                biasedGapGoalTheta = globalGoalTheta;
            else if (std::abs(leftToWaypointAngle) < std::abs(rightToWaypointAngle))
                biasedGapGoalTheta = leftTheta;
            else
                biasedGapGoalTheta = rightTheta;
        }

        // ROS_INFO_STREAM("            leftTheta: " << leftTheta << ", rightTheta: " << rightTheta << ", globalGoalTheta: " << globalGoalTheta);
        // ROS_INFO_STREAM("            leftToRightAngle: " << leftToRightAngle << ", leftToWaypointAngle: " << leftToWaypointAngle << ", rightToWaypointAngle: " << rightToWaypointAngle);

        return biasedGapGoalTheta;
    }

        //////////////////////////////////////////////////////////////////////
    //                      GOAL PLACEMENT SKEW                         //
    //////////////////////////////////////////////////////////////////////
    //
    // Lateral shift of the gap goal, in fraction-of-aperture units.
    //
    //   c_hat = (leftPt - rightPt).normalized()      right point -> left point
    //
    //   s_L = -leftGapVel.dot(c_hat)     left  endpoint closing speed, > 0 = inward
    //   s_R =  rightGapVel.dot(c_hat)    right endpoint closing speed, > 0 = inward
    //
    //   dK  = 0.5 * (s_L - s_R) * T / W
    //
    // (s_L - s_R) equals -2x the lateral drift of the aperture mid point
    // along c_hat. So dK predicts where the aperture centre will be after T
    // seconds, as a fraction of the aperture width. It is not a tuned bias.
    //
    //   dK > 0  ->  move the goal toward the RIGHT gap point.
    //
    // VELOCITY SOURCE. This reads getLVelocity() and getRVelocity().
    // It must NOT read getManipulatedLVelocity().
    // Estimator::isolateManipGapDynamics() contains:
    //
    //     if (rgc_ || !ungap_) { xManipFrozen_[2] = 0.0; xManipFrozen_[3] = 0.0; }
    //
    // An ordinary gap point has ungap_ == false, so !ungap_ is true, so the
    // velocity is always discarded. getManipulatedLVelocity() therefore
    // returns (0, 0) on every cycle. getGapVelocity(), which getLVelocity()
    // forwards to, is filled by isolateGapDynamics() and has the linear ego
    // motion added back, so it is the true non-relative gap velocity.
    //
    //////////////////////////////////////////////////////////////////////

    float GapGoalPlacer::computeGoalSkewDelta(Gap * gap,
                                                float & closingRateDiffOut,
                                                float & gapWidthOut)
    {
        closingRateDiffOut = 0.0f;
        gapWidthOut = -1.0f;

        if (!gap)
        {
            ROS_WARN_STREAM_THROTTLE_NAMED(5.0, "GoalPlacementSkew",
                "null gap, skipping goal placement skew");

            return 0.0f;
        }

        // geometry: MANIPULATED points, because that is the aperture the goal lives in
        const Eigen::Vector2f leftPt  = gap->getManipulatedLPosition();
        const Eigen::Vector2f rightPt = gap->getManipulatedRPosition();

        // dynamics: RAW gap velocities. See the note above.
        const Eigen::Vector2f leftGapVel  = gap->getLVelocity();
        const Eigen::Vector2f rightGapVel = gap->getRVelocity();

        if (!leftPt.allFinite()      || !rightPt.allFinite() ||
            !leftGapVel.allFinite()  || !rightGapVel.allFinite())
        {
            ROS_WARN_STREAM_THROTTLE_NAMED(5.0, "GoalPlacementSkew",
                "non-finite gap geometry or velocity, skipping goal placement skew");

            return 0.0f;
        }

        const Eigen::Vector2f chord = leftPt - rightPt;
        const float gapWidth = chord.norm();

        const float minValidWidth = 1e-3f;

        if (gapWidth < minValidWidth)
        {
            ROS_WARN_STREAM_THROTTLE_NAMED(5.0, "GoalPlacementSkew",
                "aperture width unusable (" << gapWidth
                << " m), skipping goal placement skew");

            return 0.0f;
        }

        gapWidthOut = gapWidth;

        const Eigen::Vector2f cHat = chord / gapWidth;

        const float leftClosingRate  = -leftGapVel.dot(cHat);
        const float rightClosingRate =  rightGapVel.dot(cHat);

                float closingRateDiff = leftClosingRate - rightClosingRate;

        //////////////////////////////////////////////////////////////////////
        // 1. DEADBAND.
        //
        // The estimator reports 0.1 to 0.3 m/s of closing rate on a static
        // wall. Without this test the goal is pushed 10 to 20 percent of the
        // aperture on pure noise, with random sign, on every cycle. That is
        // what makes the robot wobble when it should drive straight.
        //////////////////////////////////////////////////////////////////////

        if (std::fabs(closingRateDiff) < goalSkewMinClosingRate_)
            closingRateDiff = 0.0f;

        closingRateDiffOut = closingRateDiff;

        //////////////////////////////////////////////////////////////////////
        // 2. FLOOR ON THE WIDTH.
        //
        // dK divides by W. A narrow gap therefore turns a small closing rate
        // into a full-scale shift. The floor stops that.
        //////////////////////////////////////////////////////////////////////

        const float widthForNormalisation =
            std::max(gapWidth, goalSkewMinWidth_);

        float dK = 0.5f * closingRateDiff *
                    goalSkewLookaheadTime_ / widthForNormalisation;

        dK = std::min(goalSkewMaxDelta_, std::max(-goalSkewMaxDelta_, dK));

        //////////////////////////////////////////////////////////////////////
        // 3. TEMPORAL SMOOTHING AND RATE LIMIT.
        //
        // Keyed on the two estimator IDs, so the history follows the same
        // physical gap across cycles through the association step. A gap seen
        // for the first time starts from zero, so a new gap can never produce
        // an instant jump.
        //////////////////////////////////////////////////////////////////////

        int leftModelID  = -1;
        int rightModelID = -1;

        if (gap->getLeftGapPt() && gap->getLeftGapPt()->getModel())
            leftModelID = gap->getLeftGapPt()->getModel()->getID();

        if (gap->getRightGapPt() && gap->getRightGapPt()->getModel())
            rightModelID = gap->getRightGapPt()->getModel()->getID();

        const std::pair<int, int> gapKey(leftModelID, rightModelID);

        float previousDK = 0.0f;

        std::map<std::pair<int,int>, float>::const_iterator it =
            prevGoalSkewDelta_.find(gapKey);

        if (it != prevGoalSkewDelta_.end())
            previousDK = it->second;

        float smoothedDK = goalSkewSmoothing_ * previousDK +
                            (1.0f - goalSkewSmoothing_) * dK;

        // rate limit
        const float step = smoothedDK - previousDK;
        const float limitedStep =
            std::min(goalSkewMaxRate_, std::max(-goalSkewMaxRate_, step));

        smoothedDK = previousDK + limitedStep;

        // keep the history bounded -- model IDs churn as gaps appear and vanish
        if (prevGoalSkewDelta_.size() > 512)
            prevGoalSkewDelta_.clear();

        prevGoalSkewDelta_[gapKey] = smoothedDK;

        ROS_INFO_STREAM_NAMED("GoalPlacementSkew",
            "              s_L: " << leftClosingRate
            << ", s_R: " << rightClosingRate
            << ", closingRateDiff: " << closingRateDiff
            << ", gapWidth: " << gapWidth
            << ", rawDK: " << dK
            << ", dK: " << smoothedDK);

        return smoothedDK;
    }

    //////////////////////////////////////////////////////////////////////
    // Limit the goal fraction so the goal cannot sit on an endpoint.
    //
    // GapManipulator::inflateGapSides() has already inset the aperture by
    // r_inscr * inf_ratio, so k in [0, 1] is collision free to first order.
    // These limits are a margin against the case where inflateGapSides()
    // had to reduce inf_ratio toward 1.0, which leaves the bare robot
    // radius and no margin.
    //
    // Do NOT recompute the inflation here. That counts the robot radius twice.
    //////////////////////////////////////////////////////////////////////

    float GapGoalPlacer::clampGoalFraction(const float & k) const
    {
        return std::min(goalSkewMaxFraction_,
                        std::max(goalSkewMinFraction_, k));
    }

    //////////////////////////////////////////////////////////////////////
    // RViz markers for one gap's skew. Topic: "goal_placement_skew".
    //
    // The colours are the sign check. The RED arrow must start on the LEFT
    // gap point. If red and blue are swapped, every sign in the term is
    // inverted.
    //
    // An endpoint with a near-zero velocity draws no arrow. That absence is
    // itself diagnostic -- if no arrows ever appear, the velocities are
    // still coming from a zeroed source.
    //////////////////////////////////////////////////////////////////////

    void GapGoalPlacer::publishGoalSkewMarkers(Gap * gap,
                                                const Eigen::Vector2f & baseGoalPos,
                                                const Eigen::Vector2f & skewedGoalPos)
    {
        if (!publishGoalSkewMarkers_ || !gap)
            return;

        const Eigen::Vector2f leftPt  = gap->getManipulatedLPosition();
        const Eigen::Vector2f rightPt = gap->getManipulatedRPosition();

        const Eigen::Vector2f leftGapVel  = gap->getLVelocity();
        const Eigen::Vector2f rightGapVel = gap->getRVelocity();

        if (!leftPt.allFinite() || !rightPt.allFinite() ||
            !baseGoalPos.allFinite() || !skewedGoalPos.allFinite())
        {
            return;
        }

        const int markerIdx = goalSkewMarkerIdx_++;
        const ros::Time stamp = ros::Time::now();
        const double z = 0.05;
        const float minDrawableSpeed = 0.02f;

        visualization_msgs::MarkerArray markerArray;

        //////////////////////////////////////////////////////
        // 1. the aperture chord, grey
        //////////////////////////////////////////////////////
        {
            visualization_msgs::Marker m;

            m.header.frame_id = cfg_->robot_frame_id;
            m.header.stamp = stamp;
            m.ns = "goal_skew_chord";
            m.id = markerIdx;
            m.type = visualization_msgs::Marker::LINE_LIST;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.02;
            m.color.r = 0.5; m.color.g = 0.5; m.color.b = 0.5; m.color.a = 0.8;

            geometry_msgs::Point p0, p1;
            p0.x = leftPt[0];  p0.y = leftPt[1];  p0.z = z;
            p1.x = rightPt[0]; p1.y = rightPt[1]; p1.z = z;

            m.points.push_back(p0);
            m.points.push_back(p1);

            markerArray.markers.push_back(m);
        }

        //////////////////////////////////////////////////////
        // 2. LEFT endpoint velocity, ORANGE
        //////////////////////////////////////////////////////
        if (leftGapVel.allFinite() && leftGapVel.norm() > minDrawableSpeed)
        {
            visualization_msgs::Marker m;

            m.header.frame_id = cfg_->robot_frame_id;
            m.header.stamp = stamp;
            m.ns = "goal_skew_left_vel";
            m.id = markerIdx;
            m.type = visualization_msgs::Marker::ARROW;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.030;   // shaft diameter
            m.scale.y = 0.070;   // head diameter
            m.scale.z = 0.100;   // head length
            m.color.r = 1.00; m.color.g = 0.45; m.color.b = 0.00; m.color.a = 1.0;

            const Eigen::Vector2f tip =
                leftPt + leftGapVel * goalSkewVelocityMarkerScale_;

            geometry_msgs::Point p0, p1;
            p0.x = leftPt[0]; p0.y = leftPt[1]; p0.z = z;
            p1.x = tip[0];    p1.y = tip[1];    p1.z = z;

            m.points.push_back(p0);
            m.points.push_back(p1);

            markerArray.markers.push_back(m);
        }

        //////////////////////////////////////////////////////
        // 3. RIGHT endpoint velocity, BLUE
        //////////////////////////////////////////////////////
        if (rightGapVel.allFinite() && rightGapVel.norm() > minDrawableSpeed)
        {
            visualization_msgs::Marker m;

            m.header.frame_id = cfg_->robot_frame_id;
            m.header.stamp = stamp;
            m.ns = "goal_skew_right_vel";
            m.id = markerIdx;
            m.type = visualization_msgs::Marker::ARROW;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.030;
            m.scale.y = 0.070;
            m.scale.z = 0.100;
            m.color.r = 0.35; m.color.g = 0.45; m.color.b = 1.00; m.color.a = 1.0;

            const Eigen::Vector2f tip =
                rightPt + rightGapVel * goalSkewVelocityMarkerScale_;

            geometry_msgs::Point p0, p1;
            p0.x = rightPt[0]; p0.y = rightPt[1]; p0.z = z;
            p1.x = tip[0];     p1.y = tip[1];     p1.z = z;

            m.points.push_back(p0);
            m.points.push_back(p1);

            markerArray.markers.push_back(m);
        }

        //////////////////////////////////////////////////////
        // 4. base goal (white) and skewed goal (green)
        //////////////////////////////////////////////////////
        {
            visualization_msgs::Marker m;

            m.header.frame_id = cfg_->robot_frame_id;
            m.header.stamp = stamp;
            m.ns = "goal_skew_goals";
            m.id = markerIdx;
            m.type = visualization_msgs::Marker::SPHERE_LIST;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.10;
            m.scale.y = 0.10;
            m.scale.z = 0.10;
            m.color.a = 1.0;

            geometry_msgs::Point pBase, pSkew;
            pBase.x = baseGoalPos[0];   pBase.y = baseGoalPos[1];   pBase.z = z;
            pSkew.x = skewedGoalPos[0]; pSkew.y = skewedGoalPos[1]; pSkew.z = z;

            std_msgs::ColorRGBA cBase, cSkew;
            cBase.r = 0.85f; cBase.g = 0.85f; cBase.b = 0.85f; cBase.a = 1.0f;
            cSkew.r = 0.10f; cSkew.g = 0.80f; cSkew.b = 0.30f; cSkew.a = 1.0f;

            m.points.push_back(pBase);
            m.colors.push_back(cBase);
            m.points.push_back(pSkew);
            m.colors.push_back(cSkew);

            markerArray.markers.push_back(m);
        }

        //////////////////////////////////////////////////////
        // 5. the shift, base goal to skewed goal, green
        //////////////////////////////////////////////////////
        if ((skewedGoalPos - baseGoalPos).norm() > 1e-3f)
        {
            visualization_msgs::Marker m;

            m.header.frame_id = cfg_->robot_frame_id;
            m.header.stamp = stamp;
            m.ns = "goal_skew_shift";
            m.id = markerIdx;
            m.type = visualization_msgs::Marker::LINE_LIST;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.scale.x = 0.03;
            m.color.r = 0.10; m.color.g = 0.80; m.color.b = 0.30; m.color.a = 1.0;

            geometry_msgs::Point p0, p1;
            p0.x = baseGoalPos[0];   p0.y = baseGoalPos[1];   p0.z = z;
            p1.x = skewedGoalPos[0]; p1.y = skewedGoalPos[1]; p1.z = z;

            m.points.push_back(p0);
            m.points.push_back(p1);

            markerArray.markers.push_back(m);
        }

        goalSkewMarkerPublisher_.publish(markerArray);
    }

    bool GapGoalPlacer::checkWaypointVisibility(const Eigen::Vector2f & leftPt, 
                                                const Eigen::Vector2f & rightPt,
                                                const Eigen::Vector2f & globalGoal) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        // with robot as 0,0 (globalGoal in robot frame as well)
        float dist2goal = globalGoal.norm(); // sqrt(pow(globalGoal.pose.position.x, 2) + pow(globalGoal.pose.position.y, 2));

        sensor_msgs::LaserScan scan = *scan_.get();
        auto minScanRange = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        if (dist2goal < 2 * cfg_->rbt.r_inscr)
            return true;

        // If within closest configuration space
        if (dist2goal < minScanRange - cfg_->traj.inf_ratio * cfg_->rbt.r_inscr)
            return true;

        // Should be sufficiently far, otherwise we are in trouble
        float globalGoalAngle = std::atan2(globalGoal[1], globalGoal[0]);
        int globalGoalIdx = theta2idx(globalGoalAngle);

        // Should be sufficiently far, otherwise we are in trouble

        // get gap's range at globalGoal idx

        // float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);
        // float leftToWaypointAngle = getSweptLeftToRightAngle(leftPt, globalGoal);
        // float gapGoalRange = (rightPt.norm() - leftPt.norm()) * epsilonDivide(leftToWaypointAngle, leftToRightAngle) + leftPt.norm();

        float rangeAtGoalIdx = scan.ranges.at(globalGoalIdx);

        return dist2goal < rangeAtGoalIdx;
    }
}