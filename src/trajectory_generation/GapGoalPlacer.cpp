#include <dynamic_gap/trajectory_generation/GapGoalPlacer.h>

namespace dynamic_gap
{

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

            float centerTheta = leftTheta - (0.5 * leftToRightAngle);
            float centerRange = 0.5 * (leftRange + rightRange);
            Eigen::Vector2f centerPt(centerRange * std::cos(centerTheta),
                                        centerRange * std::sin(centerTheta));
            Eigen::Vector2f centerVel = 0.5 * (leftVel + rightVel);

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

            // float biasedGapGoalIdx = theta2idx(biasedGapGoalTheta); // std::floor(biasedGapGoalTheta*half_num_scan/M_PI + half_num_scan);

            float biasedGapGoalDist = leftRange + (rightRange - leftRange) * leftToGapGoalAngle / leftToRightAngle;
            Eigen::Vector2f biasedGapGoal(biasedGapGoalDist * cos(biasedGapGoalTheta), biasedGapGoalDist * sin(biasedGapGoalTheta));
            Eigen::Vector2f biasedGapVel = leftVel + (rightVel - leftVel) * leftToGapGoalAngle / leftToRightAngle;

            ROS_INFO_STREAM_NAMED("GapGoalPlacerV2", "              original goal: " << biasedGapGoal[0] << ", " << biasedGapGoal[1]);                 

            Eigen::Vector2f gapGoalRadialOffset = 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * biasedGapGoal.normalized();

            Eigen::Vector2f inflatedBiasedGapGoal = biasedGapGoal + gapGoalRadialOffset;

            // check if goal is beyond scan
            int biasedGapGoalIdx = theta2idx(biasedGapGoalTheta);
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