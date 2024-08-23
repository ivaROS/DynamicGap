#include <dynamic_gap/trajectory_generation/GapManipulator.h>

namespace dynamic_gap 
{
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    void GapManipulator::setGapGoal(dynamic_gap::Gap * gap, 
                                    const geometry_msgs::PoseStamped & globalPathLocalWaypointRobotFrame, 
                                    const geometry_msgs::PoseStamped & globalGoalRobotFrame) 
    {
        try
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "    [setGapGoal()]");

            int leftIdx = gap->manipLeftIdx();
            int rightIdx = gap->manipRightIdx();
            float leftRange = gap->manipLeftRange();
            float rightRange = gap->manipRightRange();

            float leftTheta = idx2theta(leftIdx);
            float rightTheta = idx2theta(rightIdx);

            float xLeft = (leftRange) * cos(leftTheta);
            float yLeft = (leftRange) * sin(leftTheta);
            float xRight = (rightRange) * cos(rightTheta);
            float yRight = (rightRange) * sin(rightTheta);

            Eigen::Vector2f leftPt(xLeft, yLeft);
            Eigen::Vector2f rightPt(xRight, yRight);

            ROS_INFO_STREAM_NAMED("GapManipulator", "    [setGapGoal()]");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        gap polar points, left: (" << leftIdx << ", " << leftRange << ") , right: (" << rightIdx << ", " << rightRange << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        gap cart points, left: (" << xLeft << ", " << yLeft << ") , right: (" << xRight << ", " << yRight << ")");

            float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);

            // Second condition: if angle smaller than M_PI / 3
            // Check if arc length < 3 robot width

            Eigen::Vector2f globalGoalRobotFrameVector(globalGoalRobotFrame.pose.position.x, 
                                                        globalGoalRobotFrame.pose.position.y);

            float globalGoalTheta = std::atan2(globalGoalRobotFrameVector[1], globalGoalRobotFrameVector[0]);
            float globalGoalIdx = theta2idx(globalGoalTheta); // std::floor(goal_orientation*half_num_scan/M_PI + half_num_scan);
            
            ROS_INFO_STREAM_NAMED("GapManipulator", "        global goal idx: " << globalGoalIdx << 
                                        ", global goal: (" << globalGoalRobotFrameVector[0] << 
                                                         ", " << globalGoalRobotFrameVector[1] << ")");
            
            if (isGlobalPathLocalWaypointWithinGapAngle(globalGoalIdx, rightIdx, leftIdx) && 
                checkWaypointVisibility(leftPt, rightPt, globalGoalRobotFrameVector))
            {
                // gap->setGoal(globalGoalRobotFrameVector);
                
                // all we will do is mark it for later so we can run g2g policy on global goal.
                // Still set mid point for pursuit guidance policy and feasibility check
                gap->globalGoalWithin = true;

                std::string goalStatus = "Option 1: global goal: ";
                ROS_INFO_STREAM_NAMED("GapManipulator", "        " << goalStatus);
                ROS_INFO_STREAM_NAMED("GapManipulator", "            goal: " << globalGoalRobotFrameVector[0] << 
                                                                        ", " << globalGoalRobotFrameVector[1]);
                   
                // return;
            } else if (leftToRightAngle < M_PI / 4)
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "        Option 2: gap mid point");

                float centerTheta = leftTheta - (leftToRightAngle / 2.0);
                float centerRange = (leftRange + rightRange) / 2.;
                Eigen::Vector2f centerPt(centerRange * std::cos(centerTheta),
                                        centerRange * std::sin(centerTheta));
            
                gap->setGoal(centerPt);

                ROS_INFO_STREAM_NAMED("GapManipulator", "            finished with goal: " << centerPt[0] << ", " << centerPt[1]); 
            } else
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "        Option 3: global path local waypoint biased");

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

                gap->setGoal(biasedGapGoal);

            }


            // // needs to be distance between L/confined. Always positive.
            // float leftToGapGoalAngle = getSignedLeftToRightAngle(leftPt, biasedGapGoalUnitNorm); 

            // ROS_INFO_STREAM_NAMED("GapManipulator", "            biasedGapGoalTheta: " << biasedGapGoalTheta);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "            biasedGapGoalIdx: " << biasedGapGoalIdx);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "            leftToGapGoalAngle: " << leftToGapGoalAngle << ", leftToRightAngle: " << leftToRightAngle);


            // Eigen::Vector2f gapGoalAngularOffset(0.0, 0.0); 
            // if ( (leftToGapGoalAngle / leftToRightAngle) < 0.1) // biased gap goal on left side
            //     gapGoalAngularOffset = Rnegpi2 * unitNorm(leftPt) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;    
            // else                                                // biased gap goal on right side
            //     gapGoalAngularOffset = Rpi2 * unitNorm(rightPt) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

            // Eigen::Vector2f gapGoalRadialOffset = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * unitNorm(biasedGapGoal);
            // Eigen::Vector2f gapGoalOffset = gapGoalRadialOffset + gapGoalAngularOffset;
            
            // ROS_INFO_STREAM_NAMED("GapManipulator", "            biasedGapGoal: " << biasedGapGoal[0] << ", " << biasedGapGoal[1]);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "            gapGoalRadialOffset: " << gapGoalRadialOffset[0] << ", " << gapGoalRadialOffset[1]);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "            gapGoalAngularOffset: " << gapGoalAngularOffset[0] << ", " << gapGoalAngularOffset[1]);
            
            // Eigen::Vector2f offsetBiasedGapGoal = gapGoalOffset + biasedGapGoal;
            // ROS_INFO_STREAM_NAMED("GapManipulator", "anchor: (" << anchor[0] << ", " << anchor[1] << "), offset with r_ins " << cfg_->rbt.r_inscr << " and inf ratio " << cfg_->traj.inf_ratio << ", :(" << offset[0] << ", " << offset[1] << "), offsetBiasedGapGoal: (" << offsetBiasedGapGoal[0] << ", " << offsetBiasedGapGoal[1] << ")");

        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("GapManipulator", "[setGapGoal() failed]");
        }      
    }

    float GapManipulator::setBiasedGapGoalTheta(const float & leftTheta, const float & rightTheta, const float & globalGoalTheta,
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

        ROS_INFO_STREAM_NAMED("GapManipulator", "            leftTheta: " << leftTheta << ", rightTheta: " << rightTheta << ", globalGoalTheta: " << globalGoalTheta);
        ROS_INFO_STREAM_NAMED("GapManipulator", "            leftToRightAngle: " << leftToRightAngle << ", leftToWaypointAngle: " << leftToWaypointAngle << ", rightToWaypointAngle: " << rightToWaypointAngle);

        return biasedGapGoalTheta;
    }

    bool GapManipulator::checkWaypointVisibility(const Eigen::Vector2f & leftPt, 
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

        // get gap's range at globalGoal idx

        float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);
        float leftToWaypointAngle = getSweptLeftToRightAngle(leftPt, globalGoal);
        float gapGoalRange = (rightPt.norm() - leftPt.norm()) * epsilonDivide(leftToWaypointAngle, leftToRightAngle) + leftPt.norm();

        return dist2goal < gapGoalRange;
    }

    void GapManipulator::radialExtendGap(dynamic_gap::Gap * gap) 
    {
        try
        {
            if (!cfg_->gap_manip.radial_extend)
                return;

            int leftIdx = gap->manipLeftIdx();
            int rightIdx = gap->manipRightIdx();
            float leftRange = gap->manipLeftRange();
            float rightRange = gap->manipRightRange();

            float leftTheta = idx2theta(leftIdx);
            float rightTheta = idx2theta(rightIdx);
            float xLeft = leftRange * cos(leftTheta);
            float yLeft = leftRange * sin(leftTheta);
            float xRight = rightRange * cos(rightTheta);
            float yRight = rightRange * sin(rightTheta);
            
            Eigen::Vector2d leftPt(xLeft, yLeft);
            Eigen::Vector2d rightPt(xRight, yRight);

            ROS_INFO_STREAM_NAMED("GapManipulator", "    [radialExtendGap()]");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RE gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RE gap in cart. left: (" << leftPt[0] << ", " << leftPt[1] << "), right: (" << rightPt[0] << ", " << rightPt[1] << ")");
            
            float s = std::min(gap->getMinSafeDist(), cfg_->rbt.r_inscr * cfg_->traj.inf_ratio);

            // // ROS_INFO_STREAM_NAMED("GapManipulator", "leftPt: (" << xLeft << ", " << yLeft << "), rightPt: (" << xRight << ", " << yRight << ")");
            // // ROS_INFO_STREAM_NAMED("GapManipulator", "eL_robot: (" << eL_robot[0] << ", " << eL_robot[1] << ") , eR_robot: (" << eR_robot[0] << ", " << eR_robot[1] << ")");

            // float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);

            // float thetaCenter = (leftTheta - 0.5*leftToRightAngle);

            // // middle of gap direction
            // Eigen::Vector2f eB(std::cos(thetaCenter), std::sin(thetaCenter));
            // // ROS_INFO_STREAM_NAMED("GapManipulator", "eB: (" << eB[0] << ", " << eB[1] << ")");

            // Eigen::Vector2f norm_eB = unitNorm(eB); 
            // // angular size of gap
            // // ROS_INFO_STREAM_NAMED("GapManipulator", "normalized eB: " << norm_eB[0] << ", " << norm_eB[1]);

            // // minSafeDist is the minimum distance within the laser scan 
            // // ROS_INFO_STREAM_NAMED("GapManipulator", "min safe dist: " << s);
            
            // // point opposite direction of middle of gap, magnitude of min safe dist
            // Eigen::Vector2f extendedGapOrigin =  - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * norm_eB; //
            // // ROS_INFO_STREAM_NAMED("GapManipulator", "extendedGapOrigin: " << extendedGapOrigin[0] << ", " << extendedGapOrigin[1]);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "        finishing with gap extendedGapOrigin: " << extendedGapOrigin[0] << ", " << extendedGapOrigin[1]);

            return;
        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "radialGapExtension() failed");
        }
    }

    bool GapManipulator::inflateGapSides(dynamic_gap::Gap * gap) 
    {
        try 
        {
            // get points

            sensor_msgs::LaserScan desScan = *scan_.get();
            int leftIdx = gap->manipLeftIdx();
            int rightIdx = gap->manipRightIdx();
            float leftRange = gap->manipLeftRange();
            float rightRange = gap->manipRightRange();

            float leftTheta = idx2theta(leftIdx);
            float rightTheta = idx2theta(rightIdx);
            float xLeft = (leftRange) * cos(leftTheta);
            float yLeft = (leftRange) * sin(leftTheta);
            float xRight = (rightRange) * cos(rightTheta);
            float yRight = (rightRange) * sin(rightTheta);
            
            Eigen::Vector2f leftPt(xLeft, yLeft);
            Eigen::Vector2f rightPt(xRight, yRight);
   
            ROS_INFO_STREAM_NAMED("GapManipulator", "    [inflateGapSides()]");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-inflate gap in polar. left: (" << leftIdx << ", " << leftRange << "), right: (" << rightIdx << ", " << rightRange << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-inflate gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

            Eigen::Vector2f leftUnitNorm = unitNorm(leftPt);
            Eigen::Vector2f rightUnitNorm = unitNorm(rightPt);
            float leftToRightAngle = getSweptLeftToRightAngle(leftUnitNorm, rightUnitNorm);

            ///////////////////////
            // ANGULAR INFLATION //
            ///////////////////////

            Eigen::Vector2f leftAngularInflDir, rightAngularInflDir;
            if (leftToRightAngle <= M_PI) // convex gap
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "        convex gap");
                float centerTheta = leftTheta - (leftToRightAngle / 2.);

                ROS_INFO_STREAM_NAMED("GapManipulator", "        centerTheta: " << centerTheta);

                float midRayTheta = (M_PI / 2) - (leftToRightAngle / 2.);                

                ROS_INFO_STREAM_NAMED("GapManipulator", "        midRayTheta: " << midRayTheta);

                float r_mid_left = std::sin(midRayTheta) / (sin(M_PI / 2) / leftPt.norm());
                float r_mid_right = std::sin(midRayTheta) / (sin(M_PI / 2) / rightPt.norm());

                ROS_INFO_STREAM_NAMED("GapManipulator", "        r_mid_left: " << r_mid_left);
                ROS_INFO_STREAM_NAMED("GapManipulator", "        r_mid_right: " << r_mid_right);

                Eigen::Vector2f leftMidVec(r_mid_left * std::cos(centerTheta), 
                                            r_mid_left * std::sin(centerTheta));
                Eigen::Vector2f rightMidVec(r_mid_right * std::cos(centerTheta), 
                                            r_mid_right * std::sin(centerTheta));  

                Eigen::Vector2f leftAngularInflVec = leftMidVec - leftPt;
                Eigen::Vector2f rightAngularInflVec = rightMidVec - rightPt;
                leftAngularInflDir = leftAngularInflVec / leftAngularInflVec.norm(); 
                rightAngularInflDir = rightAngularInflVec / rightAngularInflVec.norm(); 
            } else // non-convex gap
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "        non-convex gap");

                leftAngularInflDir = Rnegpi2 * unitNorm(leftPt);
                rightAngularInflDir = Rpi2 * unitNorm(rightPt);
            }

            ROS_INFO_STREAM_NAMED("GapManipulator", "    leftAngularInflDir: " << leftAngularInflDir.transpose());
            ROS_INFO_STREAM_NAMED("GapManipulator", "    rightAngularInflDir: " << rightAngularInflDir.transpose());

            // perform inflation
            Eigen::Vector2f inflatedLeftPt = leftPt + leftAngularInflDir * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            Eigen::Vector2f inflatedRightPt = rightPt + rightAngularInflDir * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

            float inflatedLeftTheta = std::atan2(inflatedLeftPt[1], inflatedLeftPt[0]);
            float inflatedRightTheta = std::atan2(inflatedRightPt[1], inflatedRightPt[0]);

            Eigen::Vector2f inflatedLeftUnitNorm(std::cos(inflatedLeftTheta), std::sin(inflatedLeftTheta));
            Eigen::Vector2f inflatedRightUnitNorm(std::cos(inflatedRightTheta), std::sin(inflatedRightTheta));
            float newLeftToRightAngle = getSweptLeftToRightAngle(inflatedLeftUnitNorm, inflatedRightUnitNorm);


            // if gap is too small, mark it to be discarded
            if (newLeftToRightAngle > leftToRightAngle)
            {
                ROS_INFO_STREAM_NAMED("GapManipulator", "        inflation has failed");

                return false;
            }

            float inflatedLeftIdx = theta2idx(inflatedLeftTheta);
            float inflatedRightIdx = theta2idx(inflatedRightTheta);
            
            float leftToInflatedLeftAngle = getSignedLeftToRightAngle(leftUnitNorm, inflatedLeftUnitNorm);
            float leftToInflatedRightAngle = getSignedLeftToRightAngle(leftUnitNorm, inflatedRightUnitNorm);
            float inflatedLeftRange = leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedLeftAngle, leftToRightAngle);
            float inflatedRightRange =  leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedRightAngle, leftToRightAngle);

            // inflate inwards by radius * infl
            // rotate by pi/2, norm
            
            // PERFORMING ANGULAR INFLATION
            // float newLeftToRightAngle = -1.0;

            // float infl_ratio_decr = 0.99;

            // float infl_ratio = cfg_->traj.inf_ratio / infl_ratio_decr;
            // float inflatedLeftTheta = 0.0, inflatedRightTheta = 0.0;
            // Eigen::Vector2f inflatedLeftUnitNorm, inflatedRightUnitNorm;
            // while (newLeftToRightAngle < 0 && infl_ratio >= 1.0)
            // {
            //     // infl_ratio can be too large that it would destroy gap, so we will decrement the ratio until we can inflate safely
            //     infl_ratio = infl_ratio_decr * infl_ratio;
            //     // ROS_INFO_STREAM_NAMED("GapManipulator", "                infl_ratio: " << infl_ratio); // << ", leftThetaeft_infl: " << leftThetaeft_infl

            //     Eigen::Vector2f leftInflationVector = cfg_->rbt.r_inscr * infl_ratio * Rnegpi2 * leftUnitNorm;
            //     Eigen::Vector2f inflatedLeftPt = leftPt + leftInflationVector;
            //     inflatedLeftTheta = std::atan2(inflatedLeftPt[1], inflatedLeftPt[0]);

            //     // ROS_INFO_STREAM_NAMED("GapManipulator", "                leftTheta: " << leftTheta << ", inflatedLeftTheta: " << inflatedLeftTheta); // << ", leftThetaeft_infl: " << leftThetaeft_infl

            //     Eigen::Vector2f rightInflationVector = cfg_->rbt.r_inscr * infl_ratio * Rpi2 * rightUnitNorm;
            //     Eigen::Vector2f inflatedRightPt = rightPt + rightInflationVector;
            //     inflatedRightTheta = std::atan2(inflatedRightPt[1], inflatedRightPt[0]);

            //     // ROS_INFO_STREAM_NAMED("GapManipulator", "                rightTheta: " << rightTheta << ", inflatedRightTheta: " << inflatedRightTheta); // ", rightTheta_infl: " << rightTheta_infl << 

            //     inflatedLeftUnitNorm << std::cos(inflatedLeftTheta), std::sin(inflatedLeftTheta);
            //     inflatedRightUnitNorm << std::cos(inflatedRightTheta), std::sin(inflatedRightTheta);
            //     newLeftToRightAngle = getSignedLeftToRightAngle(inflatedLeftUnitNorm, inflatedRightUnitNorm);
            //     // ROS_INFO_STREAM_NAMED("GapManipulator", "                newLeftToRightAngle: " << newLeftToRightAngle);
            // }

            // if (newLeftToRightAngle < 0)
            //     return;

            // // if (newLeftToRightAngle < 0) // want to allow terminal gaps where initial were RGC'd to inflate 
            // // {
            // //     ROS_INFO_STREAM_NAMED("GapManipulator", "angular inflation would cause gap to cross, not running:");
            // //     inflatedLeftIdx = leftIdx;
            // //     inflatedRightIdx = rightIdx;

            // //     inflatedLeftRange = leftRange;
            // //     inflatedRightRange = rightRange;
            // // } else 
            // // {
            //     // need to make sure L/R don't cross each other

            //     // int inflatedLeftIdx = 0, inflatedRightIdx = 0;
            //     // float inflatedLeftRange = 0.0, inflatedRightRange = 0.0;
            // float inflatedLeftIdx = theta2idx(inflatedLeftTheta);
            // float inflatedRightIdx = theta2idx(inflatedRightTheta);
            
            // float leftToInflatedLeftAngle = getSignedLeftToRightAngle(leftUnitNorm, inflatedLeftUnitNorm);
            // float leftToInflatedRightAngle = getSignedLeftToRightAngle(leftUnitNorm, inflatedRightUnitNorm);
            // float inflatedLeftRange = leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedLeftAngle, leftToRightAngle);
            // float inflatedRightRange =  leftRange + (rightRange - leftRange) * epsilonDivide(leftToInflatedRightAngle, leftToRightAngle);
                // if (cfg_->debug.manipulation_debug_log) 
                // {
                //     ROS_INFO_STREAM_NAMED("GapManipulator", "        post-angular-inflation gap, left: " << inflatedLeftIdx << ", : " << inflatedLeftRange << 
                //                                                     ", right: " << inflatedRightIdx << ", : " << inflatedRightRange);
                //     // if (inflatedLeftRange > 8 || inflatedRightRange > 8) {
                //     //     ROS_INFO_STREAM_NAMED("GapManipulator", "            range is too big");
                //     // }
                // }
            // }

            // PERFORMING RADIAL INFLATION
            // inflatedLeftRange += 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            // if ((desScan.ranges.at(inflatedLeftIdx) - inflatedLeftRange) < (cfg_->rbt.r_inscr * cfg_->traj.inf_ratio)) // reject the change
            //     inflatedLeftRange = desScan.ranges.at(inflatedLeftIdx) - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

            // inflatedRightRange += 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            // if ((desScan.ranges.at(inflatedRightIdx) - inflatedRightRange) < (cfg_->rbt.r_inscr * cfg_->traj.inf_ratio)) // reject the change
            //     inflatedRightRange = desScan.ranges.at(inflatedRightIdx) - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

            // ROS_INFO_STREAM_NAMED("GapManipulator", "new_l_theta: " << new_l_theta << ", new_r_theta: " << new_r_theta);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "int values, left: " << inflatedLeftIdx << ", right: " << inflatedRightIdx);

            if (inflatedRightIdx == inflatedLeftIdx) // ROS_INFO_STREAM_NAMED("GapManipulator", "manipulated indices are same");
                inflatedLeftIdx++;

            gap->setManipLeftIdx(inflatedLeftIdx);
            gap->setManipRightIdx(inflatedRightIdx);      
            gap->setManipLeftRange(inflatedLeftRange);
            gap->setManipRightRange(inflatedRightRange);
            gap->getManipulatedLCartesian(xLeft, yLeft);
            gap->getManipulatedRCartesian(xRight, yRight);

            ROS_INFO_STREAM_NAMED("GapManipulator", "        post-inflate gap in polar. left: (" << inflatedLeftIdx << ", " << inflatedLeftRange << "), right: (" << inflatedRightIdx << ", " << inflatedRightRange << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        post-inflate gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

            return true;

        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "[inflateGapSides() failed]");
            return false;
        }
    }
}
