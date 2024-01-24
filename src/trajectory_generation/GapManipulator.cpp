#include <dynamic_gap/trajectory_generation/GapManipulator.h>

namespace dynamic_gap 
{
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    /*
    // Not currently used
    void GapManipulator::updateStaticEgoCircle(const sensor_msgs::LaserScan & staticScan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        staticScan_ = staticScan;
    }
    */

    /*
    // Not currently used
    void GapManipulator::updateDynamicEgoCircle(dynamic_gap::Gap * gap,
                                                const std::vector<sensor_msgs::LaserScan> & futureScans) 
    {
        // dynamicScan_ = staticScan_;
        float t_iplus1 = gap->gapLifespan_;

        int futureScanIdx = (int) (t_iplus1 / cfg_->traj.integrate_stept);

        dynamicScan_ = futureScans.at(futureScanIdx);

        float dynamicScanMinDist = *std::min_element(dynamicScan_.ranges.begin(), dynamicScan_.ranges.end());
        gap->setTerminalMinSafeDist(dynamicScanMinDist);
    }
    */
    
    void GapManipulator::setGapTerminalGoal(dynamic_gap::Gap * gap, 
                                            const geometry_msgs::PoseStamped & globalPathLocalWaypoint) 
    {
        try
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "    [setGapTerminalGoal()]");
            
            if (gap->getCategory() == "expanding" || gap->getCategory() == "static") 
            { 
                ROS_INFO_STREAM_NAMED("GapManipulator", "setting terminal goal for expanding gap");
                setGapGoal(gap, globalPathLocalWaypoint, false);
            } else if (gap->getCategory() == "closing") 
            {
                std::string closingGapType = "";
                if (gap->crossed_) 
                {
                    closingGapType = "crossed";
                    Eigen::Vector2f crossingPt = gap->getCrossingPoint();

                    gap->terminalGoal.x_ = crossingPt[0];
                    gap->terminalGoal.y_ = crossingPt[1];
                } else if (gap->closed_) 
                {
                    closingGapType = "closed";
                    ROS_INFO_STREAM_NAMED("GapManipulator", "        setting terminal goal for closed closing gap");
                    Eigen::Vector2f closing_pt = gap->getClosingPoint();
                
                    gap->terminalGoal.x_ = closing_pt[0];
                    gap->terminalGoal.y_ = closing_pt[1];
                } else 
                {
                    closingGapType = "existent";
                    setGapGoal(gap, globalPathLocalWaypoint, false);
                }

                ROS_INFO_STREAM_NAMED("GapManipulator", "        setting terminal goal for " + closingGapType + " closing gap");
            }
            ROS_INFO_STREAM_NAMED("GapManipulator", "    [setGapTerminalGoal() finished]");        
        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "[setGapTerminalGoal() failed]");
        }
    }

    void GapManipulator::setGapGoal(dynamic_gap::Gap * gap, 
                                    const geometry_msgs::PoseStamped & globalPathLocalWaypoint, 
                                    const bool & initial) 
    {
        try
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "    [setGapGoal()]");

            int leftIdx = 0, rightIdx = 0;
            float leftDist = 0.0, rightDist = 0.0;
            if (initial) 
            {
                leftIdx = gap->cvxLeftIdx();
                rightIdx = gap->cvxRightIdx();
                leftDist = gap->cvxLeftDist();
                rightDist = gap->cvxRightDist();
            } else 
            {
                leftIdx = gap->cvxTermLeftIdx();
                rightIdx = gap->cvxTermRightIdx();
                leftDist = gap->cvxTermLeftDist();
                rightDist = gap->cvxTermRightDist();
            }

            float leftTheta = idx2theta(leftIdx);
            float rightTheta = idx2theta(rightIdx);

            float xLeft = (leftDist) * cos(leftTheta);
            float yLeft = (leftDist) * sin(leftTheta);
            float xRight = (rightDist) * cos(rightTheta);
            float yRight = (rightDist) * sin(rightTheta);

            Eigen::Vector2f leftPt(xLeft, yLeft);
            Eigen::Vector2f rightPt(xRight, yRight);

            ROS_INFO_STREAM_NAMED("GapManipulator", "    [setGapGoal()]");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        gap polar points, left: (" << leftIdx << ", " << leftDist << ") , right: (" << rightIdx << ", " << rightDist << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        gap cart points, left: (" << xLeft << ", " << yLeft << ") , right: (" << xRight << ", " << yRight << ")");

            float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);

            // Second condition: if angle smaller than M_PI / 3
            // Check if arc length < 3 robot width

            Eigen::Vector2f globalPathLocalWaypointVector(globalPathLocalWaypoint.pose.position.x, globalPathLocalWaypoint.pose.position.y);

            bool smallGap = (leftToRightAngle < M_PI && sqrt(pow(xLeft - xRight, 2) + pow(yLeft - yRight, 2)) < 4 * cfg_->rbt.r_inscr);
            if (smallGap) 
            {

                float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);
                
                float thetaLeft = std::atan2(leftPt[1], leftPt[0]);

                // ROS_INFO_STREAM_NAMED("GapManipulator", "leftToRightAngle: " << leftToRightAngle);
                float thetaCenter = (thetaLeft - 0.5 * leftToRightAngle); 
                float rangeCenter = (leftPt.norm() + rightPt.norm()) / 2.0;
                Eigen::Vector2f centerGoal(rangeCenter * std::cos(thetaCenter), rangeCenter * std::sin(thetaCenter));
                // ROS_INFO_STREAM_NAMED("GapManipulator", "thetaLeft: " << thetaLeft << ", thetaRight: " << thetaRight << ", thetaCenter: " << thetaCenter);

                gap->setGoal(initial, centerGoal);

                ROS_INFO_STREAM_NAMED("GapManipulator", "        Option 1: small gap");
                ROS_INFO_STREAM_NAMED("GapManipulator", "            goal: " << centerGoal[0] << ", " << centerGoal[1]);

                return;
            }

            float globalPathLocalWaypointTheta = std::atan2(globalPathLocalWaypointVector[1], globalPathLocalWaypointVector[0]);
            float globalPathLocalWaypointIdx = theta2idx(globalPathLocalWaypointTheta); // std::floor(goal_orientation*half_num_scan/M_PI + half_num_scan);
            
            ROS_INFO_STREAM_NAMED("GapManipulator", "        local goal idx: " << globalPathLocalWaypointIdx << 
                                        ", local goal x/y: (" << globalPathLocalWaypointVector[0] << 
                                                         ", " << globalPathLocalWaypointVector[1] << ")");
            
            if (gap->artificial_ || (isGlobalPathLocalWaypointWithinGapAngle(globalPathLocalWaypointIdx, rightIdx, leftIdx) && 
                                    checkWaypointVisibility(leftPt, rightPt, globalPathLocalWaypointVector)))
            {
                gap->setGoal(initial, globalPathLocalWaypointVector);

                std::string goalStatus = gap->artificial_ ? "Option 0: artificial gap: " : "Option 2: global path local waypoint: ";
                ROS_INFO_STREAM_NAMED("GapManipulator", "        " << goalStatus);
                ROS_INFO_STREAM_NAMED("GapManipulator", "            goal: " << globalPathLocalWaypointVector[0] << ", " << globalPathLocalWaypointVector[1]);
                   
                return;
            }

            ROS_INFO_STREAM_NAMED("GapManipulator", "        Option 3: biasing gap goal towards global path local waypoint");

            float leftToWaypointAngle = getSweptLeftToRightAngle(leftPt, globalPathLocalWaypointVector);
            float rightToWaypointAngle = getSweptLeftToRightAngle(rightPt, globalPathLocalWaypointVector);
    
            float biasedGapGoalTheta = setBiasedGapGoalTheta(leftTheta, rightTheta, globalPathLocalWaypointTheta,
                                                            leftToRightAngle, rightToWaypointAngle, leftToWaypointAngle);

            float biasedGapGoalIdx = theta2idx(biasedGapGoalTheta); // std::floor(biasedGapGoalTheta*half_num_scan/M_PI + half_num_scan);

            Eigen::Vector2f biasedGapGoalUnitNorm(std::cos(biasedGapGoalTheta), std::sin(biasedGapGoalTheta));

            // needs to be distance between L/confined. Always positive.
            float leftToGapGoalAngle = getSignedLeftToRightAngle(leftPt, biasedGapGoalUnitNorm); 

            ROS_INFO_STREAM_NAMED("GapManipulator", "            biasedGapGoalTheta: " << biasedGapGoalTheta);
            ROS_INFO_STREAM_NAMED("GapManipulator", "            biasedGapGoalIdx: " << biasedGapGoalIdx);
            ROS_INFO_STREAM_NAMED("GapManipulator", "            leftToGapGoalAngle: " << leftToGapGoalAngle << ", leftToRightAngle: " << leftToRightAngle);

            float biasedGapGoalDist = leftDist + (rightDist - leftDist) * leftToGapGoalAngle / leftToRightAngle;
            Eigen::Vector2f biasedGapGoal(biasedGapGoalDist * cos(biasedGapGoalTheta), biasedGapGoalDist * sin(biasedGapGoalTheta));

            Eigen::Vector2f gapGoalAngularOffset(0.0, 0.0); 
            if ( (leftToGapGoalAngle / leftToRightAngle) < 0.1) // biased gap goal on left side
                gapGoalAngularOffset = Rnegpi2 * unitNorm(leftPt) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;    
            else                                                // biased gap goal on right side
                gapGoalAngularOffset = Rpi2 * unitNorm(rightPt) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

            Eigen::Vector2f gapGoalRadialOffset = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * unitNorm(biasedGapGoal);
            Eigen::Vector2f gapGoalOffset = gapGoalRadialOffset + gapGoalAngularOffset;
            
            ROS_INFO_STREAM_NAMED("GapManipulator", "            biasedGapGoal: " << biasedGapGoal[0] << ", " << biasedGapGoal[1]);
            ROS_INFO_STREAM_NAMED("GapManipulator", "            gapGoalRadialOffset: " << gapGoalRadialOffset[0] << ", " << gapGoalRadialOffset[1]);
            ROS_INFO_STREAM_NAMED("GapManipulator", "            gapGoalAngularOffset: " << gapGoalAngularOffset[0] << ", " << gapGoalAngularOffset[1]);
            
            Eigen::Vector2f offsetBiasedGapGoal = gapGoalOffset + biasedGapGoal;
            // ROS_INFO_STREAM_NAMED("GapManipulator", "anchor: (" << anchor[0] << ", " << anchor[1] << "), offset with r_ins " << cfg_->rbt.r_inscr << " and inf ratio " << cfg_->traj.inf_ratio << ", :(" << offset[0] << ", " << offset[1] << "), offsetBiasedGapGoal: (" << offsetBiasedGapGoal[0] << ", " << offsetBiasedGapGoal[1] << ")");
            gap->setGoal(initial, offsetBiasedGapGoal);

            ROS_INFO_STREAM_NAMED("GapManipulator", "            finished with goal: " << offsetBiasedGapGoal[0] << ", " << offsetBiasedGapGoal[1]); 

        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("GapManipulator", "[setGapGoal() failed]");
        }      
    }

    float GapManipulator::setBiasedGapGoalTheta(const float & leftTheta, const float & rightTheta, const float & globalPathLocalWaypointTheta,
                                                const float & leftToRightAngle, const float & rightToWaypointAngle, const float & leftToWaypointAngle)
    {
        float biasedGapGoalTheta = 0.0;
        if (leftTheta > rightTheta) // gap is not behind robot
        { 
            biasedGapGoalTheta = std::min(leftTheta, std::max(rightTheta, globalPathLocalWaypointTheta));
        } else // gap is behind
        { 
            if (0 < leftToWaypointAngle && leftToWaypointAngle < leftToRightAngle)
                biasedGapGoalTheta = globalPathLocalWaypointTheta;
            else if (std::abs(leftToWaypointAngle) < std::abs(rightToWaypointAngle))
                biasedGapGoalTheta = leftTheta;
            else
                biasedGapGoalTheta = rightTheta;
        }

        ROS_INFO_STREAM_NAMED("GapManipulator", "            leftTheta: " << leftTheta << ", rightTheta: " << rightTheta << ", globalPathLocalWaypointTheta: " << globalPathLocalWaypointTheta);
        ROS_INFO_STREAM_NAMED("GapManipulator", "            leftToRightAngle: " << leftToRightAngle << ", leftToWaypointAngle: " << leftToWaypointAngle << ", rightToWaypointAngle: " << rightToWaypointAngle);

        return biasedGapGoalTheta;
    }

    bool GapManipulator::checkWaypointVisibility(const Eigen::Vector2f & leftPt, 
                                                    const Eigen::Vector2f & rightPt,
                                                    const Eigen::Vector2f & globalPathLocalWaypoint) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        // with robot as 0,0 (globalPathLocalWaypoint in robot frame as well)
        float dist2goal = globalPathLocalWaypoint.norm(); // sqrt(pow(globalPathLocalWaypoint.pose.position.x, 2) + pow(globalPathLocalWaypoint.pose.position.y, 2));

        sensor_msgs::LaserScan scan = *scan_.get();
        auto minScanRange = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        if (dist2goal < 2 * cfg_->rbt.r_inscr)
            return true;

        // If within closest configuration space
        if (dist2goal < minScanRange - cfg_->traj.inf_ratio * cfg_->rbt.r_inscr)
            return true;

        // Should be sufficiently far, otherwise we are in trouble

        // get gap's range at globalPathLocalWaypoint idx

        float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);
        float leftToWaypointAngle = getSweptLeftToRightAngle(leftPt, globalPathLocalWaypoint);
        float gapGoalRange = (rightPt.norm() - leftPt.norm()) * epsilonDivide(leftToWaypointAngle, leftToRightAngle) + leftPt.norm();

        return dist2goal < gapGoalRange;
    }

    // In place modification
    void GapManipulator::reduceGap(dynamic_gap::Gap * gap, 
                                    const geometry_msgs::PoseStamped & globalPathLocalWaypoint, 
                                    const bool & initial) 
    {        
        try
        {
            // msg is from egocircle
            // only part of msg used is angle_increment

            int leftIdx = 0, rightIdx = 0;
            float leftDist = 0.0, rightDist = 0.0;
            if (initial) 
            {
                leftIdx = gap->cvxLeftIdx();
                rightIdx = gap->cvxRightIdx();
                leftDist = gap->cvxLeftDist();
                rightDist = gap->cvxRightDist();
            } else 
            {
                leftIdx = gap->cvxTermLeftIdx();
                rightIdx = gap->cvxTermRightIdx();
                leftDist = gap->cvxTermLeftDist();
                rightDist = gap->cvxTermRightDist();
            }

            float gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0.0)
                gapIdxSpan += cfg_->scan.full_scan_f; // (2*gap->half_scan);

            float gapAngle = gapIdxSpan * cfg_->scan.angle_increment;
            // ROS_INFO_STREAM_NAMED("GapManipulator",  "gap idx size: " << gapIdxSpan << std::endl;

            // threshold = pi right now
            if (gapAngle < cfg_->gap_manip.reduction_threshold)
                return;

            ROS_INFO_STREAM_NAMED("GapManipulator", "    [reduceGap()]");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-reduce gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");

            // the desired size for the reduced gap?
            // target is pi
            int targetGapIdxSpan = cfg_->gap_manip.reduction_threshold / cfg_->scan.angle_increment;
 
            int leftIdxBiasedRight = wrapScanIndex(leftIdx - targetGapIdxSpan);
            int rightIdxBiasedLeft = (rightIdx + targetGapIdxSpan) % cfg_->scan.full_scan; // num_of_scan is int version of 2*half_scan

            // ROS_INFO_STREAM_NAMED("GapManipulator", "rightIdxBiasedLeft: " << rightIdxBiasedLeft << ", leftIdxBiasedRight: " << leftIdxBiasedRight);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "r_biased_l: " << r_biased_l << ", l_biased_r: " << l_biased_r);

            float globalPathLocalWaypointTheta = std::atan2(globalPathLocalWaypoint.pose.position.y, globalPathLocalWaypoint.pose.position.x);
            int globalPathLocalWaypointIdx = theta2idx(globalPathLocalWaypointTheta); // globalPathLocalWaypointTheta / (M_PI / gap->half_scan) + gap->half_scan;
            ROS_INFO_STREAM_NAMED("GapManipulator", "        globalPathLocalWaypointIdx: " << globalPathLocalWaypointIdx);
            int halfTargetGapIdxSpan = targetGapIdxSpan / 2; // distance in scan indices

            //ROS_INFO_STREAM_NAMED("GapManipulator",  "goal orientation: " << globalPathLocalWaypointTheta << ", goal idx: " << goal_idx << ", acceptable distance: " << halfTargetGapIdxSpan << std::endl;

            leftIdxBiasedRight = wrapScanIndex(leftIdx - halfTargetGapIdxSpan);
            int leftIdxBiasedLeft = (leftIdx + halfTargetGapIdxSpan) % cfg_->scan.full_scan;

            int rightIdxBiasedRight = wrapScanIndex(rightIdx - halfTargetGapIdxSpan);
            rightIdxBiasedLeft = (rightIdx + halfTargetGapIdxSpan) % cfg_->scan.full_scan;

            bool isLocalWaypointLeftBiased = isGlobalPathLocalWaypointWithinGapAngle(globalPathLocalWaypointIdx, leftIdxBiasedRight, leftIdxBiasedLeft); 
            bool isLocalWaypointRightBiased = isGlobalPathLocalWaypointWithinGapAngle(globalPathLocalWaypointIdx, rightIdxBiasedRight, rightIdxBiasedLeft); 
            int newLeftIdx, newRightIdx;
            if (isLocalWaypointLeftBiased) // left biased
            {
                newLeftIdx = leftIdx;
                newRightIdx = leftIdxBiasedRight;   
                ROS_INFO_STREAM_NAMED("GapManipulator", "        creating left-biased gap: " << newLeftIdx << ", " << newRightIdx);
            } else if (isLocalWaypointRightBiased) // right biased
            {
                newLeftIdx = rightIdxBiasedLeft;
                newRightIdx = rightIdx;
                ROS_INFO_STREAM_NAMED("GapManipulator", "        creating right-biased gap: " << newLeftIdx << ", " << newRightIdx);
            } else // Lingering in center 
            { 
                //ROS_INFO_STREAM_NAMED("GapManipulator",  "central gap" << std::endl;
                newLeftIdx = (globalPathLocalWaypointIdx + halfTargetGapIdxSpan) % cfg_->scan.full_scan;
                newRightIdx = wrapScanIndex(globalPathLocalWaypointIdx - halfTargetGapIdxSpan);
                ROS_INFO_STREAM_NAMED("GapManipulator", "        creating goal-centered gap: " << newLeftIdx << ", " << newRightIdx);
            }

            // removed some float casting here
            float leftToNewLeftIdxSpan = wrapScanIndex(leftIdx - newLeftIdx);
            float leftToNewRightIdxSpan = wrapScanIndex(leftIdx - newRightIdx);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "orig_gap_size: " << orig_gap_size);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "leftToNewRightIdxSpan: " << leftToNewRightIdxSpan << ", leftToNewLeftIdxSpan: " << leftToNewLeftIdxSpan);

            float newLeftDist = leftDist + (rightDist - leftDist) * epsilonDivide(leftToNewLeftIdxSpan, gapIdxSpan);
            float newRightDist = leftDist +  (rightDist - leftDist) * epsilonDivide(leftToNewRightIdxSpan, gapIdxSpan);

            if (initial) 
            {
                gap->setCvxLeftIdx(newLeftIdx);
                gap->setCvxRightIdx(newRightIdx);
                gap->setCvxLeftDist(newLeftDist);            
                gap->setCvxRightDist(newRightDist);
                gap->mode.reduced_ = true;

            } else 
            {
                gap->setcvxTermLeftIdx(newLeftIdx);
                gap->setcvxTermRightIdx(newRightIdx);
                gap->setcvxTermLeftDist(newLeftDist);
                gap->setcvxTermRightDist(newRightDist);
                gap->mode.termReduced_ = true;

            }


            ROS_INFO_STREAM_NAMED("GapManipulator", "        post-reduce gap in polar. left: (" << newLeftIdx << ", " << newLeftDist << "), right: (" << newRightIdx << ", " << newRightDist << ")");
            // ROS_INFO_STREAM_NAMED("GapManipulator", "post-reduce in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
            return;
        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "[reduceGap() failed]");
        }
    }

    void GapManipulator::convertRadialGap(dynamic_gap::Gap * gap, const bool & initial) 
    {
        try 
        {
            // Return if not radial gap or disabled
            if (!gap->isRadial(initial) || !cfg_->gap_manip.radial_convert) 
            {
                return;
            }

            sensor_msgs::LaserScan desScan;
            int leftIdx = 0, rightIdx = 0;
            float leftDist = 0.0, rightDist = 0.0;
            if (initial) 
            {
                desScan = *scan_.get();
                leftIdx = gap->cvxLeftIdx();
                rightIdx = gap->cvxRightIdx();
                leftDist = gap->cvxLeftDist();
                rightDist = gap->cvxRightDist();
            } else 
            {
                desScan = *scan_.get(); // dynamicScan_;
                leftIdx = gap->cvxTermLeftIdx();
                rightIdx = gap->cvxTermRightIdx();
                leftDist = gap->cvxTermLeftDist();
                rightDist = gap->cvxTermRightDist();
            }

            ROS_INFO_STREAM_NAMED("GapManipulator", "    [convertRadialGap()]");            
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RGC gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");
            // ROS_INFO_STREAM_NAMED("GapManipulator", "pre-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

            bool right = gap->isRightType(initial);
            // Rotate radial gap by an amount theta so that its width is visible
            
            // start with a nominal pivot angle, we will adjust this angle to find the actual theta that we pivot by
            float nomPivotAngle = std::atan2(cfg_->gap_manip.epsilon2, cfg_->gap_manip.epsilon1) + 1e-3;
            // ROS_INFO_STREAM_NAMED("GapManipulator", "left: " << left << ", nomPivotAngle: " << nomPivotAngle);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "theta to pivot: " << theta);
            int nearIdx = 0, farIdx = 0;
            float nearDist = 0.0, farDist = 0.0;
            float nearTheta = 0.0, farTheta = 0.0;
            float signedNomPivotAngle = 0.0;

            if (right) 
            {
                nearIdx = rightIdx;
                nearDist = rightDist;
                farIdx = leftIdx;
                farDist = leftDist;
                signedNomPivotAngle = nomPivotAngle;
            } else 
            {
                nearIdx = leftIdx;
                farIdx = rightIdx;
                nearDist = leftDist;
                farDist = rightDist;
                signedNomPivotAngle = -nomPivotAngle;
            }

            Eigen::Matrix3f nomPivotAngleRotationMatrix;
            // nomPivotAngleRotationMatrix: SE(3) matrix that represents desired rotation amount
            nomPivotAngleRotationMatrix << cos(signedNomPivotAngle), -sin(signedNomPivotAngle), 0,
                                        sin(signedNomPivotAngle), cos(signedNomPivotAngle), 0,
                                        0, 0, 1;
            
            // obtaining near and far theta values from indices
            nearTheta = idx2theta(nearIdx);
            farTheta = idx2theta(farIdx);   

            Eigen::Matrix3f nearPtTranslationMatrix, farPtTranslationMatrix;
            // nearPtTranslationMatrix, farPtTranslationMatrix: SE(2) matrices that represent translation from rbt origin to near and far points
            nearPtTranslationMatrix << 1., 0., nearDist * cos(nearTheta),
                                    0., 1., nearDist * sin(nearTheta),
                                    0., 0., 1.;
            farPtTranslationMatrix  << 1., 0., farDist * cos(farTheta),
                                    0., 1., farDist * sin(farTheta),
                                    0., 0., 1.;

            // pivotedPtRotationMatrix: my guess, transformation matrix to desired pivot point using existing gap points
            // inverse flips direction of the translation
            Eigen::Matrix3f nearToFarTranslationMatrix = nearPtTranslationMatrix.inverse() * farPtTranslationMatrix;
            Eigen::Matrix3f pivotedPtRotationMatrix = nearPtTranslationMatrix * 
                                                            (nomPivotAngleRotationMatrix * 
                                                                nearToFarTranslationMatrix);
            // ROS_INFO_STREAM_NAMED("GapManipulator",  "pivotedPtRotationMatrix: " << pivotedPtRotationMatrix);

            // Extracting theta and idx of pivoted point
            float nomPivotedTheta = std::atan2(pivotedPtRotationMatrix(1, 2), pivotedPtRotationMatrix(0, 2));
            int nomPivotedIdx = theta2idx(nomPivotedTheta);

            // Search along current scan to from initial gap point to pivoted gap point
            // to obtain range value to assign to the pivoted gap point
            int scanSearchStartIdx = 0, scanSearchEndIdx = 0;
            if (right)
            {   
                scanSearchStartIdx = leftIdx;
                scanSearchEndIdx = nomPivotedIdx;
            } else
            {
                scanSearchStartIdx = nomPivotedIdx;
                scanSearchEndIdx = rightIdx;
            }

            // ROS_INFO_STREAM_NAMED("GapManipulator", "scanSearchStartIdx: " << scanSearchStartIdx << ", scanSearchEndIdx: " << scanSearchEndIdx);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "wrapped scanSearchStartIdx: " << scanSearchStartIdx << ", wrapped scanSearchEndIdx: " << scanSearchEndIdx);
            int scanSearchSize = scanSearchEndIdx - scanSearchStartIdx;
            if (scanSearchSize < 0)
                scanSearchSize += cfg_->scan.full_scan; // int(2*gap->half_scan);


            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // int(2*gap->half_scan);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "gapIdxSpan: " << gapIdxSpan);

            // using the law of cosines to find the index between init/final indices
            // that's shortest distance between near point and laser scan
            // ROS_INFO_STREAM_NAMED("GapManipulator", "ranges size: " << stored_scan_msgs.ranges.size());
            std::vector<float> nearPtToScanDists(scanSearchSize);

            int checkIdx = 0;
            float checkRange = 0.0, checkIdxSpan = 0.0;
            for (int i = 0; i < nearPtToScanDists.size(); i++) 
            {
                checkIdx = (i + scanSearchStartIdx) % cfg_->scan.full_scan; // int(2 * gap->half_scan);
                checkRange = desScan.ranges.at(checkIdx);
                checkIdxSpan = gapIdxSpan + (scanSearchSize - i);
                nearPtToScanDists.at(i) = sqrt(pow(nearDist, 2) + pow(checkRange, 2) -
                                            2.0 * nearDist * checkRange * cos(checkIdxSpan * cfg_->scan.angle_increment));
                // ROS_INFO_STREAM_NAMED("GapManipulator", "checking idx: " << checkIdx << ", range of: " << range << ", diff in idx: " << checkIdxSpan << ", dist of " << dist.at(i));
            }

            auto minDistIter = std::min_element(nearPtToScanDists.begin(), nearPtToScanDists.end());
            int minDistIdx = (scanSearchStartIdx + std::distance(nearPtToScanDists.begin(), minDistIter)) % cfg_->scan.full_scan; // int(2*gap->half_scan);
            float minDistRange = *minDistIter;

            // ROS_INFO_STREAM_NAMED("GapManipulator", "from " << scanSearchStartIdx << " to " << scanSearchEndIdx << ", min dist of " << minDistRange << " at " << minDistIdx);         

            // pivoting around near point, pointing towards far point or something?

            float nearToFarDistance = sqrt(pow(nearToFarTranslationMatrix(0, 2), 2) + pow(nearToFarTranslationMatrix(1, 2), 2));
            // ROS_INFO_STREAM_NAMED("GapManipulator", "nearToFarDistance: " << nearToFarDistance);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "translation norm: " << nearToFarDistance);
            
            // augmenting near to far direction by pivoted point, multiplying by min dist        
            nearToFarTranslationMatrix(0, 2) *= epsilonDivide(minDistRange, nearToFarDistance);     
            nearToFarTranslationMatrix(1, 2) *= epsilonDivide(minDistRange, nearToFarDistance);
            Eigen::Matrix3f pivotedPtMatrix = nearPtTranslationMatrix * (nomPivotAngleRotationMatrix * nearToFarTranslationMatrix);

            float pivotedPtDist = sqrt(pow(pivotedPtMatrix(0, 2), 2) + pow(pivotedPtMatrix(1, 2), 2));
            float pivotedPtTheta = std::atan2(pivotedPtMatrix(1, 2), pivotedPtMatrix(0, 2));

            int pivotedPtIdx = theta2idx(pivotedPtTheta); 

            float newLeftIdx = 0.0, newRightIdx = 0.0, newLeftDist = 0.0, newRightDist = 0.0;
            if (right) 
            {
                newLeftIdx = pivotedPtIdx;
                newLeftDist = pivotedPtDist;
                newRightIdx = nearIdx;
                newRightDist = nearDist;
            } else 
            {
                newLeftIdx = nearIdx;
                newLeftDist = nearDist;
                newRightIdx = pivotedPtIdx;
                newRightDist = pivotedPtDist;
            }

            if (initial) 
            {
                gap->setCvxLeftIdx(newLeftIdx);
                gap->setCvxRightIdx(newRightIdx);
                gap->setCvxLeftDist(newLeftDist);
                gap->setCvxRightDist(newRightDist);
                gap->mode.RGC_ = true;

            } else 
            {
                gap->setcvxTermLeftIdx(newLeftIdx);
                gap->setcvxTermRightIdx(newRightIdx);
                gap->setcvxTermLeftDist(newLeftDist);
                gap->setcvxTermRightDist(newRightDist);
                gap->mode.termRGC_ = true;

            }

            ROS_INFO_STREAM_NAMED("GapManipulator", "        post-RGC gap in polar. left: (" << newLeftIdx << ", " << newLeftDist << "), right: (" << newRightIdx << ", " << newRightDist << ")");
            // ROS_INFO_STREAM_NAMED("GapManipulator",  "post-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "convertRadialGap() failed");
        }
    }

    void GapManipulator::radialExtendGap(dynamic_gap::Gap * gap, const bool & initial) 
    {
        try
        {
            if (!cfg_->gap_manip.radial_extend)
                return;

            int leftIdx = 0, rightIdx = 0;
            float leftDist = 0.0, rightDist = 0.0;
            if (initial) 
            {
                leftIdx = gap->cvxLeftIdx();
                rightIdx = gap->cvxRightIdx();
                leftDist = gap->cvxLeftDist();
                rightDist = gap->cvxRightDist();
            } else 
            {
                leftIdx = gap->cvxTermLeftIdx();
                rightIdx = gap->cvxTermRightIdx();
                leftDist = gap->cvxTermLeftDist();
                rightDist = gap->cvxTermRightDist();
            }

            float leftTheta = idx2theta(leftIdx);
            float rightTheta = idx2theta(rightIdx);
            float xLeft = leftDist * cos(leftTheta);
            float yLeft = leftDist * sin(leftTheta);
            float xRight = rightDist * cos(rightTheta);
            float yRight = rightDist * sin(rightTheta);
            
            Eigen::Vector2f leftPt(xLeft, yLeft);
            Eigen::Vector2f rightPt(xRight, yRight);

            ROS_INFO_STREAM_NAMED("GapManipulator", "    [radialExtendGap()]");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RE gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RE gap in cart. left: (" << leftPt[0] << ", " << leftPt[1] << "), right: (" << rightPt[0] << ", " << rightPt[1] << ")");
            
            // ROS_INFO_STREAM_NAMED("GapManipulator", "leftPt: (" << xLeft << ", " << yLeft << "), rightPt: (" << xRight << ", " << yRight << ")");
            // ROS_INFO_STREAM_NAMED("GapManipulator", "eL_robot: (" << eL_robot[0] << ", " << eL_robot[1] << ") , eR_robot: (" << eR_robot[0] << ", " << eR_robot[1] << ")");

            float leftToRightAngle = getSweptLeftToRightAngle(leftPt, rightPt);

            float thetaCenter = (leftTheta - 0.5*leftToRightAngle);

            // middle of gap direction
            Eigen::Vector2f eB(std::cos(thetaCenter), std::sin(thetaCenter));
            // ROS_INFO_STREAM_NAMED("GapManipulator", "eB: (" << eB[0] << ", " << eB[1] << ")");

            Eigen::Vector2f norm_eB = unitNorm(eB); 
            // angular size of gap
            // ROS_INFO_STREAM_NAMED("GapManipulator", "normalized eB: " << norm_eB[0] << ", " << norm_eB[1]);

            // minSafeDist is the minimum distance within the laser scan 
            float s = initial ? gap->getMinSafeDist() : gap->getTerminalMinSafeDist();
            // ROS_INFO_STREAM_NAMED("GapManipulator", "min safe dist: " << s);
            
            // point opposite direction of middle of gap, magnitude of min safe dist
            Eigen::Vector2f extendedGapOrigin =  - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * norm_eB; //
            // ROS_INFO_STREAM_NAMED("GapManipulator", "extendedGapOrigin: " << extendedGapOrigin[0] << ", " << extendedGapOrigin[1]);

            
            if (initial) 
            {
                gap->extendedGapOrigin_ = extendedGapOrigin;
 
                gap->leftBezierOrigin_ =  Rnegpi2 * extendedGapOrigin;

                gap->rightBezierOrigin_ = Rpi2 * extendedGapOrigin;
                gap->mode.convex_ = true;
            
            } else 
            {
                gap->termExtendedGapOrigin_ = extendedGapOrigin;
                gap->mode.termConvex_ = true;
            }
            
            ROS_INFO_STREAM_NAMED("GapManipulator", "        finishing with gap extendedGapOrigin: " << extendedGapOrigin[0] << ", " << extendedGapOrigin[1]);

            return;
        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "radialGapExtension() failed");
        }
    }

    void GapManipulator::inflateGapSides(dynamic_gap::Gap * gap, const bool & initial) 
    {
        try 
        {
            // get points

            int leftIdx = 0, rightIdx = 0;
            float leftDist = 0.0, rightDist = 0.0;

            sensor_msgs::LaserScan desScan;
            if (initial) 
            {
                desScan = *scan_.get();
                leftIdx = gap->cvxLeftIdx();
                rightIdx = gap->cvxRightIdx();
                leftDist = gap->cvxLeftDist();
                rightDist = gap->cvxRightDist();
            } else 
            {
                desScan = *scan_.get(); // dynamicScan_;
                leftIdx = gap->cvxTermLeftIdx();
                rightIdx = gap->cvxTermRightIdx();
                leftDist = gap->cvxTermLeftDist();
                rightDist = gap->cvxTermRightDist();
            }

            float leftTheta = idx2theta(leftIdx);
            float rightTheta = idx2theta(rightIdx);
            float xLeft = (leftDist) * cos(leftTheta);
            float yLeft = (leftDist) * sin(leftTheta);
            float xRight = (rightDist) * cos(rightTheta);
            float yRight = (rightDist) * sin(rightTheta);
            
            Eigen::Vector2f leftPt(xLeft, yLeft);
            Eigen::Vector2f rightPt(xRight, yRight);
   
            ROS_INFO_STREAM_NAMED("GapManipulator", "    [inflateGapSides()]");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-inflate gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-inflate gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

            Eigen::Vector2f leftUnitNorm = unitNorm(leftPt);
            Eigen::Vector2f rightUnitNorm = unitNorm(rightPt);
            float leftToRightAngle = getSweptLeftToRightAngle(leftUnitNorm, rightUnitNorm);

            // inflate inwards by radius * infl
            // rotate by pi/2, norm
            
            // PERFORMING ANGULAR INFLATION
            Eigen::Vector2f leftInflationVector = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * Rnegpi2 * leftUnitNorm;
            Eigen::Vector2f inflatedLeftPt = leftPt + leftInflationVector;
            float inflatedLeftTheta = std::atan2(inflatedLeftPt[1], inflatedLeftPt[0]);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "leftTheta: " << leftTheta << ", inflatedLeftTheta: " << inflatedLeftTheta); // << ", leftThetaeft_infl: " << leftThetaeft_infl

            Eigen::Vector2f rightInflationVector = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * Rpi2 * rightUnitNorm;
            Eigen::Vector2f inflatedRightPt = rightPt + rightInflationVector;
            float inflatedRightTheta = std::atan2(inflatedRightPt[1], inflatedRightPt[0]);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "rightTheta: " << rightTheta << ", inflatedRightTheta: " << inflatedRightTheta); // ", rightTheta_infl: " << rightTheta_infl << 

            Eigen::Vector2f inflatedLeftUnitNorm(std::cos(inflatedLeftTheta), std::sin(inflatedLeftTheta));
            Eigen::Vector2f inflatedRightUnitNorm(std::cos(inflatedRightTheta), std::sin(inflatedRightTheta));
            float newLeftToRightAngle = getSignedLeftToRightAngle(inflatedLeftUnitNorm, inflatedRightUnitNorm);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "newLeftToRightAngle: " << newLeftToRightAngle);

            int inflatedLeftIdx = 0, inflatedRightIdx = 0;
            float inflatedLeftRange = 0.0, inflatedRightRange = 0.0;
            if (newLeftToRightAngle < 0) 
            {
                // ROS_INFO_STREAM_NAMED("GapManipulator", "angular inflation would cause gap to cross, not running:");
                inflatedRightIdx = rightIdx;
                inflatedLeftIdx = leftIdx;
                inflatedLeftRange = leftDist;
                inflatedRightRange = rightDist;
            } else 
            {
                // need to make sure L/R don't cross each other
                inflatedRightIdx = theta2idx(inflatedRightTheta);
                inflatedLeftIdx = theta2idx(inflatedLeftTheta);
                
                float leftToInflatedLeftAngle = getSignedLeftToRightAngle(leftUnitNorm, inflatedLeftUnitNorm);
                float leftToInflatedRightAngle = getSignedLeftToRightAngle(leftUnitNorm, inflatedRightUnitNorm);
                inflatedLeftRange = leftDist + (rightDist - leftDist) * epsilonDivide(leftToInflatedLeftAngle, leftToRightAngle);
                inflatedRightRange =  leftDist + (rightDist - leftDist) * epsilonDivide(leftToInflatedRightAngle, leftToRightAngle);
                // if (cfg_->debug.manipulation_debug_log) 
                // {
                //     ROS_INFO_STREAM_NAMED("GapManipulator", "        post-angular-inflation gap, left: " << inflatedLeftIdx << ", : " << inflatedLeftRange << 
                //                                                     ", right: " << inflatedRightIdx << ", : " << inflatedRightRange);
                //     // if (inflatedLeftRange > 8 || inflatedRightRange > 8) {
                //     //     ROS_INFO_STREAM_NAMED("GapManipulator", "            range is too big");
                //     // }
                // }
            }

            // PERFORMING RADIAL INFLATION
            inflatedLeftRange += 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            if ((desScan.ranges.at(inflatedLeftIdx) - inflatedLeftRange) < (cfg_->rbt.r_inscr * cfg_->traj.inf_ratio)) // reject the change
                inflatedLeftRange = desScan.ranges.at(inflatedLeftIdx) - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

            inflatedRightRange += 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            if ((desScan.ranges.at(inflatedRightIdx) - inflatedRightRange) < (cfg_->rbt.r_inscr * cfg_->traj.inf_ratio)) // reject the change
                inflatedRightRange = desScan.ranges.at(inflatedRightIdx) - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

            // ROS_INFO_STREAM_NAMED("GapManipulator", "new_l_theta: " << new_l_theta << ", new_r_theta: " << new_r_theta);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "int values, left: " << inflatedLeftIdx << ", right: " << inflatedRightIdx);

            if (inflatedRightIdx == inflatedLeftIdx) // ROS_INFO_STREAM_NAMED("GapManipulator", "manipulated indices are same");
                inflatedLeftIdx++;

            if (initial) 
            {
                gap->setCvxLeftIdx(inflatedLeftIdx);
                gap->setCvxRightIdx(inflatedRightIdx);      
                gap->setCvxLeftDist(inflatedLeftRange);
                gap->setCvxRightDist(inflatedRightRange);
                gap->getSimplifiedLCartesian(xLeft, yLeft);
                gap->getSimplifiedRCartesian(xRight, yRight);
            } else 
            {
                gap->setcvxTermLeftIdx(inflatedLeftIdx);
                gap->setcvxTermRightIdx(inflatedRightIdx);
                gap->setcvxTermLeftDist(inflatedLeftRange);
                gap->setcvxTermRightDist(inflatedRightRange);
                gap->getSimplifiedTerminalLCartesian(xLeft, yLeft);
                gap->getSimplifiedTerminalRCartesian(xRight, yRight);
            }

            ROS_INFO_STREAM_NAMED("GapManipulator", "        post-inflate gap in polar. left: (" << inflatedLeftIdx << ", " << inflatedLeftRange << "), right: (" << inflatedRightIdx << ", " << inflatedRightRange << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        post-inflate gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "[inflateGapSides() failed]");
        }
    }
}
