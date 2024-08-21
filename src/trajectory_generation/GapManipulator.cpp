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
            setGapGoal(gap, globalPathLocalWaypoint, false);

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
            
            if (isGlobalPathLocalWaypointWithinGapAngle(globalPathLocalWaypointIdx, rightIdx, leftIdx) && 
                checkWaypointVisibility(leftPt, rightPt, globalPathLocalWaypointVector))
            {
                gap->setGoal(initial, globalPathLocalWaypointVector);

                std::string goalStatus = "Option 2: global path local waypoint: ";
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

    Eigen::Vector2d GapManipulator::getRadialExtension(const float & s, const Eigen::Vector2d & p1, const bool & left)
    {
        ROS_INFO_STREAM_NAMED("GapManipulator", "        [getRadialExtension()]");

        float theta1 = std::atan2(p1(1), p1(0));
        float p1_norm = p1.norm();

        ROS_INFO_STREAM_NAMED("GapManipulator", "        theta1: " << theta1 << ", p1_norm: " << p1_norm);

        float d_safe = s;
        ROS_INFO_STREAM_NAMED("GapManipulator", "        d_safe: " << d_safe);

        if (s >= p1_norm)
            d_safe = 0.5 * p1_norm;

        float D = std::sqrt(p1_norm * p1_norm - d_safe * d_safe);
        float beta = std::acos( (D*D - p1_norm*p1_norm - d_safe*d_safe) / (-2 * p1_norm * d_safe));
        ROS_INFO_STREAM_NAMED("GapManipulator", "        D: " << D << ", beta: " << beta);

        float theta0 = theta1 + (left ? 1 : -1) * beta;
        ROS_INFO_STREAM_NAMED("GapManipulator", "        theta0: " << theta0);

        Eigen::Vector2d p0(d_safe * std::cos(theta0), d_safe * std::sin(theta0));

        return p0;
    }

    void GapManipulator::radialExtendGap(dynamic_gap::Gap * gap) 
    {
        try
        {
            if (!cfg_->gap_manip.radial_extend)
                return;

            int leftIdx = gap->cvxLeftIdx();
            int rightIdx = gap->cvxRightIdx();
            float leftDist = gap->cvxLeftDist();
            float rightDist = gap->cvxRightDist();

            float leftTheta = idx2theta(leftIdx);
            float rightTheta = idx2theta(rightIdx);
            float xLeft = leftDist * cos(leftTheta);
            float yLeft = leftDist * sin(leftTheta);
            float xRight = rightDist * cos(rightTheta);
            float yRight = rightDist * sin(rightTheta);
            
            Eigen::Vector2d leftPt(xLeft, yLeft);
            Eigen::Vector2d rightPt(xRight, yRight);

            ROS_INFO_STREAM_NAMED("GapManipulator", "    [radialExtendGap()]");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RE gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");
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
            float newLeftToRightAngle = -1.0;

            float infl_ratio_decr = 0.99;

            float infl_ratio = cfg_->traj.inf_ratio / infl_ratio_decr;
            float inflatedLeftTheta = 0.0, inflatedRightTheta = 0.0;
            Eigen::Vector2f inflatedLeftUnitNorm, inflatedRightUnitNorm;
            while (newLeftToRightAngle < 0 && infl_ratio >= 1.0)
            {
                // infl_ratio can be too large that it would destroy gap, so we will decrement the ratio until we can inflate safely
                infl_ratio = infl_ratio_decr * infl_ratio;
                // ROS_INFO_STREAM_NAMED("GapManipulator", "                infl_ratio: " << infl_ratio); // << ", leftThetaeft_infl: " << leftThetaeft_infl

                Eigen::Vector2f leftInflationVector = cfg_->rbt.r_inscr * infl_ratio * Rnegpi2 * leftUnitNorm;
                Eigen::Vector2f inflatedLeftPt = leftPt + leftInflationVector;
                inflatedLeftTheta = std::atan2(inflatedLeftPt[1], inflatedLeftPt[0]);

                // ROS_INFO_STREAM_NAMED("GapManipulator", "                leftTheta: " << leftTheta << ", inflatedLeftTheta: " << inflatedLeftTheta); // << ", leftThetaeft_infl: " << leftThetaeft_infl

                Eigen::Vector2f rightInflationVector = cfg_->rbt.r_inscr * infl_ratio * Rpi2 * rightUnitNorm;
                Eigen::Vector2f inflatedRightPt = rightPt + rightInflationVector;
                inflatedRightTheta = std::atan2(inflatedRightPt[1], inflatedRightPt[0]);

                // ROS_INFO_STREAM_NAMED("GapManipulator", "                rightTheta: " << rightTheta << ", inflatedRightTheta: " << inflatedRightTheta); // ", rightTheta_infl: " << rightTheta_infl << 

                inflatedLeftUnitNorm << std::cos(inflatedLeftTheta), std::sin(inflatedLeftTheta);
                inflatedRightUnitNorm << std::cos(inflatedRightTheta), std::sin(inflatedRightTheta);
                newLeftToRightAngle = getSignedLeftToRightAngle(inflatedLeftUnitNorm, inflatedRightUnitNorm);
                // ROS_INFO_STREAM_NAMED("GapManipulator", "                newLeftToRightAngle: " << newLeftToRightAngle);
            }

            if (newLeftToRightAngle < 0)
                return;

            // if (newLeftToRightAngle < 0) // want to allow terminal gaps where initial were RGC'd to inflate 
            // {
            //     ROS_INFO_STREAM_NAMED("GapManipulator", "angular inflation would cause gap to cross, not running:");
            //     inflatedLeftIdx = leftIdx;
            //     inflatedRightIdx = rightIdx;

            //     inflatedLeftRange = leftDist;
            //     inflatedRightRange = rightDist;
            // } else 
            // {
                // need to make sure L/R don't cross each other

                // int inflatedLeftIdx = 0, inflatedRightIdx = 0;
                // float inflatedLeftRange = 0.0, inflatedRightRange = 0.0;
            float inflatedLeftIdx = theta2idx(inflatedLeftTheta);
            float inflatedRightIdx = theta2idx(inflatedRightTheta);
            
            float leftToInflatedLeftAngle = getSignedLeftToRightAngle(leftUnitNorm, inflatedLeftUnitNorm);
            float leftToInflatedRightAngle = getSignedLeftToRightAngle(leftUnitNorm, inflatedRightUnitNorm);
            float inflatedLeftRange = leftDist + (rightDist - leftDist) * epsilonDivide(leftToInflatedLeftAngle, leftToRightAngle);
            float inflatedRightRange =  leftDist + (rightDist - leftDist) * epsilonDivide(leftToInflatedRightAngle, leftToRightAngle);
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
