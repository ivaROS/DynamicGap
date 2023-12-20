#include <dynamic_gap/trajectory_generation/GapManipulator.h>

namespace dynamic_gap 
{
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) 
    {
        boost::mutex::scoped_lock lock(egolock);
        scan_ = msg_;
    }

    void GapManipulator::updateStaticEgoCircle(const sensor_msgs::LaserScan & staticScan) 
    {
        boost::mutex::scoped_lock lock(egolock);
        staticScan_ = staticScan;
    }

    void GapManipulator::updateDynamicEgoCircle(dynamic_gap::Gap& gap,
                                                const std::vector<sensor_msgs::LaserScan> & future_scans) 
    {
        dynamicScan_ = staticScan_;
        float t_iplus1 = gap.gapLifespan_;

        int futureScanIdx = (int) (t_iplus1 / cfg_->traj.integrate_stept);

        dynamicScan_ = future_scans[futureScanIdx];

        float dynamicScanMinDist = *std::min_element(dynamicScan_.ranges.begin(), dynamicScan_.ranges.end());
        gap.setTerminalMinSafeDist(dynamicScanMinDist);
    }
    
    void GapManipulator::setTerminalGapWaypoint(dynamic_gap::Gap& gap, const geometry_msgs::PoseStamped & globalPathLocalWaypoint) 
    {
        try
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "    [setTerminalGapWaypoint()]");
            
            if (gap.getCategory() == "expanding" || gap.getCategory() == "static") 
            { 
                ROS_INFO_STREAM_NAMED("GapManipulator", "setting terminal goal for expanding gap");
                setGapWaypoint(gap, globalPathLocalWaypoint, false);
            } else if (gap.getCategory() == "closing") 
            {
                std::string closingGapType;
                if (gap.crossed_) 
                {
                    closingGapType = "crossed";
                    Eigen::Vector2f crossingPt = gap.getCrossingPoint();

                    gap.terminalGoal.x_ = crossingPt[0];
                    gap.terminalGoal.y_ = crossingPt[1];
                    // gap.terminalGoal.set = true;
                } else if (gap.closed_) 
                {
                    closingGapType = "closed";
                    ROS_INFO_STREAM_NAMED("GapManipulator", "        setting terminal goal for closed closing gap");
                    Eigen::Vector2f closing_pt = gap.getClosingPoint();
                
                    gap.terminalGoal.x_ = closing_pt[0];
                    gap.terminalGoal.y_ = closing_pt[1];
                    // gap.terminalGoal.set = true;
                } else 
                {
                    closingGapType = "existent";
                    setGapWaypoint(gap, globalPathLocalWaypoint, false);
                }

                ROS_INFO_STREAM_NAMED("GapManipulator", "        setting terminal goal for " + closingGapType + " closing gap");
            }
            ROS_INFO_STREAM_NAMED("GapManipulator", "    [setTerminalGapWaypoint() finished]");        
        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "[setTerminalGapWaypoint() failed]");
        }
    }

    void GapManipulator::setGapWaypoint(dynamic_gap::Gap& gap, const geometry_msgs::PoseStamped & globalPathLocalWaypoint, bool initial) 
    {
        try
        {
            ROS_INFO_STREAM_NAMED("GapManipulator", "    [setGapWaypoint()]");

            int leftIdx, rightIdx;
            float leftDist, rightDist;
            if (initial) 
            {
                leftIdx = gap.cvxLeftIdx();
                rightIdx = gap.cvxRightIdx();
                leftDist = gap.cvxLeftDist();
                rightDist = gap.cvxRightDist();
            } else 
            {
                leftIdx = gap.cvxTermLeftIdx();
                rightIdx = gap.cvxTermRightIdx();
                leftDist = gap.cvxTermLeftDist();
                rightDist = gap.cvxTermRightDist();
            }

            // int leftIdx = initial ? gap.cvxLeftIdx() : gap.cvxTermLeftIdx();
            // int rightIdx = initial ? gap.cvxRightIdx() : gap.cvxTermRightIdx();
            // float leftDist = initial ? gap.cvxLeftDist() : gap.cvxTermLeftDist();
            // float rightDist = initial ? gap.cvxRightDist() : gap.cvxTermRightDist();

            float leftTheta = idx2theta(leftIdx);
            float rightTheta = idx2theta(rightIdx);

            float xLeft = (leftDist) * cos(leftTheta);
            float yLeft = (leftDist) * sin(leftTheta);
            float xRight = (rightDist) * cos(rightTheta);
            float yRight = (rightDist) * sin(rightTheta);

            Eigen::Vector2f leftPt(xLeft, yLeft);
            Eigen::Vector2f rightPt(xRight, yRight);

            // auto lr = (pl - pr) / (pl - pr).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pr;
            // auto leftTheta = std::atan2(leftPt[1], leftPt[0]);
            // auto lf = (pr - pl) / (pr - pl).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pl; // why are we doing this? we are inflating already.
            // auto rightTheta = std::atan2(rightPt[1], rightPt[0]);

            ROS_INFO_STREAM_NAMED("GapManipulator", "    [setGapWaypoint()]");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        gap polar points, left: (" << leftIdx << ", " << leftDist << ") , right: (" << rightIdx << ", " << rightDist << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        gap cart points, left: (" << xLeft << ", " << yLeft << ") , right: (" << xRight << ", " << yRight << ")");

            // Eigen::Vector2f left_vect_robot = leftPt / leftPt.norm();
            // Eigen::Vector2f right_vect_robot = rightPt / rightPt.norm();
            float leftToRightAngle = getLeftToRightAngle(leftPt, rightPt, true);

            // Second condition: if angle smaller than M_PI / 3
            // Check if arc length < 3 robot width

            Eigen::Vector2f globalPathLocalWaypointVector(globalPathLocalWaypoint.pose.position.x, globalPathLocalWaypoint.pose.position.y);

            // if (gap.artificial_) 
            // {
            //     if (initial) 
            //     {
            //         gap.goal.x_ = globalPathLocalWaypointVector[0];
            //         gap.goal.y_ = globalPathLocalWaypointVector[1];
            //         // gap.goal.set = true;
            //         // gap.goal.goalwithin = true;
            //     } else 
            //     {
            //         gap.terminalGoal.x_ = globalPathLocalWaypointVector[0];
            //         gap.terminalGoal.y_ = globalPathLocalWaypointVector[1];
            //         // gap.terminalGoal.set = true;
            //         // gap.terminalGoal.goalwithin = true;
            //     }

            //     if (cfg_->debug.manipulation_debug_log) 
            //     {
            //         ROS_INFO_STREAM_NAMED("GapManipulator", "        Option 0: artificial gap");
            //         ROS_INFO_STREAM_NAMED("GapManipulator", "            goal: " << globalPathLocalWaypointVector[0] << ", " << globalPathLocalWaypointVector[1]);
            //         // Eigen::Vector2f goalPt(globalPathLocalWaypoint.pose.position.x, globalPathLocalWaypoint.pose.position.y);
            //         // Eigen::Vector2f goal_norm_vector = goalPt / goalPt.norm();
            //         // float leftToGoalAngle = getLeftToRightAngle(leftPt, goalPt, true);
            //         // if (! (0.0 < leftToGoalAngle && leftToGoalAngle < leftToRightAngle)) 
            //         // {
            //         //     ROS_INFO_STREAM_NAMED("GapManipulator", "            goal outside of gap");
            //         //     ROS_INFO_STREAM_NAMED("GapManipulator", "            leftToGoalAngle: " << leftToGoalAngle << ", leftToRightAngle: " << leftToRightAngle);
            //         //     float goal_theta = std::atan2(globalPathLocalWaypoint.pose.position.y, globalPathLocalWaypoint.pose.position.x);
            //         //     ROS_INFO_STREAM_NAMED("GapManipulator", "            left_theta: " << leftTheta << ", goal_theta: " << goal_theta << ", right_theta: " << rightTheta);
            //         // }
            //     }
            //     return;
            // }

            // bool gap_size_check = ;
            // float dist = 0;
            // bool small_gap = false;
            // if (gap_size_check) 
            // {
            //     // if smaller than M_PI/3
            //     dist = ;
            //     small_gap = dist < 4 * cfg_->rbt.r_inscr;
            //     // need to revise to be purely angle based
            // }

            bool smallGap = (leftToRightAngle < M_PI && sqrt(pow(xLeft - xRight, 2) + pow(yLeft - yRight, 2)) < 4 * cfg_->rbt.r_inscr);
            if (smallGap) 
            {
                // Eigen::Vector2f leftUnitNorm = leftPt / leftPt.norm();
                // Eigen::Vector2f rightUnitNorm = rightPt / rightPt.norm();
                
                float leftToRightAngle = getLeftToRightAngle(leftPt, rightPt, true);
                
                float thetaLeft = std::atan2(leftPt[1], leftPt[0]);

                // ROS_INFO_STREAM_NAMED("GapManipulator", "leftToRightAngle: " << leftToRightAngle);
                float thetaCenter = (thetaLeft - 0.5 * leftToRightAngle); 
                float rangeCenter = (leftPt.norm() + rightPt.norm()) / 2.0;
                float goalX = rangeCenter * std::cos(thetaCenter);
                float goalY = rangeCenter * std::sin(thetaCenter);
                // ROS_INFO_STREAM_NAMED("GapManipulator", "thetaLeft: " << thetaLeft << ", thetaRight: " << thetaRight << ", thetaCenter: " << thetaCenter);

                if (initial) 
                {
                    // gap.goal.set = true;
                    gap.goal.x_ = goalX;
                    gap.goal.y_ = goalY;
                } else 
                {
                    // gap.terminalGoal.set = true;
                    gap.terminalGoal.x_ = goalX;
                    gap.terminalGoal.y_ = goalY;
                }

                ROS_INFO_STREAM_NAMED("GapManipulator", "        Option 1: small gap");
                ROS_INFO_STREAM_NAMED("GapManipulator", "            goal: " << goalX << ", " << goalY);
                // Eigen::Vector2f goalPt(goalX, goalY);
                // // Eigen::Vector2f goal_norm_vector = goalPt / goalPt.norm();
                // float leftToGoalAngle = getLeftToRightAngle(leftPt, goalPt, true);
                // if (! (0.0 < leftToGoalAngle && leftToGoalAngle < leftToRightAngle)) 
                // {
                //     ROS_INFO_STREAM_NAMED("GapManipulator", "            goal outside of gap");
                //     ROS_INFO_STREAM_NAMED("GapManipulator", "            leftToGoalAngle: " << leftToGoalAngle << ", leftToRightAngle: " << leftToRightAngle);
                //     float goal_theta = std::atan2(goalY, goalX);
                //     ROS_INFO_STREAM_NAMED("GapManipulator", "            left_theta: " << leftTheta << ", goal_theta: " << goal_theta << ", right_theta: " << rightTheta);
                // }                

                return;
            }

            // sensor_msgs::LaserScan stored_scan_msgs = *scan_.get(); // initial ? *msg.get() : dynamic_laser_scan;
            float globalPathLocalWaypointTheta = std::atan2(globalPathLocalWaypointVector[1], globalPathLocalWaypointVector[0]);
            float globalPathLocalWaypointIdx = theta2idx(globalPathLocalWaypointTheta); // std::floor(goal_orientation*half_num_scan/M_PI + half_num_scan);
            
            ROS_INFO_STREAM_NAMED("GapManipulator", "        local goal idx: " << globalPathLocalWaypointIdx << 
                                        ", local goal x/y: (" << globalPathLocalWaypointVector[0] << 
                                                         ", " << globalPathLocalWaypointVector[1] << ")");

            // bool goal_within_gap_angle = ; // is localgoal within gap angle
            // ROS_INFO_STREAM_NAMED("GapManipulator", "goal_vis: " << goal_vis << ", " << goal_in_range);
            
            if (gap.artificial_ || (isGlobalPathLocalWaypointWithinGapAngle(globalPathLocalWaypointIdx, rightIdx, leftIdx) && 
                                    checkWaypointVisibility(leftPt, rightPt, globalPathLocalWaypointVector)))
            {
                if (initial) 
                {
                    gap.goal.x_ = globalPathLocalWaypointVector[0];
                    gap.goal.y_ = globalPathLocalWaypointVector[1];
                    // gap.goal.set = true;
                    // gap.goal.goalwithin = true;
                } else 
                {
                    gap.terminalGoal.x_ = globalPathLocalWaypointVector[0];
                    gap.terminalGoal.y_ = globalPathLocalWaypointVector[1];
                    // gap.terminalGoal.set = true;
                    // gap.terminalGoal.goalwithin = true;

                }


                std::string goalStatus = gap.artificial_ ? "Option 0: artificial gap: " : "Option 2: global path local waypoint: ";
                ROS_INFO_STREAM_NAMED("GapManipulator", "        " << goalStatus);
                ROS_INFO_STREAM_NAMED("GapManipulator", "            goal: " << globalPathLocalWaypointVector[0] << ", " << globalPathLocalWaypointVector[1]);
                // Eigen::Vector2f goalPt(globalPathLocalWaypoint.pose.position.x, globalPathLocalWaypoint.pose.position.y);
                // Eigen::Vector2f goal_norm_vector = goalPt / goalPt.norm();
                // float leftToGoalAngle = getLeftToRightAngle(leftPt, goalPt, true);
                // if (! (0.0 < leftToGoalAngle && leftToGoalAngle < leftToRightAngle)) 
                // {
                //     ROS_INFO_STREAM_NAMED("GapManipulator", "            goal outside of gap");
                //     ROS_INFO_STREAM_NAMED("GapManipulator", "            leftToGoalAngle: " << leftToGoalAngle << ", leftToRightAngle: " << leftToRightAngle);  
                //     float goal_theta = std::atan2(globalPathLocalWaypoint.pose.position.y, globalPathLocalWaypoint.pose.position.x);
                //     ROS_INFO_STREAM_NAMED("GapManipulator", "            left_theta: " << leftTheta << ", goal_theta: " << goal_theta << ", right_theta: " << rightTheta);
                // }                      
                return;
            }

            // if agc. then the shorter side need to be further in
            // lf: front (first one, left from laser scan)
            // lr: rear (second one, right from laser scan)
            // what do these do?
            ROS_INFO_STREAM_NAMED("GapManipulator", "        Option 3: biasing gap goal towards global path local waypoint");

            // Eigen::Vector2f local_goal_norm_vect(std::cos(globalPathLocalWaypointTheta), std::sin(globalPathLocalWaypointTheta));
            float leftToGoalAngle = getLeftToRightAngle(leftPt, globalPathLocalWaypointVector, true);
            float rightToGoalAngle = getLeftToRightAngle(rightPt, globalPathLocalWaypointVector, true);
    
            float biasedGapGoalTheta = setBiasedGapGoalTheta(leftTheta, rightTheta, globalPathLocalWaypointTheta,
                                                            leftToRightAngle, rightToGoalAngle, leftToGoalAngle);

            float biasedGapGoalIdx = theta2idx(biasedGapGoalTheta); // std::floor(biasedGapGoalTheta*half_num_scan/M_PI + half_num_scan);

            Eigen::Vector2f biasedGapGoalUnitNorm(std::cos(biasedGapGoalTheta), std::sin(biasedGapGoalTheta));

            // needs to be distance between L/confined. Always positive.
            float leftToGapGoalAngle = getLeftToRightAngle(leftPt, biasedGapGoalUnitNorm, false); 

            ROS_INFO_STREAM_NAMED("GapManipulator", "            biasedGapGoalTheta: " << biasedGapGoalTheta);
            ROS_INFO_STREAM_NAMED("GapManipulator", "            biasedGapGoalIdx: " << biasedGapGoalIdx);
            ROS_INFO_STREAM_NAMED("GapManipulator", "            leftToGapGoalAngle: " << leftToGapGoalAngle << ", leftToRightAngle: " << leftToRightAngle);

            // float leftDist_robot = leftDist;
            // float rightDist_robot = rightDist;

            float biasedGapGoalDist = leftDist + (rightDist - leftDist) * leftToGapGoalAngle / leftToRightAngle;
            Eigen::Vector2f biasedGapGoal(biasedGapGoalDist * cos(biasedGapGoalTheta), biasedGapGoalDist * sin(biasedGapGoalTheta));

            Eigen::Vector2f gapGoalAngularOffset(0.0, 0.0); 
            if ( (leftToGapGoalAngle / leftToRightAngle) < 0.1) // biased gap goal on left side
                gapGoalAngularOffset = Rnegpi2 * unitNorm(leftPt) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;    
            else                                                // biased gap goal on right side
                gapGoalAngularOffset = Rpi2 * unitNorm(rightPt) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

            /*
            if (biasedGapGoalTheta == rightTheta) 
            {
                gapGoalAngularOffset = Rpi2 * (rightPt / rightPt.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            } else if (biasedGapGoalTheta == leftTheta) {
                gapGoalAngularOffset = Rnegpi2 * (leftPt / leftPt.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            } else { // biasedGapGoalTheta == localGoal_theta
                if (leftToGapGoalAngle / leftToRightAngle < 0.1) { // pretty close to left side
                    gapGoalAngularOffset = Rnegpi2 * (leftPt / leftPt.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;    
                } else if (leftToGapGoalAngle / leftToRightAngle > 0.9) { // pretty close to right side
                    gapGoalAngularOffset = Rpi2 * (rightPt / rightPt.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
                }
            }
            */

            Eigen::Vector2f gapGoalRadialOffset = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * unitNorm(biasedGapGoal);
            Eigen::Vector2f gapGoalOffset = gapGoalRadialOffset + gapGoalAngularOffset;
            
            ROS_INFO_STREAM_NAMED("GapManipulator", "            biasedGapGoal: " << biasedGapGoal[0] << ", " << biasedGapGoal[1]);
            ROS_INFO_STREAM_NAMED("GapManipulator", "            gapGoalRadialOffset: " << gapGoalRadialOffset[0] << ", " << gapGoalRadialOffset[1]);
            ROS_INFO_STREAM_NAMED("GapManipulator", "            gapGoalAngularOffset: " << gapGoalAngularOffset[0] << ", " << gapGoalAngularOffset[1]);
            
            Eigen::Vector2f offsetBiasedGapGoal = gapGoalOffset + biasedGapGoal;
            // ROS_INFO_STREAM_NAMED("GapManipulator", "anchor: (" << anchor[0] << ", " << anchor[1] << "), offset with r_ins " << cfg_->rbt.r_inscr << " and inf ratio " << cfg_->traj.inf_ratio << ", :(" << offset[0] << ", " << offset[1] << "), offsetBiasedGapGoal: (" << offsetBiasedGapGoal[0] << ", " << offsetBiasedGapGoal[1] << ")");
            if (initial) 
            {
                gap.goal.x_ = offsetBiasedGapGoal(0);
                gap.goal.y_ = offsetBiasedGapGoal(1);
                // gap.goal.set = true;
            } else {
                gap.terminalGoal.x_ = offsetBiasedGapGoal(0);
                gap.terminalGoal.y_ = offsetBiasedGapGoal(1);
                // gap.terminalGoal.set = true;
            }

            ROS_INFO_STREAM_NAMED("GapManipulator", "            finished with goal: " << offsetBiasedGapGoal(0) << ", " << offsetBiasedGapGoal(1)); 
            // Eigen::Vector2f goalPt = offsetBiasedGapGoal;
            // // Eigen::Vector2f goal_norm_vector = goalPt / goalPt.norm();
            // float leftToGoalAngle = getLeftToRightAngle(leftPt, goalPt, true);
            // if (! (0.0 < leftToGoalAngle && leftToGoalAngle < leftToRightAngle)) 
            // {
            //     ROS_INFO_STREAM_NAMED("GapManipulator", "            goal outside of gap");
            //     ROS_INFO_STREAM_NAMED("GapManipulator", "            leftToGoalAngle: " << leftToGoalAngle << ", leftToRightAngle: " << leftToRightAngle);
            //     float goal_theta = std::atan2(goalPt[1], goalPt[0]);
            //     ROS_INFO_STREAM_NAMED("GapManipulator", "            left_theta: " << leftTheta << ", goal_theta: " << goal_theta << ", right_theta: " << rightTheta);         
            // }                     

        } catch (...)
        {
            ROS_FATAL_STREAM_NAMED("GapManipulator", "[setGapWaypoint() failed]");
        }      
    }

    float GapManipulator::setBiasedGapGoalTheta(float leftTheta, float rightTheta, float globalPathLocalWaypointTheta,
                                                float leftToRightAngle, float rightToGoalAngle, float leftToGoalAngle)
    {
        float biasedGapGoalTheta;
        if (leftTheta > rightTheta) // gap is not behind robot
        { 
            biasedGapGoalTheta = std::min(leftTheta, std::max(rightTheta, globalPathLocalWaypointTheta));
        } else // gap is behind
        { 
            if (0 < leftToGoalAngle && leftToGoalAngle < leftToRightAngle)
                biasedGapGoalTheta = globalPathLocalWaypointTheta;
            else if (std::abs(leftToGoalAngle) < std::abs(rightToGoalAngle))
                biasedGapGoalTheta = leftTheta;
            else
                biasedGapGoalTheta = rightTheta;
        }

        ROS_INFO_STREAM_NAMED("GapManipulator", "            leftTheta: " << leftTheta << ", rightTheta: " << rightTheta << ", globalPathLocalWaypointTheta: " << globalPathLocalWaypointTheta);
        ROS_INFO_STREAM_NAMED("GapManipulator", "            leftToRightAngle: " << leftToRightAngle << ", leftToGoalAngle: " << leftToGoalAngle << ", rightToGoalAngle: " << rightToGoalAngle);

        return biasedGapGoalTheta;
    }

    bool GapManipulator::checkWaypointVisibility(const Eigen::Vector2f & leftPt, const Eigen::Vector2f & rightPt,
                                                const Eigen::Vector2f & globalPathLocalWaypoint) 
    {
        boost::mutex::scoped_lock lock(egolock);
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
        // float goal_angle = std::atan2(globalPathLocalWaypoint.pose.position.y, globalPathLocalWaypoint.pose.position.x);

        // get gap's range at globalPathLocalWaypoint idx
        // Eigen::Vector2f left_norm_vect(std::cos(leftTheta), std::sin(leftTheta));
        // Eigen::Vector2f right_norm_vect(std::cos(rightTheta), std::sin(rightTheta));
        // Eigen::Vector2f goal_norm_vect(std::cos(goal_angle), std::sin(goal_angle));

        float leftToRightAngle = getLeftToRightAngle(leftPt, rightPt, true);
        float leftToGoalAngle = getLeftToRightAngle(leftPt, globalPathLocalWaypoint, true);
        float gapGoalRange = (rightPt.norm() - leftPt.norm()) * epsilonDivide(leftToGoalAngle, leftToRightAngle) + leftPt.norm();

        /*
        float half_angle = std::asin(cfg_->rbt.r_inscr / dist2goal);
        // int index = std::ceil(half_angle / scan.angle_increment) * 1.5;
        int index = (int)(scan.ranges.size()) / 8;
        int lower_bound = goal_index - index;
        int upper_bound = goal_index + index;
        float minScanRange_round_goal = *std::min_element(scan.ranges.begin() + lower_bound, scan.ranges.begin() + upper_bound);
        */

        return dist2goal < gapGoalRange;
    }

    // In place modification
    void GapManipulator::reduceGap(dynamic_gap::Gap& gap, const geometry_msgs::PoseStamped & globalPathLocalWaypoint, bool initial) 
    {        
        try
        {
            // msg is from egocircle
            // only part of msg used is angle_increment

            int leftIdx, rightIdx;
            float leftDist, rightDist;
            if (initial) 
            {
                leftIdx = gap.cvxLeftIdx();
                rightIdx = gap.cvxRightIdx();
                leftDist = gap.cvxLeftDist();
                rightDist = gap.cvxRightDist();
            } else {
                leftIdx = gap.cvxTermLeftIdx();
                rightIdx = gap.cvxTermRightIdx();
                leftDist = gap.cvxTermLeftDist();
                rightDist = gap.cvxTermRightDist();
            }

            // int leftIdx = initial ? gap.cvxLeftIdx() : gap.cvxTermLeftIdx();
            // int rightIdx = initial ? gap.cvxRightIdx() : gap.cvxTermRightIdx();
            // float leftDist = initial ? gap.cvxLeftDist() : gap.cvxTermLeftDist();
            // float rightDist = initial ? gap.cvxRightDist() : gap.cvxTermRightDist();

            float gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0.0)
                gapIdxSpan += cfg_->scan.full_scan_f; // (2*gap.half_scan);

            float gapAngle = gapIdxSpan * cfg_->scan.angle_increment;
            // ROS_INFO_STREAM_NAMED("GapManipulator",  "gap idx size: " << gapIdxSpan << std::endl;

            // threshold = pi right now
            if (gapAngle < cfg_->gap_manip.reduction_threshold)
                return;

            // float xLeft = (leftDist) * cos(((float) leftIdx - half_num_scan) * M_PI / half_num_scan);
            // float yLeft = (leftDist) * sin(((float) leftIdx - half_num_scan) * M_PI / half_num_scan);
            // float xRight = (rightDist) * cos(((float) rightIdx - half_num_scan) * M_PI / half_num_scan);
            // float yRight = (rightDist) * sin(((float) rightIdx - half_num_scan) * M_PI / half_num_scan);

            ROS_INFO_STREAM_NAMED("GapManipulator", "    [reduceGap()]");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-reduce gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");
            // ROS_INFO_STREAM_NAMED("GapManipulator", "pre-reduce gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

            // the desired size for the reduced gap?
            // target is pi
            int targetGapIdxSpan = cfg_->gap_manip.reduction_target / cfg_->scan.angle_increment;
            // int leftIdxBiasedRight = ();
            // int rightIdxBiasedLeft = ; 

            int leftIdxBiasedRight = subtractAndWrapScanIndices(leftIdx - targetGapIdxSpan, cfg_->scan.full_scan);
            int rightIdxBiasedLeft = (rightIdx + targetGapIdxSpan) % cfg_->scan.full_scan; // num_of_scan is int version of 2*half_scan

            // ROS_INFO_STREAM_NAMED("GapManipulator", "rightIdxBiasedLeft: " << rightIdxBiasedLeft << ", leftIdxBiasedRight: " << leftIdxBiasedRight);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "r_biased_l: " << r_biased_l << ", l_biased_r: " << l_biased_r);

            float globalPathLocalWaypointTheta = std::atan2(globalPathLocalWaypoint.pose.position.y, globalPathLocalWaypoint.pose.position.x);
            int globalPathLocalWaypointIdx = theta2idx(globalPathLocalWaypointTheta); // globalPathLocalWaypointTheta / (M_PI / gap.half_scan) + gap.half_scan;
            ROS_INFO_STREAM_NAMED("GapManipulator", "        globalPathLocalWaypointIdx: " << globalPathLocalWaypointIdx);
            int halfTargetGapIdxSpan = targetGapIdxSpan / 2; // distance in scan indices

            //ROS_INFO_STREAM_NAMED("GapManipulator",  "goal orientation: " << globalPathLocalWaypointTheta << ", goal idx: " << goal_idx << ", acceptable distance: " << halfTargetGapIdxSpan << std::endl;

            leftIdxBiasedRight = subtractAndWrapScanIndices(leftIdx - halfTargetGapIdxSpan, cfg_->scan.full_scan);
            int leftIdxBiasedLeft = (leftIdx + halfTargetGapIdxSpan) % cfg_->scan.full_scan;

            int rightIdxBiasedRight = subtractAndWrapScanIndices(rightIdx - halfTargetGapIdxSpan, cfg_->scan.full_scan);
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
                newRightIdx = subtractAndWrapScanIndices(globalPathLocalWaypointIdx - halfTargetGapIdxSpan, cfg_->scan.full_scan);
                ROS_INFO_STREAM_NAMED("GapManipulator", "        creating goal-centered gap: " << newLeftIdx << ", " << newRightIdx);
            }

            // removed some float casting here
            // float orig_gap_size = subtractAndWrapScanIndices(leftIdx - rightIdx, cfg_->scan.full_scan);
            float leftToNewLeftIdxSpan = subtractAndWrapScanIndices(leftIdx - newLeftIdx, cfg_->scan.full_scan);
            float leftToNewRightIdxSpan = subtractAndWrapScanIndices(leftIdx - newRightIdx, cfg_->scan.full_scan);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "orig_gap_size: " << orig_gap_size);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "leftToNewRightIdxSpan: " << leftToNewRightIdxSpan << ", leftToNewLeftIdxSpan: " << leftToNewLeftIdxSpan);

            float newLeftDist = leftDist + (rightDist - leftDist) * epsilonDivide(leftToNewLeftIdxSpan, gapIdxSpan);
            float newRightDist = leftDist +  (rightDist - leftDist) * epsilonDivide(leftToNewRightIdxSpan, gapIdxSpan);

            if (initial) 
            {
                gap.setCvxLeftIdx(newLeftIdx);
                gap.setCvxRightIdx(newRightIdx);
                gap.setCvxLeftDist(newLeftDist);            
                gap.setCvxRightDist(newRightDist);
                gap.mode.reduced_ = true;

                // xLeft = gap.cvxLeftDist() * cos(((float) gap.cvxLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // yLeft = gap.cvxLeftDist() * sin(((float) gap.cvxLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // xRight = gap.cvxRightDist() * cos(((float) gap.cvxRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // yRight = gap.cvxRightDist() * sin(((float) gap.cvxRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
            } else 
            {
                gap.setcvxTermLeftIdx(newLeftIdx);
                gap.setcvxTermRightIdx(newRightIdx);
                gap.setcvxTermLeftDist(newLeftDist);
                gap.setcvxTermRightDist(newRightDist);
                gap.mode.termReduced_ = true;

                // xLeft = gap.cvxTermLeftDist() * cos(((float) gap.cvxTermLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // yLeft = gap.cvxTermLeftDist() * sin(((float) gap.cvxTermLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // xRight = gap.cvxTermRightDist() * cos(((float) gap.cvxTermRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // yRight = gap.cvxTermRightDist() * sin(((float) gap.cvxTermRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
            }


            ROS_INFO_STREAM_NAMED("GapManipulator", "        post-reduce gap in polar. left: (" << newLeftIdx << ", " << newLeftDist << "), right: (" << newRightIdx << ", " << newRightDist << ")");
            // ROS_INFO_STREAM_NAMED("GapManipulator", "post-reduce in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
            return;
        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "[reduceGap() failed]");
        }
    }

    void GapManipulator::convertRadialGap(dynamic_gap::Gap& gap, bool initial) 
    {
        try 
        {
            // Return if not radial gap or disabled
            if (!gap.isRadial(initial) || !cfg_->gap_manip.radial_convert) 
            {
                return;
            }

            // if (stored_scan_msgs.ranges.size() != 512) {ROS_ERROR_STREAM_NAMED("GapManipulator", "Scan range incorrect gap manip");}

            sensor_msgs::LaserScan desScan;
            int leftIdx, rightIdx;
            float leftDist, rightDist;
            if (initial) 
            {
                desScan = *scan_.get();
                leftIdx = gap.cvxLeftIdx();
                rightIdx = gap.cvxRightIdx();
                leftDist = gap.cvxLeftDist();
                rightDist = gap.cvxRightDist();
            } else 
            {
                desScan = dynamicScan_;
                leftIdx = gap.cvxTermLeftIdx();
                rightIdx = gap.cvxTermRightIdx();
                leftDist = gap.cvxTermLeftDist();
                rightDist = gap.cvxTermRightDist();
            }

            ROS_INFO_STREAM_NAMED("GapManipulator", "    [convertRadialGap()]");            
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RGC gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");
            // ROS_INFO_STREAM_NAMED("GapManipulator", "pre-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

            // float leftTheta = idx2theta(leftIdx);
            // float rightTheta = idx2theta(rightIdx);

            bool right = gap.isRightType(initial);
            // Rotate radial gap by an amount theta so that its width is visible
            
            // start with a nominal pivot angle, we will adjust this angle to find the actual theta that we pivot by
            float nomPivotAngle = std::atan2(cfg_->gap_manip.epsilon2, cfg_->gap_manip.epsilon1) + 1e-3;
            // float theta = right ? (nomPivotAngle + 1e-3): -(nomPivotAngle + 1e-3);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "left: " << left << ", nomPivotAngle: " << nomPivotAngle);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "theta to pivot: " << theta);
            int nearIdx, farIdx;
            float nearDist, farDist;
            float nearTheta, farTheta;
            float signedNomPivotAngle;

            if (right) 
            {
                nearIdx = rightIdx;
                nearDist = rightDist;
                farIdx = leftIdx;
                farDist = leftDist;
                // nearTheta = rightTheta;
                // farTheta = leftTheta;
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
            // Eigen::Matrix3f farPtTranslationMatrix;
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
            int scanSearchStartIdx, scanSearchEndIdx;
            if (right)
            {   
                scanSearchStartIdx = leftIdx;
                scanSearchEndIdx = nomPivotedIdx;
            } else
            {
                scanSearchStartIdx = nomPivotedIdx;
                scanSearchEndIdx = rightIdx;
            }
            // offset: original right index or the pivoted left
            // int scanSearchStartIdx = right ? leftIdx : nomPivotedIdx;
            // upperbound: pivoted right index or original left  
            // int scanSearchEndIdx = right ? nomPivotedIdx : rightIdx;

            // For wraparound (check to see if this happens)
            // scanSearchStartIdx = std::max(scanSearchStartIdx, 0);
            // scanSearchEndIdx = std::min(scanSearchEndIdx, cfg_->scan.full_scan - 1);

            // int scanSearchSize = scanSearchEndIdx - scanSearchStartIdx;
            // if (scanSearchSize < 0)
            //     scanSearchSize += cfg_->scan.full_scan; // 2*int(gap.half_scan);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "scanSearchStartIdx: " << scanSearchStartIdx << ", scanSearchEndIdx: " << scanSearchEndIdx);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "wrapped scanSearchStartIdx: " << scanSearchStartIdx << ", wrapped scanSearchEndIdx: " << scanSearchEndIdx);
            int scanSearchSize = scanSearchEndIdx - scanSearchStartIdx;
            if (scanSearchSize < 0)
                scanSearchSize += cfg_->scan.full_scan; // int(2*gap.half_scan);


            int gapIdxSpan = (leftIdx - rightIdx);
            if (gapIdxSpan < 0)
                gapIdxSpan += cfg_->scan.full_scan; // int(2*gap.half_scan);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "gapIdxSpan: " << gapIdxSpan);

            // using the law of cosines to find the index between init/final indices
            // that's shortest distance between near point and laser scan
            // ROS_INFO_STREAM_NAMED("GapManipulator", "ranges size: " << stored_scan_msgs.ranges.size());
            std::vector<float> nearPtToScanDists(scanSearchSize);
            // try
            // {
            int checkIdx;
            float checkRange, checkIdxSpan;
            for (int i = 0; i < nearPtToScanDists.size(); i++) 
            {
                checkIdx = (i + scanSearchStartIdx) % cfg_->scan.full_scan; // int(2 * gap.half_scan);
                checkRange = desScan.ranges.at(checkIdx);
                checkIdxSpan = gapIdxSpan + (scanSearchSize - i);
                nearPtToScanDists.at(i) = sqrt(pow(nearDist, 2) + pow(checkRange, 2) -
                                            2.0 * nearDist * checkRange * cos(checkIdxSpan * cfg_->scan.angle_increment));
                // ROS_INFO_STREAM_NAMED("GapManipulator", "checking idx: " << checkIdx << ", range of: " << range << ", diff in idx: " << checkIdxSpan << ", dist of " << dist.at(i));
            }
            // } catch(...) {
            //     ROS_ERROR_STREAM_NAMED("GapManipulator", "convertRadialGap outofBound");
            // }

            auto minDistIter = std::min_element(nearPtToScanDists.begin(), nearPtToScanDists.end());
            int minDistIdx = (scanSearchStartIdx + std::distance(nearPtToScanDists.begin(), minDistIter)) % cfg_->scan.full_scan; // int(2*gap.half_scan);
            float minDistRange = *minDistIter;

            // ROS_INFO_STREAM_NAMED("GapManipulator", "from " << scanSearchStartIdx << " to " << scanSearchEndIdx << ", min dist of " << minDistRange << " at " << minDistIdx);         

            // pivoting around near point, pointing towards far point or something?

            float nearToFarDistance = sqrt(pow(nearToFarTranslationMatrix(0, 2), 2) + pow(nearToFarTranslationMatrix(1, 2), 2));
            // ROS_INFO_STREAM_NAMED("GapManipulator", "nearToFarDistance: " << nearToFarDistance);
            // float coefs = nearToFarTranslationMatrix.block<2, 1>(0, 2).norm();
            // ROS_INFO_STREAM_NAMED("GapManipulator", "translation norm: " << nearToFarDistance);
            
            // augmenting near to far direction by pivoted point, multiplying by min dist        
            nearToFarTranslationMatrix(0, 2) *= epsilonDivide(minDistRange, nearToFarDistance);     
            nearToFarTranslationMatrix(1, 2) *= epsilonDivide(minDistRange, nearToFarDistance);
            Eigen::Matrix3f pivotedPtMatrix = nearPtTranslationMatrix * (nomPivotAngleRotationMatrix * nearToFarTranslationMatrix);

            float pivotedPtDist = sqrt(pow(pivotedPtMatrix(0, 2), 2) + pow(pivotedPtMatrix(1, 2), 2));
            float pivotedPtTheta = std::atan2(pivotedPtMatrix(1, 2), pivotedPtMatrix(0, 2));
            // idx = int (half_num_scan * nomPivotedTheta / M_PI) + half_num_scan;
            int pivotedPtIdx = theta2idx(pivotedPtTheta); 

            float newLeftIdx, newRightIdx, newLeftDist, newRightDist;
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
                gap.setCvxLeftIdx(newLeftIdx);
                gap.setCvxRightIdx(newRightIdx);
                gap.setCvxLeftDist(newLeftDist);
                gap.setCvxRightDist(newRightDist);
                gap.mode.RGC_ = true;

                // xLeft = gap.cvxLeftDist() * cos(((float) gap.cvxLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // yLeft = gap.cvxLeftDist() * sin(((float) gap.cvxLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // xRight = gap.cvxRightDist() * cos(((float) gap.cvxRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // yRight = gap.cvxRightDist() * sin(((float) gap.cvxRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
            } else 
            {
                gap.setcvxTermLeftIdx(newLeftIdx);
                gap.setcvxTermRightIdx(newRightIdx);
                gap.setcvxTermLeftDist(newLeftDist);
                gap.setcvxTermRightDist(newRightDist);
                gap.mode.termRGC_ = true;

                // xLeft = gap.cvxTermLeftDist() * cos(((float) gap.cvxTermLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // yLeft = gap.cvxTermLeftDist() * sin(((float) gap.cvxTermLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // xRight = gap.cvxTermRightDist() * cos(((float) gap.cvxTermRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
                // yRight = gap.cvxTermRightDist() * sin(((float) gap.cvxTermRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
            }

            ROS_INFO_STREAM_NAMED("GapManipulator", "        post-RGC gap in polar. left: (" << newLeftIdx << ", " << newLeftDist << "), right: (" << newRightIdx << ", " << newRightDist << ")");
            // ROS_INFO_STREAM_NAMED("GapManipulator",  "post-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "convertRadialGap() failed");
        }
    }

    void GapManipulator::radialExtendGap(dynamic_gap::Gap& gap, bool initial) 
    {
        try
        {
            if (!cfg_->gap_manip.radial_extend)
                return;

            //  = initial ? *msg.get() : dynamic_scan;
            // int half_num_scan = gap.half_scan; // changing this
            
            // sensor_msgs::LaserScan stored_scan_msgs;
            int leftIdx, rightIdx;
            float leftDist, rightDist;
            if (initial) 
            {
                leftIdx = gap.cvxLeftIdx();
                rightIdx = gap.cvxRightIdx();
                leftDist = gap.cvxLeftDist();
                rightDist = gap.cvxRightDist();
            } else 
            {
                leftIdx = gap.cvxTermLeftIdx();
                rightIdx = gap.cvxTermRightIdx();
                leftDist = gap.cvxTermLeftDist();
                rightDist = gap.cvxTermRightDist();
            }

            // int leftIdx = initial ? gap.cvxLeftIdx() : gap.cvxTermLeftIdx();
            // int rightIdx = initial ? gap.cvxRightIdx() : gap.cvxTermRightIdx();
            // float leftDist = initial ? gap.cvxLeftDist() : gap.cvxTermLeftDist();
            // float rightDist = initial ? gap.cvxRightDist() : gap.cvxTermRightDist();

            float leftTheta = idx2theta(leftIdx);
            float rightTheta = idx2theta(rightIdx);
            float xLeft = leftDist * cos(leftTheta);
            float yLeft = leftDist * sin(leftTheta);
            float xRight = rightDist * cos(rightTheta);
            float yRight = rightDist * sin(rightTheta);
            
            Eigen::Vector2f leftPt(xLeft, yLeft);
            Eigen::Vector2f rightPt(xRight, yRight);

            // Eigen::Vector2f eL_robot = leftPt / leftPt.norm();
            // Eigen::Vector2f eR_robot = rightPt / rightPt.norm();

            ROS_INFO_STREAM_NAMED("GapManipulator", "    [radialExtendGap()]");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RE gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        pre-RE gap in cart. left: (" << leftPt[0] << ", " << leftPt[1] << "), right: (" << rightPt[0] << ", " << rightPt[1] << ")");
            
            // ROS_INFO_STREAM_NAMED("GapManipulator", "leftPt: (" << xLeft << ", " << yLeft << "), rightPt: (" << xRight << ", " << yRight << ")");
            // ROS_INFO_STREAM_NAMED("GapManipulator", "eL_robot: (" << eL_robot[0] << ", " << eL_robot[1] << ") , eR_robot: (" << eR_robot[0] << ", " << eR_robot[1] << ")");

            float leftToRightAngle = getLeftToRightAngle(leftPt, rightPt, true);

            // float thetaLeft_robot = std::atan2(leftPt[1], leftPt[0]);

            float thetaCenter = (leftTheta - 0.5*leftToRightAngle);

            // middle of gap direction
            Eigen::Vector2f eB(std::cos(thetaCenter), std::sin(thetaCenter));
            // ROS_INFO_STREAM_NAMED("GapManipulator", "eB: (" << eB[0] << ", " << eB[1] << ")");

            Eigen::Vector2f norm_eB = unitNorm(eB); 
            // angular size of gap
            // ROS_INFO_STREAM_NAMED("GapManipulator", "normalized eB: " << norm_eB[0] << ", " << norm_eB[1]);

            // minSafeDist is the minimum distance within the laser scan 
            float s = initial ? gap.getMinSafeDist() : gap.getTerminalMinSafeDist();
            // ROS_INFO_STREAM_NAMED("GapManipulator", "min safe dist: " << s);
            
            // point opposite direction of middle of gap, magnitude of min safe dist
            Eigen::Vector2f extendedGapOrigin =  - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * norm_eB; //
            // ROS_INFO_STREAM_NAMED("GapManipulator", "extendedGapOrigin: " << extendedGapOrigin[0] << ", " << extendedGapOrigin[1]);

            // Eigen::Vector2f fwd_extendedGapOrigin = -extendedGapOrigin; 
            Eigen::Matrix2f Rpi2, Rnegpi2;
            Rpi2 << 0, -1, 1, 0;
            Rnegpi2 = -Rpi2;

            
            if (initial) 
            {
                gap.extendedGapOrigin_ = extendedGapOrigin;
                // Eigen::Vector2f qLp = leftPt - extendedGapOrigin;
                // float theta_btw_qLp_and_extendedGapOrigin = std::acos(fwd_extendedGapOrigin.dot(qLp) / (fwd_extendedGapOrigin.norm() * qLp.norm()));
                // float length_along_qLp = fwd_extendedGapOrigin.norm() / cos(theta_btw_qLp_and_extendedGapOrigin);
                // ROS_INFO_STREAM_NAMED("GapManipulator", "theta between qLp and extendedGapOrigin: " << theta_btw_qLp_and_extendedGapOrigin);
                //Eigen::Vector2f left_hypotenuse = length_along_qLp * qLp / qLp.norm();        
                // ROS_INFO_STREAM_NAMED("GapManipulator", "length along qLp: " << length_along_qLp << ", left_hypotenuse: " << left_hypotenuse[0] << ", " << left_hypotenuse[1]);
                
                gap.leftBezierOrigin_ =  Rnegpi2 * extendedGapOrigin; // left_hypotenuse + extendedGapOrigin;

                // Eigen::Vector2f qRp = rightPt - extendedGapOrigin;
                // float theta_btw_qRp_and_extendedGapOrigin = std::acos(fwd_extendedGapOrigin.dot(qRp) / (fwd_extendedGapOrigin.norm() * qRp.norm()));
                // float length_along_qRp = fwd_extendedGapOrigin.norm() / cos(theta_btw_qRp_and_extendedGapOrigin);
                // ROS_INFO_STREAM_NAMED("GapManipulator", "theta between qRp and extendedGapOrigin: " << theta_btw_qRp_and_extendedGapOrigin);
                // Eigen::Vector2f right_hypotenuse = length_along_qRp * qRp / qRp.norm();        
                // ROS_INFO_STREAM_NAMED("GapManipulator", "length along qRp: " << length_along_qRp << ", right_hypotenuse: " << right_hypotenuse[0] << ", " << right_hypotenuse[1]);
                gap.rightBezierOrigin_ = Rpi2 * extendedGapOrigin; // right_hypotenuse + extendedGapOrigin;
                gap.mode.convex_ = true;
            
            } else 
            {
                gap.termExtendedGapOrigin_ = extendedGapOrigin;
                gap.mode.termConvex_ = true;
            }
            
            ROS_INFO_STREAM_NAMED("GapManipulator", "        finishing with gap extendedGapOrigin: " << extendedGapOrigin[0] << ", " << extendedGapOrigin[1]);

            return;
        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "radialGapExtension() failed");
        }
    }

    void GapManipulator::inflateGapSides(dynamic_gap::Gap& gap, bool initial) 
    {
        try 
        {
            // get points

            int leftIdx, rightIdx;
            float leftDist, rightDist;
            // sensor_msgs::LaserScan stored_scan_msgs =  // initial ? *msg.get() : dynamic_scan;
            sensor_msgs::LaserScan desScan;
            if (initial) 
            {
                desScan = *scan_.get();
                leftIdx = gap.cvxLeftIdx();
                rightIdx = gap.cvxRightIdx();
                leftDist = gap.cvxLeftDist();
                rightDist = gap.cvxRightDist();
            } else 
            {
                desScan = dynamicScan_;
                leftIdx = gap.cvxTermLeftIdx();
                rightIdx = gap.cvxTermRightIdx();
                leftDist = gap.cvxTermLeftDist();
                rightDist = gap.cvxTermRightDist();
            }
            // int leftIdx = initial ? gap.cvxLeftIdx() : gap.cvxTermLeftIdx();
            // int rightIdx = initial ? gap.cvxRightIdx() : gap.cvxTermRightIdx();
            // float leftDist = initial ? gap.cvxLeftDist() : gap.cvxTermLeftDist();
            // float rightDist = initial ? gap.cvxRightDist() : gap.cvxTermRightDist();

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
            float leftToRightAngle = getLeftToRightAngle(leftUnitNorm, rightUnitNorm, true);

            // inflate inwards by radius * infl
            // rotate by pi/2, norm
            // Eigen::Matrix2f Rpi2, Rnegpi2;
            // Rpi2 << 0, -1,
            //             1, 0;
            // Rnegpi2 = -Rpi2;
            
            // PERFORMING ANGULAR INFLATION
            Eigen::Vector2f leftInflationVector = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * Rnegpi2 * leftUnitNorm;
            Eigen::Vector2f inflatedLeftPt = leftPt + leftInflationVector;
            float inflatedLeftTheta = std::atan2(inflatedLeftPt[1], inflatedLeftPt[0]);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "leftTheta: " << leftTheta << ", inflatedLeftTheta: " << inflatedLeftTheta); // << ", leftThetaeft_infl: " << leftThetaeft_infl

            // float rightTheta_infl = ((M_PI / 2) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) / rightPt.norm(); // using s = r*theta
            // float inflatedRightTheta = rightTheta + rightTheta_infl;
            // inflatedRightTheta = atanThetaWrap(inflatedRightTheta);
            
            Eigen::Vector2f rightInflationVector = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * Rpi2 * rightUnitNorm;
            Eigen::Vector2f inflatedRightPt = rightPt + rightInflationVector;
            float inflatedRightTheta = std::atan2(inflatedRightPt[1], inflatedRightPt[0]);

            // ROS_INFO_STREAM_NAMED("GapManipulator", "rightTheta: " << rightTheta << ", inflatedRightTheta: " << inflatedRightTheta); // ", rightTheta_infl: " << rightTheta_infl << 

            Eigen::Vector2f inflatedLeftUnitNorm(std::cos(inflatedLeftTheta), std::sin(inflatedLeftTheta));
            Eigen::Vector2f inflatedRightUnitNorm(std::cos(inflatedRightTheta), std::sin(inflatedRightTheta));
            float newLeftToRightAngle = getLeftToRightAngle(inflatedLeftUnitNorm, inflatedRightUnitNorm, false);
            // ROS_INFO_STREAM_NAMED("GapManipulator", "newLeftToRightAngle: " << newLeftToRightAngle);

            int inflatedLeftIdx, inflatedRightIdx;
            float inflatedLeftRange, inflatedRightRange;
            if (newLeftToRightAngle < 0) 
            {
                // ROS_INFO_STREAM_NAMED("GapManipulator", "angular inflation would cause gap to cross, not running:");
                inflatedRightIdx = rightIdx;
                inflatedLeftIdx = leftIdx;
                inflatedLeftRange = leftDist;
                inflatedRightRange = rightDist;
            } else {
                // need to make sure L/R don't cross each other
                inflatedRightIdx = theta2idx(inflatedRightTheta);
                inflatedLeftIdx = theta2idx(inflatedLeftTheta);
                
                // float leftDist_robot = leftDist;
                // float rightDist_robot = rightDist;

                float leftToInflatedLeftAngle = getLeftToRightAngle(leftUnitNorm, inflatedLeftUnitNorm, false);
                float leftToInflatedRightAngle = getLeftToRightAngle(leftUnitNorm, inflatedRightUnitNorm, false);
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

            // float new_l_range = inflatedLeftRange; // std::min(inflatedLeftRange, stored_scan_msgs.ranges.at(inflatedLeftIdx));
            // float new_r_range = inflatedRightRange; // std::min(inflatedRightRange, stored_scan_msgs.ranges.at(inflatedRightIdx)); // should this be ranges.at - r_infl? 

            if (initial) 
            {
                gap.setCvxLeftIdx(inflatedLeftIdx);
                gap.setCvxRightIdx(inflatedRightIdx);      
                gap.setCvxLeftDist(inflatedLeftRange);
                gap.setCvxRightDist(inflatedRightRange);
                gap.getSimplifiedLCartesian(xLeft, yLeft);
                gap.getSimplifiedRCartesian(xRight, yRight);
            } else 
            {
                gap.setcvxTermLeftIdx(inflatedLeftIdx);
                gap.setcvxTermRightIdx(inflatedRightIdx);
                gap.setcvxTermLeftDist(inflatedLeftRange);
                gap.setcvxTermRightDist(inflatedRightRange);
                gap.getSimplifiedTerminalLCartesian(xLeft, yLeft);
                gap.getSimplifiedTerminalRCartesian(xRight, yRight);
            }

            ROS_INFO_STREAM_NAMED("GapManipulator", "        post-inflate gap in polar. left: (" << inflatedLeftIdx << ", " << inflatedLeftRange << "), right: (" << inflatedRightIdx << ", " << inflatedRightRange << ")");
            ROS_INFO_STREAM_NAMED("GapManipulator", "        post-inflate gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");

        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapManipulator", "[inflateGapSides() failed]");
        }
    }
}
