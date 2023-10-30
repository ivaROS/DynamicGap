#include <dynamic_gap/trajectory_generation/GapManipulator.h>

namespace dynamic_gap 
{
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) 
    {
        boost::mutex::scoped_lock lock(egolock);
        scan_ = msg_;
        // num_of_scan = (int)(scan_.get()->ranges.size());
        // half_num_scan = num_of_scan / 2;
        // angle_min = scan_.get()->angle_min;
        // angle_increment = scan_.get()->angle_increment;
    }

    void GapManipulator::updateStaticEgoCircle(const sensor_msgs::LaserScan & staticScan) 
    {
        boost::mutex::scoped_lock lock(egolock);
        staticScan_ = staticScan;
        // num_of_scan = (int) (static_scan.ranges.size());
        // half_num_scan = num_of_scan / 2;

        // angle_min = static_scan.angle_min;
        // angle_increment = static_scan.angle_increment;
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
        if (cfg_->debug.manipulation_debug_log) 
            ROS_INFO_STREAM("    [setTerminalGapWaypoint()]");
        
        if (gap.getCategory() == "expanding" || gap.getCategory() == "static") 
        { 
            if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("setting terminal goal for expanding gap");
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
                ROS_INFO_STREAM("        setting terminal goal for closed closing gap");
                Eigen::Vector2f closing_pt = gap.getClosingPoint();
            
                gap.terminalGoal.x_ = closing_pt[0];
                gap.terminalGoal.y_ = closing_pt[1];
                // gap.terminalGoal.set = true;
            } else 
            {
                closingGapType = "existent";
                setGapWaypoint(gap, globalPathLocalWaypoint, false);
            }

            if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("        setting terminal goal for " + closingGapType + " closing gap");
        }
    }

    void GapManipulator::setGapWaypoint(dynamic_gap::Gap& gap, const geometry_msgs::PoseStamped & globalPathLocalWaypoint, bool initial) 
    {
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

        if (cfg_->debug.manipulation_debug_log) 
        { 
            ROS_INFO_STREAM("    [setGapWaypoint()]");
            ROS_INFO_STREAM("        gap polar points, left: (" << leftIdx << ", " << leftDist << ") , right: (" << rightIdx << ", " << rightDist << ")");
            ROS_INFO_STREAM("        gap cart points, left: (" << xLeft << ", " << yLeft << ") , right: (" << xRight << ", " << yRight << ")");
        }

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
        //         ROS_INFO_STREAM("        Option 0: artificial gap");
        //         ROS_INFO_STREAM("            goal: " << globalPathLocalWaypointVector[0] << ", " << globalPathLocalWaypointVector[1]);
        //         // Eigen::Vector2f goalPt(globalPathLocalWaypoint.pose.position.x, globalPathLocalWaypoint.pose.position.y);
        //         // Eigen::Vector2f goal_norm_vector = goalPt / goalPt.norm();
        //         // float leftToGoalAngle = getLeftToRightAngle(leftPt, goalPt, true);
        //         // if (! (0.0 < leftToGoalAngle && leftToGoalAngle < leftToRightAngle)) 
        //         // {
        //         //     ROS_INFO_STREAM("            goal outside of gap");
        //         //     ROS_INFO_STREAM("            leftToGoalAngle: " << leftToGoalAngle << ", leftToRightAngle: " << leftToRightAngle);
        //         //     float goal_theta = std::atan2(globalPathLocalWaypoint.pose.position.y, globalPathLocalWaypoint.pose.position.x);
        //         //     ROS_INFO_STREAM("            left_theta: " << leftTheta << ", goal_theta: " << goal_theta << ", right_theta: " << rightTheta);
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
            // Eigen::Vector2f left_norm_vect_robot = leftPt / leftPt.norm();
            // Eigen::Vector2f right_norm_vect_robot = rightPt / rightPt.norm();
            
            float leftToRightAngle = getLeftToRightAngle(leftPt, rightPt, true);
            
            float thetaLeft = std::atan2(leftPt[1], leftPt[0]);

            // ROS_INFO_STREAM("leftToRightAngle: " << leftToRightAngle);
            float thetaCenter = (thetaLeft - 0.5 * leftToRightAngle); 
            float rangeCenter = (leftPt.norm() + rightPt.norm()) / 2.0;
            float goalX = rangeCenter * std::cos(thetaCenter);
            float goalY = rangeCenter * std::sin(thetaCenter);
            // ROS_INFO_STREAM("thetaLeft: " << thetaLeft << ", thetaRight: " << thetaRight << ", thetaCenter: " << thetaCenter);

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

            if (cfg_->debug.manipulation_debug_log) 
            {
                ROS_INFO_STREAM("        Option 1: small gap");
                ROS_INFO_STREAM("            goal: " << goalX << ", " << goalY);
                // Eigen::Vector2f goalPt(goalX, goalY);
                // // Eigen::Vector2f goal_norm_vector = goalPt / goalPt.norm();
                // float leftToGoalAngle = getLeftToRightAngle(leftPt, goalPt, true);
                // if (! (0.0 < leftToGoalAngle && leftToGoalAngle < leftToRightAngle)) 
                // {
                //     ROS_INFO_STREAM("            goal outside of gap");
                //     ROS_INFO_STREAM("            leftToGoalAngle: " << leftToGoalAngle << ", leftToRightAngle: " << leftToRightAngle);
                //     float goal_theta = std::atan2(goalY, goalX);
                //     ROS_INFO_STREAM("            left_theta: " << leftTheta << ", goal_theta: " << goal_theta << ", right_theta: " << rightTheta);
                // }                
            }
            return;
        }

        // sensor_msgs::LaserScan stored_scan_msgs = *scan_.get(); // initial ? *msg.get() : dynamic_laser_scan;
        float globalPathLocalWaypointTheta = std::atan2(globalPathLocalWaypointVector[1], globalPathLocalWaypointVector[0]);
        float globalPathLocalWaypointIdx = theta2idx(globalPathLocalWaypointTheta); // std::floor(goal_orientation*half_num_scan/M_PI + half_num_scan);
        
        if (cfg_->debug.manipulation_debug_log)
            ROS_INFO_STREAM("        local goal idx: " << globalPathLocalWaypointIdx << 
                            ", local goal x/y: (" << globalPathLocalWaypointVector[0] << 
                                             ", " << globalPathLocalWaypointVector[1] << ")");

        // bool goal_within_gap_angle = ; // is localgoal within gap angle
        // ROS_INFO_STREAM("goal_vis: " << goal_vis << ", " << goal_in_range);
        
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

            if (cfg_->debug.manipulation_debug_log) 
            {
                std::string goalStatus = gap.artificial_ ? "Option 0: artificial gap: " : "Option 2: global path local waypoint: ";
                ROS_INFO_STREAM("        " << goalStatus);
                ROS_INFO_STREAM("            goal: " << globalPathLocalWaypointVector[0] << ", " << globalPathLocalWaypointVector[1]);
                // Eigen::Vector2f goalPt(globalPathLocalWaypoint.pose.position.x, globalPathLocalWaypoint.pose.position.y);
                // Eigen::Vector2f goal_norm_vector = goalPt / goalPt.norm();
                // float leftToGoalAngle = getLeftToRightAngle(leftPt, goalPt, true);
                // if (! (0.0 < leftToGoalAngle && leftToGoalAngle < leftToRightAngle)) 
                // {
                //     ROS_INFO_STREAM("            goal outside of gap");
                //     ROS_INFO_STREAM("            leftToGoalAngle: " << leftToGoalAngle << ", leftToRightAngle: " << leftToRightAngle);  
                //     float goal_theta = std::atan2(globalPathLocalWaypoint.pose.position.y, globalPathLocalWaypoint.pose.position.x);
                //     ROS_INFO_STREAM("            left_theta: " << leftTheta << ", goal_theta: " << goal_theta << ", right_theta: " << rightTheta);
                // }                      
            }
            return;
        }

        // if agc. then the shorter side need to be further in
        // lf: front (first one, left from laser scan)
        // lr: rear (second one, right from laser scan)
        // what do these do?
        if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("        Option 3: biasing gap goal towards global path local waypoint");

        // Eigen::Vector2f local_goal_norm_vect(std::cos(globalPathLocalWaypointTheta), std::sin(globalPathLocalWaypointTheta));
        float leftToGoalAngle = getLeftToRightAngle(leftPt, globalPathLocalWaypointVector, true);
        float rightToGoalAngle = getLeftToRightAngle(rightPt, globalPathLocalWaypointVector, true);
 
        float biasedGapGoalTheta = setBiasedGapGoalTheta(leftTheta, rightTheta, globalPathLocalWaypointTheta,
                                                         leftToRightAngle, rightToGoalAngle, leftToGoalAngle);

        float biasedGapGoalIdx = theta2idx(biasedGapGoalTheta); // std::floor(biasedGapGoalTheta*half_num_scan/M_PI + half_num_scan);

        Eigen::Vector2f biasedGapGoalUnitNorm(std::cos(biasedGapGoalTheta), std::sin(biasedGapGoalTheta));

        // needs to be distance between L/confined. Always positive.
        float leftToGapGoalAngle = getLeftToRightAngle(leftPt, biasedGapGoalUnitNorm, false); 

        if (cfg_->debug.manipulation_debug_log)
        {
            ROS_INFO_STREAM("            biasedGapGoalTheta: " << biasedGapGoalTheta);
            ROS_INFO_STREAM("            biasedGapGoalIdx: " << biasedGapGoalIdx);
            ROS_INFO_STREAM("            leftToGapGoalAngle: " << leftToGapGoalAngle << ", leftToRightAngle: " << leftToRightAngle);
        }

        // float leftDist_robot = leftDist;
        // float rightDist_robot = rightDist;

        float biasedGapGoalDist = rightDist + (rightDist - leftDist) * leftToGapGoalAngle / leftToRightAngle;
        Eigen::Vector2f biasedGapGoal(biasedGapGoalDist * cos(biasedGapGoalTheta), biasedGapGoalDist * sin(biasedGapGoalTheta));

        Eigen::Vector2f gapGoalAngularOffset(0.0, 0.0); 
        if ( (leftToGapGoalAngle / leftToRightAngle) < 0.1) // biased gap goal on left side
            gapGoalAngularOffset = Rnegpi2 * (leftPt / leftPt.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;    
        else                                                // biased gap goal on right side
            gapGoalAngularOffset = Rpi2 * (rightPt / rightPt.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

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

        Eigen::Vector2f gapGoalRadialOffset = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * biasedGapGoal / biasedGapGoal.norm();
        Eigen::Vector2f gapGoalOffset = gapGoalRadialOffset + gapGoalAngularOffset;
        
        if (cfg_->debug.manipulation_debug_log)
        {
            ROS_INFO_STREAM("            biasedGapGoal: " << biasedGapGoal[0] << ", " << biasedGapGoal[1]);
            ROS_INFO_STREAM("            gapGoalRadialOffset: " << gapGoalRadialOffset[0] << ", " << gapGoalRadialOffset[1]);
            ROS_INFO_STREAM("            gapGoalAngularOffset: " << gapGoalAngularOffset[0] << ", " << gapGoalAngularOffset[1]);
        }
        
        Eigen::Vector2f offsetBiasedGapGoal = gapGoalOffset + biasedGapGoal;
        // ROS_INFO_STREAM("anchor: (" << anchor[0] << ", " << anchor[1] << "), offset with r_ins " << cfg_->rbt.r_inscr << " and inf ratio " << cfg_->traj.inf_ratio << ", :(" << offset[0] << ", " << offset[1] << "), offsetBiasedGapGoal: (" << offsetBiasedGapGoal[0] << ", " << offsetBiasedGapGoal[1] << ")");
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

        if (cfg_->debug.manipulation_debug_log) 
        {
            ROS_INFO_STREAM("            goal: " << offsetBiasedGapGoal(0) << ", " << offsetBiasedGapGoal(1)); 
            // Eigen::Vector2f goalPt = offsetBiasedGapGoal;
            // // Eigen::Vector2f goal_norm_vector = goalPt / goalPt.norm();
            // float leftToGoalAngle = getLeftToRightAngle(leftPt, goalPt, true);
            // if (! (0.0 < leftToGoalAngle && leftToGoalAngle < leftToRightAngle)) 
            // {
            //     ROS_INFO_STREAM("            goal outside of gap");
            //     ROS_INFO_STREAM("            leftToGoalAngle: " << leftToGoalAngle << ", leftToRightAngle: " << leftToRightAngle);
            //     float goal_theta = std::atan2(goalPt[1], goalPt[0]);
            //     ROS_INFO_STREAM("            left_theta: " << leftTheta << ", goal_theta: " << goal_theta << ", right_theta: " << rightTheta);         
            // }                     
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

        if (cfg_->debug.manipulation_debug_log)
        {
            ROS_INFO_STREAM("            leftTheta: " << leftTheta << ", rightTheta: " << rightTheta << ", globalPathLocalWaypointTheta: " << globalPathLocalWaypointTheta);
            ROS_INFO_STREAM("            leftToRightAngle: " << leftToRightAngle << ", leftToGoalAngle: " << leftToGoalAngle << ", rightToGoalAngle: " << rightToGoalAngle);
        }

        return biasedGapGoalTheta;
    }

    bool GapManipulator::checkWaypointVisibility(const Eigen::Vector2f & leftPt, const Eigen::Vector2f & rightPt,
                                                const Eigen::Vector2f & globalPathLocalWaypoint) 
    {
        boost::mutex::scoped_lock lock(egolock);
        // with robot as 0,0 (localgoal in robot frame as well)
        float dist2goal = globalPathLocalWaypoint.norm(); // sqrt(pow(localGoal.pose.position.x, 2) + pow(localGoal.pose.position.y, 2));

        sensor_msgs::LaserScan scan = *scan_.get();
        auto minScanRange = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        if (dist2goal < 2 * cfg_->rbt.r_inscr)
            return true;

        // If within closest configuration space
        if (dist2goal < minScanRange - cfg_->traj.inf_ratio * cfg_->rbt.r_inscr)
            return true;

        // Should be sufficiently far, otherwise we are in trouble
        // float goal_angle = std::atan2(localGoal.pose.position.y, localGoal.pose.position.x);
        // int goal_index = (int) round((goal_angle - cfg_->scan.angle_min) / cfg_->scan.angle_increment);

        // get gap's range at localGoal idx
        // Eigen::Vector2f left_norm_vect(std::cos(leftTheta), std::sin(leftTheta));
        // Eigen::Vector2f right_norm_vect(std::cos(rightTheta), std::sin(rightTheta));
        // Eigen::Vector2f goal_norm_vect(std::cos(goal_angle), std::sin(goal_angle));

        float leftToRightAngle = getLeftToRightAngle(leftPt, rightPt, true);
        float leftToGoalAngle = getLeftToRightAngle(leftPt, globalPathLocalWaypoint, true);
        float gapGoalRange = (rightPt.norm() - leftPt.norm()) * leftToGoalAngle / leftToRightAngle + leftPt.norm();

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
    void GapManipulator::reduceGap(dynamic_gap::Gap& gap, const geometry_msgs::PoseStamped & localGoal, bool initial) 
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

        float gap_idx_size = (leftIdx - rightIdx);
        if (gap_idx_size < 0.0)
            gap_idx_size += cfg_->scan.full_scan_f; // (2*gap.half_scan);

        float gap_theta_size = gap_idx_size * cfg_->scan.angle_increment;
        // ROS_INFO_STREAM( "gap idx size: " << gap_idx_size << std::endl;

        // threshold = pi right now
        if (gap_theta_size < cfg_->gap_manip.reduction_threshold)
            return;

        // float xLeft = (leftDist) * cos(((float) leftIdx - half_num_scan) * M_PI / half_num_scan);
        // float yLeft = (leftDist) * sin(((float) leftIdx - half_num_scan) * M_PI / half_num_scan);
        // float xRight = (rightDist) * cos(((float) rightIdx - half_num_scan) * M_PI / half_num_scan);
        // float yRight = (rightDist) * sin(((float) rightIdx - half_num_scan) * M_PI / half_num_scan);

        if (cfg_->debug.manipulation_debug_log) 
        {
            ROS_INFO_STREAM("    [reduceGap()]");
            ROS_INFO_STREAM("        pre-reduce gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");
            // ROS_INFO_STREAM("pre-reduce gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
        }

        // the desired size for the reduced gap?
        // target is pi
        int target_idx_size = cfg_->gap_manip.reduction_target / cfg_->scan.angle_increment;
        int orig_r_biased_l = (rightIdx + target_idx_size); 
        int orig_l_biased_r = (leftIdx - target_idx_size);

        int r_biased_l = orig_r_biased_l % cfg_->scan.full_scan; // num_of_scan is int version of 2*half_scan
        int l_biased_r = subtract_wrap(orig_l_biased_r, cfg_->scan.full_scan);

        // ROS_INFO_STREAM("orig_r_biased_l: " << orig_r_biased_l << ", orig_l_biased_r: " << orig_l_biased_r);
        // ROS_INFO_STREAM("r_biased_l: " << r_biased_l << ", l_biased_r: " << l_biased_r);

        float localGoalTheta = std::atan2(localGoal.pose.position.y, localGoal.pose.position.x);
        int goal_idx = theta2idx(localGoalTheta); // localGoalTheta / (M_PI / gap.half_scan) + gap.half_scan;
        if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("        goal_idx: " << goal_idx);
        int acceptable_dist = target_idx_size / 2; // distance in scan indices

        //ROS_INFO_STREAM( "goal orientation: " << localGoalTheta << ", goal idx: " << goal_idx << ", acceptable distance: " << acceptable_dist << std::endl;
        int new_l_idx, new_r_idx;

        int L_minus = subtract_wrap(leftIdx - acceptable_dist, cfg_->scan.full_scan);
        int L_plus = (leftIdx + acceptable_dist) % cfg_->scan.full_scan;

        int R_minus = subtract_wrap(rightIdx - acceptable_dist, cfg_->scan.full_scan);
        int R_plus = (rightIdx + acceptable_dist) % cfg_->scan.full_scan;

        bool left_biased = isGlobalPathLocalWaypointWithinGapAngle(goal_idx, L_minus, L_plus); 
        bool right_biased = isGlobalPathLocalWaypointWithinGapAngle(goal_idx, R_minus, R_plus); 
        if (left_biased) {
            new_l_idx = leftIdx;
            new_r_idx = l_biased_r;
            if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("        creating left-biased gap: " << new_r_idx << ", " << new_l_idx);
        } else if (right_biased) {
            new_l_idx = r_biased_l;
            new_r_idx = rightIdx;
            if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("        creating right-biased gap: " << new_r_idx << ", " << new_l_idx);
        } else { // Lingering in center
            //ROS_INFO_STREAM( "central gap" << std::endl;
            new_l_idx = (goal_idx + acceptable_dist) % cfg_->scan.full_scan;
            new_r_idx = subtract_wrap(goal_idx - acceptable_dist, cfg_->scan.full_scan);
            if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("        creating goal-centered gap: " << new_r_idx << ", " << new_l_idx);
        }

        // removed some float casting here
        float orig_gap_size = float(subtract_wrap(leftIdx - rightIdx, cfg_->scan.full_scan));
        float new_l_idx_diff = float(subtract_wrap(new_l_idx - rightIdx, cfg_->scan.full_scan));
        float new_r_idx_diff = float(subtract_wrap(new_r_idx - rightIdx, cfg_->scan.full_scan));

        // ROS_INFO_STREAM("orig_gap_size: " << orig_gap_size);
        // ROS_INFO_STREAM("new_r_idx_diff: " << new_r_idx_diff << ", new_l_idx_diff: " << new_l_idx_diff);

        float new_leftDist = new_l_idx_diff / orig_gap_size * (leftDist - rightDist) + rightDist;
        float new_rightDist = new_r_idx_diff / orig_gap_size * (leftDist - rightDist) + rightDist;

        if (initial) {
            gap.setCvxLeftIdx(new_l_idx);
            gap.setCvxRightIdx(new_r_idx);
            gap.setCvxLeftDist(new_leftDist);            
            gap.setCvxRightDist(new_rightDist);
            gap.mode.reduced_ = true;

            // xLeft = gap.cvxLeftDist() * cos(((float) gap.cvxLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // yLeft = gap.cvxLeftDist() * sin(((float) gap.cvxLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // xRight = gap.cvxRightDist() * cos(((float) gap.cvxRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // yRight = gap.cvxRightDist() * sin(((float) gap.cvxRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
        } else {
            gap.setcvxTermLeftIdx(new_l_idx);
            gap.setcvxTermRightIdx(new_r_idx);
            gap.setcvxTermLeftDist(new_leftDist);
            gap.setcvxTermRightDist(new_rightDist);
            gap.mode.termReduced_ = true;

            // xLeft = gap.cvxTermLeftDist() * cos(((float) gap.cvxTermLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // yLeft = gap.cvxTermLeftDist() * sin(((float) gap.cvxTermLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // xRight = gap.cvxTermRightDist() * cos(((float) gap.cvxTermRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // yRight = gap.cvxTermRightDist() * sin(((float) gap.cvxTermRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
        }

        if (cfg_->debug.manipulation_debug_log) {
            ROS_INFO_STREAM("        post-reduce gap in polar. left: (" << new_l_idx << ", " << new_leftDist << "), right: (" << new_r_idx << ", " << new_rightDist << ")");
            // ROS_INFO_STREAM("post-reduce in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
        }
        return;
    }

    void GapManipulator::convertRadialGap(dynamic_gap::Gap& gap, bool initial) 
    { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        // Return if not radial gap or disabled
        if (!gap.isRadial(initial) || !cfg_->gap_manip.radial_convert) 
        {
            return;
        }

        // if (stored_scan_msgs.ranges.size() != 512) {ROS_FATAL_STREAM("Scan range incorrect gap manip");}

        sensor_msgs::LaserScan desScan;
        int leftIdx, rightIdx;
        float leftDist, rightDist;
        if (initial) {
            desScan = *scan_.get();
            leftIdx = gap.cvxLeftIdx();
            rightIdx = gap.cvxRightIdx();
            leftDist = gap.cvxLeftDist();
            rightDist = gap.cvxRightDist();
        } else {
            desScan = dynamicScan_;
            leftIdx = gap.cvxTermLeftIdx();
            rightIdx = gap.cvxTermRightIdx();
            leftDist = gap.cvxTermLeftDist();
            rightDist = gap.cvxTermRightDist();
        }

        if (cfg_->debug.manipulation_debug_log) 
        {
            ROS_INFO_STREAM("    [convertRadialGap()]");            
            ROS_INFO_STREAM("        pre-AGC gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");
            // ROS_INFO_STREAM("pre-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
        }

        float leftTheta = idx2theta(leftIdx);
        float rightTheta = idx2theta(rightIdx);


        int gap_idx_size = (leftIdx - rightIdx);
        if (gap_idx_size < 0)
            gap_idx_size += cfg_->scan.full_scan; // int(2*gap.half_scan);

        // ROS_INFO_STREAM("gap_idx_size: " << gap_idx_size);

        bool right = gap.isRightType(initial);
        // Extend of rotation to the radial gap 
        // amp-ed by a **small** ratio to ensure the local goal does not exactly fall on the visibility line
        // we are pivoting around the closer point?
        float rot_val = std::atan2(cfg_->gap_manip.epsilon2 * cfg_->gap_manip.rot_ratio, cfg_->gap_manip.epsilon1);
        float theta = right ? (rot_val + 1e-3): -(rot_val + 1e-3);
        // ROS_INFO_STREAM("left: " << left << ", rot_val: " << rot_val);
        // ROS_INFO_STREAM("theta to pivot: " << theta);
        int near_idx, far_idx;
        float near_dist, far_dist;
        float near_theta, far_theta;

        if (right) {
            near_idx = rightIdx;
            far_idx = leftIdx;
            near_dist = rightDist;
            far_dist = leftDist;
            near_theta = rightTheta;
            far_theta = leftTheta;
        } else {
            near_idx = leftIdx;
            far_idx = rightIdx;
            near_dist = leftDist;
            far_dist = rightDist;
            near_theta = leftTheta;
            far_theta = rightTheta;
        }
        
        Eigen::Matrix3f rot_mat;
        // rot_mat: SE(3) matrix that represents desired rotation amount
        rot_mat << cos(theta), -sin(theta), 0,
                   sin(theta), cos(theta), 0,
                   0, 0, 1;
        
        // obtaining near and far theta values from indices
        Eigen::Matrix3f near_rbt;
        // near_rbt, far_rbt: SE(2) matrices that represent translation from rbt origin to far and near points
        near_rbt << 1, 0, near_dist * cos(near_theta),
                    0, 1, near_dist * sin(near_theta),
                    0, 0, 1;
        Eigen::Matrix3f far_rbt;
        far_rbt  << 1, 0, far_dist * cos(far_theta),
                    0, 1, far_dist * sin(far_theta),
                    0, 0, 1;

        // rot_rbt: my guess, transformation matrix FROM rbt origin TO desired pivot point
        Eigen::Matrix3f rot_rbt = near_rbt * (rot_mat * (near_rbt.inverse() * far_rbt));
        // ROS_INFO_STREAM( "rot_rbt: " << rot_rbt);

        // radius and index representing desired pivot point
        // float r = float(sqrt(pow(rot_rbt(0, 2), 2) + pow(rot_rbt(1, 2), 2)));
        float pivoted_theta = std::atan2(rot_rbt(1, 2), rot_rbt(0, 2));
        int idx = theta2idx(pivoted_theta); // int(std::floor((pivoted_theta + M_PI) / stored_scan_msgs.angle_increment));

        // ROS_INFO_STREAM("stored_scan_msgs.angle_increment: " << stored_scan_msgs.angle_increment);

        // ROS_INFO_STREAM("idx: " << idx);

        // Rotation Completed
        // Get minimum dist range val between initial gap point and pivoted gap point
        
        // offset: original right index or the pivoted left
        int init_search_idx = right ? leftIdx : idx;
        // upperbound: pivoted right index or original left  
        int final_search_idx = right ? idx : rightIdx;
        int search_size = final_search_idx - init_search_idx;
        if (search_size < 0)
            search_size += cfg_->scan.full_scan; // 2*int(gap.half_scan);

        // ROS_INFO_STREAM("init_search_idx: " << init_search_idx << ", final_search_idx: " << final_search_idx);

        /*
        if (search_size < 3) // Arbitrary value 
        {
            if (initial)
                gap.goal.discard = true;
            else
                gap.terminalGoal.discard = true;
            return;
        }
        */

        // For wraparound (check to see if this happens)
        init_search_idx = std::max(init_search_idx, 0);
        final_search_idx = std::min(final_search_idx, cfg_->scan.full_scan - 1);

        // ROS_INFO_STREAM("wrapped init_search_idx: " << init_search_idx << ", wrapped final_search_idx: " << final_search_idx);
        search_size = final_search_idx - init_search_idx;
        if (search_size < 0)
            search_size += cfg_->scan.full_scan; // int(2*gap.half_scan);

        std::vector<float> dist(search_size);

        if (search_size == 0) {
            // This shouldn't happen
            return;
        }

        // using the law of cosines to find the index between init/final indices
        // that's shortest distance between near point and laser scan
        // ROS_INFO_STREAM("ranges size: " << stored_scan_msgs.ranges.size());
        try
        {
            int check_idx;
            float range, diff_in_idx;
            for (int i = 0; i < dist.size(); i++) 
            {
                check_idx = (i + init_search_idx) % cfg_->scan.full_scan; // int(2 * gap.half_scan);
                range = desScan.ranges.at(check_idx);
                diff_in_idx = gap_idx_size + (search_size - i);
                dist.at(i) = sqrt(pow(near_dist, 2) + pow(range, 2) -
                    2.0 * near_dist * range * cos(diff_in_idx * cfg_->scan.angle_increment));
                // ROS_INFO_STREAM("checking idx: " << check_idx << ", range of: " << range << ", diff in idx: " << diff_in_idx << ", dist of " << dist.at(i));
            }
        } catch(...) {
            ROS_FATAL_STREAM("convertRadialGap outofBound");
        }

        auto farside_iter = std::min_element(dist.begin(), dist.end());
        int min_dist_idx = (init_search_idx + std::distance(dist.begin(), farside_iter)) % cfg_->scan.full_scan; // int(2*gap.half_scan);
        float min_dist = *farside_iter;

        // ROS_INFO_STREAM("from " << init_search_idx << " to " << final_search_idx << ", min dist of " << min_dist << " at " << min_dist_idx);         

        // pivoting around near point, pointing towards far point or something?
        Eigen::Matrix3f far_near = near_rbt.inverse() * far_rbt;

        float translation_norm = sqrt(pow(far_near(0, 2), 2) + pow(far_near(1, 2), 2));
        // ROS_INFO_STREAM("translation_norm: " << translation_norm);
        // float coefs = far_near.block<2, 1>(0, 2).norm();
        // ROS_INFO_STREAM("translation norm: " << translation_norm);
        // normalizing the pivot direction, multiplying by min dist        
        far_near(0, 2) *= min_dist / translation_norm;     
        far_near(1, 2) *= min_dist / translation_norm;
        Eigen::Matrix3f short_pt = near_rbt * (rot_mat * far_near);

        float r = sqrt(pow(short_pt(0, 2), 2) + pow(short_pt(1, 2), 2));
        float final_theta = std::atan2(short_pt(1, 2), short_pt(0, 2));
        // idx = int (half_num_scan * pivoted_theta / M_PI) + half_num_scan;
        idx = theta2idx(final_theta); // (int) std::floor((final_theta + M_PI) / stored_scan_msgs.angle_increment);

        float new_r_idx, new_l_idx, new_rightDist, new_leftDist;
        if (right) {
            new_l_idx = idx;
            new_r_idx = near_idx;
            new_leftDist = r;
            new_rightDist = near_dist;
        } else {
            new_l_idx = near_idx;
            new_r_idx = idx;
            new_leftDist = near_dist;
            new_rightDist = r;
        }

        if (initial) {
            gap.setCvxLeftIdx(new_l_idx);
            gap.setCvxRightIdx(new_r_idx);
            gap.setCvxLeftDist(new_leftDist);
            gap.setCvxRightDist(new_rightDist);
            gap.mode.RGC_ = true;

            // xLeft = gap.cvxLeftDist() * cos(((float) gap.cvxLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // yLeft = gap.cvxLeftDist() * sin(((float) gap.cvxLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // xRight = gap.cvxRightDist() * cos(((float) gap.cvxRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // yRight = gap.cvxRightDist() * sin(((float) gap.cvxRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
        } else {
            gap.setcvxTermLeftIdx(new_l_idx);
            gap.setcvxTermRightIdx(new_r_idx);
            gap.setcvxTermLeftDist(new_leftDist);
            gap.setcvxTermRightDist(new_rightDist);
            gap.mode.termRGC_ = true;

            // xLeft = gap.cvxTermLeftDist() * cos(((float) gap.cvxTermLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // yLeft = gap.cvxTermLeftDist() * sin(((float) gap.cvxTermLeftIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // xRight = gap.cvxTermRightDist() * cos(((float) gap.cvxTermRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // yRight = gap.cvxTermRightDist() * sin(((float) gap.cvxTermRightIdx() - gap.half_scan) / gap.half_scan * M_PI);
        }

        if (cfg_->debug.manipulation_debug_log) 
        {
            ROS_INFO_STREAM("        post-AGC gap in polar. left: (" << new_l_idx << ", " << new_leftDist << "), right: (" << new_r_idx << ", " << new_rightDist << ")");
            // ROS_INFO_STREAM( "post-AGC gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
        }
    }

    void GapManipulator::radialExtendGap(dynamic_gap::Gap& gap, bool initial) 
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
        float xLeft = (leftDist) * cos(leftTheta);
        float yLeft = (leftDist) * sin(leftTheta);
        float xRight = (rightDist) * cos(rightTheta);
        float yRight = (rightDist) * sin(rightTheta);
        
        Eigen::Vector2f leftPt(xLeft, yLeft);
        Eigen::Vector2f rightPt(xRight, yRight);

        Eigen::Vector2f eL_robot = leftPt / leftPt.norm();
        Eigen::Vector2f eR_robot = rightPt / rightPt.norm();

        if (cfg_->debug.manipulation_debug_log) 
        {
            ROS_INFO_STREAM("    [radialExtendGap()]");
            ROS_INFO_STREAM("        pre-RE gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");
            ROS_INFO_STREAM("        pre-RE gap in cart. left: (" << leftPt[0] << ", " << leftPt[1] << "), right: (" << rightPt[0] << ", " << rightPt[1] << ")");
        }
        
        // ROS_INFO_STREAM("leftPt: (" << xLeft << ", " << yLeft << "), rightPt: (" << xRight << ", " << yRight << ")");
        // ROS_INFO_STREAM("eL_robot: (" << eL_robot[0] << ", " << eL_robot[1] << ") , eR_robot: (" << eR_robot[0] << ", " << eR_robot[1] << ")");

        float leftToRightAngle = getLeftToRightAngle(eL_robot, eR_robot, true);

        float thetaLeft_robot = std::atan2(eL_robot[1], eL_robot[0]);

        float thetaCenter = (thetaLeft_robot - 0.5*leftToRightAngle);

        // middle of gap direction
        Eigen::Vector2f eB(std::cos(thetaCenter), std::sin(thetaCenter));
        // ROS_INFO_STREAM("eB: (" << eB[0] << ", " << eB[1] << ")");

        Eigen::Vector2f norm_eB = eB / eB.norm();
        // angular size of gap
        // ROS_INFO_STREAM("normalized eB: " << norm_eB[0] << ", " << norm_eB[1]);

        // minSafeDist is the minimum distance within the laser scan 
        float s = initial ? gap.getMinSafeDist() : gap.getTerminalMinSafeDist();
        // ROS_INFO_STREAM("min safe dist: " << s);
        
        // point opposite direction of middle of gap, magnitude of min safe dist
        Eigen::Vector2f extendedGapOrigin =  - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * norm_eB; //
        // ROS_INFO_STREAM("extendedGapOrigin: " << extendedGapOrigin[0] << ", " << extendedGapOrigin[1]);

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
            // ROS_INFO_STREAM("theta between qLp and extendedGapOrigin: " << theta_btw_qLp_and_extendedGapOrigin);
            //Eigen::Vector2f left_hypotenuse = length_along_qLp * qLp / qLp.norm();        
            // ROS_INFO_STREAM("length along qLp: " << length_along_qLp << ", left_hypotenuse: " << left_hypotenuse[0] << ", " << left_hypotenuse[1]);
            
            gap.leftBezierOrigin_ =  Rnegpi2 * extendedGapOrigin; // left_hypotenuse + extendedGapOrigin;

            // Eigen::Vector2f qRp = rightPt - extendedGapOrigin;
            // float theta_btw_qRp_and_extendedGapOrigin = std::acos(fwd_extendedGapOrigin.dot(qRp) / (fwd_extendedGapOrigin.norm() * qRp.norm()));
            // float length_along_qRp = fwd_extendedGapOrigin.norm() / cos(theta_btw_qRp_and_extendedGapOrigin);
            // ROS_INFO_STREAM("theta between qRp and extendedGapOrigin: " << theta_btw_qRp_and_extendedGapOrigin);
            // Eigen::Vector2f right_hypotenuse = length_along_qRp * qRp / qRp.norm();        
            // ROS_INFO_STREAM("length along qRp: " << length_along_qRp << ", right_hypotenuse: " << right_hypotenuse[0] << ", " << right_hypotenuse[1]);
            gap.rightBezierOrigin_ = Rpi2 * extendedGapOrigin; // right_hypotenuse + extendedGapOrigin;
        } else 
        {
            gap.termExtendedGapOrigin_ = extendedGapOrigin;
        }

        if (initial)
            gap.mode.convex_ = true;
        else
            gap.mode.termConvex_ = true;
        
        if (cfg_->debug.manipulation_debug_log)
            ROS_INFO_STREAM("        gap extendedGapOrigin: " << extendedGapOrigin[0] << ", " << extendedGapOrigin[1]);

        return;
    }

    void GapManipulator::inflateGapSides(dynamic_gap::Gap& gap, bool initial) 
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

        if (cfg_->debug.manipulation_debug_log) 
        {        
            ROS_INFO_STREAM("    [inflateGapSides()]");
            ROS_INFO_STREAM("        pre-inflate gap in polar. left: (" << leftIdx << ", " << leftDist << "), right: (" << rightIdx << ", " << rightDist << ")");
            ROS_INFO_STREAM("        pre-inflate gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
        }

        Eigen::Vector2f left_norm_vect_robot = leftPt / leftPt.norm();
        Eigen::Vector2f right_norm_vect_robot = rightPt / rightPt.norm();
        float leftToRightAngle = getLeftToRightAngle(left_norm_vect_robot, right_norm_vect_robot, true);

        // inflate inwards by radius * infl
        // rotate by pi/2, norm
        Eigen::Matrix2f Rpi2, Rnegpi2;
        Rpi2 << 0, -1,
                    1, 0;
        Rnegpi2 = -Rpi2;

        
        // PERFORMING ANGULAR INFLATION
        Eigen::Vector2f left_angular_inflation = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * Rnegpi2 * left_norm_vect_robot;
        Eigen::Vector2f new_left_pt = leftPt + left_angular_inflation;
        float new_leftTheta = std::atan2(new_left_pt[1], new_left_pt[0]);

        // ROS_INFO_STREAM("leftTheta: " << leftTheta << ", new_leftTheta: " << new_leftTheta); // << ", leftThetaeft_infl: " << leftThetaeft_infl

        // float rightTheta_infl = ((M_PI / 2) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) / rightPt.norm(); // using s = r*theta
        // float new_rightTheta = rightTheta + rightTheta_infl;
        // new_rightTheta = atanThetaWrap(new_rightTheta);
        
        Eigen::Vector2f right_angular_inflation = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * Rpi2 * right_norm_vect_robot;
        Eigen::Vector2f new_right_pt = rightPt + right_angular_inflation;
        float new_rightTheta = std::atan2(new_right_pt[1], new_right_pt[0]);

        // ROS_INFO_STREAM("rightTheta: " << rightTheta << ", new_rightTheta: " << new_rightTheta); // ", rightTheta_infl: " << rightTheta_infl << 

        Eigen::Vector2f new_left_norm_vect_robot(std::cos(new_leftTheta), std::sin(new_leftTheta));
        Eigen::Vector2f new_right_norm_vect_robot(std::cos(new_rightTheta), std::sin(new_rightTheta));
        float new_leftToRightAngle = getLeftToRightAngle(new_left_norm_vect_robot, new_right_norm_vect_robot, false);
        // ROS_INFO_STREAM("new_leftToRightAngle: " << new_leftToRightAngle);

        int new_r_idx, new_l_idx;
        float range_l_p, range_r_p;
        if (new_leftToRightAngle < 0) 
        {
            // ROS_INFO_STREAM("angular inflation would cause gap to cross, not running:");
            new_r_idx = rightIdx;
            new_l_idx = leftIdx;
            range_l_p = leftDist;
            range_r_p = rightDist;
        } else {
            // need to make sure L/R don't cross each other
            new_r_idx = theta2idx(new_rightTheta); // int((new_rightTheta + M_PI) / stored_scan_msgs.angle_increment);
            new_l_idx = theta2idx(new_leftTheta); // int((new_leftTheta + M_PI) / stored_scan_msgs.angle_increment);
            
            float leftDist_robot = leftDist;
            float rightDist_robot = rightDist;

            float L_to_Lp_angle = getLeftToRightAngle(left_norm_vect_robot, new_left_norm_vect_robot, false);
            float L_to_Rp_angle = getLeftToRightAngle(left_norm_vect_robot, new_right_norm_vect_robot, false);
            range_l_p = (rightDist_robot - leftDist_robot) * L_to_Lp_angle / leftToRightAngle + leftDist_robot;
            range_r_p = (rightDist_robot - leftDist_robot) * L_to_Rp_angle / leftToRightAngle + leftDist_robot;
            if (cfg_->debug.manipulation_debug_log) 
            {
                ROS_INFO_STREAM("        post-angular-inflation gap, left: " << new_l_idx << ", : " << range_l_p << ", right: " << new_r_idx << ", : " << range_r_p << "");
                if (range_l_p > 8 || range_r_p > 8) {
                    ROS_INFO_STREAM("            range is too big");
                }
            }
        }

        // PERFORMING RADIAL INFLATION
        float new_range_l_p = range_l_p + 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        if (desScan.ranges.at(new_l_idx) - new_range_l_p < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) {
            // reject the change
            new_range_l_p = desScan.ranges.at(new_l_idx) - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        }

        float new_range_r_p = range_r_p + 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        if (desScan.ranges.at(new_r_idx) - new_range_r_p < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) {
            // reject the change
            new_range_r_p = desScan.ranges.at(new_r_idx) - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        }

        // ROS_INFO_STREAM("new_l_theta: " << new_l_theta << ", new_r_theta: " << new_r_theta);
        // ROS_INFO_STREAM("int values, left: " << new_l_idx << ", right: " << new_r_idx);

        if (new_r_idx == new_l_idx) // ROS_INFO_STREAM("manipulated indices are same");
            new_l_idx++;

        float new_l_range = range_l_p; // std::min(range_l_p, stored_scan_msgs.ranges.at(new_l_idx));
        float new_r_range = range_r_p; // std::min(range_r_p, stored_scan_msgs.ranges.at(new_r_idx)); // should this be ranges.at - r_infl? 

        if (initial) 
        {
            gap.setCvxLeftIdx(new_l_idx);
            gap.setCvxRightIdx(new_r_idx);      
            gap.setCvxLeftDist(new_l_range);
            gap.setCvxRightDist(new_r_range);
        } else 
        {
            gap.setcvxTermLeftIdx(new_l_idx);
            gap.setcvxTermRightIdx(new_r_idx);
            gap.setcvxTermLeftDist(new_l_range);
            gap.setcvxTermRightDist(new_r_range);
        }

        if (cfg_->debug.manipulation_debug_log) 
        {
            new_leftTheta = idx2theta(new_l_idx);
            new_rightTheta = idx2theta(new_r_idx);
            xLeft = new_l_range * cos(new_leftTheta);
            yLeft = new_l_range * sin(new_leftTheta);
            xRight = new_r_range * cos(new_rightTheta);
            yRight = new_r_range * sin(new_rightTheta);

            ROS_INFO_STREAM("        post-inflate gap in polar. left: (" << new_l_idx << ", " << new_l_range << "), right: (" << new_r_idx << ", " << new_r_range << ")");
            ROS_INFO_STREAM("        post-inflate gap in cart. left: (" << xLeft << ", " << yLeft << "), right: (" << xRight << ", " << yRight << ")");
        }
    }
}
