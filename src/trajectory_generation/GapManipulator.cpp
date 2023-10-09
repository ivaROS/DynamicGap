#include <dynamic_gap/trajectory_generation/GapManipulator.h>

namespace dynamic_gap 
{
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) 
    {
        boost::mutex::scoped_lock lock(egolock);
        msg = msg_;
        num_of_scan = (int)(msg.get()->ranges.size());
        half_num_scan = num_of_scan / 2;
        angle_min = msg.get()->angle_min;
        angle_increment = msg.get()->angle_increment;
    }

    void GapManipulator::updateStaticEgoCircle(const sensor_msgs::LaserScan & static_scan_) 
    {
        boost::mutex::scoped_lock lock(egolock);
        static_scan = static_scan_;
        num_of_scan = (int) (static_scan.ranges.size());
        half_num_scan = num_of_scan / 2;

        angle_min = static_scan.angle_min;
        angle_increment = static_scan.angle_increment;
    }

    void GapManipulator::updateDynamicEgoCircle(dynamic_gap::Gap& gap,
                                                const std::vector<sensor_msgs::LaserScan> & future_scans) 
    {
        dynamic_scan = static_scan;
        double t_iplus1 = gap.gap_lifespan;

        int future_scan_idx = (int) (t_iplus1 / cfg_->traj.integrate_stept);

        dynamic_scan = future_scans[future_scan_idx];

        auto terminal_min_dist = *std::min_element(dynamic_scan.ranges.begin(), dynamic_scan.ranges.end());
        gap.setTerminalMinSafeDist(terminal_min_dist);
    }
    
    void GapManipulator::setTerminalGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal) 
    {
        if (gap.getCategory() == "expanding" || gap.getCategory() == "static") 
        { 
            if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("setting terminal goal for expanding gap");
            setGapWaypoint(gap, localgoal, false);
        } else if (gap.getCategory() == "closing") 
        {
            std::string closing_gap_type;
            if (gap.gap_crossed) 
            {
                closing_gap_type = "crossed";
                Eigen::Vector2f crossing_pt = gap.getCrossingPoint();

                gap.terminal_goal.x = crossing_pt[0];
                gap.terminal_goal.y = crossing_pt[1];
                gap.terminal_goal.set = true;
            } else if (gap.gap_closed) 
            {
                closing_gap_type = "closed";
                ROS_INFO_STREAM("setting terminal goal for closed closing gap");
                Eigen::Vector2f closing_pt = gap.getClosingPoint();
            
                gap.terminal_goal.x = closing_pt[0];
                gap.terminal_goal.y = closing_pt[1];
                gap.terminal_goal.set = true;
            } else 
            {
                closing_gap_type = "existent";
                setGapWaypoint(gap, localgoal, false);
            }

            if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("setting terminal goal for " + closing_gap_type + " closing gap");
        }
    }


    // helper functions need to be placed above where they are used
    bool goal_within(int goal_idx, int idx_lower, int idx_upper, int full_scan) 
    {
        if (idx_lower < idx_upper) 
        {
            // ROS_INFO_STREAM("no wrapping, is goal idx between " << idx_lower << " and " << idx_upper);
            return (goal_idx > idx_lower && goal_idx < idx_upper); //if no wrapping occurs
        } else 
        {
            // ROS_INFO_STREAM("wrapping, is goal idx between " << idx_lower << " and " << full_scan << ", or between " << 0 << " and " << idx_upper);
            return (goal_idx > idx_lower && goal_idx < full_scan) || (goal_idx > 0 && goal_idx < idx_upper); // if wrapping occurs
        }
    }

    void GapManipulator::setGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal, bool initial) 
    { //, sensor_msgs::LaserScan const dynamic_laser_scan){
        int lidx, ridx;
        float ldist, rdist;
        if (initial) 
        {
            lidx = gap.cvx_LIdx();
            ridx = gap.cvx_RIdx();
            ldist = gap.cvx_LDist();
            rdist = gap.cvx_RDist();
        } else 
        {
            lidx = gap.cvx_term_LIdx();
            ridx = gap.cvx_term_RIdx();
            ldist = gap.cvx_term_LDist();
            rdist = gap.cvx_term_RDist();
        }

        // int lidx = initial ? gap.cvx_LIdx() : gap.cvx_term_LIdx();
        // int ridx = initial ? gap.cvx_RIdx() : gap.cvx_term_RIdx();
        // float ldist = initial ? gap.cvx_LDist() : gap.cvx_term_LDist();
        // float rdist = initial ? gap.cvx_RDist() : gap.cvx_term_RDist();

        float theta_l = idx2theta(lidx); // ((float) lidx - half_num_scan) * M_PI / half_num_scan;
        float theta_r = idx2theta(ridx); // ((float) ridx - half_num_scan) * M_PI / half_num_scan;

        float x_l = (ldist) * cos(theta_l);
        float y_l = (ldist) * sin(theta_l);
        float x_r = (rdist) * cos(theta_r);
        float y_r = (rdist) * sin(theta_r);

        Eigen::Vector2f pt_l(x_l, y_l);
        Eigen::Vector2f pt_r(x_r, y_r);

        // auto lr = (pl - pr) / (pl - pr).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pr;
        // auto theta_l = std::atan2(pt_l[1], pt_l[0]);
        // auto lf = (pr - pl) / (pr - pl).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pl; // why are we doing this? we are inflating already.
        // auto theta_r = std::atan2(pt_r[1], pt_r[0]);

        if (cfg_->debug.manipulation_debug_log) 
        { 
            ROS_INFO_STREAM( "~running setGapWaypoint");
            ROS_INFO_STREAM("gap polar points, left: (" << lidx << ", " << ldist << ") , right: (" << ridx << ", " << rdist << ")");
            ROS_INFO_STREAM("gap cart points, left: (" << x_l << ", " << y_l << ") , right: (" << x_r << ", " << y_r << ")");
        }

        Eigen::Vector2f left_vect_robot = pt_l / pt_l.norm();
        Eigen::Vector2f right_vect_robot = pt_r / pt_r.norm();
        float L_to_R_angle = getLeftToRightAngle(left_vect_robot, right_vect_robot, true);

        // Second condition: if angle smaller than M_PI / 3
        // Check if arc length < 3 robot width

        if (gap.artificial) 
        {
            if (initial) 
            {
                gap.goal.x = localgoal.pose.position.x;
                gap.goal.y = localgoal.pose.position.y;
                gap.goal.set = true;
                gap.goal.goalwithin = true;
            } else 
            {
                gap.terminal_goal.x = localgoal.pose.position.x;
                gap.terminal_goal.y = localgoal.pose.position.y;
                gap.terminal_goal.set = true;
                gap.terminal_goal.goalwithin = true;
            }

            if (cfg_->debug.manipulation_debug_log) 
            {
                ROS_INFO_STREAM("Option 0: artificial gap");
                ROS_INFO_STREAM("goal: " << localgoal.pose.position.x << ", " << localgoal.pose.position.y);
                Eigen::Vector2f goal_vector(localgoal.pose.position.x, localgoal.pose.position.y);
                Eigen::Vector2f goal_norm_vector = goal_vector / goal_vector.norm();
                float L_to_goal_angle = getLeftToRightAngle(left_vect_robot, goal_norm_vector, true);
                if (! (0.0 < L_to_goal_angle && L_to_goal_angle < L_to_R_angle)) 
                {
                    ROS_INFO_STREAM("goal outside of gap");
                    ROS_INFO_STREAM("L_to_goal_angle: " << L_to_goal_angle << ", L_to_R_angle: " << L_to_R_angle);
                    float goal_theta = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
                    ROS_INFO_STREAM("left_theta: " << theta_l << ", goal_theta: " << goal_theta << ", right_theta: " << theta_r);
                }
            }
            return;
        }


        bool gap_size_check = L_to_R_angle < M_PI;
        float dist = 0;
        bool small_gap = false;
        if (gap_size_check && !cfg_->planning.planning_inflated) 
        {
            // if smaller than M_PI/3
            dist = sqrt(pow(x_l - x_r, 2) + pow(y_l - y_r, 2));
            small_gap = dist < 4 * cfg_->rbt.r_inscr;
            // need to revise to be purely angle based
        }

        if (small_gap) { // thetalr < thetalf || 
            // FLIPPING HERE
            Eigen::Vector2f left_norm_vect_robot = pt_l / pt_l.norm();
            Eigen::Vector2f right_norm_vect_robot = pt_r / pt_r.norm();
            
            float L_to_R_angle = getLeftToRightAngle(left_norm_vect_robot, right_norm_vect_robot, true);
            
            float beta_left = std::atan2(left_norm_vect_robot[1], left_norm_vect_robot[0]);

            // ROS_INFO_STREAM("L_to_R_angle: " << L_to_R_angle);
            float beta_center = (beta_left - 0.5 * L_to_R_angle); 
            float range_center = (pt_r.norm() + pt_l.norm()) / 2.0;
            float goal_x = range_center * std::cos(beta_center);
            float goal_y = range_center * std::sin(beta_center);
            // ROS_INFO_STREAM("beta_left: " << beta_left << ", beta_right: " << beta_right << ", beta_center: " << beta_center);

            if (initial) {
                gap.goal.set = true;
                gap.goal.x = goal_x;
                gap.goal.y = goal_y;
            } else {
                gap.terminal_goal.set = true;
                gap.terminal_goal.x = goal_x;
                gap.terminal_goal.y = goal_y;
            }

            if (cfg_->debug.manipulation_debug_log) {
                ROS_INFO_STREAM("Option 1: behind gap or small gap");
                ROS_INFO_STREAM("goal: " << goal_x << ", " << goal_y);
                Eigen::Vector2f goal_vector(goal_x, goal_y);
                Eigen::Vector2f goal_norm_vector = goal_vector / goal_vector.norm();
                float L_to_goal_angle = getLeftToRightAngle(left_vect_robot, goal_norm_vector, true);
                if (! (0.0 < L_to_goal_angle && L_to_goal_angle < L_to_R_angle)) {
                    ROS_INFO_STREAM("goal outside of gap");
                    ROS_INFO_STREAM("L_to_goal_angle: " << L_to_goal_angle << ", L_to_R_angle: " << L_to_R_angle);
                    float goal_theta = std::atan2(goal_y, goal_x);
                    ROS_INFO_STREAM("left_theta: " << theta_l << ", goal_theta: " << goal_theta << ", right_theta: " << theta_r);
                }                
            }
            return;
        }

        sensor_msgs::LaserScan stored_scan_msgs = *msg.get(); // initial ? *msg.get() : dynamic_laser_scan;
        float goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        double local_goal_idx = theta2idx(goal_orientation); // std::floor(goal_orientation*half_num_scan/M_PI + half_num_scan);
        
        if (cfg_->debug.manipulation_debug_log) {
            ROS_INFO_STREAM("local goal idx: " << local_goal_idx << ", local goal x/y: (" << localgoal.pose.position.x << ", " << localgoal.pose.position.y << ")");
        }

        bool goal_within_gap_angle = goal_within(local_goal_idx, ridx, lidx, int(2*half_num_scan)); // is localgoal within gap angle
        // ROS_INFO_STREAM("goal_vis: " << goal_vis << ", " << goal_in_range);
        
        if (goal_within_gap_angle) 
        {
            bool goal_vis = checkGoalVisibility(localgoal, theta_r, theta_l, rdist, ldist, stored_scan_msgs); // is localgoal within gap range
            if (goal_vis) {

                if (initial) {
                    gap.goal.x = localgoal.pose.position.x;
                    gap.goal.y = localgoal.pose.position.y;
                    gap.goal.set = true;
                    gap.goal.goalwithin = true;
                } else {
                    gap.terminal_goal.x = localgoal.pose.position.x;
                    gap.terminal_goal.y = localgoal.pose.position.y;
                    gap.terminal_goal.set = true;
                    gap.terminal_goal.goalwithin = true;

                }

                if (cfg_->debug.manipulation_debug_log) {
                    ROS_INFO_STREAM("Option 2: local goal");
                    ROS_INFO_STREAM("goal: " << localgoal.pose.position.x << ", " << localgoal.pose.position.y);
                    Eigen::Vector2f goal_vector(localgoal.pose.position.x, localgoal.pose.position.y);
                    Eigen::Vector2f goal_norm_vector = goal_vector / goal_vector.norm();
                    float L_to_goal_angle = getLeftToRightAngle(left_vect_robot, goal_norm_vector, true);
                    if (! (0.0 < L_to_goal_angle && L_to_goal_angle < L_to_R_angle)) {
                        ROS_INFO_STREAM("goal outside of gap");
                        ROS_INFO_STREAM("L_to_goal_angle: " << L_to_goal_angle << ", L_to_R_angle: " << L_to_R_angle);  
                        float goal_theta = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
                        ROS_INFO_STREAM("left_theta: " << theta_l << ", goal_theta: " << goal_theta << ", right_theta: " << theta_r);
                    }                      
                }
                return;
            }
        }

        // if agc. then the shorter side need to be further in
        // lf: front (first one, left from laser scan)
        // lr: rear (second one, right from laser scan)
        // what do these do?
        if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("Option 3: biasing");

        Eigen::Vector2f local_goal_norm_vect(std::cos(goal_orientation), std::sin(goal_orientation));
        float L_to_goal_angle = getLeftToRightAngle(left_vect_robot, local_goal_norm_vect, true);
        float R_to_goal_angle = getLeftToRightAngle(right_vect_robot, local_goal_norm_vect, true);
 
        if (cfg_->debug.manipulation_debug_log)
        {
            ROS_INFO_STREAM("theta_l: " << theta_l << ", theta_r: " << theta_r << ", theta_localgoal: " << goal_orientation);
            ROS_INFO_STREAM("L_to_R_angle: " << L_to_R_angle << ", L_to_goal_angle: " << L_to_goal_angle << ", R_to_goal_angle: " << R_to_goal_angle);
        }

        float confined_theta; //
        if (theta_r < theta_l) { // gap is not behind
            confined_theta = std::min(theta_l, std::max(theta_r, goal_orientation));
        } else { // gap is behind
            if (0 < L_to_goal_angle && L_to_goal_angle < L_to_R_angle) {
                confined_theta = goal_orientation;
            } else if (std::abs(L_to_goal_angle) < std::abs(R_to_goal_angle)) {
                confined_theta = theta_l;
            } else {
                confined_theta = theta_r;
            }
        }

        double confined_idx = theta2idx(confined_theta); // std::floor(confined_theta*half_num_scan/M_PI + half_num_scan);

        Eigen::Vector2f confined_theta_vect(std::cos(confined_theta), std::sin(confined_theta));

        // needs to be distance between L/confined. Always positive.
        float L_to_conf_angle = getLeftToRightAngle(left_vect_robot, confined_theta_vect, false); 

        if (cfg_->debug.manipulation_debug_log)
        {
            ROS_INFO_STREAM("confined_theta: " << confined_theta);
            ROS_INFO_STREAM("confined idx: " << confined_idx);
            ROS_INFO_STREAM("L_to_conf_angle: " << L_to_conf_angle << ", L_to_R_angle: " << L_to_R_angle);
        }

        float ldist_robot = ldist;
        float rdist_robot = rdist;

        float confined_r = (rdist_robot - ldist_robot) * L_to_conf_angle / L_to_R_angle + ldist_robot;
        Eigen::Vector2f anchor(confined_r * cos(confined_theta), confined_r * sin(confined_theta));
        Eigen::Matrix2f r_pi2, r_negpi2;
        r_pi2 << 0, -1, 1,0; // PI/2 counter clockwise
        r_negpi2 = -r_pi2; // PI/2 clockwise;


        Eigen::Vector2f radial_offset = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * anchor / anchor.norm();

        Eigen::Vector2f angular_offset(0.0, 0.0); 
        if (confined_theta == theta_r) {
            angular_offset = r_pi2 * (pt_r / pt_r.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        } else if (confined_theta == theta_l) {
            angular_offset = r_negpi2 * (pt_l / pt_l.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        } else { // confined_theta == localgoal_theta
            if (L_to_conf_angle / L_to_R_angle < 0.1) { // pretty close to left side
                angular_offset = r_negpi2 * (pt_l / pt_l.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;    
            } else if (L_to_conf_angle / L_to_R_angle > 0.9) { // pretty close to right side
                angular_offset = r_pi2 * (pt_r / pt_r.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            }
        }

        Eigen::Vector2f offset = radial_offset + angular_offset;
        
        if (cfg_->debug.manipulation_debug_log)
        {
            ROS_INFO_STREAM("anchor: " << anchor[0] << ", " << anchor[1]);
            ROS_INFO_STREAM("radial_offset: " << radial_offset[0] << ", " << radial_offset[1]);
            ROS_INFO_STREAM("angular_offset: " << angular_offset[0] << ", " << angular_offset[1]);
        }
        
        Eigen::Vector2f goal_pt = offset + anchor;
        // ROS_INFO_STREAM("anchor: (" << anchor[0] << ", " << anchor[1] << "), offset with r_ins " << cfg_->rbt.r_inscr << " and inf ratio " << cfg_->traj.inf_ratio << ", :(" << offset[0] << ", " << offset[1] << "), goal_pt: (" << goal_pt[0] << ", " << goal_pt[1] << ")");
        if (initial) {
            gap.goal.x = goal_pt(0);
            gap.goal.y = goal_pt(1);
            gap.goal.set = true;
        } else {
            gap.terminal_goal.x = goal_pt(0);
            gap.terminal_goal.y = goal_pt(1);
            gap.terminal_goal.set = true;
        }

        if (cfg_->debug.manipulation_debug_log) {
            ROS_INFO_STREAM("goal: " << goal_pt(0) << ", " << goal_pt(1)); 
            Eigen::Vector2f goal_vector = goal_pt;
            Eigen::Vector2f goal_norm_vector = goal_vector / goal_vector.norm();
            float L_to_goal_angle = getLeftToRightAngle(left_vect_robot, goal_norm_vector, true);
            if (! (0.0 < L_to_goal_angle && L_to_goal_angle < L_to_R_angle)) {
                ROS_INFO_STREAM("goal outside of gap");
                ROS_INFO_STREAM("L_to_goal_angle: " << L_to_goal_angle << ", L_to_R_angle: " << L_to_R_angle);
                float goal_theta = std::atan2(goal_norm_vector[1], goal_norm_vector[0]);
                ROS_INFO_STREAM("left_theta: " << theta_l << ", goal_theta: " << goal_theta << ", right_theta: " << theta_r);         
            }                     
        }           
    }

    bool GapManipulator::checkGoalVisibility(geometry_msgs::PoseStamped localgoal, float theta_r, float theta_l, float rdist, float ldist, sensor_msgs::LaserScan const scan) {
        boost::mutex::scoped_lock lock(egolock);
        // with robot as 0,0 (localgoal in robot frame as well)
        float dist2goal = sqrt(pow(localgoal.pose.position.x, 2) + pow(localgoal.pose.position.y, 2));

        // auto scan = *msg.get();
        auto min_val = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        if (dist2goal < 2 * cfg_->rbt.r_inscr)
            return true;

        // If within closest configuration space
        if (dist2goal < min_val - cfg_->traj.inf_ratio * cfg_->rbt.r_inscr)
            return true;

        // Should be sufficiently far, otherwise we are in trouble
        float goal_angle = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_index = (int) round((goal_angle - scan.angle_min) / scan.angle_increment);

        // get gap's range at localgoal idx
        Eigen::Vector2f left_norm_vect(std::cos(theta_l), std::sin(theta_l));
        Eigen::Vector2f right_norm_vect(std::cos(theta_r), std::sin(theta_r));
        Eigen::Vector2f goal_norm_vect(std::cos(goal_angle), std::sin(goal_angle));

        float L_to_R_angle = getLeftToRightAngle(left_norm_vect, right_norm_vect, true);
        float L_to_goal_angle = getLeftToRightAngle(left_norm_vect, goal_norm_vect, true);
        float localgoal_r = (rdist - ldist) * L_to_goal_angle / L_to_R_angle + ldist;

        /*
        float half_angle = std::asin(cfg_->rbt.r_inscr / dist2goal);
        // int index = std::ceil(half_angle / scan.angle_increment) * 1.5;
        int index = (int)(scan.ranges.size()) / 8;
        int lower_bound = goal_index - index;
        int upper_bound = goal_index + index;
        float min_val_round_goal = *std::min_element(scan.ranges.begin() + lower_bound, scan.ranges.begin() + upper_bound);
        */

        return dist2goal < localgoal_r;
    }

    int subtract_wrap(int a, int b) {
        return (a < 0) ? a+b : a;
    }

    // In place modification
    void GapManipulator::reduceGap(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        if (!msg) return; 
        // msg is from egocircle
        // only part of msg used is angle_increment

        int lidx, ridx;
        float ldist, rdist;
        if (initial) {
            lidx = gap.cvx_LIdx();
            ridx = gap.cvx_RIdx();
            ldist = gap.cvx_LDist();
            rdist = gap.cvx_RDist();
        } else {
            lidx = gap.cvx_term_LIdx();
            ridx = gap.cvx_term_RIdx();
            ldist = gap.cvx_term_LDist();
            rdist = gap.cvx_term_RDist();
        }

        // int lidx = initial ? gap.cvx_LIdx() : gap.cvx_term_LIdx();
        // int ridx = initial ? gap.cvx_RIdx() : gap.cvx_term_RIdx();
        // float ldist = initial ? gap.cvx_LDist() : gap.cvx_term_LDist();
        // float rdist = initial ? gap.cvx_RDist() : gap.cvx_term_RDist();

        double gap_idx_size = (lidx - ridx);
        if (gap_idx_size < 0.0) {
            gap_idx_size += (2*gap.half_scan);
        }
        double gap_theta_size = gap_idx_size * angle_increment;
        // ROS_INFO_STREAM( "gap idx size: " << gap_idx_size << std::endl;

        // threshold = pi right now
        if (gap_theta_size < cfg_->gap_manip.reduction_threshold){
            return;
        }

        // float x_l = (ldist) * cos(((float) lidx - half_num_scan) * M_PI / half_num_scan);
        // float y_l = (ldist) * sin(((float) lidx - half_num_scan) * M_PI / half_num_scan);
        // float x_r = (rdist) * cos(((float) ridx - half_num_scan) * M_PI / half_num_scan);
        // float y_r = (rdist) * sin(((float) ridx - half_num_scan) * M_PI / half_num_scan);

        if (cfg_->debug.manipulation_debug_log) {
            ROS_INFO_STREAM("~running reduceGap~");
            ROS_INFO_STREAM( "pre-reduce gap in polar. left: (" << lidx << ", " << ldist << "), right: (" << ridx << ", " << rdist << ")");
            // ROS_INFO_STREAM("pre-reduce gap in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");
        }

        // the desired size for the reduced gap?
        // target is pi
        int target_idx_size = cfg_->gap_manip.reduction_target / msg.get()->angle_increment;
        int orig_r_biased_l = (ridx + target_idx_size); 
        int orig_l_biased_r = (lidx - target_idx_size);

        int r_biased_l = orig_r_biased_l % num_of_scan; // num_of_scan is int version of 2*half_scan
        int l_biased_r = subtract_wrap(orig_l_biased_r, num_of_scan);

        // ROS_INFO_STREAM("orig_r_biased_l: " << orig_r_biased_l << ", orig_l_biased_r: " << orig_l_biased_r);
        // ROS_INFO_STREAM("r_biased_l: " << r_biased_l << ", l_biased_r: " << l_biased_r);

        float goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_idx = theta2idx(goal_orientation); // goal_orientation / (M_PI / gap.half_scan) + gap.half_scan;
        if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("goal_idx: " << goal_idx);
        int acceptable_dist = target_idx_size / 2; // distance in scan indices

        //ROS_INFO_STREAM( "goal orientation: " << goal_orientation << ", goal idx: " << goal_idx << ", acceptable distance: " << acceptable_dist << std::endl;
        int new_l_idx, new_r_idx;

        int L_minus = subtract_wrap(lidx - acceptable_dist, num_of_scan);
        int L_plus = (lidx + acceptable_dist) % num_of_scan;

        int R_minus = subtract_wrap(ridx - acceptable_dist, num_of_scan);
        int R_plus = (ridx + acceptable_dist) % num_of_scan;

        bool left_biased = goal_within(goal_idx, L_minus, L_plus, num_of_scan); 
        bool right_biased = goal_within(goal_idx, R_minus, R_plus, num_of_scan); 
        if (left_biased) {
            new_l_idx = lidx;
            new_r_idx = l_biased_r;
            if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("creating left-biased gap: " << new_r_idx << ", " << new_l_idx);
        } else if (right_biased) {
            new_l_idx = r_biased_l;
            new_r_idx = ridx;
            if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("creating right-biased gap: " << new_r_idx << ", " << new_l_idx);
        } else { // Lingering in center
            //ROS_INFO_STREAM( "central gap" << std::endl;
            new_l_idx = (goal_idx + acceptable_dist) % num_of_scan;
            new_r_idx = subtract_wrap(goal_idx - acceptable_dist, num_of_scan);
            if (cfg_->debug.manipulation_debug_log) ROS_INFO_STREAM("creating goal-centered gap: " << new_r_idx << ", " << new_l_idx);
        }

        // removed some float casting here
        float orig_gap_size = float(subtract_wrap(lidx - ridx, num_of_scan));
        float new_l_idx_diff = float(subtract_wrap(new_l_idx - ridx, num_of_scan));
        float new_r_idx_diff = float(subtract_wrap(new_r_idx - ridx, num_of_scan));

        // ROS_INFO_STREAM("orig_gap_size: " << orig_gap_size);
        // ROS_INFO_STREAM("new_r_idx_diff: " << new_r_idx_diff << ", new_l_idx_diff: " << new_l_idx_diff);

        float new_ldist = new_l_idx_diff / orig_gap_size * (ldist - rdist) + rdist;
        float new_rdist = new_r_idx_diff / orig_gap_size * (ldist - rdist) + rdist;

        if (initial) {
            gap.setCvxLIdx(new_l_idx);
            gap.setCvxRIdx(new_r_idx);
            gap.setCvxLDist(new_ldist);            
            gap.setCvxRDist(new_rdist);
            gap.mode.reduced = true;

            // x_l = gap.cvx_LDist() * cos(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // y_l = gap.cvx_LDist() * sin(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // x_r = gap.cvx_RDist() * cos(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // y_r = gap.cvx_RDist() * sin(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
        } else {
            gap.setCvxTermLIdx(new_l_idx);
            gap.setCvxTermRIdx(new_r_idx);
            gap.setCvxTermLDist(new_ldist);
            gap.setCvxTermRDist(new_rdist);
            gap.mode.terminal_reduced = true;

            // x_l = gap.cvx_term_LDist() * cos(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // y_l = gap.cvx_term_LDist() * sin(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // x_r = gap.cvx_term_RDist() * cos(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // y_r = gap.cvx_term_RDist() * sin(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
        }

        if (cfg_->debug.manipulation_debug_log) {
            ROS_INFO_STREAM("post-reduce gap in polar. left: (" << new_l_idx << ", " << new_ldist << "), right: (" << new_r_idx << ", " << new_rdist << ")");
            // ROS_INFO_STREAM("post-reduce in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");
        }
        return;
    }

    void GapManipulator::convertRadialGap(dynamic_gap::Gap& gap, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        // Return if not radial gap or disabled
        if (!gap.isRadial(initial) || !cfg_->gap_manip.radial_convert) 
        {
            return;
        }

        // if (stored_scan_msgs.ranges.size() != 512) {ROS_FATAL_STREAM("Scan range incorrect gap manip");}

        sensor_msgs::LaserScan stored_scan_msgs;
        int lidx, ridx;
        float ldist, rdist;
        if (initial) {
            stored_scan_msgs = *msg.get();
            lidx = gap.cvx_LIdx();
            ridx = gap.cvx_RIdx();
            ldist = gap.cvx_LDist();
            rdist = gap.cvx_RDist();
        } else {
            stored_scan_msgs = dynamic_scan;
            lidx = gap.cvx_term_LIdx();
            ridx = gap.cvx_term_RIdx();
            ldist = gap.cvx_term_LDist();
            rdist = gap.cvx_term_RDist();
        }

        float theta_l = idx2theta(lidx); //  ((float) lidx - half_num_scan) * M_PI / half_num_scan;
        float theta_r = idx2theta(ridx); // ((float) ridx - half_num_scan) * M_PI / half_num_scan;
        // float x_l = (ldist) * cos(theta_l);
        // float y_l = (ldist) * sin(theta_l);
        // float x_r = (rdist) * cos(theta_r);
        // float y_r = (rdist) * sin(theta_r);

        if (cfg_->debug.manipulation_debug_log) {
            ROS_INFO_STREAM("~running convertRadialGap~");            
            ROS_INFO_STREAM("pre-AGC gap in polar. left: (" << lidx << ", " << ldist << "), right: (" << ridx << ", " << rdist << ")");
            // ROS_INFO_STREAM("pre-AGC gap in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");
        }

        int gap_idx_size = (lidx - ridx);
        if (gap_idx_size < 0) {
            gap_idx_size += int(2*gap.half_scan);
        }
        // ROS_INFO_STREAM("gap_idx_size: " << gap_idx_size);

        bool right = gap.isRightType(initial);
        // Extend of rotation to the radial gap 
        // amp-ed by a **small** ratio to ensure the local goal does not exactly fall on the visibility line
        // we are pivoting around the closer point?
        float rot_val = (float) std::atan2(cfg_->gap_manip.epsilon2 * cfg_->gap_manip.rot_ratio, cfg_->gap_manip.epsilon1);
        float theta = right ? (rot_val + 1e-3): -(rot_val + 1e-3);
        // ROS_INFO_STREAM("left: " << left << ", rot_val: " << rot_val);
        // ROS_INFO_STREAM("theta to pivot: " << theta);
        int near_idx, far_idx;
        float near_dist, far_dist;
        float near_theta, far_theta;

        if (right) {
            near_idx = ridx;
            far_idx = lidx;
            near_dist = rdist;
            far_dist = ldist;
            near_theta = theta_r;
            far_theta = theta_l;
        } else {
            near_idx = lidx;
            far_idx = ridx;
            near_dist = ldist;
            far_dist = rdist;
            near_theta = theta_l;
            far_theta = theta_r;
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
        int init_search_idx = right ? lidx : idx;
        // upperbound: pivoted right index or original left  
        int final_search_idx = right ? idx : ridx;
        int search_size = final_search_idx - init_search_idx;
        if (search_size < 0)
            search_size += 2*int(gap.half_scan);

        // ROS_INFO_STREAM("init_search_idx: " << init_search_idx << ", final_search_idx: " << final_search_idx);

        if (search_size < 3) // Arbitrary value 
        {
            if (initial)
                gap.goal.discard = true;
            else
                gap.terminal_goal.discard = true;
            return;
        }

        // For wraparound (check to see if this happens)
        init_search_idx = std::max(init_search_idx, 0);
        final_search_idx = std::min(final_search_idx, num_of_scan - 1);

        // ROS_INFO_STREAM("wrapped init_search_idx: " << init_search_idx << ", wrapped final_search_idx: " << final_search_idx);
        search_size = final_search_idx - init_search_idx;
        if (search_size < 0) {
            search_size += int(2*gap.half_scan);
        }

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
                check_idx = (i + init_search_idx) % int(2 * gap.half_scan);
                range = stored_scan_msgs.ranges.at(check_idx);
                diff_in_idx = gap_idx_size + (search_size - i);
                dist.at(i) = sqrt(pow(near_dist, 2) + pow(range, 2) -
                    2.0 * near_dist * range * cos(diff_in_idx * stored_scan_msgs.angle_increment));
                // ROS_INFO_STREAM("checking idx: " << check_idx << ", range of: " << range << ", diff in idx: " << diff_in_idx << ", dist of " << dist.at(i));
            }
        } catch(...) {
            ROS_FATAL_STREAM("convertRadialGap outofBound");
        }

        auto farside_iter = std::min_element(dist.begin(), dist.end());
        int min_dist_idx = (init_search_idx + std::distance(dist.begin(), farside_iter)) % int(2*gap.half_scan);
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

        float r = float(sqrt(pow(short_pt(0, 2), 2) + pow(short_pt(1, 2), 2)));
        float final_theta = std::atan2(short_pt(1, 2), short_pt(0, 2));
        // idx = int (half_num_scan * pivoted_theta / M_PI) + half_num_scan;
        idx = theta2idx(final_theta); // (int) std::floor((final_theta + M_PI) / stored_scan_msgs.angle_increment);

        double new_r_idx, new_l_idx, new_rdist, new_ldist;
        if (right) {
            new_l_idx = idx;
            new_r_idx = near_idx;
            new_ldist = r;
            new_rdist = near_dist;
        } else {
            new_l_idx = near_idx;
            new_r_idx = idx;
            new_ldist = near_dist;
            new_rdist = r;
        }

        if (initial) {
            gap.setCvxLIdx(new_l_idx);
            gap.setCvxRIdx(new_r_idx);
            gap.setCvxLDist(new_ldist);
            gap.setCvxRDist(new_rdist);
            gap.mode.agc = true;

            // x_l = gap.cvx_LDist() * cos(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // y_l = gap.cvx_LDist() * sin(((float) gap.cvx_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // x_r = gap.cvx_RDist() * cos(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // y_r = gap.cvx_RDist() * sin(((float) gap.cvx_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
        } else {
            gap.setCvxTermLIdx(new_l_idx);
            gap.setCvxTermRIdx(new_r_idx);
            gap.setCvxTermLDist(new_ldist);
            gap.setCvxTermRDist(new_rdist);
            gap.mode.terminal_agc = true;

            // x_l = gap.cvx_term_LDist() * cos(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // y_l = gap.cvx_term_LDist() * sin(((float) gap.cvx_term_LIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // x_r = gap.cvx_term_RDist() * cos(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
            // y_r = gap.cvx_term_RDist() * sin(((float) gap.cvx_term_RIdx() - gap.half_scan) / gap.half_scan * M_PI);
        }

        if (cfg_->debug.manipulation_debug_log) {
            ROS_INFO_STREAM( "post-AGC gap in polar. left: (" << new_l_idx << ", " << new_ldist << "), right: (" << new_r_idx << ", " << new_rdist << ")");
            // ROS_INFO_STREAM( "post-AGC gap in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");
        }
    }

    void GapManipulator::radialExtendGap(dynamic_gap::Gap& gap, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        if (!cfg_->gap_manip.radial_extend) {
            ROS_DEBUG_STREAM_THROTTLE(1, "Radial Extension is off");
            return;
        }

        //  = initial ? *msg.get() : dynamic_scan;
        // int half_num_scan = gap.half_scan; // changing this
        
        sensor_msgs::LaserScan stored_scan_msgs;
        int lidx, ridx;
        float ldist, rdist;
        if (initial) {
            lidx = gap.cvx_LIdx();
            ridx = gap.cvx_RIdx();
            ldist = gap.cvx_LDist();
            rdist = gap.cvx_RDist();
        } else {
            lidx = gap.cvx_term_LIdx();
            ridx = gap.cvx_term_RIdx();
            ldist = gap.cvx_term_LDist();
            rdist = gap.cvx_term_RDist();
        }

        // int lidx = initial ? gap.cvx_LIdx() : gap.cvx_term_LIdx();
        // int ridx = initial ? gap.cvx_RIdx() : gap.cvx_term_RIdx();
        // float ldist = initial ? gap.cvx_LDist() : gap.cvx_term_LDist();
        // float rdist = initial ? gap.cvx_RDist() : gap.cvx_term_RDist();

        float theta_l = idx2theta(lidx); // ((float) lidx - half_num_scan) * M_PI / half_num_scan;
        float theta_r = idx2theta(ridx); // ((float) ridx - half_num_scan) * M_PI / half_num_scan;
        float x_l = (ldist) * cos(theta_l);
        float y_l = (ldist) * sin(theta_l);
        float x_r = (rdist) * cos(theta_r);
        float y_r = (rdist) * sin(theta_r);
        
        Eigen::Vector2f pt_l(x_l, y_l);
        Eigen::Vector2f pt_r(x_r, y_r);

        Eigen::Vector2f eL_robot = pt_l / pt_l.norm();
        Eigen::Vector2f eR_robot = pt_r / pt_r.norm();

        if (cfg_->debug.manipulation_debug_log) 
        {
            ROS_INFO_STREAM("running radialExtendGap");
            ROS_INFO_STREAM("pre-RE gap in polar. left: (" << lidx << ", " << ldist << "), right: (" << ridx << ", " << rdist << ")");
            ROS_INFO_STREAM("pre-RE gap in cart. left: (" << pt_l[0] << ", " << pt_l[1] << "), right: (" << pt_r[0] << ", " << pt_r[1] << ")");
        }
        
        // ROS_INFO_STREAM("pt_l: (" << x_l << ", " << y_l << "), pt_r: (" << x_r << ", " << y_r << ")");
        // ROS_INFO_STREAM("eL_robot: (" << eL_robot[0] << ", " << eL_robot[1] << ") , eR_robot: (" << eR_robot[0] << ", " << eR_robot[1] << ")");

        float L_to_R_angle = getLeftToRightAngle(eL_robot, eR_robot, true);

        double beta_left_robot = std::atan2(eL_robot[1], eL_robot[0]);

        double beta_center = (beta_left_robot - 0.5*L_to_R_angle);

        // middle of gap direction
        Eigen::Vector2f eB(std::cos(beta_center), std::sin(beta_center));
        // ROS_INFO_STREAM("eB: (" << eB[0] << ", " << eB[1] << ")");

        Eigen::Vector2f norm_eB = eB / eB.norm();
        // angular size of gap
        // ROS_INFO_STREAM("normalized eB: " << norm_eB[0] << ", " << norm_eB[1]);

        // minSafeDist is the minimum distance within the laser scan 
        float s = initial ? gap.getMinSafeDist() : gap.getTerminalMinSafeDist();
        // ROS_INFO_STREAM("min safe dist: " << s);
        
        // point opposite direction of middle of gap, magnitude of min safe dist
        Eigen::Vector2f qB =  - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * norm_eB; //
        // ROS_INFO_STREAM("qB: " << qB[0] << ", " << qB[1]);

        // Eigen::Vector2f fwd_qB = -qB; 
        Eigen::Matrix2f r_pi2, r_negpi2;
        r_pi2 << 0, -1, 1, 0;
        r_negpi2 = -r_pi2;

        
        if (initial) 
        {
            gap.qB = qB;
            // Eigen::Vector2f qLp = pt_l - qB;
            // float theta_btw_qLp_and_qB = std::acos(fwd_qB.dot(qLp) / (fwd_qB.norm() * qLp.norm()));
            // float length_along_qLp = fwd_qB.norm() / cos(theta_btw_qLp_and_qB);
            // ROS_INFO_STREAM("theta between qLp and qB: " << theta_btw_qLp_and_qB);
            //Eigen::Vector2f left_hypotenuse = length_along_qLp * qLp / qLp.norm();        
            // ROS_INFO_STREAM("length along qLp: " << length_along_qLp << ", left_hypotenuse: " << left_hypotenuse[0] << ", " << left_hypotenuse[1]);
            
            gap.left_bezier_origin =  r_negpi2 * qB; // left_hypotenuse + qB;

            // Eigen::Vector2f qRp = pt_r - qB;
            // float theta_btw_qRp_and_qB = std::acos(fwd_qB.dot(qRp) / (fwd_qB.norm() * qRp.norm()));
            // float length_along_qRp = fwd_qB.norm() / cos(theta_btw_qRp_and_qB);
            // ROS_INFO_STREAM("theta between qRp and qB: " << theta_btw_qRp_and_qB);
            // Eigen::Vector2f right_hypotenuse = length_along_qRp * qRp / qRp.norm();        
            // ROS_INFO_STREAM("length along qRp: " << length_along_qRp << ", right_hypotenuse: " << right_hypotenuse[0] << ", " << right_hypotenuse[1]);
            gap.right_bezer_origin = r_pi2 * qB; // right_hypotenuse + qB;
        } else 
        {
            gap.terminal_qB = qB;
        }

        if (initial)
            gap.mode.convex = true;
        else
            gap.mode.terminal_convex = true;
        
        if (cfg_->debug.manipulation_debug_log)
            ROS_INFO_STREAM("gap qB: " << qB[0] << ", " << qB[1]);

        return;
    }

    void GapManipulator::inflateGapSides(dynamic_gap::Gap& gap, bool initial) {
        // get points

        int lidx, ridx;
        float ldist, rdist;
        if (initial) {
            lidx = gap.cvx_LIdx();
            ridx = gap.cvx_RIdx();
            ldist = gap.cvx_LDist();
            rdist = gap.cvx_RDist();
        } else {
            lidx = gap.cvx_term_LIdx();
            ridx = gap.cvx_term_RIdx();
            ldist = gap.cvx_term_LDist();
            rdist = gap.cvx_term_RDist();
        }
        // int lidx = initial ? gap.cvx_LIdx() : gap.cvx_term_LIdx();
        // int ridx = initial ? gap.cvx_RIdx() : gap.cvx_term_RIdx();
        // float ldist = initial ? gap.cvx_LDist() : gap.cvx_term_LDist();
        // float rdist = initial ? gap.cvx_RDist() : gap.cvx_term_RDist();

        float theta_l = idx2theta(lidx); // ((float) lidx - half_num_scan) * M_PI / half_num_scan;
        float theta_r = idx2theta(ridx); // ((float) ridx - half_num_scan) * M_PI / half_num_scan;
        float x_l = (ldist) * cos(theta_l);
        float y_l = (ldist) * sin(theta_l);
        float x_r = (rdist) * cos(theta_r);
        float y_r = (rdist) * sin(theta_r);
        
        Eigen::Vector2f pt_l(x_l, y_l);
        Eigen::Vector2f pt_r(x_r, y_r);

        if (cfg_->debug.manipulation_debug_log) {        
            ROS_INFO_STREAM("~running inflateGapSides");
            ROS_INFO_STREAM("pre-inflate gap in polar. left: (" << lidx << ", " << ldist << "), right: (" << ridx << ", " << rdist << ")");
            ROS_INFO_STREAM("pre-inflate gap in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");
        }

        Eigen::Vector2f left_norm_vect_robot = pt_l / pt_l.norm();
        Eigen::Vector2f right_norm_vect_robot = pt_r / pt_r.norm();
        float L_to_R_angle = getLeftToRightAngle(left_norm_vect_robot, right_norm_vect_robot, true);

        // inflate inwards by radius * infl
        // rotate by pi/2, norm
        Eigen::Matrix2f r_pi2, r_negpi2;
        r_pi2 << 0, -1,
                    1, 0;
        r_negpi2 = -r_pi2;

        
        // PERFORMING ANGULAR INFLATION
        Eigen::Vector2f left_angular_inflation = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * r_negpi2 * left_norm_vect_robot;
        Eigen::Vector2f new_left_pt = pt_l + left_angular_inflation;
        float new_theta_l = std::atan2(new_left_pt[1], new_left_pt[0]);

        // ROS_INFO_STREAM("theta_l: " << theta_l << ", new_theta_l: " << new_theta_l); // << ", theta_left_infl: " << theta_left_infl

        // float theta_r_infl = ((M_PI / 2) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) / pt_r.norm(); // using s = r*theta
        // float new_theta_r = theta_r + theta_r_infl;
        // new_theta_r = atanThetaWrap(new_theta_r);
        
        Eigen::Vector2f right_angular_inflation = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio * r_pi2 * right_norm_vect_robot;
        Eigen::Vector2f new_right_pt = pt_r + right_angular_inflation;
        float new_theta_r = std::atan2(new_right_pt[1], new_right_pt[0]);

        // ROS_INFO_STREAM("theta_r: " << theta_r << ", new_theta_r: " << new_theta_r); // ", theta_r_infl: " << theta_r_infl << 

        Eigen::Vector2f new_left_norm_vect_robot(std::cos(new_theta_l), std::sin(new_theta_l));
        Eigen::Vector2f new_right_norm_vect_robot(std::cos(new_theta_r), std::sin(new_theta_r));
        float new_L_to_R_angle = getLeftToRightAngle(new_left_norm_vect_robot, new_right_norm_vect_robot, false);
        // ROS_INFO_STREAM("new_L_to_R_angle: " << new_L_to_R_angle);

        sensor_msgs::LaserScan stored_scan_msgs = *msg.get(); // initial ? *msg.get() : dynamic_scan;
        int new_r_idx, new_l_idx;
        float range_l_p, range_r_p;
        if (new_L_to_R_angle < 0) {
            // ROS_INFO_STREAM("angular inflation would cause gap to cross, not running:");
            new_r_idx = ridx;
            new_l_idx = lidx;
            range_l_p = ldist;
            range_r_p = rdist;
        } else {
            // need to make sure L/R don't cross each other
            new_r_idx = theta2idx(new_theta_r); // int((new_theta_r + M_PI) / stored_scan_msgs.angle_increment);
            new_l_idx = theta2idx(new_theta_l); // int((new_theta_l + M_PI) / stored_scan_msgs.angle_increment);
            
            float ldist_robot = ldist;
            float rdist_robot = rdist;

            float L_to_Lp_angle = getLeftToRightAngle(left_norm_vect_robot, new_left_norm_vect_robot, false);
            float L_to_Rp_angle = getLeftToRightAngle(left_norm_vect_robot, new_right_norm_vect_robot, false);
            range_l_p = (rdist_robot - ldist_robot) * L_to_Lp_angle / L_to_R_angle + ldist_robot;
            range_r_p = (rdist_robot - ldist_robot) * L_to_Rp_angle / L_to_R_angle + ldist_robot;
            if (cfg_->debug.manipulation_debug_log) {
                ROS_INFO_STREAM("post-angular-inflation gap, left: " << new_l_idx << ", : " << range_l_p << ", right: " << new_r_idx << ", : " << range_r_p << "");
                if (range_l_p > 8 || range_r_p > 8) {
                    ROS_INFO_STREAM("range is too big");
                }
            }
        }

        // PERFORMING RADIAL INFLATION
        float new_range_l_p = range_l_p + 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        if (stored_scan_msgs.ranges.at(new_l_idx) - new_range_l_p < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) {
            // reject the change
            new_range_l_p = stored_scan_msgs.ranges.at(new_l_idx) - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        }

        float new_range_r_p = range_r_p + 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        if (stored_scan_msgs.ranges.at(new_r_idx) - new_range_r_p < cfg_->rbt.r_inscr * cfg_->traj.inf_ratio) {
            // reject the change
            new_range_r_p = stored_scan_msgs.ranges.at(new_r_idx) - cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        }

        // ROS_INFO_STREAM("new_l_theta: " << new_l_theta << ", new_r_theta: " << new_r_theta);
        // ROS_INFO_STREAM("int values, left: " << new_l_idx << ", right: " << new_r_idx);

        if (new_r_idx == new_l_idx) {
            // ROS_INFO_STREAM("manipulated indices are same");
            new_l_idx++;
        }

        float new_l_range = range_l_p; // std::min(range_l_p, stored_scan_msgs.ranges.at(new_l_idx));
        float new_r_range = range_r_p; // std::min(range_r_p, stored_scan_msgs.ranges.at(new_r_idx)); // should this be ranges.at - r_infl? 

        if (initial) {
            gap.setCvxLIdx(new_l_idx);
            gap.setCvxRIdx(new_r_idx);      
            gap.setCvxLDist(new_l_range);
            gap.setCvxRDist(new_r_range);

        } else {
            gap.setCvxTermLIdx(new_l_idx);
            gap.setCvxTermRIdx(new_r_idx);
            gap.setCvxTermLDist(new_l_range);
            gap.setCvxTermRDist(new_r_range);
        }

        if (cfg_->debug.manipulation_debug_log) 
        {
            new_theta_l = idx2theta(new_l_idx);
            new_theta_r = idx2theta(new_r_idx);
            x_l = new_l_range * cos(new_theta_l);
            y_l = new_l_range * sin(new_theta_l);
            x_r = new_r_range * cos(new_theta_r);
            y_r = new_r_range * sin(new_theta_r);

            ROS_INFO_STREAM("post-inflate gap in polar. left: (" << new_l_idx << ", " << new_l_range << "), right: (" << new_r_idx << ", " << new_r_range << ")");
            ROS_INFO_STREAM( "post-inflate gap in cart. left: (" << x_l << ", " << y_l << "), right: (" << x_r << ", " << y_r << ")");
        }
    }
}
