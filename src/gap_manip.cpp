#include <dynamic_gap/gap_manip.h>

namespace dynamic_gap {
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egolock);
        msg = msg_;
        num_of_scan = (int)(msg.get()->ranges.size());
    }

    void GapManipulator::updateStaticEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egolock);
        static_msg = msg_;
    }

    void GapManipulator::updateDynamicEgoCircle(std::vector<dynamic_gap::Gap> curr_raw_gaps, 
                                                dynamic_gap::Gap& gap,
                                                std::vector<geometry_msgs::Pose> & _agent_odoms, 
                                                std::vector<geometry_msgs::Vector3Stamped> _agent_vels,
                                                dynamic_gap::TrajectoryArbiter * trajArbiter) {
        dynamic_scan = *static_msg.get();
        double t_i = 0.0;
        double t_iplus1 = gap.gap_lifespan;
        /*
        std::vector<dynamic_gap::cart_model *> raw_models;
        for (auto gap : curr_raw_gaps) {
            raw_models.push_back(gap.left_model);
            raw_models.push_back(gap.right_model);
        }
        */

        trajArbiter->recoverDynamicEgocircleCheat(t_i, t_iplus1, _agent_odoms, _agent_vels, dynamic_scan, false);

        auto terminal_min_dist = *std::min_element(dynamic_scan.ranges.begin(), dynamic_scan.ranges.end());
        gap.setTerminalMinSafeDist(terminal_min_dist);
        // trajArbiter->recoverDynamicEgoCircle(t_i, t_iplus1, raw_models, dynamic_scan);
    }
    
    void GapManipulator::setTerminalGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal) {
        if (gap.getCategory() == "expanding" || gap.getCategory() == "static") { 
            ROS_INFO_STREAM("setting terminal goal for expanding gap");
            setGapWaypoint(gap, localgoal, false);
        } else if (gap.getCategory() == "closing") {
            if (gap.gap_crossed) {
                ROS_INFO_STREAM("setting terminal goal for crossed closing gap");
                Eigen::Vector2f crossing_pt = gap.getCrossingPoint();

                gap.terminal_goal.x = crossing_pt[0];
                gap.terminal_goal.y = crossing_pt[1];
                gap.terminal_goal.set = true;
            } else if (gap.gap_closed) {
                ROS_INFO_STREAM("setting terminal goal for closed closing gap");
                Eigen::Vector2f closing_pt = gap.getClosingPoint();
            
                gap.terminal_goal.x = closing_pt[0];
                gap.terminal_goal.y = closing_pt[1];
                gap.terminal_goal.set = true;
            } else {
                ROS_INFO_STREAM("setting terminal goal for existent closing gap");
                setGapWaypoint(gap, localgoal, false);
            }
        }  
    }


    // helper functions need to be placed above where they are used
    bool goal_within(int goal_idx, int idx_lower, int idx_upper, int full_scan) {
        if (idx_lower < idx_upper) {
            ROS_INFO_STREAM("no wrapping, is goal idx between " << idx_lower << " and " << idx_upper);
            return (goal_idx > idx_lower && goal_idx < idx_upper); //if no wrapping occurs
        } else {
            ROS_INFO_STREAM("wrapping, is goal idx between " << idx_lower << " and " << full_scan << ", or between " << 0 << " and " << idx_upper);
            return (goal_idx > idx_lower && goal_idx < full_scan) || (goal_idx > 0 && goal_idx < idx_upper); // if wrapping occurs
        }
    }

    void GapManipulator::setGapWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan){
        //ROS_INFO_STREAM( "~running setGapWaypoint" << std::endl;
        auto half_num_scan = gap.half_scan;

        int lidx = initial ? gap.convex.convex_lidx : gap.convex.terminal_lidx;
        int ridx = initial ? gap.convex.convex_ridx : gap.convex.terminal_ridx;
        float ldist = initial ? gap.convex.convex_ldist : gap.convex.terminal_ldist;
        float rdist = initial ? gap.convex.convex_rdist : gap.convex.terminal_rdist;

        float x1, x2, y1, y2;
        x1 = (ldist) * cos(-((float) half_num_scan - lidx) / half_num_scan * M_PI);
        y1 = (ldist) * sin(-((float) half_num_scan - lidx) / half_num_scan * M_PI);

        x2 = (rdist) * cos(-((float) half_num_scan - ridx) / half_num_scan * M_PI);
        y2 = (rdist) * sin(-((float) half_num_scan - ridx) / half_num_scan * M_PI);

        Eigen::Vector2f pl(x1, y1);
        Eigen::Vector2f pr(x2, y2);
        
        ROS_INFO_STREAM("gap index/dist, left: (" << lidx << ", " << ldist << ") , right: (" << ridx << ", " << rdist << ")");
        // ROS_INFO_STREAM("gap x/y, left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")");
        
        auto left_ori = lidx * msg.get()->angle_increment + msg.get()->angle_min;
        auto right_ori = ridx * msg.get()->angle_increment + msg.get()->angle_min;

        // Second condition: if angle smaller than M_PI / 3
        // Check if arc length < 3 robot width
        double gap_angle = right_ori - left_ori;;
        if (gap_angle < 0) {
            gap_angle += 2*M_PI;
        }

        if (gap.artificial) {
            ROS_INFO_STREAM("Option 0: artificial gap");
            if (initial) {
                gap.goal.x = localgoal.pose.position.x;
                gap.goal.y = localgoal.pose.position.y;
                gap.goal.set = true;
                gap.goal.goalwithin = true;
                ROS_INFO_STREAM("goal: " << gap.goal.x << ", " << gap.goal.y);
            } else {
                gap.terminal_goal.x = localgoal.pose.position.x;
                gap.terminal_goal.y = localgoal.pose.position.y;
                gap.terminal_goal.set = true;
                gap.terminal_goal.goalwithin = true;
                ROS_INFO_STREAM("terminal goal: " << gap.terminal_goal.x << ", " << gap.terminal_goal.y);
            }
            return;
        }


        bool gap_size_check = gap_angle < M_PI;
        float dist = 0;
        bool small_gap = false;
        if (gap_size_check && !cfg_->planning.planning_inflated) {
            // if smaller than M_PI/3
            dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
            small_gap = dist < 4 * cfg_->rbt.r_inscr;
        }

        if (small_gap) { // thetalr < thetalf || 
            ROS_INFO_STREAM("Option 1: behind gap or small gap");
            Eigen::Vector2f left_bearing_norm_vect = pr / pr.norm();
            Eigen::Vector2f right_bearing_norm_vect = pl / pl.norm();
            //  ROS_INFO_STREAM("left bearing vector: " << left_bearing_norm_vect[0] << ", " << left_bearing_norm_vect[1]);
            // ROS_INFO_STREAM("right bearing vector: " << right_bearing_norm_vect[0] << ", " << right_bearing_norm_vect[1]);
            float beta_left = std::atan2(left_bearing_norm_vect[1], left_bearing_norm_vect[0]);
            float beta_right = std::atan2(right_bearing_norm_vect[1], right_bearing_norm_vect[0]);
            float det = left_bearing_norm_vect[0]*right_bearing_norm_vect[1] - left_bearing_norm_vect[1]*right_bearing_norm_vect[0];      
            float dot = left_bearing_norm_vect[0]*right_bearing_norm_vect[0] + left_bearing_norm_vect[1]*right_bearing_norm_vect[1];

            float swept_check = -std::atan2(det, dot);     
            float L_to_R_angle = swept_check;

            if (L_to_R_angle < 0) {
                L_to_R_angle += 2*M_PI; 
            }

            // ROS_INFO_STREAM("L_to_R_angle: " << L_to_R_angle);
            float subtract_angle = 0.5f * L_to_R_angle;
            // ROS_INFO_STREAM("subtract_angle: " << subtract_angle);
            float beta_center = (beta_left - subtract_angle); 
            float range_center = (pl.norm() + pr.norm()) / 2.0;
            float goal_x = range_center * std::cos(beta_center);
            float goal_y = range_center * std::sin(beta_center);
            // ROS_INFO_STREAM("beta_left: " << beta_left << ", beta_right: " << beta_right << ", beta_center: " << beta_center);

            if (initial) {
                gap.goal.set = true;
                gap.goal.x = goal_x;
                gap.goal.y = goal_y;
                ROS_INFO_STREAM("goal: " << gap.goal.x << ", " << gap.goal.y);
            } else {
                gap.terminal_goal.set = true;
                gap.terminal_goal.x = goal_x;
                gap.terminal_goal.y = goal_y;
                ROS_INFO_STREAM("terminal goal: " << gap.terminal_goal.x << ", " << gap.terminal_goal.y);
            }

            return;
        }

        sensor_msgs::LaserScan stored_scan_msgs = *msg.get(); // initial ? *msg.get() : dynamic_laser_scan;
        float goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        double local_goal_idx = std::floor(goal_orientation*half_num_scan/M_PI + half_num_scan);
        ROS_INFO_STREAM("local goal idx: " << local_goal_idx << ", local goal x/y: (" << localgoal.pose.position.x << ", " << localgoal.pose.position.y << ")");
        bool goal_vis = checkGoalVisibility(localgoal, stored_scan_msgs); // is localgoal within scan
        bool goal_in_range = goal_within(local_goal_idx, lidx, ridx, int(2*half_num_scan)); // is localgoal within gap
        // ROS_INFO_STREAM("goal_vis: " << goal_vis << ", " << goal_in_range);
        
        if (goal_vis && goal_in_range) {
            ROS_INFO_STREAM("Option 2: local goal");
            if (initial) {
                gap.goal.x = localgoal.pose.position.x;
                gap.goal.y = localgoal.pose.position.y;
                gap.goal.set = true;
                gap.goal.goalwithin = true;
                ROS_INFO_STREAM("goal: " << gap.goal.x << ", " << gap.goal.y);
            } else {
                gap.terminal_goal.x = localgoal.pose.position.x;
                gap.terminal_goal.y = localgoal.pose.position.y;
                gap.terminal_goal.set = true;
                gap.terminal_goal.goalwithin = true;
                ROS_INFO_STREAM("terminal goal: " << gap.terminal_goal.x << ", " << gap.terminal_goal.y);

            }
            return;
        }

        ROS_INFO_STREAM("Option 3: biasing");
        // if agc. then the shorter side need to be further in
        // lf: front (first one, left from laser scan)
        // lr: rear (second one, right from laser scan)
        // what do these do?
        auto lf = (pr - pl) / (pr - pl).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pl;
        auto thetalf = car2pol(lf)(1);
        auto lr = (pl - pr) / (pl - pr).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pr;
        auto thetalr = car2pol(lr)(1);
        
        ROS_INFO_STREAM("thetalf: " << thetalf << ", thetalr: " << thetalr << ", theta_localgoal: " << goal_orientation);

        float confined_theta; //
        if (thetalf < thetalr) { // gap is not behind
            confined_theta = std::min(thetalr, std::max(thetalf, goal_orientation));
        } else { // gap is behind
            if (goal_orientation > 0) {
                confined_theta = std::max(thetalf, goal_orientation);
            } else {
                confined_theta = std::min(thetalr, goal_orientation);
            }
        }

        ROS_INFO_STREAM("confined_theta: " << confined_theta);

        double confined_idx = std::floor(confined_theta*half_num_scan/M_PI + half_num_scan);
        // ROS_INFO_STREAM("confined idx: " << confined_idx);
        float confined_r = (rdist - ldist) * (confined_theta - thetalf) / (thetalr - thetalf) + ldist;
        float xg = confined_r * cos(confined_theta);
        float yg = confined_r * sin(confined_theta);
        Eigen::Vector2f anchor(xg, yg);
        Eigen::Matrix2f r_pi2, r_negpi2;
        r_pi2 << 0, -1, 1,0; // PI/2 counter clockwise
        r_negpi2 = -r_pi2; // PI/2 clockwise;


        ROS_INFO_STREAM("anchor: " << anchor[0] << ", " << anchor[1]);
        Eigen::Vector2f offset(0.0, 0.0);
        if (confined_theta == thetalf) {
            offset = r_pi2 * (lf / lf.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        } else if (confined_theta == thetalr) {
            offset = r_negpi2 * (lr / lr.norm()) * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        }

        ROS_INFO_STREAM("offset: " << offset[0] << ", " << offset[1]);


        auto goal_pt = offset + anchor;
        // ROS_INFO_STREAM("anchor: (" << anchor[0] << ", " << anchor[1] << "), offset with r_ins " << cfg_->rbt.r_inscr << " and inf ratio " << cfg_->traj.inf_ratio << ", :(" << offset[0] << ", " << offset[1] << "), goal_pt: (" << goal_pt[0] << ", " << goal_pt[1] << ")");
        if (initial) {
            gap.goal.x = goal_pt(0);
            gap.goal.y = goal_pt(1);
            gap.goal.set = true;
            ROS_INFO_STREAM("goal: " << gap.goal.x << ", " << gap.goal.y);            
        } else {
            gap.terminal_goal.x = goal_pt(0);
            gap.terminal_goal.y = goal_pt(1);
            gap.terminal_goal.set = true;
            ROS_INFO_STREAM("terminal goal: " << gap.terminal_goal.x << ", " << gap.terminal_goal.y);
        }

    }

    bool GapManipulator::checkGoalWithinGapAngleRange(dynamic_gap::Gap& gap, double gap_goal_idx, float lidx, float ridx) { 
        return (lidx <= gap_goal_idx && gap_goal_idx <= ridx);
    }

    bool GapManipulator::checkGoalVisibility(geometry_msgs::PoseStamped localgoal, sensor_msgs::LaserScan const scan) {
        boost::mutex::scoped_lock lock(egolock);
        // with robot as 0,0 (localgoal in robot frame as well)
        double dist2goal = sqrt(pow(localgoal.pose.position.x, 2) + pow(localgoal.pose.position.y, 2));

        // auto scan = *msg.get();
        auto min_val = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        if (dist2goal < 2 * cfg_->rbt.r_inscr) {
            return true;
        }

        // If within closest configuration space
        if (dist2goal < min_val - cfg_->traj.inf_ratio * cfg_->rbt.r_inscr) {
            return true;
        }

        // Should be sufficiently far, otherwise we are in trouble
        double goal_angle = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_index = (int) round((goal_angle - scan.angle_min) / scan.angle_increment);

        double half_angle = std::asin(cfg_->rbt.r_inscr / dist2goal);
        // int index = std::ceil(half_angle / scan.angle_increment) * 1.5;
        int index = (int)(scan.ranges.size()) / 8;
        int lower_bound = goal_index - index;
        int upper_bound = goal_index + index;
        float min_val_round_goal = *std::min_element(scan.ranges.begin() + lower_bound, scan.ranges.begin() + upper_bound);

        return dist2goal < min_val_round_goal;
    }

    int subtract_wrap(int a, int b) {
        return (a < 0) ? a+b : a;
    }

    // In place modification
    void GapManipulator::reduceGap(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        if (!msg) return; 
        // msg is from egocircle
        // only part of msg used is angle_increment

        int lidx = initial ? gap.convex.convex_lidx : gap.convex.terminal_lidx;
        int ridx = initial ? gap.convex.convex_ridx : gap.convex.terminal_ridx;
        float ldist = initial ? gap.convex.convex_ldist : gap.convex.terminal_ldist;
        float rdist = initial ? gap.convex.convex_rdist : gap.convex.terminal_rdist;

        double gap_idx_size = (ridx - lidx);

        if (gap_idx_size < 0.0) {
            ROS_INFO_STREAM("reducing behind gap");
            gap_idx_size += (2*gap.half_scan);
        }

        double gap_theta_size = gap_idx_size * (msg.get()->angle_increment);
        // ROS_INFO_STREAM( "gap idx size: " << gap_idx_size << std::endl;

        // threshold = pi right now
        if (gap_theta_size < cfg_->gap_manip.reduction_threshold){
            return;
        }

        // ROS_INFO_STREAM("~running reduceGap~");

        float x1, x2, y1, y2;
        x1 = (ldist) * cos(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);
        y1 = (ldist) * sin(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);

        x2 = (rdist) * cos(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);
        y2 = (rdist) * sin(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);

        ROS_INFO_STREAM( "pre-reduce gap in polar. left: (" << lidx << ", " << ldist << ") , right: (" << ridx << ", " << rdist << ")");
        ROS_INFO_STREAM("pre-reduce gap in cart. left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")");


        // the desired size for the reduced gap?
        // target is pi
        int target_idx_size = cfg_->gap_manip.reduction_target / msg.get()->angle_increment;
        int orig_l_biased_r = (lidx + target_idx_size); 
        int orig_r_biased_l = (ridx - target_idx_size);

        int l_biased_r = orig_l_biased_r % num_of_scan; // num_of_scan is int version of 2*half_scan
        int r_biased_l = subtract_wrap(orig_r_biased_l, num_of_scan);

        ROS_INFO_STREAM("orig_l_biased_r: " << orig_l_biased_r << ", orig_r_biased_l: " << orig_r_biased_l);
        ROS_INFO_STREAM("l_biased_r: " << l_biased_r << ", r_biased_l: " << r_biased_l);

        double goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_idx = goal_orientation / (M_PI / gap.half_scan) + gap.half_scan;
        ROS_INFO_STREAM("goal_idx: " << goal_idx);
        int acceptable_dist = target_idx_size / 2; // distance in scan indices

        //ROS_INFO_STREAM( "goal orientation: " << goal_orientation << ", goal idx: " << goal_idx << ", acceptable distance: " << acceptable_dist << std::endl;
        int new_l_idx, new_r_idx;

        int R_minus = subtract_wrap(ridx - acceptable_dist, num_of_scan);
        int R_plus = (ridx + acceptable_dist) % num_of_scan;

        int L_minus = subtract_wrap(lidx - acceptable_dist, num_of_scan);
        int L_plus = (lidx + acceptable_dist) % num_of_scan;

        ROS_INFO_STREAM("testing right_biased with lower: " << R_minus << ", " << R_plus);
        bool right_biased = goal_within(goal_idx, R_minus, R_plus, num_of_scan); 
        ROS_INFO_STREAM("testing left_biased with lower: " << L_minus << ", " << L_plus);
        bool left_biased = goal_within(goal_idx, L_minus, L_plus, num_of_scan); 
        if (right_biased) {
            new_r_idx = ridx;
            new_l_idx = r_biased_l;
            ROS_INFO_STREAM("creating right-biased gap: " << new_l_idx << ", " << new_r_idx);
        } else if (left_biased) {
            new_r_idx = l_biased_r;
            new_l_idx = lidx;
            ROS_INFO_STREAM("creating left-biased gap: " << new_l_idx << ", " << new_r_idx);
        } else { // Lingering in center
            //ROS_INFO_STREAM( "central gap" << std::endl;
            new_l_idx = subtract_wrap(goal_idx - acceptable_dist, num_of_scan);
            new_r_idx = (goal_idx + acceptable_dist) % num_of_scan;
            ROS_INFO_STREAM("creating goal-centered gap: " << new_l_idx << ", " << new_r_idx);
        }

        // removed some float casting here
        float orig_gap_size = float(subtract_wrap(ridx - lidx, num_of_scan));
        ROS_INFO_STREAM("orig_gap_size: " << orig_gap_size);
        float new_l_idx_diff = float(subtract_wrap(new_l_idx - lidx, num_of_scan));
        float new_r_idx_diff = float(subtract_wrap(new_r_idx - lidx, num_of_scan));
        ROS_INFO_STREAM("new_l_idx_diff: " << new_l_idx_diff << ", new_r_idx_diff: " << new_r_idx_diff);


        float new_ldist = new_l_idx_diff / orig_gap_size * (rdist - ldist) + ldist;
        float new_rdist = new_r_idx_diff / orig_gap_size * (rdist - ldist) + ldist;
        
        if (initial) {
            gap.convex.convex_lidx = new_l_idx;
            gap.convex.convex_ridx = new_r_idx;
            gap.convex.convex_ldist = new_ldist + cfg_->gap_viz.viz_jitter;
            gap.convex.convex_rdist = new_rdist + cfg_->gap_viz.viz_jitter;
            gap.mode.reduced = true;

            x1 = (gap.convex.convex_ldist) * cos(-((float) gap.half_scan - gap.convex.convex_lidx) / gap.half_scan * M_PI);
            y1 = (gap.convex.convex_ldist) * sin(-((float) gap.half_scan - gap.convex.convex_lidx) / gap.half_scan * M_PI);

            x2 = (gap.convex.convex_rdist) * cos(-((float) gap.half_scan - gap.convex.convex_ridx) / gap.half_scan * M_PI);
            y2 = (gap.convex.convex_rdist) * sin(-((float) gap.half_scan - gap.convex.convex_ridx) / gap.half_scan * M_PI);
            ROS_INFO_STREAM("post-reduce gap in polar. left: (" << gap.convex.convex_lidx << ", " << gap.convex.convex_ldist << "), right: (" << gap.convex.convex_ridx << ", " << gap.convex.convex_rdist << ")");
        } else {
            gap.convex.terminal_lidx = new_l_idx;
            gap.convex.terminal_ridx = new_r_idx;
            gap.convex.terminal_ldist = new_ldist + cfg_->gap_viz.viz_jitter;
            gap.convex.terminal_rdist = new_rdist + cfg_->gap_viz.viz_jitter;
            gap.mode.terminal_reduced = true;

            x1 = (gap.convex.terminal_ldist) * cos(-((float) gap.half_scan - gap.convex.terminal_lidx) / gap.half_scan * M_PI);
            y1 = (gap.convex.terminal_ldist) * sin(-((float) gap.half_scan - gap.convex.terminal_lidx) / gap.half_scan * M_PI);

            x2 = (gap.convex.terminal_rdist) * cos(-((float) gap.half_scan - gap.convex.terminal_ridx) / gap.half_scan * M_PI);
            y2 = (gap.convex.terminal_rdist) * sin(-((float) gap.half_scan - gap.convex.terminal_ridx) / gap.half_scan * M_PI);
            ROS_INFO_STREAM("post-reduce gap in polar. left: (" << gap.convex.terminal_lidx << ", " << gap.convex.terminal_ldist << "), right: (" << gap.convex.terminal_ridx << ", " << gap.convex.terminal_rdist << ")");
        }
        ROS_INFO_STREAM("post-reduce in cart. left: (" << x1 << ", " << y1 << "), right: (" << x2 << ", " << y2 << ")");
        return;
    }

    void GapManipulator::convertAxialGap(dynamic_gap::Gap& gap, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        // Return if not axial gap or disabled
        if (!gap.isAxial(initial) || !cfg_->gap_manip.axial_convert) {
            return;
        }
        ROS_INFO_STREAM("~running convertAxialGap~");

        sensor_msgs::LaserScan stored_scan_msgs = initial ? *msg.get() : dynamic_scan;
        if (stored_scan_msgs.ranges.size() != 512) {
            ROS_FATAL_STREAM("Scan range incorrect gap manip");
        }


        int lidx = initial ? gap.convex.convex_lidx : gap.convex.terminal_lidx;
        int ridx = initial ? gap.convex.convex_ridx : gap.convex.terminal_ridx;
        float ldist = initial ? gap.convex.convex_ldist : gap.convex.terminal_ldist;
        float rdist = initial ? gap.convex.convex_rdist : gap.convex.terminal_rdist;

        float x1, x2, y1, y2;
        x1 = (ldist) * cos(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);
        y1 = (ldist) * sin(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);

        x2 = (rdist) * cos(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);
        y2 = (rdist) * sin(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);

        ROS_INFO_STREAM("pre-AGC gap in polar. left: (" << lidx << ", " << ldist << ") , right: (" << ridx << ", " << rdist << ")");
        ROS_INFO_STREAM("pre-AGC gap in cart. left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")");
        
        int gap_idx_size = (ridx - lidx);
        if (gap_idx_size < 0) {
            gap_idx_size += int(2*gap.half_scan);
        }

        ROS_INFO_STREAM("gap_idx_size: " << gap_idx_size);

        bool left = gap.isLeftType(initial);
        // Extend of rotation to the radial gap 
        // amp-ed by a **small** ratio to ensure the local goal does not exactly fall on the visibility line
        // we are pivoting around the closer point?
        float rot_val = (float) std::atan2(cfg_->gap_manip.epsilon2 * cfg_->gap_manip.rot_ratio, cfg_->gap_manip.epsilon1);
        float theta = left ? (rot_val + 1e-3): -(rot_val + 1e-3);
        // ROS_INFO_STREAM("left: " << left << ", rot_val: " << rot_val);
        // ROS_INFO_STREAM("theta to pivot: " << theta);
        int near_idx, far_idx;
        float near_dist, far_dist;
        dynamic_gap::cart_model * near_model;
        
        if (left) {
            near_idx = lidx;
            far_idx = ridx;
            near_dist = ldist;
            far_dist = rdist;
            gap.pivoted_left = false;
        } else {
            near_idx = ridx;
            far_idx = lidx;
            near_dist = rdist;
            far_dist = ldist;
            gap.pivoted_left = true;
        }

        ROS_INFO_STREAM( "near point in polar: " << near_idx << ", " << near_dist << ", far point in polar: " << far_idx << ", " << far_dist);
        
        Eigen::Matrix3f rot_mat;
        // rot_mat: SE(3) matrix that represents desired rotation amount
        rot_mat << cos(theta), -sin(theta), 0,
                   sin(theta), cos(theta), 0,
                   0, 0, 1;

        auto half_num_scan = gap.half_scan;
        
        // obtaining near and far theta values from indices
        double near_theta = M_PI * (near_idx - half_num_scan) / half_num_scan;
        double far_theta = M_PI * (far_idx - half_num_scan) / half_num_scan;
        Eigen::Matrix3f near_rbt;
        // near_rbt, far_robt: SE(3) matrices that represent translation from rbt origin to far and near points
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
        int idx = int(std::floor((pivoted_theta + M_PI) / stored_scan_msgs.angle_increment));

        ROS_INFO_STREAM("idx: " << idx);

        // Rotation Completed
        // Get minimum dist range val between initial gap point and pivoted gap point
        
        // offset: original right index or the pivoted left
        int init_search_idx = left ? ridx : idx;
        // upperbound: pivoted right index or original left  
        int final_search_idx = left ? idx : lidx;
        int search_size = final_search_idx - init_search_idx;
        if (search_size < 0) {
            search_size += 2*int(gap.half_scan);
        }
        ROS_INFO_STREAM("init_search_idx: " << init_search_idx << ", final_search_idx: " << final_search_idx);

        if (final_search_idx < init_search_idx) {
            ROS_INFO_STREAM("convert axial gap for behind gap");
        }

        if (search_size < 3) {
            // Arbitrary value
            ROS_INFO_STREAM("setting discard to true");
            if (initial) {
                gap.goal.discard = true;
            } else {
                gap.terminal_goal.discard = true;
            }
            return;
        }

        // For wraparound (check to see if this happens)
        init_search_idx = std::max(init_search_idx, 0);
        final_search_idx = std::min(final_search_idx, num_of_scan - 1);

        ROS_INFO_STREAM("wrapped init_search_idx: " << init_search_idx << ", wrapped final_search_idx: " << final_search_idx);
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
        try{
            for (int i = 0; i < dist.size(); i++) {
                int check_idx = (i + init_search_idx) % int(2 * gap.half_scan);
                ROS_INFO_STREAM("ranges at idx: " << check_idx);
                float range = stored_scan_msgs.ranges.at(check_idx);
                float diff_in_idx = gap_idx_size + (search_size - i);
                dist.at(i) = sqrt(pow(near_dist, 2) + pow(range, 2) -
                    2.0 * near_dist * range * cos(diff_in_idx * stored_scan_msgs.angle_increment));
                // ROS_INFO_STREAM("checking idx: " << check_idx << ", range of: " << range << ", diff in idx: " << diff_in_idx << ", dist of " << dist.at(i));
            }
        } catch(...) {
            ROS_FATAL_STREAM("convertAxialGap outofBound");
        }

        auto farside_iter = std::min_element(dist.begin(), dist.end());
        int min_dist_idx = (init_search_idx + std::distance(dist.begin(), farside_iter)) % int(2*gap.half_scan);
        float min_dist = *farside_iter;

        ROS_INFO_STREAM("from " << init_search_idx << " to " << final_search_idx << ", min dist of " << min_dist << " at " << min_dist_idx);         

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
        idx = (int) std::floor((final_theta + M_PI) / stored_scan_msgs.angle_increment);

        // Recalculate end point location based on length
        if (initial) {
            gap.convex.convex_lidx = left ? near_idx : idx;
            gap.convex.convex_ldist = left ? near_dist : r;
            gap.convex.convex_ridx = left ? idx : near_idx;
            gap.convex.convex_rdist = left ? r : near_dist;

            x1 = (gap.convex.convex_ldist) * cos(-((float) gap.half_scan - gap.convex.convex_lidx) / gap.half_scan * M_PI);
            y1 = (gap.convex.convex_ldist) * sin(-((float) gap.half_scan - gap.convex.convex_lidx) / gap.half_scan * M_PI);
            x2 = (gap.convex.convex_rdist) * cos(-((float) gap.half_scan - gap.convex.convex_ridx) / gap.half_scan * M_PI); 
            y2 = (gap.convex.convex_rdist) * sin(-((float) gap.half_scan - gap.convex.convex_ridx) / gap.half_scan * M_PI); 
            ROS_INFO_STREAM( "post-AGC gap in polar. left: (" << gap.convex.convex_lidx << ", " << gap.convex.convex_ldist << "), right: (" << gap.convex.convex_ridx << ", " << gap.convex.convex_rdist << ")");
            gap.mode.agc = true;
        } else {
            gap.convex.terminal_lidx = left ? near_idx : idx;
            gap.convex.terminal_ldist = left ? near_dist : r;
            gap.convex.terminal_ridx = left ? idx : near_idx;
            gap.convex.terminal_rdist = left ? r : near_dist;
            
            x1 = (gap.convex.terminal_ldist) * cos(-((float) gap.half_scan - gap.convex.terminal_lidx) / gap.half_scan * M_PI);
            y1 = (gap.convex.terminal_ldist) * sin(-((float) gap.half_scan - gap.convex.terminal_lidx) / gap.half_scan * M_PI);
            x2 = (gap.convex.terminal_rdist) * cos(-((float) gap.half_scan - gap.convex.terminal_ridx) / gap.half_scan * M_PI); 
            y2 = (gap.convex.terminal_rdist) * sin(-((float) gap.half_scan - gap.convex.terminal_ridx) / gap.half_scan * M_PI); 
            ROS_INFO_STREAM( "post-AGC gap in polar. left: (" << gap.convex.terminal_lidx << ", " << gap.convex.terminal_ldist << "), right: (" << gap.convex.terminal_ridx << ", " << gap.convex.terminal_rdist << ")");
            gap.mode.terminal_agc = true;
        }
        ROS_INFO_STREAM( "post-AGC gap in cart. left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")");
    }

    void GapManipulator::radialExtendGap(dynamic_gap::Gap& gap, bool initial) { //, sensor_msgs::LaserScan const dynamic_laser_scan) {
        if (!cfg_->gap_manip.radial_extend) {
            ROS_DEBUG_STREAM_THROTTLE(1, "Radial Extension is off");
            return;
        }

        ROS_INFO_STREAM("running radialExtendGap");

        sensor_msgs::LaserScan stored_scan_msgs = initial ? *msg.get() : dynamic_scan;
        int half_num_scan = gap.half_scan; // changing this
        
        int lidx = initial ? gap.convex.convex_lidx : gap.convex.terminal_lidx;
        int ridx = initial ? gap.convex.convex_ridx : gap.convex.terminal_ridx;
        float ldist = initial ? gap.convex.convex_ldist : gap.convex.terminal_ldist;
        float rdist = initial ? gap.convex.convex_rdist : gap.convex.terminal_rdist;

        float x1, x2, y1, y2;
        x1 = (ldist) * cos(-((float) half_num_scan - lidx) / half_num_scan * M_PI);
        y1 = (ldist) * sin(-((float) half_num_scan - lidx) / half_num_scan * M_PI);

        x2 = (rdist) * cos(-((float) half_num_scan - ridx) / half_num_scan * M_PI);
        y2 = (rdist) * sin(-((float) half_num_scan - ridx) / half_num_scan * M_PI);

        ROS_INFO_STREAM( "pre-RE gap in polar. left: (" << lidx << ", " << ldist << ") , right: (" << ridx << ", " << rdist << ")");
        
        Eigen::Vector2f left_point(x1, y1);
        Eigen::Vector2f right_point(x2, y2);

        ROS_INFO_STREAM("left_point: (" << x1 << ", " << y1 << ") , right_point: (" << x2 << ", " << y2 << ")");

        Eigen::Vector2f eL = left_point / left_point.norm();
        Eigen::Vector2f eR = right_point / right_point.norm();

        ROS_INFO_STREAM("eL: (" << eL[0] << ", " << eL[1] << ") , eR: (" << eR[0] << ", " << eR[1] << ")");

        double beta_left = std::atan2(eL[1], eL[0]);
        double beta_right = std::atan2(eR[1], eR[0]);

        double det = eR[0]*eL[1] - eR[1]*eL[0];      
        double dot = eR[0]*eL[0] + eR[1]*eL[1];
        double swept_check = -std::atan2(det, dot); // this value is the angle swept clockwise from L to R (ranging from -pi to pi)
        double L_to_R_angle = swept_check;

        if (L_to_R_angle < 0) {
            L_to_R_angle += 2*M_PI; 
        }
        double beta_center = (beta_right - (L_to_R_angle / 2.0));

        // middle of gap direction
        Eigen::Vector2f eB(std::cos(beta_center), std::sin(beta_center));
        ROS_INFO_STREAM("eB: (" << eB[0] << ", " << eB[1] << ")");

        Eigen::Vector2f norm_eB = eB / eB.norm();
        // angular size of gap
        ROS_INFO_STREAM("normalized eB: " << norm_eB[0] << ", " << norm_eB[1]);
        float gap_size = beta_right - beta_left; // std::acos(eL.dot(eR));
        if (gap_size < 0) {
            gap_size += 2*M_PI;
        }
        ROS_INFO_STREAM("gap_size: " << gap_size);
        // minSafeDist is the minimum distance within the laser scan 
        float s = initial ? gap.getMinSafeDist() : gap.getTerminalMinSafeDist();
        ROS_INFO_STREAM("min safe dist: " << s);
        
        // point opposite direction of middle of gap, magnitude of min safe dist
        Eigen::Vector2f qB = -s * norm_eB;
        ROS_INFO_STREAM("qB: " << qB[0] << ", " << qB[1]);
        if (initial) {
            gap.qB = qB;
        } else {
            gap.terminal_qB = qB;
        }

        // push out left and right points
        Eigen::Vector2f qLp = left_point - qB;
        Eigen::Vector2f qRp = right_point - qB;

        // (x,y) to (r, theta) 
        Eigen::Vector2f pLp = car2pol(qLp);
        Eigen::Vector2f pRp = car2pol(qRp);

        // angular difference between right and left
        float phiB = pRp(1) - pLp(1);
        ROS_INFO_STREAM("phiB: " << phiB);

        Eigen::Vector2f pB = car2pol(-qB);
        ROS_INFO_STREAM("pB: " << pB[0] << ", " << pB[1]);

        float thL = pB(1) - gap_size / 4; // left side 1/2 throgh
        float thR = pB(1) + gap_size / 4; // right side 1/2 through

        ROS_INFO_STREAM("thL: " << thL << ", " << "thR: " << thR);
        Eigen::Vector2f pLn = pTheta(thL, phiB, pLp, pRp);
        Eigen::Vector2f pRn = pTheta(thR, phiB, pLp, pRp);

        Eigen::Vector2f qLn = pol2car(pLn) + qB;
        Eigen::Vector2f qRn = pol2car(pRn) + qB;

        // Store info back to original gap;
        Eigen::Vector2f polqLn = car2pol(qLn);
        Eigen::Vector2f polqRn = car2pol(qRn);

        double first_new_left_idx = polqLn(1) / M_PI * half_num_scan + half_num_scan;
        double first_new_right_idx = polqRn(1) / M_PI * half_num_scan + half_num_scan;

        ROS_INFO_STREAM("new_l_theta: " << polqLn(1) << ", new_r_theta: " << polqRn(1));
                
        int new_left_idx = std::max((int) std::floor(first_new_left_idx), 0);
        int new_right_idx = std::min((int) std::ceil(first_new_right_idx), 511);

        // ROS_INFO_STREAM("int values " << new_left_idx << ", " << new_right_idx);
        
        /*
        if (new_left_idx == new_right_idx) {
            ROS_INFO_STREAM("manipulated indices are same");
            new_right_idx++;
        }
        */

        // ROS_INFO_STREAM("new_left_idx: " << new_left_idx << ", new_right_idx: " << new_right_idx);

        if (new_left_idx == new_right_idx) {
            // ROS_INFO_STREAM( "post-RE, indices are the same");
            new_right_idx += 1;
        }

        if (initial) {
            gap.convex.convex_lidx = new_left_idx;
            gap.convex.convex_ridx = new_right_idx;
            gap.convex.convex_ldist = polqLn(0);
            gap.convex.convex_rdist = polqRn(0);
            gap.mode.convex = true;
            x1 = (gap.convex.convex_ldist) * cos(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
            y1 = (gap.convex.convex_ldist) * sin(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
            x2 = (gap.convex.convex_rdist) * cos(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
            y2 = (gap.convex.convex_rdist) * sin(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
            ROS_INFO_STREAM( "post-RE gap in polar, left: (" << gap.convex.convex_lidx << ", " << gap.convex.convex_ldist << "), right: (" << gap.convex.convex_ridx << ", " << gap.convex.convex_rdist << ")");
        } else {
            gap.convex.terminal_lidx = new_left_idx;
            gap.convex.terminal_ridx = new_right_idx;
            gap.convex.terminal_ldist = polqLn(0);
            gap.convex.terminal_rdist = polqRn(0);
            gap.mode.terminal_convex = true;
            x1 = (gap.convex.terminal_ldist) * cos(-((float) half_num_scan - gap.convex.terminal_lidx) / half_num_scan * M_PI);
            y1 = (gap.convex.terminal_ldist) * sin(-((float) half_num_scan - gap.convex.terminal_lidx) / half_num_scan * M_PI);
            x2 = (gap.convex.terminal_rdist) * cos(-((float) half_num_scan - gap.convex.terminal_ridx) / half_num_scan * M_PI);
            y2 = (gap.convex.terminal_rdist) * sin(-((float) half_num_scan - gap.convex.terminal_ridx) / half_num_scan * M_PI);
            ROS_INFO_STREAM( "post-RE gap in polar. left: (" << gap.convex.terminal_lidx << ", " << gap.convex.terminal_ldist << "), right: (" << gap.convex.terminal_ridx << ", " << gap.convex.terminal_rdist << ")");
        }
        ROS_INFO_STREAM( "post-RE gap in cart. left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")");

        // ROS_INFO_STREAM( "after radial extension:" << std::endl;

        // ROS_INFO_STREAM( "qB: " << qB[0] << ", " << qB[1] << std::endl;

        // points are not actually extended here, qB is just calculated.
        // points are moved though, might as well do it here
        
        /*
        gap.left_model->inflate_model(x1, y1);
        gap.right_model->inflate_model(x2, y2);

        Matrix<double, 4, 1> left_cartesian_state = gap.left_model->get_cartesian_state();
        Matrix<double, 4, 1> right_cartesian_state = gap.right_model->get_cartesian_state();
        ROS_INFO_STREAM( "inflated models in cart, left: (" << left_cartesian_state[0] << ", " << left_cartesian_state[1] << ", " << left_cartesian_state[2] << ", " << left_cartesian_state[3] << "), ";
        ROS_INFO_STREAM( "right: (" << right_cartesian_state[0] << ", " << right_cartesian_state[1] << ", " << right_cartesian_state[2] << ", " << right_cartesian_state[3] << ")" << std::endl;
        */
        // we will see this in trajectory generator
        //gap.left_model->extend_model_origin(qB);
        //gap.right_model->extend_model_origin(qB);
        
        //ROS_DEBUG_STREAM("l: " << gap._left_idx << " to " << gap.convex_lidx << ", r: " << gap._right_idx << " to " << gap.convex_ridx);
        //ROS_DEBUG_STREAM("ldist: " << gap._ldist << " to " << gap.convex_ldist << ", rdist: " << gap._rdist << " to " << gap.convex_rdist);

        return;
    }

    void GapManipulator::inflateGapSides(dynamic_gap::Gap& gap, bool initial) {
        // get points
        int lidx = initial ? gap.convex.convex_lidx : gap.convex.terminal_lidx;
        int ridx = initial ? gap.convex.convex_ridx : gap.convex.terminal_ridx;
        float ldist = initial ? gap.convex.convex_ldist : gap.convex.terminal_ldist;
        float rdist = initial ? gap.convex.convex_rdist : gap.convex.terminal_rdist;

        float x1, x2, y1, y2;
        x1 = (ldist) * cos(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);
        y1 = (ldist) * sin(-((float) gap.half_scan - lidx) / gap.half_scan * M_PI);

        x2 = (rdist) * cos(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);
        y2 = (rdist) * sin(-((float) gap.half_scan - ridx) / gap.half_scan * M_PI);
        Eigen::Vector2f pl(x1, y1);
        Eigen::Vector2f pr(x2, y2);

        ROS_INFO_STREAM( "pre-inflate gap in polar. left: (" << lidx << ", " << ldist << "), right: (" << ridx << ", " << rdist << ")");
        ROS_INFO_STREAM( "pre-inflate gap in cart. left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")");
        
        // inflate inwards by radius * infl
        // rotate by pi/2, norm
        Eigen::Matrix2f r_negpi2;
        r_negpi2 << 0,1,
                   -1,0;

        // pivot "left" point by -r_negpi2
        // Eigen::Vector2f left_angular_inflate = -r_negpi2 * pl / pl.norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        // pivot "right" point by r_negpi2
        // Eigen::Vector2f right_angular_inflate = r_negpi2 * pr / pr.norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

        // inflate upward by radius * infl
        Eigen::Vector2f left_radial_inflate = pl / pl.norm() * 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
        Eigen::Vector2f right_radial_inflate = pr / pr.norm() * 2 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;

        Eigen::Vector2f new_l = pl + left_radial_inflate;
        Eigen::Vector2f new_r = pr + right_radial_inflate;

        // float new_l_theta = std::atan2(new_l[1], new_l[0]);
        // float new_r_theta = std::atan2(new_r[1], new_r[0]);
        int new_l_idx = lidx; // std::max((int) std::floor(gap.half_scan * new_l_theta / M_PI + gap.half_scan), 0);
        int new_r_idx = ridx; // std::min((int) std::ceil(gap.half_scan * new_r_theta / M_PI + gap.half_scan), 511);
        // ROS_INFO_STREAM("new_l_theta: " << new_l_theta << ", new_r_theta: " << new_r_theta);
        ROS_INFO_STREAM("int values, left: " << new_l_idx << ", right: " << new_r_idx);
        
        if (new_l_idx > new_r_idx) {
            return;
        }

        if (new_l_idx == new_r_idx) {
            ROS_INFO_STREAM("manipulated indices are same");
            new_r_idx++;
        }

        sensor_msgs::LaserScan stored_scan_msgs = initial ? *msg.get() : dynamic_scan;
        float new_l_range = std::min(new_l.norm(), stored_scan_msgs.ranges.at(new_l_idx)); // should this be ranges.at - r_infl? 
        float new_r_range = std::min(new_r.norm(), stored_scan_msgs.ranges.at(new_r_idx));



        if (initial) {
            gap.convex.convex_lidx = new_l_idx;
            gap.convex.convex_ridx = new_r_idx;
            gap.convex.convex_ldist = new_l_range;
            gap.convex.convex_rdist = new_r_range;
            
            x1 = (gap.convex.convex_ldist) * cos(-((float) gap.half_scan - gap.convex.convex_lidx) / gap.half_scan * M_PI);
            y1 = (gap.convex.convex_ldist) * sin(-((float) gap.half_scan - gap.convex.convex_lidx) / gap.half_scan * M_PI);
            x2 = (gap.convex.convex_rdist) * cos(-((float) gap.half_scan - gap.convex.convex_ridx) / gap.half_scan * M_PI); 
            y2 = (gap.convex.convex_rdist) * sin(-((float) gap.half_scan - gap.convex.convex_ridx) / gap.half_scan * M_PI); 
            ROS_INFO_STREAM( "post-inflate gap in polar. left: (" << gap.convex.convex_lidx << ", " << gap.convex.convex_ldist << "), right: (" << gap.convex.convex_ridx << ", " << gap.convex.convex_rdist << ")");
        } else {
            gap.convex.terminal_lidx = new_l_idx;
            gap.convex.terminal_ridx = new_r_idx;
            gap.convex.terminal_ldist = new_l_range;
            gap.convex.terminal_rdist = new_r_range;
            
            x1 = (gap.convex.terminal_ldist) * cos(-((float) gap.half_scan - gap.convex.terminal_lidx) / gap.half_scan * M_PI);
            y1 = (gap.convex.terminal_ldist) * sin(-((float) gap.half_scan - gap.convex.terminal_lidx) / gap.half_scan * M_PI);
            x2 = (gap.convex.terminal_rdist) * cos(-((float) gap.half_scan - gap.convex.terminal_ridx) / gap.half_scan * M_PI); 
            y2 = (gap.convex.terminal_rdist) * sin(-((float) gap.half_scan - gap.convex.terminal_ridx) / gap.half_scan * M_PI); 
            ROS_INFO_STREAM( "post-inflate gap in polar. left: (" << gap.convex.terminal_lidx << ", " << gap.convex.terminal_ldist << "), right: (" << gap.convex.terminal_ridx << ", " << gap.convex.terminal_rdist << ")");
        }

        ROS_INFO_STREAM( "post-inflate gap in cart. left: (" << x1 << ", " << y1 << ") , right: (" << x2 << ", " << y2 << ")");
    }

    Eigen::Vector2f GapManipulator::car2pol(Eigen::Vector2f a) {
        return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    }

    Eigen::Vector2f GapManipulator::pol2car(Eigen::Vector2f a) {
        return Eigen::Vector2f(cos(a(1)) * a(0), sin(a(1)) * a(0));
    }

    Eigen::Vector2f GapManipulator::pTheta(
        float th, float phiB, Eigen::Vector2f pLp, Eigen::Vector2f pRp) {
        return pRp * (th - pLp(1)) / phiB + pLp * (pRp(1) - th) / phiB;
    }

}
