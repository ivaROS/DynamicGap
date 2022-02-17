#include <dynamic_gap/gap_manip.h>

namespace dynamic_gap {
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egolock);
        msg = msg_;
        num_of_scan = (int)(msg.get()->ranges.size());
    }

    bool GapManipulator::indivGapFeasibilityCheck(dynamic_gap::Gap& gap) {
        // std::cout << "FEASIBILITY CHECK" << std::endl;
        bool feasible;
        auto half_num_scan = gap.half_scan;
        float x1, x2, y1, y2;
        int mid_idx;
        x1 = (gap.convex.convex_ldist) * cos(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (gap.convex.convex_ldist) * sin(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);

        x2 = (gap.convex.convex_rdist) * cos(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (gap.convex.convex_rdist) * sin(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
       
        Eigen::Vector2f pl(x1, y1);
        Eigen::Vector2f pr(x2, y2);
        Eigen::Vector2f pg = (pl + pr) / 2.0;
        // I don't think this is guaranteed
        dynamic_gap::MP_model* left_model;
        dynamic_gap::MP_model* right_model;
        determineLeftRightModels(&left_model, &right_model, gap, pg);
        std::cout << "x1, y1: (" << x1 << ", " << y1 << "), x2,y2: (" << x2 << ", " << y2 << ")" << std::endl; 
        auto left_ori = gap.convex.convex_lidx * msg.get()->angle_increment + msg.get()->angle_min;
        auto right_ori = gap.convex.convex_ridx * msg.get()->angle_increment + msg.get()->angle_min;
        // TODO: make sure that this is always less than 180, make sure convex
        Matrix<double, 5, 1> left_model_state = left_model->get_state();
        Matrix<double, 5, 1> right_model_state = right_model->get_state();
        Eigen::Vector4d left_cartesian_state = left_model->get_cartesian_state();
        Eigen::Vector4d right_cartesian_state = right_model->get_cartesian_state();
        Matrix<double, 2, 1> v_ego = left_model->get_v_ego();
        std::cout << "left cartesian: " << left_cartesian_state[0] << ", " << left_cartesian_state[1] << ", " << left_cartesian_state[2] << ", " << left_cartesian_state[3] << std::endl;
        std::cout << "right cartesian: " << right_cartesian_state[0] << ", " << right_cartesian_state[1] << ", " << right_cartesian_state[2] << ", " << right_cartesian_state[3] << std::endl;
        std::cout << "v_ego: " << v_ego[0] << ", " << v_ego[1] << std::endl;
        Matrix<double, 2, 1> gap_target_left_vel(left_cartesian_state[2] + v_ego[0], left_cartesian_state[3] + v_ego[1]);
        Matrix<double, 2, 1> gap_target_right_vel(right_cartesian_state[2] + v_ego[0], right_cartesian_state[3] + v_ego[1]);
        std::cout << "gap_target_left_vel: " << gap_target_left_vel[0] << ", " << gap_target_left_vel[1] << std::endl;
        std::cout << "gap_target_right_vel: " << gap_target_right_vel[0] << ", " << gap_target_right_vel[1] << std::endl;
        double gap_phys_betadot_left = (left_cartesian_state[0]*gap_target_left_vel[1] - left_cartesian_state[1]*gap_target_left_vel[0])*(pow(left_model_state[0],2));
        double gap_phys_betadot_right = (right_cartesian_state[0]*gap_target_right_vel[1] - right_cartesian_state[1]*gap_target_right_vel[0])*(pow(right_model_state[0],2));
        std::cout << "gap_phys_betadot_left: " << gap_phys_betadot_left << std::endl;
        std::cout << "gap_phys_betadot_right: " << gap_phys_betadot_right << std::endl;

        double gap_angle;
        if (right_ori > left_ori) {
            gap_angle = right_ori - left_ori;
        } else {
            gap_angle = left_ori - right_ori;
        }
        
        //std::cout << "left idx: " << gap.convex.convex_lidx << ", right idx: " << gap.convex.convex_ridx << std::endl;
        //std::cout << "left ori: " << left_ori << ", right ori: " << right_ori << ", gap angle: " << gap_angle << std::endl;

        //std::cout << " gap convex l_idx: " << gap.convex.convex_lidx << ", gap convex r_idx: " << gap.convex.convex_ridx << std::endl;

        // gap indices always return convex. Right index always greater than left index. Does that mean gaps are all swapped?
        std::cout << "y_left: " << left_model_state(0) << ", " << left_model_state(1) << ", " << left_model_state(2) << ", " << left_model_state(3) << ", " << left_model_state(4) << std::endl;
        std::cout << "y_right: " << right_model_state(0) << ", " << right_model_state(1) << ", " << right_model_state(2) << ", " << right_model_state(3) << ", " << right_model_state(4) << std::endl; 
        std::cout << "corrected betadot left: " << gap_phys_betadot_left << ", corrected betadot right: " << gap_phys_betadot_right << std::endl;

        double nom_rbt_vel = 0.25;
        double T_rbt2arc = std::max(gap.convex.convex_ldist, gap.convex.convex_rdist) / nom_rbt_vel;

        double left_betadot_check = gap_phys_betadot_left;
        double right_betadot_check = gap_phys_betadot_right;
        // FEASIBILITY CHECK
        if ((left_betadot_check >= 0  && right_betadot_check > 0) || (left_betadot_check <= 0  && right_betadot_check < 0 )) {
            // CATEGORY 1: TRANSLATING       
            std::cout << "translating gap" << std::endl;
            if (left_betadot_check - right_betadot_check > 0) {
                std::cout << "gap is expanding on the whole, is feasible" << std::endl;
                gap.gap_lifespan = cfg_->traj.integrate_maxt;
                feasible = true;
            } else {
                gap.gap_lifespan = gap_angle / (right_betadot_check - left_betadot_check);  
                if (gap.gap_lifespan > T_rbt2arc) {
                    std::cout << "gap lifespan longer than Trbt2arc, is feasible" << std::endl;
                    feasible = true;
                } else {
                    std::cout << "gap lifespan shorter than Trbt2arc, not feasible" << std::endl;
                    feasible = false;
                }
            }
        } else if (left_betadot_check <= 0 && right_betadot_check >= 0)  {
            // CATEGORY 2: STATIC/CLOSING
            gap.gap_lifespan = gap_angle / (right_betadot_check - left_betadot_check);
            std::cout << "closing gap" << std::endl;
            if (gap.gap_lifespan > T_rbt2arc) {
                std::cout << "gap lifespan longer than Trbt2arc, is feasible" << std::endl;
                feasible = true;
            } else {
                std::cout << "gap lifespan shorter than Trbt2arc, not feasible" << std::endl;
                feasible = false;
            }
        } else {
            // CATEGORY 3: EXPANDING
            std::cout << "expanding gap, is feasible" << std::endl;
            gap.gap_lifespan = cfg_->traj.integrate_maxt;
            feasible = true;
        }

        return feasible;
    }
    
    void GapManipulator::feasibilityCheck(std::vector<dynamic_gap::Gap>& manip_set) {
        bool gap_i_feasible;
        int num_gaps = manip_set.size();
        for (size_t i = 0; i < num_gaps; i++) {
            std::cout << "feasiblity check for gap " << i << std::endl;
            gap_i_feasible = indivGapFeasibilityCheck(manip_set.at(i));
            if (!gap_i_feasible) {
                std::cout << "deleting gap" << std::endl;
                manip_set.erase(manip_set.begin() + i);
                num_gaps = manip_set.size();
                i--;
            }
        }
    }
    
    void GapManipulator::determineLeftRightModels(dynamic_gap::MP_model** left_model, dynamic_gap::MP_model** right_model, dynamic_gap::Gap& selectedGap, Eigen::Vector2f pg) {
        Matrix<double, 5, 1> model_one = selectedGap.left_model->get_state();
        //std::cout << "nominal left model: " << model_one[0] << ", " << model_one[1] << ", " << model_one[2] << std::endl;
        Matrix<double, 5, 1> model_two = selectedGap.right_model->get_state();
        //std::cout << "nominal right model: " << model_two[0] << ", " << model_two[1] << ", " << model_two[2] << std::endl;
        
        // are these the correct angles to check?
        double angle_one = atan2(model_one[1], model_one[2]);
        double angle_two = atan2(model_two[1], model_two[2]);
        double angle_goal = atan2(-pg[0], pg[1]);
        // if all three are positive or negative
        std::cout << "angle_one: " << angle_one << ", angle_two: " << angle_two << ", angle_goal: " << angle_goal << std::endl;
        //std::cout << "picking gap sides" << std::endl;
        // seems to only do 2,7,9
        if ((angle_one > 0 && angle_two > 0 && angle_goal > 0) ||
            (angle_one < 0 && angle_two < 0 && angle_goal < 0)) {
            if (angle_one > angle_two) {
                std::cout << "1" << std::endl;
                *left_model = selectedGap.left_model;
                *right_model = selectedGap.right_model;
            } else {
                std::cout << "2" << std::endl;
                *left_model = selectedGap.right_model;
                *right_model = selectedGap.left_model;
            }
        } else if ((angle_one > 0 && angle_two > 0) || (angle_one < 0 && angle_two < 0)) {
            if (angle_one > angle_two) {
                std::cout << "3" << std::endl;
                *right_model = selectedGap.left_model;
                *left_model = selectedGap.right_model;
            } else {
                std::cout << "4" << std::endl;
                *right_model = selectedGap.right_model;
                *left_model = selectedGap.left_model;
            }
        } else if (angle_one > 0 && angle_goal > 0 && angle_two < 0) {
            if (angle_goal < angle_one) {
                std::cout << "5" << std::endl;
                *left_model = selectedGap.left_model;
                *right_model = selectedGap.right_model;
            } else {
                std::cout << "6" << std::endl;
                *left_model = selectedGap.right_model;
                *right_model = selectedGap.left_model;
            }
        } else if (angle_two > 0 && angle_goal > 0 && angle_one < 0) {
            if (angle_goal < angle_two) {
                std::cout << "7" << std::endl;
                *left_model = selectedGap.right_model;
                *right_model = selectedGap.left_model;
            } else {
                std::cout << "8" << std::endl;
                *left_model = selectedGap.left_model;
                *right_model = selectedGap.right_model;
            }
        } else if (angle_one < 0 && angle_goal < 0 && angle_two > 0) {
            if (angle_goal > angle_one) {
                std::cout << "9" << std::endl;
                *right_model = selectedGap.left_model;
                *left_model = selectedGap.right_model;
            } else {
                std::cout << "10" << std::endl;
                *left_model = selectedGap.left_model;
                *right_model = selectedGap.right_model;
            }
        } else if (angle_two < 0 && angle_goal < 0 && angle_one > 0) {
            if (angle_goal > angle_two) {
                std::cout << "11" << std::endl;
                *right_model = selectedGap.right_model;
                *left_model = selectedGap.left_model;
            } else {
                std::cout << "12" << std::endl;
                *left_model = selectedGap.right_model;
                *right_model = selectedGap.left_model;
            }
        }
        return;
    }

    void GapManipulator::setGapGoal(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal) {
        std::cout << "in setGapGoal" << std::endl;
        auto half_num_scan = gap.half_scan;
        float x1, x2, y1, y2;
        int mid_idx;
        x1 = (gap.convex.convex_ldist) * cos(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (gap.convex.convex_ldist) * sin(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);

        x2 = (gap.convex.convex_rdist) * cos(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (gap.convex.convex_rdist) * sin(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
       
        Eigen::Vector2f pl(x1, y1);
        Eigen::Vector2f pr(x2, y2);
        Eigen::Vector2f pg = (pl + pr) / 2.0;

        // LEFT AND RIGHT ORI USE THE LIDAR INDEX L/R WHICH IS FLIPPED FROM MODEL L/R
        auto left_ori = gap.convex.convex_lidx * msg.get()->angle_increment + msg.get()->angle_min;
        auto right_ori = gap.convex.convex_ridx * msg.get()->angle_increment + msg.get()->angle_min;
        
        double nom_rbt_vel = 0.25; // 1/2 of max vel?
        double T_rbt2arc = std::max(gap.convex.convex_ldist, gap.convex.convex_rdist) / nom_rbt_vel;

        // I don't think this is guaranteed
        dynamic_gap::MP_model* left_model;
        dynamic_gap::MP_model* right_model;
        determineLeftRightModels(&left_model, &right_model, gap, pg);

        double swept_left_ori = 0.0;
        double swept_right_ori = 0.0;
        
        Matrix<double, 1, 5> left_model_state = left_model->get_state();
        Matrix<double, 1, 5> right_model_state = right_model->get_state();

        // FEASIBILITY CHECK
        // NOTE: left/right model states are flipped from left/right indices
        if ((left_model_state[4] >= 0  && right_model_state[4] > 0) || (left_model_state[4] <= 0  && right_model_state[4] < 0 )) {
            // CATEGORY 1: TRANSLATING       
            std::cout << "gap is translating" << std::endl;
            // FLIPPING GOING ON HERE
            double left_swept = right_model_state[4]*T_rbt2arc; // in radians
            double right_swept = left_model_state[4]*T_rbt2arc; // in radians
            swept_left_ori = left_ori + left_swept;
            swept_right_ori = right_ori + right_swept;
            int swept_left_idx = (swept_left_ori - msg.get()->angle_min) / msg.get()->angle_increment;
            int swept_right_idx = (swept_right_ori - msg.get()->angle_min) / msg.get()->angle_increment;
            //std::cout << "original swept left idx: " << swept_left_idx << ", original swept right idx: " << swept_right_idx << std::endl;
            //std::cout << "convex lidx: " << gap.convex.convex_lidx << ", convex ridx: " << gap.convex.convex_ridx << std::endl;
            int count = 0;
            if (swept_left_idx > gap.convex.convex_ridx || swept_right_idx < gap.convex.convex_lidx) {
                // if swept left idx crosses original right idx or swept right idx crosses left idx: no valid slice
                // std::cout << "no valid slice, halving time" << std::endl;
                gap.swept_convex_ldist = gap.convex.convex_ldist;
                gap.swept_convex_rdist = gap.convex.convex_rdist;
                gap.swept_convex_lidx = gap.convex.convex_lidx;
                gap.swept_convex_ridx = gap.convex.convex_ridx;
                gap.no_valid_slice = true;
            } else {
                // if valid slice exists, put goal in there
                gap.swept_convex_ldist = gap.convex.convex_ldist;
                gap.swept_convex_rdist = gap.convex.convex_rdist;
                gap.swept_convex_lidx = std::max(swept_left_idx, gap.convex.convex_lidx); // taking max to keep within gap at T=0
                gap.swept_convex_ridx = std::min(swept_right_idx, gap.convex.convex_ridx); // taking min to keep within gap at T=0
            }
        } else if (left_model_state[4] <= 0 && right_model_state[4] >= 0)  {
            // CATEGORY 2: CLOSING
            // std::cout << "gap is closing" << std::endl;
            // FLIPPING GOING ON HERE
            double left_swept = right_model_state[4]*T_rbt2arc; // in radians
            double right_swept = left_model_state[4]*T_rbt2arc; // in radians
            swept_left_ori = left_ori + left_swept;
            swept_right_ori = right_ori + right_swept;
            int swept_left_idx = (swept_left_ori - msg.get()->angle_min) / msg.get()->angle_increment;
            int swept_right_idx = (swept_right_ori - msg.get()->angle_min) / msg.get()->angle_increment;
            // set valid gap between these
            gap.swept_convex_ldist = gap.convex.convex_ldist;
            gap.swept_convex_rdist = gap.convex.convex_rdist;
            gap.swept_convex_lidx = swept_left_idx;
            gap.swept_convex_ridx = swept_right_idx;
        } else {
            // CATEGORY 3: STATIC/EXPANDING
            // std::cout << "gap is static or expanding" << std::endl;
            gap.swept_convex_ldist = gap.convex.convex_ldist;
            gap.swept_convex_rdist = gap.convex.convex_rdist;
            gap.swept_convex_lidx = gap.convex.convex_lidx;
            gap.swept_convex_ridx = gap.convex.convex_ridx;
        }

        // do we need wrapping?
        while (gap.swept_convex_lidx < 0) {
            gap.swept_convex_lidx += 2*gap.half_scan;
        }
        while (gap.swept_convex_lidx > 512) {
            gap.swept_convex_lidx -= 2*gap.half_scan;
        }
        while (gap.swept_convex_ridx < 0) {
            gap.swept_convex_ridx += 2*gap.half_scan;
        }
        while (gap.swept_convex_ridx > 512) {
            gap.swept_convex_ridx -= 2*gap.half_scan;
        }

        std::cout << "slice indices" << std::endl;
        std::cout << "left idx: " << gap.convex.convex_lidx << ", right idx: " << gap.convex.convex_ridx << std::endl;
        std::cout << "swept left idx: " << gap.swept_convex_lidx << ", swept right idx: " << gap.swept_convex_ridx << std::endl;

        setValidSliceWaypoint(gap, localgoal);
    }
    
    // at this point, all gaps are feasible
    void GapManipulator::setValidSliceWaypoint(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal){
        // std::cout << "in setValidSliceWaypoint" << std::endl;
        auto half_num_scan = gap.half_scan;
        float x1, x2, y1, y2;
        int mid_idx;
        x1 = (gap.swept_convex_ldist) * cos(-((float) half_num_scan - gap.swept_convex_lidx) / half_num_scan * M_PI);
        y1 = (gap.swept_convex_ldist) * sin(-((float) half_num_scan - gap.swept_convex_lidx) / half_num_scan * M_PI);

        x2 = (gap.swept_convex_rdist) * cos(-((float) half_num_scan - gap.swept_convex_ridx) / half_num_scan * M_PI);
        y2 = (gap.swept_convex_rdist) * sin(-((float) half_num_scan - gap.swept_convex_ridx) / half_num_scan * M_PI);
       
        Eigen::Vector2f pl(x1, y1);
        Eigen::Vector2f pr(x2, y2);
        std::cout << "local goal: " << localgoal.pose.position.x << ", " << localgoal.pose.position.y << std::endl;
        std::cout << "x1: " << x1 << ", y1: " << y1 << ". x2: " << x2 << ", y2: " << y2 << std::endl;
        // I don't think this is guaranteed
        auto left_ori = gap.swept_convex_lidx * msg.get()->angle_increment + msg.get()->angle_min;
        auto right_ori = gap.swept_convex_ridx * msg.get()->angle_increment + msg.get()->angle_min;
        // TODO: make sure that this is always less than 180, make sure convex
        double gap_angle = right_ori - left_ori;

        //double closing_rate = right_model_state[4] - left_model_state[4];
        //double time_to_pass = gap_angle / closing_rate;
        //std::cout << "time to pass: " time_to_pass << ", rbt time to arc: " << rbt_time_to_arc << std::endl;
        
        // need to check if gap is feasible. If not, just set gap goal to 0
        
        // if agc (AXIAL GAP CONVERSION). then the shorter side need to be further in
        auto lf = (pr - pl) / (pr - pl).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pl;
        auto thetalf = car2pol(lf)(1);
        auto lr = (pl - pr) / (pl - pr).norm() * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + pr;
        auto thetalr = car2pol(lr)(1);
        
        // Second condition: if angle smaller than M_PI / 3
        // Check if arc length < 3 robot width
        bool gap_size_check = right_ori - left_ori < M_PI;
        float dist = 0;
        bool small_gap = false;
        if (gap_size_check && !cfg_->planning.planning_inflated) {
            // if smaller than M_PI/3
            dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
            small_gap = dist < 4 * cfg_->rbt.r_inscr;
        }

        // no chance it can be non-convex here
        std::cout << "thetalr: " << thetalr << ", thetalf: " << thetalf << std::endl;
        // taking out thetalr < thetalf ||
        if (small_gap) {
            gap.goal.x = (x1 + x2) / 2;
            gap.goal.y = (y1 + y2) / 2;
            std::cout << "first case: " << gap.goal.x << ", " << gap.goal.y << std::endl;
            // gap.goal.discard = thetalr < thetalf;
            // std::cout << "setting discard to " << (thetalr < thetalf) << std::endl;
            gap.goal.set = true;
            return;
        }

        // if localgoal is within gap, just put it there
        if (checkGoalVisibility(localgoal)) {
            gap.goal.x = localgoal.pose.position.x;
            gap.goal.y = localgoal.pose.position.y;
            std::cout << "second case: " << gap.goal.x << ", " << gap.goal.y << std::endl;
            gap.goal.set = true;
            gap.goal.goalwithin = true;
            return;
        }
        
        float goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        std::cout << "goal orientation: " << goal_orientation << std::endl;
        // confined_theta: confining this value to within angular space of gap, bias it to be closer to local goal
        // potential problem: can local goal be within angular space of gap here?
        float confined_theta; // = std::min(thetalr, std::max(thetalf, goal_orientation));
        float lr_ang_diff = std::abs(thetalr - goal_orientation);
        float lf_ang_diff = std::abs(thetalf - goal_orientation);
        if (lr_ang_diff < lf_ang_diff) {
            confined_theta = thetalr;
        } else {
            confined_theta = thetalf;
        }

        // like convex combination, interpolating from l_dist to r_dist 
        float confined_r = (gap.swept_convex_rdist - gap.swept_convex_ldist) * (confined_theta - thetalf) / (thetalr - thetalf) + gap.swept_convex_ldist;

        float xg = confined_r * cos(confined_theta);
        float yg = confined_r * sin(confined_theta);
        // anchor is then on arc of gap, within angular space
        Eigen::Vector2f anchor(xg, yg);

        double anchor_orientation = std::atan2(yg, xg);
        double anchor_idx = std::floor(anchor_orientation*half_num_scan/M_PI + half_num_scan);
        std::cout << "anchor idx: " << anchor_idx << ", anchor: " << xg << ", " << yg << std::endl;
        Eigen::Matrix2f r_negpi2;
            r_negpi2 << 0,1,-1,0;
        auto offset = r_negpi2 * (pr - pl);
        // goal point is offset so that it's slightly beyond gap, biases it to be more in the middle of the gap
        auto goal_pt = offset * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio + anchor;

        gap.goal.x = goal_pt(0);
        gap.goal.y = goal_pt(1);
        gap.goal.set = true;


        // THIRD CASE WILL PUSH IT OUT. anchor is always fine. offset pushes it out. maybe bound to edges of valid slice
        double gap_goal_orientation = std::atan2(gap.goal.y, gap.goal.x);
        double gap_goal_idx = std::floor(gap_goal_orientation*half_num_scan/M_PI + half_num_scan);
        std::cout << "gap goal idx: " << gap_goal_idx << std::endl;

        if (gap.swept_convex_lidx < gap.swept_convex_rdist) {
            if (!(gap.swept_convex_lidx <= gap_goal_idx && gap_goal_idx <= gap.swept_convex_ridx)) {
                std::cout << "gap goal outside gap" << std::endl;
                gap.goal.x = anchor(0);
                gap.goal.y = anchor(1);
            }
        } else {
            if (gap_goal_idx <= 2*gap.half_scan) {
                if (!(gap.swept_convex_lidx <= gap_goal_idx && gap_goal_idx <= (gap.swept_convex_ridx + 2*gap.half_scan))) {
                    std::cout << "gap goal outside gap" << std::endl;
                    gap.goal.x = anchor(0);
                    gap.goal.y = anchor(1);
                }
            } else {
                if (!(gap.swept_convex_lidx <= (gap_goal_idx + 2*gap.half_scan) && (gap_goal_idx + 2*gap.half_scan) <= (gap.swept_convex_ridx + 2*gap.half_scan))) {
                    std::cout << "gap goal outside gap" << std::endl;
                    gap.goal.x = anchor(0);
                    gap.goal.y = anchor(1);
                }
            }

        }
        std::cout << "third case: " << gap.goal.x << ", " << gap.goal.y << std::endl;
        // need to check again if gap goals being placed outside gaps
        return;
    }

    bool GapManipulator::checkGoalVisibility(geometry_msgs::PoseStamped localgoal) {
        boost::mutex::scoped_lock lock(egolock);
        // with robot as 0,0 (localgoal in robot frame as well)
        double dist2goal = sqrt(pow(localgoal.pose.position.x, 2) + pow(localgoal.pose.position.y, 2));

        auto scan = *msg.get();
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
        int incident_angle = (int) round((goal_angle - scan.angle_min) / scan.angle_increment);

        double half_angle = std::asin(cfg_->rbt.r_inscr / dist2goal);
        // int index = std::ceil(half_angle / scan.angle_increment) * 1.5;
        int index = (int)(scan.ranges.size()) / 8;
        int lower_bound = std::max(incident_angle - index, 0);
        int upper_bound = std::min(incident_angle + index, int(scan.ranges.size() - 1));
        auto min_val_round_goal = *std::min_element(scan.ranges.begin() + lower_bound, scan.ranges.begin() + upper_bound);
        return dist2goal < min_val_round_goal;
    }

    // In place modification
    void GapManipulator::reduceGap(dynamic_gap::Gap& gap, geometry_msgs::PoseStamped localgoal) {
        int lidx = gap.LIdx();
        int ridx = gap.RIdx();
        if (!msg) return; 
        // is this truthful to non-convex situations?
        // msg is from egocircle
        double angular_size = (ridx - lidx) * (msg.get()->angle_increment);
        // std::cout << "lidx: " << lidx << ", ridx: " << ridx << std::endl;
        // std::cout << "angular size: " << angular_size << std::endl;

        // also pi
        if (angular_size < cfg_->gap_manip.reduction_threshold){
            return;
        }

        // the desired size for the reduced gap?
        // target is pi
        int gap_size = cfg_->gap_manip.reduction_target / msg.get()->angle_increment;
        int l_biased_r = lidx + gap_size;
        int r_biased_l = ridx - gap_size;
        // std::cout << "l_biased_r: " << l_biased_r << ", r_biased_l: " << r_biased_l << std::endl;
        double goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_idx = goal_orientation / (M_PI / (num_of_scan / 2)) + (num_of_scan / 2);

        int acceptable_dist = gap_size / 2; // distance in scan indices

        int new_l, new_r;
        if (goal_idx + acceptable_dist > ridx){
            // r-biased Gap
            new_r = ridx;
            new_l = r_biased_l;
        } else if (goal_idx - acceptable_dist < lidx) {
            // l-biased gap
            new_r = l_biased_r;
            new_l = lidx;
        } else {
            // Lingering in center
            new_l = goal_idx - acceptable_dist;
            new_r = goal_idx + acceptable_dist;
        }

 
        float ldist = gap.LDist();
        float rdist = gap.RDist();
        float new_ldist = float(new_l - lidx) / float(ridx - lidx) * (rdist - ldist) + ldist;
        float new_rdist = float(new_r - lidx) / float(ridx - lidx) * (rdist - ldist) + ldist;
        //std::cout << "new_l: " << new_l << ", new_r: " << new_r << std::endl;
        // std::cout << "new_l dist: " << new_ldist + cfg_->gap_viz.viz_jitter << ", new_r dist: " << new_rdist + cfg_->gap_viz.viz_jitter << std::endl;
        //std::cout << "new angular size: " << (new_r - new_l) * (msg.get()->angle_increment) << std::endl;
        gap.convex.convex_lidx = new_l;
        gap.convex.convex_ridx = new_r;
        gap.convex.convex_ldist = new_ldist + cfg_->gap_viz.viz_jitter;
        gap.convex.convex_rdist = new_rdist + cfg_->gap_viz.viz_jitter;
        gap.life_time = 50;
        gap.mode.reduced = true;
        return;
    }

    void GapManipulator::convertAxialGap(dynamic_gap::Gap& gap) {
        // Return if not axial gap or disabled
        if (!gap.isAxial() || !cfg_->gap_manip.axial_convert) {
            return;
        }

        auto stored_scan_msgs = *msg.get();
        
        bool left = gap.isLeftType();
        // Extend of rotation to the radial gap 
        // amp-ed by a **small** ratio to ensure the local goal does not exactly fall on the
        // visibility line
        float rot_val = (float) std::atan2(cfg_->gap_manip.epsilon2 * cfg_->gap_manip.rot_ratio, cfg_->gap_manip.epsilon1);
        float theta = left ? (rot_val + 1e-3): -(rot_val + 1e-3);
        int near_idx, far_idx;
        float near_dist, far_dist;
        
        if (left) {
            near_idx = gap.LIdx();
            far_idx = gap.RIdx();
            near_dist = gap.LDist();
            far_dist = gap.RDist();
        } else {
            far_idx = gap.LIdx();
            near_idx = gap.RIdx();
            far_dist = gap.LDist();
            near_dist = gap.RDist();
        }
        
        Eigen::Matrix3f rot_mat;
        rot_mat << cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta), 0,
                    0, 0, 1;

        auto half_num_scan = gap.half_scan;
        
        Eigen::Matrix3f near_rbt;
        near_rbt << 1, 0, near_dist * cos(M_PI / half_num_scan * (near_idx - half_num_scan)),
                    0, 1, near_dist * sin(M_PI / half_num_scan * (near_idx - half_num_scan)),
                    0, 0, 1;
        Eigen::Matrix3f far_rbt;
        far_rbt  << 1, 0, far_dist * cos(M_PI / half_num_scan * (far_idx - half_num_scan)),
                    0, 1, far_dist * sin(M_PI / half_num_scan * (far_idx - half_num_scan)),
                    0, 0, 1;
        
        Eigen::Matrix3f rot_rbt = near_rbt * (rot_mat * (near_rbt.inverse() * far_rbt));

        float r = float(sqrt(pow(rot_rbt(0, 2), 2) + pow(rot_rbt(1, 2), 2)));
        int idx = int (std::atan2(rot_rbt(1, 2), rot_rbt(0, 2)) / M_PI * half_num_scan) + half_num_scan;

        // Rotation Completed
        // Get minimum dist range val from start to target index location
        // For wraparound
        int offset = left ? gap._right_idx : idx;
        int upperbound = left ? idx : gap._left_idx;
        int intermediate_pt = offset + 1;
        int second_inter_pt = intermediate_pt;
        int size = upperbound - offset;


        if ((upperbound - offset) < 3) {
            // Arbitrary value
            gap.goal.discard = true;
            return;
        }

        offset = std::max(offset, 0);
        upperbound = std::min(upperbound, num_of_scan - 1);
        std::vector<float> min_dist(upperbound - offset);

        if (size == 0) {
            // This shouldn't happen
            return;
        }


        if (stored_scan_msgs.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect gap manip");
        }

        try{
            for (int i = 0; i < min_dist.size(); i++) {
                min_dist.at(i) = sqrt(pow(near_dist, 2) + pow(stored_scan_msgs.ranges.at(i + offset), 2) -
                    2 * near_dist * stored_scan_msgs.ranges.at(i + offset) * cos((i + offset - near_idx) * stored_scan_msgs.angle_increment));
            }
        } catch(...) {
            ROS_FATAL_STREAM("convertAxialGap outofBound");
        }

        auto farside_iter = std::min_element(min_dist.begin(), min_dist.end());
        float farside = *farside_iter;

        Eigen::Matrix3f far_near = near_rbt.inverse() * far_rbt;
        float coefs = far_near.block<2, 1>(0, 2).norm();
        // ROS_INFO_STREAM()
        far_near(0, 2) *= farside / coefs;
        far_near(1, 2) *= farside / coefs;
        Eigen::Matrix3f short_pt = near_rbt * (rot_mat * far_near);

        r = float(sqrt(pow(short_pt(0, 2), 2) + pow(short_pt(1, 2), 2)));
        idx = int (std::atan2(short_pt(1, 2), short_pt(0, 2)) / M_PI * half_num_scan) + half_num_scan;


        // Recalculate end point location based on length
        gap.convex.convex_lidx = left ? near_idx : idx;
        gap.convex.convex_ldist = left ? near_dist : r;
        gap.convex.convex_ridx = left ? idx : near_idx;
        gap.convex.convex_rdist = left ? r : near_dist;

        if (left && gap.convex.convex_ridx < gap.convex.convex_lidx) {
            gap.goal.discard = true;
        }

        if (!left && gap.convex.convex_ridx < gap.convex.convex_lidx) {
            gap.goal.discard = true;
        }

        gap.mode.agc = true;
    }

    void GapManipulator::radialExtendGap(dynamic_gap::Gap& selected_gap) {
        if (!cfg_->gap_manip.radial_extend) {
            ROS_DEBUG_STREAM_THROTTLE(1, "Radial Extension is off");
            return;
        }

        int half_num_scan = (int)(msg.get()->ranges.size()) / 2;
        float s = selected_gap.getMinSafeDist();

        float x1, x2, y1, y2;
        x1 = (selected_gap.convex.convex_ldist) * cos(-((float) half_num_scan - selected_gap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (selected_gap.convex.convex_ldist) * sin(-((float) half_num_scan - selected_gap.convex.convex_lidx) / half_num_scan * M_PI);

        x2 = (selected_gap.convex.convex_rdist) * cos(-((float) half_num_scan - selected_gap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (selected_gap.convex.convex_rdist) * sin(-((float) half_num_scan - selected_gap.convex.convex_ridx) / half_num_scan * M_PI);

        Eigen::Vector2f gL(x1, y1);
        Eigen::Vector2f gR(x2, y2);

        Eigen::Vector2f eL = gL / gL.norm();
        Eigen::Vector2f eR = gR / gR.norm();

        Eigen::Vector2f eB = (eL + eR) / 2;
        eB /= eB.norm();
        float gap_size = std::acos(eL.dot(eR));

        Eigen::Vector2f qB = -s * eB;
        
        // Shifted Back Frame
        Eigen::Vector2f qLp = gL - qB;
        Eigen::Vector2f qRp = gR - qB;

        Eigen::Vector2f pLp = car2pol(qLp);
        // pLp(1) += M_PI;
        Eigen::Vector2f pRp = car2pol(qRp);
        // pRp(1) += M_PI;

        float phiB = pRp(1) - pLp(1);

        Eigen::Vector2f pB = car2pol(-qB);
        // pB(2) += M_PI;

        float thL = pB(1) - gap_size / 4;
        float thR = pB(1) + gap_size / 4;

        Eigen::Vector2f pLn = pTheta(thL, phiB, pLp, pRp);
        Eigen::Vector2f pRn = pTheta(thR, phiB, pLp, pRp);

        Eigen::Vector2f qLn = pol2car(pLn) + qB;
        Eigen::Vector2f qRn = pol2car(pRn) + qB;

        // Store info back to original gap;
        Eigen::Vector2f polqLn = car2pol(qLn);
        Eigen::Vector2f polqRn = car2pol(qRn);

        selected_gap.convex.convex_lidx = polqLn(1) / M_PI * half_num_scan + half_num_scan;
        selected_gap.convex.convex_ridx = polqRn(1) / M_PI * half_num_scan + half_num_scan;
        selected_gap.convex.convex_ldist = polqLn(0);
        selected_gap.convex.convex_rdist = polqRn(0);
        selected_gap.mode.convex = true;

        selected_gap.qB = qB;
        ROS_DEBUG_STREAM("l: " << selected_gap._left_idx << " to " << selected_gap.convex_lidx
         << ", r: " << selected_gap._right_idx << " to " << selected_gap.convex_ridx);
        
        ROS_DEBUG_STREAM("ldist: " << selected_gap._ldist << " to " << selected_gap.convex_ldist
        << ", rdist: " << selected_gap._rdist << " to " << selected_gap.convex_rdist);

        return;
    }

    Eigen::Vector2f GapManipulator::car2pol(Eigen::Vector2f a) {
        return Eigen::Vector2f(a.norm(), float(std::atan2(a(1), a(0))));
    }

    Eigen::Vector2f GapManipulator::pol2car(Eigen::Vector2f a) {
        return Eigen::Vector2f(cos(a(1)) * a(0), sin(a(1)) * a(0));
    }

    Eigen::Vector2f GapManipulator::pTheta(
        float th, float phiB, Eigen::Vector2f pRp, Eigen::Vector2f pLp) {
        return pLp * (th - pRp(1)) / phiB + pRp * (pLp(1) - th) / phiB;
    }

}
