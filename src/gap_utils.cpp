#include <dynamic_gap/gap_utils.h>

namespace dynamic_gap {
    GapUtils::GapUtils() {}

    GapUtils::~GapUtils() {}

    GapUtils::GapUtils(const DynamicGapConfig& cfg) {
        cfg_ = & cfg;
    }

    ////////////////// GAP DETECTION ///////////////////////

    bool GapUtils::sweptGapStartedOrEnded(float scan_dist_i, float scan_dist_imin1, float max_scan_dist)
    {
        return scan_dist_imin1 < max_scan_dist != scan_dist_i < max_scan_dist;
    }

    // Checking if swept gap is either very large, or if robot can fit within gap (precondition to swept gap)
    bool GapUtils::sweptGapSizeCheck(dynamic_gap::Gap detected_gap, float half_scan)
    {
        bool large_gap = detected_gap.LIdx() - detected_gap.RIdx() > (3 * half_scan / 2);
        bool robot_can_fit = detected_gap.get_gap_euclidean_dist() > 3 * cfg_->rbt.r_inscr;
        
        return large_gap || robot_can_fit; //  || cfg_->planning.planning_inflated);
    }

    // Checking if robot can fit between gap between consecutive scan points (precondition to radial gap)
    bool GapUtils::radialGapSizeCheck(float scan_dist_i, float scan_dist_imin1, float max_scan_dist, float gap_angle)
    {
        if (!(scan_dist_imin1 < max_scan_dist && scan_dist_i < max_scan_dist))
            return false;

        // distance between current and previous points
        float scan_diff = sqrt(pow(scan_dist_imin1, 2) + pow(scan_dist_i, 2) - 2 * scan_dist_imin1 * scan_dist_i * cos(gap_angle));

        bool can_robot_fit = scan_diff > 3 * cfg_->rbt.r_inscr;
        // Inscribed radius gets enforced here, or unless using inflated egocircle, then no need for range diff
        bool finite_scan_dists = (cfg_->planning.planning_inflated && scan_dist_i < max_scan_dist && scan_dist_imin1 < max_scan_dist);


        return (can_robot_fit || finite_scan_dists);
    }   

    bool GapUtils::bridgeCondition(std::vector<dynamic_gap::Gap> raw_gaps, int scan_size)
    {
        bool multiple_gaps = raw_gaps.size() > 1;
        bool bordering_first_last_gaps = (raw_gaps.front().RIdx() == 0 && raw_gaps.back().LIdx() == (scan_size - 1));
        
        return multiple_gaps && bordering_first_last_gaps;
    }

    std::vector<dynamic_gap::Gap> GapUtils::gapDetection(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser, 
                                                         geometry_msgs::PoseStamped final_goal_rbt)
    {
        // ROS_INFO_STREAM("running gapDetection");
        std::vector<dynamic_gap::Gap> raw_gaps;
        sensor_msgs::LaserScan scan = *sharedPtr_laser.get();
        // get half scan value
        float half_scan = float(scan.ranges.size() / 2);

        float max_scan_dist = *std::max_element(scan.ranges.begin(), scan.ranges.end());
        float min_scan_dist = *std::min_element(scan.ranges.begin(), scan.ranges.end());
        // ROS_INFO_STREAM("gapDetection min_dist: " << min_dist);

        std::string frame = scan.header.frame_id;
        // starting the left point of the gap at front facing value
        // std::cout << "max laser scan range: " << scan.range_max << std::endl;
        int gap_ridx = 0;
        float gap_rdist = scan.ranges[0];
        // last as in previous scan
        bool mid_swept_gap = gap_rdist >= max_scan_dist;
        float scan_dist_i = scan.ranges[0];
        float scan_dist_imin1 = scan.ranges[0];

        // iterating through scan
        //std::cout << "finding raw gaps: " << std::endl;
        for (unsigned int it = 1; it < scan.ranges.size(); ++it)
        {
            scan_dist_i = scan.ranges[it];
            // ROS_INFO_STREAM("iter: " << it << ", dist: " << scan_dist_i);

            if (radialGapSizeCheck(scan_dist_i, scan_dist_imin1, max_scan_dist, scan.angle_increment)) 
            {
                // initializing a radial gap
                dynamic_gap::Gap detected_gap(frame, it - 1, scan_dist_imin1, true, half_scan, min_scan_dist);
                detected_gap.addLeftInformation(it, scan_dist_i);
                raw_gaps.push_back(detected_gap);
                // ROS_INFO_STREAM("adding radial gap from: (" << detected_gap.RIdx() << ", " << detected_gap.RDist() << "), to (" << detected_gap.LIdx() << ", " << detected_gap.LDist() << ")");
            }

            // Either previous distance finite and current distance infinite or vice-versa, 
            if (sweptGapStartedOrEnded(scan_dist_i, scan_dist_imin1, max_scan_dist))
            {
                // Signals the ending of a gap
                if (mid_swept_gap)
                {
                    mid_swept_gap = false;                    
                    // ROS_INFO_STREAM("gap ending: infinity to finite");
                    dynamic_gap::Gap detected_gap(frame, gap_ridx, gap_rdist, false, half_scan, min_scan_dist);
                    detected_gap.addLeftInformation(it, scan_dist_i);
                    //std::cout << "candidate swept gap from (" << gap_ridx << ", " << gap_rdist << "), to (" << it << ", " << scan_dist << ")" << std::endl;
                    // Inscribed radius gets enforced here, or unless using inflated egocircle, then no need for range diff
                    // Max: added first condition for if gap is sufficiently large. E.g. if agent directly behind robot, can get big gap but L/R points are close together
                    if (sweptGapSizeCheck(detected_gap, half_scan)) {
                        //std::cout << "adding candidate swept gap" << std::endl;
                        // ROS_INFO_STREAM("adding swept gap from: (" << detected_gap.RIdx() << ", " << detected_gap.RDist() << "), to (" << detected_gap.LIdx() << ", " << detected_gap.LDist() << ")");                
                        raw_gaps.push_back(detected_gap);
                    }
                }
                else // signals the beginning of a gap
                {
                    // ROS_INFO_STREAM("gap starting: finite to infinity");
                    gap_ridx = it - 1;
                    gap_rdist = scan_dist_imin1;
                    mid_swept_gap = true;
                }

            }
            scan_dist_imin1 = scan_dist_i;
        }

        // Catch the last gap (could be in the middle of a swept gap but laser scan ends)
        if (mid_swept_gap) 
        {
            // ROS_INFO_STREAM("catching last gap");
            dynamic_gap::Gap detected_gap(frame, gap_ridx, gap_rdist, false, half_scan, min_scan_dist);
            int last_scan_idx = scan.ranges.size() - 1;
            double last_scan_dist = *(scan.ranges.end() - 1);
            detected_gap.addLeftInformation(last_scan_idx, last_scan_dist);
            // ROS_INFO_STREAM("gap_ridx: " << gap_ridx << ", gap_rdist: " << gap_rdist);
            // ROS_INFO_STREAM("last_scan_idx: " << last_scan_idx << ", last_scan_dist: " << last_scan_dist);
            // ROS_INFO_STREAM("lidx: " << detected_gap.LIdx() << ", ridx: " << detected_gap.RIdx());
            // ROS_INFO_STREAM("gap side dist: " << gap_dist_side);
            if (sweptGapSizeCheck(detected_gap, half_scan)) 
            {
                // ROS_INFO_STREAM("adding candidate last gap");
                raw_gaps.push_back(detected_gap);
                // ROS_INFO_STREAM("adding last gap: (" << detected_gap.RIdx() << ", " << detected_gap.RDist() << "), to (" << detected_gap.LIdx() << ", " << detected_gap.LDist() << ")");                
            }
        }
        
        // Bridge the last gap around
        if (bridgeCondition(raw_gaps, scan.ranges.size()))
        {
            // ROS_INFO_STREAM("bridging first and last gaps");
            float start_side_dist = raw_gaps.front().LDist();
            int start_side_idx = raw_gaps.front().LIdx();

            raw_gaps.back().addLeftInformation(start_side_idx, start_side_dist);
            raw_gaps.erase(raw_gaps.begin());
            // ROS_INFO_STREAM("revising last gap: (" << raw_gaps.back().RIdx() << ", " << raw_gaps.back().RDist() << "), to (" << raw_gaps.back().LIdx() << ", " << raw_gaps.back().LDist() << ")");                
        }
        
        // if terminal_goal within laserscan and not within a gap, create a gap
        int final_goal_idx;
        if (terminalGoalGapCheck(final_goal_rbt, scan, final_goal_idx))
            addTerminalGoal(final_goal_idx, raw_gaps, scan);

        return raw_gaps;
    }

    bool GapUtils::terminalGoalGapCheck(geometry_msgs::PoseStamped final_goal_rbt, 
                                        sensor_msgs::LaserScan scan,
                                        int & final_goal_idx)
    {
        double final_goal_dist = sqrt(pow(final_goal_rbt.pose.position.x, 2) + pow(final_goal_rbt.pose.position.y, 2));
        double final_goal_theta = std::atan2(final_goal_rbt.pose.position.y, final_goal_rbt.pose.position.x);

        double final_goal_idx_double = (final_goal_theta + M_PI) / scan.angle_increment;
        final_goal_idx = int(std::floor(final_goal_idx_double));

        double final_goal_idx_scan_dist = scan.ranges.at(final_goal_idx);

        return (final_goal_dist < final_goal_idx_scan_dist);
    }   

    void GapUtils::addTerminalGoal(int final_goal_idx,
                                    std::vector<dynamic_gap::Gap> &raw_gaps,
                                    sensor_msgs::LaserScan scan) {
        ROS_INFO_STREAM("running addTerminalGoal");
        ROS_INFO_STREAM("final_goal_idx: " << final_goal_idx);
        int gap_idx = 0;
        int half_num_scan = scan.ranges.size() / 2;
        auto min_dist = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        for (dynamic_gap::Gap g : raw_gaps) {
            // if final_goal idx is within gap, return
            // ROS_INFO_STREAM("checking against: " << g.RIdx() << " to " << g.LIdx());
            if (final_goal_idx >= g.RIdx() && final_goal_idx <= g.LIdx()) {
                ROS_INFO_STREAM("final goal is in gap: " << g.RIdx() << ", " << g.LIdx());
                return;
            }
            gap_idx += 1;
        }

        std::string frame = scan.header.frame_id;
        int artificial_gap_span = half_num_scan / 12;
        int right_idx = std::max(final_goal_idx - artificial_gap_span, 0);
        int left_idx = std::min(final_goal_idx + artificial_gap_span, 2*half_num_scan - 1);
        ROS_INFO_STREAM("creating gap " << right_idx << ", to " << left_idx);

        dynamic_gap::Gap detected_gap(frame, right_idx, scan.ranges.at(right_idx), true, half_num_scan, min_dist);
        detected_gap.addLeftInformation(left_idx, scan.ranges.at(left_idx));
        detected_gap.artificial = true;
        raw_gaps.insert(raw_gaps.begin() + gap_idx, detected_gap);        
        return;
    }    

    ////////////////// GAP SIMPLIFICATION ///////////////////////

    // iterating backwards through simplified gaps to see if/where they can be merged
    int GapUtils::checkSimplifiedGapsMergeability(dynamic_gap::Gap raw_gap, 
                                        std::vector<dynamic_gap::Gap> simplified_gaps,
                                        sensor_msgs::LaserScan scan)
    {
        // if coefs = 0. as long as raw_gap's rdist is less than or equal to curr_gaps' ldist, we will merge
        float coefs = cfg_->planning.planning_inflated ? 0 : 2; // MAX: changed
        int last_mergable = -1;

        int start_idx, end_idx;
        // ROS_INFO_STREAM("attempting merge with raw gap: (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ") to (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ")");
        for (int j = (simplified_gaps.size() - 1); j >= 0; j--)
        {
            // ROS_INFO_STREAM("on simplified gap " << j << " of " << simplified_gaps.size() << ": ");
            // ROS_INFO_STREAM("points: (" << simplified_gaps[j].RIdx() << ", " << simplified_gaps[j].RDist() << ") to (" << simplified_gaps[j].LIdx() << ", " << simplified_gaps[j].LDist() << ")");
            start_idx = std::min(simplified_gaps[j].LIdx(), raw_gap.RIdx());
            end_idx = std::max(simplified_gaps[j].LIdx(), raw_gap.RIdx());
            float min_intergap_dist = *std::min_element(scan.ranges.begin() + start_idx, scan.ranges.begin() + end_idx);
            float inflated_min_intergap_dist = min_intergap_dist - coefs * cfg_->rbt.r_inscr;

            // 1. Checking if raw gap left and simplified gap right (widest distances, encompassing both gaps) dist 
            //    is less than the dist of whatever separates the two gaps
            bool intergap_dist_test = raw_gap.LDist() <= inflated_min_intergap_dist && 
                                                 simplified_gaps[j].RDist() <= inflated_min_intergap_dist;
            
            // 2. Checking if current simplified gap is either right dist < left dist or swept 
            bool left_or_radial = simplified_gaps[j].isRightType() || !simplified_gaps[j].isRadial();

            // 3. Making sure that this merged gap is not too large
            bool idx_diff = (raw_gap.LIdx() - simplified_gaps[j].RIdx()) < cfg_->gap_manip.max_idx_diff;

            // ROS_INFO_STREAM("simp_left_raw_right_dist_test: " << simp_left_raw_right_dist_test << ", left_or_radial: " << left_or_radial << ", idx_diff: " << idx_diff);
            if (intergap_dist_test && left_or_radial && idx_diff) {
                last_mergable = j;
            } 
        }

        return last_mergable;
    }

    bool GapUtils::mergeSweptGapCondition(dynamic_gap::Gap raw_gap, std::vector<dynamic_gap::Gap> simplified_gaps)
    {
        // checking if difference between raw gap left dist and simplified gap right (widest distances, encompassing both gaps)
        // dist is sufficiently small (to fit robot)
        bool raw_left_simp_right_dist_diff_check = std::abs(raw_gap.LDist() - simplified_gaps.back().RDist()) < 3 * cfg_->rbt.r_inscr;

        // checking if difference is sufficiently small, and that current simplified gap is radial and right dist < left dist
        return raw_left_simp_right_dist_diff_check && simplified_gaps.back().isRadial() && simplified_gaps.back().isRightType();
    }


    std::vector<dynamic_gap::Gap> GapUtils::gapSimplification(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
                                                              std::vector<dynamic_gap::Gap> raw_gaps)
    {
        //double start_time = ros::Time::now().toSec();
        std::vector<dynamic_gap::Gap> simplified_gaps;

        sensor_msgs::LaserScan scan = *sharedPtr_laser.get();

        // Insert first
        bool mark_to_start = true;

        float curr_left_dist = 0.0;
        int last_mergable = -1;
        
        // ROS_INFO_STREAM("running gapSimplification: ");
        for (dynamic_gap::Gap raw_gap : raw_gaps)
        {
            // ROS_INFO_STREAM("on raw gap: (" << raw_gap.RIdx() << ", " << raw_gap.RDist() << ") to (" << raw_gap.LIdx() << ", " << raw_gap.LDist() << ")");
            
            if (mark_to_start)
            {   
                // if we have not started simplification, this raw gap is swept, and right dist < left dist, then we can merge gaps
                if (raw_gap.isRadial() && raw_gap.isRightType())
                {
                    // ROS_INFO_STREAM("starting simplification");
                    mark_to_start = false;
                }

                simplified_gaps.push_back(raw_gap);
            } else {
                if (raw_gap.isRadial()) // if gap is radial
                {
                    if (raw_gap.isRightType()) // if right dist < left dist
                    {
                        // ROS_INFO_STREAM("adding raw gap (radial, right<left)");
                        simplified_gaps.push_back(raw_gap);
                    }
                    else
                    {
                        curr_left_dist = raw_gap.LDist();
                        last_mergable = checkSimplifiedGapsMergeability(raw_gap, simplified_gaps, scan);

                        if (last_mergable != -1) {
                            // ROS_INFO_STREAM("erasing simplified gaps from " << (last_mergable + 1) << " to " << simplified_gaps.size());
                            simplified_gaps.erase(simplified_gaps.begin() + last_mergable + 1, simplified_gaps.end());
                            simplified_gaps.back().addLeftInformation(raw_gap.LIdx(), raw_gap.LDist());
                            // ROS_INFO_STREAM("merging last simplified gap into (" << simplified_gaps.back().RIdx() << ", " << simplified_gaps.back().RDist() << ") to (" << simplified_gaps.back().LIdx() << ", " << simplified_gaps.back().LDist() << ")");
                        } else {
                            // ROS_INFO_STREAM("no merge, adding raw gap (swept, left<right)");                            
                            simplified_gaps.push_back(raw_gap);
                        }
                    }
                }
                else
                { // If current raw gap is swept
                    curr_left_dist = raw_gap.LDist();
                    if (mergeSweptGapCondition(raw_gap, simplified_gaps))
                    {
                        simplified_gaps.back().addLeftInformation(raw_gap.LIdx(), raw_gap.LDist());
                        // ROS_INFO_STREAM("merging last simplifed gap to (" << simplified_gaps.back().RIdx() << ", " << simplified_gaps.back().RDist() << ") to (" << simplified_gaps.back().LIdx() << ", " << simplified_gaps.back().LDist() << ")");
                    } else {
                        // ROS_INFO_STREAM("adding raw gap (swept)");                            
                        simplified_gaps.push_back(raw_gap);
                    }
                }
            }
            // ROS_INFO_STREAM("---");
        }
        //ROS_INFO_STREAM("gapSimplification time elapsed: " << ros::Time::now().toSec() - start_time); 

        return simplified_gaps;
    }

    ////////////////// STATIC SCAN SEPARATION ///////////////////////

    bool compareModelBearingValues(dynamic_gap::rot_frame_kf* model_one, dynamic_gap::rot_frame_kf* model_two) {
        Eigen::Matrix<double, 4, 1> state_one = model_one->get_frozen_cartesian_state();
        Eigen::Matrix<double, 4, 1> state_two = model_two->get_frozen_cartesian_state();
        
        return atan2(state_one[1], state_one[0]) < atan2(state_two[1], state_two[0]);
    }

    bool checkModelSimilarity(dynamic_gap::rot_frame_kf * curr_model, dynamic_gap::rot_frame_kf * prev_model) {
        double eps = 0.00001;
        
        Eigen::Matrix<double, 4, 1> curr_state = curr_model->get_frozen_cartesian_state();
        Eigen::Matrix<double, 4, 1> prev_state = prev_model->get_frozen_cartesian_state();
        
        Eigen::Vector2d curr_vel(curr_state[2], curr_state[3]);
        Eigen::Vector2d prev_vel(prev_state[2], prev_state[3]);

        Eigen::Vector2d curr_vel_dir = curr_vel / (curr_vel.norm() + eps);
        Eigen::Vector2d prev_vel_dir = prev_vel / (prev_vel.norm() + eps);
        // ROS_INFO_STREAM("curr_velocity: " << curr_state[2] << ", " << curr_state[3] << ", prev_velocity: " << prev_state[2] << ", " << prev_state[3]);
        // ROS_INFO_STREAM("curr_vel_dir: " << curr_vel_dir[0] << ", " << curr_vel_dir[1]);
        // ROS_INFO_STREAM("prev_vel_dir: " << prev_vel_dir[0] << ", " << prev_vel_dir[1]);
        double dot_prod = curr_vel_dir.dot(prev_vel_dir);
        // ROS_INFO_STREAM("dot product: " << dot_prod);
        double norm_ratio = prev_vel.norm() / (curr_vel.norm() + eps);
        // ROS_INFO_STREAM("norm ratio: " << norm_ratio);

        return (dot_prod > 0.5 && curr_vel.norm() > 0.1 && std::abs(norm_ratio - 1.0) < 0.1);
    }

    void createAgentFromModels(dynamic_gap::rot_frame_kf * curr_model,    
                               dynamic_gap::rot_frame_kf * prev_model,
                               std::vector<Eigen::Matrix<double, 4, 1> > & agents,
                               bool print) {
        Eigen::Matrix<double, 4, 1> curr_state = curr_model->get_frozen_cartesian_state();
        Eigen::Matrix<double, 4, 1> prev_state = prev_model->get_frozen_cartesian_state();
        
        Eigen::Matrix<double, 4, 1> new_agent = (curr_state + prev_state) / 2;
        // if (print) ROS_INFO_STREAM("instantiating agent: " << new_agent[0] << ", " << new_agent[1] << ", " << new_agent[2] << ", " << new_agent[3]);
        agents.push_back(new_agent);                                  
    }

    void clearAgentFromStaticScan(dynamic_gap::rot_frame_kf * curr_model, 
                               dynamic_gap::rot_frame_kf * prev_model,
                               sensor_msgs::LaserScan & curr_scan) {

        int half_num_scan = curr_scan.ranges.size() / 2;

        Eigen::Vector2d curr_meas = curr_model->get_x_tilde();
        Eigen::Vector2d prev_meas = prev_model->get_x_tilde();

        int curr_idx = int(std::round(std::atan2(curr_meas[1], curr_meas[0]) * (half_num_scan / M_PI))) + half_num_scan;
        int prev_idx = int(std::round(std::atan2(prev_meas[1], prev_meas[0]) * (half_num_scan / M_PI))) + half_num_scan;
        // ROS_INFO_STREAM("prev_idx: " << prev_idx << ", curr_idx: " << curr_idx);

        int prev_free_idx, curr_free_idx;
        float prev_free_dist, curr_free_dist;
        for (int j = 1; j < 2; j++) {
            prev_free_idx = (prev_idx - j);
            if (prev_free_idx < 0) {
                prev_free_idx += 2*half_num_scan;
            }
            prev_free_dist = curr_scan.ranges.at(prev_free_idx);
            // ROS_INFO_STREAM("range at " << prev_free_idx << ": " << prev_free_dist);
        }

        for (int j = 1; j < 2; j++) {
            curr_free_idx = (curr_idx + j) % (2*half_num_scan);
            curr_free_dist = curr_scan.ranges.at(curr_free_idx);
            // ROS_INFO_STREAM("range at " << curr_free_idx << ": " << curr_free_dist);
        }

        // need distance from curr to prev
        float prev_to_curr_idx_dist = (curr_free_idx - prev_free_idx);
        if (prev_to_curr_idx_dist < 0) {
            prev_to_curr_idx_dist += 2*half_num_scan;
        }
        // ROS_INFO_STREAM("prev_to_curr_idx_dist: " << prev_to_curr_idx_dist);

        for (int counter = 1; counter < prev_to_curr_idx_dist; counter++) {
            int intermediate_idx = (prev_free_idx + counter) % (2*half_num_scan);
            float new_dist = (curr_free_dist - prev_free_dist) * (counter / prev_to_curr_idx_dist) + prev_free_dist;
            // ROS_INFO_STREAM("replacing range at " << intermediate_idx << " from " << curr_scan.ranges.at(intermediate_idx) << " to " << new_dist);
            curr_scan.ranges.at(intermediate_idx) = new_dist;
        }
    }

    sensor_msgs::LaserScan GapUtils::staticDynamicScanSeparation(std::vector<dynamic_gap::Gap> observed_gaps, 
                                                                boost::shared_ptr<sensor_msgs::LaserScan const> msg,
                                                                bool print) {

        sensor_msgs::LaserScan curr_scan = *msg.get(); 
        if (observed_gaps.size() == 0) {
            return curr_scan;
        }

        std::vector<dynamic_gap::rot_frame_kf *> obs_models;
        for (auto gap : observed_gaps) {
            obs_models.push_back(gap.left_model);
            obs_models.push_back(gap.right_model);
        }

        for (auto & model : obs_models) {
            model->freeze_robot_vel();
        }
        
        sort(obs_models.begin(), obs_models.end(), compareModelBearingValues);

        // iterate through models
        dynamic_gap::rot_frame_kf * prev_model = obs_models[0];
        dynamic_gap::rot_frame_kf * curr_model = obs_models[1];

        std::vector<Eigen::Matrix<double, 4, 1> > agents;

        // ROS_INFO_STREAM("looping through models");
        for (int i = 1; i < obs_models.size(); i++) {
            curr_model = obs_models[i];

            if (checkModelSimilarity(curr_model, prev_model)) {
                clearAgentFromStaticScan(curr_model, prev_model, curr_scan);
                createAgentFromModels(curr_model, prev_model, agents, print);
            }

            prev_model = curr_model;
        }

        // bridging models
        prev_model = obs_models[obs_models.size() - 1];
        curr_model = obs_models[0];

        if (checkModelSimilarity(curr_model, prev_model)) {
            clearAgentFromStaticScan(curr_model, prev_model, curr_scan);
            createAgentFromModels(curr_model, prev_model, agents, print);
        }

        curr_agents = agents;
        return curr_scan;

        // create a static scan (would need to interpolate to fill in spots where agents are)
        
        // create a pose for each agent, forward propagate it, somehow turn that pose into bearings for scan 

        // 
    }

    std::vector<Eigen::Matrix<double, 4, 1> > GapUtils::getCurrAgents() {
        return curr_agents;
    }


}