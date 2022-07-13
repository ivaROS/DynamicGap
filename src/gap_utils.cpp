#include <dynamic_gap/gap_utils.h>

namespace dynamic_gap {
    GapUtils::GapUtils() {}

    GapUtils::~GapUtils() {}

    GapUtils::GapUtils(const DynamicGapConfig& cfg) {
        cfg_ = & cfg;
    }

    std::vector<dynamic_gap::Gap> GapUtils::hybridScanGap(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser, geometry_msgs::PoseStamped final_goal_rbt)
    {
        // ROS_INFO_STREAM("running hybridScanGap");
        // clear gaps
        double start_time = ros::Time::now().toSec();
        std::vector<dynamic_gap::Gap> raw_gaps;
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        // get half scan value
        float half_scan = float(stored_scan_msgs.ranges.size() / 2);
        bool prev = true;
        auto max_dist_iter = std::max_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.end());
        float max_scan_dist = *max_dist_iter;
        auto min_dist = *std::min_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.end());
        // ROS_INFO_STREAM("hybridScanGap min_dist: " << min_dist);
        int gap_size = 0;
        std::string frame = stored_scan_msgs.header.frame_id;
        // starting the left point of the gap at front facing value
        // std::cout << "max laser scan range: " << stored_scan_msgs.range_max << std::endl;
        int gap_ridx = 0;
        float gap_rdist = stored_scan_msgs.ranges[0];
        // last as in previous scan
        float last_scan = stored_scan_msgs.ranges[0];
        bool prev_lgap = gap_rdist >= max_scan_dist;
        float scan_dist;
        float scan_diff;

        // iterating through scan
        //std::cout << "finding raw gaps: " << std::endl;
        for (std::vector<float>::size_type it = 1; it < stored_scan_msgs.ranges.size(); ++it)
        {
            scan_dist = stored_scan_msgs.ranges[it];
            // ROS_INFO_STREAM("iter: " << it << ", dist: " << scan_dist);
            // difference in distance between current and previous rays
            scan_diff = scan_dist - last_scan;
            // Arbitrary small threshold for a range difference to be considered
            if (std::abs(scan_diff) > 0.2) 
            {
                // If both current and last values are not infinity, meaning this is not a swept gap
                if (scan_dist < max_scan_dist && last_scan < max_scan_dist) 
                {
                    // initializing a radial gap
                    dynamic_gap::Gap detected_gap(frame, it - 1, last_scan, true, half_scan);
                    detected_gap.addLeftInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    //std::cout << "candiate radial gap from (" << (it-1) << ", " << last_scan << "), to (" << it << ", " << scan_dist << ")" << std::endl;
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    if (detected_gap.get_dist_side() > 4 * cfg_->rbt.r_inscr || cfg_->planning.planning_inflated) { 
                        //std::cout << "adding candidate radial gap" << std::endl;
                        raw_gaps.push_back(detected_gap);
                    }
                }
                
            }

            // Either previous distance finite and current distance infinite or vice-versa, 
            if (last_scan < max_scan_dist != scan_dist < max_scan_dist)
            {
                // Signals the ending of a gap
                if (prev_lgap)
                {
                    // ROS_INFO_STREAM("gap ending: infinity to finite");
                    prev_lgap = false;
                    dynamic_gap::Gap detected_gap(frame, gap_ridx, gap_rdist, false, half_scan);
                    detected_gap.addLeftInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    //std::cout << "candidate swept gap from (" << gap_ridx << ", " << gap_rdist << "), to (" << it << ", " << scan_dist << ")" << std::endl;
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    // Max: added first condition for if gap is sufficiently large. E.g. if agent directly behind robot, can get big gap but L/R points are close together
                    if (detected_gap.LIdx() - detected_gap.RIdx() > (3 * half_scan / 2) || detected_gap.get_dist_side() > 4 * cfg_->rbt.r_inscr || cfg_->planning.planning_inflated) {
                        //std::cout << "adding candidate swept gap" << std::endl;
                        raw_gaps.push_back(detected_gap);
                    }
                }
                else // signals the beginning of a gap
                {
                    // ROS_INFO_STREAM("gap starting: finite to infinity");
                    gap_ridx = it - 1;
                    gap_rdist = last_scan;
                    prev_lgap = true;
                }
            }
            last_scan = scan_dist;
        }

        // Catch the last gap (could be in the middle of a swept gap but laser scan ends)
        if (prev_lgap) 
        {
            // ROS_INFO_STREAM("catching last gap");
            dynamic_gap::Gap detected_gap(frame, gap_ridx, gap_rdist, false, half_scan);
            int last_scan_idx = stored_scan_msgs.ranges.size() - 1;
            double last_scan_dist = *(stored_scan_msgs.ranges.end() - 1);
            detected_gap.addLeftInformation(last_scan_idx, last_scan_dist);
            // ROS_INFO_STREAM("gap_ridx: " << gap_ridx << ", gap_rdist: " << gap_rdist);
            // ROS_INFO_STREAM("last_scan_idx: " << last_scan_idx << ", last_scan_dist: " << last_scan_dist);
            detected_gap.setMinSafeDist(min_dist);
            // ROS_INFO_STREAM("lidx: " << detected_gap.LIdx() << ", ridx: " << detected_gap.RIdx());
            float gap_dist_side = detected_gap.get_dist_side();
            // ROS_INFO_STREAM("gap side dist: " << gap_dist_side);
            //std::cout << "candidate last gap from (" << gap_ridx << ", " << gap_rdist << "), to (" << last_scan_idx << ", " << last_scan_dist << ")" << std::endl;
            if (detected_gap.LIdx() - detected_gap.RIdx() > (3 * half_scan / 2) || gap_dist_side > 4 * cfg_->rbt.r_inscr) {
                // ROS_INFO_STREAM("adding candidate last gap");
                raw_gaps.push_back(detected_gap);
            }
        }
        
        // Bridge the last gap around
        if (raw_gaps.size() > 1)
        {
            // ROS_INFO_STREAM("bridging");
            int last_raw_gap_idx = raw_gaps.size() - 1;
            //std::cout << "checking: " << raw_gaps[0].RIdx() << " and " << raw_gaps[raw_gaps.size() - 1].LIdx() << std::endl;
            if (raw_gaps[0].RIdx() == 0 && raw_gaps[last_raw_gap_idx].LIdx() == (stored_scan_msgs.ranges.size() - 1)) // Magic number?
            {
                // ROS_INFO_STREAM("bridging first and last gaps");
                // Both ends
                // currently only changing distances?
                float start_side_dist = raw_gaps[0].LDist();
                int start_side_idx = raw_gaps[0].LIdx();
                float end_side_dist = raw_gaps[last_raw_gap_idx].RDist();
                int end_side_idx = raw_gaps[last_raw_gap_idx].RIdx();
                // ROS_INFO_STREAM("original first gap: (" << raw_gaps[0].RIdx() << ", " << raw_gaps[0].RDist() << ") to (" << raw_gaps[0].LIdx() << ", " << raw_gaps[0].LDist()  << ")");
                // ROS_INFO_STREAM("original last gap: (" << raw_gaps[raw_gaps.size() - 1].RIdx() << ", " << raw_gaps[raw_gaps.size() - 1].RDist() << ") to (" << raw_gaps[raw_gaps.size() - 1].LIdx() << ", " << raw_gaps[raw_gaps.size() - 1].LDist()  << ")");
                // int total_size = (stored_scan_msgs.ranges.size() - 1) - end_side_idx + start_side_idx;
                // float result = (end_side_dist - start_side_dist) * (float (start_side_idx) / float (total_size)) + start_side_dist;
                // raw_gaps[0].setLeftObs();
                // raw_gaps[raw_gaps.size() - 1].setRightObs();
                // raw_gaps[raw_gaps.size() - 1].addLeftInformation(511, result);
                // raw_gaps[0].setRDist(result);
                // ROS_INFO_STREAM("revised first gap: (" << raw_gaps[0].RIdx() << ", " << raw_gaps[0].RDist() << ") to (" << raw_gaps[0].LIdx() << ", " << raw_gaps[0].LDist()  << ")");
                // ROS_INFO_STREAM("revised last gap: (" << raw_gaps[raw_gaps.size() - 1].RIdx() << ", " << raw_gaps[raw_gaps.size() - 1].RDist() << ") to (" << raw_gaps[raw_gaps.size() - 1].LIdx() << ", " << raw_gaps[raw_gaps.size() - 1].LDist()  << ")");

                //std::cout << "merging into last gap: (" << raw_gaps[last_raw_gap_idx].RIdx() << ", " << raw_gaps[last_raw_gap_idx].RDist() << ") to (" << raw_gaps[last_raw_gap_idx].LIdx() << ", " << raw_gaps[last_raw_gap_idx].LDist()  << ")" << std::endl;
                raw_gaps[last_raw_gap_idx].addLeftInformation(start_side_idx, start_side_dist);
                raw_gaps.erase(raw_gaps.begin(), raw_gaps.begin() + 1);
                //std::cout << "revised first gap: (" << raw_gaps[0].RIdx() << ", " << raw_gaps[0].RDist() << ") to (" << raw_gaps[0].LIdx() << ", " << raw_gaps[0].LDist()  << ")" << std::endl;
            }
        }

        //ROS_INFO_STREAM("hybridScanGap time elapsed: " << ros::Time::now().toSec() - start_time); 
        
        // if terminal_goal within laserscan and not in a gap, create a gap
        //ROS_INFO_STREAM("laserscan CB");
        double final_goal_dist = sqrt(pow(final_goal_rbt.pose.position.x, 2) + pow(final_goal_rbt.pose.position.y, 2));
        if (final_goal_dist > 0) {
            double final_goal_theta = std::atan2(final_goal_rbt.pose.position.y, final_goal_rbt.pose.position.x);
            int half_num_scan = stored_scan_msgs.ranges.size() / 2;
            
            double final_goal_idx_double = (final_goal_theta + M_PI) / stored_scan_msgs.angle_increment;
            int final_goal_idx = int(std::floor(final_goal_idx_double));
            // ROS_INFO_STREAM("final_goal_idx: " << final_goal_idx);
            double scan_dist = stored_scan_msgs.ranges.at(final_goal_idx);
            
            if (final_goal_dist < scan_dist) {
                raw_gaps = addTerminalGoal(final_goal_idx, raw_gaps, stored_scan_msgs);
            }
        }

        return raw_gaps;
    }

    std::vector<dynamic_gap::Gap> GapUtils::addTerminalGoal(int final_goal_idx,
                                                            std::vector<dynamic_gap::Gap> &raw_gaps,
                                                            sensor_msgs::LaserScan stored_scan_msgs) {
        ROS_INFO_STREAM("running addTerminalGoal");
        ROS_INFO_STREAM("final_goal_idx: " << final_goal_idx);
        int gap_idx = 0;
        int half_num_scan = stored_scan_msgs.ranges.size() / 2;
        auto min_dist = *std::min_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.end());

        for (dynamic_gap::Gap g : raw_gaps) {
            // if final_goal idx is within gap, return
            // ROS_INFO_STREAM("checking against: " << g.RIdx() << " to " << g.LIdx());
            if (final_goal_idx >= g.RIdx() && final_goal_idx <= g.LIdx()) {
                ROS_INFO_STREAM("final goal is in gap: " << g.RIdx() << ", " << g.LIdx());
                return raw_gaps;
            }
            gap_idx += 1;
        }

        std::string frame = stored_scan_msgs.header.frame_id;
        int half_gap_span = half_num_scan / 12;
        int right_idx = std::max(final_goal_idx - half_gap_span, 0);
        int left_idx = std::min(final_goal_idx + half_gap_span, 2*half_num_scan - 1);
        ROS_INFO_STREAM("creating gap " << right_idx << ", to " << left_idx);

        dynamic_gap::Gap detected_gap(frame, right_idx, stored_scan_msgs.ranges.at(right_idx), true, half_num_scan);
        detected_gap.addLeftInformation(left_idx, stored_scan_msgs.ranges.at(left_idx));
        detected_gap.setMinSafeDist(min_dist);
        detected_gap.artificial = true;
        raw_gaps.insert(raw_gaps.begin() + gap_idx, detected_gap);        
        return raw_gaps;
    }    

    std::vector<dynamic_gap::Gap> GapUtils::mergeGapsOneGo(
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<dynamic_gap::Gap>& raw_gaps)
    {
        //double start_time = ros::Time::now().toSec();

        int observed_size = (int) raw_gaps.size();
        std::vector<dynamic_gap::Gap> simplified_gaps;

        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        // Termination Condition

        // Insert first
        bool mark_to_start = true;
        bool last_type_left = true;
        int left_counter = 0;
        bool changed = true;

        int num_raw_gaps = (int) raw_gaps.size();
        // ROS_INFO_STREAM("running MergeGapsOneGo: ");
        for (int i = 0; i < num_raw_gaps; i++)
        {
            // ROS_INFO_STREAM("raw gap " << i << " of " << num_raw_gaps << ": ");
            // ROS_INFO_STREAM("points: (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ") to (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ")");
            // ROS_INFO_STREAM("axial: " << raw_gaps[i].isAxial() << ", left: " << raw_gaps[i].isRightType());
            // if we are starting merging, this raw gap is swept, and left dist < right dist, then we may be able to merge
            if (mark_to_start && raw_gaps.at(i).isAxial() && raw_gaps.at(i).isRightType())
            {   
                mark_to_start = false;
                // ROS_INFO_STREAM("adding first swept left raw gap: (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ") to (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ")");
                simplified_gaps.push_back(raw_gaps[i]);
            } else {
                if (!mark_to_start) // if we have already found a mergable gap
                {
                    if (raw_gaps.at(i).isAxial()) // if gap is axial
                    {
                        if (raw_gaps.at(i).isRightType()) // if l_dist < r_dist
                        {
                            // ROS_INFO_STREAM("adding a raw gap (swept, left): (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ") to (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ")");
                            simplified_gaps.push_back(raw_gaps[i]);
                        }
                        else
                        {
                            float curr_rdist = raw_gaps[i].LDist();
                            int erase_counter = 0;
                            int last_mergable = -1;

                            float coefs = cfg_->planning.planning_inflated ? 0 : 2; // MAX: changed
                            // if coefs = 0. as long as raw_gap's rdist is less than or equal to curr_gaps' ldist, we will merge
                            // iterating backwards through simplified gaps to see if they can be merged
                            // ROS_INFO_STREAM("attempting merge with raw gap: (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ") to (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ")");
                            for (int j = (int) (simplified_gaps.size() - 1); j >= 0; j--)
                            {
                                // ROS_INFO_STREAM("on simplified gap " << j << " of " << simplified_gaps.size() << ": ");
                                // ROS_INFO_STREAM("points: (" << simplified_gaps[j].RIdx() << ", " << simplified_gaps[j].RDist() << ") to (" << simplified_gaps[j].LIdx() << ", " << simplified_gaps[j].LDist() << ")");
                                int start_idx = std::min(simplified_gaps[j].LIdx(), raw_gaps[i].RIdx());
                                int end_idx = std::max(simplified_gaps[j].LIdx(), raw_gaps[i].RIdx());
                                auto farside_iter = std::min_element(stored_scan_msgs.ranges.begin() + start_idx, stored_scan_msgs.ranges.begin() + end_idx);
                                float min_dist = *farside_iter;
                                // second test is checking if simplified gap dist is less than current min dist of raw gap
                                // Changed this from - to + to make merging easier
                                bool simp_left_raw_right_dist_test = curr_rdist <= (min_dist - coefs * cfg_->rbt.r_inscr) && 
                                                                     simplified_gaps[j].RDist() <= (min_dist - coefs * cfg_->rbt.r_inscr);
                                
                                // left type just means if left side distance is less than right side distance
                                bool left_or_radial = simplified_gaps[j].isRightType() || !simplified_gaps[j].isAxial();
                                // making sure that this merged gap is not too large? max_idx_diff is 256 I'm pretty sure
                                bool idx_diff = raw_gaps[i].LIdx() - simplified_gaps[j].RIdx() < cfg_->gap_manip.max_idx_diff;
                                // ROS_INFO_STREAM("simp_left_raw_right_dist_test: " << simp_left_raw_right_dist_test << ", left_or_radial: " << left_or_radial << ", idx_diff: " << idx_diff);
                                if (simp_left_raw_right_dist_test && left_or_radial && idx_diff) {
                                    last_mergable = j;
                                } 
                            }

                            if (last_mergable != -1) {
                                // ROS_INFO_STREAM("erasing simplified gaps from " << (last_mergable + 1) << " to " << simplified_gaps.size());
                                simplified_gaps.erase(simplified_gaps.begin() + last_mergable + 1, simplified_gaps.end());
                                simplified_gaps.back().addLeftInformation(raw_gaps[i].LIdx(), raw_gaps[i].LDist());
                                // ROS_INFO_STREAM("merging simplified gap into (" << simplified_gaps.back().RIdx() << ", " << simplified_gaps.back().RDist() << ") to (" << simplified_gaps.back().LIdx() << ", " << simplified_gaps.back().LDist() << ")");
                            } else {
                                // ROS_INFO_STREAM("no merge, adding raw gap (swept, right): (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ") to (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ")");                            
                                simplified_gaps.push_back(raw_gaps.at(i));
                            }
                        }
                    }
                    else
                    { // If current raw gap is radial
                        float curr_rdist = raw_gaps.at(i).LDist();
                        // ROS_INFO_STREAM("comparing raw right dist of " << curr_rdist << " to simplified left dist of " << simplified_gaps.back().RDist());
                        if (std::abs(curr_rdist - simplified_gaps.back().RDist()) < 0.2 && simplified_gaps.back().isAxial() && simplified_gaps.back().isRightType())
                        {
                            // ROS_INFO_STREAM("adjusting simplifed gap to (" << simplified_gaps.back().RIdx() << ", " << simplified_gaps.back().RDist() << ") to (" << simplified_gaps.back().LIdx() << ", " << simplified_gaps.back().LDist() << ")");
                            simplified_gaps.back().addLeftInformation(raw_gaps[i].LIdx(), raw_gaps[i].LDist());
                        } else {
                            // ROS_INFO_STREAM("adding raw gap (radial): (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ") to (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ")");                            
                            simplified_gaps.push_back(raw_gaps[i]);

                            //std::cout << "adding raw gap from (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ") to (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ")" << std::endl;
                        }
                    }
                }
                else
                {
                    // Prior to marking start
                    simplified_gaps.push_back(raw_gaps[i]);
                    // ROS_INFO_STREAM("before marking start, adding raw gap: (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ") to (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ")");                            
                }
            }
            last_type_left = raw_gaps[i].isRightType();
            // ROS_INFO_STREAM("---");
        }
        //ROS_INFO_STREAM("mergeGapsOneGo time elapsed: " << ros::Time::now().toSec() - start_time); 

        // raw_gaps.clear();
        return simplified_gaps;
    }

}