#include <dynamic_gap/gap_utils.h>

namespace dynamic_gap {
    GapUtils::GapUtils() {}

    GapUtils::~GapUtils() {}

    GapUtils::GapUtils(const DynamicGapConfig& cfg) {
        cfg_ = & cfg;
    }

    std::vector<dynamic_gap::Gap> GapUtils::hybridScanGap(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser, int * model_idx)
    {
        // clear gaps
        std::vector<dynamic_gap::Gap> raw_gaps;
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        // get half scan value
        float half_scan = float(stored_scan_msgs.ranges.size() / 2);
        bool prev = true;
        auto max_dist_iter = std::max_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.end());
        float max_scan_dist = *max_dist_iter;
        auto min_dist = *std::min_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.end());
        int gap_size = 0;
        std::string frame = stored_scan_msgs.header.frame_id;
        // starting the left point of the gap at front facing value
        int gap_lidx = 0;
        float gap_ldist = stored_scan_msgs.ranges[0];
        // last as in previous scan
        float last_scan = stored_scan_msgs.ranges[0];
        bool prev_lgap = gap_ldist >= max_scan_dist;
        float scan_dist;
        float scan_diff;
        int wrap = 0;

        // iterating through scan
        for (std::vector<float>::size_type it = 1; it < stored_scan_msgs.ranges.size(); ++it)
        {
            scan_dist = stored_scan_msgs.ranges[it];
            // difference between current and previous rays
            scan_diff = scan_dist - last_scan;
            
            // Arbitrary small threshold for a range difference to be considered
            if (std::abs(scan_diff) > 0.2) 
            {
                // If both current and last values are not infinity, meaning this is not a swept gap
                if (scan_dist < max_scan_dist && last_scan < max_scan_dist) 
                {
                    // initializing a radial gap
                    // std::cout << "A. before constructor, model_idx: " << model_idx << std::endl;
                    dynamic_gap::Gap detected_gap(frame, it - 1, last_scan, true, half_scan, model_idx);
                    // model_idx += 2;
                    detected_gap.addRightInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    if (detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr || cfg_->planning.planning_inflated) raw_gaps.push_back(detected_gap);
                }
                
            }

            // Beginning or the end of a reading into infinity => swept gap
            // either last scan finite and current scan infinite or vice-versa, 
            if (last_scan < max_scan_dist != scan_dist < max_scan_dist)
            {
                // If previously marked gap, meaning ending of a gap
                if (prev_lgap)
                {
                    prev_lgap = false;
                    // std::cout << "B. before constructor, model_idx: " << model_idx << std::endl;
                    dynamic_gap::Gap detected_gap(frame, gap_lidx, gap_ldist, false, half_scan, model_idx);
                    // model_idx += 2;
                    detected_gap.addRightInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    if (detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr || cfg_->planning.planning_inflated) raw_gaps.push_back(detected_gap);
                }
                else // previously not marked a gap, not marking the gap
                {
                    gap_lidx = it - 1;
                    gap_ldist = last_scan;
                    prev_lgap = true;
                }
            }
            last_scan = scan_dist;
        }

        // Catch the last gap
        if (prev_lgap) 
        {
            // std::cout << "C. before constructor, model_idx: " << model_idx << std::endl;
            dynamic_gap::Gap detected_gap(frame, gap_lidx, gap_ldist, false, half_scan, model_idx);
            // model_idx += 2;
            detected_gap.addRightInformation(int(stored_scan_msgs.ranges.size() - 1), *(stored_scan_msgs.ranges.end() - 1));
            detected_gap.setMinSafeDist(min_dist);
            if (detected_gap._right_idx - detected_gap._left_idx > 500 || detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr) raw_gaps.push_back(detected_gap);
        }
        
        // Bridge the last gap around
        if (raw_gaps.size() > 1)
        {
            if (raw_gaps[0].LIdx() == 0 && raw_gaps[raw_gaps.size() - 1].RIdx() == stored_scan_msgs.ranges.size() - 1) // Magic number?
            {
                // Both ends
                float start_side_dist = raw_gaps[0].RDist();
                float end_side_dist = raw_gaps[raw_gaps.size() - 1].LDist();
                int start_side_idx = raw_gaps[0].RIdx();
                int end_side_idx = raw_gaps[raw_gaps.size() - 1].LIdx();

                int total_size = 511 - end_side_idx + start_side_idx;
                float result = (end_side_dist - start_side_dist) * (float (start_side_idx) / float (total_size)) + start_side_dist;
                raw_gaps[0].setLeftObs();
                raw_gaps[raw_gaps.size() - 1].setRightObs();
                raw_gaps[raw_gaps.size() - 1].addRightInformation(511, result);
                raw_gaps[0].setLDist(result);
            }
        }
        
        return raw_gaps;
    }

    std::vector<dynamic_gap::Gap> GapUtils::mergeGapsOneGo(
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<dynamic_gap::Gap>& raw_gaps)
    {
        // int left_idx = -1;
        // int right_idx = -1;
        // float right_dist = 3;
        // float left_dist = 3; // TODO: Make this reconfigurable
        int observed_size = (int) raw_gaps.size();
        std::vector<dynamic_gap::Gap> simplified_gaps;

        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        // Termination Condition

        // Insert first
        bool mark_to_start = true;
        bool last_type_left = true;
        int left_counter = 0;
        bool changed = true;

        for (int i = 0; i < (int) raw_gaps.size(); i++)
        {
            // axial means swept gap
            if (mark_to_start && raw_gaps.at(i).isAxial() && raw_gaps.at(i).isLeftType())
            {
                // Wait until the first mergable gap aka swept left type gap
                mark_to_start = false;
                simplified_gaps.push_back(raw_gaps[i]);
            } else {
                if (!mark_to_start)
                {
                    if (raw_gaps.at(i).isAxial())
                    {
                        if (raw_gaps.at(i).isLeftType())
                        {
                            simplified_gaps.push_back(raw_gaps[i]);
                        }
                        else
                        {
                            float curr_rdist = raw_gaps[i].RDist();
                            int erase_counter = 0;
                            int last_mergable = -1;

                            float coefs = cfg_->planning.planning_inflated ? 0 : 2;
                            for (int j = (int) (simplified_gaps.size() - 1); j >= 0; j--)
                            {
                                int start_idx = std::min(simplified_gaps[j].RIdx(), raw_gaps[i].LIdx());
                                int end_idx = std::max(simplified_gaps[j].RIdx(), raw_gaps[i].LIdx());
                                auto farside_iter = std::min_element(stored_scan_msgs.ranges.begin() + start_idx, stored_scan_msgs.ranges.begin() + end_idx);
                                bool second_test = curr_rdist <= (*farside_iter - coefs * cfg_->rbt.r_inscr) && simplified_gaps[j].LDist() <= (*farside_iter - coefs * cfg_->rbt.r_inscr);
                                bool dist_diff = simplified_gaps[j].isLeftType() || !simplified_gaps[j].isAxial();
                                bool idx_diff = raw_gaps[i].RIdx() - simplified_gaps[j].LIdx() < cfg_->gap_manip.max_idx_diff;
                                if (second_test && dist_diff && idx_diff) {
                                    last_mergable = j;
                                } 
                            }

                            if (last_mergable != -1) {
                                simplified_gaps.erase(simplified_gaps.begin() + last_mergable + 1, simplified_gaps.end());
                                simplified_gaps.back().addRightInformation(raw_gaps[i].RIdx(), raw_gaps[i].RDist());
                            } else {
                                simplified_gaps.push_back(raw_gaps.at(i));
                            }
                        }
                    }
                    else
                    {
                        // If not axial gap, 
                        float curr_rdist = raw_gaps.at(i).RDist();
                        if (std::abs(curr_rdist - simplified_gaps.back().LDist()) < 0.2 && simplified_gaps.back().isAxial() && simplified_gaps.back().isLeftType())
                        {
                            simplified_gaps.back().addRightInformation(raw_gaps[i].RIdx(), raw_gaps[i].RDist());
                        } else {
                            simplified_gaps.push_back(raw_gaps[i]);
                        }
                    }
                }
                else
                {
                    // A swept gap solely on its own
                    simplified_gaps.push_back(raw_gaps[i]);
                }
            }
            last_type_left = raw_gaps[i].isLeftType();
        }

        // raw_gaps.clear();
        return simplified_gaps;

    }
}