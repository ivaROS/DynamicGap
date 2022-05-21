#include <dynamic_gap/gap_utils.h>

namespace dynamic_gap {
    GapUtils::GapUtils() {}

    GapUtils::~GapUtils() {}

    GapUtils::GapUtils(const DynamicGapConfig& cfg) {
        cfg_ = & cfg;
    }

    std::vector<dynamic_gap::Gap> GapUtils::hybridScanGap(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser)
    {
        //std::cout << "running hybridScanGap" << std::endl;
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
        // std::cout << "max laser scan range: " << stored_scan_msgs.range_max << std::endl;
        int gap_lidx = 0;
        float gap_ldist = stored_scan_msgs.ranges[0];
        // last as in previous scan
        float last_scan = stored_scan_msgs.ranges[0];
        bool prev_lgap = gap_ldist >= max_scan_dist;
        float scan_dist;
        float scan_diff;
        int wrap = 0;

        // iterating through scan
        //std::cout << "finding raw gaps: " << std::endl;
        for (std::vector<float>::size_type it = 1; it < stored_scan_msgs.ranges.size(); ++it)
        {
            scan_dist = stored_scan_msgs.ranges[it];
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
                    detected_gap.addRightInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    //std::cout << "candiate radial gap from (" << (it-1) << ", " << last_scan << "), to (" << it << ", " << scan_dist << ")" << std::endl;
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    if (detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr || cfg_->planning.planning_inflated) { 
                        //std::cout << "adding candiate radial gap" << std::endl;
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
                    prev_lgap = false;
                    dynamic_gap::Gap detected_gap(frame, gap_lidx, gap_ldist, false, half_scan);
                    detected_gap.addRightInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    //std::cout << "candidate swept gap from (" << gap_lidx << ", " << gap_ldist << "), to (" << it << ", " << scan_dist << ")" << std::endl;
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    if (detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr || cfg_->planning.planning_inflated) {
                        //std::cout << "adding candidate swept gap" << std::endl;
                        raw_gaps.push_back(detected_gap);
                    }
                }
                else // signals the beginning of a gap
                {
                    gap_lidx = it - 1;
                    gap_ldist = last_scan;
                    prev_lgap = true;
                }
            }
            last_scan = scan_dist;
        }

        // Catch the last gap (could be in the middle of a swept gap but laser scan ends)
        if (prev_lgap) 
        {
            //std::cout << "catching last gap" << std::endl;
            dynamic_gap::Gap detected_gap(frame, gap_lidx, gap_ldist, false, half_scan);
            int last_scan_idx = stored_scan_msgs.ranges.size() - 1;
            double last_scan_dist = *(stored_scan_msgs.ranges.end() - 1);
            detected_gap.addRightInformation(last_scan_idx, last_scan_dist);
            detected_gap.setMinSafeDist(min_dist);
            //std::cout << "candidate last gap from (" << gap_lidx << ", " << gap_ldist << "), to (" << last_scan_idx << ", " << last_scan_dist << ")" << std::endl;
            if (detected_gap._right_idx - detected_gap._left_idx > 500 || detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr) {
                //std::cout << "adding candidate last gap" << std::endl;
                raw_gaps.push_back(detected_gap);
            }
        }
        
        // Bridge the last gap around
        if (raw_gaps.size() > 1)
        {
            int last_raw_gap_idx = raw_gaps.size() - 1;
            //std::cout << "checking: " << raw_gaps[0].LIdx() << " and " << raw_gaps[raw_gaps.size() - 1].RIdx() << std::endl;
            if (raw_gaps[0].LIdx() == 0 && raw_gaps[last_raw_gap_idx].RIdx() == (stored_scan_msgs.ranges.size() - 1)) // Magic number?
            {
                //std::cout << "bridging first and last gaps" << std::endl;
                // Both ends
                // currently only changing distances?
                float start_side_dist = raw_gaps[0].RDist();
                float end_side_dist = raw_gaps[last_raw_gap_idx].LDist();
                int start_side_idx = raw_gaps[0].RIdx();
                int end_side_idx = raw_gaps[last_raw_gap_idx].LIdx();
                //std::cout << "original first gap: (" << raw_gaps[0].LIdx() << ", " << raw_gaps[0].LDist() << ") to (" << raw_gaps[0].RIdx() << ", " << raw_gaps[0].RDist()  << ")" << std::endl;
                //std::cout << "original last gap: (" << raw_gaps[raw_gaps.size() - 1].LIdx() << ", " << raw_gaps[raw_gaps.size() - 1].LDist() << ") to (" << raw_gaps[raw_gaps.size() - 1].RIdx() << ", " << raw_gaps[raw_gaps.size() - 1].RDist()  << ")" << std::endl;
                int total_size = (stored_scan_msgs.ranges.size() - 1) - end_side_idx + start_side_idx;
                float result = (end_side_dist - start_side_dist) * (float (start_side_idx) / float (total_size)) + start_side_dist;
                raw_gaps[0].setLeftObs();
                raw_gaps[raw_gaps.size() - 1].setRightObs();
                raw_gaps[raw_gaps.size() - 1].addRightInformation(511, result);
                raw_gaps[0].setLDist(result);
                //raw_gaps[last_raw_gap_idx].addRightInformation(start_side_idx, start_side_dist);
                //std::cout << "merging into last gap: (" << raw_gaps[last_raw_gap_idx].LIdx() << ", " << raw_gaps[last_raw_gap_idx].LDist() << ") to (" << raw_gaps[last_raw_gap_idx].RIdx() << ", " << raw_gaps[last_raw_gap_idx].RDist()  << ")" << std::endl;

                //raw_gaps.erase(raw_gaps.begin(), raw_gaps.begin() + 1);
                //std::cout << "revised first gap: (" << raw_gaps[0].LIdx() << ", " << raw_gaps[0].LDist() << ") to (" << raw_gaps[0].RIdx() << ", " << raw_gaps[0].RDist()  << ")" << std::endl;
            }
        }
        
        return raw_gaps;
    }

    std::vector<dynamic_gap::Gap> GapUtils::mergeGapsOneGo(
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<dynamic_gap::Gap>& raw_gaps)
    {
        int observed_size = (int) raw_gaps.size();
        std::vector<dynamic_gap::Gap> simplified_gaps;

        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        // Termination Condition

        // Insert first
        bool mark_to_start = true;
        bool last_type_left = true;
        int left_counter = 0;
        bool changed = true;

        //std::cout << "running MergeGapsOneGo: " << std::endl;
        for (int i = 0; i < (int) raw_gaps.size(); i++)
        {
            // if we are starting merging, this raw gap is swept, and left dist < right dist, then we may be able to merge
            if (mark_to_start && raw_gaps.at(i).isSwept() && raw_gaps.at(i).isLeftType())
            {
                mark_to_start = false;
                simplified_gaps.push_back(raw_gaps[i]);
                //std::cout << "adding first swept left raw gap: (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ") to (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ")" << std::endl;
            } else {
                if (!mark_to_start) // if we have already found a mergable gap
                {
                    if (raw_gaps.at(i).isSwept()) // if gap is swept
                    {
                        if (raw_gaps.at(i).isLeftType()) // if l_dist < r_dist
                        {
                            simplified_gaps.push_back(raw_gaps[i]);
                            //std::cout << "adding a raw gap (swept, left): (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ") to (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ")" << std::endl;
                            //std::cout << "adding raw gap from (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ") to (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ")" << std::endl;
                        }
                        else
                        {
                            float curr_rdist = raw_gaps[i].RDist();
                            int erase_counter = 0;
                            int last_mergable = -1;

                            float coefs = cfg_->planning.planning_inflated ? 0 : 2;
                            // iterating backwards through simplified gaps to see if they can be merged
                            for (int j = (int) (simplified_gaps.size() - 1); j >= 0; j--)
                            {
                                int start_idx = std::min(simplified_gaps[j].RIdx(), raw_gaps[i].LIdx());
                                int end_idx = std::max(simplified_gaps[j].RIdx(), raw_gaps[i].LIdx());
                                auto farside_iter = std::min_element(stored_scan_msgs.ranges.begin() + start_idx, stored_scan_msgs.ranges.begin() + end_idx);
                                float min_dist = *farside_iter;
                                // second test is checking if simplified gap dist is less than current min dist of raw gap
                                bool second_test = curr_rdist <= (min_dist - coefs * cfg_->rbt.r_inscr) && 
                                                   simplified_gaps[j].LDist() <= (min_dist - coefs * cfg_->rbt.r_inscr);
                                
                                // left type just means if left side distance is less than right side distance
                                bool dist_diff = simplified_gaps[j].isLeftType() || !simplified_gaps[j].isSwept();
                                // making sure that this merged gap is not too large?
                                bool idx_diff = raw_gaps[i].RIdx() - simplified_gaps[j].LIdx() < cfg_->gap_manip.max_idx_diff;
                                if (second_test && dist_diff && idx_diff) {
                                    last_mergable = j;
                                } 
                            }

                            if (last_mergable != -1) {
                                //std::cout << "erasing simplified gaps from " << (last_mergable + 1) << " to " << simplified_gaps.size() << std::endl;
                                simplified_gaps.erase(simplified_gaps.begin() + last_mergable + 1, simplified_gaps.end());
                                simplified_gaps.back().addRightInformation(raw_gaps[i].RIdx(), raw_gaps[i].RDist());
                                //std::cout << "merging simplified gap into (" << simplified_gaps.back().LIdx() << ", " << simplified_gaps.back().LDist() << ") to (" << simplified_gaps.back().RIdx() << ", " << simplified_gaps.back().RDist() << ")" << std::endl;
                            } else {
                                simplified_gaps.push_back(raw_gaps.at(i));
                                //std::cout << "no merge, adding raw gap (swept, right): (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ") to (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ")" << std::endl;                            
                                //std::cout << "adding raw gap from (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ") to (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ")" << std::endl;
                            }
                        }
                    }
                    else
                    { // If current raw gap is radial
                        float curr_rdist = raw_gaps.at(i).RDist();
                        if (std::abs(curr_rdist - simplified_gaps.back().LDist()) < 0.2 && simplified_gaps.back().isSwept() && simplified_gaps.back().isLeftType())
                        {
                            simplified_gaps.back().addRightInformation(raw_gaps[i].RIdx(), raw_gaps[i].RDist());
                            //std::cout << "adjusting simplifed gap to (" << simplified_gaps.back().LIdx() << ", " << simplified_gaps.back().LDist() << ") to (" << simplified_gaps.back().RIdx() << ", " << simplified_gaps.back().RDist() << ")" << std::endl;
                        } else {
                            simplified_gaps.push_back(raw_gaps[i]);
                            //std::cout << "adding raw gap (radial): (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ") to (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ")" << std::endl;                            

                            //std::cout << "adding raw gap from (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ") to (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ")" << std::endl;
                        }
                    }
                }
                else
                {
                    // Prior to marking start
                    simplified_gaps.push_back(raw_gaps[i]);
                    // std::cout << "before marking start, adding raw gap: (" << raw_gaps[i].LIdx() << ", " << raw_gaps[i].LDist() << ") to (" << raw_gaps[i].RIdx() << ", " << raw_gaps[i].RDist() << ")" << std::endl;                            
                }
            }
            last_type_left = raw_gaps[i].isLeftType();
        }

        // raw_gaps.clear();
        return simplified_gaps;

    }
}