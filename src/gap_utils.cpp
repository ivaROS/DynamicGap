#include <dynamic_gap/gap_utils.h>

namespace dynamic_gap {
    GapUtils::GapUtils() {}

    GapUtils::~GapUtils() {}

    GapUtils::GapUtils(const DynamicGapConfig& cfg) {
        cfg_ = & cfg;
    }
    
    void GapUtils::bisectNonConvexGap(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<dynamic_gap::Gap> & observed_gaps) {
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        std::string frame = stored_scan_msgs.header.frame_id;
        float half_scan = float(stored_scan_msgs.ranges.size() / 2);
        auto min_dist = *std::min_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.end());

        for (int i = 0; i < (int) observed_gaps.size(); i++)
        {
            std::cout << "checking bisect of single: (" << observed_gaps.at(i).LIdx() << ", " << observed_gaps.at(i).RIdx() << std::endl;
            bool needToBisect = observed_gaps.at(i).RIdx() - observed_gaps.at(i).LIdx() > half_scan;
            if (needToBisect) {
                dynamic_gap::Gap gap = observed_gaps.at(i);
                auto half_num_scan = gap.half_scan;
                float x1, x2, y1, y2;
                x1 = (gap.convex.convex_ldist) * cos(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
                y1 = (gap.convex.convex_ldist) * sin(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);

                x2 = (gap.convex.convex_rdist) * cos(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
                y2 = (gap.convex.convex_rdist) * sin(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
                std::cout << "bisecting x1, y1: (" << x1 << ", " << y1 << "), x2,y2: (" << x2 << ", " << y2 << ")" << std::endl; 
                Eigen::Vector2f pl(x1, y1);
                Eigen::Vector2f pr(x2, y2);

                // this means that the idces of gap say it's convex, but the local goal is put on non-convex side, making it non-convex.
                double l_idx = gap.LIdx();
                double r_idx = gap.RIdx();
                // std::cout << "l_idx: " << l_idx << ", r_idx:" << r_idx << std::endl;
                double l_dist = gap.LDist();
                double r_dist = gap.RDist();
                int mid_idx;

                if (l_idx < half_num_scan && r_idx > half_num_scan) {
                    // straddling
                    mid_idx = std::floor(l_idx + r_idx) / 2;
                } else {
                    mid_idx = half_num_scan*2 - std::floor(l_idx + r_idx) / 2;
                }
                //std::cout << "mid_idx: " << mid_idx << std::endl;
                
                double new_dist = stored_scan_msgs.ranges.at(mid_idx);
                double next_new_dist = stored_scan_msgs.ranges.at(mid_idx + 1);
                //std::cout << "new_dist: " << new_dist << ", next_new_dist: " << next_new_dist << std::endl;
                
                // straddling
                // have to initialize to get models
                // make first gap
                dynamic_gap::Gap first_gap(frame, l_idx, l_dist, gap.half_scan);
                // std::cout << "creating first gap" << std::endl;
                first_gap.addRightInformation(mid_idx, new_dist);
                first_gap.setMinSafeDist(min_dist);
                // make second gap
                // std::cout << "creating second gap" << std::endl;
                dynamic_gap::Gap second_gap(frame, mid_idx + 1, next_new_dist, gap.half_scan);
                second_gap.addRightInformation(r_idx, r_dist);
                second_gap.setMinSafeDist(min_dist);
                
                observed_gaps.at(i) = first_gap;
                observed_gaps.push_back(second_gap);
                float first_x1,first_y1,first_x2,first_y2;
                first_x1 = (first_gap.convex.convex_ldist) * cos(-((float) half_num_scan - first_gap.convex.convex_lidx) / half_num_scan * M_PI);
                first_y1 = (first_gap.convex.convex_ldist) * sin(-((float) half_num_scan - first_gap.convex.convex_lidx) / half_num_scan * M_PI);

                first_x2 = (first_gap.convex.convex_rdist) * cos(-((float) half_num_scan - first_gap.convex.convex_ridx) / half_num_scan * M_PI);
                first_y2 = (first_gap.convex.convex_rdist) * sin(-((float) half_num_scan - first_gap.convex.convex_ridx) / half_num_scan * M_PI);
                std::cout << "first bisector: (" << first_x1 << ", " << first_y1 << "), (" << first_x2 << ", " << first_y2 << ")" << std::endl;
                
                float second_x1,second_y1,second_x2,second_y2;
                second_x1 = (second_gap.convex.convex_ldist) * cos(-((float) half_num_scan - second_gap.convex.convex_lidx) / half_num_scan * M_PI);
                second_y1 = (second_gap.convex.convex_ldist) * sin(-((float) half_num_scan - second_gap.convex.convex_lidx) / half_num_scan * M_PI);

                second_x2 = (second_gap.convex.convex_rdist) * cos(-((float) half_num_scan - second_gap.convex.convex_ridx) / half_num_scan * M_PI);
                second_y2 = (second_gap.convex.convex_rdist) * sin(-((float) half_num_scan - second_gap.convex.convex_ridx) / half_num_scan * M_PI);
                std::cout << "second bisector: (" << second_x1 << ", " << second_y1 << "), (" << second_x2 << ", " << second_y2 << ")" << std::endl;


                // std::cout << "adding new gaps" << std::endl;
                return;
            }
        }
    }

    void GapUtils::hybridScanGap(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<dynamic_gap::Gap> & observed_gaps)
    {
        // clear gaps
        observed_gaps.clear();
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
                    dynamic_gap::Gap detected_gap(frame, it - 1, last_scan, true, half_scan);
                    detected_gap.addRightInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    if (detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr || cfg_->planning.planning_inflated) observed_gaps.push_back(detected_gap);
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
                    dynamic_gap::Gap detected_gap(frame, gap_lidx, gap_ldist, half_scan);
                    detected_gap.addRightInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    if (detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr || cfg_->planning.planning_inflated) observed_gaps.push_back(detected_gap);
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
            dynamic_gap::Gap detected_gap(frame, gap_lidx, gap_ldist, half_scan);
            detected_gap.addRightInformation(int(stored_scan_msgs.ranges.size() - 1), *(stored_scan_msgs.ranges.end() - 1));
            detected_gap.setMinSafeDist(min_dist);
            if (detected_gap._right_idx - detected_gap._left_idx > 500 || detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr) observed_gaps.push_back(detected_gap);
        }
        
        // Bridge the last gap around
        if (observed_gaps.size() > 1)
        {
            if (observed_gaps[0].LIdx() == 0 && observed_gaps[observed_gaps.size() - 1].RIdx() == stored_scan_msgs.ranges.size() - 1) // Magic number?
            {
                // Both ends
                float start_side_dist = observed_gaps[0].RDist();
                float end_side_dist = observed_gaps[observed_gaps.size() - 1].LDist();
                int start_side_idx = observed_gaps[0].RIdx();
                int end_side_idx = observed_gaps[observed_gaps.size() - 1].LIdx();

                int total_size = 511 - end_side_idx + start_side_idx;
                float result = (end_side_dist - start_side_dist) * (float (start_side_idx) / float (total_size)) + start_side_dist;
                observed_gaps[0].setLeftObs();
                observed_gaps[observed_gaps.size() - 1].setRightObs();
                observed_gaps[observed_gaps.size() - 1].addRightInformation(511, result);
                observed_gaps[0].setLDist(result);
            }
        }
    }

    void GapUtils::mergeGapsOneGo(
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<dynamic_gap::Gap>& observed_gaps)
    {
        // int left_idx = -1;
        // int right_idx = -1;
        // float right_dist = 3;
        // float left_dist = 3; // TODO: Make this reconfigurable
        int observed_size = (int) observed_gaps.size();
        std::vector<dynamic_gap::Gap> second_gap;

        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        // Termination Condition

        // Insert first
        bool mark_to_start = true;
        bool last_type_left = true;
        int left_counter = 0;
        bool changed = true;
        for (int i = 0; i < (int) observed_gaps.size(); i++)
        {
            if (mark_to_start && observed_gaps.at(i).isAxial() && observed_gaps.at(i).isLeftType())
            {
                // Wait until the first mergable gap aka swept left type gap
                mark_to_start = false;
                second_gap.push_back(observed_gaps[i]);
            } else {
                if (!mark_to_start)
                {
                    if (observed_gaps.at(i).isAxial())
                    {
                        if (observed_gaps.at(i).isLeftType())
                        {
                            second_gap.push_back(observed_gaps[i]);
                        }
                        else
                        {
                            float curr_rdist = observed_gaps[i].RDist();
                            int erase_counter = 0;
                            int last_mergable = -1;

                            float coefs = cfg_->planning.planning_inflated ? 0 : 2;
                            for (int j = (int) (second_gap.size() - 1); j >= 0; j--)
                            {
                                int start_idx = std::min(second_gap[j].RIdx(), observed_gaps[i].LIdx());
                                int end_idx = std::max(second_gap[j].RIdx(), observed_gaps[i].LIdx());
                                auto farside_iter = std::min_element(stored_scan_msgs.ranges.begin() + start_idx, stored_scan_msgs.ranges.begin() + end_idx);
                                bool second_test = curr_rdist <= (*farside_iter - coefs * cfg_->rbt.r_inscr) && second_gap[j].LDist() <= (*farside_iter - coefs * cfg_->rbt.r_inscr);
                                bool dist_diff = second_gap[j].isLeftType() || !second_gap[j].isAxial();
                                bool idx_diff = observed_gaps[i].RIdx() - second_gap[j].LIdx() < cfg_->gap_manip.max_idx_diff;
                                if (second_test && dist_diff && idx_diff) {
                                    last_mergable = j;
                                } 
                            }

                            if (last_mergable != -1) {
                                second_gap.erase(second_gap.begin() + last_mergable + 1, second_gap.end());
                                second_gap.back().addRightInformation(observed_gaps[i].RIdx(), observed_gaps[i].RDist());
                            } else {
                                second_gap.push_back(observed_gaps.at(i));
                            }
                        }
                    }
                    else
                    {
                        // If not axial gap, 
                        float curr_rdist = observed_gaps.at(i).RDist();
                        if (std::abs(curr_rdist - second_gap.back().LDist()) < 0.2 && second_gap.back().isAxial() && second_gap.back().isLeftType())
                        {
                            second_gap.back().addRightInformation(observed_gaps[i].RIdx(), observed_gaps[i].RDist());
                        } else {
                            second_gap.push_back(observed_gaps[i]);
                        }
                    }
                }
                else
                {
                    // A swept gap solely on its own
                    second_gap.push_back(observed_gaps[i]);
                }
            }
            last_type_left = observed_gaps[i].isLeftType();
        }
        observed_gaps.clear();
        observed_gaps = second_gap;

    }
}