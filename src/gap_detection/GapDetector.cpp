#include <dynamic_gap/gap_detection/GapDetector.h>

namespace dynamic_gap
{
    ////////////////// GAP DETECTION ///////////////////////
    
    /**
     * Checking if scan distance registers an obstacle (finite range value)
    **/
    bool GapDetector::isFinite(float rayDist)
    {
        return rayDist < maxScanDist_;
    }

    /**
     * Determining if swept gap has either started (finite scan --> infinite scan)
     * or ended (infinite scan --> finite scan)
    **/
    bool GapDetector::sweptGapStartedOrEnded(float currRayDist, float prevRayDist)
    {
        return isFinite(prevRayDist) != isFinite(currRayDist);
    }

    // Checking if swept gap is either very large, or if robot can fit within gap (precondition to swept gap)
    bool GapDetector::sweptGapSizeCheck(const dynamic_gap::Gap & gap)
    {
        bool largeGap = gap.LIdx() - gap.RIdx() > (3 * halfScanRayCount_ / 2);
        bool canRobotFit = gap.get_gap_euclidean_dist() > 3 * cfg_->rbt.r_inscr;
        
        return largeGap || canRobotFit;
    }

    // Checking if robot can fit between gap between consecutive scan points (precondition to radial gap)
    bool GapDetector::radialGapSizeCheck(float currRayDist, float prevRayDist, float gapAngle)
    {
        if (!(prevRayDist < maxScanDist_ && currRayDist < maxScanDist_))
            return false;

        // Euclidean distance between current and previous points
        float consecScanPointDist = sqrt(pow(prevRayDist, 2) + pow(currRayDist, 2) - 2 * prevRayDist * currRayDist * cos(gapAngle));

        bool canRobotFit = consecScanPointDist > 3 * cfg_->rbt.r_inscr;
        
        return canRobotFit;
    }   

    bool GapDetector::bridgeCondition(const std::vector<dynamic_gap::Gap> & rawGaps)
    {
        bool multipleGaps = rawGaps.size() > 1;
        bool firstAndLastGapsBorder = (rawGaps.front().RIdx() == 0 && 
                                          rawGaps.back().LIdx() == (fullScanRayCount_ - 1));
        
        return multipleGaps && firstAndLastGapsBorder;
    }

    std::vector<dynamic_gap::Gap> GapDetector::gapDetection(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr, 
                                                            geometry_msgs::PoseStamped globalGoalRbtFrame)
    {
        if (cfg_->debug.gap_detection_debug_log) ROS_INFO_STREAM("[gapDetection()]");
        std::vector<dynamic_gap::Gap> rawGaps;
        scan_ = *scanPtr.get();
        // get half scan value
        fullScanRayCount_ = scan_.ranges.size();
        halfScanRayCount_ = float(fullScanRayCount_ / 2);

        minScanDist_ = *std::min_element(scan_.ranges.begin(), scan_.ranges.end());
        maxScanDist_ = *std::max_element(scan_.ranges.begin(), scan_.ranges.end());
        // ROS_INFO_STREAM("gapDetection min_dist: " << min_dist);

        std::string frame = scan_.header.frame_id;
        // starting the left point of the gap at front facing value
        // std::cout << "max laser scan range: " << scan.range_max << std::endl;
        int gapRIdx = 0;
        float gapRDist = scan_.ranges[0];
        // last as in previous scan
        bool withinSweptGap = gapRDist >= maxScanDist_;
        float currRayDist = scan_.ranges[0];
        float prevRayDist = currRayDist;

        // iterating through scan
        for (unsigned int it = 1; it < fullScanRayCount_; ++it)
        {
            currRayDist = scan_.ranges[it];
            // if (cfg_->debug.gap_detection_debug_log) ROS_INFO_STREAM("    iter: " << it << ", dist: " << currRayDist);

            if (radialGapSizeCheck(currRayDist, prevRayDist, scan_.angle_increment)) 
            {
                // initializing a radial gap
                dynamic_gap::Gap gap(frame, it - 1, prevRayDist, true, halfScanRayCount_, minScanDist_);
                gap.addLeftInformation(it, currRayDist);
                gap.setRadial();

                rawGaps.push_back(gap);

                // if (cfg_->debug.gap_detection_debug_log) ROS_INFO_STREAM("    adding radial gap from: (" << gap.RIdx() << ", " << gap.RDist() << "), to (" << gap.LIdx() << ", " << gap.LDist() << ")");
            }

            // Either previous distance finite and current distance infinite or vice-versa, 
            if (sweptGapStartedOrEnded(currRayDist, prevRayDist))
            {
                if (withinSweptGap) // Signals the ending of a gap
                {
                    withinSweptGap = false;                    
                    if (cfg_->debug.gap_detection_debug_log) ROS_INFO_STREAM("    gap ending: infinity to finite");
                    dynamic_gap::Gap gap(frame, gapRIdx, gapRDist, false, halfScanRayCount_, minScanDist_);
                    gap.addLeftInformation(it, currRayDist);
                    gap.setRadial();

                    //std::cout << "candidate swept gap from (" << gapRIdx << ", " << gapRDist << "), to (" << it << ", " << scan_dist << ")" << std::endl;
                    // Inscribed radius gets enforced here, or unless using inflated egocircle, then no need for range diff
                    // Max: added first condition for if gap is sufficiently large. E.g. if agent directly behind robot, can get big gap but L/R points are close together
                    if (sweptGapSizeCheck(gap)) 
                    {
                        //std::cout << "adding candidate swept gap" << std::endl;
                        // if (cfg_->debug.gap_detection_debug_log) ROS_INFO_STREAM("    adding swept gap from: (" << gap.RIdx() << ", " << gap.RDist() << "), to (" << gap.LIdx() << ", " << gap.LDist() << ")");                
                        rawGaps.push_back(gap);
                    }
                }
                else // signals the beginning of a gap
                {
                    // ROS_INFO_STREAM("gap starting: finite to infinity");
                    gapRIdx = it - 1;
                    gapRDist = prevRayDist;
                    withinSweptGap = true;
                }

            }
            prevRayDist = currRayDist;
        }

        // Catch the last gap (could be in the middle of a swept gap when laser scan ends)
        if (withinSweptGap) 
        {
            // if (cfg_->debug.gap_detection_debug_log) ROS_INFO_STREAM("    catching last gap");
            dynamic_gap::Gap gap(frame, gapRIdx, gapRDist, false, halfScanRayCount_, minScanDist_);
            gap.addLeftInformation(fullScanRayCount_ - 1, *(scan_.ranges.end() - 1));
            gap.setRadial();
            
            // ROS_INFO_STREAM("gapRIdx: " << gapRIdx << ", gapRDist: " << gapRDist);
            // ROS_INFO_STREAM("last_scan_idx: " << last_scan_idx << ", last_scan_dist: " << last_scan_dist);
            // ROS_INFO_STREAM("lidx: " << gap.LIdx() << ", ridx: " << gap.RIdx());
            // ROS_INFO_STREAM("gap side dist: " << gap_dist_side);
            if (sweptGapSizeCheck(gap)) 
            {
                // if (cfg_->debug.gap_detection_debug_log) ROS_INFO_STREAM("    adding candidate last gap");
                rawGaps.push_back(gap);
                // ROS_INFO_STREAM("adding last gap: (" << gap.RIdx() << ", " << gap.RDist() << "), to (" << gap.LIdx() << ", " << gap.LDist() << ")");                
            }
        }
        
        // Bridge the last gap around
        if (bridgeCondition(rawGaps))
        {
            // if (cfg_->debug.gap_detection_debug_log) ROS_INFO_STREAM("    bridging first and last gaps");
            rawGaps.back().addLeftInformation(rawGaps.front().LIdx(), rawGaps.front().LDist());
            rawGaps.erase(rawGaps.begin());
            // ROS_INFO_STREAM("revising last gap: (" << rawGaps.back().RIdx() << ", " << rawGaps.back().RDist() << "), to (" << rawGaps.back().LIdx() << ", " << rawGaps.back().LDist() << ")");                
        }
        
        // if terminal_goal within laserscan and not within a gap, create a gap
        int globalGoalScanIdx;
        if (isGlobalGoalWithinGap(globalGoalRbtFrame, globalGoalScanIdx))
            addGapForGlobalGoal(globalGoalScanIdx, rawGaps);

        return rawGaps;
    }

    bool GapDetector::isGlobalGoalWithinGap(geometry_msgs::PoseStamped globalGoalRbtFrame,
                                            int & globalGoalScanIdx)
    {
        float finalGoalDist = sqrt(pow(globalGoalRbtFrame.pose.position.x, 2) + pow(globalGoalRbtFrame.pose.position.y, 2));
        float globalGoalOrientationRbtFrame = std::atan2(globalGoalRbtFrame.pose.position.y, globalGoalRbtFrame.pose.position.x);

        globalGoalScanIdx = int(std::floor((globalGoalOrientationRbtFrame + M_PI) / scan_.angle_increment));

        float globalGoalScanIdxDist = scan_.ranges.at(globalGoalScanIdx);

        return (finalGoalDist < globalGoalScanIdxDist);
    }   

    void GapDetector::addGapForGlobalGoal(int globalGoalScanIdx,
                                          std::vector<dynamic_gap::Gap> & rawGaps) 
    {
        ROS_INFO_STREAM("running addGapForGlobalGoal");
        ROS_INFO_STREAM("globalGoalScanIdx: " << globalGoalScanIdx);
        int gap_idx = 0;
        int half_num_scan = scan_.ranges.size() / 2;
        auto min_dist = *std::min_element(scan_.ranges.begin(), scan_.ranges.end());

        for (dynamic_gap::Gap g : rawGaps) {
            // if final_goal idx is within gap, return
            // ROS_INFO_STREAM("checking against: " << g.RIdx() << " to " << g.LIdx());
            if (globalGoalScanIdx >= g.RIdx() && globalGoalScanIdx <= g.LIdx()) {
                ROS_INFO_STREAM("final goal is in gap: " << g.RIdx() << ", " << g.LIdx());
                return;
            }
            gap_idx += 1;
        }

        std::string frame = scan_.header.frame_id;
        int artificial_gap_span = half_num_scan / 12;
        int right_idx = std::max(globalGoalScanIdx - artificial_gap_span, 0);
        int left_idx = std::min(globalGoalScanIdx + artificial_gap_span, 2*half_num_scan - 1);
        ROS_INFO_STREAM("creating gap " << right_idx << ", to " << left_idx);

        dynamic_gap::Gap gap(frame, right_idx, scan_.ranges.at(right_idx), true, half_num_scan, min_dist);
        gap.addLeftInformation(left_idx, scan_.ranges.at(left_idx));
        gap.setRadial();
        
        gap.artificial = true;
        rawGaps.insert(rawGaps.begin() + gap_idx, gap);        
        return;
    }    

    ////////////////// GAP SIMPLIFICATION ///////////////////////

    // iterating backwards through simplified gaps to see if/where they can be merged
    int GapDetector::checkSimplifiedGapsMergeability(const dynamic_gap::Gap & raw_gap, 
                                        const std::vector<dynamic_gap::Gap> & simplified_gaps)
    {
        int last_mergable = -1;

        int start_idx, end_idx;
        // ROS_INFO_STREAM("attempting merge with raw gap: (" << rawGaps[i].RIdx() << ", " << rawGaps[i].RDist() << ") to (" << rawGaps[i].LIdx() << ", " << rawGaps[i].LDist() << ")");
        for (int j = (simplified_gaps.size() - 1); j >= 0; j--)
        {
            // ROS_INFO_STREAM("on simplified gap " << j << " of " << simplified_gaps.size() << ": ");
            // ROS_INFO_STREAM("points: (" << simplified_gaps[j].RIdx() << ", " << simplified_gaps[j].RDist() << ") to (" << simplified_gaps[j].LIdx() << ", " << simplified_gaps[j].LDist() << ")");
            start_idx = std::min(simplified_gaps[j].LIdx(), raw_gap.RIdx());
            end_idx = std::max(simplified_gaps[j].LIdx(), raw_gap.RIdx());
            float min_intergap_dist = *std::min_element(scan_.ranges.begin() + start_idx, scan_.ranges.begin() + end_idx);
            float inflated_min_intergap_dist = min_intergap_dist - 2 * cfg_->rbt.r_inscr;

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

    bool GapDetector::mergeSweptGapCondition(const dynamic_gap::Gap & raw_gap, 
                                             const std::vector<dynamic_gap::Gap> & simplified_gaps)
    {
        // checking if difference between raw gap left dist and simplified gap right (widest distances, encompassing both gaps)
        // dist is sufficiently small (to fit robot)
        bool raw_left_simp_right_dist_diff_check = std::abs(raw_gap.LDist() - simplified_gaps.back().RDist()) < 3 * cfg_->rbt.r_inscr;

        // checking if difference is sufficiently small, and that current simplified gap is radial and right dist < left dist
        return raw_left_simp_right_dist_diff_check && simplified_gaps.back().isRadial() && simplified_gaps.back().isRightType();
    }

    std::vector<dynamic_gap::Gap> GapDetector::gapSimplification(const std::vector<dynamic_gap::Gap> & rawGaps)
    {
        if (cfg_->debug.gap_simplification_debug_log) ROS_INFO_STREAM("[gapSimplification()]");

        //double start_time = ros::Time::now().toSec();
        std::vector<dynamic_gap::Gap> simplified_gaps;

        // Insert first
        bool mark_to_start = true;

        float curr_left_dist = 0.0;
        int last_mergable = -1;
        
        for (dynamic_gap::Gap raw_gap : rawGaps)
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
                        last_mergable = checkSimplifiedGapsMergeability(raw_gap, simplified_gaps);

                        if (last_mergable != -1) {
                            // ROS_INFO_STREAM("erasing simplified gaps from " << (last_mergable + 1) << " to " << simplified_gaps.size());
                            simplified_gaps.erase(simplified_gaps.begin() + last_mergable + 1, simplified_gaps.end());
                            simplified_gaps.back().addLeftInformation(raw_gap.LIdx(), raw_gap.LDist());
                            simplified_gaps.back().setRadial();
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
                        simplified_gaps.back().setRadial();
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
}