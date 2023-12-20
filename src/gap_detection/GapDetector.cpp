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
        std::vector<dynamic_gap::Gap> rawGaps;

        try
        {
            ROS_INFO_STREAM_NAMED("GapDetector", "[gapDetection()]");
            scan_ = *scanPtr.get();
            // get half scan value
            fullScanRayCount_ = scan_.ranges.size();
            ROS_WARN_STREAM_COND_NAMED(fullScanRayCount_ != cfg_->scan.full_scan, "GapDetector", "Scan is wrong size, should be " << cfg_->scan.full_scan);

            halfScanRayCount_ = float(fullScanRayCount_ / 2);

            minScanDist_ = *std::min_element(scan_.ranges.begin(), scan_.ranges.end());
            maxScanDist_ = *std::max_element(scan_.ranges.begin(), scan_.ranges.end());
            ROS_INFO_STREAM_NAMED("GapDetector", "gapDetection min_dist: " << minScanDist_);

            std::string frame = scan_.header.frame_id;
            // starting the left point of the gap at front facing value
            // std::cout << "max laser scan range: " << scan.range_max << std::endl;
            int gapRIdx = 0;
            float gapRDist = scan_.ranges.at(0);
            // last as in previous scan
            bool withinSweptGap = gapRDist >= maxScanDist_;
            float currRayDist = scan_.ranges.at(0);
            float prevRayDist = currRayDist;

            // iterating through scan
            for (unsigned int it = 1; it < fullScanRayCount_; ++it)
            {
                currRayDist = scan_.ranges.at(it);
                ROS_INFO_STREAM_NAMED("GapDetector", "    iter: " << it << ", dist: " << currRayDist);

                if (radialGapSizeCheck(currRayDist, prevRayDist, scan_.angle_increment)) 
                {
                    // initializing a radial gap
                    dynamic_gap::Gap gap(frame, it - 1, prevRayDist, true, minScanDist_);
                    gap.addLeftInformation(it, currRayDist);
                    // gap.setRadial();

                    rawGaps.push_back(gap);

                    ROS_INFO_STREAM_NAMED("GapDetector", "    adding radial gap from: (" << gap.RIdx() << ", " << gap.RDist() << "), to (" << gap.LIdx() << ", " << gap.LDist() << ")");
                }

                // Either previous distance finite and current distance infinite or vice-versa, 
                if (sweptGapStartedOrEnded(currRayDist, prevRayDist))
                {
                    if (withinSweptGap) // Signals the ending of a gap
                    {
                        withinSweptGap = false;                    
                        ROS_INFO_STREAM_NAMED("GapDetector", "    gap ending: infinity to finite");
                        dynamic_gap::Gap gap(frame, gapRIdx, gapRDist, false, minScanDist_);
                        gap.addLeftInformation(it, currRayDist);
                        // gap.setRadial();

                        //std::cout << "candidate swept gap from (" << gapRIdx << ", " << gapRDist << "), to (" << it << ", " << scan_dist << ")" << std::endl;
                        // Inscribed radius gets enforced here, or unless using inflated egocircle, then no need for range diff
                        // Max: added first condition for if gap is sufficiently large. E.g. if agent directly behind robot, can get big gap but L/R points are close together
                        if (sweptGapSizeCheck(gap)) 
                        {
                            //std::cout << "adding candidate swept gap" << std::endl;
                            ROS_INFO_STREAM_NAMED("GapDetector", "    adding swept gap from: (" << gap.RIdx() << ", " << gap.RDist() << "), to (" << gap.LIdx() << ", " << gap.LDist() << ")");                
                            rawGaps.push_back(gap);
                        }
                    }
                    else // signals the beginning of a gap
                    {
                        ROS_INFO_STREAM_NAMED("GapDetector", "gap starting: finite to infinity");
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
                // ROS_INFO_STREAM_NAMED("GapDetector", "    catching last gap");
                dynamic_gap::Gap gap(frame, gapRIdx, gapRDist, false, minScanDist_);
                gap.addLeftInformation(fullScanRayCount_ - 1, *(scan_.ranges.end() - 1));
                // gap.setRadial();
                
                // ROS_INFO_STREAM_NAMED("GapDetector", "gapRIdx: " << gapRIdx << ", gapRDist: " << gapRDist);
                // ROS_INFO_STREAM_NAMED("GapDetector", "last_scan_idx: " << last_scan_idx << ", last_scan_dist: " << last_scan_dist);
                // ROS_INFO_STREAM_NAMED("GapDetector", "lidx: " << gap.LIdx() << ", ridx: " << gap.RIdx());
                // ROS_INFO_STREAM_NAMED("GapDetector", "gap side dist: " << gap_dist_side);
                if (sweptGapSizeCheck(gap)) 
                {
                    ROS_INFO_STREAM_NAMED("GapDetector", "    adding candidate last gap");
                    rawGaps.push_back(gap);
                    ROS_INFO_STREAM_NAMED("GapDetector", "adding last gap: (" << gap.RIdx() << ", " << gap.RDist() << "), to (" << gap.LIdx() << ", " << gap.LDist() << ")");                
                }
            }
            
            // Bridge the last gap around
            if (bridgeCondition(rawGaps))
            {
                ROS_INFO_STREAM_NAMED("GapDetector", "    bridging first and last gaps");
                rawGaps.back().addLeftInformation(rawGaps.front().LIdx(), rawGaps.front().LDist());
                rawGaps.erase(rawGaps.begin());
                ROS_INFO_STREAM_NAMED("GapDetector", "revising last gap: (" << rawGaps.back().RIdx() << ", " << rawGaps.back().RDist() << "), to (" << rawGaps.back().LIdx() << ", " << rawGaps.back().LDist() << ")");                
            }
            
            // if terminal_goal within laserscan and not within a gap, create a gap
            int globalGoalScanIdx;
            if (isGlobalGoalWithinGap(globalGoalRbtFrame, globalGoalScanIdx))
                addGapForGlobalGoal(globalGoalScanIdx, rawGaps);
        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapDetector", "[gapDetection() failed]");
        }

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
        ROS_INFO_STREAM_NAMED("GapDetector", "running addGapForGlobalGoal");
        ROS_INFO_STREAM_NAMED("GapDetector", "globalGoalScanIdx: " << globalGoalScanIdx);
        int gapIdx = 0;
        // int half_num_scan = scan_.ranges.size() / 2;
        // auto min_dist = *std::min_element(scan_.ranges.begin(), scan_.ranges.end());

        for (const dynamic_gap::Gap & rawGap : rawGaps) 
        {
            // if final_goal idx is within gap, return
            // ROS_INFO_STREAM_NAMED("GapDetector", "checking against: " << g.RIdx() << " to " << g.LIdx());
            if (globalGoalScanIdx >= rawGap.RIdx() && globalGoalScanIdx <= rawGap.LIdx()) 
            {
                ROS_INFO_STREAM_NAMED("GapDetector", "final goal is in gap: " << rawGap.RIdx() << ", " << rawGap.LIdx());
                return;
            }
            gapIdx += 1;
        }

        std::string frame = scan_.header.frame_id;
        int artificialGapIdxSpan = cfg_->scan.half_scan_f / 12;
        int rightIdx = std::max(globalGoalScanIdx - artificialGapIdxSpan, 0);
        int leftIdx = std::min(globalGoalScanIdx + artificialGapIdxSpan, cfg_->scan.full_scan - 1);
        ROS_INFO_STREAM_NAMED("GapDetector", "creating gap " << rightIdx << ", to " << leftIdx);

        dynamic_gap::Gap gap(frame, rightIdx, scan_.ranges.at(rightIdx), true, minScanDist_);
        gap.addLeftInformation(leftIdx, scan_.ranges.at(leftIdx));
        // gap.setRadial();
        
        gap.artificial_ = true;
        rawGaps.insert(rawGaps.begin() + gapIdx, gap);        
        return;
    }    

    ////////////////// GAP SIMPLIFICATION ///////////////////////

    // iterating backwards through simplified gaps to see if/where they can be merged
    int GapDetector::checkSimplifiedGapsMergeability(const dynamic_gap::Gap & rawGap, 
                                                     const std::vector<dynamic_gap::Gap> & simplifiedGaps)
    {
        int lastMergeable = -1;

        int startIdx, endIdx;
        // ROS_INFO_STREAM_NAMED("GapDetector", "attempting merge with raw gap: (" << rawGaps.at(i).RIdx() << ", " << rawGaps.at(i).RDist() << ") to (" << rawGaps.at(i).LIdx() << ", " << rawGaps.at(i).LDist() << ")");
        for (int j = (simplifiedGaps.size() - 1); j >= 0; j--)
        {
            // ROS_INFO_STREAM_NAMED("GapDetector", "on simplified gap " << j << " of " << simplifiedGaps.size() << ": ");
            // ROS_INFO_STREAM_NAMED("GapDetector", "points: (" << simplifiedGaps.at(j).RIdx() << ", " << simplifiedGaps.at(j).RDist() << ") to (" << simplifiedGaps.at(j).LIdx() << ", " << simplifiedGaps.at(j).LDist() << ")");
            startIdx = std::min(simplifiedGaps.at(j).LIdx(), rawGap.RIdx());
            endIdx = std::max(simplifiedGaps.at(j).LIdx(), rawGap.RIdx());
            float minIntergapRange = *std::min_element(scan_.ranges.begin() + startIdx, scan_.ranges.begin() + endIdx);
            float inflatedMinIntergapRange = minIntergapRange - 2 * cfg_->rbt.r_inscr;

            // 1. Checking if raw gap left and simplified gap right (widest distances, encompassing both gaps) dist 
            //    is less than the dist of whatever separates the two gaps
            bool intergapDistTest = rawGap.LDist() <= inflatedMinIntergapRange && 
                                    simplifiedGaps.at(j).RDist() <= inflatedMinIntergapRange;
            
            // 2. Checking if current simplified gap is either right dist < left dist or swept 
            bool rightTypeOrSweptGap = simplifiedGaps.at(j).isRightType() || !simplifiedGaps.at(j).isRadial();

            // 3. Making sure that this merged gap is not too large
            bool mergedGapSizeCheck = (rawGap.LIdx() - simplifiedGaps.at(j).RIdx()) < cfg_->gap_manip.max_idx_diff;

            // ROS_INFO_STREAM_NAMED("GapDetector", "simp_left_raw_right_dist_test: " << simp_left_raw_right_dist_test << ", rightTypeOrSweptGap: " << rightTypeOrSweptGap << ", mergedGapSizeCheck: " << mergedGapSizeCheck);
            if (intergapDistTest && rightTypeOrSweptGap && mergedGapSizeCheck)
                lastMergeable = j;
        }

        return lastMergeable;
    }

    bool GapDetector::mergeSweptGapCondition(const dynamic_gap::Gap & rawGap, 
                                             const std::vector<dynamic_gap::Gap> & simplifiedGaps)
    {
        // checking if difference between raw gap left dist and simplified gap right (widest distances, encompassing both gaps)
        // dist is sufficiently small (to fit robot)
        bool adjacentGapPtDistDiffCheck = std::abs(rawGap.LDist() - simplifiedGaps.back().RDist()) < 3 * cfg_->rbt.r_inscr;

        // checking if difference is sufficiently small, and that current simplified gap is radial and right dist < left dist
        return adjacentGapPtDistDiffCheck && simplifiedGaps.back().isRadial() && simplifiedGaps.back().isRightType();
    }

    std::vector<dynamic_gap::Gap> GapDetector::gapSimplification(const std::vector<dynamic_gap::Gap> & rawGaps)
    {
        ROS_INFO_STREAM_NAMED("GapDetector", "[gapSimplification()]");

        std::vector<dynamic_gap::Gap> simplifiedGaps;

        try
        {
            // Insert first
            bool markToStart = true;

            // float curr_left_dist = 0.0;
            int lastMergeable = -1;
            
            for (const dynamic_gap::Gap & rawGap : rawGaps)
            {
                // ROS_INFO_STREAM_NAMED("GapDetector", "on raw gap: (" << rawGap.RIdx() << ", " << rawGap.RDist() << ") to (" << rawGap.LIdx() << ", " << rawGap.LDist() << ")");
                
                if (markToStart)
                {   
                    // if we have not started simplification, this raw gap is swept, and right dist < left dist, then we can merge gaps
                    if (rawGap.isRadial() && rawGap.isRightType())
                    {
                        // ROS_INFO_STREAM_NAMED("GapDetector", "starting simplification");
                        markToStart = false;
                    }

                    simplifiedGaps.push_back(rawGap);
                } else {
                    if (rawGap.isRadial()) // if gap is radial
                    {
                        if (rawGap.isRightType()) // if right dist < left dist
                        {
                            // ROS_INFO_STREAM_NAMED("GapDetector", "adding raw gap (radial, right<left)");
                            simplifiedGaps.push_back(rawGap);
                        }
                        else
                        {
                            // curr_left_dist = rawGap.LDist();
                            lastMergeable = checkSimplifiedGapsMergeability(rawGap, simplifiedGaps);

                            if (lastMergeable != -1) 
                            {
                                // ROS_INFO_STREAM_NAMED("GapDetector", "erasing simplified gaps from " << (lastMergeable + 1) << " to " << simplifiedGaps.size());
                                simplifiedGaps.erase(simplifiedGaps.begin() + lastMergeable + 1, simplifiedGaps.end());
                                simplifiedGaps.back().addLeftInformation(rawGap.LIdx(), rawGap.LDist());
                                // simplifiedGaps.back().setRadial();
                                // ROS_INFO_STREAM_NAMED("GapDetector", "merging last simplified gap into (" << simplifiedGaps.back().RIdx() << ", " << simplifiedGaps.back().RDist() << ") to (" << simplifiedGaps.back().LIdx() << ", " << simplifiedGaps.back().LDist() << ")");
                            } else 
                            {
                                // ROS_INFO_STREAM_NAMED("GapDetector", "no merge, adding raw gap (swept, left<right)");                            
                                simplifiedGaps.push_back(rawGap);
                            }
                        }
                    }
                    else
                    { // If current raw gap is swept
                        // curr_left_dist = rawGap.LDist();
                        if (mergeSweptGapCondition(rawGap, simplifiedGaps))
                        {
                            simplifiedGaps.back().addLeftInformation(rawGap.LIdx(), rawGap.LDist());
                            // simplifiedGaps.back().setRadial();
                            // ROS_INFO_STREAM_NAMED("GapDetector", "merging last simplifed gap to (" << simplifiedGaps.back().RIdx() << ", " << simplifiedGaps.back().RDist() << ") to (" << simplifiedGaps.back().LIdx() << ", " << simplifiedGaps.back().LDist() << ")");
                        } else 
                        {
                            // ROS_INFO_STREAM_NAMED("GapDetector", "adding raw gap (swept)");                            
                            simplifiedGaps.push_back(rawGap);
                        }
                    }
                }
                // ROS_INFO_STREAM_NAMED("GapDetector", "---");
            }
        } catch (...)
        {
            ROS_ERROR_STREAM_NAMED("GapDetector", "[gapSimplification() failed]");
        }

        return simplifiedGaps;
    }
}