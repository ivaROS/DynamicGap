#include <dynamic_gap/scan_processing/DynamicScanPropagator.h>

namespace dynamic_gap
{
    DynamicScanPropagator::DynamicScanPropagator(ros::NodeHandle& nh, const DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;        
        propagatedEgocirclePublisher_ = nh.advertise<sensor_msgs::LaserScan>("propagated_egocircle", 1);
    }

    void DynamicScanPropagator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        scan_ = scan;
    }

    void DynamicScanPropagator::visualizePropagatedEgocircle(const sensor_msgs::LaserScan & propagatedScan) 
    {
        propagatedEgocirclePublisher_.publish(propagatedScan);

        // ROS_INFO_STREAM("propagated egocircle ranges: ");
        // for (int i = 0; i < propagatedScan.ranges.size(); i++)
        //     ROS_INFO_STREAM("       i: " << i << ", range: " <<  propagatedScan.ranges.at(i) << ", intensity: " << propagatedScan.intensities.at(i));

    }

    std::vector<sensor_msgs::LaserScan> DynamicScanPropagator::propagateCurrentLaserScan(const std::vector<dynamic_gap::Gap *> & rawGaps)
    {
        // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", " [propagateCurrentLaserScan]: ");

        sensor_msgs::LaserScan tmpScan = sensor_msgs::LaserScan();
        std::vector<sensor_msgs::LaserScan> futureScans(int(cfg_->traj.integrate_maxt/cfg_->traj.integrate_stept) + 1, tmpScan);
    
        // set first scan to current scan
        sensor_msgs::LaserScan scan = *scan_.get();

        futureScans.at(0) = scan; // at t = 0.0

        sensor_msgs::LaserScan visPropScan = scan;
        visPropScan.intensities.resize(visPropScan.ranges.size());

        // order models by index
        std::map<int, dynamic_gap::Estimator *> rawModels;
        for (const dynamic_gap::Gap * rawGap : rawGaps)
        {
            // left
            rawGap->leftGapPtModel_->isolateGapDynamics();
            float leftGapPtTheta = rawGap->leftGapPtModel_->getGapBearing();
            int leftGapPtIdx = theta2idx(leftGapPtTheta);

            if (leftGapPtIdx >= 0 && leftGapPtIdx < scan.ranges.size())
                rawModels.insert(std::pair<int, dynamic_gap::Estimator *>(leftGapPtIdx, rawGap->leftGapPtModel_));
            else
                ROS_WARN_STREAM_NAMED("DynamicScanPropagator", "        left gap pt idx out of bounds");

            // right
            rawGap->rightGapPtModel_->isolateGapDynamics();
            float rightGapPtTheta = rawGap->rightGapPtModel_->getGapBearing();
            int rightGapPtIdx = theta2idx(rightGapPtTheta);

            if (rightGapPtIdx >= 0 && rightGapPtIdx < scan.ranges.size())
                rawModels.insert(std::pair<int, dynamic_gap::Estimator *>(rightGapPtIdx, rawGap->rightGapPtModel_));
            else
                ROS_WARN_STREAM_NAMED("DynamicScanPropagator", "        right gap pt idx out of bounds");

        }

        // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "    rawModels: ");
        // for (std::map<int, dynamic_gap::Estimator *>::iterator iter = rawModels.begin();
        //             iter != rawModels.end();
        //             ++iter)
        // {
        //     ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        idx: " << iter->first << ", ID: " << iter->second->getID());
        //     ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "            model state: " << iter->second->getGapState().transpose());
        // }

        sensor_msgs::LaserScan defaultScan = scan;
        sensor_msgs::LaserScan wipedScan = scan;

        // indices for each point
        std::vector<int> pointwiseModelIndices(defaultScan.ranges.size());

        // for each point in scan
        // ROS_INFO_STREAM("defaultScan");
        for (int i = 0; i < defaultScan.ranges.size(); i++)
        {
            // ROS_INFO_STREAM("       i: " << i);
            // get right hand side model (model idx should be less than scan idx)
            int rightHandSideModelIdx = -1;
            std::map<int, dynamic_gap::Estimator *>::iterator right_iter;
            for (right_iter = rawModels.begin(); right_iter != rawModels.end(); ++right_iter)
            {
                // ROS_INFO_STREAM("           checking righthand model : " << right_iter->first);
                if (right_iter->first <= i)
                {
                    // ROS_INFO_STREAM("               replacing");
                    rightHandSideModelIdx = right_iter->first;
                    // no break, keep going
                }
            }
            
            if (rightHandSideModelIdx == -1) // no attachment
            {
                // ROS_INFO_STREAM("           no attachments for righthand model");            
                rightHandSideModelIdx = prev(rawModels.end())->first;
            }

            // get left hand side model
            int leftHandSideModelIdx = -1;
            std::map<int, dynamic_gap::Estimator *>::iterator left_iter;
            for (left_iter = rawModels.begin(); left_iter != rawModels.end(); ++left_iter)
            {
                // ROS_INFO_STREAM("           checking lefthand model : " << left_iter->first);

                if (left_iter->first >= i)
                {
                    // ROS_INFO_STREAM("           setting");
                    leftHandSideModelIdx = left_iter->first;
                    break;
                }
            }
            
            if (leftHandSideModelIdx == -1) // no attachment
            {
                // ROS_INFO_STREAM("           no attachments for lefthand model");                        
                leftHandSideModelIdx = rawModels.begin()->first;
            }
            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        at scan idx: " << i << ", LHS model idx: " << leftHandSideModelIdx << ", RHS model idx: " << rightHandSideModelIdx);

            // run distance check on LHS and RHS model positions
            float range = defaultScan.ranges.at(i);
            float theta = idx2theta(i);

            Eigen::Vector2f scanPt(range*cos(theta), range*sin(theta));

            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        scanPt: " << scanPt.transpose());

            Eigen::Vector2f lhsPt = rawModels.at(leftHandSideModelIdx)->getGapPosition();
            Eigen::Vector2f lhsVel = rawModels.at(leftHandSideModelIdx)->getGapVelocity();
            Eigen::Vector2f rhsPt = rawModels.at(rightHandSideModelIdx)->getGapPosition();
            Eigen::Vector2f rhsVel = rawModels.at(rightHandSideModelIdx)->getGapVelocity();

            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        lhsPt: " << lhsPt.transpose());
            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        lhsVel: " << lhsVel.transpose());
            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        rhsPt: " << rhsPt.transpose());
            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        rhsVel: " << rhsVel.transpose());

            // float scanToLHSDist = (scanPt - lhsPt).norm();
            // float scanToRHSDist = (scanPt - rhsPt).norm();

            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        LHS model dist: " << scanToLHSDist << 
            //                                                      ", RHS model dist: " << scanToRHSDist);

            // bit hacky, we just know moving obstacles will be roughly robot sized
            // bool distCheck = (scanToLHSDist < 3 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio && 
            //                   scanToRHSDist < 3 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio);
            bool distCheck = (lhsPt - rhsPt).norm() < 4 * cfg_->rbt.r_inscr * cfg_->traj.inf_ratio;
            bool speedCheck = (lhsVel.norm() >= 0.10 && rhsVel.norm() >= 0.10);
            // run angle check on LHS and RHS model velocities

            float vectorProj = lhsVel.dot(rhsVel) / (lhsVel.norm() * rhsVel.norm() + eps);
            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        vectorProj: " << vectorProj);
            bool angleCheck = (vectorProj > 0.0);

            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        distCheck: " << distCheck << ", speedCheck: " << speedCheck << ", angleCheck: " << angleCheck);

            // if attached
            if (distCheck && speedCheck && angleCheck)
            {
                // set default scan range to max
                wipedScan.ranges.at(i) = cfg_->scan.range_max; // set to max range
                visPropScan.intensities.at(i) = 255;

                // set pointwise model index
                if ( (scanPt - lhsPt).norm() < (scanPt - rhsPt).norm())
                    pointwiseModelIndices.at(i) = leftHandSideModelIdx; // attach model
                else
                    pointwiseModelIndices.at(i) = rightHandSideModelIdx; // attach model
                // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        attaching scan idx: " << i << " to model position :" << lhsPt.transpose() << " and velocity: " << lhsVel.transpose());

            } else
            {
                // set pointwise model index
                pointwiseModelIndices.at(i) = -1; // attach no model
            }
    
        }

        // for each timestep
        float t_i = 0.0, t_iplus1 = 0.0;
        for (int futureScanTimeIdx = 1; futureScanTimeIdx < futureScans.size(); futureScanTimeIdx++) 
        {        
            t_iplus1 = t_i + cfg_->traj.integrate_stept;
            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        propagating scan at t: " << t_iplus1);

            sensor_msgs::LaserScan propagatedScan = wipedScan;

            // for each point in scan
            for (int i = 0; i < defaultScan.ranges.size(); i++)
            {            
                if (pointwiseModelIndices.at(i) > 0)
                {
                    // polar to cartesian
                    float range = defaultScan.ranges.at(i);
                    float theta = idx2theta(i);

                    Eigen::Vector2f scanPt(range*cos(theta), range*sin(theta));
                    Eigen::Vector2f attachedPos = rawModels[pointwiseModelIndices.at(i)]->getGapPosition();
                    Eigen::Vector2f attachedVel = rawModels[pointwiseModelIndices.at(i)]->getGapVelocity();
                    // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "            pointwiseModelIndices.at(i): " << pointwiseModelIndices.at(i));
                    // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "            attachedPos: " << attachedPos.transpose());
                    // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "            attachedVel: " << attachedVel.transpose());

                    // propagate in cartesian
                    Eigen::Vector2f propagatedPt = scanPt + t_iplus1 * attachedVel;
                    // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "            propagating scan point: " << scanPt.transpose() << " to " << propagatedPt.transpose());

                    // cartesian to polar
                    float propagatedTheta = std::atan2(propagatedPt[1], propagatedPt[0]);
                    int propagatedIdx = theta2idx(propagatedTheta);
                    float propagatedNorm = propagatedPt.norm();

                    if (propagatedIdx >= 0 && propagatedIdx < propagatedScan.ranges.size())
                    {
                        // if (propagated range at theta is shorter than existing range at theta)
                        if (propagatedScan.ranges.at(propagatedIdx) > propagatedNorm)
                        {
                            // update
                            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "            at idx: " << i << ", replacing range " << propagatedScan.ranges.at(propagatedIdx) << " with " << propagatedNorm);
                            propagatedScan.ranges.at(propagatedIdx) = propagatedNorm;
                        }
                    } else
                    {
                        // ROS_WARN_STREAM_NAMED("DynamicScanPropagator", "            propagated index out of bounds");
                        // ROS_WARN_STREAM_NAMED("DynamicScanPropagator", "                propagated point" << propagatedPt.transpose());
                        // ROS_WARN_STREAM_NAMED("DynamicScanPropagator", "                propagated theta" << propagatedTheta);
                        // ROS_WARN_STREAM_NAMED("DynamicScanPropagator", "                propagated idx" << propagatedIdx);
                    
                    }
                }
            }

            futureScans.at(futureScanTimeIdx) = propagatedScan;

            t_i = t_iplus1;
        }

        // for the *first* entry to futureScans, we set the intensity values
        // to max for the scan points that *are* attached to models, meaning the
        // scan points that we estimate to be dynamic, we then visualize this scan
        // to verify what portions of the scan are being estimated as dynamic


        visualizePropagatedEgocircle(visPropScan);        

        return futureScans;
    }
}