#include <dynamic_gap/scan_processing/DynamicScanPropagator.h>

namespace dynamic_gap
{
    DynamicScanPropagator::DynamicScanPropagator(ros::NodeHandle& nh, const DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;        
        propagatedEgocirclePublisher_ = nh.advertise<sensor_msgs::LaserScan>("propagated_egocircle", 1);

        staticLaserSub_ = nh.subscribe("/robot" + std::to_string(cfg_->env.num_agents) + "/laser_0", 5, &DynamicScanPropagator::staticLaserScanCB, this);

        // TODO: change this to a vector of pointers to laser scans
        sensor_msgs::LaserScan tmpScan = sensor_msgs::LaserScan();
        futureScans_ = std::vector<sensor_msgs::LaserScan>(int(cfg_->traj.integrate_maxt/cfg_->traj.integrate_stept) + 1, tmpScan);
    }

    void DynamicScanPropagator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        // boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    void DynamicScanPropagator::staticLaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> staticScan)
    {
        // boost::mutex::scoped_lock lock(staticScanMutex_);
        staticScan_ = staticScan;        
    }

    void DynamicScanPropagator::visualizePropagatedEgocircle(const sensor_msgs::LaserScan & propagatedScan) 
    {
        propagatedEgocirclePublisher_.publish(propagatedScan);
    }

    std::vector<sensor_msgs::LaserScan> DynamicScanPropagator::propagateCurrentLaserScanCheat(const std::map<std::string, geometry_msgs::Pose> & currentTrueAgentPoses,
                                                                                                const std::map<std::string, geometry_msgs::Vector3Stamped> & currentTrueAgentVels)
    {
        // boost::mutex::scoped_lock staticScanLock(staticScanMutex_);
        // boost::mutex::scoped_lock scanLock(scanMutex_);

        try
        {
            ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "[getFutureScans()]");

            // set first scan to current scan
            sensor_msgs::LaserScan scan = *scan_.get();

            for (int i = 0; i < futureScans_.size(); i++)
                futureScans_.at(i) = scan;

            // futureScans_.at(0) = scan; // at t = 0.0

            // sensor_msgs::LaserScan staticScan = *staticScan_.get();

            // sensor_msgs::LaserScan propagatedScan = staticScan; // sensor_msgs::LaserScan();
            // propagatedScan.header.stamp = ros::Time::now();
            // propagatedScan.header.frame_id = cfg_->robot_frame_id;
            // propagatedScan.header.seq = 0;
            // // propagatedScan.angle_min = scan.angle_min;
            // // propagatedScan.angle_max = scan.angle_max;
            // // propagatedScan.angle_increment = scan.angle_increment;
            // // propagatedScan.time_increment = scan.time_increment;
            // // propagatedScan.scan_time = scan.scan_time;
            // // propagatedScan.range_min = scan.range_min;
            // // propagatedScan.range_max = scan.range_max;
            // // propagatedScan.ranges = scan.ranges;


            // std::vector<Eigen::Vector4f> currentAgents;
            
            // Eigen::Vector4f ithAgentState;
            // for ( const auto &pair : currentTrueAgentPoses ) 
            // {
            // // for (itr = currentTrueAgentPoses.begin(); itr != currentTrueAgentPoses.end(); ++itr)     
            // // {        
            // // for (int i = 0; i < currentTrueAgentPoses.size(); i++) 
            // // {
            //     std::string key = pair.first;
            //     geometry_msgs::Pose agentPose = pair.second;
            //     geometry_msgs::Vector3Stamped agentVel = currentTrueAgentVels.find(key)->second;
            //     ithAgentState << agentPose.position.x, agentPose.position.y, 
            //                         agentVel.vector.x, agentVel.vector.y;
            //     currentAgents.push_back(ithAgentState);
            // }

            // // ROS_INFO_STREAM_NAMED("propagatedScanPropagator", "    detected agents: ");
            // // for (int i = 0; i < currentAgents.size(); i++)
            // //     ROS_INFO_STREAM_NAMED("propagatedScanPropagator", "        agent" << i << " position: " << currentAgents.at(i)[0] << ", " << currentAgents.at(i)[1] << ", velocity: " << currentAgents.at(i)[2] << ", " << currentAgents.at(i)[3]);
            
            // float t_i = 0.0, t_iplus1 = 0.0;
            // for (int futureScanTimeIdx = 1; futureScanTimeIdx < futureScans_.size(); futureScanTimeIdx++) 
            // {
            //     t_iplus1 = t_i + cfg_->traj.integrate_stept;
                
            //     propagatedScan.ranges = staticScan.ranges; // reset scan ranges

            //     recoverDynamicEgocircle(t_i, t_iplus1, currentAgents, propagatedScan);
                
            //     // futureScanTimeIdx = (int) (t_iplus1 / cfg_->traj.integrate_stept);
            //     // ROS_INFO_STREAM("adding scan from " << t_i << " to " << t_iplus1 << " at idx: " << futureScanTimeIdx);
            //     futureScans_.at(futureScanTimeIdx) = propagatedScan;

            //     t_i = t_iplus1;
            // }
            // visualizePropagatedEgocircle(futureScans_.at(futureScans_.size() - 1));        
        } catch (...)
        {
            ROS_WARN_STREAM_NAMED("DynamicScanPropagator", "  getFutureScans failed");
        }

        return futureScans_;
    }

    std::vector<Eigen::Vector4f> DynamicScanPropagator::sortAndPrune(const std::vector<Eigen::Vector4f> & agentPoses)
    {
        // Declare vector of pairs
        std::vector< Eigen::Vector4f > A;
        
        // Copy key-value pair from Map
        // to vector of pairs
        float dist = 0.0;
        for (int i = 0; i < agentPoses.size(); i++) 
        {
            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", it.first);
            //ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "pose: " << std::to_string(it.second[0]) << ", " << std::to_string(it.second[1]));
            //ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "ego pose: " << pt1[0] << ", " << pt1[1]);

            // std::vector<float> agentPose{agentPoses.at(i)[0], agentPoses.at(i)[1]};
            
            dist = sqrt(pow(agentPoses[i][0], 2) + pow(agentPoses[i][1], 2));
            //ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "dist: " << dist);
            if (dist < cfg_->scan.range_max) 
            {
                A.push_back(agentPoses[i]);
            }
        }
        
        // Sort using comparator function
        sort(A.begin(), A.end(), Comparator());
        
        // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "printing pruned vect");
        // Print the sorted value
        /*
        for (auto& it : A) {
    
            ROS_INFO_STREAM_NAMED("DynamicScanPropagator", it.first << ' ' << it.second[0] << ", " << it.second[1]);
        }
        */
        

        return A;
    } 

    void DynamicScanPropagator::recoverDynamicEgocircle(const float & t_i, 
                                                        const float & t_iplus1, 
                                                        std::vector<Eigen::Vector4f> & propagatedAgents,
                                                        sensor_msgs::LaserScan & propagatedScan) 
    {   
        // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "    [recoverDynamicEgocircle()]");
        float interval = t_iplus1 - t_i;
        if (interval <= 0.0)
            return;

        // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        recovering dynamic egocircle for interval: " << t_i << " to " << t_iplus1);

        // propagate poses forward (all odoms and vels are in robot frame)
        for (int i = 0; i < propagatedAgents.size(); i++) 
        {
            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "            agent" << i << " moving from (" << propagatedAgents.at(i)[0] << ", " << propagatedAgents.at(i)[1] << ")");

            propagatedAgents.at(i)[0] += propagatedAgents.at(i)[2]*interval;
            propagatedAgents.at(i)[1] += propagatedAgents.at(i)[3]*interval;
            
            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "                       (" << propagatedAgents.at(i)[0] << ", " << propagatedAgents.at(i)[1] << ")");
        }

        // basically running modify_scan                                                          
        Eigen::Vector2d scanPt, centeredAgentPt, centeredScanPt, intersection0, intersection1, 
                        intersection0MinusAgent, intersection0MinusScan, 
                        intersection1MinusAgent, intersection1MinusScan, 
                        scanPtMinus;
        float theta = 0.0, range = 0.0, deltaX = 0.0, deltaY = 0.0, 
                dr = 0.0, drSq = 0.0, determinant = 0.0, discriminant = 0.0, dist0 = 0.0, dist1 = 0.0;
        Eigen::Vector4f agentPose;

        std::vector<Eigen::Vector4f> agentPoses = sortAndPrune(propagatedAgents);

        for (int i = 0; i < propagatedScan.ranges.size(); i++) 
        {
            theta = idx2theta(i);
            // cout << "original distance " << modified_laser_scan.ranges.at(i) << endl;
            propagatedScan.ranges.at(i) = std::min(propagatedScan.ranges.at(i), cfg_->scan.range_max - eps);
            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "i: " << i << ", rad: " << rad << ", range: " << propagatedScan.ranges.at(i));
            range = propagatedScan.ranges.at(i);
            
            scanPt << range*cos(theta), range*sin(theta);

            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        scan point (polar)" << i << ": " << theta << ", " << range);

            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        scan point " << i << ": " << scanPt[0] << ", " << scanPt[1]);

            // TODO: sort map here according to distance from robot. Then, can break after first intersection
            for (int j = 0; j < agentPoses.size(); j++) 
            {
                // agent position (in robot frame)
                agentPose = agentPoses.at(j);
                // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "            agent idx " << j << ", pose: " << agentPose[0] << ", " << agentPose[1]);
                
                // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "EGO ROBOT ODOM: " << pt1[0] << ", " << pt1[1]);

                // this algorithm essentially checks if point B intersects with segment AC: 
                //              C (static laser scan point)
                //             /
                //            /
                //           /
                //          B (agent j position)
                //         /
                //        /
                //       /
                //      A (ego robot position)

                // centering robot pose (which in robot frame is 0.0, 0.0) 
                centeredAgentPt << 0.0 - agentPose[0], 0.0 - agentPose[1]; 
                // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "centeredAgentPt: " << centeredAgentPt[0] << ", " << centeredAgentPt[1]);

                // centering static laser scan point
                centeredScanPt << scanPt[0] - agentPose[0], scanPt[1] - agentPose[1]; 
                // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "centered_pt2: " << centered_pt2[0] << ", " << centered_pt2[1]);

                deltaX = centeredScanPt[0] - centeredAgentPt[0];
                deltaY = centeredScanPt[1] - centeredAgentPt[1];

                dr = sqrt(pow(deltaX, 2) + pow(deltaY, 2)); 
                drSq = pow(dr, 2);
                determinant = centeredAgentPt[0]*centeredScanPt[1] - centeredScanPt[0]*centeredAgentPt[1];
                discriminant = pow(cfg_->rbt.r_inscr,2) * pow(dr, 2) - pow(determinant, 2);

                if (discriminant > 0) 
                {
                    intersection0 << epsilonDivide(determinant*deltaY + signum(deltaY) * deltaX * sqrt(discriminant), drSq),
                                     epsilonDivide(-determinant * deltaX + abs(deltaY)*sqrt(discriminant), drSq);
                                        
                    intersection1 << epsilonDivide(determinant*deltaY - signum(deltaY) * deltaX * sqrt(discriminant), drSq),
                                     epsilonDivide(-determinant * deltaX - abs(deltaY)*sqrt(discriminant), drSq);
                    intersection0MinusAgent = intersection0 - centeredAgentPt;
                    intersection1MinusAgent = intersection1 - centeredAgentPt;
                    scanPtMinus = centeredScanPt - centeredAgentPt;

                    dist0 = intersection0MinusAgent.norm();
                    dist1 = intersection1MinusAgent.norm();
                    
                    if (dist0 < dist1) 
                    {
                        intersection0MinusScan = intersection0 - centeredScanPt;

                        if (dist0 < propagatedScan.ranges.at(i) && dist0 < scanPtMinus.norm() && 
                            intersection0MinusScan.norm() < scanPtMinus.norm()) 
                        {
                            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        at i: " << i << ", changed distance from " << 
                            //                                             propagatedScan.ranges.at(i) << " to " << dist0);

                            propagatedScan.ranges.at(i) = dist0;
                            break;
                        }
                    } else 
                    {
                        intersection1MinusScan = intersection1 - centeredScanPt;

                        if (dist1 < propagatedScan.ranges.at(i) && dist1 < scanPtMinus.norm() && 
                            intersection1MinusScan.norm() < scanPtMinus.norm()) 
                        {
                            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        at i: " << i << ", changed distance from " << 
                            //                 propagatedScan.ranges.at(i) << " to " << dist1); 

                            propagatedScan.ranges.at(i) = dist1;
                            break;
                        }
                    }
                }
            }
        }
    }
}