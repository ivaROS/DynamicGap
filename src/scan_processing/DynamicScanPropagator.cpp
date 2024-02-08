#include <dynamic_gap/scan_processing/DynamicScanPropagator.h>

namespace dynamic_gap
{
    DynamicScanPropagator::DynamicScanPropagator(const DynamicGapConfig& cfg)
    {
        cfg_ = &cfg;        
        
        // TODO: change this to a vector of pointers to laser scans
        sensor_msgs::LaserScan tmpScan = sensor_msgs::LaserScan();
        futureScans_ = std::vector<sensor_msgs::LaserScan>(int(cfg_->traj.integrate_maxt/cfg_->traj.integrate_stept) + 1, tmpScan);
    }

    void DynamicScanPropagator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    std::vector<sensor_msgs::LaserScan> DynamicScanPropagator::propagateCurrentLaserScanCheat(const std::vector<geometry_msgs::Pose> & currentTrueAgentPoses,
                                                                                                const std::vector<geometry_msgs::Vector3Stamped> & currentTrueAgentVels)
    {
        try
        {
            ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "[getFutureScans()]");
            sensor_msgs::LaserScan scan = *scan_.get();
            sensor_msgs::LaserScan dynamicScan = scan; // sensor_msgs::LaserScan();

            // dynamicScan.header = scan.header;
            // dynamicScan.angle_min = scan.angle_min;
            // dynamicScan.angle_max = scan.angle_max;
            // dynamicScan.angle_increment = scan.angle_increment;
            // dynamicScan.time_increment = scan.time_increment;
            // dynamicScan.scan_time = scan.scan_time;
            // dynamicScan.range_min = scan.range_min;
            // dynamicScan.range_max = scan.range_max;
            // dynamicScan.ranges = scan.ranges;

            futureScans_.at(0) = dynamicScan; // at t = 0.0

            std::vector<Eigen::Vector4f> currentAgents;
            
            Eigen::Vector4f ithAgentState;
            for (int i = 0; i < currentTrueAgentPoses.size(); i++) 
            {
                ithAgentState << currentTrueAgentPoses.at(i).position.x, currentTrueAgentPoses.at(i).position.y, 
                                currentTrueAgentVels.at(i).vector.x, currentTrueAgentVels.at(i).vector.y;
                currentAgents.push_back(ithAgentState);
            }

            ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "    detected agents: ");
            for (int i = 0; i < currentAgents.size(); i++)
                ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        agent" << i << " position: " << currentAgents.at(i)[0] << ", " << currentAgents.at(i)[1] << ", velocity: " << currentAgents.at(i)[2] << ", " << currentAgents.at(i)[3]);
            
            float t_i = 0.0, t_iplus1 = 0.0;
            for (int futureScanTimeIdx = 1; futureScanTimeIdx <= int(cfg_->traj.integrate_maxt / cfg_->traj.integrate_stept); futureScanTimeIdx++) 
            {
                t_iplus1 = t_i + futureScanTimeIdx * cfg_->traj.integrate_stept;
                
                dynamicScan.ranges = scan.ranges; // reset scan ranges

                recoverDynamicEgocircle(t_i, t_iplus1, currentAgents, dynamicScan);
                
                futureScanTimeIdx = (int) (t_iplus1 / cfg_->traj.integrate_stept);
                // ROS_INFO_STREAM("adding scan from " << t_i << " to " << t_iplus1 << " at idx: " << futureScanTimeIdx);
                futureScans_.at(futureScanTimeIdx) = dynamicScan;

                t_i = t_iplus1;
            }
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
                                                        sensor_msgs::LaserScan & dynamicLaserScan) 
    {   
        ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "    [recoverDynamicEgocircle()]");
        float interval = t_iplus1 - t_i;
        if (interval <= 0.0)
            return;

        ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        recovering dynamic egocircle for interval: " << t_i << " to " << t_iplus1);

        // propagate poses forward (all odoms and vels are in robot frame)
        for (int i = 0; i < propagatedAgents.size(); i++) 
        {
            ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        agent" << i << " moving from (" << propagatedAgents.at(i)[0] << ", " << propagatedAgents.at(i)[1] << ")");

            propagatedAgents.at(i)[0] += propagatedAgents.at(i)[2]*interval;
            propagatedAgents.at(i)[1] += propagatedAgents.at(i)[3]*interval;
            
            ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "                                  to (" << propagatedAgents.at(i)[0] << ", " << propagatedAgents.at(i)[1] << ")");
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

        for (int i = 0; i < dynamicLaserScan.ranges.size(); i++) 
        {
            theta = theta2idx(i);
            // cout << "i: " << i << " rad: " << rad << endl;
            // cout << "original distance " << modified_laser_scan.ranges.at(i) << endl;
            dynamicLaserScan.ranges.at(i) = std::min(dynamicLaserScan.ranges.at(i), cfg_->scan.range_max - eps);
            // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "i: " << i << ", rad: " << rad << ", range: " << dynamicLaserScan.ranges.at(i));
            range = dynamicLaserScan.ranges.at(i);
            
            scanPt << range*cos(theta), range*sin(theta);

            // TODO: sort map here according to distance from robot. Then, can break after first intersection
            for (int j = 0; j < agentPoses.size(); j++) 
            {
                agentPose = agentPoses.at(j);
                // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "ODOM MAP SECOND: " << agentPose[0] << ", " << agentPose[1]);
                // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "EGO ROBOT ODOM: " << pt1[0] << ", " << pt1[1]);
                // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "ODOM VECT SECOND: " << agentPose[0] << ", " << agentPose[1]);

                // centered ego robot state
                centeredAgentPt << -agentPose[0], -agentPose[1]; 
                // ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "centeredAgentPt: " << centeredAgentPt[0] << ", " << centeredAgentPt[1]);

                // static laser scan point
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

                        if (dist0 < dynamicLaserScan.ranges.at(i) && dist0 < scanPtMinus.norm() && 
                            intersection0MinusScan.norm() < scanPtMinus.norm()) 
                        {
                            ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        at i: " << i << ", changed distance from " << 
                                                                        dynamicLaserScan.ranges.at(i) << " to " << dist0);

                            dynamicLaserScan.ranges.at(i) = dist0;
                            break;
                        }
                    } else 
                    {
                        intersection1MinusScan = intersection1 - centeredScanPt;

                        if (dist1 < dynamicLaserScan.ranges.at(i) && dist1 < scanPtMinus.norm() && 
                            intersection1MinusScan.norm() < scanPtMinus.norm()) 
                        {
                            ROS_INFO_STREAM_NAMED("DynamicScanPropagator", "        at i: " << i << ", changed distance from " << 
                                            dynamicLaserScan.ranges.at(i) << " to " << dist1); 

                            dynamicLaserScan.ranges.at(i) = dist1;
                            break;
                        }
                    }
                }
            }
        }
    }
}