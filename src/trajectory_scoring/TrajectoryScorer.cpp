#include <dynamic_gap/trajectory_scoring/TrajectoryScorer.h>


namespace dynamic_gap 
{
    TrajectoryScorer::TrajectoryScorer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = & cfg;
    }

    void TrajectoryScorer::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> scan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        scan_ = scan;
    }

    /*
    void TrajectoryScorer::updateStaticEgoCircle(const sensor_msgs::LaserScan & staticScan) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        staticScan_ = staticScan;
    }
    */

    void TrajectoryScorer::transformGlobalPathLocalWaypointToRbtFrame(const geometry_msgs::PoseStamped & globalPathLocalWaypointOdomFrame, 
                                                                      const geometry_msgs::TransformStamped & odom2rbt) 
    {
        boost::mutex::scoped_lock lock(globalPlanMutex_);
        tf2::doTransform(globalPathLocalWaypointOdomFrame, globalPathLocalWaypointRobotFrame_, odom2rbt);
    }

    std::vector<float> TrajectoryScorer::scoreTrajectory(const dynamic_gap::Trajectory & traj,
                                                         const std::vector<sensor_msgs::LaserScan> & futureScans) 
    {    
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "         [scoreTrajectory()]");
        // Requires LOCAL FRAME
        // Should be no racing condition

        geometry_msgs::PoseArray path = traj.getPathRbtFrame();
        std::vector<float> pathTiming = traj.getPathTiming();
        
        float totalTrajCost = 0.0;
        std::vector<float> posewiseCosts;

        sensor_msgs::LaserScan dynamicLaserScan = sensor_msgs::LaserScan();

        float t_i = 0.0;
        float t_iplus1 = 0.0;

        /*
        int min_dist_idx, future_scan_idx;
        float theta, range;
        Eigen::Vector2d min_dist_pt(0.0, 0.0);
        if (rawGaps.size() > 0) {
            std::vector<float> dynamic_cost_val(path.poses.size());
            for (int i = 0; i < dynamic_cost_val.size(); i++) {
                // std::cout << "regular range at " << i << ": ";
                t_iplus1 = pathTiming[i];
                future_scan_idx = (int) (t_iplus1 / cfg_->traj.integrate_stept);
                // ROS_INFO_STREAM_NAMED("TrajectoryScorer", "pulling scan for t_iplus1: " << t_iplus1 << " at idx: " << future_scan_idx);
                dynamicLaserScan = futureScans[future_scan_idx];
                // ROS_INFO_STREAM_NAMED("TrajectoryScorer", "pulled scan");
                min_dist_idx = dynamicGetMinDistIndex(path.poses.at(i), dynamicLaserScan, print);
                // ROS_INFO_STREAM_NAMED("TrajectoryScorer", "min_dist_idx: " << min_dist_idx);
                // add point to min_dist_array
                theta = min_dist_idx * dynamicLaserScan.angle_increment + dynamicLaserScan.angle_min;
                range = dynamicLaserScan.ranges.at(min_dist_idx);
                min_dist_pt << range*std::cos(theta), range*std::sin(theta);
 
                
                // if (t_iplus1 == 2.5 && vis) 
                // {
                //     ROS_INFO_STREAM_NAMED("TrajectoryScorer", "visualizing dynamic egocircle from " << t_i << " to " << t_iplus1);
                //     visualizePropagatedEgocircle(dynamicLaserScan); // if I do i ==0, that's just original scan
                // }
                


                // get cost of point
                dynamic_cost_val.at(i) = dynamicScorePose(path.poses.at(i), theta, range);
                // ROS_INFO_STREAM_NAMED("TrajectoryScorer", "dynamic_cost_val: " << dynamic_cost_val.at(i));
                
                if (dynamic_cost_val.at(i) < 0.0 && cfg_->debug.traj_debug_log) {
                    ROS_INFO_STREAM_NAMED("TrajectoryScorer", "at pose: " << i << " of " << dynamic_cost_val.size() << 
                                    "(t: " << t_iplus1 << "), score of: " << 
                                    dynamic_cost_val.at(i) << ", robot pose: " << 
                                    path.poses.at(i).position.x << ", " << path.poses.at(i).position.y << 
                                    ", closest point: " << min_dist_pt[0] << ", " << min_dist_pt[1] << ", distance: " << dist2Pose(theta, range, path.poses.at(i)));
                }
                
                
                t_i = t_iplus1;
            }

            totalTrajCost = std::accumulate(dynamic_cost_val.begin(), dynamic_cost_val.end(), float(0));
            cost_val = dynamic_cost_val;
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "dynamic pose-wise cost: " << totalTrajCost);
        } else {
        */

        std::vector<float> staticPosewiseCosts(path.poses.size());
        for (int i = 0; i < staticPosewiseCosts.size(); i++) 
        {
            // std::cout << "regular range at " << i << ": ";
            staticPosewiseCosts.at(i) = scorePose(path.poses.at(i)); //  / staticPosewiseCosts.size()
        }
        totalTrajCost = std::accumulate(staticPosewiseCosts.begin(), staticPosewiseCosts.end(), float(0));
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "             static pose-wise cost: " << totalTrajCost);
        // }


        posewiseCosts = staticPosewiseCosts;
        if (posewiseCosts.size() > 0) 
        {
            // obtain terminalGoalCost, scale by w1
            float w1 = 1.0; // cfg_->traj.terminal_weight
            float terminalCost = w1 * terminalGoalCost(*std::prev(path.poses.end()));
            // if the ending cost is less than 1 and the total cost is > -10, return trajectory of 100s
            if (terminalCost < 0.25 && totalTrajCost >= 0) 
            {
                // std::cout << "returning really good trajectory" << std::endl;
                return std::vector<float>(path.poses.size(), 100);
            }
            // Should be safe, subtract terminal pose cost from first pose cost
            ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            terminal cost: " << -terminalCost);
            posewiseCosts.at(0) -= terminalCost;
        }
        
        // ROS_INFO_STREAM_NAMED("TrajectoryScorer", "scoreTrajectory time taken:" << ros::WallTime::now().toSec() - start_time);
        return posewiseCosts;
    }

    float TrajectoryScorer::terminalGoalCost(const geometry_msgs::Pose & pose) 
    {
        boost::mutex::scoped_lock planlock(globalPlanMutex_);
        // ROS_INFO_STREAM_NAMED("TrajectoryScorer", pose);
        ROS_INFO_STREAM_NAMED("GapTrajectoryGenerator", "            final pose: (" << pose.position.x << ", " << pose.position.y << "), local goal: (" << globalPathLocalWaypointRobotFrame_.pose.position.x << ", " << globalPathLocalWaypointRobotFrame_.pose.position.y << ")");
        float dx = pose.position.x - globalPathLocalWaypointRobotFrame_.pose.position.x;
        float dy = pose.position.y - globalPathLocalWaypointRobotFrame_.pose.position.y;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    /*
    int TrajectoryScorer::dynamicGetMinDistIndex(const geometry_msgs::Pose & pose, 
                                                 const sensor_msgs::LaserScan & dynamicLaserScan, 
                                                 bool print) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);

        // obtain orientation and idx of pose
        // float poseTheta = std::atan2(pose.position.y + 1e-3, pose.position.x + 1e-3);
        // int center_idx = theta2idx(poseTheta); // (int) std::round((pose_ori + M_PI) / scan_.get()->angle_increment);
        
        // int scan_size = (int) ;
        // dist is size of scan
        std::vector<float> scan2RbtDists(dynamicLaserScan.ranges.size());

        // This size **should** be ensured
        // if (dynamicLaserScan.ranges.size() < 500) {
        //     ROS_FATAL_STREAM("Scan range incorrect scorePose");
        // }

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        float currScan2RbtDist;
        for (int i = 0; i < scan2RbtDists.size(); i++) 
        {
            currScan2RbtDist = dynamicLaserScan.ranges.at(i);
            // currScan2RbtDist = (currScan2RbtDist == 5) ? currScan2RbtDist + cfg_->traj.rmax : currScan2RbtDist; // WHY IS THIS HAPPENING
            scan2RbtDists.at(i) = dist2Pose(idx2theta(i), currScan2RbtDist, pose); // i * dynamicLaserScan.angle_increment - M_PI
        }

        auto iter = std::min_element(scan2RbtDists.begin(), scan2RbtDists.end());
        // int min_dist_index = 
        return std::distance(scan2RbtDists.begin(), iter);
    }
    */
    
    float TrajectoryScorer::dynamicScorePose(const geometry_msgs::Pose & pose, const float & theta, const float & range) 
    {
        float dist = dist2Pose(theta, range, pose);
        float cost = dynamicChapterScore(dist);

        return cost;
    }

    float TrajectoryScorer::scorePose(const geometry_msgs::Pose & pose) 
    {
        boost::mutex::scoped_lock lock(scanMutex_);
        sensor_msgs::LaserScan scan = *scan_.get();

        // obtain orientation and idx of pose

        // dist is size of scan
        std::vector<float> scan2RbtDists(scan.ranges.size());

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        // float currScan2RbtDist = 0.0;
        for (int i = 0; i < scan2RbtDists.size(); i++) 
        {
            scan2RbtDists.at(i) = dist2Pose(idx2theta(i), scan.ranges.at(i), pose);
        }

        auto iter = std::min_element(scan2RbtDists.begin(), scan2RbtDists.end());
        // std::cout << "robot pose: " << pose.position.x << ", " << pose.position.y << ")" << std::endl;
        int minDistIdx = std::distance(scan2RbtDists.begin(), iter);
        float range = scan.ranges.at(minDistIdx);
        float theta = idx2theta(minDistIdx);
        float cost = chapterScore(*iter);
        //std::cout << *iter << ", regular cost: " << cost << std::endl;
        ROS_INFO_STREAM_NAMED("TrajectoryScorer", "            robot pose: " << pose.position.x << ", " << pose.position.y << 
                    ", closest scan point: " << range * std::cos(theta) << ", " << range * std::sin(theta) << ", static cost: " << cost);
        return cost;
    }

    float TrajectoryScorer::chapterScore(const float & rbtToScanDist) 
    {
        // if the distance at the pose is less than the inscribed radius of the robot, return negative infinity
        // std::cout << "in chapterScore with distance: " << d << std::endl;
        float inflRbtRad = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; 

        if (rbtToScanDist < inflRbtRad) 
        {   
            return -std::numeric_limits<float>::infinity();
        }

        // if distance is essentially infinity, return 0
        if (rbtToScanDist > cfg_->traj.max_pose_pen_dist) 
            return 0;

        return cfg_->traj.c_obs * std::exp(-cfg_->traj.pose_exp_weight * (rbtToScanDist - inflRbtRad));
    }

    float TrajectoryScorer::dynamicChapterScore(const float & rbtToScanDist) 
    {
        // if the ditance at the pose is less than the inscribed radius of the robot, return negative infinity
        float inflRbtRad = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; 

        if (rbtToScanDist < inflRbtRad) 
        {
            // std::cout << "distance: " << d << ", r_inscr * inf_ratio: " << r_inscr * cfg_->traj.inf_ratio << std::endl;
            return -std::numeric_limits<float>::infinity();
        }

        // if distance is beyond scan, return 0
        if (rbtToScanDist > cfg_->traj.max_pose_pen_dist) 
            return 0;

        return cfg_->traj.c_obs * std::exp(-cfg_->traj.pose_exp_weight * (rbtToScanDist - inflRbtRad));
    }

}