#include <dynamic_gap/trajectory_scoring/TrajectoryScorer.h>


namespace dynamic_gap {
    TrajectoryScorer::TrajectoryScorer(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = & cfg;
        propagatedEgocirclePublisher = nh.advertise<sensor_msgs::LaserScan>("propagated_egocircle", 1);
    }

    void TrajectoryScorer::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) 
    {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        msg = msg_;
    }

    void TrajectoryScorer::updateStaticEgoCircle(const sensor_msgs::LaserScan & static_scan_) 
    {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        static_scan = static_scan_;
    }

    void TrajectoryScorer::updateLocalGoal(geometry_msgs::PoseStamped lg, geometry_msgs::TransformStamped odom2rbt) 
    {
        boost::mutex::scoped_lock lock(gplan_mutex);
        tf2::doTransform(lg, local_goal, odom2rbt);
    }

    bool compareModelBearings(dynamic_gap::RotatingFrameCartesianKalmanFilter* model_one, 
                              dynamic_gap::RotatingFrameCartesianKalmanFilter* model_two) 
    {
        Eigen::Matrix<float, 4, 1> state_one = model_one->getGapState();
        Eigen::Matrix<float, 4, 1> state_two = model_two->getGapState();
        
        return atan2(state_one[1], state_one[0]) < atan2(state_two[1], state_two[0]);
    }

    int TrajectoryScorer::sgn_star(float dy) {
        if (dy < 0) {
            return -1;
        } else {
            return 1;
        }
    }

    struct Comparator {
        bool operator() (std::vector<float> & a, std::vector<float> & b) {
            float a_dist = pow(a[0], 2) + pow(a[1], 2);
            float b_dist = pow(b[0], 2) + pow(b[1], 2);
            return a_dist < b_dist;
        }
    };

    std::vector< std::vector<float> > TrajectoryScorer::sort_and_prune(const std::vector<Eigen::Matrix<float, 4, 1> > & _odom_vects)
    {
    
        // Declare vector of pairs
        std::vector< std::vector<float> > A;
        
        // Copy key-value pair from Map
        // to vector of pairs
        float dist;
        for (int i = 0; i < _odom_vects.size(); i++) {
            // ROS_INFO_STREAM(it.first);
            //ROS_INFO_STREAM("pose: " << std::to_string(it.second[0]) << ", " << std::to_string(it.second[1]));
            //ROS_INFO_STREAM("ego pose: " << pt1[0] << ", " << pt1[1]);

            std::vector<float> odom_vect{_odom_vects[i][0], _odom_vects[i][1]};
            
            dist = sqrt(pow(odom_vect[0], 2) + pow(odom_vect[1], 2));
            //ROS_INFO_STREAM("dist: " << dist);
            if (dist < cfg_->rbt.max_range) {
                A.push_back(odom_vect);
            }
        }
        
        // Sort using comparator function
        sort(A.begin(), A.end(), Comparator());
        
        // ROS_INFO_STREAM("printing pruned vect");
        // Print the sorted value
        /*
        for (auto& it : A) {
    
            ROS_INFO_STREAM(it.first << ' ' << it.second[0] << ", " << it.second[1]);
        }
        */
        

        return A;
    }

    void TrajectoryScorer::recoverDynamicEgoCircle(float t_i, float t_iplus1, 
                                                    std::vector<Eigen::Matrix<float, 4, 1> > & curr_agents_lc,
                                                    sensor_msgs::LaserScan & dynamic_laser_scan,
                                                    bool print) 
    {   
        if (print) ROS_INFO_STREAM("    [recoverDynamicEgoCircle()]");
        float interval = t_iplus1 - t_i;
        if (interval <= 0.0)
            return;

        if (print) ROS_INFO_STREAM("        recovering dynamic egocircle for interval: " << t_i << " to " << t_iplus1);
        // for EVERY interval, start with static scan
        dynamic_laser_scan.ranges = static_scan.ranges;

        float max_range = cfg_->rbt.max_range;
        // propagate poses forward (all odoms and vels are in robot frame)
        for (int i = 0; i < curr_agents_lc.size(); i++) 
        {
            if (print) {
                ROS_INFO_STREAM("        agent" << i << " moving from (" << curr_agents_lc[i][0] << ", " << curr_agents_lc[i][1] << ")");
            }

            curr_agents_lc[i][0] += curr_agents_lc[i][2]*interval;
            curr_agents_lc[i][1] += curr_agents_lc[i][3]*interval;
            
            if (print) {
                ROS_INFO_STREAM("                                  to (" << curr_agents_lc[i][0] << ", " << curr_agents_lc[i][1] << ")");
            }
        }

        // basically run modify_scan                                                          
        Eigen::Vector2d pt2, centered_pt1, centered_pt2, dx_dy, intersection0, intersection1, 
                        int0_min_cent_pt1, int0_min_cent_pt2, int1_min_cent_pt1, int1_min_cent_pt2, 
                        cent_pt2_min_cent_pt1;
        float rad, dist, dx, dy, dr, D, discriminant, dist0, dist1;
        std::vector<float> other_state;

        std::vector<std::vector<float>> odom_vect = sort_and_prune(curr_agents_lc);

        for (int i = 0; i < dynamic_laser_scan.ranges.size(); i++) 
        {
            rad = dynamic_laser_scan.angle_min + i*dynamic_laser_scan.angle_increment;
            // cout << "i: " << i << " rad: " << rad << endl;
            // cout << "original distance " << modified_laser_scan.ranges[i] << endl;
            dynamic_laser_scan.ranges[i] = std::min(dynamic_laser_scan.ranges[i], max_range);
            // ROS_INFO_STREAM("i: " << i << ", rad: " << rad << ", range: " << dynamic_laser_scan.ranges[i]);
            dist = dynamic_laser_scan.ranges[i];
            
            pt2 << dist*cos(rad), dist*sin(rad);

            // map<string, vector<float>>::iterator it;
            //vector<pair<string, vector<float> >> odom_vect = sort_and_prune(odom_map);
            // TODO: sort map here according to distance from robot. Then, can break after first intersection
            for (int j = 0; j < odom_vect.size(); j++) 
            {
                other_state = odom_vect[j];
                // ROS_INFO_STREAM("ODOM MAP SECOND: " << other_state[0] << ", " << other_state[1]);
                // int idx_dist = std::distance(odom_map.begin(), it);
                // ROS_INFO_STREAM("EGO ROBOT ODOM: " << pt1[0] << ", " << pt1[1]);
                // ROS_INFO_STREAM("ODOM VECT SECOND: " << other_state[0] << ", " << other_state[1]);

                // centered ego robot state
                centered_pt1 << -other_state[0], -other_state[1]; 
                // ROS_INFO_STREAM("centered_pt1: " << centered_pt1[0] << ", " << centered_pt1[1]);

                // static laser scan point
                centered_pt2 << pt2[0] - other_state[0], pt2[1] - other_state[1]; 
                // ROS_INFO_STREAM("centered_pt2: " << centered_pt2[0] << ", " << centered_pt2[1]);

                dx = centered_pt2[0] - centered_pt1[0];
                dy = centered_pt2[1] - centered_pt1[1];
                dx_dy << dx, dy;
                dr = dx_dy.norm();

                D = centered_pt1[0]*centered_pt2[1] - centered_pt2[0]*centered_pt1[1];
                discriminant = pow(cfg_->rbt.r_inscr,2) * pow(dr, 2) - pow(D, 2);

                if (discriminant > 0) {
                    intersection0 << (D*dy + sgn_star(dy) * dx * sqrt(discriminant)) / pow(dr, 2),
                                     (-D * dx + abs(dy)*sqrt(discriminant)) / pow(dr, 2);
                                        
                    intersection1 << (D*dy - sgn_star(dy) * dx * sqrt(discriminant)) / pow(dr, 2),
                                     (-D * dx - abs(dy)*sqrt(discriminant)) / pow(dr, 2);
                    int0_min_cent_pt1 = intersection0 - centered_pt1;
                    int1_min_cent_pt1 = intersection1 - centered_pt1;
                    cent_pt2_min_cent_pt1 = centered_pt2 - centered_pt1;

                    dist0 = int0_min_cent_pt1.norm();
                    dist1 = int1_min_cent_pt1.norm();
                    
                    if (dist0 < dist1) {
                        int0_min_cent_pt2 = intersection0 - centered_pt2;

                        if (dist0 < dynamic_laser_scan.ranges[i] && dist0 < cent_pt2_min_cent_pt1.norm() && int0_min_cent_pt2.norm() < cent_pt2_min_cent_pt1.norm() ) {
                            if (print) {
                                ROS_INFO_STREAM("        at i: " << i << ", changed distance from " << 
                                                dynamic_laser_scan.ranges[i] << " to " << dist0);
                            }

                            dynamic_laser_scan.ranges[i] = dist0;
                            break;
                        }
                    } else {
                        int1_min_cent_pt2 = intersection1 - centered_pt2;

                        if (dist1 < dynamic_laser_scan.ranges[i] && dist1 < cent_pt2_min_cent_pt1.norm() && int1_min_cent_pt2.norm() < cent_pt2_min_cent_pt1.norm() ) {
                            if (print) {
                                ROS_INFO_STREAM("        at i: " << i << ", changed distance from " << 
                                                dynamic_laser_scan.ranges[i] << " to " << dist1); 
                            }          

                            dynamic_laser_scan.ranges[i] = dist1;
                            break;
                        }
                    }
                }
            }
        }
    }

    // Does things in rbt frame
    std::vector<float> TrajectoryScorer::scoreGaps()
    {
        boost::mutex::scoped_lock planlock(gplan_mutex);
        boost::mutex::scoped_lock egolock(egocircle_mutex);
        if (gaps.size() < 1) {
            //ROS_WARN_STREAM("Observed num of gap: 0");
            return std::vector<float>(0);
        }

        // How fix this
        int num_of_scan = msg.get()->ranges.size();
        float goal_orientation = std::atan2(local_goal.pose.position.y, local_goal.pose.position.x);
        int idx = goal_orientation / (M_PI / (num_of_scan / 2)) + (num_of_scan / 2);
        ROS_DEBUG_STREAM("Goal Orientation: " << goal_orientation << ", idx: " << idx);
        ROS_DEBUG_STREAM(local_goal.pose.position);
        auto costFn = [](dynamic_gap::Gap g, int goal_idx) -> float
        {
            int leftdist = std::abs(g.RIdx() - goal_idx);
            int rightdist = std::abs(g.LIdx() - goal_idx);
            return std::min(leftdist, rightdist);
        };

        std::vector<float> cost(gaps.size());
        for (int i = 0; i < cost.size(); i++) 
        {
            cost.at(i) = costFn(gaps.at(i), idx);
        }

        return cost;
    }

    void TrajectoryScorer::visualizePropagatedEgocircle(const sensor_msgs::LaserScan & dynamic_laser_scan) 
    {
        propagatedEgocirclePublisher.publish(dynamic_laser_scan);
    }

    std::vector<float> TrajectoryScorer::scoreTrajectory(const geometry_msgs::PoseArray & traj, 
                                                          const std::vector<float> & time_arr, 
                                                          const std::vector<dynamic_gap::Gap> & current_raw_gaps,              
                                                          const std::vector<geometry_msgs::Pose> & agentPoses,
                                                          const std::vector<geometry_msgs::Vector3Stamped> & agentVels,
                                                          const std::vector<sensor_msgs::LaserScan> & future_scans,
                                                          bool print,
                                                          bool vis) 
    {    
        if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("            [scoreTrajectory()]");
        // Requires LOCAL FRAME
        // Should be no racing condition
        // float start_time = ros::WallTime::now().toSec();

        /*
        std::vector<dynamic_gap::Estimator *> raw_models;
        for (auto gap : current_raw_gaps) {
            raw_models.push_back(gap.right_model);
            raw_models.push_back(gap.left_model);
        }
        
        
        // std::cout << "starting setting sides and freezing velocities" << std::endl;
        for (auto & model : raw_models) {
            model->isolateGapDynamics();
        }
        */
        
        float total_val = 0.0;
        std::vector<float> cost_val;

        
        sensor_msgs::LaserScan dynamic_laser_scan = sensor_msgs::LaserScan();

        float t_i = 0.0;
        float t_iplus1 = 0.0;
        // int counts = std::min(cfg_->planning.num_feasi_check, int(traj.poses.size()));        

        /*
        int min_dist_idx, future_scan_idx;
        float theta, range;
        Eigen::Vector2d min_dist_pt(0.0, 0.0);
        if (current_raw_gaps.size() > 0) {
            std::vector<float> dynamic_cost_val(traj.poses.size());
            for (int i = 0; i < dynamic_cost_val.size(); i++) {
                // std::cout << "regular range at " << i << ": ";
                t_iplus1 = time_arr[i];
                future_scan_idx = (int) (t_iplus1 / cfg_->traj.integrate_stept);
                // ROS_INFO_STREAM("pulling scan for t_iplus1: " << t_iplus1 << " at idx: " << future_scan_idx);
                dynamic_laser_scan = future_scans[future_scan_idx];
                // ROS_INFO_STREAM("pulled scan");
                min_dist_idx = dynamicGetMinDistIndex(traj.poses.at(i), dynamic_laser_scan, print);
                // ROS_INFO_STREAM("min_dist_idx: " << min_dist_idx);
                // add point to min_dist_array
                theta = min_dist_idx * dynamic_laser_scan.angle_increment + dynamic_laser_scan.angle_min;
                range = dynamic_laser_scan.ranges.at(min_dist_idx);
                min_dist_pt << range*std::cos(theta), range*std::sin(theta);
 
                
                // if (t_iplus1 == 2.5 && vis) {
                //     ROS_INFO_STREAM("visualizing dynamic egocircle from " << t_i << " to " << t_iplus1);
                //     visualizePropagatedEgocircle(dynamic_laser_scan); // if I do i ==0, that's just original scan
                // }
                


                // get cost of point
                dynamic_cost_val.at(i) = dynamicScorePose(traj.poses.at(i), theta, range);
                // ROS_INFO_STREAM("dynamic_cost_val: " << dynamic_cost_val.at(i));
                
                if (dynamic_cost_val.at(i) < 0.0 && cfg_->debug.traj_debug_log) {
                    ROS_INFO_STREAM("at pose: " << i << " of " << dynamic_cost_val.size() << 
                                    "(t: " << t_iplus1 << "), score of: " << 
                                    dynamic_cost_val.at(i) << ", robot pose: " << 
                                    traj.poses.at(i).position.x << ", " << traj.poses.at(i).position.y << 
                                    ", closest point: " << min_dist_pt[0] << ", " << min_dist_pt[1] << ", distance: " << dist2Pose(theta, range, traj.poses.at(i)));
                }
                
                
                t_i = t_iplus1;
            }

            total_val = std::accumulate(dynamic_cost_val.begin(), dynamic_cost_val.end(), float(0));
            cost_val = dynamic_cost_val;
            if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("dynamic pose-wise cost: " << total_val);
        } else {
        */

        std::vector<float> static_cost_val(traj.poses.size());
        for (int i = 0; i < static_cost_val.size(); i++) 
        {
            // std::cout << "regular range at " << i << ": ";
            static_cost_val.at(i) = scorePose(traj.poses.at(i)); //  / static_cost_val.size()
        }
        total_val = std::accumulate(static_cost_val.begin(), static_cost_val.end(), float(0));
        cost_val = static_cost_val;
        if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("                static pose-wise cost: " << total_val);
        // }

        if (cost_val.size() > 0) 
        {
            // obtain terminalGoalCost, scale by w1
            float w1 = 1.0;
            auto terminal_cost = w1 * terminalGoalCost(*std::prev(traj.poses.end()));
            // if the ending cost is less than 1 and the total cost is > -10, return trajectory of 100s
            if (terminal_cost < 0.25 && total_val >= 0) {
                // std::cout << "returning really good trajectory" << std::endl;
                return std::vector<float>(traj.poses.size(), 100);
            }
            // Should be safe, subtract terminal pose cost from first pose cost
            if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("                terminal cost: " << -terminal_cost);
            cost_val.at(0) -= terminal_cost;
        }
        
        // ROS_INFO_STREAM("scoreTrajectory time taken:" << ros::WallTime::now().toSec() - start_time);
        return cost_val;
    }

    float TrajectoryScorer::terminalGoalCost(geometry_msgs::Pose pose) 
    {
        boost::mutex::scoped_lock planlock(gplan_mutex);
        // ROS_INFO_STREAM(pose);
        if (cfg_->debug.traj_debug_log) ROS_INFO_STREAM("                    final pose: (" << pose.position.x << ", " << pose.position.y << "), local goal: (" << local_goal.pose.position.x << ", " << local_goal.pose.position.y << ")");
        float dx = pose.position.x - local_goal.pose.position.x;
        float dy = pose.position.y - local_goal.pose.position.y;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    // if we wanted to incorporate how egocircle can change, 
    float TrajectoryScorer::dist2Pose(float theta, float range, geometry_msgs::Pose pose) 
    {
        // ego circle point in local frame, pose in local frame
        // ROS_INFO_STREAM("   theta: " << theta << ", range: " << range);
        // ROS_INFO_STREAM("   rbt_x: " << pose.position.x << ", rbt_y: " << pose.position.y);
        float x = range * std::cos(theta);
        float y = range * std::sin(theta);
        float dist = sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2)); 
        // ROS_INFO_STREAM("   dist: " << dist);
        return dist;
    }

    int TrajectoryScorer::dynamicGetMinDistIndex(geometry_msgs::Pose pose, 
                                                 const sensor_msgs::LaserScan & dynamic_laser_scan, 
                                                 bool print) 
    {
        boost::mutex::scoped_lock lock(egocircle_mutex);

        // obtain orientation and idx of pose
        float pose_ori = std::atan2(pose.position.y + 1e-3, pose.position.x + 1e-3);
        int center_idx = (int) std::round((pose_ori + M_PI) / msg.get()->angle_increment);
        
        int scan_size = (int) dynamic_laser_scan.ranges.size();
        // dist is size of scan
        std::vector<float> dist(scan_size);

        // This size **should** be ensured
        if (dynamic_laser_scan.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect scorePose");
        }

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        float curr_dist;
        for (int i = 0; i < dist.size(); i++) {
            curr_dist = dynamic_laser_scan.ranges.at(i);
            // curr_dist = (curr_dist == 5) ? curr_dist + cfg_->traj.rmax : curr_dist; // WHY IS THIS HAPPENING
            dist.at(i) = dist2Pose(i * dynamic_laser_scan.angle_increment - M_PI, curr_dist, pose);
        }

        auto iter = std::min_element(dist.begin(), dist.end());
        int min_dist_index = std::distance(dist.begin(), iter);
        return min_dist_index;
    }

    float TrajectoryScorer::dynamicScorePose(geometry_msgs::Pose pose, float theta, float range) 
    {
        
        float dist = dist2Pose(theta, range, pose);
        float cost = dynamicChapterScore(dist);

        return cost;
    }

    float TrajectoryScorer::scorePose(geometry_msgs::Pose pose) 
    {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        sensor_msgs::LaserScan stored_scan = *msg.get();

        // obtain orientation and idx of pose
        //float pose_ori = std::atan2(pose.position.y + 1e-3, pose.position.x + 1e-3);
        //int center_idx = (int) std::round((pose_ori + M_PI) / msg.get()->angle_increment);
        
        int scan_size = (int) stored_scan.ranges.size();
        // dist is size of scan
        std::vector<float> dist(scan_size);

        // This size **should** be ensured
        if (stored_scan.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect scorePose");
        }

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        float curr_dist;
        for (int i = 0; i < dist.size(); i++) {
            curr_dist = stored_scan.ranges.at(i);
            // curr_dist = curr_dist == 5 ? curr_dist + cfg_->traj.rmax : curr_dist; // WHY IS THIS HAPPENING
            dist.at(i) = dist2Pose(i * stored_scan.angle_increment - M_PI, curr_dist, pose);
        }

        auto iter = std::min_element(dist.begin(), dist.end());
        // std::cout << "robot pose: " << pose.position.x << ", " << pose.position.y << ")" << std::endl;
        float range = stored_scan.ranges.at(std::distance(dist.begin(), iter));
        float theta = std::distance(dist.begin(), iter) * stored_scan.angle_increment - M_PI;
        float cost = chapterScore(*iter);
        //std::cout << *iter << ", regular cost: " << cost << std::endl;
        ROS_INFO_STREAM("                robot pose: " << pose.position.x << ", " << pose.position.y << ", closest scan point: " << range * std::cos(theta) << ", " << range * std::sin(theta) << ", static cost: " << cost);
        return cost;
    }

    float TrajectoryScorer::dynamicChapterScore(float d) 
    {
        // if the ditance at the pose is less than the inscribed radius of the robot, return negative infinity
        // std::cout << "in chapterScore with distance: " << d << std::endl;
        float r_infl = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; 
        if (d < r_infl) { //  
            // std::cout << "distance: " << d << ", r_inscr * inf_ratio: " << r_inscr * cfg_->traj.inf_ratio << std::endl;
            return -std::numeric_limits<float>::infinity();
        }
        // if distance is beyond scan, return 0
        if (d > cfg_->traj.max_pose_pen_dist) return 0;

        return cfg_->traj.cobs * std::exp(-cfg_->traj.pose_exp_weight * (d - r_infl));
    }

    float TrajectoryScorer::chapterScore(float d) {
        // if the ditance at the pose is less than the inscribed radius of the robot, return negative infinity
        // std::cout << "in chapterScore with distance: " << d << std::endl;
        float r_infl = cfg_->rbt.r_inscr * cfg_->traj.inf_ratio; 
        if (d < r_infl) { //   
            // std::cout << "distance: " << d << ", r_inscr * inf_ratio: " << r_inscr * cfg_->traj.inf_ratio << std::endl;
            return -std::numeric_limits<float>::infinity();
        }
        // if distance is essentially infinity, return 0
        if (d > cfg_->traj.max_pose_pen_dist) return 0;

        return cfg_->traj.cobs * std::exp(-cfg_->traj.pose_exp_weight * (d - r_infl));
    }
}