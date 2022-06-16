#include <dynamic_gap/trajectory_scoring.h>
#include <numeric>


namespace dynamic_gap {
    TrajectoryArbiter::TrajectoryArbiter(ros::NodeHandle& nh, const dynamic_gap::DynamicGapConfig& cfg)
    {
        cfg_ = & cfg;
        r_inscr = cfg_->rbt.r_inscr;
        rmax = cfg_->traj.rmax;
        cobs = cfg_->traj.cobs;
        w = cfg_->traj.w;
        propagatedEgocirclePublisher = nh.advertise<sensor_msgs::LaserScan>("propagated_egocircle", 500);
    }

    void TrajectoryArbiter::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        msg = msg_;
    }
    void TrajectoryArbiter::updateGapContainer(const std::vector<dynamic_gap::Gap> observed_gaps) {
        boost::mutex::scoped_lock lock(gap_mutex);
        gaps.clear();
        gaps = observed_gaps;
    }

    void TrajectoryArbiter::updateLocalGoal(geometry_msgs::PoseStamped lg, geometry_msgs::TransformStamped odom2rbt) {
        boost::mutex::scoped_lock lock(gplan_mutex);
        tf2::doTransform(lg, local_goal, odom2rbt);
    }

    bool compareModelBearings(dynamic_gap::cart_model* model_one, dynamic_gap::cart_model* model_two) {
        Matrix<double, 4, 1> state_one = model_one->get_cartesian_state();
        Matrix<double, 4, 1> state_two = model_two->get_cartesian_state();
        
        return state_one[1] < state_two[1];
        // return atan2(state_one[1], state_one[2]) < atan2(state_two[1], state_two[2]);
    }

    void TrajectoryArbiter::recoverDynamicEgoCircle(double t_i, double t_iplus1, std::vector<dynamic_gap::cart_model *> raw_models, sensor_msgs::LaserScan& dynamic_laser_scan) {
        // freeze models
        // std::cout << "num gaps: " << current_raw_gaps.size() << std::endl;

        // std::cout << "finished setting sides and freezing velocities" << std::endl;

        //sensor_msgs::LaserScan dynamic_laser_scan = sensor_msgs::LaserScan();
        //dynamic_laser_scan.set_ranges_size(2*gap.half_scan);


        // iterate
        // std::cout << "propagating models" << std::endl;
        double interval = t_iplus1 - t_i;
        if (interval <= 0.0) {
            return;
        }
        //std::cout << "time: " << t_iplus1 << std::endl;
        for (double i = 0.0; i < interval; i += cfg_->traj.integrate_stept) {
            for (auto & model : raw_models) {
                model->frozen_state_propagate(cfg_->traj.integrate_stept);
            }
        }
        // std::cout << "sorting models" << std::endl;
        sort(raw_models.begin(), raw_models.end(), compareModelBearings);

        bool searching_for_left = true;
        bool start = true;
        dynamic_gap::cart_model * curr_left = NULL;
        dynamic_gap::cart_model * curr_right = NULL;
        dynamic_gap::cart_model * first_left = NULL;
        dynamic_gap::cart_model * first_right = NULL;
        dynamic_gap::cart_model * last_model = raw_models[raw_models.size() - 1];
        dynamic_gap::cart_model * model = NULL;
        Matrix<double, 4, 1> left_state, model_state;
        double curr_beta;
        int curr_idx;

        // std::cout << "iterating through models" << std::endl;
        for (int j = 0; j < raw_models.size(); j++) {
            // std::cout << "model " << j << std::endl;
            model = raw_models[j];
            model_state = model->get_frozen_modified_polar_state();
            curr_beta = model_state[1]; // atan2(model_state[1], model_state[2]);
            curr_idx = (int) ((curr_beta + M_PI) / msg.get()->angle_increment);
            //std::cout << "candidate model with idx: " << curr_idx << std::endl;

            if (model->get_side() == "left") {

                if (start) {
                    first_left = model;
                    start = false;
                } 
                
                if (searching_for_left) {
                    searching_for_left = false;
                    curr_left = model;
                    //std::cout << "setting current left index to " << curr_idx << std::endl;
                } else {
                    left_state = curr_left->get_frozen_modified_polar_state();
                    if (model_state[0] > left_state[0]) {
                        //std::cout << "swapping current left to" << curr_idx << std::endl;
                        curr_left = model;
                    }
                }

                if (curr_right != nullptr && curr_left != nullptr) {
                    //std::cout << "obstacle space between right: " << right_idx << " and left: " << left_idx << ", right beta: " << right_beta << ", left beta: " << left_beta << std::endl;
                    populateDynamicLaserScan(curr_left, curr_right, dynamic_laser_scan, 0);
                }
            } else if (model->get_side() == "right") {

                //std::cout << "setting current right to " << curr_idx << std::endl;
                curr_right = model;
                if (first_right == nullptr) {
                    first_right = model;
                }
                if (curr_right != nullptr && curr_left != nullptr) {
                    //std::cout << "free space between left: " << left_idx << " and right: " << right_idx <<  ", left beta: " << left_beta << ", right beta: " << right_beta << std::endl;
                    populateDynamicLaserScan(curr_left, curr_right, dynamic_laser_scan, 1);
                    searching_for_left = true;
                }
            }
        }

        if (last_model->get_side() == "left") {
            // wrapping around last free space
            //std::cout << "wrapped free space between right: " << right_idx << " and left: " << left_idx << ", right beta: " << right_beta << ", left beta: " << left_beta << std::endl;
            populateDynamicLaserScan(last_model, first_right, dynamic_laser_scan, 1);
        } else {
            // wrapping around last obstacle space
            //std::cout << "wrapped obstacle space between right: " << right_idx << " and left: " << left_idx << ", right beta: " << right_beta << ", left beta: " << left_beta << std::endl;
            populateDynamicLaserScan(first_left, last_model, dynamic_laser_scan, 0);
        }
        
    }

    void TrajectoryArbiter::populateDynamicLaserScan(dynamic_gap::cart_model * left_model, dynamic_gap::cart_model * right_model, sensor_msgs::LaserScan & dynamic_laser_scan, bool free) {
        // std::cout << "populating part of laser scan" << std::endl;
        Matrix<double, 4, 1> left_state = left_model->get_frozen_modified_polar_state();
        Matrix<double, 4, 1> right_state = right_model->get_frozen_modified_polar_state();
        //std::cout << "left state: " << left_state[0] << ", "  << left_state[1] << ", "  << left_state[2] << ", "  << left_state[3] << ", "  << left_state[4] << std::endl;
        //std::cout << "right state: " << right_state[0] << ", "  << right_state[1] << ", "  << right_state[2] << ", "  << right_state[3] << ", "  << right_state[4] << std::endl;         
        double left_beta = left_state[1]; // atan2(left_state[1], left_state[2]);
        double right_beta = right_state[1]; // atan2(right_state[1], right_state[2]);
        int left_idx = (int) ((left_beta + M_PI) / msg.get()->angle_increment);
        int right_idx = (int) ((right_beta + M_PI) / msg.get()->angle_increment);
        double left_range = 1 / left_state[0];
        double right_range = 1  / right_state[0];

        int start_idx, end_idx;
        double start_range, end_range;
        if (free) {
            start_idx = left_idx;
            end_idx = right_idx;
            start_range = left_range;
            end_range = right_range;
        } else {
            start_idx = right_idx;
            end_idx = left_idx;
            start_range = right_range;
            end_range = left_range;
        }

        int idx_span, entry_idx;
        double new_range;
        if (start_idx <= end_idx) {
            idx_span = end_idx - start_idx;
        } else {
            idx_span = (512 - start_idx) + end_idx;
        }
        for (int idx = 0; idx < idx_span; idx++) {
            new_range = setDynamicLaserScanRange(idx, idx_span, start_idx, end_idx, start_range, end_range, free);
            entry_idx = (start_idx + idx) % 512;
            // std::cout << "updating range at " << entry_idx << " to " << new_range << std::endl;
            dynamic_laser_scan.ranges[entry_idx] = new_range;
        }
        // std::cout << "done populating" << std::endl;
    }

    double TrajectoryArbiter::setDynamicLaserScanRange(double idx, double idx_span, double start_idx, double end_idx, double start_range, double end_range, bool free) {
        if (free) {
            return 5;
        } else {
            if (start_idx != end_idx) {
                return start_range + (end_range - start_range) * (idx / idx_span);
            } else {
                return std::min(start_range, end_range);
            }
        }
    }

    // Does things in rbt frame
    std::vector<double> TrajectoryArbiter::scoreGaps()
    {
        boost::mutex::scoped_lock planlock(gplan_mutex);
        boost::mutex::scoped_lock egolock(egocircle_mutex);
        if (gaps.size() < 1) {
            //ROS_WARN_STREAM("Observed num of gap: 0");
            return std::vector<double>(0);
        }

        // How fix this
        int num_of_scan = msg.get()->ranges.size();
        double goal_orientation = std::atan2(local_goal.pose.position.y, local_goal.pose.position.x);
        int idx = goal_orientation / (M_PI / (num_of_scan / 2)) + (num_of_scan / 2);
        ROS_DEBUG_STREAM("Goal Orientation: " << goal_orientation << ", idx: " << idx);
        ROS_DEBUG_STREAM(local_goal.pose.position);
        auto costFn = [](dynamic_gap::Gap g, int goal_idx) -> double
        {
            int leftdist = std::abs(g._left_idx - goal_idx);
            int rightdist = std::abs(g._right_idx - goal_idx);
            return std::min(leftdist, rightdist);
        };

        std::vector<double> cost(gaps.size());
        for (int i = 0; i < cost.size(); i++) 
        {
            cost.at(i) = costFn(gaps.at(i), idx);
        }

        return cost;
    }

    void TrajectoryArbiter::visualizePropagatedEgocircle(sensor_msgs::LaserScan dynamic_laser_scan) {
        propagatedEgocirclePublisher.publish(dynamic_laser_scan);
    }


    std::vector<double> TrajectoryArbiter::scoreTrajectory(geometry_msgs::PoseArray traj, std::vector<double> time_arr, std::vector<dynamic_gap::Gap>& current_raw_gaps) {
        // Requires LOCAL FRAME
        // Should be no racing condition

                 
        std::vector<dynamic_gap::cart_model *> raw_models;
        for (auto gap : current_raw_gaps) {
            raw_models.push_back(gap.left_model);
            raw_models.push_back(gap.right_model);
        }
        
        
        // std::cout << "starting setting sides and freezing velocities" << std::endl;
        int count = 0;
        for (auto & model : raw_models) {
            if (count % 2 == 0) {
                model->set_side("left");
            } else {
                model->set_side("right");
            }
            count++;
            model->freeze_robot_vel();
        }
        
        
        // std::cout << "num models: " << raw_models.size() << std::endl;
        std::vector<double> dynamic_cost_val(traj.poses.size());
        std::vector<double> static_cost_val(traj.poses.size());
        double total_val = 0.0;
        std::vector<double> cost_val;

        sensor_msgs::LaserScan stored_scan = *msg.get();
        sensor_msgs::LaserScan dynamic_laser_scan = sensor_msgs::LaserScan();
        dynamic_laser_scan.header = stored_scan.header;
        dynamic_laser_scan.angle_min = stored_scan.angle_min;
        dynamic_laser_scan.angle_max = stored_scan.angle_max;
        dynamic_laser_scan.angle_increment = stored_scan.angle_increment;
        dynamic_laser_scan.time_increment = stored_scan.time_increment;
        dynamic_laser_scan.scan_time = stored_scan.scan_time;
        dynamic_laser_scan.range_min = stored_scan.range_min;
        dynamic_laser_scan.range_max = stored_scan.range_max;
        dynamic_laser_scan.ranges = stored_scan.ranges;
        std::vector<float> scan_intensities(stored_scan.ranges.size(), 0.5);
        dynamic_laser_scan.intensities = scan_intensities;

        double t_i = 0.0;
        double t_iplus1 = 0.0;
        if (current_raw_gaps.size() > 0) {
            for (int i = 0; i < dynamic_cost_val.size(); i++) {
                // std::cout << "regular range at " << i << ": ";
                t_iplus1 = time_arr[i];
                recoverDynamicEgoCircle(t_i, t_iplus1, raw_models, dynamic_laser_scan);
                /*
                if (i == 1) {
                    visualizePropagatedEgocircle(dynamic_laser_scan);
                }
                */
                dynamic_cost_val.at(i) = dynamicScorePose(traj.poses.at(i), dynamic_laser_scan) / dynamic_cost_val.size();
                t_i = t_iplus1;
            }
            auto dynamic_total_val = std::accumulate(dynamic_cost_val.begin(), dynamic_cost_val.end(), double(0));
            total_val = dynamic_total_val;
            cost_val = dynamic_cost_val;
            ROS_INFO_STREAM("dynamic pose-wise cost: " << dynamic_total_val);
        } else {
            for (int i = 0; i < static_cost_val.size(); i++) {
                // std::cout << "regular range at " << i << ": ";
                static_cost_val.at(i) = scorePose(traj.poses.at(i)) / static_cost_val.size();
            }
            auto static_total_val = std::accumulate(static_cost_val.begin(), static_cost_val.end(), double(0));
            total_val = static_total_val;
            cost_val = static_cost_val;
            ROS_INFO_STREAM("static pose-wise cost: " << static_total_val);
        }

        int counts = std::min(cfg_->planning.num_feasi_check, int(traj.poses.size()));        
        //std::cout << "r_inscr: " << r_inscr << ", inf_ratio: " << cfg_->traj.inf_ratio << std::endl;
        for (int i = 0; i < counts; i++) {
            // std::cout << "regular range at " << i << ": ";
            if (cost_val.at(i) == -std::numeric_limits<double>::infinity()) {
                ROS_INFO_STREAM("-inf at pose " << i << " of " << cost_val.size() << " with distance of: " << getClosestDist(traj.poses.at(i)));
            }
        }


        if (cost_val.size() > 0) 
        {
            // obtain terminalGoalCost, scale by w1
            double w1 = 1;
            auto terminal_cost = w1 * terminalGoalCost(*std::prev(traj.poses.end()));
            // if the ending cost is less than 1 and the total cost is > -10, return trajectory of 100s
            if (terminal_cost < 1 && total_val >= -10) {
                // std::cout << "returning really good trajectory" << std::endl;
                return std::vector<double>(traj.poses.size(), 100);
            }
            // Should be safe, subtract terminal pose cost from first pose cost
            ROS_INFO_STREAM("terminal cost: " << -terminal_cost);
            cost_val.at(0) -= terminal_cost;
        }
        
        return cost_val;
    }

    double TrajectoryArbiter::terminalGoalCost(geometry_msgs::Pose pose) {
        boost::mutex::scoped_lock planlock(gplan_mutex);
        // ROS_INFO_STREAM(pose);
        ROS_INFO_STREAM("final pose: (" << pose.position.x << ", " << pose.position.y << "), local goal: (" << local_goal.pose.position.x << ", " << local_goal.pose.position.y << ")");
        double dx = pose.position.x - local_goal.pose.position.x;
        double dy = pose.position.y - local_goal.pose.position.y;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    // if we wanted to incorporate how egocircle can change, 
    double TrajectoryArbiter::dist2Pose(float theta, float dist, geometry_msgs::Pose pose) {
        // ego circle point in local frame, pose in local frame
        float x = dist * std::cos(theta);
        float y = dist * std::sin(theta);
        return sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2));
    }

    double TrajectoryArbiter::dynamicScorePose(geometry_msgs::Pose pose, sensor_msgs::LaserScan dynamic_laser_scan) {
        boost::mutex::scoped_lock lock(egocircle_mutex);

        // obtain orientation and idx of pose
        double pose_ori = std::atan2(pose.position.y + 1e-3, pose.position.x + 1e-3);
        int center_idx = (int) std::round((pose_ori + M_PI) / msg.get()->angle_increment);
        
        int scan_size = (int) dynamic_laser_scan.ranges.size();
        // dist is size of scan
        std::vector<double> dist(scan_size);

        // This size **should** be ensured
        if (dynamic_laser_scan.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect scorePose");
        }

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        for (int i = 0; i < dist.size(); i++) {
            float this_dist = dynamic_laser_scan.ranges.at(i);
            this_dist = this_dist == 5 ? this_dist + cfg_->traj.rmax : this_dist;
            dist.at(i) = dist2Pose(i * dynamic_laser_scan.angle_increment - M_PI, this_dist, pose);
        }

        auto iter = std::min_element(dist.begin(), dist.end());
        double theta = std::distance(dist.begin(), iter) * dynamic_laser_scan.angle_increment - M_PI;
        double range = dynamic_laser_scan.ranges.at(std::distance(dist.begin(), iter) );
        //std::cout << "closest point: (" << x << ", " << y << "), robot pose: " << pose.position.x << ", " << pose.position.y << ")" << std::endl;
        double cost = chapterScore(*iter);
        //std::cout << *iter << ", regular cost: " << cost << std::endl;
        // std::cout << "dynamic cost: " << cost << ", robot pose: " << pose.position.x << ", " << pose.position.y << ", closest position: " << range * std::cos(theta) << ", " << range * std::sin(theta) << std::endl;
        return cost;
    }

    double TrajectoryArbiter::getClosestDist(geometry_msgs::Pose pose) {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        sensor_msgs::LaserScan stored_scan = *msg.get();

        int scan_size = (int) stored_scan.ranges.size();
        // dist is size of scan
        std::vector<double> dist(scan_size);

        // This size **should** be ensured
        if (stored_scan.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect scorePose");
        }

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        for (int i = 0; i < dist.size(); i++) {
            float this_dist = stored_scan.ranges.at(i);
            this_dist = this_dist == 5 ? this_dist + cfg_->traj.rmax : this_dist;
            dist.at(i) = dist2Pose(i * stored_scan.angle_increment - M_PI,
                this_dist, pose);
        }

        auto iter = std::min_element(dist.begin(), dist.end());

        return *iter;
    }

    double TrajectoryArbiter::scorePose(geometry_msgs::Pose pose) {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        sensor_msgs::LaserScan stored_scan = *msg.get();

        // obtain orientation and idx of pose
        //double pose_ori = std::atan2(pose.position.y + 1e-3, pose.position.x + 1e-3);
        //int center_idx = (int) std::round((pose_ori + M_PI) / msg.get()->angle_increment);
        
        int scan_size = (int) stored_scan.ranges.size();
        // dist is size of scan
        std::vector<double> dist(scan_size);

        // This size **should** be ensured
        if (stored_scan.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect scorePose");
        }

        // iterate through ranges and obtain the distance from the egocircle point and the pose
        // Meant to find where is really small
        for (int i = 0; i < dist.size(); i++) {
            float this_dist = stored_scan.ranges.at(i);
            this_dist = this_dist == 5 ? this_dist + cfg_->traj.rmax : this_dist;
            dist.at(i) = dist2Pose(i * stored_scan.angle_increment - M_PI,
                this_dist, pose);
        }

        auto iter = std::min_element(dist.begin(), dist.end());
        //std::cout << "closest point: (" << x << ", " << y << "), robot pose: " << pose.position.x << ", " << pose.position.y << ")" << std::endl;
        double cost = chapterScore(*iter);
        //std::cout << *iter << ", regular cost: " << cost << std::endl;
        // std::cout << "static cost: " << cost << ", robot pose: " << pose.position.x << ", " << pose.position.y << ", closest position: " << range * std::cos(theta) << ", " << range * std::sin(theta) << std::endl;
        return cost;
    }

    double TrajectoryArbiter::chapterScore(double d) {
        // if the ditance at the pose is less than the inscribed radius of the robot, return negative infinity
        // std::cout << "in chapterScore with distance: " << d << std::endl;
        if (d < r_inscr * cfg_->traj.inf_ratio) { //   
            // std::cout << "distance: " << d << ", r_inscr * inf_ratio: " << r_inscr * cfg_->traj.inf_ratio << std::endl;
            return -std::numeric_limits<double>::infinity();
        }
        // if distance is essentially infinity, return 0
        if (d > rmax) return 0;

        return cobs * std::exp(- w * (d - r_inscr * cfg_->traj.inf_ratio));
    }

    int TrajectoryArbiter::searchIdx(geometry_msgs::Pose pose) {
        if (!msg) return 1;
        double r = sqrt(pow(pose.position.x, 2) + pow(pose.position.y, 2));
        double eval = double(cfg_->rbt.r_inscr) / r;
        if (eval > 1) return 1;
        float theta = float(std::acos( eval ));
        int searchIdx = (int) std::ceil(theta / msg.get()->angle_increment);
        return searchIdx;
    }

    dynamic_gap::Gap TrajectoryArbiter::returnAndScoreGaps() {
        boost::mutex::scoped_lock gaplock(gap_mutex);
        std::vector<double> cost = scoreGaps();
        auto decision_iter = std::min_element(cost.begin(), cost.end());
        int gap_idx = std::distance(cost.begin(), decision_iter);
        // ROS_INFO_STREAM("Selected Gap Index " << gap_idx);
        auto selected_gap = gaps.at(gap_idx);
        return selected_gap;
    }
    

}