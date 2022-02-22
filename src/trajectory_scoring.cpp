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
    /*
    // Again, in rbt frame
    std::vector<double> TrajectoryArbiter::scoreTrajectories (
        std::vector<geometry_msgs::PoseArray> sample_traj) {
        // This will be in robot frame
        
        return std::vector<double>(sample_traj.size());
    }
    */

    std::vector<double> TrajectoryArbiter::scoreTrajectory(geometry_msgs::PoseArray traj, std::vector<double> time_arr, std::vector<dynamic_gap::Gap>& current_raw_gaps) {
        // Requires LOCAL FRAME
        // Should be no racing condition
        //geometry_msgs::PoseArray traj = std::get<0>(return_tuple);
        //std::vector<double> left_ranges = std::get<1>(return_tuple);
        //std::vector<double> right_ranges = std::get<2>(return_tuple);


        std::vector<dynamic_gap::MP_model *> raw_models;
        for (auto gap : current_raw_gaps) {
            raw_models.push_back(gap.left_model);
            raw_models.push_back(gap.right_model);
        }

        std::vector<double> test_cost_val(traj.poses.size());
        std::vector<double> cost_val(traj.poses.size());

    
        // adjust future_raw_gap model values to simulate robot not moving (vo = 0, ao = 0)
        for (auto & model : raw_models) {
            model->freeze_robot_vel();
        }

        double delta = 0.01;
        double prior_dt = 0.0;
        double dt = 0.0;

        double dist = 1.0;
        double range = 0.0;
        double beta = 0.0;

        double min_dist = 0.0;
        double min_beta = 0.0;
        double min_range = 0.0;
        std::cout << "DYNAMIC SCORING" << std::endl;
        for (int i = 0; i < cost_val.size(); i++) {
            double min_dist = std::numeric_limits<double>::infinity();
            double min_beta = 0.0;
            dt = time_arr[i];
            delta = dt - prior_dt;
            // std::cout << "dt: " << dt << ", prior dt: " << prior_dt << std::endl;
            for (auto & model : raw_models) {
                model->frozen_state_propagate(delta); // OR integrate by 0.01 x times
                Matrix<double, 4, 1> cartesian_model_state = model->get_frozen_cartesian_state();
                // std::cout << "frozen cartesian model. rx: " << cartesian_model_state[0] << ", ry: " << cartesian_model_state[1] << ", vx: " << cartesian_model_state[2] << ", vy: " << cartesian_model_state[3] << std::endl;
                Matrix<double, 5, 1> model_state = model->get_frozen_state();
                range = 1.0 / model_state[0]; // recovering r (wrt robot frame)
                beta = std::atan2(model_state[1], model_state[2]);
                dist = dynamicDist2Pose(traj.poses.at(i), range, beta);
                if (dist < min_dist) {
                    min_dist = dist;
                    min_beta = beta;
                    min_range = range;
                }
            }
            prior_dt = dt;
            cost_val.at(i) = dynamicScorePose(traj.poses.at(i), min_range, min_beta);
            std::cout << "dynamic range at " << i << ": " << min_dist << ", score: " << cost_val.at(i) << std::endl;
            std::cout << "robot pose: " << traj.poses.at(i).position.x << ", " << traj.poses.at(i).position.y << ", closest position: " << min_range * -1 * std::sin(min_beta) << ", " << min_range * std::cos(min_beta) << std::endl;

        }

        std::cout << "------" << std::endl;
        
        /*
        for (int i = 0; i < cost_val.size(); i++) {
            std::cout << "regular range at " << i << ": ";
            cost_val.at(i) = scorePose(traj.poses.at(i));
            
            // do we add here?
            // cost_val.at(i) += scoreGapRanges(left_ranges.at(i), right_ranges.at(i));
        }
        */
        

        // cumulative cost of poses
        // ADDING IN AVERAGE INSTEAD
        auto total_val = std::accumulate(cost_val.begin(), cost_val.end(), double(0)) / cost_val.size();
        // auto dynamic_total_val = std::accumulate(test_cost_val.begin(), test_cost_val.end(), double(0));

        // cumulative cost of ranges of gap

        std::cout << "pose-wise cost: " << total_val << std::endl;
        // std::cout << "dynamic pose-wise cost: " << dynamic_total_val << std::endl;
        // 
        if (cost_val.size() > 0) // && ! cost_val.at(0) == -std::numeric_limits<double>::infinity())
        {
            // obtain terminalGoalCost, scale by w1
            double w1 = 1;
            auto terminal_cost = w1 * terminalGoalCost(*std::prev(traj.poses.end()));
            // if the ending cost is less than 1 and the total cost is > -10, return trajectory of 100s
            if (terminal_cost < 1 && total_val >= 0) {
                std::cout << "returning really good trajectory" << std::endl;
                return std::vector<double>(traj.poses.size(), 100);
            }
            // Should be safe, subtract terminal pose cost from first pose cost
            std::cout << "terminal cost: " << -terminal_cost << std::endl;
            cost_val.at(0) -= terminal_cost;
        }
        
        return cost_val;
    }

    double TrajectoryArbiter::terminalGoalCost(geometry_msgs::Pose pose) {
        boost::mutex::scoped_lock planlock(gplan_mutex);
        // ROS_INFO_STREAM(pose);
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

    double TrajectoryArbiter::scoreGapRanges(double left_range, double right_range) {
        double range = std::min(left_range, right_range);
        if (range < r_inscr * cfg_->traj.inf_ratio) {
            return -std::numeric_limits<double>::infinity();
        }
        // if distance is essentially infinity, return 0
        if (range > rmax) return 0;

        return cobs * std::exp(- w * (range - r_inscr * cfg_->traj.inf_ratio));
    }

    double TrajectoryArbiter::dynamicDist2Pose(geometry_msgs::Pose pose, double range, double beta) {

        double x = range * -1.0 * std::sin(beta);
        double y = range * std::cos(beta);
        double pose_dist = sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2));
        // std::cout << "robot pose: " << pose.position.x << ", " << pose.position.y << ", agent pose: " << x << ", " << y << std::endl;
        // std::cout << "range: " << range << ", beta: " << beta << ", x: " << x << ", y: " << y << std::endl;
        return pose_dist;
    }

    double TrajectoryArbiter::dynamicScorePose(geometry_msgs::Pose pose, double range, double beta) {

        double x = range * -1.0 * std::sin(beta);
        double y = range * std::cos(beta);
        double pose_dist = dynamicDist2Pose(pose, range, beta);
        // std::cout << "range: " << range << ", beta: " << beta << ", x: " << x << ", y: " << y << std::endl;
        return chapterScore(pose_dist);
    }


    double TrajectoryArbiter::scorePose(geometry_msgs::Pose pose) {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        sensor_msgs::LaserScan stored_scan = *msg.get();

        // obtain orientation and idx of pose
        double pose_ori = std::atan2(pose.position.y + 1e-3, pose.position.x + 1e-3);
        int center_idx = (int) std::round((pose_ori + M_PI) / msg.get()->angle_increment);
        
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
            this_dist = this_dist == 3 ? this_dist + cfg_->traj.rmax : this_dist;
            dist.at(i) = dist2Pose(i * stored_scan.angle_increment - M_PI,
                this_dist, pose);
        }

        auto iter = std::min_element(dist.begin(), dist.end());
        double theta = std::distance(dist.begin(), iter) * stored_scan.angle_increment - M_PI;
        double range = stored_scan.ranges.at(std::distance(dist.begin(), iter) );
        double cost = chapterScore(*iter);
        std::cout << *iter << ", regular cost: " << cost << std::endl;
        std::cout << "robot pose: " << pose.position.x << ", " << pose.position.y << ", closest position: " << range * std::cos(theta) << ", " << range * std::sin(theta) << std::endl;
        return cost;
    }

    double TrajectoryArbiter::chapterScore(double d) {
        // if the ditance at the pose is less than the inscribed radius of the robot, return negative infinity
        // std::cout << "in chapterScore with distance: " << d << std::endl;
        if (d < r_inscr * cfg_->traj.inf_ratio) {
            // sum of betadot leftsstd::cout << "pose too close to obstacle, returning -inf" << std::endl;
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