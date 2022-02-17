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

    std::vector<double> TrajectoryArbiter::scoreTrajectory(geometry_msgs::PoseArray traj) {
        // Requires LOCAL FRAME
        // Should be no racing condition
        //geometry_msgs::PoseArray traj = std::get<0>(return_tuple);
        //std::vector<double> left_ranges = std::get<1>(return_tuple);
        //std::vector<double> right_ranges = std::get<2>(return_tuple);


        std::vector<double> cost_val(traj.poses.size());

        for (int i = 0; i < cost_val.size(); i++) {
            cost_val.at(i) = scorePose(traj.poses.at(i));
            // do we add here?
            // cost_val.at(i) += scoreGapRanges(left_ranges.at(i), right_ranges.at(i));
        }

        // cumulative cost of poses
        auto total_val = std::accumulate(cost_val.begin(), cost_val.end(), double(0));

        // cumulative cost of ranges of gap

        std::cout << "pose-wise cost: " << total_val << std::endl;

        // 
        if (cost_val.size() > 0) // && ! cost_val.at(0) == -std::numeric_limits<double>::infinity())
        {
            // obtain terminalGoalCost, scale by w1
            double w1 = 10;
            auto terminal_cost = w1 * terminalGoalCost(*std::prev(traj.poses.end()));
            // if the ending cost is less than 1 and the total cost is > -10, return trajectory of 100s
            if (terminal_cost < 1 && total_val > -10) {
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
        return chapterScore(*iter);
    }

    double TrajectoryArbiter::chapterScore(double d) {
        // if the ditance at the pose is less than the inscribed radius of the robot, return negative infinity
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