#include <dynamic_gap/goal_selector.h>

namespace dynamic_gap {
    GoalSelector::GoalSelector(ros::NodeHandle& nh,
    const dynamic_gap::DynamicGapConfig& cfg) {
        cfg_ = &cfg;
    }

    bool GoalSelector::setGoal(
        const std::vector<geometry_msgs::PoseStamped> &plan) {
        // Incoming plan is in map frame
        boost::mutex::scoped_lock lock(goal_select_mutex);
        boost::mutex::scoped_lock gplock(gplan_mutex);
        global_plan.clear();
        global_plan = plan;
        // transform plan to robot frame such as base_link
    }

    void GoalSelector::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg) {
        boost::mutex::scoped_lock lock(lscan_mutex);
        sharedPtr_laser = msg;
    }

    void GoalSelector::updateLocalGoal(geometry_msgs::TransformStamped map2rbt) {
        if (global_plan.size() < 2) {
            // No Global Goal
            return;
        }

        if ((*sharedPtr_laser.get()).ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect goalselector");
        }

        boost::mutex::scoped_lock glock(goal_select_mutex);
        boost::mutex::scoped_lock llock(lscan_mutex);
        // getting snippet of global trajectory in robot frame (snippet is whatever part of global trajectory is within laser scan)
        auto local_gplan = getRelevantGlobalPlan(map2rbt);
        if (local_gplan.size() < 1) return;


        // finding first place in global plan where we are visible/obstructed 
        auto result_rev = std::find_if(local_gplan.rbegin(), local_gplan.rend(), 
            std::bind1st(std::mem_fun(&GoalSelector::VisibleOrPossiblyObstructed), this));


        // finding first place where we are not visible?
        auto result_fwd = std::find_if(local_gplan.begin(), local_gplan.end(), 
            std::bind1st(std::mem_fun(&GoalSelector::NoTVisibleOrPossiblyObstructed), this));

        if (cfg_->planning.far_feasible) {
            // if we have gotten all the way to the end of the snippet, set that as the local goal
            // if whole snippet is not visible/ possibly obstructed?
            if (result_rev == local_gplan.rend()) result_rev = std::prev(result_rev); 
            local_goal = *result_rev;
        } else {
            result_fwd = result_fwd == local_gplan.end() ? result_fwd - 1 : result_fwd;
            local_goal = local_gplan.at(result_fwd - local_gplan.begin());
        }
    }

    bool GoalSelector::NoTVisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose) {
        int laserScanIdx = PoseIndexInSensorMsg(pose);
        float epsilon2 = float(cfg_->gap_manip.epsilon2);
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        // this boolean is flipped from VisibleOrPossiblyObstructed. 
        bool check = dist2rbt(pose) > (double (stored_scan_msgs.ranges.at(laserScanIdx)) - cfg_->rbt.r_inscr / 2);
        return check;
    }

    // We are iterating through the global trajectory snippet, checking each pose 
    bool GoalSelector::VisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose) {
        int laserScanIdx = PoseIndexInSensorMsg(pose);
        float epsilon2 = float(cfg_->gap_manip.epsilon2);
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        // first piece of bool: is the distance from pose to rbt less than laserscan range - robot diameter (z-buffer idea)
        // second piece of bool: distance from pose to rbt greater than laserscan range + some epsilon*2 (obstructed?)
        bool check = dist2rbt(pose) < (double (stored_scan_msgs.ranges.at(laserScanIdx)) - cfg_->rbt.r_inscr / 2) || 
            dist2rbt(pose) > (double (stored_scan_msgs.ranges.at(laserScanIdx)) + epsilon2 * 2);
        return check;
    }

    int GoalSelector::PoseIndexInSensorMsg(geometry_msgs::PoseStamped pose) {
        auto orientation = getPoseOrientation(pose);
        auto index = float(orientation + M_PI) / (sharedPtr_laser.get()->angle_increment);
        return int(std::floor(index));
    }

    double GoalSelector::getPoseOrientation(geometry_msgs::PoseStamped pose) {
        return  std::atan2(pose.pose.position.y + 1e-3, pose.pose.position.x + 1e-3);
    }

    std::vector<geometry_msgs::PoseStamped> GoalSelector::getRelevantGlobalPlan(geometry_msgs::TransformStamped map2rbt) {
        // Global Plan is now in robot frame
        // Do magic with egocircle
        boost::mutex::scoped_lock gplock(gplan_mutex);
        mod_plan.clear();
        // where is global_plan coming from?
        mod_plan = global_plan;


        if (mod_plan.size() == 0) {
            ROS_FATAL_STREAM("Global Plan Length = 0");
        }

        // transforming plan into robot frame
        for (int i = 0; i < mod_plan.size(); i++) {
            tf2::doTransform(mod_plan.at(i), mod_plan.at(i), map2rbt);
        }

        // calculating distance to robot at each step of plan
        std::vector<double> distance(mod_plan.size());
        for (int i = 0; i < distance.size(); i++) {
            distance.at(i) = dist2rbt(mod_plan.at(i));
        }

        // Finding the largest distance in the laser scan
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        threshold = (double) *std::max_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.end());

        // Find closest pose to robot to start the global plan snippet
        auto start_pose = std::min_element(distance.begin(), distance.end());

        // find_if returns iterator for which predicate is true within range (start_pose, distance.end()). You would
        // imagine that as distance goes on, the distances get greater.
        // bind1st: binds "this" object to the isNotWithin function?

        // end_pose is the first point within distance that lies beyond the robot scan.
        auto end_pose = std::find_if(start_pose, distance.end(),
            std::bind1st(std::mem_fun(&GoalSelector::isNotWithin), this));

        if (start_pose == distance.end()) {
            ROS_FATAL_STREAM("No Global Plan pose within Robot scan");
            return std::vector<geometry_msgs::PoseStamped>(0);
        } else if (mod_plan.size() == 0) {
            ROS_WARN_STREAM("No Global Plan Received or Size 0");
            return std::vector<geometry_msgs::PoseStamped>(0);
        }

        int start_idx = std::distance(distance.begin(), start_pose);
        int end_idx = std::distance(distance.begin(), end_pose);

        auto start_gplan = mod_plan.begin() + start_idx;
        auto end_gplan = mod_plan.begin() + end_idx;

        std::vector<geometry_msgs::PoseStamped> local_gplan(start_gplan, end_gplan);
        return local_gplan;
    }

    double GoalSelector::dist2rbt(geometry_msgs::PoseStamped pose) {
        return sqrt(pow(pose.pose.position.x, 2) + pow(pose.pose.position.y, 2));
    }

    bool GoalSelector::isNotWithin(const double dist) {
        return dist > threshold;
    }

    geometry_msgs::PoseStamped GoalSelector::getCurrentLocalGoal(geometry_msgs::TransformStamped rbt2odom) {
        geometry_msgs::PoseStamped result;
        tf2::doTransform(local_goal, result, rbt2odom);
        // This should return something in odom frame
        return result;
    }

    std::vector<geometry_msgs::PoseStamped> GoalSelector::getOdomGlobalPlan() {
        return global_plan;
    }


}