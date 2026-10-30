#include <dynamic_gap/Planner.h>
#include <dynamic_gap/trajectory_generation/GoalApproach.h>
#include <std_msgs/String.h>
#include <memory>

namespace dynamic_gap
{
bool Planner::tryGoalApproach(Trajectory& trajectory, int& flag)
{
    if (!cfg_.mppi.enabled || !cfg_.goal_approach.enabled || cfg_.planning.holonomic ||
        cfg_.ctrl.man_ctrl || !cfg_.ctrl.feedback_ctrl || !haveTFs_)
        return false;
    geometry_msgs::PoseStamped goal_robot;
    tf2::doTransform(globalGoalOdomFrame_, goal_robot, odom2rbt_);
    const double distance = std::hypot(goal_robot.pose.position.x, goal_robot.pose.position.y);
    if (!std::isfinite(distance) || distance > cfg_.goal_approach.activation_distance)
        return false;

    // A nearby goal around a wall still needs the global route. Compare the
    // remaining full path, rather than visibility (a pedestrian may hide the goal).
    const auto plan = globalPlanManager_->getGlobalPathRobotFrame(map2rbt_);
    if (plan.empty()) return false;
    std::size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < plan.size(); ++i)
    {
        const double d = std::hypot(plan[i].pose.position.x, plan[i].pose.position.y);
        if (d < nearest_distance) { nearest_distance = d; nearest = i; }
    }
    double route_length = nearest_distance;
    for (std::size_t i = nearest + 1; i < plan.size(); ++i)
        route_length += std::hypot(plan[i].pose.position.x - plan[i - 1].pose.position.x,
                                   plan[i].pose.position.y - plan[i - 1].pose.position.y);
    route_length += std::hypot(plan.back().pose.position.x - goal_robot.pose.position.x,
                               plan.back().pose.position.y - goal_robot.pose.position.y);
    if (route_length > 1.5 * distance + 0.25) return false;

    // Dynamic occlusion may require yielding, but static walls require routing.
    // The map is read independently of the future laser occupancy used below.
    {
        boost::mutex::scoped_lock lock(mapMutex_);
        if (!hasStaticMap_) return false;
        try
        {
            geometry_msgs::PoseStamped origin, robot_map, goal_map;
            origin.header.frame_id = cfg_.robot_frame_id;
            origin.pose.orientation.w = 1;
            const auto robot_to_map = tfBuffer_.lookupTransform(staticMap_.header.frame_id, cfg_.robot_frame_id, ros::Time(0));
            const auto odom_to_map = tfBuffer_.lookupTransform(staticMap_.header.frame_id, cfg_.odom_frame_id, ros::Time(0));
            tf2::doTransform(origin, robot_map, robot_to_map);
            tf2::doTransform(globalGoalOdomFrame_, goal_map, odom_to_map);
            // The exact goal can be too close to a wall for the extra clearance,
            // while a valid stopping point inside its tolerance remains reachable.
            // Match the servo's stopping radius; every actual rollout is still
            // collision checked, including its first command-hold segment.
            auto corridor_goal = goal_map.pose;
            const double dx = corridor_goal.position.x - robot_map.pose.position.x;
            const double dy = corridor_goal.position.y - robot_map.pose.position.y;
            const double length = std::hypot(dx, dy);
            const double allowance = std::min(length, 0.7 * double(cfg_.goal.xy_global_goal_tolerance));
            if (length > 1e-6)
            {
                corridor_goal.position.x -= allowance * dx / length;
                corridor_goal.position.y -= allowance * dy / length;
            }
            if (!goalApproachStaticCorridorClear(staticMap_, robot_map.pose, corridor_goal,
                    cfg_.rbt.r_inscr * cfg_.traj.inf_ratio + cfg_.goal_approach.clearance))
                return false;
        }
        catch (const tf2::TransformException&) { return false; }
    }

    flag = GOAL_APPROACH;
    geometry_msgs::PoseArray stopped;
    stopped.header.frame_id = cfg_.robot_frame_id;
    stopped.header.stamp = ros::Time::now();
    stopped.poses.push_back(rbtPoseInRbtFrame_.pose);
    trajectory = Trajectory(stopped, {0.f});
    trajectory.setPathOdomFrame(gapTrajGenerator_->transformPath(stopped, rbt2odom_));
    trajectory.v_cmd = 0;
    trajectory.w_cmd = 0;
    std::string status = "yield";
    try
    {
        auto raw = deepCopyCurrentRawGaps();
        std::vector<std::unique_ptr<Gap>> owned;
        for (auto* gap : raw) owned.emplace_back(gap);
        boost::shared_ptr<const sensor_msgs::LaserScan> scan;
        {
            boost::mutex::scoped_lock lock(gapMutex_);
            scan = scan_;
        }
        const double age = scan ? (ros::Time::now() - scan->header.stamp).toSec() : 1e9;
        if (!scan || age < -0.01 || age > cfg_.goal_approach.max_scan_age)
            status = "stale_scan";
        else
        {
            auto future = cfg_.planning.future_scan_propagation ?
                dynamicScanPropagator_->propagateCurrentLaserScan(raw) :
                std::vector<sensor_msgs::LaserScan>(11, *scan);
            publishFutureScans(future);
            const auto robot_to_sensor = tfBuffer_.lookupTransform(cfg_.sensor_frame_id, cfg_.robot_frame_id, ros::Time(0));
            geometry_msgs::PoseStamped origin, initial_sensor, goal_sensor;
            origin.header.frame_id = cfg_.robot_frame_id;
            origin.pose.orientation.w = 1;
            tf2::doTransform(origin, initial_sensor, robot_to_sensor);
            tf2::doTransform(goal_robot, goal_sensor, robot_to_sensor);
            const auto result = planGoalApproach(initial_sensor.pose, goal_sensor.pose, future, cfg_);
            if (result.safe)
            {
                trajectory = Trajectory(result.path, result.times);
                const auto sensor_to_odom = tfBuffer_.lookupTransform(cfg_.odom_frame_id, cfg_.sensor_frame_id, ros::Time(0));
                trajectory.setPathOdomFrame(gapTrajGenerator_->transformPath(result.path, sensor_to_odom));
                trajectory.v_cmd = result.command.linear.x;
                trajectory.w_cmd = result.command.angular.z;
                status = trajectory.v_cmd > 1e-4 ? "approach" :
                    (std::abs(trajectory.w_cmd) > 1e-4 ? "align" : "yield");
            }
        }
    }
    catch (const std::exception& error)
    {
        // Keep the zero command; do not fall back to an unchecked goal segment.
        ROS_WARN_STREAM_THROTTLE_NAMED(1., "GoalApproach", "Final approach stopped: " << error.what());
        status = "unavailable_prediction";
    }
    setCurrentTraj(Trajectory());
    setCurrentLeftGapPtModelID(nullptr);
    setCurrentRightGapPtModelID(nullptr);
    trajVisualizer_->drawMppiTrajectories({});
    trajVisualizer_->drawCurrentTrajectory(trajectory);
    std_msgs::String message;
    message.data = status;
    goalApproachStatusPublisher_.publish(message);
    ROS_INFO_STREAM_THROTTLE_NAMED(1., "GoalApproach", status << " distance=" << distance
        << " v=" << trajectory.v_cmd << " w=" << trajectory.w_cmd);
    return true;
}
}
