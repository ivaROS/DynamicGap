#include <dynamic_gap/trajectory_generation/GoalApproach.h>
#include <dynamic_gap/trajectory_generation/MppiControl.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace dynamic_gap
{
namespace
{
struct Point { double x, y; };
using Cloud = std::vector<Point>;

bool validPose(const geometry_msgs::Pose& pose)
{
    const auto& q = pose.orientation;
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
        std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
        q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w > 1e-12;
}

double distance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
{
    return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
}

geometry_msgs::Twist servo(const geometry_msgs::Pose& pose, const geometry_msgs::Pose& goal,
                          const DynamicGapConfig& cfg, double scale, double bias)
{
    geometry_msgs::Twist command;
    const double d = distance(pose, goal);
    const double yaw = tf::getYaw(pose.orientation);
    if (d < 0.7 * cfg.goal.xy_global_goal_tolerance)
    {
        const double error = std::remainder(tf::getYaw(goal.orientation) - yaw, 2 * M_PI);
        if (std::abs(error) > cfg.goal.yaw_global_goal_tolerance)
            command.angular.z = std::clamp(2 * error, -double(cfg.rbt.vang_absmax), double(cfg.rbt.vang_absmax));
        return command;
    }
    const double error = std::remainder(std::atan2(goal.position.y - pose.position.y,
                                                 goal.position.x - pose.position.x) - yaw, 2 * M_PI);
    command.angular.z = std::clamp(2 * error + bias, -double(cfg.rbt.vang_absmax), double(cfg.rbt.vang_absmax));
    // Align in place instead of making a sweeping arc when the goal is behind us.
    if (std::abs(error) < 0.65)
        command.linear.x = scale * std::min({cfg.goal_approach.max_speed,
            double(cfg.rbt.vx_absmax), 0.9 * d}) * std::pow(std::cos(error), 2);
    return command;
}
}

bool goalApproachStaticCorridorClear(const nav_msgs::OccupancyGrid& map,
                                    const geometry_msgs::Pose& start,
                                    const geometry_msgs::Pose& goal, double radius)
{
    const double resolution = map.info.resolution;
    if (!std::isfinite(resolution) || resolution <= 0 || map.info.width == 0 || map.info.height == 0 ||
        map.data.size() != std::size_t(map.info.width) * map.info.height ||
        !std::isfinite(radius) || radius <= 0 || !validPose(start) || !validPose(goal) || !validPose(map.info.origin))
        return false;
    const double yaw = tf::getYaw(map.info.origin.orientation);
    const double c = std::cos(yaw), s = std::sin(yaw);
    const auto local = [&](const geometry_msgs::Pose& p) {
        const double x = p.position.x - map.info.origin.position.x;
        const double y = p.position.y - map.info.origin.position.y;
        return Point{c*x + s*y, -s*x + c*y};
    };
    const auto a = local(start), b = local(goal);
    const int steps = std::max(1, int(std::ceil(distance(start, goal) / (0.5 * resolution))));
    // Including half a cell diagonal also catches thin occupied boundary cells.
    const double inflated = radius + resolution * std::sqrt(0.5);
    for (int j = 0; j <= steps; ++j)
    {
        const double f = double(j) / steps;
        const double x = a.x + f*(b.x-a.x), y = a.y + f*(b.y-a.y);
        for (int ix = int(std::floor((x-inflated)/resolution)); ix <= int(std::ceil((x+inflated)/resolution)); ++ix)
            for (int iy = int(std::floor((y-inflated)/resolution)); iy <= int(std::ceil((y+inflated)/resolution)); ++iy)
            {
                if (std::hypot((ix+.5)*resolution-x, (iy+.5)*resolution-y) > inflated) continue;
                if (ix < 0 || iy < 0 || ix >= int(map.info.width) || iy >= int(map.info.height)) return false;
                const auto cell = map.data[std::size_t(iy)*map.info.width + ix];
                if (cell < 0 || cell >= 50) return false;
            }
    }
    return true;
}

GoalApproachResult planGoalApproach(const geometry_msgs::Pose& initial,
                                   const geometry_msgs::Pose& goal,
                                   const std::vector<sensor_msgs::LaserScan>& scans,
                                   const DynamicGapConfig& cfg)
{
    GoalApproachResult best;
    const double scan_dt = cfg.traj.integrate_stept;
    if (scans.size() < 2 || !std::isfinite(scan_dt) || scan_dt <= 0 ||
        !validPose(initial) || !validPose(goal) ||
        !std::isfinite(cfg.rbt.vx_absmax) || cfg.rbt.vx_absmax <= 0 ||
        !std::isfinite(cfg.rbt.vang_absmax) || cfg.rbt.vang_absmax <= 0 ||
        !std::isfinite(cfg.rbt.r_inscr) || cfg.rbt.r_inscr <= 0 ||
        !std::isfinite(cfg.traj.inf_ratio) || cfg.traj.inf_ratio <= 0 ||
        !std::isfinite(cfg.goal.xy_global_goal_tolerance) || cfg.goal.xy_global_goal_tolerance <= 0 ||
        !std::isfinite(cfg.goal.yaw_global_goal_tolerance) || cfg.goal.yaw_global_goal_tolerance < 0 ||
        !std::isfinite(cfg.goal_approach.clearance) || cfg.goal_approach.clearance < 0 ||
        !std::isfinite(cfg.goal_approach.max_speed) || cfg.goal_approach.max_speed <= 0 ||
        !std::isfinite(cfg.goal_approach.prediction_horizon) || cfg.goal_approach.prediction_horizon <= 0)
        return best;
    const double horizon = std::min(cfg.goal_approach.prediction_horizon, scan_dt * (scans.size() - 1));
    if (horizon < 0.5)
        return best;
    const int count = std::max(1, int(std::ceil(horizon / 0.05)));
    const double dt = horizon / count;
    const double radius = cfg.rbt.r_inscr * cfg.traj.inf_ratio + cfg.goal_approach.clearance;
    if (!std::isfinite(radius) || radius <= 0)
        return best;
    std::vector<Cloud> clouds(scans.size());
    for (std::size_t k = 0; k < scans.size(); ++k)
    {
        const auto& scan = scans[k];
        if (scan.ranges.empty() || scan.header.frame_id.empty() || !std::isfinite(scan.angle_min) ||
            !std::isfinite(scan.angle_increment) || scan.angle_increment <= 0 ||
            scan.header.frame_id != scans.front().header.frame_id)
            return best;
        std::size_t valid = 0;
        for (std::size_t i = 0; i < scan.ranges.size(); ++i)
        {
            const double range = scan.ranges[i];
            if (std::isinf(range) && range > 0) { ++valid; continue; }
            if (!std::isfinite(range) || range < scan.range_min) continue;
            ++valid;
            const double angle = scan.angle_min + i * scan.angle_increment;
            clouds[k].push_back({range * std::cos(angle), range * std::sin(angle)});
        }
        if (valid < scan.ranges.size() * 0.9) return best;
    }
    const auto clearance = [&](const geometry_msgs::Pose& pose, double t) {
        const auto lo = std::min(std::size_t(t / scan_dt), clouds.size() - 1);
        const auto hi = std::min(lo + 1, clouds.size() - 1);
        double nearest = std::numeric_limits<double>::infinity();
        // Check both bracketing predictions at 50 ms motion steps. The first
        // command also has to clear the current measured scan, not just a model.
        for (std::size_t k : {lo, hi, t <= 0.25 ? std::size_t(0) : lo})
            for (const auto& point : clouds[k])
                nearest = std::min(nearest, std::hypot(point.x - pose.position.x, point.y - pose.position.y));
        return nearest - radius;
    };
    double best_score = std::numeric_limits<double>::infinity();
    for (double scale : {1., 0.65, 0.35, 0.})
    {
        for (double bias : {0., 0.25, -0.25})
        {
            GoalApproachResult candidate;
            candidate.path.header = scans.front().header;
            auto pose = initial;
            candidate.path.poses.push_back(pose);
            candidate.times.push_back(0);
            double accumulated_distance = 0, obstacle_cost = 0;
            bool safe = clearance(pose, 0) >= 0;
            geometry_msgs::Twist command;
            const int first_hold = std::max(1, int(std::ceil(0.25 / dt)));
            const int control_hold = std::max(1, int(std::ceil(0.1 / dt)));
            for (int j = 0; j < count && safe; ++j)
            {
                // Collision substeps are finer than the actual 10 Hz controller.
                // Hold the first command for 250 ms to cover scheduling delay;
                // do not assume the robot executes a new servo command at 20 Hz.
                if (j == 0 || (j >= first_hold && (j-first_hold) % control_hold == 0))
                    command = servo(pose, goal, cfg, scale, bias);
                if (j == 0) candidate.command = command;
                auto next = integrateMppiUnicycle(pose, command, dt);
                const double clear = clearance(next, (j + 1) * dt);
                // No rollout is allowed to steer away and loop back to the goal.
                safe = clear >= 0 && distance(next, goal) <= distance(pose, goal) + 1e-6;
                if (!safe) break;
                pose = next;
                accumulated_distance += distance(pose, goal);
                obstacle_cost += 0.002 / std::max(0.02, clear);
                candidate.path.poses.push_back(pose);
                candidate.times.push_back((j + 1) * dt);
            }
            if (!safe) continue;
            const double score = 5 * distance(pose, goal) + accumulated_distance / count +
                obstacle_cost / count + 0.02 * std::pow(candidate.command.angular.z, 2);
            if (score < best_score)
            {
                best_score = score;
                candidate.safe = true;
                best = std::move(candidate);
            }
        }
    }
    // No checked candidate means a stop, never a fallback blind goal command.
    return best;
}
}
