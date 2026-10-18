#include <dynamic_gap/trajectory_generation/MppiOptimizer.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dynamic_gap
{
namespace
{
constexpr float infinity = std::numeric_limits<float>::infinity();

geometry_msgs::Twist boundedControl(geometry_msgs::Twist control, const DynamicGapConfig& cfg)
{
    control.linear.x = std::clamp(control.linear.x, -double(cfg.rbt.vx_absmax), double(cfg.rbt.vx_absmax));
    control.angular.z = std::clamp(control.angular.z, -double(cfg.rbt.vang_absmax), double(cfg.rbt.vang_absmax));
    return control;
}
}

MppiOptimizer::MppiOptimizer(const DynamicGapConfig& cfg, std::uint32_t seed)
    : cfg_(cfg), random_(seed)
{
    const auto& p = cfg.mppi;
    if (p.horizon < 1 || p.total_samples < 1 || p.iterations < 1 ||
        !std::isfinite(p.temperature) || p.temperature <= 0 ||
        !std::isfinite(p.sigma_v) || p.sigma_v < 0 ||
        !std::isfinite(p.sigma_w) || p.sigma_w < 0 ||
        !std::isfinite(p.update_fraction) || p.update_fraction <= 0 || p.update_fraction > 1 ||
        !std::isfinite(cfg.traj.integrate_stept) || cfg.traj.integrate_stept <= 0 ||
        !std::isfinite(cfg.rbt.vx_absmax) || cfg.rbt.vx_absmax <= 0 ||
        !std::isfinite(cfg.rbt.vang_absmax) || cfg.rbt.vang_absmax <= 0)
        throw std::invalid_argument("Invalid MPPI parameters or robot velocity limits");
}

MppiResult MppiOptimizer::optimize(const geometry_msgs::Pose& initial,
                                  const MppiControls& nominal,
                                  std::size_t sample_count,
                                  const std::string& frame,
                                  const Score& score,
                                  std::optional<double> fixed_forward_speed)
{
    MppiResult result;
    if (nominal.empty() || sample_count == 0 || !score)
        return result;
    if (fixed_forward_speed && (!std::isfinite(*fixed_forward_speed) ||
        *fixed_forward_speed < 0 || *fixed_forward_speed > cfg_.rbt.vx_absmax))
        return result;
    MppiControls controls = nominal;
    for (auto& u : controls)
    {
        if (!std::isfinite(u.linear.x) || !std::isfinite(u.angular.z))
            return result;
        u = boundedControl(u, cfg_);
        if (fixed_forward_speed)
            u.linear.x = *fixed_forward_speed;
    }
    auto evaluate = [&](const MppiControls& sequence) {
        auto trajectory = rolloutMppiControls(initial, sequence, cfg_.traj.integrate_stept, frame);
        trajectory.totalTrajCost = trajectory.positions.empty() ? infinity : score(trajectory);
        if (!std::isfinite(trajectory.totalTrajCost))
            trajectory.totalTrajCost = infinity;
        return trajectory;
    };
    auto keepBest = [&](const dwa_Trajectory& trajectory, const MppiControls& sequence) {
        if (std::isfinite(trajectory.totalTrajCost) &&
            (!result.valid || trajectory.totalTrajCost < result.trajectory.totalTrajCost))
        {
            result.valid = true;
            result.trajectory = trajectory;
            result.controls = sequence;
        }
    };
    keepBest(evaluate(controls), controls);
    std::normal_distribution<double> gaussian(0.0, 1.0);
    result.samples.reserve(sample_count * cfg_.mppi.iterations);
    for (int iteration = 0; iteration < cfg_.mppi.iterations; ++iteration)
    {
        std::vector<MppiControls> proposals(sample_count, controls);
        std::vector<float> costs(sample_count, infinity);
        float min_cost = infinity;
        for (std::size_t k = 0; k < sample_count; ++k)
        {
            for (auto& u : proposals[k])
            {
                if (!fixed_forward_speed)
                    u.linear.x += cfg_.mppi.sigma_v * gaussian(random_);
                u.angular.z += cfg_.mppi.sigma_w * gaussian(random_);
                u = boundedControl(u, cfg_);
            }
            auto trajectory = evaluate(proposals[k]);
            costs[k] = trajectory.totalTrajCost;
            if (std::isfinite(costs[k]))
            {
                ++result.feasible_samples;
                min_cost = std::min(min_cost, costs[k]);
                keepBest(trajectory, proposals[k]);
            }
            result.samples.push_back(std::move(trajectory));
        }
        // Never subtract infinity from infinity, or average colliding trajectories.
        if (!std::isfinite(min_cost))
            continue;
        std::vector<double> weights(sample_count, 0.0);
        double sum = 0;
        for (std::size_t k = 0; k < sample_count; ++k)
        {
            if (std::isfinite(costs[k]))
                weights[k] = std::exp(-(double(costs[k]) - min_cost) / cfg_.mppi.temperature);
            sum += weights[k];
        }
        MppiControls updated = controls;
        for (std::size_t t = 0; t < controls.size(); ++t)
        {
            double delta_v = 0, delta_w = 0;
            for (std::size_t k = 0; k < sample_count; ++k)
            {
                const double weight = weights[k] / sum;
                // Effective perturbation after saturation, as in the handoff prototype.
                delta_v += weight * (proposals[k][t].linear.x - controls[t].linear.x);
                delta_w += weight * (proposals[k][t].angular.z - controls[t].angular.z);
            }
            updated[t].linear.x += cfg_.mppi.update_fraction * delta_v;
            updated[t].angular.z += cfg_.mppi.update_fraction * delta_w;
            updated[t] = boundedControl(updated[t], cfg_);
            if (fixed_forward_speed)
                updated[t].linear.x = *fixed_forward_speed;
        }
        auto updated_trajectory = evaluate(updated);
        keepBest(updated_trajectory, updated);
        // The weighted average can collide even when its constituent paths are safe.
        // Keep a checked nominal; do not send an unchecked average down the pipeline.
        if (std::isfinite(updated_trajectory.totalTrajCost))
        {
            controls = std::move(updated);
            // MPPI returns the checked weighted nominal, not the cheapest sample.
            result.trajectory = std::move(updated_trajectory);
            result.controls = controls;
            result.valid = true;
        }
        else if (result.valid)
            controls = result.controls;
    }
    return result;
}

bool evaluateMppiOutputTrajectory(dwa_Trajectory& trajectory,
                                  TrajectoryEvaluator& evaluator,
                                  const DynamicGapConfig& cfg,
                                  const std::vector<sensor_msgs::LaserScan>& future_scans,
                                  const std::vector<geometry_msgs::PoseStamped>& visible_plan,
                                  Gap* gap,
                                  std::optional<double> expected_spacing)
{
    trajectory.totalTrajCost = infinity;
    trajectory.TerminalPoseCost = infinity;
    trajectory.PoseCosts.clear();
    trajectory.PathCosts.clear();
    trajectory.RelVelPoseCosts.clear();
    if (expected_spacing && (!std::isfinite(*expected_spacing) || *expected_spacing < 0))
        return false;
    if (trajectory.pose_array.poses.size() != kGapTrajectoryPoseCount ||
        trajectory.positions.size() != kGapTrajectoryPoseCount ||
        trajectory.yaws.size() != kGapTrajectoryPoseCount ||
        trajectory.times.size() != kGapTrajectoryPoseCount ||
        future_scans.size() < kGapTrajectoryPoseCount ||
        trajectory.pose_array.header.frame_id != cfg.sensor_frame_id ||
        !std::isfinite(cfg.traj.integrate_stept) || cfg.traj.integrate_stept <= 0)
        return false;
    for (std::size_t i = 0; i < kGapTrajectoryPoseCount; ++i)
    {
        const auto& pose = trajectory.pose_array.poses[i];
        if (!std::isfinite(trajectory.times[i]) ||
            std::abs(trajectory.times[i] - i * cfg.traj.integrate_stept) > 1e-4 ||
            !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
            future_scans[i].ranges.empty())
            return false;
        if (expected_spacing && i > 0)
        {
            const auto& previous = trajectory.pose_array.poses[i - 1].position;
            const double distance = std::hypot(pose.position.x - previous.x, pose.position.y - previous.y);
            if (std::abs(distance - *expected_spacing) > std::max(1e-5, 1e-4 * *expected_spacing))
                return false;
        }
    }
    // This is the original evaluator and packaging objective, including its
    // existing gap-specific social/CBF terms. Do not export the internal common
    // endpoint objective or append an MPPI-only speed penalty to these fields.
    evaluator.dwa_evaluateTrajectory(trajectory.totalTrajCost, trajectory,
        trajectory.PoseCosts, trajectory.PathCosts, trajectory.TerminalPoseCost,
        future_scans, 0, visible_plan, gap, trajectory.RelVelPoseCosts);
    return std::isfinite(trajectory.totalTrajCost) &&
        trajectory.PoseCosts.size() == kGapTrajectoryPoseCount &&
        trajectory.PathCosts.size() == kGapTrajectoryPoseCount &&
        trajectory.RelVelPoseCosts.size() == kGapTrajectoryPoseCount;
}

float evaluateMppiTrajectory(dwa_Trajectory& trajectory,
                             TrajectoryEvaluator& evaluator,
                             const DynamicGapConfig& cfg,
                             const std::vector<sensor_msgs::LaserScan>& future_scans,
                             const std::vector<geometry_msgs::PoseStamped>& visible_plan,
                             Gap* gap)
{
    trajectory.totalTrajCost = infinity;
    trajectory.TerminalPoseCost = infinity;
    const auto& poses = trajectory.pose_array.poses;
    if (poses.size() < 2 || poses.size() > future_scans.size() ||
        poses.size() != trajectory.times.size())
        return infinity;
    for (std::size_t i = 0; i < poses.size(); ++i)
    {
        if (future_scans[i].ranges.empty() ||
            !std::isfinite(poses[i].position.x) || !std::isfinite(poses[i].position.y) ||
            std::abs(trajectory.times[i] - i * cfg.traj.integrate_stept) > 1e-4)
            return infinity;
    }
    evaluator.dwa_evaluateTrajectory(trajectory.totalTrajCost, trajectory,
        trajectory.PoseCosts, trajectory.PathCosts, trajectory.TerminalPoseCost,
        future_scans, 0, visible_plan, gap, trajectory.RelVelPoseCosts);
    if (!std::isfinite(trajectory.totalTrajCost) || trajectory.PoseCosts.size() != poses.size() ||
        trajectory.PathCosts.size() != poses.size() || trajectory.RelVelPoseCosts.size() != poses.size())
        return trajectory.totalTrajCost = infinity;

    // Reuse the native inflated-footprint checker between samples. Checking both
    // bracketing predictions is conservative about obstacle occupancy in that interval;
    // this does not invent interpolated scan correspondences for moving endpoints.
    const double spacing = std::max(0.01, 0.5 * double(cfg.rbt.r_inscr * cfg.traj.inf_ratio));
    for (std::size_t i = 1; i < poses.size(); ++i)
    {
        const auto& a = poses[i - 1].position;
        const auto& b = poses[i].position;
        const int steps = std::max(1, int(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / spacing)));
        for (int j = 1; j < steps; ++j)
        {
            geometry_msgs::Pose pose = poses[i - 1];
            const double fraction = double(j) / steps;
            pose.position.x = a.x + fraction * (b.x - a.x);
            pose.position.y = a.y + fraction * (b.y - a.y);
            if (!std::isfinite(evaluator.dwa_evaluatePose(pose, future_scans[i - 1])) ||
                !std::isfinite(evaluator.dwa_evaluatePose(pose, future_scans[i])))
                return trajectory.totalTrajCost = infinity;
        }
    }
    // Keep the optional native speed cost in the exported scalar components too:
    // pickTraj reconstructs terminal + mean(pose costs), not totalTrajCost.
    double speed_cost = 0;
    for (std::size_t i = 1; i < poses.size(); ++i)
    {
        const auto& a = poses[i - 1].position;
        const auto& b = poses[i].position;
        const float speed = std::hypot(b.x - a.x, b.y - a.y) / cfg.traj.integrate_stept;
        speed_cost += evaluator.calcSpeedCost(speed, cfg.rbt.vx_absmax);
    }
    const float extra = cfg.traj.w_speed * speed_cost / (poses.size() - 1);
    trajectory.TerminalPoseCost += extra;
    trajectory.totalTrajCost += extra;
    return trajectory.totalTrajCost;
}
}
