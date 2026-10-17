#pragma once

#include <dynamic_gap/trajectory_generation/MppiControl.h>
#include <dynamic_gap/trajectory_evaluation/TrajectoryEvaluator.h>
#include <cstdint>
#include <functional>
#include <random>

namespace dynamic_gap
{
struct MppiResult
{
    dwa_Trajectory trajectory;
    MppiControls controls;
    std::vector<dwa_Trajectory> samples; // every iteration, including rejected rollouts
    std::size_t feasible_samples = 0;
    bool valid = false;
};

// A fresh instance/nominal per current candidate; no gap identity tracking or warm start.
class MppiOptimizer
{
public:
    using Score = std::function<float(dwa_Trajectory&)>;
    MppiOptimizer(const DynamicGapConfig& cfg, std::uint32_t seed);
    MppiResult optimize(const geometry_msgs::Pose& initial,
                        const MppiControls& nominal,
                        std::size_t sample_count,
                        const std::string& frame,
                        const Score& score);

private:
    const DynamicGapConfig& cfg_;
    std::mt19937 random_;
};

// The native objective plus native collision checks between prediction timestamps.
// No Python cost or obstacle representation is used.
float evaluateMppiTrajectory(dwa_Trajectory& trajectory,
                             TrajectoryEvaluator& evaluator,
                             const DynamicGapConfig& cfg,
                             const std::vector<sensor_msgs::LaserScan>& future_scans,
                             const std::vector<geometry_msgs::PoseStamped>& visible_plan,
                             Gap* gap);
}
