#pragma once

#include <cstddef>
#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Core>

namespace dynamic_gap
{
// Shared output layout of generateGapTrajsV2's original Bezier branch and MPPI.
// Downstream evaluation pairs pose i with futureScans[i] at integrate_stept.
inline constexpr std::size_t kGapTrajectoryPoseCount = 11;
inline constexpr std::size_t kGapTrajectoryControlCount = kGapTrajectoryPoseCount - 1;

struct GapBezierCommand
{
    float linear = 0;
    float angular = 0;
    bool valid = false;
};

// Extracted from the original Bezier branch, preserving its acceleration proxy,
// curvature estimate and lambda/sigma timing. Both generators use the same code.
inline GapBezierCommand computeGapBezierCommand(const std::vector<Eigen::Vector2f>& curve,
                                                float current_speed, float max_speed,
                                                float max_omega, float dt)
{
    GapBezierCommand command;
    if (curve.size() < 3 || !std::isfinite(current_speed) || current_speed < 0 ||
        !std::isfinite(max_speed) || max_speed <= 0 ||
        !std::isfinite(max_omega) || max_omega <= 0 || !std::isfinite(dt) || dt <= 0)
        return command;
    const Eigen::Vector2f v1 = curve[1] - curve[0];
    const Eigen::Vector2f v2 = curve[2] - curve[1];
    const float length = v1.norm();
    if (!v1.allFinite() || !v2.allFinite() || length < 1e-6f)
        return command;
    const float v_des = std::min(current_speed + max_omega * dt, max_speed);
    const float curvature = (v1.x() * v2.y() - v1.y() * v2.x()) / std::pow(length, 3.0f);
    const float w_des = v_des * curvature;
    const float lambda = std::min(1.0f, v_des / length);
    const float angular = std::fabs(lambda * w_des);
    const float sigma = angular == 0 ? 1.0f : std::min(max_omega / angular, 1.0f);
    command.linear = sigma * lambda * v_des;
    command.angular = sigma * lambda * w_des;
    command.valid = std::isfinite(command.linear) && std::isfinite(command.angular);
    return command;
}
}
