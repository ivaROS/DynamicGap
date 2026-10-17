#include <dynamic_gap/trajectory_generation/MppiControl.h>
#include <dynamic_gap/trajectory_tracking/TrajectoryController.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dynamic_gap
{
namespace
{
bool validPose(const geometry_msgs::Pose& pose)
{
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
           std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
           std::isfinite(pose.orientation.w);
}

bool validControl(const geometry_msgs::Twist& control)
{
    return std::isfinite(control.linear.x) && std::isfinite(control.angular.z);
}

geometry_msgs::Pose planarPose(const geometry_msgs::Pose& input)
{
    geometry_msgs::Pose result = input;
    const auto& q = input.orientation;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    double yaw = 0.0;
    if (norm > 1e-12)
    {
        tf::Quaternion normalized(q.x / norm, q.y / norm, q.z / norm, q.w / norm);
        yaw = tf::getYaw(normalized);
    }
    result.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return result;
}
}  // namespace

geometry_msgs::Pose integrateMppiUnicycle(const geometry_msgs::Pose& initial,
                                        const geometry_msgs::Twist& control,
                                        double dt)
{
    if (!validPose(initial) || !validControl(control) || !std::isfinite(dt) || dt <= 0.0)
        throw std::invalid_argument("MPPI unicycle step requires finite pose/control and positive dt");

    geometry_msgs::Pose next = planarPose(initial);
    const double yaw = std::remainder(tf::getYaw(next.orientation) + control.angular.z * dt,
                                     2.0 * M_PI);
    next.position.x += control.linear.x * std::cos(yaw) * dt;
    next.position.y += control.linear.x * std::sin(yaw) * dt;
    next.orientation = tf::createQuaternionMsgFromYaw(yaw);
    if (!validPose(next))
        throw std::invalid_argument("MPPI unicycle step overflowed");
    return next;
}

geometry_msgs::PoseArray bezierReferencePoses(const std::vector<Eigen::Vector2f>& curve,
                                            const std::string& frame)
{
    geometry_msgs::PoseArray reference;
    reference.header.frame_id = frame;
    for (const auto& point : curve)
        if (!point.allFinite())
            return reference;

    reference.poses.reserve(curve.size());
    double previousYaw = 0.0;
    for (std::size_t i = 0; i < curve.size(); ++i)
    {
        // Skip coincident samples when determining the tangent; never normalize
        // a zero-length segment. The endpoint keeps the last valid heading.
        for (std::size_t j = i + 1; j < curve.size(); ++j)
        {
            const Eigen::Vector2f tangent = curve[j] - curve[i];
            if (tangent.squaredNorm() > 1e-12f)
            {
                previousYaw = std::atan2(tangent.y(), tangent.x());
                break;
            }
        }
        geometry_msgs::Pose pose;
        pose.position.x = curve[i].x();
        pose.position.y = curve[i].y();
        pose.orientation = tf::createQuaternionMsgFromYaw(previousYaw);
        reference.poses.push_back(pose);
    }
    return reference;
}

MppiControls generateNominalControlsFromBezier(const geometry_msgs::PoseArray& reference,
                                             const geometry_msgs::Pose& initial,
                                             TrajectoryController& controller,
                                             const DynamicGapConfig& cfg,
                                             std::size_t horizon,
                                             double dt)
{
    MppiControls controls;
    if (reference.poses.empty() || horizon == 0 || !validPose(initial) ||
        !std::isfinite(dt) || dt <= 0.0 || !std::isfinite(cfg.rbt.vx_absmax) ||
        cfg.rbt.vx_absmax < 0.0f || !std::isfinite(cfg.rbt.vang_absmax) ||
        cfg.rbt.vang_absmax < 0.0f || !std::isfinite(cfg.rbt.r_inscr) ||
        !std::isfinite(cfg.traj.inf_ratio) || cfg.rbt.r_inscr * cfg.traj.inf_ratio <= 0.0f ||
        !std::isfinite(cfg.goal.xy_waypoint_tolerance) || cfg.ctrl.ctrl_ahead_pose < 0)
        return controls;

    geometry_msgs::PoseArray normalizedReference = reference;
    for (auto& pose : normalizedReference.poses)
    {
        if (!validPose(pose))
            return controls;
        pose = planarPose(pose);
    }

    geometry_msgs::Pose simulated = planarPose(initial);
    const auto& endpoint = normalizedReference.poses.back().position;
    const double tolerance = std::max(0.0f, cfg.goal.xy_waypoint_tolerance);
    controls.reserve(horizon);
    for (std::size_t step = 0; step < horizon; ++step)
    {
        const double distance = std::hypot(endpoint.x - simulated.position.x,
                                           endpoint.y - simulated.position.y);
        if (distance <= tolerance)
        {
            controls.resize(horizon);  // stop and pad the remaining horizon
            break;
        }

        const int target = controller.extractTargetPoseIdx(simulated, normalizedReference);
        if (target < 0 || target >= static_cast<int>(normalizedReference.poses.size()))
            return {};

        float speed = cfg.rbt.vx_absmax;
        if (target == static_cast<int>(normalizedReference.poses.size()) - 1)
            speed = static_cast<float>(std::min(static_cast<double>(speed), distance / dt));
        geometry_msgs::Twist command = controller.constantVelocityControlLawNonHolonomicLookahead(
            simulated, normalizedReference.poses[target], speed);
        if (!validControl(command))
            return {};
        command.linear.x = std::clamp(command.linear.x, -static_cast<double>(cfg.rbt.vx_absmax),
                                     static_cast<double>(cfg.rbt.vx_absmax));
        command.angular.z = std::clamp(command.angular.z, -static_cast<double>(cfg.rbt.vang_absmax),
                                      static_cast<double>(cfg.rbt.vang_absmax));
        controls.push_back(command);
        simulated = integrateMppiUnicycle(simulated, command, dt);
    }
    return controls;
}

dwa_Trajectory rolloutMppiControls(const geometry_msgs::Pose& initial,
                                  const MppiControls& controls,
                                  double dt,
                                  const std::string& frame)
{
    dwa_Trajectory rollout;
    if (!validPose(initial) || !std::isfinite(dt) || dt <= 0.0)
        return rollout;
    for (const auto& command : controls)
        if (!validControl(command))
            return rollout;

    geometry_msgs::Pose simulated = planarPose(initial);
    rollout.positions.reserve(controls.size() + 1);
    rollout.yaws.reserve(controls.size() + 1);
    rollout.times.reserve(controls.size() + 1);
    rollout.pose_array.header.frame_id = frame;
    rollout.pose_array.poses.reserve(controls.size() + 1);
    const auto append = [&](double time)
    {
        rollout.positions.emplace_back(simulated.position.x, simulated.position.y);
        rollout.yaws.push_back(tf::getYaw(simulated.orientation));
        rollout.times.push_back(time);
        rollout.pose_array.poses.push_back(simulated);
    };
    append(0.0);
    try
    {
        for (std::size_t i = 0; i < controls.size(); ++i)
        {
            simulated = integrateMppiUnicycle(simulated, controls[i], dt);
            append((i + 1) * dt);
        }
    }
    catch (const std::invalid_argument&)
    {
        return {};
    }
    if (!controls.empty())
    {
        rollout.v_cmd = controls.front().linear.x;
        rollout.w_cmd = controls.front().angular.z;
    }
    return rollout;
}
}  // namespace dynamic_gap
