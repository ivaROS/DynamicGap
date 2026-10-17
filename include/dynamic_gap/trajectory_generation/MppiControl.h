#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>

#include <dynamic_gap/config/DynamicGapConfig.h>
#include <dynamic_gap/utils/dwa_traj.h>

namespace dynamic_gap
{
class TrajectoryController;
using MppiControls = std::vector<geometry_msgs::Twist>;

// Same updated-yaw Euler step as the existing Planner DWA diagnostic rollout.
// Normalizes the supplied quaternion (zero means identity), wraps yaw, and throws
// std::invalid_argument for nonfinite inputs or a nonpositive time step.
geometry_msgs::Pose integrateMppiUnicycle(const geometry_msgs::Pose& initial,
                                        const geometry_msgs::Twist& control,
                                        double dt);

// Preserve the existing Bezier samples and give each a valid tangent heading.
// Empty or nonfinite curves produce an empty reference.
geometry_msgs::PoseArray bezierReferencePoses(const std::vector<Eigen::Vector2f>& curve,
                                            const std::string& frame);

// Repeatedly use the production target selector and lookahead law at a LOCAL
// simulated pose. Bounds are signed +/- the configured linear/angular limits.
// The endpoint uses the existing waypoint tolerance; its last approach speed is
// limited to the remaining distance per step. Invalid inputs return no controls.
MppiControls generateNominalControlsFromBezier(const geometry_msgs::PoseArray& reference,
                                             const geometry_msgs::Pose& initial,
                                             TrajectoryController& controller,
                                             const DynamicGapConfig& cfg,
                                             std::size_t horizon,
                                             double dt);

// H controls produce H+1 states at times 0, dt, ..., H*dt. Invalid inputs produce
// an empty trajectory. No controller, scan, TF, or real robot state is modified.
dwa_Trajectory rolloutMppiControls(const geometry_msgs::Pose& initial,
                                  const MppiControls& controls,
                                  double dt,
                                  const std::string& frame);
}  // namespace dynamic_gap
