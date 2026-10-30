#pragma once

#include <dynamic_gap/config/DynamicGapConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

namespace dynamic_gap
{
struct GoalApproachResult
{
    geometry_msgs::Twist command;
    geometry_msgs::PoseArray path;
    std::vector<float> times;
    bool safe = false;
};

// Start/goal are in the grid's header frame; occupied, unknown, and off-map
// space block the swept footprint. Dynamic obstacles are checked by the rollout.
bool goalApproachStaticCorridorClear(const nav_msgs::OccupancyGrid& map,
                                    const geometry_msgs::Pose& start,
                                    const geometry_msgs::Pose& goal, double radius);

// Poses and all predicted scans share one fixed sensor frame. The returned first
// command is the command checked by the rollout (no fixed-speed path tracker).
GoalApproachResult planGoalApproach(const geometry_msgs::Pose& initial,
                                   const geometry_msgs::Pose& goal,
                                   const std::vector<sensor_msgs::LaserScan>& scans,
                                   const DynamicGapConfig& cfg);
}
