#include <dynamic_gap/trajectory_generation/GoalApproach.h>
#include <dynamic_gap/trajectory_generation/MppiControl.h>
#include <dynamic_gap/global_plan_management/GlobalPlanManager.h>
#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>

namespace dynamic_gap
{
namespace
{
geometry_msgs::Pose pose(double x = 0, double y = 0, double yaw = 0)
{
    geometry_msgs::Pose p;
    p.position.x = x; p.position.y = y;
    p.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return p;
}

DynamicGapConfig config()
{
    DynamicGapConfig cfg;
    cfg.rbt.r_inscr = .225;
    cfg.rbt.vx_absmax = .75;
    cfg.rbt.vang_absmax = 1;
    cfg.traj.inf_ratio = 1;
    cfg.traj.integrate_stept = .5;
    cfg.goal.xy_global_goal_tolerance = .25;
    cfg.goal.yaw_global_goal_tolerance = 6.28;
    return cfg;
}

// Laser input, not oracle collision answers: raycast room walls and an optional
// moving pedestrian disk, then use the same production final-approach planner.
std::vector<sensor_msgs::LaserScan> scans(double wall_x = 8, double wall_y = -8,
                                        bool pedestrian = false, double time = 0)
{
    std::vector<sensor_msgs::LaserScan> result(11);
    for (std::size_t k = 0; k < result.size(); ++k)
    {
        auto& s = result[k];
        s.header.frame_id = "laser";
        s.angle_min = -M_PI;
        s.angle_increment = 2 * M_PI / 511;
        s.angle_max = M_PI;
        s.range_min = .03;
        s.range_max = 10;
        for (int i = 0; i < 512; ++i)
        {
            const double angle = s.angle_min + i*s.angle_increment;
            const double dx = std::cos(angle), dy = std::sin(angle);
            double range = 8;
            if (dx > 1e-6) range = std::min(range, wall_x/dx);
            if (dy < -1e-6) range = std::min(range, wall_y/dy);
            if (pedestrian)
            {
                const double cx = .8, cy = 1.0 - (time + .5*k);
                const double b = cx*dx + cy*dy;
                const double discriminant = b*b - (cx*cx + cy*cy - .3*.3);
                if (discriminant >= 0 && b - std::sqrt(discriminant) > 0)
                    range = std::min(range, b - std::sqrt(discriminant));
            }
            s.ranges.push_back(range);
        }
    }
    return result;
}

TEST(GoalApproach, ReachesWallAdjacentGoalWithoutOvershootOrFullSpeedTracking)
{
    const auto cfg = config();
    const auto predictions = scans(1.9, -.4);
    auto initial = pose();
    const auto goal = pose(1.5);
    double previous = 1.5;
    bool slowed = false;
    for (int i = 0; i < 100; ++i)
    {
        auto result = planGoalApproach(initial, goal, predictions, cfg);
        ASSERT_TRUE(result.safe);
        EXPECT_LE(result.command.linear.x, .5);
        EXPECT_GE(result.command.linear.x, 0);
        initial = integrateMppiUnicycle(initial, result.command, .1);
        const double distance = std::hypot(initial.position.x - goal.position.x, initial.position.y);
        EXPECT_LE(distance, previous + 1e-6);
        if (distance < .5 && result.command.linear.x < .45) slowed = true;
        previous = distance;
        if (distance < cfg.goal.xy_global_goal_tolerance) break;
    }
    EXPECT_LT(previous, cfg.goal.xy_global_goal_tolerance);
    EXPECT_TRUE(slowed);
}

TEST(GoalApproach, GoalBehindRobotAlignsInPlaceInsteadOfSweepingAway)
{
    const auto cfg = config();
    auto initial = pose();
    const auto goal = pose(-1.5);
    const auto predictions = scans();
    const auto first = planGoalApproach(initial, goal, predictions, cfg);
    ASSERT_TRUE(first.safe);
    EXPECT_DOUBLE_EQ(first.command.linear.x, 0);
    EXPECT_GT(std::abs(first.command.angular.z), .5);
    double previous = 1.5;
    for (int i = 0; i < 160; ++i)
    {
        auto result = planGoalApproach(initial, goal, predictions, cfg);
        ASSERT_TRUE(result.safe);
        initial = integrateMppiUnicycle(initial, result.command, .1);
        const double distance = std::hypot(initial.position.x + 1.5, initial.position.y);
        EXPECT_LE(distance, previous + 1e-6);
        previous = distance;
        if (distance < .25) break;
    }
    EXPECT_LT(previous, .25);
}

TEST(GoalApproach, PublishedCommandIsCheckedForAnEntireDelayedControlInterval)
{
    const auto result = planGoalApproach(pose(), pose(1.5,.5), scans(), config());
    ASSERT_TRUE(result.safe);
    ASSERT_GE(result.path.poses.size(), 6u);
    double x = 0, y = 0;
    for (int j=1; j<=5; ++j)
    {
        const double yaw = j * .05 * result.command.angular.z;
        x += .05 * result.command.linear.x * std::cos(yaw);
        y += .05 * result.command.linear.x * std::sin(yaw);
        EXPECT_NEAR(result.path.poses[j].position.x, x, 1e-8);
        EXPECT_NEAR(result.path.poses[j].position.y, y, 1e-8);
        EXPECT_NEAR(tf::getYaw(result.path.poses[j].orientation), yaw, 1e-8);
    }
}

TEST(GoalApproach, HonorsFinalYawToleranceWithZeroTranslation)
{
    auto cfg = config();
    cfg.goal.yaw_global_goal_tolerance = .05;
    auto initial = pose(0,0,1.5);
    for (int j=0; j<60 && std::abs(tf::getYaw(initial.orientation))>=.05; ++j)
    {
        const auto result = planGoalApproach(initial, pose(), scans(), cfg);
        ASSERT_TRUE(result.safe);
        EXPECT_DOUBLE_EQ(result.command.linear.x, 0);
        initial = integrateMppiUnicycle(initial, result.command, .1);
    }
    EXPECT_LT(std::abs(tf::getYaw(initial.orientation)), .05);
}

TEST(GoalApproach, CrossingPersonIsAvoidedAndRobotResumesToGoal)
{
    const auto cfg = config();
    auto initial = pose();
    const auto goal = pose(1.5);
    bool yielded = false;
    double min_center_distance = 100;
    for (int i = 0; i < 140; ++i)
    {
        const double t = .1 * i;
        auto result = planGoalApproach(initial, goal, scans(8, -8, true, t), cfg);
        if (t < 2 && result.command.linear.x < .1) yielded = true;
        // Check the physical moving disk at 10 ms intervals, independently of
        // the planner's prediction timestamps and sensor endpoint checker.
        for (int j = 0; j < 10; ++j)
        {
            initial = integrateMppiUnicycle(initial, result.command, .01);
            min_center_distance = std::min(min_center_distance,
                std::hypot(initial.position.x - .8, initial.position.y - (1 - t - .01*(j+1))));
        }
        if (std::hypot(initial.position.x - 1.5, initial.position.y) < .25) break;
    }
    EXPECT_TRUE(yielded);
    EXPECT_GT(min_center_distance, .3 + cfg.rbt.r_inscr);
    EXPECT_LT(std::hypot(initial.position.x - 1.5, initial.position.y), .25);
}

TEST(GoalApproach, SolidBlockageNeverGetsAnUncheckedForwardCommand)
{
    auto result = planGoalApproach(pose(), pose(1.5), scans(.4), config());
    EXPECT_DOUBLE_EQ(result.command.linear.x, 0);
}

TEST(GoalApproach, MissingOrInvalidPredictionsFailClosed)
{
    auto cfg = config();
    auto predictions = scans();
    predictions[1].ranges.clear();
    EXPECT_FALSE(planGoalApproach(pose(), pose(1.5), predictions, cfg).safe);
    EXPECT_FALSE(planGoalApproach(pose(), pose(1.5), {}, cfg).safe);
    cfg.goal_approach.clearance = -1;
    EXPECT_FALSE(planGoalApproach(pose(), pose(1.5), scans(), cfg).safe);
    cfg = config();
    auto invalid = pose();
    invalid.orientation.w = 0;
    EXPECT_FALSE(planGoalApproach(invalid, pose(1.5), scans(), cfg).safe);
}

TEST(GoalApproach, StaticMapPreventsShortcutsThroughWallsAndUnknownSpace)
{
    nav_msgs::OccupancyGrid grid;
    grid.info.width = grid.info.height = 50;
    grid.info.resolution = .1;
    grid.info.origin.orientation.w = 1;
    grid.data.assign(2500, 0);
    EXPECT_TRUE(goalApproachStaticCorridorClear(grid, pose(1,1), pose(3,1), .325));
    for (int y=0; y<50; ++y) grid.data[y*50+20] = 100;
    EXPECT_FALSE(goalApproachStaticCorridorClear(grid, pose(1,1), pose(3,1), .325));
    EXPECT_TRUE(goalApproachStaticCorridorClear(grid, pose(1,1), pose(1,3), .325));
    grid.data[15*50+10] = -1;
    EXPECT_FALSE(goalApproachStaticCorridorClear(grid, pose(1,1), pose(1,3), .325));
    EXPECT_FALSE(goalApproachStaticCorridorClear(grid, pose(.1,1), pose(.1,3), .325));
}

TEST(GoalApproach, GlobalRouteSnapshotUsesRobotFrameWithoutChangingStoredMapPath)
{
    auto cfg = config();
    GlobalPlanManager manager(cfg);
    std::vector<geometry_msgs::PoseStamped> plan(2);
    plan[0].header.frame_id = plan[1].header.frame_id = "map";
    plan[0].pose = pose(10,5); plan[1].pose = pose(12,5);
    manager.updateGlobalPathMapFrame(plan);
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "base";
    transform.child_frame_id = "map";
    transform.transform.rotation.w = 1;
    transform.transform.translation.x = -10;
    transform.transform.translation.y = -5;
    const auto local = manager.getGlobalPathRobotFrame(transform);
    ASSERT_EQ(local.size(), 2u);
    EXPECT_DOUBLE_EQ(local[0].pose.position.x, 0);
    EXPECT_DOUBLE_EQ(local[1].pose.position.x, 2);
    EXPECT_DOUBLE_EQ(local[1].pose.position.y, 0);
    EXPECT_DOUBLE_EQ(manager.getGlobalPathOdomFrame()[1].pose.position.x, 12);
}
}
}
