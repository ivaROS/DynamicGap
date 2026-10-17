#include <dynamic_gap/trajectory_generation/MppiOptimizer.h>
#include <gtest/gtest.h>

#include <limits>

namespace dynamic_gap
{
namespace
{
geometry_msgs::Pose initialPose()
{
    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    return pose;
}

TEST(MppiOptimizer, WeightedNominalUsesWholeHorizonAndImprovesSeed)
{
    ros::Time::init();
    DynamicGapConfig cfg;
    cfg.mppi.iterations = 3;
    cfg.mppi.sigma_v = 0.6;
    cfg.mppi.sigma_w = 0.1;
    cfg.mppi.temperature = 0.1;
    cfg.traj.integrate_stept = 0.2;
    MppiOptimizer optimizer(cfg, 42);
    MppiControls nominal(12);
    auto score = [](dwa_Trajectory& trajectory) {
        const auto& end = trajectory.pose_array.poses.back().position;
        return float(std::pow(end.x - 1.0, 2) + 2 * end.y * end.y);
    };
    const auto result = optimizer.optimize(initialPose(), nominal, 128, "laser", score);
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.samples.size(), 384u);
    EXPECT_EQ(result.feasible_samples, 384u);
    ASSERT_EQ(result.controls.size(), nominal.size());
    ASSERT_EQ(result.trajectory.pose_array.poses.size(), nominal.size() + 1);
    EXPECT_LT(result.trajectory.totalTrajCost, 0.5f);
    bool matches_random_sample = false;
    for (const auto& sample : result.samples)
    {
        bool matches = true;
        for (std::size_t i = 0; i < sample.positions.size(); ++i)
            matches = matches && (sample.positions[i] - result.trajectory.positions[i]).norm() < 1e-6;
        matches_random_sample = matches_random_sample || matches;
    }
    EXPECT_FALSE(matches_random_sample) << "MPPI must return the checked weighted nominal, not shooting's best random sample";
    const auto replay = rolloutMppiControls(initialPose(), result.controls, cfg.traj.integrate_stept, "laser");
    for (std::size_t i = 0; i < result.controls.size(); ++i)
    {
        EXPECT_LE(std::abs(result.controls[i].linear.x), cfg.rbt.vx_absmax);
        EXPECT_LE(std::abs(result.controls[i].angular.z), cfg.rbt.vang_absmax);
        EXPECT_LT((replay.positions[i] - result.trajectory.positions[i]).norm(), 1e-6);
    }
}

TEST(MppiOptimizer, AllCollidingSamplesNeverProduceAnExecutableNominal)
{
    ros::Time::init();
    DynamicGapConfig cfg;
    cfg.mppi.iterations = 2;
    MppiOptimizer optimizer(cfg, 7);
    const auto result = optimizer.optimize(initialPose(), MppiControls(10), 19, "laser",
        [](dwa_Trajectory&) { return std::numeric_limits<float>::infinity(); });
    EXPECT_FALSE(result.valid);
    EXPECT_TRUE(result.controls.empty());
    EXPECT_EQ(result.samples.size(), 38u);
    EXPECT_EQ(result.feasible_samples, 0u);
    for (const auto& sample : result.samples) EXPECT_TRUE(std::isinf(sample.totalTrajCost));
}

TEST(MppiOptimizer, RejectsInvalidTemperatureAndNonfiniteSeeds)
{
    ros::Time::init();
    DynamicGapConfig cfg;
    cfg.mppi.temperature = 0;
    EXPECT_THROW(MppiOptimizer(cfg, 1), std::invalid_argument);
    cfg.mppi.temperature = 1;
    MppiOptimizer optimizer(cfg, 1);
    MppiControls nominal(10);
    nominal[3].angular.z = std::numeric_limits<double>::quiet_NaN();
    const auto result = optimizer.optimize(initialPose(), nominal, 20, "laser",
        [](dwa_Trajectory&) { return 0.0f; });
    EXPECT_FALSE(result.valid);
    EXPECT_TRUE(result.samples.empty());
}

TEST(MppiOptimizer, CollidingWeightedMeanUsesCheckedFallback)
{
    ros::Time::init();
    DynamicGapConfig cfg;
    cfg.traj.integrate_stept = 1;
    cfg.mppi.sigma_v = 1;
    cfg.mppi.sigma_w = 0;
    MppiOptimizer optimizer(cfg, 7);
    const auto result = optimizer.optimize(initialPose(), MppiControls(1), 500, "laser",
        [](dwa_Trajectory& trajectory) {
            return std::abs(trajectory.positions.back().x()) >= 0.3f ? 0.0f :
                std::numeric_limits<float>::infinity();
        });
    ASSERT_TRUE(result.valid);
    EXPECT_GE(std::abs(result.trajectory.positions.back().x()), 0.3f);
    EXPECT_TRUE(std::isfinite(result.controls.front().linear.x));
}

TEST(MppiOptimizer, SeededRunsAreReproducibleAndWeightsHandleLargeCosts)
{
    ros::Time::init();
    DynamicGapConfig cfg;
    cfg.mppi.temperature = 1e-6;
    MppiOptimizer first(cfg, 42), second(cfg, 42);
    const auto score = [](dwa_Trajectory& trajectory) {
        return float(1e15 + 1e13 * std::abs(trajectory.positions.back().x() - 1));
    };
    const auto a = first.optimize(initialPose(), MppiControls(10), 31, "laser", score);
    const auto b = second.optimize(initialPose(), MppiControls(10), 31, "laser", score);
    ASSERT_TRUE(a.valid);
    ASSERT_TRUE(b.valid);
    for (std::size_t t = 0; t < a.controls.size(); ++t)
    {
        EXPECT_TRUE(std::isfinite(a.controls[t].linear.x));
        EXPECT_DOUBLE_EQ(a.controls[t].linear.x, b.controls[t].linear.x);
        EXPECT_DOUBLE_EQ(a.controls[t].angular.z, b.controls[t].angular.z);
    }
}

TEST(MppiNativeCost, RejectsBetweenStepCollisionAndMisalignedTiming)
{
    ros::Time::init();
    DynamicGapConfig cfg;
    cfg.traj.integrate_stept = 1;
    cfg.rbt.r_inscr = 0.1;
    cfg.traj.inf_ratio = 1;
    TrajectoryEvaluator evaluator(cfg);
    geometry_msgs::TransformStamped identity;
    identity.transform.rotation.w = 1;
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = 2;
    goal.pose.orientation.w = 1;
    evaluator.transformGlobalPathLocalWaypointToRbtFrame(goal, identity);
    sensor_msgs::LaserScan scan;
    scan.ranges.resize(512, 10.0);
    std::vector<sensor_msgs::LaserScan> scans(2, scan);
    MppiControls controls(1);
    controls[0].linear.x = 1;
    auto free = rolloutMppiControls(initialPose(), controls, 1, "laser");
    EXPECT_TRUE(std::isfinite(evaluateMppiTrajectory(free, evaluator, cfg, scans, {}, nullptr)));
    // Clear now, but occupied when the robot arrives: using only the current
    // scan would incorrectly accept this trajectory.
    scans[1].ranges[256] = 1.0;
    auto future_collision = rolloutMppiControls(initialPose(), controls, 1, "laser");
    EXPECT_TRUE(std::isinf(evaluateMppiTrajectory(future_collision, evaluator, cfg, scans, {}, nullptr)));
    // Endpoints (0,0) and (1,0) are clear, but the robot crosses the scan return.
    scans[0].ranges[256] = 0.5;
    scans[1].ranges[256] = 0.5;
    auto blocked = rolloutMppiControls(initialPose(), controls, 1, "laser");
    EXPECT_TRUE(std::isinf(evaluateMppiTrajectory(blocked, evaluator, cfg, scans, {}, nullptr)));
    auto misaligned = free;
    misaligned.times.back() = 0.3;
    EXPECT_TRUE(std::isinf(evaluateMppiTrajectory(misaligned, evaluator, cfg, scans, {}, nullptr)));
}
}  // namespace
}  // namespace dynamic_gap
