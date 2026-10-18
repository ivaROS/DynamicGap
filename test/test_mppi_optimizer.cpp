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

TEST(MppiGapContract, SharedBezierTimingPreservesOriginalSpeedAndCurvatureScaling)
{
    // Known outputs of the original inline timing calculation, including its
    // max-omega acceleration proxy, lambda scaling and angular saturation.
    const auto straight = computeGapBezierCommand({{0, 0}, {0.25f, 0}, {0.5f, 0}}, 0, 1, 1, 0.5f);
    ASSERT_TRUE(straight.valid);
    EXPECT_FLOAT_EQ(straight.linear, 0.5f);
    EXPECT_FLOAT_EQ(straight.angular, 0);
    const auto turning = computeGapBezierCommand({{0, 0}, {0.2f, 0}, {0.4f, 0.1f}}, 0, 1, 1, 0.5f);
    ASSERT_TRUE(turning.valid);
    EXPECT_NEAR(turning.linear, 0.4f, 1e-6);
    EXPECT_NEAR(turning.angular, 1.0f, 1e-6);
    const auto large_segment = computeGapBezierCommand({{0, 0}, {2, 0}, {4, 0}}, 0, 1, 1, 0.5f);
    ASSERT_TRUE(large_segment.valid);
    EXPECT_FLOAT_EQ(large_segment.linear, 0.125f);
    const auto moving = computeGapBezierCommand({{0, 0}, {0.25f, 0}, {0.5f, 0}}, 0.8f, 1, 1, 0.5f);
    ASSERT_TRUE(moving.valid);
    EXPECT_FLOAT_EQ(moving.linear, 1);
    EXPECT_FALSE(computeGapBezierCommand({{0, 0}, {0, 0}, {0, 0}}, 0, 1, 1, 0.5f).valid);
}

TEST(MppiGapContract, SteeringOptimizationKeepsNativeSpacingAndTotalDistance)
{
    ros::Time::init();
    DynamicGapConfig cfg;
    cfg.traj.integrate_stept = 0.5;
    cfg.mppi.sigma_v = 100; // a production fixed-speed candidate must ignore this
    cfg.mppi.sigma_w = 0.4;
    cfg.mppi.iterations = 2;
    MppiControls seed(kGapTrajectoryControlCount);
    for (std::size_t i = 0; i < seed.size(); ++i) seed[i].linear.x = i % 2 ? 0 : 1;
    const double speed = 0.4;
    MppiOptimizer optimizer(cfg, 42);
    const auto result = optimizer.optimize(initialPose(), seed, 64, "laser",
        [](dwa_Trajectory& path) {
            const auto end = path.positions.back();
            return (end - Eigen::Vector2f(1.5f, 1.0f)).squaredNorm();
        }, speed);
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(result.samples.size(), 128u);
    const auto check = [&](const dwa_Trajectory& path) {
        ASSERT_EQ(path.positions.size(), 11u);
        ASSERT_EQ(path.times.size(), 11u);
        double length = 0;
        for (std::size_t i = 1; i < path.positions.size(); ++i)
        {
            const double spacing = (path.positions[i] - path.positions[i - 1]).norm();
            EXPECT_NEAR(spacing, speed * cfg.traj.integrate_stept, 1e-6);
            EXPECT_FLOAT_EQ(path.times[i], i * cfg.traj.integrate_stept);
            length += spacing;
        }
        EXPECT_NEAR(length, 10 * speed * cfg.traj.integrate_stept, 1e-5);
    };
    check(result.trajectory);
    for (const auto& sample : result.samples) check(sample);
    for (const auto& u : result.controls) EXPECT_DOUBLE_EQ(u.linear.x, speed);
    EXPECT_LT(result.trajectory.totalTrajCost, 1.25f); // steering still optimizes
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

TEST(MppiNativeOutput, KeepsOriginalLayoutAndRecomputesOriginalCostComponents)
{
    ros::Time::init();
    DynamicGapConfig cfg;
    cfg.sensor_frame_id = "laser";
    cfg.traj.integrate_stept = 0.5;
    cfg.traj.w_speed = 2.0; // must not leak the internal speed penalty downstream
    TrajectoryEvaluator evaluator(cfg);
    geometry_msgs::TransformStamped identity;
    identity.transform.rotation.w = 1;
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = 4;
    goal.pose.orientation.w = 1;
    evaluator.transformGlobalPathLocalWaypointToRbtFrame(goal, identity);
    sensor_msgs::LaserScan scan;
    scan.ranges.assign(512, 10.0);
    std::vector<sensor_msgs::LaserScan> scans(kGapTrajectoryPoseCount, scan);
    MppiControls controls(kGapTrajectoryControlCount);
    for (auto& u : controls) u.linear.x = 0.2;
    auto output = rolloutMppiControls(initialPose(), controls, cfg.traj.integrate_stept, cfg.sensor_frame_id);
    auto original = output;
    evaluator.dwa_evaluateTrajectory(original.totalTrajCost, original, original.PoseCosts,
        original.PathCosts, original.TerminalPoseCost, scans, 0, {}, nullptr, original.RelVelPoseCosts);

    // Simulate optimizer-internal cost fields; the output boundary must replace them.
    output.PoseCosts.assign(11, 1234);
    output.TerminalPoseCost = 5678;
    ASSERT_TRUE(evaluateMppiOutputTrajectory(output, evaluator, cfg, scans, {}, nullptr, 0.1));
    ASSERT_EQ(output.pose_array.poses.size(), 11u);
    ASSERT_EQ(output.times.size(), 11u);
    EXPECT_FLOAT_EQ(output.times.back(), 5.0f);
    for (std::size_t i = 0; i < 11; ++i)
        EXPECT_FLOAT_EQ(output.times[i], i * 0.5f);
    EXPECT_EQ(output.PoseCosts, original.PoseCosts);
    EXPECT_EQ(output.PathCosts, original.PathCosts);
    EXPECT_EQ(output.RelVelPoseCosts, original.RelVelPoseCosts);
    EXPECT_FLOAT_EQ(output.TerminalPoseCost, original.TerminalPoseCost);
    EXPECT_FLOAT_EQ(output.totalTrajCost, original.totalTrajCost);
    auto wrong_spacing = output;
    wrong_spacing.pose_array.poses[5].position.y += 0.1;
    EXPECT_FALSE(evaluateMppiOutputTrajectory(wrong_spacing, evaluator, cfg, scans, {}, nullptr, 0.1));

    for (std::size_t count : {5u, 20u})
    {
        auto wrong_size = rolloutMppiControls(initialPose(), MppiControls(count), 0.5, "laser");
        EXPECT_FALSE(evaluateMppiOutputTrajectory(wrong_size, evaluator, cfg, scans, {}, nullptr));
    }
    auto wrong_time = output;
    wrong_time.times.back() = 4.9;
    EXPECT_FALSE(evaluateMppiOutputTrajectory(wrong_time, evaluator, cfg, scans, {}, nullptr));
    auto wrong_frame = output;
    wrong_frame.pose_array.header.frame_id = "odom";
    EXPECT_FALSE(evaluateMppiOutputTrajectory(wrong_frame, evaluator, cfg, scans, {}, nullptr));
    scans.pop_back();
    EXPECT_FALSE(evaluateMppiOutputTrajectory(output, evaluator, cfg, scans, {}, nullptr));
}
}  // namespace
}  // namespace dynamic_gap
