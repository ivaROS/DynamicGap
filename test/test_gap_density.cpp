#include <gtest/gtest.h>
#include <dynamic_gap/gap_density/GapDensity.h>
#include <dynamic_gap/trajectory_generation/MppiOptimizer.h>
#include "mppi_test_scene.h"

namespace dynamic_gap
{
TEST(DensitySector, WrapIncludesBothBoundariesAndStaticRadius)
{
    std::vector<DensityEndpoint> points = {
        {510, {2,0}, .05f, true, false},
        {0, {4,0}, 0, false, true},
        {20, {1,0}, .1f, true, true},
        {21, {9,0}, 1, true, false},
        {-1, {9,0}, 1, true, false}};
    const auto sector = computeDensitySector(points, 510, 20, 512, .01f);
    EXPECT_EQ(sector.count, 3);
    EXPECT_EQ(sector.dynamic_count, 2);
    EXPECT_EQ(sector.ground_truth_count, 2);
    EXPECT_FLOAT_EQ(sector.radius, 4);
    EXPECT_NEAR(sector.angle, .22, 1e-6);
    EXPECT_NEAR(sector.area, 1.76, 1e-6);
    EXPECT_NEAR(sector.density, 2.0/1.76, 1e-6);
    EXPECT_TRUE(densityIndexInside(10, 5, 10, 512));
    EXPECT_FALSE(densityIndexInside(4, 5, 10, 512));
    EXPECT_FALSE(densityIndexInside(0, -1, 20, 512));
    EXPECT_FLOAT_EQ(computeDensitySector(points, 10, 10, 512, .01f).density, 0);
    EXPECT_FLOAT_EQ(computeDensitySector(points, 0, 20, 512, 0).density, 0);
    EXPECT_FLOAT_EQ(computeDensitySector({}, 0, 20, 512, .01f).density, 0);
}

class GapDensityTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!ros::isInitialized())
        {
            ros::M_string remappings;
            ros::init(remappings, "test_gap_density", ros::init_options::AnonymousName);
        }
        nh = std::make_unique<ros::NodeHandle>("density_test");
        scene = std::make_unique<dynamic_gap_test::Scene>(*nh);
        nh->setParam("gap_density/source", "gru");
        density = std::make_shared<GapDensity>();
        density->initialize(*nh, *nh, scene->cfg.robot_frame_id);
        density->update(scene->gaps, scene->gaps, *scene->scan, {}, {}, false);
        id = scene->gaps.front()->getLeftGapPt()->getModel()->getID();
        message.header = scene->scan->header;
        message.model_id = id;
        message.side = "left";
        message.valid = true;
        message.seq_len_used = 10;
        message.output_names = {"gt_sector_density"};
        message.output_values = {2};
    }
    std::unique_ptr<ros::NodeHandle> nh;
    std::unique_ptr<dynamic_gap_test::Scene> scene;
    std::shared_ptr<GapDensity> density;
    GapFeaturePrediction message;
    int id;
};

TEST_F(GapDensityTest, PredictionFallbackAndIdentityLifecycle)
{
    const auto now = message.header.stamp;
    EXPECT_FLOAT_EQ(density->cost(id, now), 0);
    density->prediction(message);
    EXPECT_FLOAT_EQ(density->cost(id, now), 10);
    EXPECT_FLOAT_EQ(density->cost(-1, now), 0);
    EXPECT_FLOAT_EQ(density->cost(id+10000, now), 0);
    EXPECT_FLOAT_EQ(density->cost(id, now + ros::Duration(3.01)), 0);
    EXPECT_FLOAT_EQ(density->cost(id, now - ros::Duration(3.01)), 0);
    auto invalid = message;
    invalid.valid = false;
    density->prediction(invalid);
    EXPECT_FLOAT_EQ(density->cost(id, now), 0);
    density->prediction(message);
    invalid = message; invalid.output_names = {"future_density"};
    density->prediction(invalid);
    EXPECT_FLOAT_EQ(density->cost(id, now), 0);
    density->prediction(message);
    invalid = message; invalid.output_values.clear();
    density->prediction(invalid);
    EXPECT_FLOAT_EQ(density->cost(id, now), 0);
    invalid = message; invalid.output_values = {std::numeric_limits<float>::quiet_NaN()};
    density->prediction(invalid);
    EXPECT_FLOAT_EQ(density->cost(id, now), 0);
    invalid = message; invalid.output_values = {-2};
    density->prediction(invalid);
    EXPECT_FLOAT_EQ(density->cost(id, now), 0);
    invalid = message; invalid.side = "right";
    density->prediction(invalid);
    EXPECT_FLOAT_EQ(density->cost(id, now), 0);
    density->prediction(message);
    density->update({}, {}, *scene->scan, {}, {}, false);
    EXPECT_FLOAT_EQ(density->cost(id, now), 0);
    // A disappeared/reused identity must warm up again, even with the same numeric ID.
    density->update(scene->gaps, scene->gaps, *scene->scan, {}, {}, false);
    EXPECT_FLOAT_EQ(density->cost(id, now), 0);
    density->prediction(message);
    auto rewind = *scene->scan; rewind.header.stamp -= ros::Duration(1);
    density->update(scene->gaps, scene->gaps, rewind, {}, {}, false);
    EXPECT_FLOAT_EQ(density->cost(id, rewind.header.stamp), 0);
}

TEST_F(GapDensityTest, PublishesNamedFeaturesUnknownTargetsAndRobotFrameMarkers)
{
    GapFeatureObservation::ConstPtr observation;
    visualization_msgs::MarkerArray::ConstPtr markers;
    auto observationSub = nh->subscribe<GapFeatureObservation>("gap_feature_observation", 100,
        [&](const GapFeatureObservation::ConstPtr& msg) { if (msg->model_id == id) observation = msg; });
    auto markerSub = nh->subscribe<visualization_msgs::MarkerArray>("contained_raw_gap_points", 10,
        [&](const visualization_msgs::MarkerArray::ConstPtr& msg) { markers = msg; });
    const auto wait = [&](const std::function<bool()>& ready) {
        const auto deadline = ros::WallTime::now() + ros::WallDuration(2);
        while (!ready() && ros::WallTime::now() < deadline)
        { ros::spinOnce(); ros::WallDuration(.01).sleep(); }
        return ready();
    };
    ASSERT_TRUE(wait([&]() { return observationSub.getNumPublishers() && markerSub.getNumPublishers(); }));
    density->update(scene->gaps, scene->gaps, *scene->scan, {}, {}, false);
    ASSERT_TRUE(wait([&]() { return bool(observation) && bool(markers); }));
    EXPECT_EQ(observation->feature_names, (std::vector<std::string>{"sector_density",
        "sector_dynamic_raw_gap_point_count", "contained_raw_gap_point_count",
        "sector_area", "sector_angle_rad", "sector_radius"}));
    EXPECT_EQ(observation->feature_values.size(), 6u);
    EXPECT_TRUE(observation->target_names.empty());
    EXPECT_TRUE(observation->target_values.empty());
    EXPECT_EQ(observation->side, "left");
    EXPECT_EQ(observation->header.frame_id, scene->cfg.robot_frame_id);
    ASSERT_FALSE(markers->markers.empty());
    EXPECT_EQ(markers->markers.front().action, visualization_msgs::Marker::DELETEALL);
    ASSERT_EQ(markers->markers.size(), 1 + 2*scene->gaps.size());
    for (std::size_t i = 1; i < markers->markers.size(); ++i)
        EXPECT_EQ(markers->markers[i].header.frame_id, scene->cfg.robot_frame_id);
    float x, y;
    scene->gaps.front()->getLCartesian(x,y);
    geometry_msgs::Pose pose; pose.position.x = x; pose.position.y = y;
    geometry_msgs::Vector3Stamped velocity; velocity.vector.x = .2;
    observation.reset(); markers.reset();
    density->update(scene->gaps, scene->gaps, *scene->scan, {{"pedestrian", pose}}, {{"pedestrian", velocity}}, true);
    ASSERT_TRUE(wait([&]() { return bool(observation); }));
    ASSERT_EQ(observation->target_names.size(), 2u);
    EXPECT_EQ(observation->target_names.front(), "gt_sector_density");
    EXPECT_GT(observation->target_values.front(), 0);
}

TEST_F(GapDensityTest, GroundTruthBypassesGruAndAddsOneNativeTerminalPenalty)
{
    nh->setParam("gap_density/source", "ground_truth");
    density = std::make_shared<GapDensity>();
    density->initialize(*nh, *nh, scene->cfg.robot_frame_id);
    TrajectoryEvaluator evaluator(scene->cfg);
    scene->initializeEvaluator(evaluator);
    evaluator.setGapDensity(density);
    geometry_msgs::Pose origin; origin.orientation.w = 1;
    MppiControls controls(10);
    for (auto& control : controls) control.linear.x = .2;
    auto trajectory = rolloutMppiControls(origin, controls, scene->cfg.traj.integrate_stept, scene->cfg.sensor_frame_id);
    auto* gap = scene->gaps.front();
    ASSERT_TRUE(evaluateMppiOutputTrajectory(trajectory, evaluator, scene->cfg, scene->future_scans, scene->plan, gap));
    const float baseline = trajectory.totalTrajCost;
    const float baselineTerminal = trajectory.TerminalPoseCost;
    float x, y;
    gap->getLCartesian(x,y);
    geometry_msgs::Pose pose; pose.position.x = x; pose.position.y = y;
    geometry_msgs::Vector3Stamped velocity; velocity.vector.x = .2;
    const std::map<std::string, geometry_msgs::Pose> poses{{"pedestrian", pose}};
    const std::map<std::string, geometry_msgs::Vector3Stamped> velocities{{"pedestrian", velocity}};
    density->update(scene->gaps, scene->gaps, *scene->scan, poses, velocities, true);
    const auto now = scene->scan->header.stamp;
    const float penalty = density->cost(id, now);
    ASSERT_GT(penalty, 0);
    // The same endpoint geometry/GT classifier supplies the training target and test cost.
    std::vector<DensityEndpoint> endpoints;
    for (auto* raw : scene->gaps)
        for (bool left : {false, true})
        {
            float px, py;
            if (left) raw->getLCartesian(px,py); else raw->getRCartesian(px,py);
            endpoints.push_back({left ? raw->LIdx() : raw->RIdx(), {px,py}, 0,
                std::hypot(px-x,py-y) < .6, left});
        }
    const auto expected = computeDensitySector(endpoints, gap->RIdx(), gap->LIdx(), scene->scan->ranges.size(), scene->scan->angle_increment);
    EXPECT_NEAR(penalty, 5 * expected.ground_truth_density, 1e-6);
    message.output_values = {10000};
    density->prediction(message);  // A running GRU node cannot override GT.
    EXPECT_FLOAT_EQ(density->cost(id, now), penalty);
    ASSERT_TRUE(evaluateMppiOutputTrajectory(trajectory, evaluator, scene->cfg, scene->future_scans, scene->plan, gap));
    EXPECT_NEAR(trajectory.totalTrajCost - baseline, penalty, 1e-4);
    EXPECT_NEAR(trajectory.TerminalPoseCost - baselineTerminal, penalty, 1e-4);
    ASSERT_TRUE(evaluateMppiOutputTrajectory(trajectory, evaluator, scene->cfg, scene->future_scans, scene->plan, gap));
    EXPECT_NEAR(trajectory.totalTrajCost - baseline, penalty, 1e-4);
    EXPECT_FLOAT_EQ(density->cost(id, now + ros::Duration(.51)), 0);
    EXPECT_FLOAT_EQ(density->cost(-1, now), 0);
    // Missing simulator data immediately removes old density, rather than retaining it.
    density->update(scene->gaps, scene->gaps, *scene->scan, poses, velocities, false);
    EXPECT_FLOAT_EQ(density->cost(id, now), 0);
    // A received empty simulator state is known empty, with zero density.
    density->update(scene->gaps, scene->gaps, *scene->scan, {}, {}, true);
    EXPECT_FLOAT_EQ(density->cost(id, now), 0);
}

TEST_F(GapDensityTest, NativeMppiOutputChargesOnceAndRunningPathUsesFreshPrediction)
{
    TrajectoryEvaluator evaluator(scene->cfg);
    scene->initializeEvaluator(evaluator);
    evaluator.setGapDensity(density);
    geometry_msgs::Pose origin; origin.orientation.w = 1;
    MppiControls controls(10);
    for (auto& control : controls) control.linear.x = .2;
    auto baseline = rolloutMppiControls(origin, controls, scene->cfg.traj.integrate_stept, scene->cfg.sensor_frame_id);
    auto* gap = scene->gaps.front();
    ASSERT_TRUE(evaluateMppiOutputTrajectory(baseline, evaluator, scene->cfg, scene->future_scans, scene->plan, gap));
    density->prediction(message);
    auto candidate = rolloutMppiControls(origin, controls, scene->cfg.traj.integrate_stept, scene->cfg.sensor_frame_id);
    ASSERT_TRUE(evaluateMppiOutputTrajectory(candidate, evaluator, scene->cfg, scene->future_scans, scene->plan, gap));
    EXPECT_NEAR(candidate.TerminalPoseCost - baseline.TerminalPoseCost, 10, 1e-4);
    EXPECT_NEAR(candidate.totalTrajCost - baseline.totalTrajCost, 10, 1e-4);
    // A second evaluation starts from native costs, so it must not accumulate penalties.
    ASSERT_TRUE(evaluateMppiOutputTrajectory(candidate, evaluator, scene->cfg, scene->future_scans, scene->plan, gap));
    EXPECT_NEAR(candidate.totalTrajCost - baseline.totalTrajCost, 10, 1e-4);
    Trajectory running(candidate.pose_array, candidate.times);
    running.leftGapPtID = id;
    std::vector<float> costs;
    float terminal1, terminal2;
    evaluator.dwa_evaluateTrajectory_reducedCurrentTraj(running, costs, terminal1, scene->future_scans, 0, -1, id, Eigen::Vector2f::Zero());
    message.output_values = {3}; density->prediction(message);
    evaluator.dwa_evaluateTrajectory_reducedCurrentTraj(running, costs, terminal2, scene->future_scans, 0, -1, id, Eigen::Vector2f::Zero());
    EXPECT_NEAR(terminal2 - terminal1, 5, 1e-4);
    // Generic paths with no gap ID (idling, ungap, goal approach) remain exempt.
    running.leftGapPtID = -1;
    evaluator.evaluateTrajectory(running, costs, terminal1, scene->future_scans, 0);
    running.leftGapPtID = id;
    evaluator.evaluateTrajectory(running, costs, terminal2, scene->future_scans, 0);
    EXPECT_NEAR(terminal2 - terminal1, 15, 1e-4);
}
}
