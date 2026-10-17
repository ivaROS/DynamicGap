#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <dynamic_gap/trajectory_generation/MppiControl.h>
#include <dynamic_gap/trajectory_tracking/TrajectoryController.h>
#include <dynamic_gap/utils/bezier_utils.h>

namespace dynamic_gap
{
namespace
{
geometry_msgs::Pose poseAt(double x, double y, double yaw)
{
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    return pose;
}

geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose, double yaw,
                                  double tx, double ty)
{
    return poseAt(tx + std::cos(yaw) * pose.position.x - std::sin(yaw) * pose.position.y,
                  ty + std::sin(yaw) * pose.position.x + std::cos(yaw) * pose.position.y,
                  tf::getYaw(pose.orientation) + yaw);
}

class MppiControlTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!ros::isInitialized())
        {
            ros::M_string remappings;
            ros::init(remappings, "test_mppi_control", ros::init_options::AnonymousName |
                                                        ros::init_options::NoSigintHandler);
        }
        nh_.reset(new ros::NodeHandle());
        cfg_.rbt.vx_absmax = 0.6f;
        cfg_.rbt.vang_absmax = 1.2f;
        cfg_.ctrl.ctrl_ahead_pose = 1;
        controller_.reset(new TrajectoryController(*nh_, cfg_));
    }

    geometry_msgs::PoseArray curvedReference()
    {
        std::vector<Eigen::Vector2f> curve;
        for (int i = 0; i <= 10; ++i)
            curve.push_back(bezier(Eigen::Vector2f(0, 0), Eigen::Vector2f(1, 0),
                                   Eigen::Vector2f(2, 1), i / 10.0f));
        return bezierReferencePoses(curve, "base_link");
    }

    DynamicGapConfig cfg_;
    std::unique_ptr<ros::NodeHandle> nh_;
    std::unique_ptr<TrajectoryController> controller_;
};

TEST(MppiUnicycle, UsesExistingUpdatedYawEulerAndNormalizesInput)
{
    geometry_msgs::Pose initial;  // existing planner's zero quaternion is accepted
    geometry_msgs::Twist control;
    control.linear.x = 0.7;
    control.angular.z = 0.8;
    const auto next = integrateMppiUnicycle(initial, control, 0.5);
    EXPECT_NEAR(next.position.x, 0.35 * std::cos(0.4), 1e-12);
    EXPECT_NEAR(next.position.y, 0.35 * std::sin(0.4), 1e-12);
    EXPECT_NEAR(tf::getYaw(next.orientation), 0.4, 1e-12);
    EXPECT_DOUBLE_EQ(initial.orientation.w, 0.0);

    auto scaled = poseAt(1.0, -2.0, M_PI - 0.1);
    scaled.orientation.z *= 3.0;
    scaled.orientation.w *= 3.0;
    const auto wrapped = integrateMppiUnicycle(scaled, control, 0.5);
    EXPECT_NEAR(tf::getYaw(wrapped.orientation), -M_PI + 0.3, 1e-12);
    EXPECT_NEAR(std::hypot(wrapped.orientation.z, wrapped.orientation.w), 1.0, 1e-12);
}

TEST(MppiUnicycle, RolloutHasFullTimingAndRejectsInvalidControls)
{
    MppiControls controls(4);
    for (auto& control : controls)
    {
        control.linear.x = 0.3;
        control.angular.z = -0.2;
    }
    const auto initial = poseAt(0.1, 0.2, 0.3);
    const auto rollout = rolloutMppiControls(initial, controls, 0.25, "base_link");
    ASSERT_EQ(rollout.positions.size(), 5u);
    ASSERT_EQ(rollout.pose_array.poses.size(), 5u);
    EXPECT_EQ(rollout.pose_array.header.frame_id, "base_link");
    for (std::size_t i = 0; i < rollout.times.size(); ++i)
        EXPECT_FLOAT_EQ(rollout.times[i], i * 0.25f);
    EXPECT_FLOAT_EQ(rollout.v_cmd, 0.3f);
    EXPECT_FLOAT_EQ(rollout.w_cmd, -0.2f);
    EXPECT_NEAR(rollout.yaws.back(), 0.1, 1e-6);
    EXPECT_EQ(rolloutMppiControls(initial, {}, 0.25, "base_link").positions.size(), 1u);

    controls[1].angular.z = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(rolloutMppiControls(initial, controls, 0.25, "base_link").positions.empty());
    EXPECT_THROW(integrateMppiUnicycle(initial, geometry_msgs::Twist(), 0.0),
                 std::invalid_argument);
}

TEST(MppiBezierReference, HandlesDuplicateSamplesAndInvalidGeometry)
{
    const auto path = bezierReferencePoses({{1, 2}, {1, 2}, {1, 3}, {1, 3}}, "sensor");
    ASSERT_EQ(path.poses.size(), 4u);
    EXPECT_EQ(path.header.frame_id, "sensor");
    for (const auto& pose : path.poses)
        EXPECT_NEAR(tf::getYaw(pose.orientation), M_PI / 2, 1e-6);
    EXPECT_TRUE(bezierReferencePoses({}, "sensor").poses.empty());
    EXPECT_TRUE(bezierReferencePoses({{0, 0}, {NAN, 1}}, "sensor").poses.empty());
}

TEST_F(MppiControlTest, FirstCommandMatchesProductionLookaheadAndConfiguredLimits)
{
    const auto reference = curvedReference();
    const auto initial = poseAt(0.0, 0.0, -0.7);
    const int target = controller_->extractTargetPoseIdx(initial, reference);
    auto expected = controller_->constantVelocityControlLawNonHolonomicLookahead(
        initial, reference.poses[target], cfg_.rbt.vx_absmax);
    expected.linear.x = std::clamp(expected.linear.x, -static_cast<double>(cfg_.rbt.vx_absmax),
                                  static_cast<double>(cfg_.rbt.vx_absmax));
    expected.angular.z = std::clamp(expected.angular.z, -static_cast<double>(cfg_.rbt.vang_absmax),
                                   static_cast<double>(cfg_.rbt.vang_absmax));
    const auto controls = generateNominalControlsFromBezier(reference, initial, *controller_,
                                                           cfg_, 80, 0.05);
    ASSERT_EQ(controls.size(), 80u);
    EXPECT_NEAR(controls.front().linear.x, expected.linear.x, 1e-7);
    EXPECT_NEAR(controls.front().angular.z, expected.angular.z, 1e-7);
    for (const auto& command : controls)
    {
        EXPECT_LE(std::abs(command.linear.x), cfg_.rbt.vx_absmax);
        EXPECT_LE(std::abs(command.angular.z), cfg_.rbt.vang_absmax);
        EXPECT_DOUBLE_EQ(command.linear.y, 0.0);
    }
}

TEST_F(MppiControlTest, AdvancesSimulatedStateAcrossCurvedHorizon)
{
    const auto reference = curvedReference();
    const auto initial = poseAt(0, 0, 0);
    const auto controls = generateNominalControlsFromBezier(reference, initial, *controller_,
                                                           cfg_, 80, 0.05);
    ASSERT_EQ(controls.size(), 80u);
    bool changed = false;
    for (const auto& control : controls)
        changed = changed || std::abs(control.angular.z - controls.front().angular.z) > 1e-3;
    EXPECT_TRUE(changed);  // regression against repeating a one-shot lookahead command
    const auto rollout = rolloutMppiControls(initial, controls, 0.05, "base_link");
    ASSERT_EQ(rollout.positions.size(), 81u);
    EXPECT_LT((rollout.positions.back() - Eigen::Vector2f(2, 1)).norm(), 0.35f);
    for (const auto& position : rollout.positions)
    {
        float nearest = std::numeric_limits<float>::infinity();
        for (const auto& referencePose : reference.poses)
            nearest = std::min(nearest, (position - Eigen::Vector2f(
                referencePose.position.x, referencePose.position.y)).norm());
        EXPECT_LT(nearest, 0.3f);
    }
    EXPECT_DOUBLE_EQ(initial.position.x, 0.0);
    EXPECT_DOUBLE_EQ(initial.orientation.w, 1.0);
}

TEST_F(MppiControlTest, CommandsAreInvariantUnderCommonPlanarFrameTransform)
{
    const auto reference = curvedReference();
    const auto initial = poseAt(0, 0, 0.1);
    auto transformedReference = reference;
    for (auto& pose : transformedReference.poses)
        pose = transformPose(pose, 0.8, 3.0, -2.0);
    const auto transformedInitial = transformPose(initial, 0.8, 3.0, -2.0);
    const auto original = generateNominalControlsFromBezier(reference, initial, *controller_,
                                                           cfg_, 30, 0.05);
    const auto transformed = generateNominalControlsFromBezier(transformedReference,
        transformedInitial, *controller_, cfg_, 30, 0.05);
    ASSERT_EQ(original.size(), 30u);
    ASSERT_EQ(transformed.size(), original.size());
    for (std::size_t i = 0; i < original.size(); ++i)
    {
        EXPECT_NEAR(original[i].linear.x, transformed[i].linear.x, 1e-5);
        EXPECT_NEAR(original[i].angular.z, transformed[i].angular.z, 1e-5);
    }
}

TEST_F(MppiControlTest, StopsAtEndpointAndRejectsEmptyOrInvalidInputs)
{
    const auto reference = curvedReference();
    const auto stopped = generateNominalControlsFromBezier(reference, reference.poses.back(),
                                                           *controller_, cfg_, 12, 0.1);
    ASSERT_EQ(stopped.size(), 12u);
    for (const auto& command : stopped)
    {
        EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
        EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
    }
    const auto initial = poseAt(0, 0, 0);
    EXPECT_TRUE(generateNominalControlsFromBezier({}, initial, *controller_, cfg_, 12, 0.1).empty());
    EXPECT_TRUE(generateNominalControlsFromBezier(reference, initial, *controller_, cfg_, 0, 0.1).empty());
    EXPECT_TRUE(generateNominalControlsFromBezier(reference, initial, *controller_, cfg_, 12, -0.1).empty());
    auto invalid = initial;
    invalid.position.x = std::numeric_limits<double>::infinity();
    EXPECT_TRUE(generateNominalControlsFromBezier(reference, invalid, *controller_, cfg_, 12, 0.1).empty());
}
}  // namespace
}  // namespace dynamic_gap
