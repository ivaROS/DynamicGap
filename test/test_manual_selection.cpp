#include <gtest/gtest.h>

#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int16.h>

#include "planner_test_access.h"

using dynamic_gap::Planner;
using dynamic_gap::PlannerTestAccess;
using dynamic_gap::TimeKeeper;
using dynamic_gap::Trajectory;

// Fix A: one candidate list; display ids are dense over shown candidates
TEST(ManualSelectionHelpers, DenseDisplayIds)
{
    EXPECT_EQ((std::vector<int>{0, -1, 1, 2, -1}), dynamic_gap::denseDisplayIds({true, false, true, true, false}));
    EXPECT_TRUE(dynamic_gap::denseDisplayIds({}).empty());
}

// scene edges: two per raw gap in scan order; each has a gap neighbour and an
// obstacle neighbour, around the full circle
TEST(ManualSelectionHelpers, LinkEdges)
{
    std::vector<manual_gap_msgs::ManualGapEdge> edges(6);
    dynamic_gap::linkEdges(edges);
    std::vector<int> gap, obstacle;
    for (const auto & e : edges)
    {
        gap.push_back(e.gap_neighbor);
        obstacle.push_back(e.obstacle_neighbor);
    }
    EXPECT_EQ((std::vector<int>{1, 0, 3, 2, 5, 4}), gap);
    EXPECT_EQ((std::vector<int>{5, 2, 1, 4, 3, 0}), obstacle);
}

TEST(ManualSelectionHelpers, FillSampleEdges)
{
    Planner planner;
    // right point at the lower scan index, left at the higher
    std::vector<dynamic_gap::Gap *> gaps = {new dynamic_gap::Gap("base_link", 40, 2.0f, 10, 3.0f, false),
                                            new dynamic_gap::Gap("base_link", 300, 4.0f, 200, 5.0f, false)};
    manual_gap_msgs::ManualGapSample sample;
    PlannerTestAccess::fillSampleEdges(planner, gaps, sample);
    ASSERT_EQ(4u, sample.edges.size());
    EXPECT_EQ((std::vector<int>{10, 40, 200, 300}),
              (std::vector<int>{sample.edges[0].scan_idx, sample.edges[1].scan_idx,
                                sample.edges[2].scan_idx, sample.edges[3].scan_idx}));
    EXPECT_FLOAT_EQ(3.0f, sample.edges[0].range);
    EXPECT_FLOAT_EQ(dynamic_gap::idx2theta(10), sample.edges[0].theta);
    EXPECT_EQ(1, sample.edges[0].gap_neighbor);
    EXPECT_EQ(2, sample.edges[1].obstacle_neighbor);
    // the estimator's id and state (velocity relative to the robot)
    dynamic_gap::Estimator * model = gaps[0]->getRightGapPt()->getModel();
    EXPECT_EQ(model->getID(), sample.edges[0].model_id);
    EXPECT_FLOAT_EQ(model->getState()[2], sample.edges[0].model_vel.x);
    for (auto * gap : gaps)
        delete gap;
}

TEST(ManualSelectionHelpers, HasModelIDs)
{
    EXPECT_TRUE(dynamic_gap::hasModelIDs(3, 4));
    EXPECT_TRUE(dynamic_gap::hasModelIDs(0, 0));
    EXPECT_FALSE(dynamic_gap::hasModelIDs(-1, 4));
    EXPECT_FALSE(dynamic_gap::hasModelIDs(-1, -1));
}

TEST(ManualSelectionHelpers, NearestFeasibleCandidate)
{
    std::vector<std::pair<float, float>> terminals = {{0.0f, 0.0f}, {1.0f, 0.0f}, {5.0f, 0.0f}};
    // the infeasible candidate at (1, 0) is skipped
    EXPECT_EQ(0, dynamic_gap::nearestCandidate(terminals, {true, false, true}, 1.1f, 0.0f));
    EXPECT_EQ(1, dynamic_gap::nearestCandidate(terminals, {true, true, true}, 1.1f, 0.0f));
    EXPECT_EQ(2, dynamic_gap::nearestCandidate(terminals, {true, true, true}, 4.0f, 0.0f));
    EXPECT_EQ(-1, dynamic_gap::nearestCandidate(terminals, {false, false, false}, 1.0f, 0.0f));
    // ties go to the lower index
    EXPECT_EQ(0, dynamic_gap::nearestCandidate({{1.0f, 0.0f}, {-1.0f, 0.0f}}, {true, true}, 0.0f, 0.0f));
}

// IL fix 43: only gaps in the cursor's direction from the robot are selectable
TEST(ManualSelectionHelpers, InCursorDirection)
{
    // robot at (1, 1); terminals east, north, west, and east but infeasible
    std::vector<std::pair<float, float>> terminals = {{4.0f, 1.0f}, {1.0f, 4.0f}, {-2.0f, 1.0f}, {4.0f, 1.5f}};
    std::vector<bool> feasible = {true, true, true, false};
    const float sixty = M_PI / 3.0;

    // cursor east: only the feasible east terminal
    EXPECT_EQ((std::vector<bool>{true, false, false, false}),
              dynamic_gap::inCursorDirection(terminals, feasible, 1.0f, 1.0f, 3.0f, 1.0f, sixty));
    // cursor north-east (45 deg): east and north are both within 60 deg
    EXPECT_EQ((std::vector<bool>{true, true, false, false}),
              dynamic_gap::inCursorDirection(terminals, feasible, 1.0f, 1.0f, 2.0f, 2.0f, sixty));
    // cursor south-west: nothing
    EXPECT_EQ((std::vector<bool>{false, false, false, false}),
              dynamic_gap::inCursorDirection(terminals, feasible, 1.0f, 1.0f, 0.0f, -1.0f, sixty));
    // west across the +-pi wrap
    EXPECT_EQ((std::vector<bool>{false, false, true, false}),
              dynamic_gap::inCursorDirection(terminals, feasible, 1.0f, 1.0f, -1.0f, 0.9f, sixty));
}

// IL fix 43: the followed gap is kept unless another is clearly nearer the cursor
TEST(ManualSelectionHelpers, HoldCurrentCandidate)
{
    // cursor at the origin; terminals 1.0, 0.8 and 0.5 away
    std::vector<std::pair<float, float>> terminals = {{1.0f, 0.0f}, {0.0f, 0.8f}, {-0.5f, 0.0f}};
    // 0.8 is not below 0.7 * 1.0: keep the current one
    EXPECT_EQ(0, dynamic_gap::holdCurrentCandidate(terminals, 0.0f, 0.0f, 1, 0, 0.7f));
    // 0.5 is below 0.7 * 1.0: switch
    EXPECT_EQ(2, dynamic_gap::holdCurrentCandidate(terminals, 0.0f, 0.0f, 2, 0, 0.7f));
    // ratio 1 is plain nearest
    EXPECT_EQ(1, dynamic_gap::holdCurrentCandidate(terminals, 0.0f, 0.0f, 1, 0, 1.0f));
    // no current gap: nearest; nothing selectable: none
    EXPECT_EQ(1, dynamic_gap::holdCurrentCandidate(terminals, 0.0f, 0.0f, 1, -1, 0.7f));
    EXPECT_EQ(-1, dynamic_gap::holdCurrentCandidate(terminals, 0.0f, 0.0f, -1, -1, 0.7f));
}

TEST(ManualSelectionHelpers, CursorHeldWithoutNewMessages)
{
    // the last known position stays valid; only switching the tool off ends it
    EXPECT_TRUE(dynamic_gap::isCursorHeld(true));
    EXPECT_FALSE(dynamic_gap::isCursorHeld(false));
}

TEST(ManualSelectionHelpers, ResetWindow)
{
    EXPECT_TRUE(dynamic_gap::isInResetWindow(3, 2, 10.0, 1.0));   // reset 3 in progress
    EXPECT_TRUE(dynamic_gap::isInResetWindow(3, 3, 0.5, 1.0));    // just after reset 3
    EXPECT_FALSE(dynamic_gap::isInResetWindow(3, 3, 1.5, 1.0));
    EXPECT_FALSE(dynamic_gap::isInResetWindow(-1, -1, -1.0, 1.0)); // no reset seen
}

// ticks that end before selection record no selection (not the message default 0)
TEST(ManualSelectionHelpers, NewSampleHasNoSelection)
{
    manual_gap_msgs::ManualGapSample sample = dynamic_gap::newManualSample();
    EXPECT_EQ(sample.STATUS_PLANNED, sample.status);
    EXPECT_EQ(sample.SOURCE_NONE, sample.source);
    EXPECT_EQ(-1, sample.selected_idx);
    EXPECT_EQ(-1, sample.selected_left_model_id);
    EXPECT_EQ(-1, sample.selected_right_model_id);
    EXPECT_EQ(-1, sample.dg_pick_idx);
    EXPECT_TRUE(sample.candidates.empty());
}

// DynamicGap's cost, as pickTraj ranks it and manual mode records it
TEST(ManualSelectionHelpers, DgTrajCost)
{
    EXPECT_FLOAT_EQ(2.0f, dynamic_gap::meanPoseCost({1.0f, 3.0f}));
    EXPECT_TRUE(std::isnan(dynamic_gap::meanPoseCost({})));
    EXPECT_FLOAT_EQ(7.0f, dynamic_gap::dgTrajCost({1.0f, 3.0f}, 5.0f, 2));
    // fewer than 2 poses: never picked
    EXPECT_TRUE(std::isinf(dynamic_gap::dgTrajCost({1.0f}, 5.0f, 1)));
    EXPECT_TRUE(std::isinf(dynamic_gap::dgTrajCost({}, 0.0f, 0)));
}

TEST(ManualSelectionHelpers, DownsamplePathKeepsLastPose)
{
    geometry_msgs::PoseArray path;
    for (int i = 0; i < 12; i++)
    {
        geometry_msgs::Pose pose;
        pose.position.x = i;
        path.poses.push_back(pose);
    }
    std::vector<geometry_msgs::Point> points = dynamic_gap::downsamplePath(path, 5);
    ASSERT_EQ(4u, points.size());
    EXPECT_EQ(0.0, points[0].x);
    EXPECT_EQ(5.0, points[1].x);
    EXPECT_EQ(10.0, points[2].x);
    EXPECT_EQ(11.0, points[3].x);
    EXPECT_TRUE(dynamic_gap::downsamplePath(geometry_msgs::PoseArray(), 5).empty());
}

TEST(ManualSelectionHelpers, ScopeExitRunsOnEveryReturn)
{
    int calls = 0;
    auto run = [&](bool early)
    {
        dynamic_gap::ScopeExit<std::function<void()>> guard([&]() { calls++; });
        if (early)
            return;
    };
    run(true);
    run(false);
    EXPECT_EQ(2, calls);
}

namespace
{
    class ManualPlannerTest : public ::testing::Test
    {
        protected:
            void SetUp() override
            {
                PlannerTestAccess::setTimeKeeper(planner_, new TimeKeeper());
                PlannerTestAccess::setManualMode(planner_, true);
                PlannerTestAccess::setOdomFrame(planner_, "odom");
                PlannerTestAccess::setHaveTFs(planner_, true);

                // map -> odom: odom origin at (1, 2) in the map
                geometry_msgs::TransformStamped mapToOdom;
                mapToOdom.header.frame_id = "odom";
                mapToOdom.child_frame_id = "map";
                mapToOdom.transform.translation.x = -1.0;
                mapToOdom.transform.translation.y = -2.0;
                mapToOdom.transform.rotation.w = 1.0;
                PlannerTestAccess::tfBuffer(planner_).setTransform(mapToOdom, "test", true);
            }

            manual_gap_msgs::CursorState::Ptr cursor(bool active, bool pickOk, double x, double y)
            {
                manual_gap_msgs::CursorState::Ptr msg(new manual_gap_msgs::CursorState());
                msg->header.frame_id = "map";
                msg->header.stamp = ros::Time::now();
                msg->tool_active = active;
                msg->pick_ok = pickOk;
                msg->point.x = x;
                msg->point.y = y;
                return msg;
            }

            Planner planner_;
    };
}

// Cursor: own frame transformed into odom; a failed pick keeps the point;
// an inactive tool removes it
TEST_F(ManualPlannerTest, CursorTransformedFromItsFrame)
{
    PlannerTestAccess::cursorCB(planner_, cursor(true, true, 3.0, 4.0));
    ASSERT_TRUE(PlannerTestAccess::hasCursorPos(planner_));
    EXPECT_DOUBLE_EQ(2.0, PlannerTestAccess::cursorOdom(planner_).x);
    EXPECT_DOUBLE_EQ(2.0, PlannerTestAccess::cursorOdom(planner_).y);
    EXPECT_TRUE(PlannerTestAccess::cursorUsable(planner_));
}

TEST_F(ManualPlannerTest, FailedPickKeepsLastPoint)
{
    PlannerTestAccess::cursorCB(planner_, cursor(true, true, 3.0, 4.0));
    PlannerTestAccess::cursorCB(planner_, cursor(true, false, 9.0, 9.0));
    EXPECT_TRUE(PlannerTestAccess::hasCursorPos(planner_));
    EXPECT_DOUBLE_EQ(2.0, PlannerTestAccess::cursorOdom(planner_).x);
    EXPECT_TRUE(PlannerTestAccess::cursorUsable(planner_));
}

TEST_F(ManualPlannerTest, InactiveToolRemovesCursor)
{
    PlannerTestAccess::cursorCB(planner_, cursor(true, true, 3.0, 4.0));
    PlannerTestAccess::cursorCB(planner_, cursor(false, false, 0.0, 0.0));
    EXPECT_FALSE(PlannerTestAccess::hasCursorPos(planner_));
    EXPECT_FALSE(PlannerTestAccess::cursorUsable(planner_));
}

TEST_F(ManualPlannerTest, UnknownFrameIsIgnored)
{
    manual_gap_msgs::CursorState::Ptr msg = cursor(true, true, 3.0, 4.0);
    msg->header.frame_id = "nowhere";
    PlannerTestAccess::cursorCB(planner_, msg);
    EXPECT_FALSE(PlannerTestAccess::hasCursorPos(planner_));
}

// PL-16: no gaps in manual mode -> hold still; the controller returns zero
TEST_F(ManualPlannerTest, NoGapsHoldsStill)
{
    PlannerTestAccess::setTrajVisualizer(planner_);
    PlannerTestAccess::setInitialized(planner_, true);
    PlannerTestAccess::setHasLaserScan(planner_, true);
    PlannerTestAccess::setHasGlobalGoal(planner_, true);

    Trajectory chosenTraj;
    int trajFlag = dynamic_gap::GAP;
    planner_.runPlanningLoop(chosenTraj, trajFlag);

    EXPECT_EQ(dynamic_gap::NONE, trajFlag);
    EXPECT_TRUE(PlannerTestAccess::holdStill(planner_));
}

TEST_F(ManualPlannerTest, HoldStillGivesZeroVelocity)
{
    PlannerTestAccess::setHoldStill(planner_, true);
    int trajFlag = dynamic_gap::NONE;
    geometry_msgs::Twist cmd = planner_.ctrlGeneration(geometry_msgs::PoseArray(), trajFlag);
    EXPECT_EQ(0.0, cmd.linear.x);
    EXPECT_EQ(0.0, cmd.linear.y);
    EXPECT_EQ(0.0, cmd.angular.z);
}

// Fix 37: in collision the manual planner holds still instead of planning nothing
TEST_F(ManualPlannerTest, CollidingHoldsStill)
{
    PlannerTestAccess::setInitialized(planner_, true);
    PlannerTestAccess::setHasLaserScan(planner_, true);
    PlannerTestAccess::setHasGlobalGoal(planner_, true);
    PlannerTestAccess::setColliding(planner_, true);

    Trajectory chosenTraj;
    int trajFlag = dynamic_gap::GAP;
    planner_.runPlanningLoop(chosenTraj, trajFlag);

    EXPECT_EQ(dynamic_gap::NONE, trajFlag);
    EXPECT_TRUE(PlannerTestAccess::holdStill(planner_));
}

TEST_F(ManualPlannerTest, NotReadyHoldsStill)
{
    Trajectory chosenTraj;
    int trajFlag = dynamic_gap::GAP;
    planner_.runPlanningLoop(chosenTraj, trajFlag);

    EXPECT_EQ(dynamic_gap::NONE, trajFlag);
    EXPECT_TRUE(PlannerTestAccess::holdStill(planner_));
}

// Fix 37: without a selection the controller never falls back to obstacle
// avoidance or idling (no trajectory controller is set here, so running one
// would crash)
TEST_F(ManualPlannerTest, NoSelectionNeverRunsPlannerControl)
{
    PlannerTestAccess::setHoldStill(planner_, false);

    geometry_msgs::PoseArray path;
    path.poses.resize(3);

    for (int flag : {dynamic_gap::NONE, dynamic_gap::IDLING, dynamic_gap::UNGAP})
    {
        int trajFlag = flag;
        geometry_msgs::Twist cmd = planner_.ctrlGeneration(path, trajFlag);
        EXPECT_EQ(0.0, cmd.linear.x) << "trajFlag " << flag;
        EXPECT_EQ(0.0, cmd.linear.y) << "trajFlag " << flag;
        EXPECT_EQ(0.0, cmd.angular.z) << "trajFlag " << flag;
    }

    // a selected gap whose trajectory is empty
    int trajFlag = dynamic_gap::GAP;
    geometry_msgs::Twist cmd = planner_.ctrlGeneration(geometry_msgs::PoseArray(), trajFlag);
    EXPECT_EQ(0.0, cmd.linear.x);
    EXPECT_EQ(0.0, cmd.angular.z);
}

// Reset: gaps of the old scene and click-to-move state are dropped (manual mode)
TEST_F(ManualPlannerTest, ResetClearsManualState)
{
    PlannerTestAccess::addGaps(planner_, 2);
    PlannerTestAccess::setHasTrajCreationPoint(planner_, true);
    PlannerTestAccess::setTrackingCreatedTraj(planner_, true);

    std_msgs::Int16::Ptr reset(new std_msgs::Int16());
    reset->data = 4;
    PlannerTestAccess::scenarioResetCB(planner_, reset);

    EXPECT_EQ(0u, PlannerTestAccess::gapCount(planner_));
    EXPECT_FALSE(PlannerTestAccess::hasTrajCreationPoint(planner_));
    EXPECT_FALSE(PlannerTestAccess::trackingCreatedTraj(planner_));
    EXPECT_EQ(4, PlannerTestAccess::episodeId(planner_));
}

TEST_F(ManualPlannerTest, ResetKeepsGapsInAutonomousMode)
{
    PlannerTestAccess::setManualMode(planner_, false);
    PlannerTestAccess::addGaps(planner_, 2);

    std_msgs::Int16::Ptr reset(new std_msgs::Int16());
    reset->data = 1;
    PlannerTestAccess::scenarioResetCB(planner_, reset);

    EXPECT_EQ(8u, PlannerTestAccess::gapCount(planner_));
}

// Fix A/B: one candidate list per tick, selected from and recorded
namespace
{
    Trajectory trajTo(double x, double y, int poses = 6)
    {
        geometry_msgs::PoseArray path;
        path.header.frame_id = "odom";
        std::vector<float> timing;
        for (int i = 0; i < poses; i++)
        {
            geometry_msgs::Pose pose;
            pose.position.x = x * (i + 1) / poses;
            pose.position.y = y * (i + 1) / poses;
            pose.orientation.w = 1.0;
            path.poses.push_back(pose);
            timing.push_back(0.5f * i);
        }
        Trajectory traj(path, timing);
        traj.setPathOdomFrame(path);
        return traj;
    }

    struct Tick
    {
        std::vector<dynamic_gap::GapTube *> tubes;
        std::vector<Trajectory> trajs;
        ~Tick() { for (auto * tube : tubes) delete tube; }
    };

    void addCandidate(Tick & tick, const Trajectory & traj)
    {
        dynamic_gap::Gap gap("base_link", 10, 2.0f, 20, 2.0f, false);
        tick.tubes.push_back(new dynamic_gap::GapTube(&gap));
        tick.trajs.push_back(traj);
    }
}

TEST_F(ManualPlannerTest, SelectsNearestFeasibleAndRecordsTheList)
{
    PlannerTestAccess::setTrajVisualizer(planner_);
    PlannerTestAccess::cursorCB(planner_, cursor(true, true, 4.0, 1.0));  // odom (3, -1)

    Tick tick;
    addCandidate(tick, trajTo(3.0, 1.0));
    addCandidate(tick, Trajectory());  // infeasible
    addCandidate(tick, trajTo(3.0, -1.0));

    manual_gap_msgs::ManualGapSample sample;
    Trajectory chosen;
    int trajFlag = dynamic_gap::NONE;
    int selected = PlannerTestAccess::selectManualTrajectory(planner_, trajFlag, tick.tubes, tick.trajs, sample, chosen);

    EXPECT_EQ(2, selected);
    EXPECT_EQ(dynamic_gap::GAP, trajFlag);
    ASSERT_EQ(3u, sample.candidates.size());
    EXPECT_EQ(0, sample.candidates[0].display_id);
    EXPECT_EQ(-1, sample.candidates[1].display_id);
    EXPECT_FALSE(sample.candidates[1].feasible);
    EXPECT_EQ(1, sample.candidates[2].display_id);
    EXPECT_EQ(2, sample.candidates[2].traj_idx);
    EXPECT_EQ(sample.SOURCE_GAP_CURSOR, sample.source);
    EXPECT_EQ(2, sample.selected_idx);
    EXPECT_NEAR(0.0, sample.candidates[2].cursor_dist, 1e-6);
    EXPECT_NEAR(2.0, sample.candidates[0].cursor_dist, 1e-6);
    // the cursor recorded is the one used
    EXPECT_DOUBLE_EQ(3.0, sample.cursor_odom.x);
    EXPECT_DOUBLE_EQ(-1.0, sample.cursor_odom.y);
    EXPECT_TRUE(sample.cursor_valid);
    EXPECT_TRUE(sample.cursor_live);
    // the chosen trajectory is the selected one; the lifespan is its timing span
    ASSERT_EQ(6u, chosen.getPathOdomFrame().poses.size());
    EXPECT_DOUBLE_EQ(-1.0, chosen.getPathOdomFrame().poses.back().position.y);
    EXPECT_NEAR(2.5, PlannerTestAccess::currentLifespan(planner_).toSec(), 1e-6);
}

// DynamicGap's costs are recorded per candidate; its pick is the lowest of them
// and does not change the operator's selection
TEST_F(ManualPlannerTest, RecordsDynamicGapCostsAndPick)
{
    PlannerTestAccess::setTrajVisualizer(planner_);
    PlannerTestAccess::cursorCB(planner_, cursor(true, true, 4.0, 1.0));  // odom (3, -1)

    Tick tick;
    addCandidate(tick, trajTo(3.0, 1.0));
    addCandidate(tick, Trajectory());  // infeasible
    addCandidate(tick, trajTo(3.0, -1.0));
    std::vector<std::vector<float>> poseCosts = {std::vector<float>(6, 1.0f), {}, std::vector<float>(6, 2.0f)};
    std::vector<float> terminalCosts = {0.5f, 0.0f, 0.5f};

    manual_gap_msgs::ManualGapSample sample;
    Trajectory chosen;
    int trajFlag = dynamic_gap::NONE;
    int selected = PlannerTestAccess::selectManualTrajectory(planner_, trajFlag, tick.tubes, tick.trajs,
                                                             poseCosts, terminalCosts, sample, chosen);

    ASSERT_EQ(3u, sample.candidates.size());
    EXPECT_FLOAT_EQ(1.5f, sample.candidates[0].dg_cost);
    EXPECT_FLOAT_EQ(0.5f, sample.candidates[0].dg_terminal_cost);
    EXPECT_FLOAT_EQ(1.0f, sample.candidates[0].dg_clearance_cost);
    EXPECT_TRUE(std::isinf(sample.candidates[1].dg_cost));
    EXPECT_FLOAT_EQ(2.5f, sample.candidates[2].dg_cost);

    int dgFlag = dynamic_gap::NONE;
    int dgIdx = -1;
    PlannerTestAccess::pickTraj(planner_, dgFlag, dgIdx, tick.trajs, poseCosts, terminalCosts);
    EXPECT_EQ(dynamic_gap::GAP, dgFlag);
    EXPECT_EQ(0, dgIdx);
    // the operator's choice stands
    EXPECT_EQ(2, selected);
    EXPECT_EQ(2, sample.selected_idx);
    EXPECT_EQ(dynamic_gap::GAP, trajFlag);
    EXPECT_DOUBLE_EQ(-1.0, PlannerTestAccess::currentTraj(planner_).getPathOdomFrame().poses.back().position.y);
}

TEST_F(ManualPlannerTest, CostsNotScoredAreNaN)
{
    PlannerTestAccess::setTrajVisualizer(planner_);

    Tick tick;
    addCandidate(tick, trajTo(3.0, 1.0));
    manual_gap_msgs::ManualGapSample sample;
    Trajectory chosen;
    int trajFlag = dynamic_gap::NONE;
    PlannerTestAccess::selectManualTrajectory(planner_, trajFlag, tick.tubes, tick.trajs, sample, chosen);
    ASSERT_EQ(1u, sample.candidates.size());
    EXPECT_TRUE(std::isnan(sample.candidates[0].dg_cost));
}

TEST_F(ManualPlannerTest, NoCursorSelectsNothing)
{
    PlannerTestAccess::setTrajVisualizer(planner_);

    Tick tick;
    addCandidate(tick, trajTo(3.0, 1.0));

    manual_gap_msgs::ManualGapSample sample;
    Trajectory chosen;
    int trajFlag = dynamic_gap::GAP;
    EXPECT_EQ(-1, PlannerTestAccess::selectManualTrajectory(planner_, trajFlag, tick.tubes, tick.trajs, sample, chosen));
    EXPECT_EQ(dynamic_gap::NONE, trajFlag);
    EXPECT_EQ(sample.SOURCE_NONE, sample.source);
    EXPECT_EQ(1u, sample.candidates.size());
    EXPECT_FALSE(sample.cursor_valid);
    EXPECT_TRUE(chosen.getPathOdomFrame().poses.empty());
}

TEST_F(ManualPlannerTest, ResetIsAppliedAtTheNextTick)
{
    PlannerTestAccess::setTrajVisualizer(planner_);
    PlannerTestAccess::setCurrentTraj(planner_, trajTo(5.0, 0.0));

    std_msgs::Int16::Ptr reset(new std_msgs::Int16());
    reset->data = 2;
    PlannerTestAccess::scenarioResetCB(planner_, reset);
    // the callback does not touch the planning thread's trajectory
    EXPECT_FALSE(PlannerTestAccess::currentTraj(planner_).getPathOdomFrame().poses.empty());

    Tick tick;
    manual_gap_msgs::ManualGapSample sample;
    Trajectory chosen;
    int trajFlag = dynamic_gap::NONE;
    PlannerTestAccess::selectManualTrajectory(planner_, trajFlag, tick.tubes, tick.trajs, sample, chosen);
    EXPECT_TRUE(PlannerTestAccess::currentTraj(planner_).getPathOdomFrame().poses.empty());
}

TEST_F(ManualPlannerTest, OldCursorStillSelects)
{
    PlannerTestAccess::setTrajVisualizer(planner_);
    PlannerTestAccess::cursorCB(planner_, cursor(true, true, 4.0, 1.0));
    // no cursor message for a minute (RViz stopped publishing)
    PlannerTestAccess::setCursorRxWall(planner_, ros::WallTime::now() - ros::WallDuration(60.0));

    Tick tick;
    addCandidate(tick, trajTo(3.0, -1.0));
    manual_gap_msgs::ManualGapSample sample;
    Trajectory chosen;
    int trajFlag = dynamic_gap::NONE;
    PlannerTestAccess::selectManualTrajectory(planner_, trajFlag, tick.tubes, tick.trajs, sample, chosen);
    // the last known position is still the operator's selection
    EXPECT_EQ(0, sample.selected_idx);
    EXPECT_EQ(dynamic_gap::GAP, trajFlag);
    EXPECT_TRUE(sample.cursor_valid);
    EXPECT_TRUE(sample.cursor_live);
}

TEST_F(ManualPlannerTest, FailedTransformIsNotLive)
{
    PlannerTestAccess::setTrajVisualizer(planner_);
    PlannerTestAccess::cursorCB(planner_, cursor(true, true, 4.0, 1.0));
    manual_gap_msgs::CursorState::Ptr msg = cursor(true, true, 9.0, 9.0);
    msg->header.frame_id = "nowhere";
    PlannerTestAccess::cursorCB(planner_, msg);

    Tick tick;
    addCandidate(tick, trajTo(3.0, -1.0));
    manual_gap_msgs::ManualGapSample sample;
    Trajectory chosen;
    int trajFlag = dynamic_gap::NONE;
    PlannerTestAccess::selectManualTrajectory(planner_, trajFlag, tick.tubes, tick.trajs, sample, chosen);
    // the last good point is still used, but the label is not live
    EXPECT_EQ(0, sample.selected_idx);
    EXPECT_TRUE(sample.cursor_valid);
    EXPECT_FALSE(sample.cursor_live);
}
