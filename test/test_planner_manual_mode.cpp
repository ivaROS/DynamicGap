#include <gtest/gtest.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int16.h>

#include "planner_test_access.h"

using dynamic_gap::ManualModeParam;
using dynamic_gap::Planner;
using dynamic_gap::PlannerTestAccess;
using dynamic_gap::TimeKeeper;

// Issue 06: /man_gap parsing
TEST(ManualModeParamTest, MissingIsOff)
{
    bool mode = true;
    EXPECT_EQ(ManualModeParam::MISSING, dynamic_gap::parseManualModeParam(false, XmlRpc::XmlRpcValue(true), mode));
    EXPECT_FALSE(mode);
}

TEST(ManualModeParamTest, BoolIsUsed)
{
    bool mode = false;
    EXPECT_EQ(ManualModeParam::BOOL, dynamic_gap::parseManualModeParam(true, XmlRpc::XmlRpcValue(true), mode));
    EXPECT_TRUE(mode);
    EXPECT_EQ(ManualModeParam::BOOL, dynamic_gap::parseManualModeParam(true, XmlRpc::XmlRpcValue(false), mode));
    EXPECT_FALSE(mode);
}

TEST(ManualModeParamTest, StringIsParsedCaseInsensitively)
{
    bool mode = false;
    EXPECT_EQ(ManualModeParam::STRING, dynamic_gap::parseManualModeParam(true, XmlRpc::XmlRpcValue(std::string("True")), mode));
    EXPECT_TRUE(mode);
    EXPECT_EQ(ManualModeParam::STRING, dynamic_gap::parseManualModeParam(true, XmlRpc::XmlRpcValue(std::string("FALSE")), mode));
    EXPECT_FALSE(mode);
}

TEST(ManualModeParamTest, AnythingElseIsOff)
{
    bool mode = true;
    EXPECT_EQ(ManualModeParam::INVALID, dynamic_gap::parseManualModeParam(true, XmlRpc::XmlRpcValue(std::string("yes")), mode));
    EXPECT_FALSE(mode);
    mode = true;
    EXPECT_EQ(ManualModeParam::INVALID, dynamic_gap::parseManualModeParam(true, XmlRpc::XmlRpcValue(1), mode));
    EXPECT_FALSE(mode);
}

TEST(ManualModeParamTest, DefaultIsOff)
{
    Planner planner;
    PlannerTestAccess::setTimeKeeper(planner, new TimeKeeper());
    EXPECT_FALSE(PlannerTestAccess::manualMode(planner));
}

// Issue 18: click-to-move tracking end
TEST(CreatedTrajDoneTest, EndsAtTargetOrAfterTwiceTheEstimate)
{
    EXPECT_FALSE(dynamic_gap::isCreatedTrajDone(1.0f, 1.5, 1.0, 0.25f));
    EXPECT_TRUE(dynamic_gap::isCreatedTrajDone(0.2f, 0.5, 1.0, 0.25f));
    EXPECT_FALSE(dynamic_gap::isCreatedTrajDone(1.0f, 2.0, 1.0, 0.25f));
    EXPECT_TRUE(dynamic_gap::isCreatedTrajDone(1.0f, 2.01, 1.0, 0.25f));
}

namespace
{
    class PlannerCallbackTest : public ::testing::Test
    {
        protected:
            void SetUp() override
            {
                PlannerTestAccess::setTimeKeeper(planner_, new TimeKeeper());
            }

            Planner planner_;
    };

    geometry_msgs::Twist stillTwist() { return geometry_msgs::Twist(); }
}

// Issue 20: no stall reset in manual mode
TEST_F(PlannerCallbackTest, StallFailsInAutonomousMode)
{
    PlannerTestAccess::setManualMode(planner_, false);
    bool accepted = true;
    for (int i = 0; i < 5; i++)
        accepted = planner_.recordAndCheckVel(stillTwist(), dynamic_gap::GAP);
    EXPECT_FALSE(accepted);
}

TEST_F(PlannerCallbackTest, StallIsAcceptedInManualMode)
{
    PlannerTestAccess::setManualMode(planner_, true);
    for (int i = 0; i < 10; i++)
        EXPECT_TRUE(planner_.recordAndCheckVel(stillTwist(), dynamic_gap::GAP));
}

// Issue 24: scan_ is not replaced by a scan whose gaps were not computed
TEST_F(PlannerCallbackTest, CollidingScanDoesNotReplaceScan)
{
    PlannerTestAccess::setGapDetector(planner_, PlannerTestAccess::makeGapDetector(planner_));
    PlannerTestAccess::setRobotInscribedRadius(planner_, 0.2f);

    boost::shared_ptr<sensor_msgs::LaserScan> scan(new sensor_msgs::LaserScan());
    scan->ranges = {1.0f, 0.1f, 1.0f};
    PlannerTestAccess::laserScanCB(planner_, scan);

    EXPECT_TRUE(PlannerTestAccess::colliding(planner_));
    EXPECT_FALSE(PlannerTestAccess::scan(planner_));
}

// Issue 25: ground-truth agent maps are rebuilt per message and cleared on reset
namespace
{
    pedsim_msgs::AgentStatesPtr agents(const std::vector<std::string> & ids)
    {
        pedsim_msgs::AgentStatesPtr msg(new pedsim_msgs::AgentStates());
        for (size_t i = 0; i < ids.size(); i++)
        {
            pedsim_msgs::AgentState state;
            state.id = ids[i];
            state.header.frame_id = "map";
            state.pose.position.x = 1.0 + i;
            state.pose.orientation.w = 1.0;
            msg->agent_states.push_back(state);
        }
        return msg;
    }
}

TEST_F(PlannerCallbackTest, AgentMapsFollowLatestMessageAndReset)
{
    PlannerTestAccess::setAgentVisualizer(planner_, PlannerTestAccess::makeAgentVisualizer(planner_));
    PlannerTestAccess::setRobotFrame(planner_, "base_link");
    PlannerTestAccess::setHaveTFs(planner_, true);

    geometry_msgs::TransformStamped identity;
    identity.header.frame_id = "base_link";
    identity.child_frame_id = "map";
    identity.transform.rotation.w = 1.0;
    PlannerTestAccess::tfBuffer(planner_).setTransform(identity, "test", true);

    PlannerTestAccess::pedOdomCB(planner_, agents({"1", "2", "3"}));
    EXPECT_EQ(3u, PlannerTestAccess::agentPoses(planner_).size());

    PlannerTestAccess::pedOdomCB(planner_, agents({"2"}));
    ASSERT_EQ(1u, PlannerTestAccess::agentPoses(planner_).size());
    EXPECT_EQ(1u, PlannerTestAccess::agentPoses(planner_).count("2"));
    EXPECT_EQ(1u, PlannerTestAccess::agentVels(planner_).size());

    std_msgs::Int16::Ptr reset(new std_msgs::Int16());
    reset->data = 1;
    PlannerTestAccess::scenarioResetCB(planner_, reset);
    EXPECT_TRUE(PlannerTestAccess::agentPoses(planner_).empty());
    EXPECT_TRUE(PlannerTestAccess::agentVels(planner_).empty());
}

// Issue 18: click-to-move is ignored in autonomous mode
TEST_F(PlannerCallbackTest, TrajCreationPointIgnoredInAutonomousMode)
{
    PlannerTestAccess::setManualMode(planner_, false);
    PlannerTestAccess::setHaveTFs(planner_, true);
    PlannerTestAccess::setTrackingCreatedTraj(planner_, true);

    geometry_msgs::PointStamped::Ptr point(new geometry_msgs::PointStamped());
    point->header.frame_id = "map";
    PlannerTestAccess::trajCreationPointCB(planner_, point);

    EXPECT_FALSE(PlannerTestAccess::hasTrajCreationPoint(planner_));
    EXPECT_FALSE(PlannerTestAccess::trackingCreatedTraj(planner_));
}

TEST_F(PlannerCallbackTest, TrajCreationPointKeptInManualMode)
{
    PlannerTestAccess::setManualMode(planner_, true);
    PlannerTestAccess::setHaveTFs(planner_, true);

    geometry_msgs::PointStamped::Ptr point(new geometry_msgs::PointStamped());
    point->header.frame_id = "map";
    PlannerTestAccess::trajCreationPointCB(planner_, point);

    EXPECT_TRUE(PlannerTestAccess::hasTrajCreationPoint(planner_));
}
