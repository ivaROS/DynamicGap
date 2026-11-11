#include <gtest/gtest.h>

#include "planner_test_access.h"

using dynamic_gap::Planner;
using dynamic_gap::PlannerTestAccess;
using dynamic_gap::TimeKeeper;
using dynamic_gap::Trajectory;

namespace
{
    // Stand-in for whatever value the caller's trajFlag held before the call
    // (an uninitialised stack int in DynamicGapPlanner before the fix).
    const int kStaleFlag = 12345;

    class TrajFlagEarlyReturnTest : public ::testing::Test
    {
        protected:
            void SetUp() override
            {
                // Planner deletes its TimeKeeper in the destructor.
                PlannerTestAccess::setTimeKeeper(planner_, new TimeKeeper());
            }

            int runLoop()
            {
                Trajectory chosenTraj;
                int trajFlag = kStaleFlag;
                planner_.runPlanningLoop(chosenTraj, trajFlag);
                return trajFlag;
            }

            Planner planner_;
    };
}

TEST_F(TrajFlagEarlyReturnTest, NotInitializedSetsNone)
{
    PlannerTestAccess::setInitialized(planner_, false);
    PlannerTestAccess::setHasLaserScan(planner_, true);
    PlannerTestAccess::setHasGlobalGoal(planner_, true);
    EXPECT_EQ(dynamic_gap::NONE, runLoop());
}

TEST_F(TrajFlagEarlyReturnTest, NoLaserScanSetsNone)
{
    PlannerTestAccess::setInitialized(planner_, true);
    PlannerTestAccess::setHasLaserScan(planner_, false);
    PlannerTestAccess::setHasGlobalGoal(planner_, true);
    EXPECT_EQ(dynamic_gap::NONE, runLoop());
}

TEST_F(TrajFlagEarlyReturnTest, NoGlobalGoalSetsNone)
{
    PlannerTestAccess::setInitialized(planner_, true);
    PlannerTestAccess::setHasLaserScan(planner_, true);
    PlannerTestAccess::setHasGlobalGoal(planner_, false);
    EXPECT_EQ(dynamic_gap::NONE, runLoop());
}

TEST_F(TrajFlagEarlyReturnTest, CollidingSetsNone)
{
    PlannerTestAccess::setInitialized(planner_, true);
    PlannerTestAccess::setHasLaserScan(planner_, true);
    PlannerTestAccess::setHasGlobalGoal(planner_, true);
    PlannerTestAccess::setColliding(planner_, true);
    EXPECT_EQ(dynamic_gap::NONE, runLoop());
}
