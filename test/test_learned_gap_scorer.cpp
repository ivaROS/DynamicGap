#include <gtest/gtest.h>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "planner_test_access.h"

using dynamic_gap::GapScorerInput;
using dynamic_gap::LearnedGapScorer;
using dynamic_gap::Planner;
using dynamic_gap::PlannerTestAccess;
using dynamic_gap::TimeKeeper;
using dynamic_gap::Trajectory;

// The C++ scorer against il_model on the same samples. Fixtures are written by
// il_tools/il_scorer_fixtures; LEARNED_GAP_SCORER_FIXTURES names one more
// folder of them (e.g. one made from a trained model with --model).
namespace
{
    const std::string FIXTURES = SCORER_FIXTURE_DIR;

    std::vector<std::string> caseFiles(const std::string & dir)
    {
        std::vector<std::string> files;
        DIR * d = opendir(dir.c_str());
        if (!d)
            return files;
        const std::string suffix = "_cases.json";
        while (dirent * entry = readdir(d))
        {
            std::string name = entry->d_name;
            if (name.size() > suffix.size() && name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
                files.push_back(dir + "/" + name);
        }
        closedir(d);
        return files;
    }

    manual_gap_msgs::ManualGapSample sampleOf(const YAML::Node & s)
    {
        manual_gap_msgs::ManualGapSample sample;
        std::vector<double> pose = s["robot_pose"].as<std::vector<double>>();
        sample.robot_pose_odom.position.x = pose[0];
        sample.robot_pose_odom.position.y = pose[1];
        sample.robot_pose_odom.orientation.z = pose[2];
        sample.robot_pose_odom.orientation.w = pose[3];
        std::vector<double> vel = s["robot_vel"].as<std::vector<double>>();
        sample.robot_vel.linear.x = vel[0];
        sample.robot_vel.linear.y = vel[1];
        sample.robot_vel.angular.z = vel[2];
        sample.goal_robot_frame.x = s["goal"][0].as<double>();
        sample.goal_robot_frame.y = s["goal"][1].as<double>();
        sample.local_waypoint_robot_frame.x = s["waypoint"][0].as<double>();
        sample.local_waypoint_robot_frame.y = s["waypoint"][1].as<double>();
        for (const YAML::Node & c : s["candidates"])
        {
            manual_gap_msgs::ManualGapCandidate m;
            m.feasible = c["feasible"].as<bool>();
            m.shown = c["shown"].as<bool>();
            m.left_range = c["left_range"].as<float>();
            m.left_theta = c["left_theta"].as<float>();
            m.right_range = c["right_range"].as<float>();
            m.right_theta = c["right_theta"].as<float>();
            m.bearing = c["bearing"].as<float>();
            m.left_scan_idx = c["left_scan_idx"].as<int>();
            m.right_scan_idx = c["right_scan_idx"].as<int>();
            m.left_model_pos.x = c["left_model"][0].as<float>();
            m.left_model_pos.y = c["left_model"][1].as<float>();
            m.left_model_vel.x = c["left_model"][2].as<float>();
            m.left_model_vel.y = c["left_model"][3].as<float>();
            m.right_model_pos.x = c["right_model"][0].as<float>();
            m.right_model_pos.y = c["right_model"][1].as<float>();
            m.right_model_vel.x = c["right_model"][2].as<float>();
            m.right_model_vel.y = c["right_model"][3].as<float>();
            m.terminal_pose.position.x = c["terminal"][0].as<double>();
            m.terminal_pose.position.y = c["terminal"][1].as<double>();
            m.dg_terminal_cost = c["dg_terminal_cost"].as<float>();
            m.dg_clearance_cost = c["dg_clearance_cost"].as<float>();
            m.dg_cost = c["dg_cost"].as<float>();
            sample.candidates.push_back(m);
        }
        for (const YAML::Node & e : s["edges"])
        {
            manual_gap_msgs::ManualGapEdge m;
            m.scan_idx = e["scan_idx"].as<int>();
            m.model_id = e["model_id"].as<int>();
            m.range = e["range"].as<float>();
            m.theta = e["theta"].as<float>();
            m.model_pos.x = e["model_pos"][0].as<float>();
            m.model_pos.y = e["model_pos"][1].as<float>();
            m.model_vel.x = e["model_vel"][0].as<float>();
            m.model_vel.y = e["model_vel"][1].as<float>();
            m.gap_neighbor = e["gap_neighbor"].as<int>();
            m.obstacle_neighbor = e["obstacle_neighbor"].as<int>();
            sample.edges.push_back(m);
        }
        return sample;
    }

    // relative to the size of the value: inputs reach tens, scores more
    void expectNear(double expected, double actual, const std::string & what)
    {
        EXPECT_NEAR(expected, actual, 1e-4 * std::max(1.0, std::abs(expected))) << what;
    }

    void checkFixture(const std::string & file)
    {
        SCOPED_TRACE(file);
        const YAML::Node root = YAML::LoadFile(file);
        const std::string dir = file.substr(0, file.rfind('/'));

        LearnedGapScorer scorer;
        std::string error;
        ASSERT_TRUE(scorer.load(dir + "/" + root["model"].as<std::string>(), error)) << error;

        int caseIdx = 0;
        for (const YAML::Node & c : root["cases"])
        {
            const std::string where = "case " + std::to_string(caseIdx++);
            manual_gap_msgs::ManualGapSample sample = sampleOf(c["sample"]);
            GapScorerInput in = scorer.featurize(sample);
            const int n = sample.candidates.size();
            ASSERT_EQ(n, in.cand.rows()) << where;

            for (int k = 0; k < in.ctx.size(); k++)
                expectNear(c["ctx"][k].as<double>(), in.ctx(k), where + " ctx " + std::to_string(k));
            for (int i = 0; i < n; i++)
            {
                const std::string cand = where + " candidate " + std::to_string(i);
                for (int k = 0; k < in.cand.cols(); k++)
                    expectNear(c["cand"][i][k].as<double>(), in.cand(i, k),
                               cand + " " + LearnedGapScorer::candidateFeatures()[k]);
                for (int k = 0; k < 2; k++)
                    expectNear(c["dg"][i][k].as<double>(), in.dg(i, k), cand + " dg");
                EXPECT_EQ(c["dg_avail"][i].as<float>(), in.dgAvail(i)) << cand;
                EXPECT_EQ(c["mask"][i].as<bool>(), in.mask[i]) << cand;
            }

            Eigen::VectorXf scores = scorer.score(in);
            for (int i = 0; i < n; i++)
                expectNear(c["scores"][i].as<double>(), scores(i), where + " score " + std::to_string(i));

            Eigen::VectorXf chosenScores;
            EXPECT_EQ(c["choice"].as<int>(), scorer.choose(sample, {}, chosenScores)) << where;
        }
        EXPECT_GT(caseIdx, 0);
    }

    std::string writeModified(const std::string & from, const std::string & find, const std::string & replace)
    {
        std::ifstream in(from);
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        size_t pos = text.find(find);
        EXPECT_NE(std::string::npos, pos) << find;
        if (pos != std::string::npos)
            text.replace(pos, find.size(), replace);
        std::string path = "/tmp/dynamic_gap_test_scorer_" + std::to_string(::getpid()) + ".json";
        std::ofstream(path) << text;
        return path;
    }
}

TEST(LearnedGapScorer, MatchesPythonOnFixtures)
{
    std::vector<std::string> files = caseFiles(FIXTURES);
    ASSERT_FALSE(files.empty()) << "no fixtures in " << FIXTURES << " (run il_tools/il_scorer_fixtures)";
    if (const char * extra = std::getenv("LEARNED_GAP_SCORER_FIXTURES"))
    {
        std::vector<std::string> more = caseFiles(extra);
        EXPECT_FALSE(more.empty()) << "no fixtures in " << extra;
        files.insert(files.end(), more.begin(), more.end());
    }
    for (const std::string & file : files)
        checkFixture(file);
}

TEST(LearnedGapScorer, RefusesMissingOrForeignModels)
{
    LearnedGapScorer scorer;
    std::string error;
    EXPECT_FALSE(scorer.load("/nonexistent/gap_scorer.json", error));
    EXPECT_FALSE(error.empty());
    EXPECT_FALSE(scorer.isLoaded());

    // a model trained on other features is refused, not run
    const std::string model = FIXTURES + "/attention_model.json";
    std::string renamed = writeModified(model, "\"edge_vel_known\"]", "\"edge_vel_knowm\"]");
    error.clear();
    EXPECT_FALSE(scorer.load(renamed, error));
    EXPECT_NE(std::string::npos, error.find("feature")) << error;
    EXPECT_FALSE(scorer.isLoaded());

    std::string other = writeModified(model, "dynamic_gap_gap_scorer", "something_else");
    EXPECT_FALSE(scorer.load(other, error));
    std::remove(renamed.c_str());
    std::remove(other.c_str());

    // an unloaded scorer chooses nothing
    manual_gap_msgs::ManualGapSample sample;
    sample.candidates.resize(2);
    sample.candidates[0].feasible = sample.candidates[0].shown = true;
    Eigen::VectorXf scores;
    EXPECT_EQ(-1, scorer.choose(sample, {}, scores));
}

// Planner: the learned scorer replaces DynamicGap's selection; without a usable
// choice the robot holds still and DynamicGap's own pick is never followed
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
        traj.setPathRbtFrame(path);
        return traj;
    }

    class LearnedPlannerTest : public ::testing::Test
    {
        protected:
            void SetUp() override
            {
                PlannerTestAccess::setTimeKeeper(planner_, new TimeKeeper());
                PlannerTestAccess::setLearnedMode(planner_, true);
                PlannerTestAccess::setOdomFrame(planner_, "odom");
                PlannerTestAccess::setHaveTFs(planner_, true);
                PlannerTestAccess::setTrajVisualizer(planner_);

                // four candidates: feasible, infeasible, feasible, and feasible but
                // with a pose inside the inflated robot (infinite clearance cost)
                for (const Trajectory & traj : {trajTo(3.0, 1.0), Trajectory(), trajTo(3.0, -1.0), trajTo(2.0, 0.0)})
                {
                    dynamic_gap::Gap gap("base_link", 10, 2.0f, 20, 2.0f, false);
                    tubes_.push_back(new dynamic_gap::GapTube(&gap));
                    trajs_.push_back(traj);
                }
                poseCosts_ = {std::vector<float>(6, 0.2f), {}, std::vector<float>(6, 0.9f),
                              std::vector<float>(6, std::numeric_limits<float>::infinity())};
                terminalCosts_ = {0.5f, 0.0f, 3.0f, 0.1f};
            }

            void TearDown() override
            {
                for (auto * tube : tubes_)
                    delete tube;
            }

            int select(manual_gap_msgs::ManualGapSample & sample, int & trajFlag, Trajectory & chosen)
            {
                return PlannerTestAccess::selectLearnedTrajectory(planner_, trajFlag, tubes_, trajs_,
                                                                  poseCosts_, terminalCosts_, sample, chosen);
            }

            Planner planner_;
            std::vector<dynamic_gap::GapTube *> tubes_;
            std::vector<Trajectory> trajs_;
            std::vector<std::vector<float>> poseCosts_;
            std::vector<float> terminalCosts_;
    };
}

TEST_F(LearnedPlannerTest, NotLoadedSelectsNothing)
{
    manual_gap_msgs::ManualGapSample sample = dynamic_gap::newManualSample();
    Trajectory chosen;
    int trajFlag = dynamic_gap::GAP;

    EXPECT_EQ(-1, select(sample, trajFlag, chosen));
    EXPECT_EQ(dynamic_gap::NONE, trajFlag);
    EXPECT_TRUE(chosen.getPathOdomFrame().poses.empty());
    EXPECT_EQ(sample.SOURCE_NONE, sample.source);
    EXPECT_EQ(-1, sample.selected_idx);
    // the tick is still recorded
    ASSERT_EQ(4u, sample.candidates.size());
    EXPECT_TRUE(std::isnan(sample.candidates[0].model_score));
}

TEST_F(LearnedPlannerTest, FollowsHighestScoreInfiniteCostIncluded)
{
    ASSERT_TRUE(PlannerTestAccess::loadLearnedScorer(planner_, FIXTURES + "/attention_model.json"));

    manual_gap_msgs::ManualGapSample sample = dynamic_gap::newManualSample();
    Trajectory chosen;
    int trajFlag = dynamic_gap::NONE;
    int selected = select(sample, trajFlag, chosen);

    ASSERT_EQ(4u, sample.candidates.size());
    EXPECT_EQ(sample.SOURCE_MODEL, sample.source);
    EXPECT_TRUE(std::isinf(sample.candidates[3].dg_cost));
    // the recorded scores are the scorer's on the recorded sample
    Eigen::VectorXf scores = PlannerTestAccess::learnedScorer(planner_).score(
        PlannerTestAccess::learnedScorer(planner_).featurize(sample));
    EXPECT_TRUE(std::isnan(sample.candidates[1].model_score));  // infeasible
    for (int i : {0, 2, 3})
        EXPECT_FLOAT_EQ(scores(i), sample.candidates[i].model_score) << i;

    // 3 has an infinite cost and is an option all the same
    int expected = 0;
    for (int i : {2, 3})
        if (scores(i) > scores(expected))
            expected = i;
    EXPECT_EQ(expected, selected);
    EXPECT_EQ(expected, sample.selected_idx);
    EXPECT_EQ(dynamic_gap::GAP, trajFlag);
    EXPECT_DOUBLE_EQ(trajs_[expected].getPathOdomFrame().poses.back().position.y,
                     chosen.getPathOdomFrame().poses.back().position.y);
    EXPECT_DOUBLE_EQ(trajs_[expected].getPathOdomFrame().poses.back().position.y,
                     PlannerTestAccess::currentTraj(planner_).getPathOdomFrame().poses.back().position.y);
}

TEST_F(LearnedPlannerTest, NewChoiceStartsOneTrackedTrajectory)
{
    ASSERT_TRUE(PlannerTestAccess::loadLearnedScorer(planner_, FIXTURES + "/attention_model.json"));
    Trajectory chosen;
    int trajFlag = dynamic_gap::NONE;
    int changes = PlannerTestAccess::trajectoryChangeCount(planner_);

    manual_gap_msgs::ManualGapSample first = dynamic_gap::newManualSample();
    int selected = select(first, trajFlag, chosen);
    ASSERT_GE(selected, 0);
    EXPECT_EQ(changes + 1, PlannerTestAccess::trajectoryChangeCount(planner_));
    EXPECT_DOUBLE_EQ(2.5, PlannerTestAccess::currentLifespan(planner_).toSec());

    // the same gap at the next tick is the same tracked trajectory
    manual_gap_msgs::ManualGapSample second = dynamic_gap::newManualSample();
    EXPECT_EQ(selected, select(second, trajFlag, chosen));
    EXPECT_EQ(changes + 1, PlannerTestAccess::trajectoryChangeCount(planner_));
}

TEST_F(LearnedPlannerTest, NoUsableChoiceSelectsNothing)
{
    ASSERT_TRUE(PlannerTestAccess::loadLearnedScorer(planner_, FIXTURES + "/attention_model.json"));
    // no option: every candidate infeasible
    for (size_t i = 0; i < trajs_.size(); i++)
    {
        trajs_[i] = Trajectory();
        poseCosts_[i].clear();
    }

    manual_gap_msgs::ManualGapSample sample = dynamic_gap::newManualSample();
    Trajectory chosen;
    int trajFlag = dynamic_gap::GAP;
    EXPECT_EQ(-1, select(sample, trajFlag, chosen));
    EXPECT_EQ(dynamic_gap::NONE, trajFlag);
    EXPECT_EQ(sample.SOURCE_MODEL, sample.source);
    EXPECT_EQ(-1, sample.selected_idx);
}

TEST_F(LearnedPlannerTest, NoGapsHoldsStill)
{
    PlannerTestAccess::setInitialized(planner_, true);
    PlannerTestAccess::setHasLaserScan(planner_, true);
    PlannerTestAccess::setHasGlobalGoal(planner_, true);

    Trajectory chosenTraj;
    int trajFlag = dynamic_gap::GAP;
    planner_.runPlanningLoop(chosenTraj, trajFlag);

    EXPECT_EQ(dynamic_gap::NONE, trajFlag);
    EXPECT_TRUE(PlannerTestAccess::holdStill(planner_));
}

TEST_F(LearnedPlannerTest, CollidingHoldsStill)
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

// without a choice the controller never runs DynamicGap's control laws (no
// trajectory controller is set here, so running one would crash)
TEST_F(LearnedPlannerTest, NoChoiceNeverRunsPlannerControl)
{
    PlannerTestAccess::setHoldStill(planner_, false);
    geometry_msgs::PoseArray path;
    path.poses.resize(3);

    for (int flag : {dynamic_gap::NONE, dynamic_gap::IDLING, dynamic_gap::UNGAP})
    {
        int trajFlag = flag;
        geometry_msgs::Twist cmd = planner_.ctrlGeneration(path, trajFlag);
        EXPECT_EQ(0.0, cmd.linear.x) << "trajFlag " << flag;
        EXPECT_EQ(0.0, cmd.angular.z) << "trajFlag " << flag;
    }
}
