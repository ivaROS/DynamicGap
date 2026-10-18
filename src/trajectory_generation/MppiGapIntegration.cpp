#include <dynamic_gap/Planner.h>
#include <dynamic_gap/trajectory_generation/MppiOptimizer.h>
#include <dynamic_gap/utils/bezier_utils.h>
#include <algorithm>

namespace dynamic_gap
{
bool Planner::generateMppiGapTrajs(const std::vector<GapTube*>& gapTubes,
                                  std::vector<Trajectory>& trajectories,
                                  std::vector<std::vector<float>>& poseCosts,
                                  std::vector<float>& terminalCosts,
                                  const std::vector<sensor_msgs::LaserScan>& futureScans)
{
    const auto start = ros::WallTime::now();
    const float infinity = std::numeric_limits<float>::infinity();
    std::vector<MppiGapVisualization> visualization;
    const auto publish = [&]() {
        trajVisualizer_->drawMppiTrajectories(cfg_.mppi.visualize ? visualization :
            std::vector<MppiGapVisualization>{},
            std::max(0, cfg_.mppi.max_visualized_samples_per_gap),
            std::max(1, cfg_.mppi.visualization_point_stride));
    };
    if (futureScans.size() < kGapTrajectoryPoseCount)
    {
        ROS_WARN_THROTTLE(1.0, "MPPI needs all 11 native prediction timestamps; refusing a shortened output");
        publish();
        return false;
    }
    if (cfg_.mppi.horizon != int(kGapTrajectoryControlCount))
        ROS_WARN_THROTTLE(5.0, "mppi/horizon does not change the planner output: using the original 10 intervals");
    if (!cfg_.planning.future_scan_propagation)
        ROS_WARN_THROTTLE(5.0, "MPPI: future_scan_propagation is off; collision scoring uses static scans");

    // Keep tube indices: pickTraj uses the selected array index to retrieve its gap.
    // The original generator currently uses only the first gap in each tube too.
    std::vector<std::size_t> valid;
    std::vector<geometry_msgs::PoseArray> references(gapTubes.size());
    std::vector<GapBezierCommand> referenceCommands(gapTubes.size());
    const float currentSpeed = std::hypot(currentRbtVel_.twist.linear.x, currentRbtVel_.twist.linear.y);
    for (std::size_t i = 0; i < gapTubes.size(); ++i)
    {
        if (!gapTubes[i] || gapTubes[i]->size() == 0 || !gapTubes[i]->at(0)->isAvailable())
            continue;
        const Eigen::Vector2f goal = gapTubes[i]->at(0)->getGoal()->getOrigGoalPos();
        if (!goal.allFinite() || !v_dir_.allFinite() || !std::isfinite(min_scan_dist_))
            continue;
        // Exactly the active legacy center curve (k=0), using its existing helper,
        // control-point convention and six geometric samples. No rotated goals.
        const Eigen::Vector2f p0(0, 0);
        const Eigen::Vector2f p1 = p0 + (min_scan_dist_ / 2.0f) * v_dir_;
        std::vector<Eigen::Vector2f> curve;
        for (int j = 0; j <= 5; ++j)
            curve.push_back(bezier(p0, p1, goal, float(j) / 5));
        references[i] = bezierReferencePoses(curve, cfg_.sensor_frame_id);
        referenceCommands[i] = computeGapBezierCommand(curve, currentSpeed,
            cfg_.rbt.vx_absmax, cfg_.rbt.vang_absmax, cfg_.traj.integrate_stept);
        if (!references[i].poses.empty() && referenceCommands[i].valid)
            valid.push_back(i);
    }
    if (valid.empty())
    {
        publish();
        return false;
    }
    // A fixed total budget cannot give every gap a rollout if there are more gaps
    // than samples. Fail explicitly to the original planner, never silently omit gaps.
    if (cfg_.mppi.total_samples < int(valid.size()))
    {
        ROS_WARN_THROTTLE(1.0, "mppi/total_samples is smaller than the valid gap count");
        publish();
        return false;
    }
    constexpr std::size_t horizon = kGapTrajectoryControlCount;
    // Legacy Beziers and scan scoring assume base/laser planar origins coincide.
    // rbtPoseInSensorFrame_ is not populated in this checkout's tfCB; use the
    // valid, initialized origin pose already used by the robot-frame pipeline.
    const geometry_msgs::Pose initial = rbtPoseInRbtFrame_.pose;
    const auto visiblePlan = globalPlanManager_->getVisibleGlobalPlanSnippetRobotFrame(map2rbt_);

    // Same objective for every optimizer, including the single-nominal baseline.
    // Optional endpoint costs are otherwise gap-dependent. Their maximum across
    // current gap contexts keeps native costs without adding a synthetic gate cost.
    const auto score = [&](dwa_Trajectory& trajectory) {
        if (!cfg_.planning.social_cost_function && !cfg_.traj.use_h_cost)
            return evaluateMppiTrajectory(trajectory, *trajEvaluator_, cfg_, futureScans, visiblePlan, nullptr);
        dwa_Trajectory worst;
        float worstCost = -infinity;
        for (std::size_t index : valid)
        {
            auto contextual = trajectory;
            const float cost = evaluateMppiTrajectory(contextual, *trajEvaluator_, cfg_,
                futureScans, visiblePlan, gapTubes[index]->at(0));
            if (cost > worstCost)
            {
                worst = std::move(contextual);
                worstCost = cost;
            }
        }
        trajectory = std::move(worst);
        return worstCost;
    };

    std::vector<std::size_t> optimizerIndices = valid;
    if (cfg_.mppi.mode == "standard")
    {
        // Standard MPPI starts from a forward control sequence, without a gap
        // reference. Associate its one output slot with the local-goal direction
        // solely for the existing leader's indexing/CBF metadata contract.
        const auto waypoint = globalPlanManager_->getGlobalPathLocalWaypointRobotFrame().pose.position;
        const Eigen::Vector2f goal(waypoint.x, waypoint.y);
        const auto nearest = std::min_element(valid.begin(), valid.end(), [&](std::size_t a, std::size_t b) {
            return (gapTubes[a]->at(0)->getGoal()->getOrigGoalPos() - goal).squaredNorm() <
                   (gapTubes[b]->at(0)->getGoal()->getOrigGoalPos() - goal).squaredNorm();
        });
        optimizerIndices = {*nearest};
    }
    trajectories.assign(gapTubes.size(), Trajectory());
    poseCosts.assign(gapTubes.size(), std::vector<float>{infinity});
    terminalCosts.assign(gapTubes.size(), infinity);
    const unsigned int cycle = mppiCycle_++;
    bool anyValid = false;
    std::size_t totalSamples = 0;
    DynamicGapConfig optimizerConfig = cfg_;
    optimizerConfig.mppi.horizon = horizon;
    for (std::size_t optimizerIndex = 0; optimizerIndex < optimizerIndices.size(); ++optimizerIndex)
    {
        const std::size_t i = optimizerIndices[optimizerIndex];
        const double forwardSpeed = referenceCommands[i].linear;
        MppiControls nominal;
        if (cfg_.mppi.mode == "standard")
        {
            geometry_msgs::Twist forward;
            forward.linear.x = forwardSpeed;
            nominal.assign(horizon, forward);
        }
        else
            nominal = generateNominalControlsFromBezier(references[i], initial, *trajController_,
                cfg_, horizon, cfg_.traj.integrate_stept, forwardSpeed);
        if (nominal.empty())
        {
            // A short reference can reach its endpoint before the native distance
            // horizon. Seed with the existing Bezier command rather than pad with
            // stops (which would shorten the output). Every rollout is still checked.
            geometry_msgs::Twist seed;
            seed.linear.x = forwardSpeed;
            seed.angular.z = referenceCommands[i].angular;
            nominal.assign(horizon, seed);
        }

        const std::size_t count = cfg_.mppi.total_samples / optimizerIndices.size() +
            (optimizerIndex < cfg_.mppi.total_samples % optimizerIndices.size() ? 1 : 0);
        MppiOptimizer optimizer(optimizerConfig, unsigned(cfg_.mppi.seed) + 1664525u * cycle + 1013904223u * unsigned(i));
        auto result = optimizer.optimize(initial, nominal, count, cfg_.sensor_frame_id, score, forwardSpeed);
        Gap* gap = gapTubes[i]->at(0);
        // Re-evaluate exactly the candidate exported to the existing pipeline.
        // Never export internal common costs computed against other gaps.
        if (result.valid)
        {
            result.trajectory = rolloutMppiControls(initial, result.controls,
                cfg_.traj.integrate_stept, cfg_.sensor_frame_id);
            result.valid = evaluateMppiOutputTrajectory(result.trajectory, *trajEvaluator_,
                cfg_, futureScans, visiblePlan, gap, forwardSpeed * cfg_.traj.integrate_stept);
        }
        MppiGapVisualization view;
        view.gap_index = i;
        view.left_id = gap->getLeftGapPt()->getModel()->getID();
        view.right_id = gap->getRightGapPt()->getModel()->getID();
        view.total_samples = result.samples.size();
        view.feasible_samples = result.feasible_samples;
        view.cost = result.valid ? result.trajectory.totalTrajCost : infinity;
        const auto goal = globalPlanManager_->getGlobalPathLocalWaypointRobotFrame().pose.position;
        const double initialDistance = std::hypot(initial.position.x - goal.x, initial.position.y - goal.y);
        for (const auto& sample : result.samples)
        {
            view.samples.push_back(sample.pose_array);
            const bool feasible = std::isfinite(sample.totalTrajCost);
            view.sample_feasible.push_back(feasible);
            if (feasible && !sample.positions.empty() &&
                std::hypot(sample.positions.back().x() - goal.x, sample.positions.back().y() - goal.y) < initialDistance)
                ++view.progress_samples;
        }
        totalSamples += view.total_samples;
        if (result.valid)
        {
            auto& optimized = result.trajectory;
            view.nominal = optimized.pose_array;
            Trajectory trajectory(optimized.pose_array, optimized.times);
            trajectory.setPathOdomFrame(gapTrajGenerator_->transformPath(optimized.pose_array, rbt2odom_));
            trajectory.v_cmd = optimized.v_cmd;
            trajectory.w_cmd = optimized.w_cmd;
            trajectory.gap = gap;
            trajectory.leftGapPtID = view.left_id;
            trajectory.rightGapPtID = view.right_id;
            trajectory.humanVelLeft = optimized.humanVelLeft;
            trajectory.humanVelRight = optimized.humanVelRight;
            trajectory.gapPosLeft = optimized.gapPosLeft;
            trajectory.gapPosRight = optimized.gapPosRight;
            trajectory.trajPosLeft = optimized.trajPosLeft;
            trajectory.robotVel = optimized.robotVel;
            trajectories[i] = std::move(trajectory);
            poseCosts[i].clear();
            for (std::size_t t = 0; t < optimized.PoseCosts.size(); ++t)
                poseCosts[i].push_back(optimized.PoseCosts[t] + cfg_.traj.w_path * optimized.PathCosts[t] +
                                      cfg_.traj.w_relvel * optimized.RelVelPoseCosts[t]);
            terminalCosts[i] = optimized.TerminalPoseCost;
            anyValid = true;
        }
        ROS_DEBUG_STREAM_NAMED("MPPI", "gap=" << i << " endpoints=" << view.left_id << "," << view.right_id
            << " samples=" << view.total_samples << " feasible=" << view.feasible_samples
            << " progress=" << view.progress_samples << " cost=" << view.cost);
        visualization.push_back(std::move(view));
    }
    publish(); // exactly one complete sample/nominal frame, after all optimizers
    ROS_INFO_STREAM_THROTTLE_NAMED(1.0, "MPPI", "mode=" << cfg_.mppi.mode << " gaps=" << valid.size()
        << " optimizers=" << optimizerIndices.size() << " samples=" << totalSamples
        << " generation_ms=" << 1000 * (ros::WallTime::now() - start).toSec());
    if (anyValid)
        trajVisualizer_->drawGapTubeTrajectories(trajectories);
    return anyValid;
}
}
