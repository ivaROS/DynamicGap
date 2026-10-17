#include "mppi_test_scene.h"
#include <dynamic_gap/trajectory_generation/MppiOptimizer.h>
#include <dynamic_gap/trajectory_tracking/TrajectoryController.h>
#include <dynamic_gap/visualization/TrajectoryVisualizer.h>

#include <chrono>
#include <iostream>
#include <set>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mppi_multigap_demo");
    ros::NodeHandle nh("~");
    // Production evaluators have verbose per-pose INFO logging. Keep this demo's
    // deterministic metrics readable and avoid measuring console throughput.
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
    ros::console::notifyLoggerLevelsChanged();
    try
    {
        dynamic_gap_test::Scene scene(nh);
        auto& cfg = scene.cfg;
        cfg.mppi.enabled = true;
        cfg.mppi.horizon = 20;
        cfg.mppi.total_samples = 192;
        cfg.mppi.iterations = 2;
        cfg.mppi.sigma_v = 0.20;
        cfg.mppi.sigma_w = 0.30;
        cfg.mppi.temperature = 0.5;
        nh.param("horizon", cfg.mppi.horizon, cfg.mppi.horizon);
        nh.param("total_samples", cfg.mppi.total_samples, cfg.mppi.total_samples);
        nh.param("iterations", cfg.mppi.iterations, cfg.mppi.iterations);
        dynamic_gap::TrajectoryController controller(nh, cfg);
        dynamic_gap::TrajectoryEvaluator evaluator(cfg);
        scene.initializeEvaluator(evaluator);
        dynamic_gap::TrajectoryVisualizer visualizer(nh, cfg);
        ros::Publisher scene_pub = nh.advertise<visualization_msgs::MarkerArray>("scene", 1, true);
        ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1, true);

        std::size_t changed_predictions = 0;
        for (std::size_t i = 0; i < scene.scan->ranges.size(); ++i)
            if (std::abs(scene.future_scans.back().ranges[i] - scene.scan->ranges[i]) > 0.01)
                ++changed_predictions;
        if (changed_predictions == 0) throw std::runtime_error("Native obstacle predictions did not move");

        geometry_msgs::Pose initial;
        initial.orientation.w = 1;
        const std::vector<float> passage_goals{-2.1f, 0.0f, 2.1f};
        std::vector<dynamic_gap::MppiGapVisualization> displays;
        std::vector<dynamic_gap::Trajectory> nominals;
        std::vector<std::vector<float>> pose_costs;
        std::size_t total_samples = 0, feasible_samples = 0;
        int selected = -1;
        float best_cost = std::numeric_limits<float>::infinity();
        const auto start = std::chrono::steady_clock::now();
        for (std::size_t i = 0; i < passage_goals.size(); ++i)
        {
            // Three known passages in the detected scene. Use the same Bezier
            // primitive and simulated lookahead helper used by the Planner hook.
            auto reference = dynamic_gap::bezierReferencePoses(scene.curve(passage_goals[i]), cfg.sensor_frame_id);
            const auto controls = dynamic_gap::generateNominalControlsFromBezier(
                reference, initial, controller, cfg, cfg.mppi.horizon, cfg.traj.integrate_stept);
            if (controls.size() != std::size_t(cfg.mppi.horizon))
                throw std::runtime_error("Bezier warm start did not fill the complete horizon");
            dynamic_gap::MppiOptimizer optimizer(cfg, 7 + i);
            const auto score = [&](dwa_Trajectory& candidate) {
                return dynamic_gap::evaluateMppiTrajectory(candidate, evaluator, cfg,
                    scene.future_scans, scene.plan, nullptr);
            };
            auto result = optimizer.optimize(initial, controls, cfg.mppi.total_samples / 3 +
                                              (i < cfg.mppi.total_samples % 3 ? 1 : 0),
                                              cfg.sensor_frame_id, score);
            if (!result.valid) throw std::runtime_error("A passage produced no feasible nominal");
            dynamic_gap::MppiGapVisualization display;
            display.gap_index = i;
            display.nominal = result.trajectory.pose_array;
            display.cost = result.trajectory.totalTrajCost;
            display.total_samples = result.samples.size();
            display.feasible_samples = result.feasible_samples;
            for (const auto& sample : result.samples)
            {
                display.samples.push_back(sample.pose_array);
                display.sample_feasible.push_back(std::isfinite(sample.totalTrajCost));
                if (sample.pose_array.poses.back().position.x > 0.1)
                    ++display.progress_samples;
            }
            total_samples += result.samples.size();
            feasible_samples += result.feasible_samples;
            dynamic_gap::Trajectory trajectory;
            trajectory.setPathRbtFrame(result.trajectory.pose_array);
            trajectory.setPathOdomFrame(result.trajectory.pose_array);
            trajectory.setPathTiming(result.trajectory.times);
            trajectory.v_cmd = result.controls.front().linear.x;
            trajectory.w_cmd = result.controls.front().angular.z;
            nominals.push_back(trajectory);
            pose_costs.push_back(result.trajectory.PoseCosts);
            displays.push_back(std::move(display));
            if (result.trajectory.totalTrajCost < best_cost)
            {
                selected = i;
                best_cost = result.trajectory.totalTrajCost;
            }
        }
        // A single unconditioned optimizer gets the identical total population,
        // prediction horizon, native cost callback, and iteration count.
        const double gap_milliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        dynamic_gap::MppiControls standard_seed(cfg.mppi.horizon);
        for (auto& u : standard_seed) u.linear.x = cfg.rbt.vx_absmax;
        dynamic_gap::MppiOptimizer standard(cfg, 7);
        const auto baseline = standard.optimize(initial, standard_seed, cfg.mppi.total_samples,
            cfg.sensor_frame_id, [&](dwa_Trajectory& candidate) {
                return dynamic_gap::evaluateMppiTrajectory(candidate, evaluator, cfg,
                    scene.future_scans, scene.plan, nullptr);
            });
        if (baseline.samples.size() != total_samples || !baseline.valid)
            throw std::runtime_error("Standard baseline budget or feasibility check failed");
        const double milliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
        std::cout << "{\"detected_raw_gaps\":" << scene.gaps.size()
                  << ",\"optimized_passages\":" << displays.size()
                  << ",\"sample_trajectories\":" << total_samples
                  << ",\"feasible_samples\":" << feasible_samples
                  << ",\"standard_sample_trajectories\":" << baseline.samples.size()
                  << ",\"changed_prediction_rays\":" << changed_predictions
                  << ",\"selected_passage\":" << selected
                  << ",\"selected_cost\":" << best_cost
                  << ",\"standard_cost\":" << baseline.trajectory.totalTrajCost
                  << ",\"gap_guided_ms\":" << gap_milliseconds
                  << ",\"standard_ms\":" << milliseconds - gap_milliseconds
                  << ",\"both_modes_ms\":" << milliseconds << "}" << std::endl;

        bool once = false;
        nh.param("once", once, once);
        ros::Rate rate(3);
        do
        {
            scene.scan->header.stamp = ros::Time::now();
            scene_pub.publish(scene.markers());
            scan_pub.publish(scene.scan);
            visualizer.drawMppiTrajectories(displays);
            visualizer.drawCurrentTrajectory(nominals.at(selected));
            ros::spinOnce();
            if (once) break;
            rate.sleep();
        } while (ros::ok());
    }
    catch (const std::exception& error)
    {
        std::cerr << "MPPI multi-gap demo failed: " << error.what() << std::endl;
        return 1;
    }
    return 0;
}
