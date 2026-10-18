# MPPI integration and the downstream trajectory contract

`Planner::generateGapTrajsV2()` selects the optional MPPI generator when
`mppi/enabled` and the existing `dwa_method` are enabled. MPPI defaults to off.
Each current available gap gets an independent optimizer seeded from its existing
center quadratic Bezier. The resulting candidate enters the original trajectory
selection, switching, and feedback-control pipeline.

## What downstream code actually executes

`Planner::ctrlGeneration()` reads the chosen odometry-frame path, calls
`TrajectoryController::extractTargetPoseIdx()`, and targets that pose with
`constantVelocityControlLawNonHolonomicLookahead()`. The target selector minimizes
position distance plus `0.5 * abs(yaw_error)`, then advances `ctrl_ahead_pose`
indices. Consequently, preserving waypoint spacing matters to physical lookahead.

For gap trajectories, the tracking speed supplied to the controller is
`cfg_.rbt.vx_absmax`. Direct execution of the trajectory's stored `v_cmd` and
`w_cmd` is commented out. Existing CBF, projection, and velocity limiting then
process the feedback command. MPPI therefore shapes a path for this controller;
its controls are not an independently executed velocity schedule.

## Preserved output

Both generators share `GapTrajectoryContract.h`:

- 11 poses and 11 timestamps at `0, dt, ..., 10*dt`;
- `dt = cfg_.traj.integrate_stept`, matching native obstacle predictions;
- the original center-Bezier speed calculation, including its curvature and
  lambda/sigma scaling;
- a constant distance spacing of `v_bezier * dt` and a nominal distance horizon
  of `10 * v_bezier * dt`;
- original sensor/odometry frames, gap indices and endpoint IDs, and CBF metadata;
- native per-pose and terminal cost components computed for the exported path.

The distance horizon refers to distance traveled along the path, not displacement
from the start to the endpoint. The original Bezier sampler measures distances
along its sampled polyline; chords drawn across polyline corners can be slightly
shorter than its requested arc intervals. MPPI's existing Euler rollout has one
straight segment per interval and preserves the requested interval length.

To preserve this physical contract, the ROS integration holds each candidate's
forward speed fixed and optimizes steering. This intentionally reduces the
optimization variables. `sigma_v` and variable-speed optimization remain available
to standalone experiments, but do not change ROS candidate spacing. Both ROS
comparison modes use this constraint and the same total rollout budget.

`mppi/horizon` does not change the ROS output size. If all 11 prediction timestamps
are unavailable, the MPPI branch declines rather than exporting a shorter path.
When the nominal helper reaches a short reference endpoint before completing that
distance, the existing Bezier command initializes the remaining optimization
instead of padding the nominal with stops. All candidates are collision checked.

`evaluateMppiOutputTrajectory()` checks the output layout and expected spacing,
then calls the original `dwa_evaluateTrajectory()` for that gap. Internal MPPI
comparison costs are replaced before packaging the candidate. `pickTraj()` and
`compareToCurrentTraj()` retain their existing scoring and switching behavior.

Prediction timestamps remain nominal: the downstream lookahead controller and
safety filters can change actual speed. This timing mismatch already exists in
the original path-tracking pipeline; this integration does not claim exact
execution of sampled controls.

## Reuse and deliberate differences

Reused unchanged: gap detection, gap propagation/association, Bezier interpolation,
the target selector and lookahead law, dynamic scan propagation, native obstacle,
path, goal, social and CBF cost functions, trajectory transforms, `pickTraj()`,
`compareToCurrentTraj()`, and the execution safety filters.

Modified: `generateGapTrajsV2()` dispatches to MPPI; its original inline speed
calculation is extracted into `computeGapBezierCommand()` and used by both branches.
Configuration loads MPPI parameters. `TrajectoryVisualizer` publishes the combined
sample population, and raises the existing selected-path arrows above it when
MPPI is enabled. `rviz/arena.rviz` includes the sample display.

Added: full-horizon nominal generation and unicycle rollout in `MppiControl`,
weighted optimization and native cost adapters in `MppiOptimizer`, the per-gap
integration helper, shared output contract, marker builder, focused tests and a
standalone demonstration.

Compared with the Python reference: no synthetic gaps, circles, gate penalty,
Python lookahead gains, or Python trajectory cost are used. The rollout uses the
repository's updated-yaw Euler convention. Noise is independent Gaussian noise;
the prototype's temporal smoothing is not implemented. The weighted update is
the prototype's cost-weighted perturbation method; it does not add a theoretical
importance-sampling correction. Default update fraction is 1 rather than the
demo's 0.35. There is no warm start or new gap-tracking mechanism.

## Configuration, visualization and testing

Load `config/mppi.yaml` under the planner's private parameter namespace before
initialization, normally `/rto/move_base_flex/DynamicGapPlannerROS`, then set
`mppi/enabled: true`. Parameters are read during initialization. Default mode is
`gap_guided`; `standard` uses one forward nominal and the same aggregate sample
budget. Defaults are 96 samples per iteration and one iteration.

All gaps' sampled trajectories and optimized nominals are collected in one
`MarkerArray` and published once on `/rto/mppi_trajectories`. Each gap has distinct
marker addressing and a color from the existing palette. Scoped deletions remove
stale samples. `max_visualized_samples_per_gap: 0` draws every sample. The selected
or retained trajectory uses the existing red `curr_exec_dg_traj` display.

Build and run focused tests without launching Arena or RViz:

```bash
catkin build dynamic_gap --no-deps --cmake-args -DCMAKE_BUILD_TYPE=Release
cmake --build ~/arena_ws/build/dynamic_gap --target run_tests_dynamic_gap -j2
catkin_test_results ~/arena_ws/build/dynamic_gap/test_results
```

The focused tests cover native timing parity, pose/timestamp layout, fixed-speed
distance invariants, original final cost components, simulated lookahead,
weighted updates and safety fallbacks, future and between-step collisions, and
combined multi-gap markers and stale deletion.

An earlier Arena/Flatland smoke test verified one complete frame containing 96
samples across six gaps, six optimized candidates, unique marker keys, and a
single frame and timestamp; see `artifacts/arena_marker_verification.json` and
`artifacts/mppi_arena_rviz.png`. That smoke test predates the fixed-spacing change.
The fixed-spacing change is verified with focused tests; Arena was not restarted.
Earlier unoptimized timings did not meet the planning rate, so production runtime
still needs measuring with the chosen build flags, scene and sample budget.
