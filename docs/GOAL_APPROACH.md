# Prediction-checked final approach

MPPI used to return before the original go-to-goal generator ran. Its fixed
distance horizon and full-speed lookahead tracker could then steer past a close
goal or keep selecting gaps pointing away from it. The final approach now runs
before gap generation and trajectory retention. It applies to both MPPI modes
for nonholonomic robots with feedback control enabled; manual control and the
MPPI-disabled planner keep their existing trajectory generators.

The default activation distance is **4 m**, covering the original maximum
3.75 m gap trajectory horizon at 0.75 m/s. A 2 m trigger was insufficient in a
full Arena run: the robot circled outside that radius for 180 seconds. The final
4 m implementation completed that same empty-room route in about 43 seconds.

Activation also requires a short remaining global route and a clear swept
footprint through the static map. An occupied, unknown, or off-map corridor
leaves routing to the normal planner. Thus a goal beside a wall can use a direct
approach, while a goal behind a wall still follows the global path.
The corridor ends at a stopping point inside the configured goal tolerance;
the exact goal centre need not have the extra clearance if the robot can finish
safely within tolerance. The full predicted motion is still collision checked.

For an eligible approach, twelve deterministic steering/speed candidates predict
two seconds of motion. Each uses a distance-dependent speed bounded by 0.5 m/s
and the robot's configured limits. A badly misaligned robot turns in place.
Predicted motion cannot increase distance to the goal. Candidates are checked
against both bracketing future laser scans at motion steps no longer than 50 ms,
with an extra 0.10 m footprint clearance. The first command is held for 250 ms in
the safety rollout to allow for scheduling delay; subsequent servo updates use
100 ms holds. These checked controls are executed directly. The existing
full-speed gap-path tracker is bypassed only for this terminal control mode.

If every candidate is blocked, prediction data is unavailable, or the scan is
more than 250 ms old, the terminal controller commands a stop. It rechecks on
each planning cycle so a blocked approach can resume. Final yaw tolerances use
absolute wrapped yaw error. Global-plan updates and goal checks are serialized
with command computation, and an existing uninitialized trajectory flag on
early returns is initialized; a detected collision explicitly selects a stop.

The normal eleven-pose MPPI gap output contract remains in effect outside this
mode. The terminal trajectory has its own fine time steps, is shown on
`/rto/curr_exec_dg_traj`, and clears the gap sample display. The diagnostic string
topic `/rto/goal_approach_status` reports `approach`, `align`, `yield`,
`stale_scan`, or `unavailable_prediction`.

Parameters are loaded at initialization under
`/rto/move_base_flex/DynamicGapPlannerROS/goal_approach`:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `enabled` | `true` | Allow the terminal mode when MPPI is enabled |
| `activation_distance` | `4.0` | Maximum final-goal distance in metres |
| `max_speed` | `0.5` | Terminal forward-speed cap in m/s |
| `clearance` | `0.10` | Extra clearance outside the inflated robot radius |
| `prediction_horizon` | `2.0` | Prediction duration in seconds, at least 0.5 |
| `max_scan_age` | `0.25` | Maximum measured scan age in seconds |

`config/mppi.yaml` includes these defaults. Restart the planner after parameter
changes while keeping the ROS master alive. To compare against the preceding
MPPI behavior, disable `goal_approach/enabled` before restarting.

## Validation

Nine focused tests exercise wall-adjacent convergence and slowing, alignment
without an outward arc, pedestrian crossing followed by goal completion,
solid blockage, invalid/missing predictions, full first-command hold checking,
final yaw alignment, static/unknown map blockage, and global-path frame handling.
The existing MPPI suites are also run. The final build uses `RelWithDebInfo`
(`-O2 -g -DNDEBUG`) so the planner remains optimized and debugger stacks are useful.

Full Arena results are recorded in
[artifacts/goal_approach_validation.json](artifacts/goal_approach_validation.json)
and summarized in [the results table](artifacts/goal_approach_results.md) and
[the validation plot](artifacts/goal_approach_validation.png). The test matrix
includes three complete room traversals, two wall-adjacent goals (including one
only about 0.25 m from the wall), a goal starting behind the robot, a short
approach, and a crossing person. A pass requires both
the measured goal tolerance and Move Base action success, footprint clearance,
and no significant outward detour after entering the activation radius.

Arena test setup uses runtime parameters and temporary scenario selection; it
does not edit Arena source files. In this checkout Arena reads `auto_reset` but
still starts its automatic reset timer. Its task-level distance tolerance can
reset an episode before the planner reaches its tighter tolerance. Set the
task tolerance **before Arena initializes its robot manager**, and capture each
trial only through its first arrival, excluding subsequent reset jumps.

Source ROS and the workspace in every ROS terminal:

```bash
source /opt/ros/noetic/setup.bash
source ~/arena_ws/devel/setup.bash
```

Keep a separate `roscore` running. From the Dynamic Gap directory, before
starting Arena:

```bash
rosparam set /goal_radius 0.18
rosparam load config/mppi.yaml /rto/move_base_flex/DynamicGapPlannerROS
rosparam set /rto/move_base_flex/DynamicGapPlannerROS/mppi/enabled true
rosparam set /rto/move_base_flex/DynamicGapPlannerROS/mppi/total_samples 144
roslaunch arena_bringup start_arena.launch \
  local_planner:=dgap model:=rto simulator:=flatland map_file:=map_empty \
  tm_robots:=scenario tm_obstacles:=scenario timeout:=240
```

From another sourced terminal in this package:

```bash
for scenario in empty_long diagonal reverse short wall_adjacent very_close_wall facing_away crossing_person; do
  /usr/bin/python3 test/run_goal_approach_arena.py \
    --label "$scenario" --scenario "test/scenarios/goal_approach/$scenario.json"
done
```

The runner selects scenarios through dynamic reconfigure, waits for callbacks,
resets the task, checks the applied goal/start, records motion and action results,
and returns a failing status for timeouts, missing commands, contact, or excessive
regression. If commands disappear, it sends a zero velocity command. JSON
summaries and detailed traces are saved under `/tmp/dynamic_gap_goal_tests`.

These tests validate specific simulated encounters, not every pedestrian motion.
Safety depends on fresh laser data, the existing gap velocity predictions, and
controller timing. The controller cannot guarantee separation from an unobserved
person or one who walks into a stationary robot. Earlier actionlib errors and a
Move Base exit were observed before the final implementation; no cause was
established from that last log line alone. Runtime actionlib and pedsim warnings
are recorded separately from the final navigation outcomes.
