# Live Arena obstacle-weight comparison

Ran 2026-11-18 using `roslaunch arena_bringup start_arena.launch tm_modules:=benchmark`
from `/home/azaro/arena_ws`, after sourcing `devel/setup.bash`.
Stopped nodes with `rosnode kill -a`, built with `catkin build dynamic_gap --no-deps -j4`,
and launched a fresh benchmark for each Q value. The benchmark automatically
relaunches itself with the debug suite. Only `DynamicGapPlannerROS/Q` in the
Arena planner YAML was changed (0, 1, 20); the original YAML was restored afterward.
The simulation is stopped at the end.

Scenario: `static_1agent_headon_1unitaway.json`, map `debugging_map_empty`.
Robot start (10,10,0.7), goal (20,10,0); person at (15,11).
Recorded person displacement was exactly zero in all three trials.
Each recorder reset the task and captured one traversal. Initial measured poses
were within 0.054 m of the configured start; asynchronous execution introduces
small timing differences. These are live closed-loop trials, not identical-sample
replays. MPPI uses its compiled defaults (enabled, gap_guided, seed 7, 96 samples,
one iteration); the normal projection operator and goal-approach controller remain active.

| Q multiplier | Closest person center (m) | Nearest laser return (m) | Recorded simulated time (s) |
| --- | ---: | ---: | ---: |
| 0 | 0.965 | 0.646 | 14.0 |
| 1 | 0.993 | 0.673 | 14.0 |
| 20 | 1.218 | 0.899 | 14.0 |

Recording stops within 1.05 m of the goal to precede the benchmark's automatic
reset at its 1 m goal radius. All trials reached this recording threshold;
this does not establish Move Base success at its tighter 0.25 m tolerance.
Distances to person centers are not footprint clearances. One trial per weight
shows a directional response, not statistical significance. The stationary person
is offset 1 m from the straight start-to-goal line despite the scenario name.

`paths.png` overlays measured paths. `summary.json` includes parameters and
metrics; `*x_trace.json` retain robot, laser, command, pedestrian, phase and result
observations. `record.py` is the exact recorder used (workspace-specific paths).
A pedsim AgentStates UTF-8 decoding warning appeared in each run; position samples
were received and showed the stationary person at its configured coordinates.
