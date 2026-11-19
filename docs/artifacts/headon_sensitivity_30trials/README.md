# Head-on Arena sensitivity: 10 trials per weight

On 2026-11-18, ran 30 fresh-launch trials in order: ten at Q=0, ten at Q=10,
ten at Q=30. All reached the recording threshold, and recorded person displacement
was zero throughout every trial. Other reported planner parameters were identical.

| Q multiplier | Closest person-center distance, mean ± sample SD (m) | Range (m) | Mean nearest laser return (m) | Mean recorded simulation time (s) |
| --- | --- | --- | --- | --- |
| 0 | 0.983 ± 0.017 | 0.961–1.010 | 0.664 | 13.988 |
| 10 | 1.130 ± 0.014 | 1.111–1.153 | 0.812 | 14.006 |
| 30 | 1.194 ± 0.015 | 1.166–1.219 | 0.874 | 14.010 |

Mean closest separation increased by 0.147 m from 0× to 10×, and by another
0.064 m from 10× to 30×. The observed per-weight ranges do not overlap.
These are repeated runs in one environment with the same compiled random seed,
not a sweep across independent seeds or environments. Runtime scheduling and
sensor timing vary; live trajectories and sampled populations diverge as the
controller reacts. No claim about general statistical significance is made.

## Setup and measurement

Launch command, from `/home/azaro/arena_ws` after sourcing `devel/setup.bash`:

```bash
roslaunch arena_bringup start_arena.launch tm_modules:=benchmark
```

The benchmark relaunches into its configured debug suite:
`debugging_map_empty`, `static_1agent_headon_1unitaway.json`, robot start
(10,10,0.7), goal (20,10,0), stationary person (15,11). Thus the person is
1 m off the direct start-to-goal line. Changed only the Arena planner YAML's
`DynamicGapPlannerROS/Q`; retained baseline Q_f=0.05 and all other settings.
MPPI compiled defaults are enabled, gap_guided, seed=7, 96 samples, one iteration;
the existing projection operator and goal-approach controller remain active.
No production code changes were needed after the preceding successful build.

Each trial launched a fresh environment/planner, attached the recorder, reset
the scenario, and stopped with `rosnode kill -a` followed by shutdown of remaining
Arena roslaunch processes. The original planner YAML was restored after every
trial. Environment is stopped after the final trial.

Recording starts upon observing the robot near its configured start after reset.
First measured x was 10.042–10.054 m; yaw was 0.688–0.714 rad. Recording stops at
less than 1.05 m from the goal, before the benchmark automatically resets at
its 1 m arrival radius. This reports arrival at that recording threshold,
not Move Base success at its tighter 0.25 m goal tolerance. Distances to person
centers and laser returns are not robot-footprint clearances. The first 0× trial
ran slower in wall time (16.27 s) but comparable simulated time; use simulation
time for comparison. A pedsim AgentStates UTF-8 warning was present in logs;
position records were received and verified at the configured stationary location.

## Artifacts

- `comparison.png`: all 30 paths and individual minimum-distance measurements.
- `summary.json`: grouped means, sample standard deviations, minima and maxima.
- `trials.csv`: metrics for each trial, including measured starting pose.
- `*x_trial_*.json.gz`: full recorded traces and runtime planner parameters.
- `record.py`, `trial.sh`: exact workspace-specific scripts used.
- `original_planner.yaml`: restored baseline configuration.
- `analyze.py`: regenerate summary and plot with `/usr/bin/python3 analyze.py`.

Raw launch and recorder logs remain in `/tmp/dynamic_gap_headon_30trials`.
The archived runner expects that temporary directory, its recorder and original
YAML to exist; it modifies Arena's planner YAML temporarily and stops all ROS nodes.
