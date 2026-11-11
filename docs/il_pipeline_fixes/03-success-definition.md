# 03 — Success never checked the goal

**Status:** Fixed · **Repo(s):** arena-evaluation · **Files:** `arena_evaluation/arena_evaluation/metrics.py`, `arena_evaluation/arena_evaluation/metrics_static.py`, `arena_evaluation/test/test_start_goal_per_episode.py`, `arena_evaluation/README.md`, `arena_evaluation/plot_declarations/sample_schema.yaml`, `arena_evaluation/scripts/create_plots`, `arena_evaluation/test/test_metrics_episode_results.py`
**Behaviour change:** Metrics only. `result` is `GOAL_REACHED` only when the last recorded pose is at the goal; a new value `ABORTED` covers the rest.

## Problem
`Metrics._get_success(time, collisions)` returned `TIMEOUT` if the episode lasted at least `Config.TIMEOUT_TRESHOLD` (180 s, hard-coded), `COLLISION` if a collision was counted, and `GOAL_REACHED` in every other case. It never looked at the goal. The task generator's timeout is a launch argument (`start_trials.launch` `timeout`, written to `/task_generator_server/timeout`), so with `timeout:=60` an episode that timed out after 60 s scored as a success. An episode cut short when the run was stopped also scored as a success.

## Why it mattered for data collection
The reported success rates (80% manual, 50–60% autonomous) counted timeouts below 180 s and truncated episodes as goals reached. Any comparison between manual and autonomous runs, or between runs with different timeouts, was unreliable.

## What changed
`metrics.py`, `metrics_static.py` (same change in both):
- New `DoneReason.ABORTED`: the episode ended before the timeout, away from the goal.
- `_get_run_params()` reads `task_timeout`, `record_frequency`, `goal_radius` and `goal_tolerance_angle` from `params.yaml` (issue 05). Missing values fall back to `Config`: 180 s, 400 ms, and the task generator defaults 1.0 m and 2π (`TaskGenerator.cfg`). A negative timeout means no timeout, as in the task generator.
- `_get_success(time, collisions, final_position, goal)`, in order:
  1. `TIMEOUT` if `time >= timeout - 2 * record interval`. Rows are written once per record interval, so the last row of a timed-out episode can be up to one interval before the timeout; the second interval covers the clock tick that triggers the write.
  2. `COLLISION` if at least `MAX_COLLISIONS` collisions were counted (unchanged).
  3. `GOAL_REACHED` if the last recorded pose is within `goal_radius + speed × (record interval + 0.5 s)` of the goal and its heading within `goal_tolerance_angle`. This is the test the task generator uses (`RobotManager._is_goal_reached`), with a margin for timing: the task generator checks for the goal every 0.5 s and rows are written every record interval, so the last row can be from before the robot entered the radius. `speed` is the linear speed of the last row, so a robot standing short of the goal gets no margin.
  4. `ABORTED` otherwise.
- `_get_mean_position` (the episode's start and goal) takes the most frequent value of the episode. It used to take the lexicographically smallest, which for old per-tick files was often the recorder default `[0, 0, 0]` written before the task generator set the param; with the goal now deciding the result, that would have scored those episodes as `ABORTED`.
- `README.md`, `sample_schema.yaml` and the `create_plots` help text list the new value.

## Effects
- Autonomous mode: no change.
- Manual mode: no change.
- Recorded data / metrics: success rates drop wherever episodes timed out under 180 s or were cut short. Old folders (no `task_timeout` in `params.yaml`) use 180 s, so a timeout below that shows as `ABORTED` rather than `TIMEOUT`; either way it is no longer a success. The 180 s threshold now fires 0.8 s earlier (two record intervals).
- Downstream: `create_plots` counts results by value and filters on `GOAL_REACHED`; both work with the new value. Plot declarations that list `desired_results` can add `ABORTED`.

## Test
`arena_evaluation/test/test_metrics_episode_results.py`, `SuccessTest`: four episodes in one folder (at the goal, short of the goal before the timeout, within one tick of the timeout, at the goal with a collision) give `GOAL_REACHED`, `ABORTED`, `TIMEOUT`, `COLLISION`; a last row 1.4 m out at 1 m/s counts as reached, the same row at standstill or 2 m out does not; `goal_radius` and `goal_tolerance_angle` from `params.yaml` are applied; old `params.yaml` files use the defaults; a negative timeout never times out. Checked for both `metrics` and `metrics_static`. All but the legacy-defaults case fail on the old code.

```
cd src/arena/evaluation/arena_evaluation && python3 -m unittest discover -s test -v
```
