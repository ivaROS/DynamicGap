# 34 — Pedestrian heading from force, no velocity recorded

**Status:** Fixed · **Repo(s):** arena-evaluation · **Files:** `arena_evaluation/scripts/data_recorder_node`, `arena_evaluation/arena_evaluation/utils.py`, `arena_evaluation/test/test_scan_min_and_ped_velocity.py`
**Behaviour change:** None in the simulation. `pedsim_agents_data.csv` entries get a `velocity` field and `theta` follows the walking direction.

## Problem
`DataCollector.pedsim_callback` set each pedestrian's `theta` to `atan2(force.y, force.x)`, the direction of the social force on the agent, and did not record its velocity. The force points away from obstacles and other agents as much as along the walk, so `theta` was not the heading. `pedsim_msgs/AgentState` carries the velocity in `twist.linear`.

## Why it mattered for data collection
The "looked at by pedestrians" metrics use `theta` as the pedestrian's gaze direction. Without velocity, pedestrian motion could not be recovered from the CSVs.

## What changed
- `data_recorder_node`: `theta` is `atan2(vy, vx)` when the planar speed is above `PEDESTRIAN_MIN_SPEED` (0.01 m/s) and the force direction otherwise (a standing agent has no walking direction). Each entry gets `velocity: [vx, vy]` from `twist.linear`.
- `utils.py`: `Pedestrian` has an optional `velocity` field (default `None`), so old entries without it still parse.

## Effects
- Autonomous mode: no change.
- Manual mode: no change.
- Recorded data / metrics: `time_looked_at_by_pedestrians` and its total use the walking direction in new folders. Old folders parse as before (`velocity` is `None`).

## Test
`arena_evaluation/test/test_scan_min_and_ped_velocity.py`: a walking agent pushed sideways gets `theta` = its walking direction and its velocity; a standing agent keeps the force direction; an old entry without `velocity` parses with `velocity` `None`. These fail on the old code.

```
cd src/arena/evaluation/arena_evaluation && python3 -m unittest discover -s test -v
```
