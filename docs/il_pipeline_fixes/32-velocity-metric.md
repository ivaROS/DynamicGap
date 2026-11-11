# 32 — Velocity metric computed from positions

**Status:** Fixed · **Repo(s):** arena-evaluation · **Files:** `arena_evaluation/arena_evaluation/metrics.py`, `arena_evaluation/arena_evaluation/metrics_static.py`, `arena_evaluation/README.md`, `arena_evaluation/test/test_metrics_episode_results.py`
**Behaviour change:** Metrics only. `velocity`, `acceleration`, `jerk` and `avg_velocity_in_personal_space` change value.

## Problem
`Metrics._analyze_episode` built `velocities` from `frame["position"]` (`metrics.py:273` before the change), so "velocity" was the norm of `[x, y, yaw]`, the distance from the map origin mixed with the heading. Acceleration and jerk were differences of that value.

## Why it mattered for data collection
The speed, acceleration, jerk and personal-space speed metrics had no physical meaning and changed with where the map origin is.

## What changed
`metrics.py`, `metrics_static.py`: `velocities` come from `frame["velocity"][:2]`, the odometry linear velocity `(vx, vy)`. The recorder has always written `velocity: [vx, vy, wz]`, so old folders work. The angular rate is left out of the speed. `README.md` describes the field.

## Effects
- Autonomous mode: no change.
- Manual mode: no change.
- Recorded data / metrics: `velocity` is the linear speed in m/s per row; `acceleration` and `jerk` are its first and second differences per row (as before, not divided by time). Values from before the fix are not comparable.

## Test
`arena_evaluation/test/test_metrics_episode_results.py`, `VelocityTest`: an episode far from the origin with odometry velocity `(0.3, 0.4, 2.0)` gives speed 0.5 on every row and zero acceleration. Fails on the old code.

```
cd src/arena/evaluation/arena_evaluation && python3 -m unittest discover -s test -v
```
