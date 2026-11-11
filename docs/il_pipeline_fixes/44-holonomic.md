# 44 — dynamic_gap drove the holonomic rto as a diff-drive robot

**Status:** Fixed · **Repo(s):** simulation-setup · **Files:** `configs/mbf/local/dgap_local_planner_params.yaml`
**Behaviour change:** The robot now strafes: `cmd_vel` carries `linear.y`, and `angular.z` is 0 (the robot keeps its heading).

## Problem
rto is simulated with the `HoloSecondOrder` flatland plugin (`entities/robots/rto/rto.model.yaml`, `model_params.yaml: is_holonomic: True`) and has `max_vel_y: 0.75`, but the dgap parameters set `holonomic: false`. The planner then used `constantVelocityControlLawNonHolonomicLookahead` + `processCmdVelNonHolonomic`: the robot turned towards the gap and drove forward only, so it could not follow the planned trajectories directly.

## What changed
`holonomic: true`. `Planner::ctrlGeneration` now uses the holonomic laws: `constantVelocityControlLaw` (constant speed `vx_absmax` towards the lookahead pose, in x and y), `obstacleAvoidanceControlLaw` when there is no trajectory, and `processCmdVel` (projection operator on (vx, vy), then clip to `max_vel_x`/`max_vel_y`). No rebuild needed, only a relaunch.

## Effect on recorded data
- The velocity metric already uses (vx, vy) (IL fix 32).
- Sessions recorded before this change are non-holonomic; do not mix them with new ones without marking them.
- The 360° laser means a fixed heading loses no view.

## Test
`rosparam get /<ns>/move_base_flex/DynamicGapPlannerROS/holonomic` → `true`; during a run, `rostopic echo /<ns>/cmd_vel` shows non-zero `linear.y` and `angular.z: 0.0`.
