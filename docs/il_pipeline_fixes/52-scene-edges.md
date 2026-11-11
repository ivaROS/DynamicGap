# 52: Scene edges recorded; gap scorer reads the scene instead of DynamicGap's costs

## Recording (schema 4)

`ManualGapSample.edges` (`ManualGapEdge`): both points of every raw gap
(`currRawGaps_`, before simplification), in scan order, right point then left
point, copied under the same lock as the planning gaps and the scan
(`Planner::fillSampleEdges`). Per edge: scan ray, range, angle, estimator id, and
the estimator state from `Estimator::getState()` (velocity relative to the robot).
Neighbours (`linkEdges`): the other point of its gap, and the next edge on the
other side, the other end of the obstacle between them; the scan covers the full
circle, so the last obstacle closes on the first edge.

`ManualGapCandidate.left_scan_idx` / `right_scan_idx`: the scan rays of the
candidate gap's points before manipulation, which match edges' `scan_idx`.

Older sessions read as before, with no edges.

## Model (il_model.py, il_train.DEFAULTS)

- No term on DynamicGap's costs (`residual` off; `il_train --residual` puts it back).
- Scene: one token per link (gap or obstacle) with both edges' positions and
  ground velocities. Each candidate reads the scene through cross-attention
  (`SceneAttention`), then the candidates see each other (`CandidateAttention`).
- Ground velocity = relative velocity + the robot's linear velocity, as
  `Estimator::isolateGapDynamics` does. The robot's turning is not removed;
  in every recorded session the robot never turned (|yaw rate| = 0).
- Per candidate: its two points and their ground velocities (from the edges at
  the same scan ray, at most `EDGE_MATCH_RAYS` = 3 apart; `edge_vel_known` 0 if none).
- Per tick: robot velocity, goal, local waypoint. No laser scan as input.

Why not the candidates' recorded `left_model_vel`: those come from the planning
gaps' estimator copies, which hold DynamicGap's manipulated state
(`RotatingFrameCartesianKalmanFilter.cpp:143-146`). In the static cs1 session,
63% of them are exactly 0 while the robot drives 12.5 m.

## Compatibility

Export format version 2 (scene); the planner still loads version 1 exports, and
models trained on a shorter candidate feature list. Training on data without
edges turns the scene off with a note.

## Not yet verified

The recording has not run in simulation: the share of candidate points matched
to an edge and the edge velocities on live data are unchecked.
