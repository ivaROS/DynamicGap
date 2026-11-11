# 26 — Operator sees ground truth in RViz

**Status:** Docs only (decision 2026-10-17) · **Repo(s):** dynamic_gap · **Files:** none changed
**Behaviour change:** None.

## Problem
`rviz/arena.rviz` shows pedsim's tracked persons, groups, walls and waypoints (`/pedsim_visualizer/*`) and the planner's ground-truth pedestrian histories (`/rto/pedestrian_histories`). The operator sees every pedestrian, including ones the laser scan does not see, and where they are heading.

## Why it matters for data collection
The model is given the scan, robot velocity, goal and candidate gaps (plan decision D5). Choices the operator makes from information the model never gets look like noise to the model (plan R-11).

## Decision
Not changed in this pass. The RViz config used is saved with every session (`view.rviz`, issue 07), so it is known per session what the operator saw. Cutting the RViz view down to the model's inputs is part of writing the collection protocol (plan 1.8).
