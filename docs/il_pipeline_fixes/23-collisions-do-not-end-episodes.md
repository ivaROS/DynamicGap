# 23 — Collisions do not end episodes

**Status:** Docs only (decision 2026-10-17) · **Repo(s):** arena-rosnav · **Files:** none changed
**Behaviour change:** None.

## Problem
The task generator ends an episode only when the robot reaches the goal or the timeout passes (`RobotManager._is_goal_reached`, task timeout). A collision does not end or reset the episode; the robot keeps going.

## Why it matters for data collection
A collision episode can continue to the goal and look like a success in anything that only reads the end state. Ticks after a collision come from a state an expert would not be in.

## Decision
Not changed in this pass. Collisions are recorded instead, so they can be handled offline:
- metrics count contacts, including between rows (issue 33), and score `COLLISION` before `GOAL_REACHED` (issue 03);
- the IL tools flag collision episodes from the recorded scans and drop them from the dataset by default (`il_build_dataset --keep-collision-episodes` keeps and tags them).

Ending episodes on collision would change both modes and every benchmark, so it needs its own decision.
