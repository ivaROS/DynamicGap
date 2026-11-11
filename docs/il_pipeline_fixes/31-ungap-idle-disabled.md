# 31 — Ungap and idling disabled in both modes

**Status:** Docs only (decision 2026-10-17) · **Repo(s):** dynamic_gap · **Files:** none changed
**Behaviour change:** None.

## Problem
Ungap and idling trajectory generation is commented out in `runPlanningLoop` (`Planner.cpp`, "UNGAP TRAJECTORY GENERATION" and "IDLING TRAJECTORY GENERATION" blocks), so both the autonomous planner and the operator choose among gap trajectories only. The plan's R-16 originally assumed only manual mode was gap-only.

## Why it matters for data collection
"Go straight" and "wait" are not choices in either mode, so the action space of the data matches the deployed planner, but neither can be learned.

## Decision
Not changed. `ManualGapSample` keeps `ungap_would_apply` and `idle_would_apply` (always false while generation is disabled) so the schema does not change if they are enabled later (plan D6).
