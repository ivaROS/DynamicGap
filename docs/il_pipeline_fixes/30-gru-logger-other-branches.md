# 30 — GRU feature logger only on other branches

**Status:** Docs only (decision 2026-10-17) · **Repo(s):** dynamic_gap · **Files:** none changed
**Behaviour change:** None.

## Problem
The logger that writes the GRU gap-velocity features exists only on other branches of dynamic_gap (e.g. `main_gru_v2`, `gru_gap_velocity_dwa_multi_traj_v3_cbf_perpend*`, `EwanFork/main_gru`), not on the manual gap selection branch these fixes are based on.

## Why it matters for data collection
GRU features cannot be recorded with manual selection data from this branch. See also M-02 and M-07 in the IL model design.

## Decision
Not merged in this pass. The per-tick sample records the gap geometry and estimator states (`ManualGapCandidate`), from which features can be recomputed offline; bringing the GRU into the planner is a separate decision (D11).
