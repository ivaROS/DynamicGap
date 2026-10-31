# Bundle manifest

Read AGENT_PROMPT.md first. This archive is a portable reference for the basic
GRU sector-density cost and RViz visuals; the destination must be inspected first.

## Files and roles

- AGENT_PROMPT.md: destination inspection, minimal port plan and approval boundary.
- metadata/source_revision.txt: exact checkout, baseline, range, snapshot and dirty state.
- metadata/relevant_commits.txt: chronological full subjects/bodies and prerequisite commits.
- metadata/changed_files.txt: exactly the feature patch paths, one per line.
- metadata/diff_stat.txt: git diff --stat for the curated patch.
- metadata/implementation_notes.md: behavior, interfaces, defaults, frames, runtime and porting.
- metadata/comparison_to_pre_gru.txt: prerequisites relative to the user-supplied baseline.
- metadata/patch_selection.txt: patch filtering and intentional exclusions.
- metadata/portability_redactions.json: explicit changes to copied personal paths.
- metadata/stashes_excluded.txt: stash identities; none contributed content.
- metadata/verification.txt: patch, ZIP, privacy and integrity verification results.
- metadata/SHA256SUMS: SHA-256 of every other bundled file.
- metadata/archive_contents.txt: complete expected ZIP member list.
- patches/gru_gap_density_implementation.patch: feature-filtered zero-context diff.

## Complete final snapshots

All final_files come from fda9d417b2144abb52f4b2199ff7f9c94109f5e3.
They are complete files, with one documented personal-path normalization. They
contain existing implementation context and are not instructions to copy all code.
The patch omits unrelated whitespace/log severity and RViz preferences, so its
curated result intentionally differs from the full snapshot in those respects.

| Repository-relative path under final_files/ | Role |
| --- | --- |
| CMakeLists.txt | Feature implementation / message / build / RViz change |
| cfg/ControlParameters.cfg | Pre-existing dependency or integration context; outside feature patch |
| cfg/EstimationParameters.cfg | Pre-existing dependency or integration context; outside feature patch |
| dg_plugin.xml | Pre-existing dependency or integration context; outside feature patch |
| include/dynamic_gap/DynamicGapPlanner.h | Pre-existing dependency or integration context; outside feature patch |
| include/dynamic_gap/Planner.h | Feature implementation / message / build / RViz change |
| include/dynamic_gap/config/DynamicGapConfig.h | Pre-existing dependency or integration context; outside feature patch |
| include/dynamic_gap/gap_estimation/Estimator.h | Pre-existing dependency or integration context; outside feature patch |
| include/dynamic_gap/gap_estimation/RotatingFrameCartesianKalmanFilter.h | Pre-existing dependency or integration context; outside feature patch |
| include/dynamic_gap/scan_processing/DynamicScanPropagator.h | Pre-existing dependency or integration context; outside feature patch |
| include/dynamic_gap/trajectory_evaluation/TrajectoryEvaluator.h | Feature implementation / message / build / RViz change |
| include/dynamic_gap/trajectory_generation/GapTrajectoryGenerator.h | Pre-existing dependency or integration context; outside feature patch |
| include/dynamic_gap/utils/Gap.h | Pre-existing dependency or integration context; outside feature patch |
| include/dynamic_gap/utils/GapPoint.h | Pre-existing dependency or integration context; outside feature patch |
| include/dynamic_gap/utils/Trajectory.h | Pre-existing dependency or integration context; outside feature patch |
| include/dynamic_gap/utils/Utils.h | Pre-existing dependency or integration context; outside feature patch |
| include/dynamic_gap/visualization/GapVisualizer.h | Pre-existing dependency or integration context; outside feature patch |
| ml_gap_velocity/scripts/gru_gap_feature_node.py | Feature implementation / message / build / RViz change |
| ml_gap_velocity/scripts/gru_gap_velocity_node.py | Older learned-velocity interface context; NOT density inference and not required for density port |
| ml_gap_velocity/scripts/train_gap_gru.py | Feature implementation / message / build / RViz change |
| msg/GapFeatureObservation.msg | Feature implementation / message / build / RViz change |
| msg/GapFeaturePrediction.msg | Feature implementation / message / build / RViz change |
| msg/GapPointObservation.msg | Pre-existing dependency or integration context; outside feature patch |
| msg/GapVelocityPrediction.msg | Pre-existing dependency or integration context; outside feature patch |
| package.xml | Pre-existing dependency or integration context; outside feature patch |
| rviz/arena.rviz | Feature implementation / message / build / RViz change |
| src/DynamicGapPlanner.cpp | Pre-existing dependency or integration context; outside feature patch |
| src/Planner.cpp | Feature implementation / message / build / RViz change |
| src/config/DynamicGapConfig.cpp | Pre-existing dependency or integration context; outside feature patch |
| src/gap_estimation/RotatingFrameCartesianKalmanFilter.cpp | Pre-existing dependency or integration context; outside feature patch |
| src/trajectory_evaluation/TrajectoryEvaluator.cpp | Feature implementation / message / build / RViz change |
| src/trajectory_generation/GapTrajectoryGenerator.cpp | Pre-existing dependency or integration context; outside feature patch |
| src/visualization/GapVisualizer.cpp | Pre-existing dependency or integration context; outside feature patch |

## Patch verification and intentional omissions

The patch was generated from exact base Git objects in an isolated temporary
repository. It applies with `git apply --check --unidiff-zero` at the recorded
density base and passes a reverse check. Zero context excludes unchanged personal
path context. It is not a patch against the older user baseline, the current HEAD,
or the non-holonomic destination. Files already present before density (including
package.xml and older velocity messages) are supporting context, not patch changes.

No source Git checkout, index, branch or stash was changed. The ZIP contains no
.git directory and is not a standalone checkout. Model weights and stats, datasets,
maps, plots, build outputs, editor files and backup planner are omitted. No dedicated
density launch or automated test existed in the chosen snapshot. The existing
velocity-only testing utilities are incompatible with the density model and omitted.
Consult implementation_notes.md for all excluded experiments and runtime prerequisites.

Patch curation also omits the unrelated comment/layout-only rewrite of the
pre-existing computePerfectGapVelocityLabel() velocity-label function.
