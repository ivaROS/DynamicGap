# Port basic GRU gap-density scoring and RViz visuals

This bundle is a reference implementation from holonomic Dynamic GAP. Port only
basic learned gap-sector density scoring and its RViz visuals into the separate
non-holonomic Dynamic GAP repository. Exclude aspect ratio, density-rate/change,
future-density prediction, MPPI, and unrelated controller or trajectory changes.

Treat this as a reference, NOT a patch to apply literally. Read metadata/source_revision.txt,
metadata/implementation_notes.md, MANIFEST.md, and the selected patch first.
The checkout HEAD differs from the historical reference snapshot. final_files are
complete historical files with documented personal-path redactions; they include
pre-existing code that is not part of the requested port.

Before editing the destination:
1. Inspect its AGENTS.md, Git status, build, dependencies, frames, gap/model IDs,
   raw and simplified gap association, scoring, trajectory switching, and RViz setup.
2. Summarize the reference data flow and identify corresponding destination files
   and functions; explain missing prerequisites rather than assuming identical APIs.
3. List the exact destination paths you plan to modify or create and explain each
   planned edit, including defaults, namespace mapping, identity propagation and fallbacks.
4. Preserve destination-specific non-holonomic kinematics, feasibility, collision
   checks, trajectory generation, control, safety, cost aggregation and switching.
   Keep the implementation minimal and avoid unrelated refactoring.
5. STOP FOR USER APPROVAL before modifying any destination file, unless the user
   explicitly tells you to proceed immediately. Read-only inspection may proceed.

After approval, adapt the implementation rather than transplanting complete Planner
files. Maintain stable left endpoint model identity for all trajectories from a gap;
use an explicit invalid/sentinel identity for trajectories without a gap. Re-evaluate
fresh density when scoring the running trajectory if the destination does that.
Keep Kalman estimates usable; learned velocity replacement is a separate feature.
Use configurable destination-appropriate topics and log paths. Retain raw endpoint
and GRU-vs-GT text markers, adapting TF frames and namespaces. Do not interpret
missing simulator ground truth as a known zero target.

Validate build/message generation, six-feature ordering and normalization parity,
warmup/reset behavior, stale/invalid/missing predictions, negative output clamping,
identity reuse/multiple candidates, cost added exactly once, endpoint sector membership,
TF consistency, and the two RViz displays. Report changes, checks and limitations.
Do not claim this historical reference has dedicated density unit tests or that
its experimental validation establishes robustness on the destination robot.
