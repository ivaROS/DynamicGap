# MPPI nominal initialization diagnostics

The current planner initializes controls on **every call** to
`generateMppiGapTrajs`, even if the same gap endpoint IDs persist. It constructs
`nominal` from the current gap's Bezier reference, constructs a local
`MppiOptimizer`, and passes that new seed to `optimize`. No optimized control
sequence is stored for the next call. Within one call, successive MPPI iterations
can update the nominal. The default is one iteration.

The Bezier reference is directed toward the gap goal; its fixed-distance rollout
need not terminate exactly at that goal. If reference tracking cannot fill the
horizon, the existing fallback repeats the Bezier-derived command. Standard mode
initializes a forward sequence. These paths all initialize fresh controls.

## RViz

Use the existing enabled **MPPI samples and optimized paths** MarkerArray display
on `/rto/mppi_trajectories` in `rviz/arena.rviz`. If using another RViz setup, add a
MarkerArray display on that topic. MPPI visualization must be enabled.

- Thin **white** line (`.../seed`): the initial control rollout, before optimization.
- Thick **gap-colored** line (`.../nominal`): the checked optimized output.
- Existing gap label (`.../cost`) now includes initialization source, optimizer
  cycle, `warm_start=NO`, and `pair_streak`.

Example from the live validation:

```text
gap 0 [24,25] J=0.02
samples=96 drawn=96 feasible=96 progress=96
FRESH bezier_command_fallback seed; warm_start=NO cycle=135
pair_streak=82 (PERSISTING)
```

`pair_streak` counts consecutive **MPPI-generator observations** of the ordered
(left endpoint model ID, right endpoint model ID) pair, irrespective of the
candidate's array index. The counter observes tube inputs even if optimization
later fails. Duplicate pairs in a call count once. A pair absent on the previous
observation is labeled `FIRST/REAPPEARED`; unknown negative IDs are labeled
`UNKNOWN IDs`. This is an endpoint-identity diagnostic, not proof that an entire
physical opening or its goal geometry remained unchanged. It does not track
raw detections during periods when this generator is bypassed (for example final
approach). No control history is stored by this diagnostic.

Seed markers remain visible when the optimized result is invalid. Scoped DELETE
markers remove seeds and labels when candidates disappear or final approach
clears MPPI visualization. Marker lifetime also limits stale displays to 2 s.

## Logs

Logger `ros.dynamic_gap.MPPISeed` emits one INFO record per actual optimizer call:

```text
observation=136 cycle=135 gap=0 left=24 right=25 pair_streak=82 initialized=1 warm_start=0 source=bezier_command_fallback seed_v=0.75 seed_w=0.0181047 output_valid=1 mean_abs_delta_w=0.0246415
```

`observation` counts generator entries; `cycle` is the existing optimizer-cycle
counter used to seed random sampling. `mean_abs_delta_w` is the mean absolute
angular-control change across the horizon from fresh seed to returned controls,
in rad/s; interpret alongside `output_valid`. It does not compare controls across
cycles. Enable this logger at INFO if your console configuration suppresses it.

## Live verification (2026-11-18)

Stopped Arena with `rosnode kill -a`, rebuilt using
`catkin build dynamic_gap --no-deps -j4`, then launched from `/home/azaro/arena_ws`:

```bash
source devel/setup.bash
roslaunch arena_bringup start_arena.launch tm_modules:=benchmark
```

Used the existing stationary-person scenario and Q=1. Recorded the actual rosout
messages and MarkerArray topic through one reset traversal. The analysis excludes
one in-flight pre-reset record using the first observed robot pose at the reset
start. Arrival is measured at the same 1.05 m recording threshold used in the
previous sensitivity experiment, before the benchmark's 1 m automatic reset.

- **82 optimizer calls, 82 fresh initializations, zero warm starts.**
- Pair **[24,25]** persisted for all 82 observations, spanning **8.115 simulated s**
  (approximately 10 calls/s). **81** initializations occurred with an already
  persisting pair.
- 55 seeds used Bezier reference tracking; 27 used the existing Bezier-command
  fallback. All 82 outputs were valid.
- Mean absolute angular-control changes within calls ranged from 0.0214 to
  0.0946 rad/s. Optimization was changing controls, but those controls were not
  carried into the next call.
- Verified 82 seed lines, 82 optimized lines, 82 reset labels, consistent 11-pose
  paths, and seed DELETE messages when final approach took over.
- Four visualization tests and ten optimizer tests passed, including seed-only
  rendering when output is invalid, persistent/first labels, and stale removal.

This run supports persistence of the endpoint pair in this encounter. It cannot
establish gap persistence for every scene. The unconditional reseeding itself is
confirmed directly by the implementation and is independent of gap churn.
The environment was stopped after verification. Production optimization behavior
and controller weights were unchanged by this instrumentation.

[Logs, captured markers and analysis](artifacts/mppi_seed_persistence/summary.json)
are saved beside the full compressed capture (`run.json.gz`), filtered `seed.log`,
`example_label.txt`, test outputs, and scripts. Recompute checks with
`python3 docs/artifacts/mppi_seed_persistence/analyze.py`.
