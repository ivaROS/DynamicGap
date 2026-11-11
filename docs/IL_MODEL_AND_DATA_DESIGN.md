# IL Model and Data Design for the Continuous DynamicGap Planner

Oct 17, 2026 · Ewan

## Summary

Use Mixture-of-Gaussians BC over short waypoint trajectories, trained on human demonstrations, then improve it with HG-DAgger using Sirius/IWR-style weighting of intervention samples. Feed it LiDAR + the GRU's tracked positions and velocities (not the Kalman filter's), goal and ego velocity, and do not feed back the last command. Two preconditions are not met today: no teleop data exists, and the GRU does not run inside the planner.

**Update (2026-10-22, dynamic_gap `5ecc15a`).** The data-collection plan's fixes A–D are built: every manual-mode planning tick publishes a `ManualGapSample` (candidates, choice, cursor, goal, scan), sessions are recorded with `record_il:=true`, and `il_tools/il_build_dataset` builds the Alternative 1 dataset. Facts below that changed since this design was written: the planner tick is 10 Hz (≈9 Hz measured), not 5 Hz; the robot drives **holonomic** since fix 44 (strafes, heading fixed, `angular.z = 0`); manual selection uses a direction cone plus a sticky switch (fix 43), not plain nearest-endpoint. Still missing for the MoG route: GRU online (M-02), teleop (M-01), HG-DAgger infrastructure (M-05). DynamicGap's pick and costs are recorded since fix 48 (2026-10-23, M-03). The current system is described in `COMPLETE_OVERVIEW_IL_DOC.md`.

**Review finding (added 2026-10-17).** The current data-collection plan cannot produce this dataset yet. Confirmed: no teleop data exists (every human run is a cursor gap selection), and the GRU does not run inside the planner; it has only been trained and evaluated offline. The gap-selection hybrid (Alternative 1) is what the existing plan already logs; the MoG waypoint model needs extra recording work listed in the next two sections.

The five concepts are two separate choices, not five alternatives:

- **How the model is trained (output type):** BC → Gaussian BC → Mixture-of-Gaussians BC, each a generalization of the one before.
- **How the data is collected:** DAgger → HG-DAgger. Both retrain with BC on an aggregated dataset; they only change who drives and who labels.

| Concept | Role | Fixes | Still fails on | Verdict |
| --- | --- | --- | --- | --- |
| BC (regression) | Output type: one action, MSE/L1 | Simplest baseline, no reward or sim needed | Compounding error, averages left/right | Baseline only |
| Gaussian BC | Output type: mean + std, NLL | Adds an uncertainty estimate | Still one peak, still averages (the 0.08 PointMaze result) | Baseline + reference for MoG |
| Mixture-of-Gaussians BC | Output type: K peaks + weights | Keeps "pass left" and "pass right" separate | Mode collapse, mode-switching jitter | **Use this** |
| DAgger | Data: learner drives, expert relabels | Covariate shift (error grows T, not T²) | Needs a queryable expert; capped at expert skill | Optional, with a privileged sim expert |
| HG-DAgger | Data: learner drives, human takes over | Covariate shift, safely, with natural labels | Operator attention, late interventions | **Use this** |

**Recommended I/O at a glance**

- **Input (\~130–190 numbers per step):** LiDAR min-pooled to 90–180 bins; goal (distance, sin, cos); ego (v, ω) with dropout; the 6 nearest GRU-tracked points × (x, y, vx, vy, valid).
- **Output:** 4–6 trajectory modes, each 8 robot-frame waypoints at 0.25 s spacing (2 s horizon), tracked by DynamicGap's existing trajectory controller.

Two alternatives were also evaluated. A gap-conditioned hybrid (pick the gap, then the trajectory) is Viable as a safe baseline that imitates the gap choices directly, but needs candidate logging first. Diffusion or flow matching over waypoints is the planned upgrade once HG-DAgger has grown the dataset.

## Review against the IL data-collection plan

The current plan (`dynamic_gap/docs/IL_DATA_COLLECTION_PLAN.md`, 2026-10-07) already logs nearly everything Alternative 1 needs. The recommended MoG + HG-DAgger route needs four things it does not cover: GRU tracks at run time, teleop or takeover commands, DynamicGap's own pick, and a control-ownership record. Checked against `dynamic_gap` at `f27f357`, when no fix from the plan had been implemented. Status column updated 2026-10-22 (`5ecc15a`).

| Design needs | Current plan | Status |
| --- | --- | --- |
| Every candidate gap, the chosen index, the cursor and the best-vs-second margin | `ManualGapSample` with `candidates[]`, `selected_idx`, per-candidate `cursor_dist` / `cursor_bearing_diff` (Fix A, B) | **Built** (DG-01, DG-02, DG-13, DG-14) |
| Association IDs so labels survive renumbering | Left/right model-ID pair + bearing per candidate | **Built** (DG-15, R-21) |
| Goal (distance, sin, cos) | `goal_robot_frame`, `local_waypoint_robot_frame` | **Built** (R-10) |
| Ego velocity | `robot_vel` (robot frame) | **Built**; holonomic since fix 44, so (vx, vy) with ω = 0 rather than (v, ω) |
| LiDAR | Full `scan` in each sample | **Built**, but the scan is 512 beams over ±π, not 360; pooling to 90–180 bins must use 512 |
| Future odometry poses for waypoint labels | `odom` (50 Hz) and `/tf` in the bag | Recorded; labels are not built yet (`il_build_dataset` builds gap-index labels only) |
| Episode outcome | Planned (R-13) | **Built** offline: `il_bags.episode_outcomes` (`GOAL_REACHED`, `TIMEOUT`, `ABORTED`, collision flag) |
| GRU tracked positions and velocities | Not recorded; candidate `model_vel` comes from the Kalman filter | **Missing** |
| DynamicGap's own pick and costs | Dropped from v1 (D4, R-01) | **Built** (fix 48, `50e7acd`): `dg_pick_idx`, per-candidate `dg_*` costs |
| Human (v, ω) teleop commands | No teleop path; only cursor gap selection exists | **Missing** |
| HG-DAgger: control owner, takeover, policy output, human command always on | `SOURCE_MODEL` is reserved only | **Missing** |
| Runs with `changeTrajectory()` safety checks on | Manual mode skips them | **Missing** |

**Where the two disagree**

1. **No teleop data exists (confirmed).** The design assumes human (v, ω) demos. Every human run so far is cursor gap selection, where DynamicGap generates and tracks the trajectory. Odometry waypoint labels from those runs imitate DynamicGap's generator steered by human gap choice; speed and timing come from the generator, not the human. That weakens the design's case against output option D, and the design itself notes the cursor gave no speed control.
2. **D4 must flip for Alternative 1.** Its first test is "how often did the human disagree with DynamicGap's pick". In manual mode `pickTraj` and the whole comparison block are skipped (`Planner.cpp:2149-2192` at `f27f357`), so that pick does not exist. Recording it needs a side-effect-free scoring call (R-01). Done in fix 48: `pickTraj` into local outputs, no comparison block.
3. **Turning safety checks on changes robot behaviour.** The design wants human runs with `changeTrajectory()` enabled; the plan's principle is not to change behaviour. Make it a separate collection condition, flagged in every sample. (The plan has since accepted other behaviour changes in manual mode: hold still without a selection, fixes 16/37, and a sticky selection, fix 43. The sticky switch is a form of hysteresis, so M-04's "no hysteresis" no longer holds for new sessions.)
4. **The GRU does not run inside the planner (confirmed).** No planner code calls it. The only launch entry (`mbf_dgap.launch:25-45`) is commented out, points at another user's home directory, loads a pedestrian-density model rather than the velocity model, and its script `gru_gap_feature_node.py` is not in this checkout. The velocity GRUs exist only as `.pt` files in `ml_gap_velocity/models/` (sequence lengths 8 and 15).
5. **Rates.** The design runs the policy at 10 Hz. The planner tick was thought to be 5 Hz; it is 10 Hz configured (`move_base_params.yaml`), ≈9 Hz measured (plan PL-15), so the rates already match. The deployment loop should still have its own rate setting.

**Where they agree.** The design's 1–2 s pre-takeover drop and the plan's reaction-lag shift (DG-09) are the same idea and both need the wall-clock stamps the plan adds (R-14). Soft labels from cursor distance and up-weighting gap switches use fields the plan already records (R-23, R-29). Splitting by episode matches R-31.

## New issues and decisions for the register

This design adds nine issues and four decisions to the plan's register. They use a new prefix, `M-xx`, so they can be moved into the plan's "Unfiled" section as they are.

| ID | Sev | Issue | Proposed fix |
| --- | --- | --- | --- |
| M-01 | BLOCKER (MoG route) | No continuous human action data exists; all human runs are cursor gap selections | Decide D8; if teleop is wanted, add a joystick path and record the human (v, ω) |
| M-02 | HIGH | The GRU does not run inside the planner, so its tracks cannot be recorded | Restore a GRU node that publishes per-scan tracks (x, y, vx, vy, valid, age) and record it |
| M-03 | HIGH | DynamicGap's own pick is never computed in manual mode | Call a side-effect-free `pickTraj` in manual mode and record its index and costs (reverses D4). **Done, fix 48** |
| M-04 | HIGH | The 80% human result had no `changeTrajectory()` checks and no hysteresis | Re-measure as a separate condition; add `safety_checks_on` to the sample |
| M-05 | HIGH | No HG-DAgger infrastructure: arbiter, takeover input, `/control_owner`, policy-output topic | Build an arbiter node; add `control_owner` and the policy output to the sample |
| M-06 | MED | With a model driving, `cursor_live` cannot tell "watching" from "intervening" | Add an explicit takeover toggle to the RViz tool and to `CursorState` |
| M-07 | MED | The frozen GRU is not pinned: several `.pt` files with sequence length 8 or 15, and the launch file names a density model | Pick one model; record its file hash in `versions.yaml` |
| M-08 | MED | The design pools 360 beams; the robot's scan has 512 | Pool 512 beams; record the pooling in the dataset config |
| M-09 | LOW | The policy is meant to run at 10 Hz; the planner ticks at 5 Hz. *Correction: the tick is 10 Hz (PL-15), so the rates match.* | Set the deployment rate separately from the sample rate |

**Decisions to add**

| # | Decision | Suggested default |
| --- | --- | --- |
| D8 | Collect teleop demos, or build the first model from cursor gap-selection runs only? | Cursor runs first (Alternative 1 + odometry waypoints); teleop only if the waypoint model needs human speed control |
| D9 | Record DynamicGap's pick and costs in v1? | Yes; it is the Alternative 1 go/no-go test. **Done, fix 48** |
| D10 | Collect a condition with `changeTrajectory()` checks on? | Yes, as a separate, flagged session type |
| D11 | Which GRU is the frozen track estimator? | Open |

## Project context

The recommendation rests on these facts from earlier work; correct any that are out of date.

- **Planner:** DynamicGap (arXiv 2509.07239), a gap-based local planner for dense pedestrian scenes, run in Arena on ROS Noetic with a 2D LiDAR (512 beams, ±π, 5 m). The simulated robot (rto) is holonomic; dgap ran it as non-holonomic until fix 44 (2026-10-22), and now strafes with a fixed heading.
- **Why IL:** with the real-time manual trajectory selection tool, success was about 80%, versus 50–60% for the automated planner. The gap is in decision-making, not task difficulty. (See M-04: that 80% was measured without `changeTrajectory()` checks.)
- **Velocity estimation:** the Kalman filter hallucinated velocity on static objects and missed it in dense clusters. The GRU (15-frame history; inputs = KF tracked point positions + raw LiDAR) replaced its velocity step and cut collisions from about 50% to 10%. It does not run inside the planner; using it online needs a GRU node (M-02).
- **Goal for this step:** a continuous planner that outputs motion directly, acting more as a planner than a low-level controller.
- **Earlier plan:** BC on full human runs, then HG-DAgger with extra weight on overrides; Gaussian mixture BC for the control-based case. Data recorded as rosbags. No teleop data exists; all human data so far is cursor gap selection.
- **Maze lesson:** single-Gaussian BC scored 0.08 on bimodal PointMaze (28 of 50 runs jittered at the start) because it averaged two routes; DAgger reached 0.63.

## Behavior Cloning (BC)

Plain BC is the right baseline and initializer, but not the final model: it will both average left/right choices and drift into states the demos never covered.

**What.** Log (observation, expert action) pairs and train a network to map one to the other with an MSE or L1 loss. No reward, no simulator in the loop. Every other method here keeps this supervised core.

**Who.**

- ALVINN (Pomerleau, 1988): the first neural driving policy.
- PilotNet (NVIDIA, 2016): camera to steering; side cameras with corrected labels supply recovery data.
- Ross & Bagnell 2010: BC error can grow with the square of the horizon (T²ε).
- Conditional IL (Codevilla, 2018) and CILRS (Codevilla, 2019): goal-conditioning; dataset bias, "inertia" causal confusion, large seed variance.
- ChauffeurNet (Waymo, 2018): 30M examples of pure BC still failed; fixed with trajectory perturbations, extra losses and past-motion dropout.
- robomimic (Mandlekar, 2021): history and demo quality matter most; offline loss is a poor checkpoint guide.
- LiDAR navigation: Pfeiffer et al. 2017 (1080 beams + goal → (v, ω), static scenes) and SCAND (2022, social navigation BC baselines).

**When.** Plentiful demos, a low-dimensional continuous action, no reward, and roughly one sensible action per state. Always the first baseline.

**Why.** It is the simplest and most stable way to put human skill into a policy, and inference is fast enough for 10–20 Hz on a CPU.

**How it is commonly used.**

- Encoder matched to the sensor (1D CNN over LiDAR beams), goal and ego state as vector inputs, MLP head.
- Actions normalized to \[-1, 1\] per dimension, tanh output scaled to the robot's limits.
- L1 or Huber loss (more robust to joystick spikes than MSE), Adam at 1e-4 to 1e-3, batch 64–256.
- Validation split by episode, never by frame.
- Recovery data via perturbation (ChauffeurNet) or noise injected during collection (DART).

**Pros.** Simple, stable, reproducible; no reward, sim interaction or extra labeling; fast inference; strong initializer for DAgger and HG-DAgger.

**Cons.** Covariate shift (one mistake leads to an unseen state and errors compound). Mode averaging (the mean or median of "left" and "right" is "straight into the pedestrian"). Causal confusion (may copy its own velocity history or learn to stay stopped). Capped at demo quality; high variance across seeds.

| Symptom | Likely cause | Check / fix |
| --- | --- | --- |
| Hesitates or creeps toward a pedestrian | Mode averaging | Histogram demo ω at that state; if bimodal, move to MoG |
| Never restarts after stopping | Inertia (causal confusion) | Ablate or drop out the ego-velocity input |
| Good on the demo path, diverges after one error | Covariate shift | DART noise, perturbations, DAgger/HG-DAgger |
| Low validation loss, bad driving | Offline loss is a poor proxy | Select checkpoints by closed-loop success only |
| Results swing between runs | Seed variance | Report mean ± std over 3–5 seeds |

**For this project.** Train it once with the same encoder and data as the final model, as the reference every later stage must beat. Record one DART-style session (noise injected into the executed command, human's intended command logged) for cheap recovery data. This needs a teleop path, which does not exist yet (M-01).

## Gaussian BC

Gaussian BC adds a learned uncertainty to plain BC but keeps a single peak, so it fails the same way on two-way choices; use it as a reference run, not as the planner.

**What.** The network outputs a mean and a standard deviation for each action dimension, and training minimizes the negative log-likelihood (NLL) of the expert's action. With σ fixed, the loss is exactly MSE; learning it lets the model say how consistent the demos were in each state.

**Who.**

- Kendall & Gal 2017: the heteroscedastic NLL loss used for learned (aleatoric) uncertainty.
- Seitzer et al. 2022: NLL can underfit hard samples while σ inflates; proposes β-NLL.
- SAC (Haarnoja 2018, App. C): the tanh-squashed Gaussian with its log-likelihood correction.
- Library defaults: robomimic (BC-Gaussian), imitation (NLL with a small entropy bonus), d3rlpy, IQL (tanh-Gaussian actor).
- EnsembleDAgger (Menda 2018): uses ensemble spread, not one Gaussian's σ, for safety gating.

**When.** Low-dimensional continuous actions with mostly one right action per state (one consistent operator). It is the default policy inside most DAgger-family and offline-RL baselines.

**Why.** The learned σ down-weights noisy demo regions, gives a per-state confidence signal, and makes the policy a proper distribution (needed for entropy terms or later RL fine-tuning).

**How it is commonly used.**

- Same encoder as BC; two heads: μ and log σ (state-dependent), or a single learned σ vector (state-independent).
- Loss per dimension: (a − μ)² / 2σ² + log σ; clip log σ to about \[−5, 2\] or use softplus with a floor of 0.01.
- Bounds: tanh on the mean, or a full tanh-squashed distribution (clip demo targets to ±0.999 so the log-likelihood stays finite).
- Deploy with the mean; sampling at run time adds jitter.
- Optional: warm up with fixed σ (pure MSE), then free σ; or β-NLL with β = 0.5.

**Pros.** A strict superset of MSE BC at almost no cost; learned σ flags where demonstrators disagreed; drops straight into DAgger, HG-DAgger and IWR (per-sample weights on the NLL).

**Cons.** One peak: on "pass left or right" the mean lands in the middle and σ just widens, exactly the 0.08 PointMaze failure. NLL pathologies: σ can collapse to the floor or inflate while the mean stops improving. σ measures demo inconsistency, not novelty.

**Diagnostics.** NLL falls and may go negative (normal); track the mean's MSE separately and check it matches a pure-MSE model. On held-out data, (a − μ)/σ should have a spread near 1. σ spikes at decision points (just before a pedestrian crossing) are the tell that the data is multimodal.

**For this project.** Train it with fixed σ and with learned σ on the same data. Where predicted σ spikes is a cheap map of where the human split between options; if those regions coincide with failures in Arena, that is the evidence for MoG.

## Mixture-of-Gaussians BC

MoG BC is the recommended model: it is the cheapest change that lets the policy commit to "pass left" or "pass right" instead of averaging them, and it still runs in one forward pass.

**What.** The network outputs K Gaussians at once: a weight, a mean and a standard deviation for each. Training maximizes the log-likelihood of the expert action under the whole mixture. At run time you pick one component and use its mean, so the robot commits to one option.

**Who.**

- Bishop 1994, Mixture Density Networks: built for multi-valued problems where averaging valid answers gives an invalid one.
- robomimic (Mandlekar 2021): BC-RNN with a GMM head became the standard baseline on human data; mixtures mattered most on multi-operator data. Code defaults: 5 modes, softplus std, low-noise evaluation.
- Driving forecasting: MultiPath (Waymo 2019, anchor trajectories + per-mode Gaussians), MTR (2022, winner-take-all training), Wayformer.
- Critiques and stronger successors: Makansi et al. 2019 (MDN instability and collapse), Implicit BC, BeT, Diffusion Policy.

**When.** Continuous, low-dimensional actions where one observation can have several valid actions: route or gap choice, or several operators with different styles.

**Why.** A unimodal model regresses to the mean of the options, which here is the obstacle between them. A mixture keeps each option as its own peak and gives its probability.

**How it is commonly used.**

- Head outputs K logits, K × D means (tanh-bounded) and K × D standard deviations (softplus + floor).
- Loss: NLL with log-sum-exp over components and log-softmax on the weights; gradient clipping; actions normalized to \[-1, 1\].
- Anti-collapse: initialize component means by k-means on the actions, or winner-take-all training (regress only the closest component and classify which one it was), as MultiPath and MTR do.
- Inference: pick the highest-weight component and take its mean. Never use the mixture's overall mean, which brings back the averaging.
- Temporal consistency: keep the previous component unless another beats it by a margin (hysteresis), or predict whole trajectory chunks per component.

**Pros.** Explicit, inspectable probabilities over discrete choices; one forward pass, fine for 10 Hz on CPU; exact likelihood, so IWR/Sirius weights apply directly; robomimic found little sensitivity to the number of modes (5, 10, 100).

**Cons.** Mode collapse (behaves like Gaussian BC). Per-step mixtures can flip modes between cycles (left-right jitter). NLL blows up if a std shrinks toward zero. Newer methods often beat it: on real-world Push-T, Diffusion Policy scored 0.95 vs 0.20 for an LSTM-GMM.

**Diagnostics.** Defaults K = 3–5, std floor 1e-3 to 1e-2 (normalized), Adam \~1e-4, gradient clipping. Healthy: NLL ends below the single-Gaussian model; at fork states at least two components carry weight with separated means. All weight on one component means collapse. NaN or a sudden NLL plunge means stds hit the floor. Log the chosen component over time; frequent flips mean jitter.

**For this project.** Put a 4–6 mode head on the encoder, each mode a full 2 s waypoint trajectory rather than a single (v, ω). Train with winner-take-all plus mode classification, pick the top mode with hysteresis, and collision-check each mode before execution. If it collapses or jitters in Arena, step up to a diffusion or flow head over the same output, keeping MoG as the baseline.

## DAgger

DAgger fixes compounding error, but its result is capped by the expert it queries. With DynamicGap at 50–60% as the expert it would throw away the human's 80% advantage, so use it only with a stronger privileged sim expert.

**What.** The learned policy drives; an expert labels every state it visits; the labels join a growing dataset; the policy is retrained on all of it.

**Who.**

- Ross, Gordon & Bagnell 2011: the original; error grows linearly with horizon (about Tε) instead of quadratically.
- Ross et al. 2013: UAV forest flight, a human relabeling replayed video (slowed 3×, still laggy labels).
- Pan et al. 2018: AutoRally, an MPC expert teaching a camera policy.
- Learning by Cheating (Chen 2019): a privileged teacher labels a sensor-only student's rollouts.
- SafeDAgger (2016) and Prakash et al. 2020: query and data-size reduction.
- Implementation: the imitation library (SimpleDAggerTrainer; needs a Gym wrapper for ROS/Arena).

**When.** Errors compound, an expert can be queried on arbitrary states, and rolling out an imperfect learner is affordable.

**How it is commonly used.**

1. Iteration 1 is plain BC on expert data (β = 1).
2. Each round, execute the expert with probability β, the learner otherwise; β = 0 after round 1 often works best.
3. Label every visited state with the expert, aggregate, retrain.
4. Run 3–20 rounds of tens of episodes; keep the best policy on held-out rollouts, not the last.

**Pros.** Provably fixes covariate shift; works with any supervised learner; unlimited labels with an automated expert.

**Cons.** Needs a queryable expert; humans label poorly when not in control. Inherits the expert's ceiling. Unsafe learner rollouts without a gate. A stateful expert (DynamicGap tracks gaps over time) must run in shadow mode for its labels to be valid. Does not fix a unimodal policy's averaging.

**Diagnostics.** Success rises roughly monotonically (a dip around round 2 is common); learner–expert disagreement falls each round; a plateau near expert success means the expert is the ceiling.

**For this project.** Optional middle stage. Run DynamicGap with Arena's ground-truth pedestrian velocities as a privileged expert, measure its success first, and use it only if it clearly beats the BC model. Keep human demos in the mix at higher weight. Otherwise skip to HG-DAgger.

## HG-DAgger

HG-DAgger is the right interactive stage: the learned model drives, the human takes over when it goes wrong, and the takeovers become the most valuable training data, without capping the model at the planner's 50–60%.

**What.** A BC-trained policy drives while a human watches. When the human judges the state unsafe, they take control, recover and hand back. Only human-controlled segments are labeled and added; then the policy is retrained. The original also logs ensemble disagreement ("doubt") at each takeover.

**Who.**

- Kelly, Sidrane, Driggs-Campbell & Kochenderfer 2019 (Stanford): the original, on a real car.
- EnsembleDAgger (Menda 2018): ensemble variance as the gate.
- IWR (Mandlekar 2020): batches half from interventions, half from everything else.
- Sirius (Liu et al., RSS 2023): per-class reweighting; drops the \~2 s before each takeover.
- ThriftyDAgger (Hoque 2021): the robot asks for help.
- HACO (2022) and PVP (2023): also treat the overridden action as negative.

**When.** A human can spot trouble early and recover in real time, and a BC policy already drives passably.

**How it is commonly used.**

1. Train the policy (ideally a small ensemble) with BC on the demos.
2. Roll it out; the human takes over with a button when needed.
3. Keep human-controlled samples; drop the 1–2 s before each takeover.
4. Retrain with class weights: interventions \~50% of each batch, original demos at their natural share, robot-driven samples down-weighted.
5. Repeat 3–5 rounds; Sirius deploys each round until interventions reach about a third of the initial demo count.

**Pros.** Safe (the human always has authority); better labels than DAgger (on the paper's real car: zero collisions and road departures vs one and one for DAgger, one and six for BC; a small test); data lands where the model is weak; simple to build.

**Cons.** Needs full operator attention; interventions are late and only for obvious mistakes; data depends on when this operator intervenes; cannot fix a model that cannot represent two options.

**Diagnostics.** Healthy: interventions per episode and their duration fall each round; autonomous success rises; doubt rises just before takeovers. A plateau in intervention rate means capacity or multimodality, not data. A drop after a round means bad pre-intervention labels or over-weighted interventions. Oscillation at handback: hand back only in calm states, optionally blending over \~0.3 s.

**For this project.**

- Arbiter node (twist\_mux-style): takeover button > policy.
- Record `/control_owner`, policy output, human command, executed command, ensemble doubt, LiDAR and GRU outputs; always record the human input, even when not in control, to measure reaction lag. (None of this exists yet; see M-05, M-06.)
- Use a 3–5 member MoG ensemble; measure doubt as disagreement between members, not spread within one mixture.
- Run 3–5 rounds of 30–50 Arena episodes, 20–30 min sessions, fixed evaluation seeds; compare against 50–60% (planner) and 80% (human).
- Later: when doubt exceeds a learned threshold, fall back to DynamicGap.

## Inputs: how many, last command, ego state

Minimize the inputs to what the human actually used to decide (about 130–190 numbers), include ego velocity with safeguards, and leave the last control command out.

**The question.** Every extra input is another feature the network can latch onto that predicts the expert without causing the expert's choice. Leaving out something the expert relied on makes the right action ambiguous. The last command and ego velocity are the extreme case: both predict the next command very well because driving is smooth.

**Who.**

- de Haan, Jayaraman & Levine 2019, causal confusion: adding the previous action lowered validation loss and lowered task reward.
- Wen et al. 2020, copycat agents: policies learn to repeat the previous action and fail where behavior should change. Wen et al. 2021 and Chuang et al. 2022 give fixes (keyframe weighting, residual prediction).
- Codevilla 2019: ego speed is needed but causes "inertia".
- ChauffeurNet: hides past motion on 50% of training samples.
- robomimic: extra robot-state inputs cut low-dimensional agents' success by 49–88% (relative).
- Swamy et al. 2021: previous-action and speed inputs create a feedback "latching" loop.
- ACT (Zhao 2023): action chunks give smoothness without feeding past actions back in.

**How it is commonly handled.**

1. Give the policy exactly what the operator saw: scene, goal and robot motion, in the robot frame.
2. Goal as (distance, sin θ, cos θ).
3. Compress LiDAR: min-pool to 90–180 bins, clip, scale to \[0, 1\]. (This robot's scan is 512 beams; see M-08.)
4. Encode dynamics explicitly (GRU tracks) rather than stacking raw scans.
5. Ego (v, ω): 30–50% dropout to zero, small noise, optional auxiliary head predicting it from perception.
6. Smoothness from MoG trajectory chunks, not a previous-command input.
7. z-score every feature on training statistics; angles as sin/cos.

| Choice | Pros | Cons |
| --- | --- | --- |
| Minimal input set | Less causal confusion and overfitting, more data-efficient, easy to ablate | Risk of omitting something the human used |
| Large / maximal input set | Information sufficiency under partial observability | Spurious correlations, copycat shortcuts, far more data, sim-to-real sensitivity |
| Previous control command | Smoother output, proxy for actuator lag, lower offline loss | Textbook copycat: misses changepoints, which in a crowd are the moments a pedestrian cuts in |
| Ego velocity (odometry) | Non-holonomic and acceleration limits make the next action depend on it | Inertia (stays stopped or keeps cruising); roughly equal to the lagged command |

**Diagnostics.** Ablate closed-loop, never by validation loss: {scan + goal}, + ego velocity, + previous command, + history, over 3 seeds. Copycat test: train a small model to predict the action from past actions alone; if the policy barely beats it offline, it is exploiting the shortcut. Measure error only on changepoint frames. At test time, zero or perturb ego and previous-action inputs and count stuck episodes.

**Verdict.** Minimize. Include ego velocity with dropout and noise. Do not include the last command; add it back only if chunked output is still jerky, and then with ≥50% dropout or a residual target, checked with the copycat test.

## Inputs: LiDAR vs KF/GRU velocity estimates

Give both LiDAR and the GRU's tracked positions and velocities, not the Kalman filter's, and train with dropout of the track branch so the scan can override bad tracks. This depends on M-02: the GRU must run online and be recorded.

**The question.** Tracks contain motion already inferred, which a single scan cannot show. The scan contains geometry tracks discard: walls, clutter, the shape of each gap.

**Who.**

- Mid-level inputs beat raw sensors for transfer: Müller et al. 2018, Zhou et al. 2019, Sax et al. 2018.
- Crowd navigation on agent states: SA-CADRL, GA3C-CADRL, SARL / CrowdNav, mostly with perfect states in sim.
- Raw scan stacks: Long et al. 2018 (3 scans into a 1D CNN, sim only); Pfeiffer 2017 (one scan, static scenes).
- Both combined: DRL-VO (Xie & Dames 2023), 0.5 s of LiDAR plus tracked pedestrian positions and velocities; transferred to real robots without retraining.
- Pitfalls: causal confusion, shortcut learning, modality dropout (ModDrop).

**How it is commonly done.** A 1D CNN (circular padding) over the scan; a small per-track MLP followed by max or attention pooling over tracks; concatenate with goal and ego velocity; then the policy head.

| Option | Pros | Cons |
| --- | --- | --- |
| A. Raw LiDAR only | Simplest; no estimator errors; transfers well | No velocity in one scan; discards the GRU work |
| B. LiDAR + GRU tracks | Geometry and motion (DRL-VO design); scan can contradict a bad track; reuses the GRU that cut collisions 50% → 10% | Highest shortcut risk; estimator bias becomes policy bias; needs a set encoder |
| C. Tracks only | Low-dimensional, data-efficient, interpretable | No walls or gap shape; a missed pedestrian in a cluster is invisible |
| D. Stacked LiDAR history | Motion observable without an estimator | Must learn differencing from small data; ego-motion blurs the stack; invites copycat |

**If only one estimator: the GRU.** It is better on the failure modes that matter. Freeze it and feed its outputs or penultimate embedding; add a confidence signal (variance head or track age) if possible. Feeding both KF and GRU is redundant; KF covariance is at most a later ablation.

**Diagnostics.** Drop the whole track branch on 25–50% of samples and individual tracks at random. Train on the GRU's noisy outputs, never sim ground truth. Zero the tracks, zero the scan or shuffle velocities and measure how much the action changes. Inject a phantom velocity on a wall and check the robot does not swerve.

**Verdict.** B with the GRU, trained with track dropout and on noisy estimates. D is the fallback if B shows shortcut dependence; C only for very small data in open scenes; A is unsuitable for dense crowds.

## Outputs

Output a short trajectory of waypoints, not a single cmd\_vel: it makes the model a planner, gives clean labels from odometry, and lets each mixture mode be a whole maneuver that can be collision-checked.

**The question.** Each cycle the model could output (A) one cmd\_vel (v, ω), (B) a chunk of future cmd\_vels, (C) a short horizon of robot-frame waypoints tracked by a controller, or (D) one subgoal handed to DynamicGap's own trajectory generator.

**Who.** ACT and Diffusion Policy (action chunks; ACT's success rose from about 1% to 44% going from 1 to 100 predicted steps); ViNT and NoMaD (8 normalized waypoints, PD tracker, 4 Hz); Learning by Cheating and TransFuser (waypoints + PID); ChauffeurNet (pose trajectory + controller); Pan et al. 2018 (direct steering/throttle); MultiPath and MTR (mixtures over whole trajectories).

**When each is used.** A: reactive or agile control with a strong queryable expert. B: high-rate manipulation. C: the dominant choice in navigation and driving IL. D: hierarchical stacks where a classical planner keeps its guarantees.

**How it is commonly done.** Labels are future odometry poses transformed into the robot frame at time t (hindsight relabeling), so noisy joystick commands are not the target. 4–8 points at 0.1–0.5 s spacing, normalized by v\_max × Δt; inference at 4–10 Hz, the tracker faster, receding horizon.

| Option | Pros | Cons |
| --- | --- | --- |
| A. cmd\_vel | Simplest; labels straight from /cmd\_vel; trivial HG-DAgger labels | Controller, not planner; noisy, lagged labels; per-step modes dither; weakest against compounding error |
| B. cmd\_vel chunks | Commits to a short plan; smoother | Open-loop in velocity, so position drifts; hard to collision-check; ensembling blurs left and right |
| C. Waypoints + tracker | Genuinely planning; clean odometry labels; mixture over whole trajectories; each mode checkable; reuses DynamicGap's trajectory controller | Tracker to tune; tracking lag; degenerate labels when stopped (feasibility projection for a differential drive is not needed on the holonomic rto) |
| D. Subgoal to DynamicGap | Keeps DynamicGap's safety logic; smallest learning problem | Limited to DynamicGap's generator; throws away timing and speed; ambiguous labels |

**Diagnostics.** Average and final displacement error on held-out bags; minimum error over the K modes and how often each mode wins; plan-versus-odometry tracking error; jerk; fraction of modes in collision; mode-switch rate; closed-loop success, collision and timeout in Arena.

**Verdict.** Option C with a whole-trajectory MoG head. DynamicGap's TrajectoryController already tracks a PoseArray with a lookahead law (holonomic since fix 44: constant speed towards the lookahead pose, then the projection operator), so the top mode can be published straight into it. Keep D as an ablation baseline and A only for debugging.

**Review note.** D's main con ("throws away timing and speed") only holds if the demos contain human timing and speed. Cursor-selection demos do not (see the review section), so on today's data C and D learn from the same speed signal.

## Alternative 1: Gap-conditioned hybrid

Viable as a gap-selection model; Marginal for its continuous sub-head. It imitates the advantage actually demonstrated (choosing among DynamicGap's candidates), but its data is not logged yet and the 80% figure may not hold once safety checks are on. It is also the model the current data-collection plan is built for.

**What.** A shared per-gap MLP scores every candidate gap DynamicGap produces; a softmax over that variable-length set picks one, trained with cross-entropy against the chosen gap. An optional second head, conditioned on the chosen gap, outputs the trajectory through it; because the discrete choice settles left vs right, that head can be unimodal.

**Who.**

- Conditional IL (Codevilla 2018): branch per command, regression inside each branch.
- Parameterized actions from RL: Hausknecht & Stone 2016, P-DQN.
- Learned scoring of candidate trajectories: MultiPath (closest structure), VADv2, Hydra-MDP, GTRS (won NAVSIM v2), Zeng et al. 2019.
- Caution: PDM-Closed showed a simple rule-based scorer matching or beating learned planners on nuPlan.
- Same philosophy on a classical planner: APPLR.
- No published IL work learning gap selection for ground robots was found (absence not exhaustively verified).

**Fit for this project.** It fits because the 80% vs 50–60% result came from selecting DynamicGap's own trajectories, a classifier cannot average two gaps, and executed trajectories stay feasible. The tension: it is a learned scorer on DynamicGap, not the continuous planner asked for, and its ceiling is the candidate set.

**Code findings (confirmed in this review at `f27f357`).**

- The labels are not recorded. Cursor mode picks the candidate whose end pose is nearest the cursor and sets it directly (`getCursorSelectedTrajectory`, `Planner.cpp:854`). This is DG-01/DG-02 in the plan.
- Manual mode bypasses the trajectory comparison block that holds the feasibility, lifespan and cost-comparison switching logic: the manual branch returns at `Planner.cpp:2183` before it. It also skips `pickTraj`, so DynamicGap's own choice is never computed (M-03). The 80% was achieved without that safety layer and with no hysteresis (M-04).

**Status at `5ecc15a` (2026-10-22).**

- Labels are recorded: `selectManualTrajectory` builds one candidate list per tick, selects from it and publishes a `ManualGapSample` with `selected_idx` (fixes A, B). `il_tools/il_build_dataset` writes this model's dataset (per-candidate features, label index, split by episode).
- The selection rule is a direction cone (±60°) plus a sticky switch (0.7 distance ratio, fix 43), so new labels have hysteresis. The inference switch rule in step 5 below should be compared with it.
- Manual mode still skips the comparison block (M-04). Since fix 48 (2026-10-23) it runs `pickTraj` into local outputs and records DynamicGap's pick and costs; `il_build_dataset` derives `is_current` from the previous tick's selected model IDs.
- The 80% / 50–60% success figures came from the `arena_evaluation` metrics before fix 03, which could count a timeout as a success; treat them as unreliable until re-measured.

**How it would be built here.**

1. **Logging (C++).** Each cycle: all candidates and features, the chosen index, cursor position, best-vs-second margin, robot state, scan and GRU tracks. Per-gap: endpoints and their GRU velocities, width, angular span, manipulated goal, feasibility and lifespan, DynamicGap's costs, an 8-point trajectory summary, goal alignment, whether it is the current gap, and association IDs. The plan's `ManualGapSample` covers most of this; costs, GRU velocities and "is current" need adding.
2. **Model.** Per-gap MLP over gap features plus shared context (scan bins, goal, ego velocity); pad to about 12 gaps, mask infeasible ones, add an idle token. A small transformer across gaps is an optional upgrade.
3. **Loss.** Cross-entropy with label smoothing, or soft labels from cursor distance. Up-weight "switch" events.
4. **Continuous head (only if justified).** A residual on the generator's trajectory, mainly a speed or time-scaling profile.
5. **Inference.** Replace `pickTraj` with the scorer but still pass through the comparison block. Switch only if the new gap beats the current one by about 0.15 for 2–3 cycles, or the current gap becomes infeasible. Sub-millisecond via ONNX or TorchScript.
6. **HG-DAgger.** Intervene with the existing cursor, which yields a gap label directly (needs M-06's takeover toggle).

**Pros.** Matches where the human advantage was shown; no mode averaging; keeps DynamicGap's feasibility checks and controller; tiny model, little data, interpretable; the intervention interface already exists.

**Cons / risks.** Cannot invent a path when gap detection misses the opening; gap renumbering and flicker corrupt labels (R-21); cursor label noise and keep-vs-switch imbalance; the 80% baseline may not reproduce with checks on; the continuous head likely adds little because the cursor gave no speed control.

**Diagnostics.** Defaults: up to 12 gaps, hidden 128, dropout 0.1, label smoothing 0.1, switch margin 0.15 over 2 cycles; 30–60 min of selection logs, split by episode. First measure how often the human disagreed with DynamicGap's pick; under about 10% means little to learn. Healthy: top-1 accuracy clearly above "pick DynamicGap's lowest cost"; top-2 above 95%; calibrated probabilities; switch-event recall above 60%. In closed loop, compare switches per minute and split failures into "no good candidate" vs "wrong choice".

**Relation to MoG.** Complement, not replacement: the safe baseline and ablation. A MoG mode can be added as an extra candidate to get past the candidate-set ceiling.

**Cheapest deciding experiment.**

1. Add per-cycle candidate and label logging (the plan's Phase 1, plus M-03).
2. Re-run about 50 human episodes with comparison-block checks on; record success and disagreement with DynamicGap's pick.
3. Train the scorer offline and swap it in for `pickTraj` on the same Arena scenarios.

Go if the scorer reaches at least 70% success. Drop it if disagreement is under about 10%, or human success falls toward 60% with checks on.

## Alternative 2: Diffusion / flow matching over waypoints

Marginal now; Viable as the planned upgrade. It fits the output format and safety checking well, but every navigation success found used far more data than a few hours from one operator, where diffusion tends to memorize demos and may add nothing over MoG.

**What.** Start from random noise over the 8 × 2 waypoint sequence and refine it into a trajectory, by iterative denoising (DDPM/DDIM) or by integrating a learned velocity field (flow matching), conditioned on the same observation encoding. It represents any number of modes without choosing K, and as a sampler it can propose many candidates or be steered by a cost.

**Who.**

- Diffusion Policy (Chi 2023): 46.9% average gain on 12 manipulation tasks; 1D U-Net; 100 training steps, 10 inference steps, about 0.1 s on an RTX 3080. State-based Push-T: 0.95 vs 0.67 for LSTM-GMM.
- NoMaD (Sridhar 2023): 8 waypoints, 1D U-Net, 10 denoising steps, 19M parameters, >100 h of data; deploys on ROS Noetic on a Jetson Orin Nano.
- Faster sampling: Consistency Policy, flow matching, RFMP, π0.
- Driving: Diffusion-ES, DiffusionDrive (2 steps from anchors), Diffusion Planner (collision guidance, \~20 Hz, 200 h of data).
- Navigation: LDP (costmap-conditioned, beats LSTM-GMM in sim), NaviDiffusor (collision-cost guidance), SanD-Planner (warm-started partial denoising, 16 candidates at \~10 Hz).
- Critiques: He et al. 2025 (on small data, diffusion policies act like lookup tables); Diff-DAgger (uses the diffusion loss to decide when to ask the expert).
- No published diffusion IL planner on LiDAR + tracks for dense crowds was found (absence not exhaustively verified).

**Fit for this project.** Same output format as NoMaD; sample-then-filter pairs with DynamicGap's collision checker; cost guidance gives a safety hook MoG lacks. Does not fit yet: small single-operator dataset; no explicit likelihood for HG-DAgger gating; extra latency and replanning-consistency work.

**How it would be built here.**

1. Encoder: the MoG encoder unchanged, feeding a 256-d conditioning vector; keep the head swappable.
2. Denoiser: 1D conditional U-Net, NoMaD size (channels 64-128-256, about 3–8M parameters), or an MLP velocity field.
3. Schedule: DDPM with 100 training steps and DDIM at 10, or flow matching with 5–10 Euler steps; distill to 1–2 steps if needed.
4. Training: AdamW 1e-4, cosine decay, EMA 0.999+, batch 256; mirror scan, tracks and goal left-right; hold out whole episodes.
5. Inference at 10 Hz: 16–32 samples in one batch, optional clearance-cost guidance, reject colliding samples, score the rest by cost plus consistency with the previous plan, fall back to DynamicGap if none pass.
6. Replanning consistency: time-shift and partially re-noise the previous plan; penalize mode switches.
7. HG-DAgger: per-sample weighted denoising loss with Sirius-style weights; uncertainty from sample spread, denoising loss or a 3–5 member ensemble.

**Pros.** Any number of modes, no collapse; smooth trajectory chunks; guidance fits the existing checker; an analogous open codebase on ROS Noetic.

**Cons / risks.** Gains over MoG unproven at this data scale; no likelihood for gating; latency of 10 steps × 16 samples on the robot CPU unverified (rough guess 10–40 ms); flicker between gaps without warm-starting; strong guidance can push samples off-distribution; Diffusion Policy pins Python 3.9 while Noetic uses 3.8 (reimplement or run in a separate process).

**Diagnostics.** Denoising loss plateaus above zero; EMA beats raw checkpoints; held-out minimum endpoint error over 16 samples below MoG's; at forks, samples split across gaps at roughly the human's frequencies; collision-filter rejection under about 30%; distance to the nearest training trajectory (memorization); p95 latency on the target CPU.

**Relation to MoG.** The planned upgrade, after the first HG-DAgger rounds have grown the dataset.

**Cheapest deciding experiment.** On the same rosbag split and frozen encoder, train MoG and a flow-matching or DDIM-10 head. Compare held-out minimum error and mode coverage on fork frames, CPU latency at 16 samples, and closed-loop Arena success (about 200 episodes × 3 seeds). Replace MoG only if diffusion wins by more than about 5 points within a 50 ms budget.

## Final spec, ablation plan, next steps

Build one model, MoG BC over 2 s waypoint trajectories from LiDAR + GRU tracks + goal + ego velocity, then run HG-DAgger on it. The dimension choices below are engineering defaults, not values from a single paper. Before step 1 can run, the data-collection plan's Phase 1 must land (done as of 2026-10-22), plus M-02 (GRU online) and D8 (teleop or cursor-only), both still open.

**Input (per step, robot frame, normalized)**

| Input | Encoding | Size |
| --- | --- | --- |
| LiDAR | 512 beams min-pooled to 90 (or 180) bins, clipped, scaled to \[0, 1\]; 1D CNN | 90–180 |
| Goal / next waypoint | distance, sin θ, cos θ | 3 |
| Ego velocity (odometry) | (v, ω), 30–50% dropout + small noise; on the holonomic rto (fix 44) use (vx, vy), ω is 0 | 2 |
| GRU tracks (frozen) | 6 nearest points × (x, y, vx, vy, valid), sorted by distance, zero-padded; per-track MLP + pooling; whole branch dropped on 25–50% of samples | 30 |
| Optional DynamicGap gaps | 2 best gaps × endpoint (x, y) | 8 |
| Previous cmd\_vel | Excluded | 0 |

Temporal context comes only from the GRU's own 15-frame history.

**Output**

| Item | Value |
| --- | --- |
| Trajectory | 8 waypoints (x, y), 0.25 s apart, 2 s horizon, in base\_link at time t, divided by v\_max × 0.25 |
| Head | K = 4–6 modes: logit + 16-value mean + diagonal σ (floor \~1e-3) |
| Loss | Winner-take-all NLL on the closest mode + cross-entropy on which mode; per-sample weights for HG-DAgger |
| Labels | Future odometry poses from the bags, interpolated to 0.25 s; pad and mask at episode ends; down-weight stationary samples |
| Execution | 10 Hz; reject modes that collide with the scan or predicted pedestrians; publish the top mode with hysteresis to DynamicGap's TrajectoryController; fall back to DynamicGap if all modes fail |

**Training pipeline**

1. Convert rosbags to synchronized samples at a fixed rate; split by episode.
2. Train plain BC and Gaussian BC with this encoder as references.
3. Train MoG BC (3 seeds); select checkpoints by closed-loop Arena success.
4. Optional: DAgger with a ground-truth-velocity DynamicGap expert, only if it clearly beats step 3.
5. HG-DAgger, 3–5 rounds of 30–50 episodes: drop 1–2 s before each takeover, interventions at \~50% of each batch, robot-driven samples down-weighted.
6. Later: run-time fallback to DynamicGap when ensemble doubt is high; diffusion head if MoG collapses or jitters.

**Input ablations** (same demos, same MoG head, 3 seeds; success, collisions, time, jerk, split by difficulty)

| Run | Inputs | Question it answers |
| --- | --- | --- |
| A1 | Scan + goal + ego | Is motion information needed at all? |
| A2 | 5-scan stack (0.5 s) | Can the net learn motion itself? |
| A3 | GRU tracks only | Is geometry beyond the tracks needed? |
| A4 | Scan + KF tracks | How much does the GRU beat the KF here? |
| A5 | Scan + GRU tracks | The main design |
| A6 | A5 + track dropout + noise training | Does it survive bad tracks? |
| A7 | A6 with ground-truth velocities at test | Upper bound: how much error is perception? |
| A8 | A6 + previous cmd\_vel | Does the copycat effect appear? |

Stress A5/A6 with 0/25/50% track dropout, phantom wall velocities and a LiDAR noise sweep. Expected (a hypothesis): A6 matches A5 nominally and is far better under stress; if A2 comes close to A6, the simpler D design wins. A4 and A7 need KF tracks and ground-truth pedestrian velocities in the bag too.

**Open questions**

- How many hours of cursor gap-selection runs exist, and from how many operators? (No teleop data exists.)
- Does the GRU output a confidence or track age that can be fed in?
- What success rate does DynamicGap reach with ground-truth velocities (decides step 4)?
- Which GRU checkpoint is the frozen estimator (D11)?

## Sources

The source list came with the original design without links. This review did not re-open these papers; the code findings were checked against the repo.

**Methods**

- Efficient Reductions for Imitation Learning (Ross & Bagnell 2010)
- DAgger: A Reduction of IL to No-Regret Online Learning (Ross et al. 2011)
- HG-DAgger (Kelly et al. 2019)
- EnsembleDAgger (Menda et al. 2018)
- Human-in-the-Loop IL using Remote Teleoperation / IWR (Mandlekar et al. 2020)
- Sirius: Robot Learning on the Job (Liu et al. 2023)
- ThriftyDAgger (Hoque et al. 2021)
- DART (Laskey et al. 2017)
- Mixture Density Networks (Bishop 1994)
- robomimic (Mandlekar et al. 2021) and code
- Overcoming Limitations of MDNs (Makansi et al. 2019)
- Pitfalls of Heteroscedastic Uncertainty (Seitzer et al. 2022)
- Diffusion Policy (Chi et al. 2023)
- ACT (Zhao et al. 2023)

**Driving and navigation**

- PilotNet (Bojarski et al. 2016)
- Conditional IL (Codevilla et al. 2018)
- Exploring the Limitations of BC (Codevilla et al. 2019)
- ChauffeurNet (Bansal et al. 2018)
- Learning by Cheating (Chen et al. 2019)
- Agile Autonomous Driving (Pan et al. 2018)
- From Perception to Decision (Pfeiffer et al. 2017)
- SCAND (Karnan et al. 2022)
- DRL-VO (Xie & Dames 2023)
- ViNT (Shah et al. 2023) and NoMaD (Sridhar et al. 2023)
- MultiPath (Chai et al. 2019) and MTR (Shi et al. 2022)
- DynamicGap paper and repo

**Inputs and causal confusion**

- Causal Confusion in IL (de Haan et al. 2019)
- Fighting Copycat Agents (Wen et al. 2020)
- Keyframe-Focused Visual IL (Wen et al. 2021)
- Residual Action Prediction (Chuang et al. 2022)
- Feedback in IL (Swamy et al. 2021)
- Driving Policy Transfer via Modularity (Müller et al. 2018)
- Does Computer Vision Matter for Action? (Zhou et al. 2019)

**Verification notes.** Conference venues for HG-DAgger and ThriftyDAgger were not confirmed; IWR appears to be a technical report. robomimic's exact std floor depends on how its config is passed. Specific dimensions, dropout rates and round counts are engineering recommendations, not values from one paper. No primary source was found that uses MoG BC specifically for crowd navigation; that recommendation is extrapolated from manipulation and driving-forecasting work.
