# Experiment Results

This document logs results from Phase 1–2 experiments: date, test conditions (range, target, orientation, clutter, chirp config), detection outcome, and links to the corresponding dataset under `datasets/`. Simulation-side tracker evaluations are logged here too.

## Log

### 2026-08-07 — IMM (CV + CA) vs. constant-velocity motion model, simulation

Offline replay of the Phase 0 chain (`FigureEight` → `MeasurementModel` → `TrackManager`), no ROS. 60 s at 10 Hz, 10 % detection dropout, no clutter, measurement sigmas `(0.1, 0.02, 0.02, 0.1)`, `max_misses: 8`, `gate_chi2: 9.488`. CV run uses `sigma_accel: 3.0`; IMM uses `imm_sigma_accel: 2.0`, `imm_sigma_jerk: 4.0`.

Trajectories are Gerono lemniscates; "peak a" is peak lateral acceleration, `2.125·A·ω²`.

**Distinct track IDs held by the target over one run** (mean of 10 seeds — lower is better):

| Trajectory | peak a (m/s²) | CV | IMM | change |
|---|---|---|---|---|
| A=3 m, T=20 s (`sim_params.yaml` default) | 0.63 | 4.3 | 5.3 | +23 % |
| A=5 m, T=20 s | 1.05 | 5.1 | 6.6 | +29 % |
| A=5 m, T=12 s | 2.91 | 15.0 | 10.9 | −27 % |
| A=5 m, T=8 s | 6.55 | 53.1 | 24.0 | −55 % |
| A=8 m, T=8 s | 10.49 | 110.7 | 65.6 | −41 % |

**Mean position error, frames 50+** (mean of 5 seeds), and mean CA-mode probability:

| Trajectory | peak a (m/s²) | CV (m) | IMM (m) | change | mean μ_ca |
|---|---|---|---|---|---|
| A=3 m, T=20 s | 0.63 | 0.128 | 0.125 | −3 % | 0.21 |
| A=5 m, T=20 s | 1.05 | 0.130 | 0.128 | −1 % | 0.22 |
| A=5 m, T=12 s | 2.91 | 0.146 | 0.145 | −1 % | 0.40 |
| A=5 m, T=8 s | 6.55 | 0.219 | 0.157 | −28 % | 0.68 |
| A=8 m, T=8 s | 10.49 | 0.333 | 0.185 | −44 % | 0.78 |

**Findings**

1. The IMM's benefit is a function of maneuver severity, with the crossover at roughly **2–3 m/s²** peak lateral acceleration. Above it, fragmentation roughly halves and position error drops by up to 44 %. Below it the two models are indistinguishable in accuracy, and the IMM is marginally worse on ID count (on counts of ~5, within run-to-run spread).
2. `μ_ca` rises monotonically with peak acceleration (0.21 → 0.78), confirming the mode bank is discriminating on the data rather than sitting at its prior.
3. **The simulator's shipped trajectory (A=3 m, T=20 s, peak 0.63 m/s²) sits below the crossover**, so the IMM does not improve the default sim scenario. Real quadcopters maneuver an order of magnitude harder, which is why `motion_model: imm` is still the shipped default. To reproduce the CV baseline exactly, set `motion_model: cv`.

**Root cause of the residual fragmentation (important)**

The motion model was *not* the dominant cause of ID churn at these dynamics. Instrumenting the squared Mahalanobis distance of the true detection against the live track:

| Trajectory | condition | mean d² | p95 d² | true detections gated out |
|---|---|---|---|---|
| A=3, T=20 | noiseless, no dropout | 0.56 | 1.00 | 0 / 399 |
| A=3, T=20 | with measurement noise | 4.36 | 10.40 | 30 / 399 |
| A=5, T=10 | noiseless, no dropout | 8.97 | 18.44 | 146 / 399 |
| A=5, T=10 | with measurement noise | 14.49 | 30.21 | 260 / 399 |

At the sim's default gentleness the *model* contributes d² ≈ 0.6 out of ≈ 4.4 — essentially all gate rejection is measurement noise, and `gate_chi2 = 9.488` is the χ² 95th percentile, so ~5 % of correct detections are rejected **by design**.

The mechanism that turns a single rejection into a lost ID is in track management, not estimation: a detection outside every live track's gate is treated as unmatched and **spawns a competing track at the true target position with a large birth covariance**. That newcomer's gate is wider, so it wins the next frame's greedy association, the original track starves, and it is deleted after `max_misses`. Coverage of the target is essentially total throughout (>99 % of frames have some track on it) — the drone is never lost, only renamed.

Fixing that requires track re-association ("stitching" a revived ID onto a dead track's coasted state) or suppressing spawns from detections that were gated out by a healthy nearby track. Both are track-management changes and are out of scope for this entry.

> **Correction (see the 2026-08-07 duplicate-suppression entry below).** This section describes the churn as purely *sequential* — one track dies, another takes its name. That is real but it is not the whole mechanism. Measuring how many confirmed tracks sit within 1.5 m of the target *in the same frame* shows 33–50 % of frames carry two or more. A third to a half of the ID multiplicity is **concurrent duplication**, not renaming, and the IMM does nothing for it (34 % → 33 % without clutter; 39 % → 50 % with). The spawn-suppression fix named above addresses both.

### 2026-08-07 — Spawn suppression + duplicate track merging, simulation

Same offline replay harness as the entry above (`FigureEight` → `MeasurementModel` → preprocessor filters → `TrackManager`), no ROS. 60 s at 10 Hz, 10 % detection dropout, `max_misses: 8`, `gate_chi2: 9.488`, mean of 5 seeds. "Clutter" runs use the simulator's `clutter_mean_count: 0.5`.

Metrics: **dup** = % of frames with two or more confirmed tracks within 1.5 m of the target (the concurrent-duplicate rate); **IDs** = distinct confirmed track IDs that sat on the target over the run; **life** = lifetime of track 1; **gap** = % of frames with no confirmed track on the target.

**Motivation.** The IMM entry above closed the maneuver-prediction gap but left ID churn essentially unchanged at the simulator's default trajectory. Instrumenting concurrent tracks showed why: `gate_chi2 = 9.488` rejects ~5 % of a target's own detections by construction, and `process_scan` spawned a new track on every unmatched detection — so a momentarily gated-out detection births a twin **on top of the track that just rejected it**.

Two changes, both in `track_manager.py`:

- `spawn_gate_chi2` (30.0) — a second, looser gate. Association asks "is this good enough to fuse?"; initiation asks "is this plausibly the same object?" Only *confirmed* tracks suppress, and only within `MAX_SUPPRESSING_MISSES` (1) consecutive misses.
- `merge_chi2` (25.0 sim / 6.0 hardware) — collapse two tracks whose `[p, v]` states agree, keeping the lower (older) ID.

**Results, `motion_model: imm` (the shipped default):**

| Scenario | dup before → after | IDs before → after | life before → after | gap before → after |
|---|---|---|---|---|
| sim trajectory + clutter | 49.8 % → **0.0 %** | 6.40 → **1.00** | 30.8 s → **59.9 s** | 0.5 % → 0.4 % |
| sim trajectory, no clutter | 33.1 % → **0.0 %** | 4.60 → **1.00** | 26.2 s → **59.9 s** | 0.4 % → 0.4 % |
| sharp turn (A=5, T=8) + clutter | 14.3 % → **0.0 %** | 21.80 → **1.00** | 6.7 s → **59.9 s** | 0.7 % → 0.4 % |
| very sharp (A=6, T=6) + clutter | 15.5 % → **0.2 %** | 76.00 → **8.20** | 1.2 s → 11.5 s | 9.5 % → **1.9 %** |

`motion_model: cv` improves comparably on the first two rows (6.40 → 1.00, 5.00 → 1.00) and less at sharp turns (51.60 → 3.60), since it drifts further through a turn and so falls outside the suppression allowance more often.

**Findings**

1. **One drone now holds one ID for the whole run** at the simulator's shipped trajectory, under both motion models. This was the actual goal; the IMM alone did not achieve it.
2. Coverage did not regress anywhere — the `gap` column is flat or better in every scenario, and improves 5× at the hardest trajectory. Confirmed clutter tracks were unchanged (0.0–0.6 per run either way).
3. **Two regressions were found during tuning and both masked each other**, which is worth recording:
   - Ranking merge survivors by `trace(P)` alone loses the target. A track that has drifted off but been fed for seconds is far more confident than the accurate replacement just spawned on it, which still carries the 25 m²/s² birth velocity covariance — so the drifted estimate won every time. Coverage gaps hit 25 % of frames and IDs minted quadrupled (52 → 204). Fixed by ranking on `(consecutive_misses, trace(P))`: freshness first, confidence only as tie-break.
   - Letting an arbitrarily stale track suppress blocks re-acquisition after a hard turn, sending the `cv` sharp-turn gap to 20 %. Fixed by `MAX_SUPPRESSING_MISSES = 1`. That value is not arbitrary: the twin-spawn case is *exactly* one miss, because the detection that failed the gate is the same one that would start the twin.

   Each fix alone measured as no improvement, because the other defect dominated. Isolating one mechanism at a time was what separated them.
4. **`merge_chi2` is noise-dependent and must not be copied between configs.** Hardware angle sigmas are 2.5× the simulator's, which inflates `S` and shrinks the state distance between any two tracks — a noisier sensor needs a *smaller* chi-square threshold to resolve the same physical separation. Measured end-to-end against two stationary targets at 8 m, holding both IDs:

   | merge_chi2 (hardware noise) | 1.0 m apart | 1.5 m | 2.0 m | dup rate |
   |---|---|---|---|---|
   | 0 (off) | 99.2 % | 100 % | 100 % | 11.4 % |
   | 6 | 60.0 % | **100 %** | 100 % | 0.7 % |
   | 12 | 0 % | 60.0 % | 100 % | 0.1 % |
   | 25 | 0 % | **0 %** | 88.0 % | 0.0 % |

   So `hardware_params.yaml` ships **6.0**, not the simulator's 25.0: it preserves 1.5 m and 2.0 m two-target resolution outright while still cutting duplicates from 11.4 % to 0.7 %. Losing 1.0 m resolution is accepted — at 8 m range with 0.05 rad angle noise, cross-range 1σ is 0.4 m, so a 1 m pair is already marginal. Re-tune after calibration (`docs/calibration_procedure.md`).

5. Both mechanisms accept `0` to disable, restoring the previous behaviour from YAML without a code change — the same rollback story as `motion_model: cv`.

**Not done / next.** Track re-association ("stitching") is still not implemented and is now largely moot at these dynamics, since the duplicates that drove the churn are no longer created. It remains the candidate fix for the very-sharp regime (A=6, T=6), where 8.2 IDs per run still survive. **Not yet validated on hardware or in ROS** — these are offline-replay numbers; the `hardware_params.yaml` value of 6.0 is derived from the simulator's noise model scaled to hardware sigmas, not from recorded bags.
