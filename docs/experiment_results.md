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
