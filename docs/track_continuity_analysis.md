# Track Continuity Analysis — Phase 0 Simulation

**Date:** 2026-08-06
**Branch:** `track-continuity-stitching`
**Scope:** `target_tracker` track lifecycle (`track_manager.py`)

Reproduce every number below with:

```bash
python3 analysis/scripts/track_continuity_report.py ~/echo_pigeon_logs/run_*/
```

## Summary

A 10-minute Phase 0 sim run showed the tracker was kinematically accurate but
assigned the single simulated drone roughly **30 different track ids** over the
run instead of one persistent id, with two confirmed tracks riding the same
physical target simultaneously for **55% of the run**. Two changes to
`TrackManager` — coast-scaled gating with dead-track stitching, then
duplicate-track merging — reduced this to **one track id for the entire
642-second run, with zero duplicate-track time**, at no cost to position
accuracy.

| Metric | Baseline | Phase 1 | Phase 2 |
|---|---|---|---|
| Run duration | 622.6 s | 641.1 s | 642.8 s |
| Long-lived track chains (real-target fragments) | 38 | 21 | **1** |
| Duplicate-track events | 41 | 21 | **0** |
| Time with duplicate confirmed tracks | 343.6 s (55%) | 226.6 s (35%) | **0.0 s (0%)** |
| Longest single track | 955 rows (~95 s) | 1169 rows (~117 s) | **6298 rows (642.8 s)** |
| Position error, median | 0.190 m | 0.167 m | **0.166 m** |
| Position error, p90 | 0.600 m | 0.512 m | **0.392 m** |
| Distinct track ids spawned | 928 | 832 | 994 |
| Tracks reaching CONFIRMED | 54 | 33 | **16** |

Runs: baseline `run_20260806_182902`, phase 1 `run_20260806_211442`,
phase 2 `run_20260806_214903`. All used the stock `sim_params.yaml`
figure-eight profile (center 8/0/3 m, amplitude 3 m, period 20 s, 10 Hz,
`detection_probability` 0.9, `clutter_mean_count` 0.5).

## The problem

The simulator flies one drone on a fixed figure-eight. A correct tracker
should therefore hold exactly one confirmed track for the whole run. The
baseline log instead contained 38 separate long-lived tracks whose time
windows chained end-to-end across the run:

```
track 1    [  0.0,  18.8]
track 30   [ 17.8,  57.8]
track 47   [ 26.5,  47.7]   <- alive at the same time as track 30
track 79   [ 48.0,  78.0]
...
track 909  [606.3, 620.3]
```

Note this contradicted the Phase 0 end-to-end pass criterion in
`test_plan.md` ("One confirmed track spans the whole run without an identity
switch"), which had been marked verified — that check was made against a
run far shorter than the failure's onset time, so the fragmentation never
appeared.

Two distinct mechanisms turned out to be responsible:

**1. Dropout-driven fragmentation.** `TrackManager` had no memory of a track
once deleted. A track exceeding `max_misses` consecutive misses was dropped
from `self.tracks`, and the next detection in that same area minted a fresh
id via `_spawn_track`. The constant-velocity EKF in `ekf.py` (white-noise
acceleration, no turn-rate state) predicts poorly through the figure-eight's
center crossing, where the target reverses direction — precisely where a
miss streak long enough to kill the track is most likely.

**2. Duplicate-track fragmentation.** `_spawn_track` had no gating against
*live* tracks. Any measurement that lost the greedy association race in
`_associate` — including one belonging to a target already tracked under
another id — unconditionally became a new track. Two tracks then coexisted,
alternately claiming that target's detections frame to frame, each staying
alive indefinitely. This accounted for the majority of the problem and is
why 55% of the baseline run had overlapping confirmed tracks.

Tuning alone could not close this: the sim already ran looser settings than
the code defaults (`sigma_accel` 3.0 vs. 2.0, `max_misses` 8 vs. 5).

## The fix

All changes are contained to `ros2_ws/src/target_tracker/target_tracker/track_manager.py`,
with parameters threaded through `tracker_node.py` and both
`radar_bringup/config/*.yaml`. `ekf.py` and `coordinates.py` are untouched.

### Phase 1 — coast-scaled gating + track stitching

- **Coast-scaled gate** (`_associate`): the association gate widens with a
  track's consecutive-miss count,
  `gate = gate_chi2 * (1 + coast_gate_growth * consecutive_misses)`.
  A just-hit track keeps the original strict gate, so clutter rejection on
  healthy tracks is unchanged.
- **Tombstones + revival** (`_retire_dead_tracks`, `_try_revive`): a track
  that dies having reached CONFIRMED is kept as a tombstone for
  `stitch_grace_s`. Unmatched measurements are gated against tombstoned EKFs
  coasted forward to the current time; a match revives the track under its
  **original id with its prior hit/miss history and CONFIRMED status intact**,
  rather than spawning a new identity.

New params: `coast_gate_growth` (0.15), `stitch_grace_s` (2.0 s). Both are
disabled in `hardware_params.yaml` — they are tuned against the sim's
uniform-random clutter model, which real hardware clutter will not match.

### Phase 2 — duplicate-track merging

`_merge_duplicates()` runs at the end of every `process_scan`. For each pair
of CONFIRMED tracks it computes a position-only Mahalanobis distance between
their **posterior state estimates**:

```
d² = Δp^T (P_i[:3,:3] + P_j[:3,:3])⁻¹ Δp
```

If `d² <= merge_gate_chi2` the two tracks are statistically indistinguishable
and are treated as one object: the track with fewer hits is dropped
(tie-break: higher/newer id), preserving the older, more established identity.

New param: `merge_gate_chi2` (7.815 — 95th-percentile χ² at 3 DOF). Unlike
the phase-1 params this **is** enabled on hardware: merging only ever
collapses tracks already sitting on top of each other statistically, so it
does not depend on the clutter model.

Position-only comparison is deliberate — a freshly spawned duplicate converges
on position well before its velocity estimate settles (velocity is seeded from
a single doppler return with `initial_velocity_sigma` = 5 m/s), so including
velocity would suppress legitimate merges.

## Results

Phase 1 roughly halved the fragmentation (38 → 21 chains, 55% → 35% duplicate
time) but did not fix it, because it only helps a track that has *already
died* reattach to its old id — it cannot prevent a second track from spawning
onto a still-live target.

Phase 2 eliminated the remaining fragmentation. **Track id 2 spans the full
642.8 s run** (6298 consecutive rows), with no other track exceeding 50 rows
and zero duplicate-track overlap.

Accuracy did not regress — median position error improved slightly across the
three runs (0.190 → 0.167 → 0.166 m), and p90 error improved more (0.600 →
0.392 m), consistent with no longer splitting a target's detections between
competing tracks.

### On the total-id count rising (832 → 994)

Distinct ids spawned went *up* in phase 2, which looks like a regression but
is not. The lifespan breakdown:

| Track lifespan | Baseline | Phase 1 | Phase 2 |
|---|---|---|---|
| ≤ 5 rows | 2 | 1 | 39 |
| 6–50 rows | 888 | 810 | 954 |
| > 50 rows | 38 | 21 | **1** |

Every additional id is a short-lived clutter blip, and the count of tracks
reaching CONFIRMED fell from 54 to 16. Merging now collapses spurious spawns
that drift onto the real target before they can graduate into competing
confirmed tracks, so they die as brief tentative tracks instead. The id
counter is noisier; the track picture is strictly cleaner.

### Multi-target compatibility

Both phases were chosen to survive the Phase 3 multi-target work. Merging
operates on posterior track states, not raw measurements, so it remains valid
if the greedy nearest-Mahalanobis associator is replaced with Hungarian
assignment or JPDA — duplicate tracks can arise under those too. Two
regression tests guard the multi-target case explicitly:
`test_merge_does_not_merge_distinct_targets` (two confirmed tracks 6 m apart
survive untouched) and `test_stitching_does_not_merge_distinct_targets`.

## Methodology note

An early version of this analysis reported a median position error of 2.89 m
for the phase-2 run, implying a severe accuracy regression. That was an error
in the measurement, not the tracker.

Ground truth had been reconstructed by evaluating the analytic figure-eight at
`t = frame_stamp - first_frame_stamp`, which assumes the log's first frame is
the simulator's `t = 0`. That does not hold when `logger_node` subscribes after
`target_generator_node` has begun publishing; the resulting phase offset
appears as a large constant position error. A second artifact came from
sampling the track every 400 rows — almost exactly two 20 s trajectory periods
at 10 Hz — which aliased the samples onto the same trajectory phase and made
the track look stationary at the pattern's center.

All accuracy figures in this document therefore use a phase-independent
metric: the distance from each track estimate to the **nearest detection in
the same frame**, applied identically to all three runs. Analytic ground truth
is still the better metric in principle and is worth adding once the simulator
publishes its `radar/ground_truth` topic into the CSV log alongside detections
(`logger_node` currently subscribes only to `radar/detections` and `tracks`).

## Test coverage added

In `ros2_ws/src/target_tracker/test/test_track_manager.py` (28 tests pass in
the package):

| Test | Guards |
|---|---|
| `test_coast_gate_widens_with_consecutive_misses` | Gate widens only while coasting, not for a just-hit track |
| `test_stitching_reunites_track_after_full_miss_out` | Dead confirmed track revives under its original id, still CONFIRMED |
| `test_stitching_expires_after_grace_window` | No stale revival past `stitch_grace_s` |
| `test_stitching_does_not_merge_distinct_targets` | A different target's detection cannot revive a dead id |
| `test_merge_removes_duplicate_confirmed_track_on_same_target` | Duplicate collapsed, established id kept |
| `test_merge_does_not_merge_distinct_targets` | Genuinely separate targets never merged |
| `test_merge_ignores_tentative_tracks` | A tentative clutter blip cannot kill a confirmed track |

## Follow-ups

- Log `radar/ground_truth` in `data_logger` so accuracy can be measured
  against true target state instead of nearest detection.
- Re-validate the Phase 0 end-to-end criteria in `test_plan.md` over a
  ≥ 10 minute run; the existing checkmarks were earned on a run too short to
  expose this failure.
- The constant-velocity EKF remains the underlying reason tracks lose lock at
  high-curvature crossings. A turn-aware model (coordinated-turn or IMM) is
  the root-cause fix and pairs naturally with the Phase 3 multi-target work;
  it was deliberately deferred here rather than rewriting a validated filter.
- `coast_gate_growth` and `stitch_grace_s` are off on hardware pending
  characterization of real clutter (`calibration_procedure.md`).
