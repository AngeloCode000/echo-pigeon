# Test Plan

## Phase 0 — Simulation (automated + one manual check)

### Unit tests (run on every change)

```bash
cd ros2_ws
python3 -m pytest src/*/test/test_*.py -q --ignore-glob='*copyright*' \
    --ignore-glob='*flake8*' --ignore-glob='*pep257*'
```

| Suite | Covers |
|---|---|
| `target_tracker/test_coordinates.py` | Spherical↔Cartesian round-trip (randomized), angle wrapping, analytic vs. numerical Jacobians, covariance conversion positive-definiteness |
| `target_tracker/test_ekf.py` | Convergence on a noisy constant-velocity target (position error < 0.2 m in the 3–10 m envelope), figure-eight maneuver following (< 0.5 m mean), covariance symmetry/shrinkage, Mahalanobis gate separation |
| `target_tracker/test_track_manager.py` | Confirmation after N hits, deletion after M consecutive misses, survival through brief dropouts, gate rejection of far detections, clutter not stealing a confirmed track, two-target independence |
| `radar_simulator/test_trajectories.py` | Circle/figure-eight geometry, analytic velocity = numerical derivative |
| `radar_simulator/test_measurement_model.py` | Noise statistics, detection-drop rate, Poisson clutter, SNR-vs-range model, seed reproducibility |
| `radar_preprocessor/test_filters.py`, `test_clustering.py` | Threshold filters, DBSCAN labeling, SNR-weighted centroid reduction, noise-point handling |
| `data_logger/test_csv_writer.py` | CSV round-trip, run-directory layout, flush behavior |
| `ti_radar_driver/test_tlv_parser.py` | Byte-exact synthetic frames: chunked feeds, garbage resync, corrupt lengths, unknown TLVs, missing side info |
| `ti_radar_driver/test_cfg_loader.py` | Comment stripping, Done/Error handling, sensorStart deferral |

### End-to-end pass criteria (verified 2026-08-03)

Run `ros2 launch radar_bringup sim.launch.py rviz:=false` for ≥ 40 s, then inspect the newest `~/echo_pigeon_logs/run_*/tracks.csv`:

- [x] One confirmed track (confidence ≥ 0.5) spans the whole run without an identity switch.
- [x] Every confirmed-track position lies inside the commanded trajectory envelope.
- [x] Clutter-spawned tracks either never confirm or die within ~1 s.
- [x] With RViz: the green track sphere visibly follows the red ground-truth cube through dropped detections.

## Phase 1 — Radar bring-up

Progressive target order (see `project_plan.md` — do not skip ahead):

1. Walking person
2. Moving metal plate
3. Bicycle
4. Drone carried by hand
5. Powered drone on the ground
6. Hovering drone (requires `enable_static_clutter_filter: false`)

Pass criteria: serial connection and configuration succeed from `ros2 launch radar_bringup hardware.launch.py` alone; point cloud visible in RViz on a walking person at 2–5 m; a confirmed track forms on that person within 1 s and follows them; a run directory with non-empty `detections.csv` is produced; the run is repeatable after unplug/replug (driver auto-reconnects).

Pre-hardware dry run (verified 2026-08-03): launching with no device logs a clear retry warning every 3 s and never crashes.

## Phase 2 — Drone detection experiments

Controlled range tests at ~3 m, 5 m, 10 m, 15 m, increasing only when detections remain reliable. Vary: drone orientation, hover vs. translation, rotor state, elevation, clutter background, mounting angle, chirp configuration (`cfarFovCfg` range limit in the .cfg must be raised beyond 9 m for the longer tests). Record ground truth: measured geometry, drone telemetry, synchronized camera.

## Phase 3 — Persistent tracking refinement

Single-target accuracy quantified against ground truth from Phase 2 logs; association robustness under real clutter; tune `gate_chi2`, `confirm_hits`, `max_misses`, `sigma_accel` against recorded datasets replayed with `ros2 bag play` — no hardware needed for tuning iterations.
