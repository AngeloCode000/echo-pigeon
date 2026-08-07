# Architecture

## Node graph

```text
 target_generator_node          radar_driver_node
 (radar_simulator,              (ti_radar_driver,
  Phase 0 sim)                   Phase 1 hardware)
        │                              │
        │   RadarScan                  │   RadarScan
        └────────────┬─────────────────┘
                     ▼
             radar/detections ─────────────────► logger_node
                     │                           (data_logger → CSV)
                     ▼
             preprocessor_node
             (SNR / range / zero-doppler filters, DBSCAN)
                     │
                     ▼
        radar/detections_filtered
                     │
                     ▼
              tracker_node
              (EKF + track management)
                     │
                     ▼
                  tracks ──────────────────────► logger_node
                     │
                     ▼
             visualizer_node
                     │
                     ▼
              track_markers ───► RViz
```

The simulator additionally publishes `radar/ground_truth` (`geometry_msgs/PointStamped`), which the visualizer draws as a red cube for visual error checking.

## Topics

| Topic | Type | Producer | Notes |
|---|---|---|---|
| `radar/detections` | `radar_interfaces/RadarScan` | simulator **or** driver | One message per radar frame; an **empty** `detections[]` is a valid frame and is the tracker's missed-detection signal |
| `radar/detections_filtered` | `radar_interfaces/RadarScan` | preprocessor | Same contract; empty frames pass through |
| `radar/ground_truth` | `geometry_msgs/PointStamped` | simulator only | True target position |
| `tracks` | `radar_interfaces/TargetTrack` | tracker | One message per live track per frame |
| `track_markers` | `visualization_msgs/MarkerArray` | visualizer | Spheres, velocity arrows, labels, covariance ellipsoids, detection points |

## Messages (`radar_interfaces`)

- `RadarDetection` — one point: `range_m`, `azimuth_rad`, `elevation_rad`, `radial_velocity_mps`, `signal_strength`.
- `RadarScan` — `Header` + `RadarDetection[]`; the frame-level unit every pipeline stage consumes.
- `TargetTrack` — `track_id`, Cartesian position/velocity, row-major 6×6 covariance, `track_age`, `detection_count`, `miss_count`, `confidence`.

## Coordinate conventions (REP-103)

Frame `radar_link`: **x forward** along boresight, **y left**, **z up**.

- azimuth = `atan2(y, x)` — positive left
- elevation = `atan2(z, hypot(x, y))` — positive up
- radial velocity = `p·v / |p|` — positive receding

The TI demo firmware reports points as x-right / y-boresight / z-up; the driver remaps with `ros = (ti_y, −ti_x, ti_z)`.

## Packages

| Package | Role | Key modules |
|---|---|---|
| `radar_interfaces` | Message definitions | `RadarDetection`, `RadarScan`, `TargetTrack` |
| `radar_simulator` | Phase 0 synthetic target | `trajectories.py` (circle, Gerono figure-eight, analytic velocity), `measurement_model.py` (noise, drops, Poisson clutter, SNR model) |
| `ti_radar_driver` | Phase 1 serial driver | `tlv_parser.py` (streaming mmWave SDK 3.x TLV parser), `cfg_loader.py` (CLI-UART config), `config/iwr6843_ods_default.cfg` |
| `radar_preprocessor` | Detection filtering | `filters.py` (SNR / range / zero-doppler), `clustering.py` (numpy DBSCAN + centroid reduction) |
| `target_tracker` | State estimation | `coordinates.py` (spherical↔Cartesian + Jacobians), `ekf.py` (constant-velocity EKF, Joseph-form updates), `motion_models.py` (9D constant-velocity / constant-acceleration F and Q), `imm.py` (two-mode interacting-multiple-model filter), `track_manager.py` (Mahalanobis gating, confirm/delete lifecycle) |
| `track_visualizer` | RViz markers | color-coded confirmed/tentative, velocity arrows, 2σ covariance ellipsoids |
| `data_logger` | Recording | `csv_writer.py` (timestamped run directories) |
| `radar_bringup` | Orchestration | `sim.launch.py`, `hardware.launch.py`, param YAMLs, RViz config |

Design rule: **algorithm code never imports rclpy.** The pure modules above are unit-tested with plain pytest; nodes are thin parameter-and-plumbing wrappers. `target_tracker/coordinates.py` is the single home of coordinate math — the simulator, preprocessor, driver, and visualizer import it.

## Tracker pipeline (per frame)

1. Predict every track forward by the inter-frame dt (clamped by `max_coast_dt_s`).
2. Gate all track–detection pairs by squared Mahalanobis distance (`gate_chi2`, default 9.488 = χ² 95th percentile, 4 DOF).
3. Greedy nearest-first association; matched tracks get an EKF update.
4. Unmatched tracks accumulate misses; unmatched detections spawn tentative tracks (velocity seeded along the line of sight from measured doppler).
5. Tentative → confirmed after `confirm_hits` hits; deleted after `max_misses` consecutive misses.

## Motion models

The `motion_model` parameter selects what each track runs.

**`cv`** — `ekf.ConstantVelocityEKF`. State `[px, py, pz, vx, vy, vz]`, constant-velocity model with white-noise-acceleration process noise (`sigma_accel`), nonlinear spherical measurement `[r, az, el, v_r]` with analytic Jacobian, angle-wrapped innovations, Joseph-form covariance update.

**`imm`** (default) — `imm.IMMFilter`, a two-mode interacting-multiple-model filter over a shared 9D state `[p, v, a]`. One mode is constant-velocity (acceleration pinned at zero by a zeroed `F` block), the other constant-acceleration with Wiener-process jerk noise. Both modes are linear, so neither needs a process Jacobian and mixing is a plain weighted sum. Each frame: mix the modes by the Markov transition matrix, predict each, and on a detection update each and re-weight by its Gaussian likelihood (computed in log space); the merged moment-matched Gaussian is what gets gated and published.

Gating uses the **merged** state and covariance. Because the merged covariance carries the between-mode spread, the gate widens only while the modes disagree — i.e. during a maneuver — and stays tight on straight legs, so `gate_chi2` needs no re-tuning.

Both filters expose the same surface (`x`, `P`, `predict`, `mahalanobis`, `update`), so `TrackManager` holds either behind `Track.ekf`. The IMM's acceleration state is internal: `tracker_node` publishes `x[:6]` and `P[:6, :6]`, keeping `TargetTrack`'s pinned 6D layout.

The IMM's payoff scales with maneuver severity — roughly break-even below ~2 m/s² of lateral acceleration, halving track fragmentation and cutting position error up to 44 % above ~6 m/s². Measurements in `docs/experiment_results.md`.

## QoS

All topics use the default reliable QoS with depth 10. Sensor-style best-effort QoS is a deliberate non-choice for now: at 10 Hz frame rate on one host, reliability costs nothing and simplifies `ros2 bag` replay.
