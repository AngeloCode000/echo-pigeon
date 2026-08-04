# echo-pigeon — Project Plan

## Objective
Build a stationary, short-range radar ground station that detects and tracks a small drone — a sensor platform on the ground, not a payload on the drone.

Deliverables, in order:
1. Detect a small drone using 60 GHz FMCW radar.
2. Estimate its range, azimuth, elevation, and radial velocity.
3. Convert intermittent detections into a persistent target track.
4. Display the track in ROS 2 and RViz.
5. Add a camera for radar-camera association and target classification (later phase).

**v1 scope:** one cooperative drone, short range, controlled area. Small consumer drones have a low radar cross-section — prove the short-range case incrementally before expanding coverage.

## System Architecture
```text
TI mmWave radar
       │
       ▼
Serial/USB radar driver
       │
       ▼
Radar point-cloud ROS 2 node
       │
       ▼
Clustering and detection filtering
       │
       ▼
Extended Kalman filter
       │
       ▼
Persistent target track
       │
       ├──► RViz visualization
       ├──► Data logger
       └──► Camera-fusion node — Phase 4
```
ROS 2 topics carry the continuous sensor streams: radar detections, camera frames, target tracks.

## Hardware

### Radar — TI IWR6843ISK-ODS (~$183.75)
| Spec | Value |
|---|---|
| Frequency | 60–64 GHz FMCW |
| Channels | 3 TX / 4 RX |
| Azimuth FOV | 120° |
| Elevation FOV | 120° |
| Power / output | USB, point-cloud |

Use the **-ODS** variant, not the standard IWR6843ISK. The standard board has longer range but only ~30° elevation FOV — too narrow for a reliably airborne target during initial testing.

### Compute
Use the desktop for all processing through the radar-only phase. Skip the Jetson for now — an Orin Nano only becomes necessary once real-time computer vision is added in Phase 4 (~$249–$399, check current pricing).

### Drone
Use a drone already owned, or an inexpensive sub-250g model with:
- [ ] Propeller guards
- [ ] Stable position hold
- [ ] Bright visual markings
- [ ] Flight logging, if available

Do not fly it initially. Mount the powered-off drone on a pole, cart, or suspended line and move it through the radar field by hand. This isolates "can the radar detect the airframe" from "can it handle motion and prop noise."

### Budget
| Item | Cost |
|---|---:|
| TI IWR6843ISK-ODS | $183.75 |
| Tripod or rigid sensor mount | $25–$60 |
| USB cable and power accessories | $10–$25 |
| Weather-resistant enclosure (later) | $25–$60 |
| Calibration targets and misc. hardware | $20–$40 |
| **Total — initial radar station** | **$265–$370** |

Camera and embedded compute wait until Phase 4.

## Repository Structure
```text
echo-pigeon/
├── README.md
├── docs/
│   ├── system_requirements.md
│   ├── architecture.md
│   ├── test_plan.md
│   ├── calibration_procedure.md
│   └── experiment_results.md
├── ros2_ws/src/
│   ├── radar_interfaces/
│   ├── radar_simulator/
│   ├── ti_radar_driver/
│   ├── radar_preprocessor/
│   ├── target_tracker/
│   ├── track_visualizer/
│   └── data_logger/
├── analysis/
│   ├── notebooks/
│   └── scripts/
├── config/
├── test/
└── datasets/
    └── README.md
```
Do not commit large raw datasets directly into Git — use Git LFS or an external release archive.

## Development Phases

### Phase 0 — Simulation & Software Skeleton
Do this before buying hardware.

Build:
- [ ] ROS 2 workspace
- [ ] Synthetic target generator
- [ ] `RadarDetection` message
- [ ] Constant-velocity target model
- [ ] Kalman filter
- [ ] RViz markers
- [ ] CSV or ROS bag logging
- [ ] Unit tests

Simulated measurement fields:
```text
timestamp
range_m
azimuth_rad
elevation_rad
radial_velocity_mps
signal_strength
```

**Success criteria:** generate a drone flying a circular or figure-eight trajectory with measurement noise and random missed detections; verify the tracker reconstructs the path. This alone is a meaningful GitHub project, before the radar board arrives.

### Phase 1 — Radar Bring-Up
Steps:
1. Connect the TI board by USB.
2. Load a TI demonstration configuration.
3. Receive point-cloud output.
4. Parse the serial data.
5. Publish detections in ROS 2.
6. Visualize detections in RViz.
7. Record repeatable datasets.

Test targets, in order — do not skip ahead:
1. Walking person
2. Moving metal plate
3. Bicycle
4. Drone carried by hand
5. Powered drone on the ground
6. Hovering drone

Do not begin with a drone at 50 meters.

### Phase 2 — Drone Detection Experiments
Test ranges: 3 m → 5 m → 10 m → 15 m. Increase range only when detections stay reliable at the current one.

Vary:
- [ ] Drone facing toward vs. away from radar
- [ ] Hovering vs. translating
- [ ] Rotors stopped vs. spinning
- [ ] Elevation angle
- [ ] Cluttered vs. open background
- [ ] Radar mounting angle
- [ ] Chirp configuration

Record ground truth for every run: measured test geometry, drone telemetry, a synchronized camera, and marked flight positions.

**Question to answer:** under what conditions can this sensor reliably detect and localize this drone? That alone is a legitimate technical result.

### Phase 3 — Persistent Tracking
Pipeline:
1. Remove static clutter.
2. Reject points below a signal threshold.
3. Cluster detections (DBSCAN or spatial gating).
4. Convert spherical measurements to Cartesian coordinates.
5. Initialize tentative tracks.
6. Run an extended Kalman filter.
7. Associate new detections (nearest-neighbor or Mahalanobis-distance gating).
8. Confirm tracks only after repeated detections.
9. Delete tracks after a configurable number of misses.

Start with one target; add multi-target tracking later.

Track state:
```text
x = [position_x, position_y, position_z,
     velocity_x, velocity_y, velocity_z]
```
Outputs: track ID, position, velocity, track age, detection count, miss count, confidence score, covariance.

### Phase 4 — Radar-Camera Fusion
Start only after radar tracking works.

| Sensor | Contributes |
|---|---|
| Camera | Visual confirmation, drone classification, ground-truth labeling, bearing refinement, false-positive rejection |
| Radar | Range, radial velocity, performance without strong lighting, detection independent of visual texture |

A Jetson Orin Nano Super (~$249–$399, check current pricing) becomes defensible here for running object detection locally.

## Engineering Requirements
Write these before implementation:
- [ ] Detect one cooperative small drone within an initial test envelope of 3–10 meters.
- [ ] Update detections at a defined minimum rate.
- [ ] Maintain a track through brief missed detections.
- [ ] Estimate position and velocity with documented error.
- [ ] Record synchronized raw and processed data.
- [ ] Replay recorded datasets without hardware.
- [ ] Run the tracking pipeline from one launch command.
- [ ] Automated tests for coordinate conversion, filtering, association, and state estimation.

Do not declare an ambitious maximum range until data is collected — radar performance depends on configuration, target radar cross-section, orientation, clutter, and environmental conditions.

## Two-Week Sprint
Complete this before ordering the radar.

### Week 1
- [ ] Create the GitHub repository.
- [ ] Install ROS 2.
- [ ] Define the radar detection and target-track messages.
- [ ] Write the synthetic trajectory generator.
- [ ] Publish noisy simulated detections.
- [ ] Display true and measured positions in RViz.

### Week 2
- [ ] Implement Cartesian conversion.
- [ ] Implement a constant-velocity Kalman filter.
- [ ] Add missed detections and clutter to the simulator.
- [ ] Publish persistent tracks.
- [ ] Add unit tests.
- [ ] Write an architecture document and test plan.
- [ ] Produce a short screen recording of the simulated tracker.

## Operating Constraints
- Observational only: sensing, tracking, logging, camera pointing.
- No interference, jamming, takeover, or interception functions.
- Outdoor recreational flights: maintain visual line of sight, comply with airspace restrictions, stay below 400 feet.
- Comply with Remote ID requirements unless operating under a recognized identification area or other applicable exception.

## Next Deliverable
**v0.1 — simulated 3D radar tracking in ROS 2.** Purchase the IWR6843ISK-ODS only after this is functioning.
