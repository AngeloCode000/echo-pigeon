# echo-pigeon — Project Plan
## Project objective
Build a **short-range radar drone detection and tracking ground station** — a stationary sensor platform, not radar mounted on the drone itself. This is cheaper, safer, easier to validate, and more relevant to systems-integration work than a flying sensor package.
The system will:
1. Detect a small drone using 60 GHz FMCW radar.
2. Estimate its range, azimuth, elevation, and radial velocity.
3. Convert intermittent detections into a persistent target track.
4. Display the track in ROS 2 and RViz.
5. Later add a camera for radar-camera association and target classification.
The first version tracks **one cooperative drone at short range** in a controlled area. Small consumer drones have limited radar cross-section, so the system should be proven incrementally rather than assuming the first prototype will cover a wide area.
## Recommended architecture
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
       └──► Camera-fusion node — later phase
```
ROS 2 topics are intended for continuous sensor streams, which makes them appropriate for radar detections, camera frames, and target tracks.
## Hardware recommendation
### Radar: TI IWR6843ISK-ODS
Start with the **IWR6843ISK-ODS** (~$183.75 from TI). It provides:
- 60–64 GHz FMCW radar
- 3 transmit and 4 receive channels
- 120° azimuth field of view
- 120° elevation field of view
- USB power and point-cloud output

The wide elevation coverage matters because the target is airborne. The standard IWR6843ISK has a longer-range antenna pattern but only ~30° elevation coverage — potentially useful later, but less forgiving during initial testing.
### Computer: use your desktop first
Do **not** buy a Jetson immediately. Connect the radar to your existing computer and do the processing there. Migrate to an embedded computer only once the pipeline works. A Jetson Orin Nano becomes useful when adding real-time computer vision, but is unnecessary expense during the radar-only phase.
### Drone
Use a drone you already own, or an inexpensive sub-250-gram model with:
- Propeller guards
- Stable position hold
- Bright visual markings
- Flight logging, if available

Initially avoid flying altogether — attach the powered-off drone to a pole, cart, or suspended line and move it through the radar field. This isolates "can the radar detect the airframe" from "can it handle motion and prop noise."
### Initial budget
| Item | Expected cost |
|---|---:|
| TI IWR6843ISK-ODS | $183.75 |
| Tripod or rigid sensor mount | $25–$60 |
| USB cable and power accessories | $10–$25 |
| Weather-resistant enclosure later | $25–$60 |
| Calibration targets and miscellaneous hardware | $20–$40 |
| **Initial radar-station total** | **approximately $265–$370** |

A camera and embedded computer can wait until Phase 4.
## Project phases
### Phase 0 — Simulation and software skeleton
Before buying hardware, create the repository and simulate radar detections.
Build:
- ROS 2 workspace
- Synthetic target generator
- `RadarDetection` message
- Constant-velocity target model
- Kalman filter
- RViz markers
- CSV or ROS bag logging
- Unit tests

Simulated measurement fields:
```text
timestamp
range_m
azimuth_rad
elevation_rad
radial_velocity_mps
signal_strength
```
Generate a drone flying a circular or figure-eight trajectory, add measurement noise and random missed detections, and verify that the tracker reconstructs the path. This gives a meaningful GitHub project immediately, even before the board arrives.
### Phase 1 — Radar bring-up
Success criteria:
- Connect the TI board by USB.
- Load a TI demonstration configuration.
- Receive point-cloud output.
- Parse the serial data.
- Publish detections in ROS 2.
- Visualize detections in RViz.
- Record repeatable datasets.

Start with easy targets, in order:
1. Walking person
2. Moving metal plate
3. Bicycle
4. Drone carried by hand
5. Powered drone on the ground
6. Hovering drone

Do not begin with a drone at 50 meters.
### Phase 2 — Drone detection experiments
Run controlled tests at approximately 3 m, 5 m, 10 m, 15 m — increasing range only when detections remain reliable.
Test variables:
- Drone facing toward and away from radar
- Hovering versus translating
- Rotors stopped versus spinning
- Different elevations
- Cluttered versus open backgrounds
- Radar mounting angles
- Different radar chirp configurations

Record every experiment with ground truth from measured test geometry, drone telemetry, a synchronized camera, and marked flight positions.
The engineering question to answer: **Under what conditions can this sensor reliably detect and localize this particular drone?** That is already a legitimate technical result.
### Phase 3 — Persistent tracking
Raw point clouds are not tracks. Build the tracking pipeline:
1. Remove static clutter.
2. Reject points below a signal threshold.
3. Cluster detections using DBSCAN or spatial gating.
4. Convert spherical measurements into Cartesian coordinates.
5. Initialize tentative tracks.
6. Use an extended Kalman filter.
7. Associate new detections using nearest-neighbor or Mahalanobis-distance gating.
8. Confirm tracks only after repeated detections.
9. Delete tracks after a configurable number of misses.

Start with one target; add multi-target tracking later.
Track state:
```text
x = [position_x, position_y, position_z,
     velocity_x, velocity_y, velocity_z]
```
Outputs: track ID, position, velocity, track age, detection count, miss count, confidence score, covariance.
### Phase 4 — Radar-camera fusion
Only after radar tracking works, add a camera.
The camera contributes: visual confirmation, drone classification, ground-truth labeling, bearing refinement, false-positive rejection.
The radar contributes: range, radial velocity, performance without strong lighting, detection independent of visual texture.
At this point, an NVIDIA Jetson becomes defensible for running object detection locally (Jetson Orin Nano Super is roughly $249–$399 — check current pricing before purchase).
## Repository structure
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
Do not commit large raw datasets directly into Git unless using Git LFS or an external release archive.
## Engineering requirements
Write these before implementation:
- Detect one cooperative small drone within an initial test envelope of 3–10 meters.
- Update detections at a defined minimum rate.
- Maintain a track through brief missed detections.
- Estimate position and velocity with documented error.
- Record synchronized raw and processed data.
- Replay recorded datasets without hardware.
- Run the tracking pipeline from one launch command.
- Include automated tests for coordinate conversion, filtering, association, and state estimation.

Avoid declaring an ambitious maximum range until data is collected. Radar performance depends on configuration, target radar cross-section, orientation, clutter, and environmental conditions.
## First two-week sprint
### Week 1
- Create the GitHub repository.
- Install ROS 2.
- Define the radar detection and target-track messages.
- Write the synthetic trajectory generator.
- Publish noisy simulated detections.
- Display the true and measured positions in RViz.

### Week 2
- Implement Cartesian conversion.
- Implement a constant-velocity Kalman filter.
- Add missed detections and clutter to the simulator.
- Publish persistent tracks.
- Add unit tests.
- Write an architecture document and test plan.
- Produce a short screen recording of the simulated tracker.

Only after completing that sprint should the radar be ordered.
## Legal and flight boundary
Keep the project observational: sensing, tracking, logging, and camera pointing. Do not add interference, jamming, takeover, or interception functions.
For outdoor recreational flights, FAA guidance requires visual line of sight, compliance with airspace restrictions, and generally remaining below 400 feet. Drones required to use Remote ID must comply unless operated under an applicable exception such as within a recognized identification area.
## Immediate next deliverable
**Version 0.1: simulated 3D radar tracking in ROS 2** — followed by purchasing the IWR6843ISK-ODS once that pipeline is functioning.
