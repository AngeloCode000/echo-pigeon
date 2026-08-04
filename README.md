# echo-pigeon

A ground-based radar station for detecting and tracking small drones. Uses 60 GHz FMCW radar (TI IWR6843ISK-ODS) to estimate range, azimuth, elevation, and velocity, then fuses detections into persistent tracks with an extended Kalman filter. Built in ROS 2 Humble with RViz visualization; radar-camera fusion planned for a later phase.

## Quick start

```bash
sudo apt install python3-numpy python3-serial ros-humble-desktop \
    python3-colcon-common-extensions
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Phase 0 — simulated drone, no hardware needed:
ros2 launch radar_bringup sim.launch.py

# Phase 1 — with a TI IWR6843ISK-ODS connected:
ros2 launch radar_bringup hardware.launch.py
```

Full instructions — including WSL2 USB passthrough, radar flashing, and troubleshooting — are in **[docs/setup_guide.md](docs/setup_guide.md)**.

## Status

- **Phase 0 (simulation) — complete.** A simulated drone flying a figure-eight with measurement noise, dropped detections, and clutter is tracked end-to-end and displayed in RViz from one launch command.
- **Phase 1 (radar bring-up) — software ready.** The TI serial driver, TLV parser, and a known-good ODS chirp configuration are implemented and unit-tested against synthetic byte streams; awaiting hardware.

## Documentation

| Document | Contents |
|---|---|
| [docs/project_plan.md](docs/project_plan.md) | Goals, phases, hardware selection, budget |
| [docs/setup_guide.md](docs/setup_guide.md) | Clone-to-running setup (Ubuntu + WSL2) |
| [docs/architecture.md](docs/architecture.md) | Node graph, topics, message definitions |
| [docs/test_plan.md](docs/test_plan.md) | Per-phase test procedures and pass criteria |
| [docs/calibration_procedure.md](docs/calibration_procedure.md) | Hardware calibration (Phase 1+) |
| [docs/experiment_results.md](docs/experiment_results.md) | Recorded experiment outcomes |
