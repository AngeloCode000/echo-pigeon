# Architecture

Status: outline — not yet finalized.

Derived from `project_plan.md` → "Recommended architecture".

## Pipeline
```text
TI mmWave radar
       │
       ▼
Serial/USB radar driver        (ti_radar_driver)
       │
       ▼
Radar point-cloud ROS 2 node   (ti_radar_driver / radar_interfaces)
       │
       ▼
Clustering and detection filtering  (radar_preprocessor)
       │
       ▼
Extended Kalman filter         (target_tracker)
       │
       ▼
Persistent target track
       │
       ├──► RViz visualization   (track_visualizer)
       ├──► Data logger          (data_logger)
       └──► Camera-fusion node   — later phase
```

## Packages
| Package | Role |
|---|---|
| `radar_interfaces` | Shared message definitions (`RadarDetection`, `TargetTrack`) |
| `radar_simulator` | Synthetic target generator for Phase 0 |
| `ti_radar_driver` | Serial/USB driver for the TI IWR6843ISK-ODS (Phase 1) |
| `radar_preprocessor` | Clutter rejection, thresholding, clustering |
| `target_tracker` | Extended Kalman filter, track management |
| `track_visualizer` | RViz marker publishing |
| `data_logger` | CSV / ROS bag recording |

To be expanded with node graphs, topic names, and QoS decisions as each phase is implemented.
