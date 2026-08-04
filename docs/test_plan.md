# Test Plan

Status: outline — not yet finalized.

## Phase 0 — Simulation
- Unit tests for coordinate conversion (spherical → Cartesian).
- Unit tests for the constant-velocity Kalman filter (converges on a known noisy trajectory).
- End-to-end: synthetic figure-eight trajectory reconstructed by the tracker within a documented error bound.

## Phase 1 — Radar bring-up
Progressive target order (see `project_plan.md` → "Phase 1"):
1. Walking person
2. Moving metal plate
3. Bicycle
4. Drone carried by hand
5. Powered drone on the ground
6. Hovering drone

Success criteria: serial connection established, point-cloud parsed, detections published and visualized in RViz, datasets recorded and repeatable.

## Phase 2 — Drone detection experiments
Controlled range tests at ~3 m, 5 m, 10 m, 15 m, increasing only when detections remain reliable. Record: drone orientation, hover vs. translation, rotor state, elevation, clutter, mounting angle, chirp configuration — with ground truth from measured geometry, telemetry, and synchronized camera.

## Phase 3 — Persistent tracking
Single-target tracking accuracy tests; track confirmation/deletion logic; association under clutter.

To be expanded with concrete pass/fail thresholds once Phase 0 baseline error is measured.
