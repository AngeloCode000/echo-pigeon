# Experiment Results

This document logs results from Phase 1–2 experiments: date, test conditions (range, target, orientation, clutter, chirp config), detection outcome, and links to the corresponding dataset under `datasets/`.

## Log

### 2026-08-06 — Phase 0 sim: track continuity (simulated figure-eight)

Three ~10-minute `sim.launch.py` runs investigating track-id fragmentation on the stock figure-eight profile (center 8/0/3 m, amplitude 3 m, period 20 s, 10 Hz, `detection_probability` 0.9, `clutter_mean_count` 0.5).

| Run | Condition | Real-target track fragments | Duplicate-track time | Median position error |
|---|---|---|---|---|
| `run_20260806_182902` | baseline | 38 | 55% of run | 0.190 m |
| `run_20260806_211442` | + coast gating & stitching | 21 | 35% of run | 0.167 m |
| `run_20260806_214903` | + duplicate merging | **1** | **0%** | 0.166 m |

Outcome: one track id now spans the entire 642 s run (was ~30 fragments); accuracy unchanged. Full write-up in [track_continuity_analysis.md](track_continuity_analysis.md); metrics reproducible via `analysis/scripts/track_continuity_report.py`.
