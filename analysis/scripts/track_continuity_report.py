#!/usr/bin/env python3
"""Track-continuity metrics for a data_logger run directory.

Quantifies how many distinct track ids the tracker spends on what should be
a single persistent target, and how much of a run has duplicate confirmed
tracks alive on the same object. Produced the numbers in
docs/track_continuity_analysis.md.

    python3 analysis/scripts/track_continuity_report.py ~/echo_pigeon_logs/run_*/

Accuracy is measured as the distance from each track estimate to the nearest
detection in the same frame, NOT against an analytically reconstructed
trajectory. Reconstructing ground truth from the trajectory equations
requires knowing the simulator's internal t at the log's first frame, which
does not hold when the logger subscribes after the simulator has already
published — a phase offset there shows up as a large fake position error.
"""

import argparse
import csv
import math
from collections import defaultdict

import numpy as np


def spherical_to_cartesian(range_m, azimuth_rad, elevation_rad):
    cos_el = math.cos(elevation_rad)
    return np.array([range_m * cos_el * math.cos(azimuth_rad),
                     range_m * cos_el * math.sin(azimuth_rad),
                     range_m * math.sin(elevation_rad)])


def load_run(run_dir):
    detections = defaultdict(list)
    with open(f'{run_dir}/detections.csv') as handle:
        for row in csv.DictReader(handle):
            detections[(row['stamp_sec'], row['stamp_nanosec'])].append(
                spherical_to_cartesian(float(row['range_m']),
                                       float(row['azimuth_rad']),
                                       float(row['elevation_rad'])))

    tracks = defaultdict(list)
    with open(f'{run_dir}/tracks.csv') as handle:
        for row in csv.DictReader(handle):
            tracks[row['track_id']].append(row)
    return detections, tracks


def stamp_of(row):
    return float(row['stamp_sec']) + float(row['stamp_nanosec']) * 1e-9


def summarize(run_dir, min_rows=50):
    detections, tracks = load_run(run_dir)
    all_rows = [row for rows in tracks.values() for row in rows]
    t0 = min(stamp_of(row) for row in all_rows)
    duration = max(stamp_of(row) for row in all_rows) - t0

    confirmed = sum(1 for rows in tracks.values()
                    if max(int(row['detection_count']) for row in rows) >= 3)

    # Long-lived tracks are the ones plausibly riding a real target; each is
    # one "chain link" in a fragmented identity.
    spans = []
    for track_id, rows in tracks.items():
        if len(rows) > min_rows:
            times = [stamp_of(row) - t0 for row in rows]
            spans.append((track_id, min(times), max(times), len(rows)))
    spans.sort(key=lambda s: s[1])

    # All-pairs overlap: two long-lived tracks alive at once means the
    # tracker is holding duplicate identities on one physical object.
    overlap_events, overlap_time = 0, 0.0
    for i in range(len(spans)):
        for j in range(i + 1, len(spans)):
            overlap = min(spans[i][2], spans[j][2]) - max(spans[i][1], spans[j][1])
            if overlap > 0:
                overlap_events += 1
                overlap_time += overlap

    longest_id = max(tracks, key=lambda k: len(tracks[k]))
    errors = []
    for row in tracks[longest_id]:
        key = (row['stamp_sec'], row['stamp_nanosec'])
        if key not in detections:
            continue
        position = np.array([float(row['position_x']), float(row['position_y']),
                             float(row['position_z'])])
        errors.append(min(np.linalg.norm(position - d) for d in detections[key]))
    errors = sorted(errors[20:])  # skip the spawn transient

    return {
        'duration_s': duration,
        'track_ids': len(tracks),
        'confirmed': confirmed,
        'chains': len(spans),
        'overlap_events': overlap_events,
        'overlap_time_s': overlap_time,
        'overlap_fraction': overlap_time / duration if duration else 0.0,
        'longest_id': longest_id,
        'longest_rows': len(tracks[longest_id]),
        'median_err_m': errors[len(errors) // 2] if errors else float('nan'),
        'p90_err_m': errors[int(len(errors) * 0.9)] if errors else float('nan'),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dirs', nargs='+', help='data_logger run directories')
    args = parser.parse_args()

    for run_dir in args.run_dirs:
        run_dir = run_dir.rstrip('/')
        s = summarize(run_dir)
        print(f'\n=== {run_dir} ===')
        print(f"  duration                 {s['duration_s']:.1f} s")
        print(f"  distinct track ids       {s['track_ids']}")
        print(f"  reached CONFIRMED        {s['confirmed']}")
        print(f"  long-lived chains        {s['chains']}")
        print(f"  duplicate-track events   {s['overlap_events']}")
        print(f"  duplicate-track time     {s['overlap_time_s']:.1f} s "
              f"({100 * s['overlap_fraction']:.0f}% of run)")
        print(f"  longest track            id={s['longest_id']} "
              f"({s['longest_rows']} rows)")
        print(f"  position error (nearest detection)  "
              f"median={s['median_err_m']:.3f} m  p90={s['p90_err_m']:.3f} m")


if __name__ == '__main__':
    main()
