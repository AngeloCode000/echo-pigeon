"""CSV run recorder: one timestamped directory per session.

Layout:
    <output_dir>/run_YYYYmmdd_HHMMSS/
        detections.csv   one row per detection point
        tracks.csv       one row per track state update

Pure module — must not import rclpy so tests run without ROS.
"""

import csv
from datetime import datetime
from pathlib import Path

DETECTION_FIELDS = [
    'stamp_sec', 'stamp_nanosec', 'frame_seq', 'point_index',
    'range_m', 'azimuth_rad', 'elevation_rad',
    'radial_velocity_mps', 'signal_strength',
]

TRACK_FIELDS = [
    'stamp_sec', 'stamp_nanosec', 'track_id',
    'position_x', 'position_y', 'position_z',
    'velocity_x', 'velocity_y', 'velocity_z',
    'track_age_s', 'detection_count', 'miss_count', 'confidence',
    'cov_px', 'cov_py', 'cov_pz', 'cov_vx', 'cov_vy', 'cov_vz',
]


class CsvRunWriter:
    """Writes detection and track rows into a per-run directory."""

    def __init__(self, output_dir, flush_every_n=50, run_name=None):
        if run_name is None:
            run_name = datetime.now().strftime('run_%Y%m%d_%H%M%S')
        self.run_dir = Path(output_dir).expanduser() / run_name
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.flush_every_n = flush_every_n
        self._rows_since_flush = 0

        self._detections_file = open(
            self.run_dir / 'detections.csv', 'w', newline='')
        self._detections = csv.writer(self._detections_file)
        self._detections.writerow(DETECTION_FIELDS)

        self._tracks_file = open(self.run_dir / 'tracks.csv', 'w', newline='')
        self._tracks = csv.writer(self._tracks_file)
        self._tracks.writerow(TRACK_FIELDS)

        self._frame_seq = 0

    def write_scan(self, stamp_sec, stamp_nanosec, detections):
        """Log one radar frame; detections expose the RadarDetection fields."""
        for index, d in enumerate(detections):
            self._detections.writerow([
                stamp_sec, stamp_nanosec, self._frame_seq, index,
                d.range_m, d.azimuth_rad, d.elevation_rad,
                d.radial_velocity_mps, d.signal_strength,
            ])
        self._frame_seq += 1
        self._maybe_flush(len(detections))

    def write_track(self, stamp_sec, stamp_nanosec, track):
        """Log one TargetTrack-shaped object."""
        cov = track.covariance
        self._tracks.writerow([
            stamp_sec, stamp_nanosec, track.track_id,
            track.position_x, track.position_y, track.position_z,
            track.velocity_x, track.velocity_y, track.velocity_z,
            track.track_age.sec + track.track_age.nanosec * 1e-9,
            track.detection_count, track.miss_count, track.confidence,
            cov[0], cov[7], cov[14], cov[21], cov[28], cov[35],
        ])
        self._maybe_flush(1)

    def _maybe_flush(self, rows_written):
        self._rows_since_flush += rows_written
        if self._rows_since_flush >= self.flush_every_n:
            self.flush()

    def flush(self):
        self._detections_file.flush()
        self._tracks_file.flush()
        self._rows_since_flush = 0

    def close(self):
        self.flush()
        self._detections_file.close()
        self._tracks_file.close()
