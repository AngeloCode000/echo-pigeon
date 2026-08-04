"""Detection filters: SNR threshold, range bounds, static-clutter rejection.

Detections are dicts (or objects) exposing range_m, azimuth_rad,
elevation_rad, radial_velocity_mps, signal_strength attributes.
Pure module — must not import rclpy so tests run without ROS.
"""


def snr_filter(detections, min_snr_db):
    """Drop detections below a signal-strength threshold."""
    return [d for d in detections if d.signal_strength >= min_snr_db]


def range_filter(detections, min_range_m, max_range_m):
    """Keep detections inside [min_range_m, max_range_m]."""
    return [d for d in detections if min_range_m <= d.range_m <= max_range_m]


def static_clutter_filter(detections, min_abs_radial_velocity_mps):
    """Drop near-zero-doppler returns (walls, ground, furniture).

    WARNING: this also removes a perfectly stationary hovering target —
    keep it toggleable and disable it for hover experiments
    (project_plan.md Phase 2).
    """
    return [d for d in detections
            if abs(d.radial_velocity_mps) >= min_abs_radial_velocity_mps]
