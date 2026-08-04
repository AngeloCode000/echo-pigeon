"""Radar measurement model: truth -> noisy spherical detections plus clutter.

Pure numpy module — must not import rclpy so tests run without ROS.
"""

import numpy as np

from target_tracker.coordinates import cartesian_to_spherical, radial_velocity


class Measurement:
    """One simulated detection in the RadarDetection field layout."""

    __slots__ = ('range_m', 'azimuth_rad', 'elevation_rad',
                 'radial_velocity_mps', 'signal_strength')

    def __init__(self, range_m, azimuth_rad, elevation_rad,
                 radial_velocity_mps, signal_strength):
        self.range_m = float(range_m)
        self.azimuth_rad = float(azimuth_rad)
        self.elevation_rad = float(elevation_rad)
        self.radial_velocity_mps = float(radial_velocity_mps)
        self.signal_strength = float(signal_strength)


class MeasurementModel:
    """Generates one frame of measurements per call from true target state."""

    def __init__(self,
                 sigma_range_m=0.1,
                 sigma_azimuth_rad=0.02,
                 sigma_elevation_rad=0.02,
                 sigma_doppler_mps=0.1,
                 detection_probability=0.95,
                 clutter_mean_count=0.5,
                 clutter_range_bounds=(1.0, 15.0),
                 clutter_azimuth_bounds=(-1.0, 1.0),
                 clutter_elevation_bounds=(-0.3, 0.8),
                 base_snr_db=25.0,
                 reference_range_m=5.0,
                 snr_noise_db=2.0,
                 clutter_snr_db=8.0,
                 seed=None):
        self.sigmas = np.array([sigma_range_m, sigma_azimuth_rad,
                                sigma_elevation_rad, sigma_doppler_mps])
        self.detection_probability = detection_probability
        self.clutter_mean_count = clutter_mean_count
        self.clutter_range_bounds = clutter_range_bounds
        self.clutter_azimuth_bounds = clutter_azimuth_bounds
        self.clutter_elevation_bounds = clutter_elevation_bounds
        self.base_snr_db = base_snr_db
        self.reference_range_m = reference_range_m
        self.snr_noise_db = snr_noise_db
        self.clutter_snr_db = clutter_snr_db
        self.rng = np.random.default_rng(seed)

    def target_measurement(self, position, velocity):
        """Noisy detection of the true target, or None if dropped this frame."""
        if self.rng.random() > self.detection_probability:
            return None
        r, az, el = cartesian_to_spherical(*position)
        truth = np.array([r, az, el, radial_velocity(position, velocity)])
        noisy = truth + self.rng.normal(0.0, self.sigmas)
        noisy[0] = max(noisy[0], 0.01)
        snr = (self.base_snr_db
               - 20.0 * np.log10(max(r, 0.1) / self.reference_range_m)
               + self.rng.normal(0.0, self.snr_noise_db))
        return Measurement(*noisy, snr)

    def clutter_measurements(self):
        """Poisson-count clutter points, near-zero doppler, low SNR."""
        count = self.rng.poisson(self.clutter_mean_count)
        clutter = []
        for _ in range(count):
            clutter.append(Measurement(
                self.rng.uniform(*self.clutter_range_bounds),
                self.rng.uniform(*self.clutter_azimuth_bounds),
                self.rng.uniform(*self.clutter_elevation_bounds),
                self.rng.normal(0.0, 0.05),
                self.clutter_snr_db + self.rng.normal(0.0, self.snr_noise_db),
            ))
        return clutter

    def frame(self, position, velocity):
        """One full radar frame: target detection (unless dropped) + clutter."""
        detections = []
        target = self.target_measurement(position, velocity)
        if target is not None:
            detections.append(target)
        detections.extend(self.clutter_measurements())
        return detections
