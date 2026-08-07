"""Track lifecycle management: initiation, association, confirmation, deletion.

Single-target focused but structured as a track list so multi-target
association (project_plan.md Phase 3+) drops in without rework.
Pure numpy module — must not import rclpy so tests run without ROS.
"""

from enum import Enum

import numpy as np

from target_tracker.coordinates import (
    spherical_covariance_to_cartesian,
    spherical_to_cartesian,
)
from target_tracker.ekf import ConstantVelocityEKF
from target_tracker.imm import IMMFilter

MOTION_MODELS = ('cv', 'imm')


class TrackState(Enum):
    TENTATIVE = 'tentative'
    CONFIRMED = 'confirmed'


class Track:
    """One tracked target: an EKF plus hit/miss bookkeeping."""

    def __init__(self, track_id, ekf, start_time):
        self.track_id = track_id
        self.ekf = ekf
        self.state = TrackState.TENTATIVE
        self.hits = 1
        self.misses = 0
        self.consecutive_misses = 0
        self.start_time = start_time

    @property
    def confidence(self):
        return self.hits / (self.hits + self.misses)

    def age(self, now):
        return now - self.start_time


class TrackManager:
    """Turns per-frame detection lists into persistent tracks.

    Detections are (range_m, azimuth_rad, elevation_rad, radial_velocity_mps)
    tuples. Call process_scan once per radar frame — including empty frames,
    which count as misses for every live track.
    """

    def __init__(self, sigma_accel=2.0,
                 measurement_noise_diag=(0.1, 0.02, 0.02, 0.1),
                 gate_chi2=9.488, confirm_hits=3, max_misses=5,
                 initial_velocity_sigma=5.0, max_coast_dt_s=1.0,
                 motion_model='imm', initial_accel_sigma=2.0,
                 imm_sigma_accel=2.0, imm_sigma_jerk=4.0,
                 imm_p_cv_to_ca=0.05, imm_p_ca_to_cv=0.10):
        if motion_model not in MOTION_MODELS:
            raise ValueError(f"unknown motion model '{motion_model}' "
                             "(expected 'cv' or 'imm')")
        self.sigma_accel = sigma_accel
        self.measurement_noise_diag = np.asarray(measurement_noise_diag, dtype=float)
        self.gate_chi2 = gate_chi2
        self.confirm_hits = confirm_hits
        self.max_misses = max_misses
        self.initial_velocity_sigma = initial_velocity_sigma
        self.max_coast_dt_s = max_coast_dt_s
        self.motion_model = motion_model
        self.initial_accel_sigma = initial_accel_sigma
        # The IMM's CV mode gets its own process noise rather than reusing
        # sigma_accel: the YAMLs inflate sigma_accel to 3.0 to let a single CV
        # filter survive maneuvers, and feeding that to the IMM's CV mode would
        # blur it into a second CA mode and destroy the mode discrimination the
        # whole filter depends on.
        self.imm_sigma_accel = imm_sigma_accel
        self.imm_sigma_jerk = imm_sigma_jerk
        self.imm_p_cv_to_ca = imm_p_cv_to_ca
        self.imm_p_ca_to_cv = imm_p_ca_to_cv
        self.tracks = []
        self._next_id = 1
        self._last_time = None

    def process_scan(self, stamp_s, detections):
        """Advance all tracks to stamp_s and fuse one frame of detections."""
        dt = 0.0
        if self._last_time is not None:
            dt = min(max(stamp_s - self._last_time, 0.0), self.max_coast_dt_s)
        self._last_time = stamp_s

        for track in self.tracks:
            if dt > 0.0:
                track.ekf.predict(dt)

        measurements = [np.asarray(d, dtype=float) for d in detections]
        unmatched = self._associate(measurements)

        for measurement in unmatched:
            self._spawn_track(measurement, stamp_s)

        self.tracks = [t for t in self.tracks
                       if t.consecutive_misses < self.max_misses]
        return self.tracks

    def _associate(self, measurements):
        """Greedy nearest-Mahalanobis association. Returns unmatched measurements."""
        pairs = []
        for ti, track in enumerate(self.tracks):
            for mi, z in enumerate(measurements):
                d2 = track.ekf.mahalanobis(z)
                if d2 <= self.gate_chi2:
                    pairs.append((d2, ti, mi))
        pairs.sort(key=lambda p: p[0])

        used_tracks, used_meas = set(), set()
        for d2, ti, mi in pairs:
            if ti in used_tracks or mi in used_meas:
                continue
            used_tracks.add(ti)
            used_meas.add(mi)
            track = self.tracks[ti]
            track.ekf.update(measurements[mi])
            track.hits += 1
            track.consecutive_misses = 0
            if track.state is TrackState.TENTATIVE and track.hits >= self.confirm_hits:
                track.state = TrackState.CONFIRMED

        for ti, track in enumerate(self.tracks):
            if ti not in used_tracks:
                track.misses += 1
                track.consecutive_misses += 1

        return [z for mi, z in enumerate(measurements) if mi not in used_meas]

    def _spawn_track(self, measurement, stamp_s):
        range_m, azimuth, elevation, v_radial = measurement
        position = spherical_to_cartesian(range_m, azimuth, elevation)
        # Seed velocity along the line of sight from the measured doppler;
        # the cross-range components are unknown, so inflate their covariance.
        los = position / max(np.linalg.norm(position), 1e-9)
        velocity = v_radial * los

        position_covariance = spherical_covariance_to_cartesian(
            range_m, azimuth, elevation,
            self.measurement_noise_diag[0],
            self.measurement_noise_diag[1],
            self.measurement_noise_diag[2])

        if self.motion_model == 'cv':
            covariance = np.zeros((6, 6))
            covariance[:3, :3] = position_covariance
            covariance[3:, 3:] = self.initial_velocity_sigma ** 2 * np.eye(3)
            ekf = ConstantVelocityEKF(
                initial_state=np.concatenate([position, velocity]),
                initial_covariance=covariance,
                sigma_accel=self.sigma_accel,
                measurement_noise_diag=self.measurement_noise_diag)
        else:
            # Acceleration is wholly unobserved at birth, so seed it at zero
            # with a deliberately loose covariance and let the bank learn it.
            covariance = np.zeros((9, 9))
            covariance[:3, :3] = position_covariance
            covariance[3:6, 3:6] = self.initial_velocity_sigma ** 2 * np.eye(3)
            covariance[6:, 6:] = self.initial_accel_sigma ** 2 * np.eye(3)
            ekf = IMMFilter(
                initial_state=np.concatenate([position, velocity, np.zeros(3)]),
                initial_covariance=covariance,
                measurement_noise_diag=self.measurement_noise_diag,
                sigma_accel=self.imm_sigma_accel,
                sigma_jerk=self.imm_sigma_jerk,
                p_cv_to_ca=self.imm_p_cv_to_ca,
                p_ca_to_cv=self.imm_p_ca_to_cv)

        self.tracks.append(Track(self._next_id, ekf, stamp_s))
        self._next_id += 1
