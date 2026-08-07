"""Track lifecycle management: initiation, association, confirmation, deletion.

Single-target focused but structured as a track list so multi-target
association (project_plan.md Phase 3+) drops in without rework.
Pure numpy module — must not import rclpy so tests run without ROS.
"""

import copy
from enum import Enum

import numpy as np

from target_tracker.coordinates import (
    spherical_covariance_to_cartesian,
    spherical_to_cartesian,
)
from target_tracker.ekf import ConstantVelocityEKF


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
                 coast_gate_growth=0.15, stitch_grace_s=2.0,
                 merge_gate_chi2=7.815):
        self.sigma_accel = sigma_accel
        self.measurement_noise_diag = np.asarray(measurement_noise_diag, dtype=float)
        self.gate_chi2 = gate_chi2
        self.confirm_hits = confirm_hits
        self.max_misses = max_misses
        self.initial_velocity_sigma = initial_velocity_sigma
        self.max_coast_dt_s = max_coast_dt_s
        # Gate widening per consecutive miss, applied only while a track is
        # coasting (never to a just-hit track), so maneuvering targets get
        # slack without loosening clutter rejection on confirmed tracks.
        self.coast_gate_growth = coast_gate_growth
        # How long a dead (but previously confirmed) track's last state is
        # kept around to be re-matched under its original id instead of
        # spawning a fresh one — see _try_revive.
        self.stitch_grace_s = stitch_grace_s
        # Gate (3 DOF, position only) below which two CONFIRMED tracks are
        # considered statistically the same object and merged — see
        # _merge_duplicates.
        self.merge_gate_chi2 = merge_gate_chi2
        self.tracks = []
        self._tombstones = []
        self._next_id = 1
        self._last_time = None

    def process_scan(self, stamp_s, detections):
        """Advance all tracks to stamp_s and fuse one frame of detections."""
        dt = 0.0
        if self._last_time is not None:
            dt = min(max(stamp_s - self._last_time, 0.0), self.max_coast_dt_s)
        self._last_time = stamp_s

        self._expire_tombstones(stamp_s)

        for track in self.tracks:
            if dt > 0.0:
                track.ekf.predict(dt)

        measurements = [np.asarray(d, dtype=float) for d in detections]
        unmatched = self._associate(measurements)
        unmatched = [z for z in unmatched if not self._try_revive(z, stamp_s)]

        for measurement in unmatched:
            self._spawn_track(measurement, stamp_s)

        self._retire_dead_tracks(stamp_s)
        self._merge_duplicates()
        return self.tracks

    def _expire_tombstones(self, stamp_s):
        self._tombstones = [
            tomb for tomb in self._tombstones
            if stamp_s - tomb['death_time'] <= self.stitch_grace_s]

    def _retire_dead_tracks(self, stamp_s):
        """Drop tracks past max_misses, tombstoning ones worth reviving."""
        survivors = []
        for track in self.tracks:
            if track.consecutive_misses < self.max_misses:
                survivors.append(track)
            elif track.state is TrackState.CONFIRMED:
                self._tombstones.append({
                    'track_id': track.track_id,
                    'ekf': track.ekf,
                    'death_time': stamp_s,
                    'hits': track.hits,
                    'misses': track.misses,
                    'state': track.state,
                    'start_time': track.start_time,
                })
        self.tracks = survivors

    def _merge_duplicates(self):
        """Collapse CONFIRMED tracks whose position estimates coincide.

        A spurious spawn can converge onto a target that's already tracked
        under a different id, leaving two live tracks on one physical
        object. This compares posterior state estimates (not raw
        measurements), so it stays valid regardless of how association
        itself works — it only removes tracks that are already
        statistically indistinguishable from one another.
        """
        confirmed = [t for t in self.tracks if t.state is TrackState.CONFIRMED]
        to_drop = set()
        for i in range(len(confirmed)):
            track_i = confirmed[i]
            if track_i.track_id in to_drop:
                continue
            for j in range(i + 1, len(confirmed)):
                track_j = confirmed[j]
                if track_j.track_id in to_drop:
                    continue
                delta = track_i.ekf.x[:3] - track_j.ekf.x[:3]
                combined_p = track_i.ekf.P[:3, :3] + track_j.ekf.P[:3, :3]
                d2 = float(delta @ np.linalg.solve(combined_p, delta))
                if d2 <= self.merge_gate_chi2:
                    if track_i.hits != track_j.hits:
                        loser = min(track_i, track_j, key=lambda t: t.hits)
                    else:
                        loser = max(track_i, track_j, key=lambda t: t.track_id)
                    to_drop.add(loser.track_id)

        if to_drop:
            self.tracks = [t for t in self.tracks if t.track_id not in to_drop]

    def _try_revive(self, z, stamp_s):
        """Match z against tombstoned tracks coasted forward to stamp_s.

        On a match, pops the tombstone and re-adds a live track under the
        original id with its prior hit/miss history and state restored, so
        a previously confirmed track resumes confirmed immediately rather
        than re-earning confirmation from scratch.
        """
        best_idx, best_ekf, best_d2 = None, None, None
        for idx, tomb in enumerate(self._tombstones):
            dt = max(stamp_s - tomb['death_time'], 0.0)
            ekf_copy = copy.deepcopy(tomb['ekf'])
            if dt > 0.0:
                ekf_copy.predict(dt)
            d2 = ekf_copy.mahalanobis(z)
            if d2 <= self.gate_chi2 and (best_d2 is None or d2 < best_d2):
                best_idx, best_ekf, best_d2 = idx, ekf_copy, d2

        if best_idx is None:
            return False

        tomb = self._tombstones.pop(best_idx)
        best_ekf.update(z)
        track = Track(tomb['track_id'], best_ekf, tomb['start_time'])
        track.state = tomb['state']
        track.hits = tomb['hits'] + 1
        track.misses = tomb['misses']
        track.consecutive_misses = 0
        self.tracks.append(track)
        return True

    def _associate(self, measurements):
        """Greedy nearest-Mahalanobis association. Returns unmatched measurements."""
        pairs = []
        for ti, track in enumerate(self.tracks):
            gate = self.gate_chi2 * (
                1.0 + self.coast_gate_growth * track.consecutive_misses)
            for mi, z in enumerate(measurements):
                d2 = track.ekf.mahalanobis(z)
                if d2 <= gate:
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

        covariance = np.zeros((6, 6))
        covariance[:3, :3] = spherical_covariance_to_cartesian(
            range_m, azimuth, elevation,
            self.measurement_noise_diag[0],
            self.measurement_noise_diag[1],
            self.measurement_noise_diag[2])
        covariance[3:, 3:] = self.initial_velocity_sigma ** 2 * np.eye(3)

        ekf = ConstantVelocityEKF(
            initial_state=np.concatenate([position, velocity]),
            initial_covariance=covariance,
            sigma_accel=self.sigma_accel,
            measurement_noise_diag=self.measurement_noise_diag)
        self.tracks.append(Track(self._next_id, ekf, stamp_s))
        self._next_id += 1
