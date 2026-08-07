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

# How many consecutive misses a confirmed track may have and still suppress a
# nearby spawn. The twin-spawn case this exists to stop is exactly one miss:
# the detection that failed the association gate is the same one that would
# otherwise start the twin, so the track is always one frame unfed at that
# moment. A track that has genuinely slid off the target during a hard turn
# accumulates misses fast, and past this allowance it stops suppressing so the
# detection that would re-acquire the target is free to start a new track.
MAX_SUPPRESSING_MISSES = 1


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
                 imm_p_cv_to_ca=0.05, imm_p_ca_to_cv=0.10,
                 spawn_gate_chi2=30.0, merge_chi2=25.0):
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
        # Association and initiation ask different questions of the same
        # detection — "good enough to fuse?" versus "plausibly the same
        # object?" — and so deserve different thresholds. See
        # _suppressed_by_confirmed_track. Either may be set to 0 to disable.
        self.spawn_gate_chi2 = spawn_gate_chi2
        self.merge_chi2 = merge_chi2
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
            if self._suppressed_by_confirmed_track(measurement):
                continue
            self._spawn_track(measurement, stamp_s)

        self.tracks = [t for t in self.tracks
                       if t.consecutive_misses < self.max_misses]
        self._merge_duplicates()
        return self.tracks

    def _suppressed_by_confirmed_track(self, measurement):
        """True if this detection plausibly belongs to a confirmed track already.

        gate_chi2 is the chi-square 95th percentile, so about 5% of a target's
        own detections fail association by construction. Spawning on every one
        of those births a duplicate track directly on top of the target that
        just rejected it, which is the dominant source of one-object-many-IDs.
        Answering "is this the same object?" with a looser gate than "is this
        good enough to fuse?" removes the duplicate at the source and costs
        nothing on straight legs.

        Only confirmed tracks suppress. A clutter-born tentative track must
        never be able to shadow a genuinely new target.

        A track must also still be current, within MAX_SUPPRESSING_MISSES. A
        long-coasting track may have slid off the target during a hard turn,
        and if it could still suppress it would block the very detection that
        would re-acquire the target — trading duplicate IDs for outright
        dropouts, the worse failure of the two. This matters most for the 'cv'
        model, which drifts hardest through turns: with no such limit its
        sharp-turn coverage gap goes to 20% of frames.
        """
        if self.spawn_gate_chi2 <= 0.0:
            return False
        return any(
            track.state is TrackState.CONFIRMED
            and track.consecutive_misses <= MAX_SUPPRESSING_MISSES
            and track.ekf.mahalanobis(measurement) <= self.spawn_gate_chi2
            for track in self.tracks)

    def _merge_duplicates(self):
        """Collapse track pairs that have converged onto the same object.

        Spawn suppression cannot catch twins born before either track
        confirmed, because there was no confirmed track to suppress against
        yet. This is the second line of defence.
        """
        if self.merge_chi2 <= 0.0:
            return
        dropped = set()
        for i in range(len(self.tracks)):
            if i in dropped:
                continue
            for j in range(i + 1, len(self.tracks)):
                if j in dropped:
                    continue
                a, b = self.tracks[i], self.tracks[j]
                if self._state_distance(a, b) > self.merge_chi2:
                    continue
                # The lower ID is the older identity — the one downstream
                # consumers have been seeing — so it always survives.
                keep, drop = (a, b) if a.track_id < b.track_id else (b, a)
                self._absorb(keep, drop)
                dropped.add(j if keep is a else i)
                if keep is b:
                    break
        if dropped:
            self.tracks = [t for k, t in enumerate(self.tracks) if k not in dropped]

    def _state_distance(self, a, b):
        """Squared Mahalanobis distance between two tracks' [p, v] estimates.

        Restricted to the first six elements so one code path serves both the
        6D CV filter and the 9D IMM, and because acceleration is the least
        observable part of the state — including it would mostly feed noise
        into the decision.
        """
        dx = a.ekf.x[:6] - b.ekf.x[:6]
        S = a.ekf.P[:6, :6] + b.ekf.P[:6, :6]
        try:
            return float(dx @ np.linalg.solve(S, dx))
        except np.linalg.LinAlgError:
            return np.inf

    def _absorb(self, keep, drop):
        """Fold drop's evidence into keep, retaining keep's ID."""
        # Freshness first, confidence only as a tie-break. Ranking by trace(P)
        # alone loses the target outright during a hard turn: a track that has
        # drifted off but been updated for seconds is far more confident than
        # the accurate replacement just spawned on top of the real target,
        # which still carries the 25 m^2/s^2 birth velocity covariance. The
        # drifted estimate would win every time, the true one would be
        # discarded, and the resulting re-spawn thrash drove coverage gaps to
        # 25% of frames and quadrupled the IDs minted.
        if (drop.consecutive_misses, np.trace(drop.ekf.P)) < \
                (keep.consecutive_misses, np.trace(keep.ekf.P)):
            keep.ekf = drop.ekf
        # The two tracks were fed overlapping detections, so summing hits
        # would double-count the same evidence.
        keep.hits = max(keep.hits, drop.hits)
        keep.misses = min(keep.misses, drop.misses)
        keep.consecutive_misses = min(keep.consecutive_misses,
                                      drop.consecutive_misses)
        keep.start_time = min(keep.start_time, drop.start_time)
        if drop.state is TrackState.CONFIRMED or keep.hits >= self.confirm_hits:
            keep.state = TrackState.CONFIRMED

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
