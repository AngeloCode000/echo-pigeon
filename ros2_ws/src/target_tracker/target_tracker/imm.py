"""Interacting Multiple Model filter over a constant-velocity / constant-acceleration bank.

A pure constant-velocity filter mispredicts hardest exactly where a target
turns most sharply, which is what fragments a track into new IDs mid-maneuver.
Running a CV and a CA hypothesis side by side and blending them by likelihood
follows the curve instead of extrapolating through it, so the innovation stays
inside the gate without widening the gate for everything else.

Both modes share the 9D state [px, py, pz, vx, vy, vz, ax, ay, az], which makes
the mixing step a plain weighted sum — no state-space translation between modes.

The public surface (x, P, predict, mahalanobis, update, innovation) deliberately
matches ekf.ConstantVelocityEKF so TrackManager can hold either one.
Pure numpy module — must not import rclpy so tests run without ROS.
"""

import numpy as np

from target_tracker.coordinates import wrap_angle
from target_tracker.ekf import measurement_function, measurement_jacobian
from target_tracker.motion_models import (
    FULL_STATE_DIM,
    LinearModeEKF,
    constant_acceleration_matrices,
    constant_velocity_matrices,
    symmetrize,
)

MODE_CV = 0
MODE_CA = 1
N_MODES = 2

# A mode that reaches probability zero can never regain mass, so it would be
# permanently dead the next time the target maneuvers. Floor it instead.
MIN_MODE_PROB = 1e-4


class IMMFilter:
    """Two-mode IMM: constant velocity + constant acceleration.

    p_cv_to_ca / p_ca_to_cv are the per-frame Markov switching probabilities.
    They set the expected dwell time in each mode: at a 10 Hz frame rate,
    p = 0.05 corresponds to roughly a 2 s stay.
    """

    def __init__(self, initial_state, initial_covariance,
                 measurement_noise_diag, sigma_accel=2.0, sigma_jerk=4.0,
                 p_cv_to_ca=0.05, p_ca_to_cv=0.10,
                 initial_mode_probs=(0.5, 0.5)):
        state = np.asarray(initial_state, dtype=float)
        covariance = np.asarray(initial_covariance, dtype=float)
        if state.shape != (FULL_STATE_DIM,):
            raise ValueError(
                f'IMM state must have {FULL_STATE_DIM} elements, got {state.shape[0]}')

        self.sigma_accel = float(sigma_accel)
        self.sigma_jerk = float(sigma_jerk)
        self.modes = [
            LinearModeEKF(
                state, covariance,
                lambda dt: constant_velocity_matrices(dt, self.sigma_accel),
                measurement_noise_diag),
            LinearModeEKF(
                state, covariance,
                lambda dt: constant_acceleration_matrices(dt, self.sigma_jerk),
                measurement_noise_diag),
        ]
        self.transition = np.array([
            [1.0 - p_cv_to_ca, p_cv_to_ca],
            [p_ca_to_cv, 1.0 - p_ca_to_cv],
        ])
        self.mode_probs = np.asarray(initial_mode_probs, dtype=float).copy()
        self.mode_probs /= self.mode_probs.sum()
        self.x, self.P = state.copy(), covariance.copy()
        self._merge()

    @property
    def R(self):
        """Measurement noise covariance (identical across modes)."""
        return self.modes[0].R

    def predict(self, dt):
        """Mix the modes, propagate each forward by dt, and re-merge."""
        self._mix()
        for mode in self.modes:
            mode.predict(dt)
        self._merge()

    def innovation(self, z):
        """Innovation of the merged estimate, for parity with ConstantVelocityEKF."""
        nu = np.asarray(z, dtype=float) - measurement_function(self.x)
        nu[1] = wrap_angle(nu[1])
        nu[2] = wrap_angle(nu[2])
        return nu

    def mahalanobis(self, z):
        """Squared Mahalanobis distance of z from the merged estimate.

        Gating on the merged state rather than on any single mode is what keeps
        this change from loosening clutter rejection: the merged covariance
        already carries the spread between modes, so the gate widens only while
        the modes actually disagree — that is, during a maneuver — and stays
        tight on straight legs. Taking the minimum over modes would instead make
        the gate permanently as wide as the CA hypothesis.
        """
        H = measurement_jacobian(self.x)
        S = H @ self.P @ H.T + self.R
        nu = self.innovation(z)
        return float(nu @ np.linalg.solve(S, nu))

    def update(self, z):
        """Fuse z into every mode and re-weight the modes by their likelihood."""
        log_likelihoods = np.array([mode.update(z) for mode in self.modes])
        # Subtract the max before exponentiating: the largest weight becomes
        # exactly 1.0, so the normaliser can never underflow to zero however
        # badly the other mode fits.
        weights = np.exp(log_likelihoods - log_likelihoods.max())
        posterior = weights * self.mode_probs
        total = posterior.sum()
        if not np.isfinite(total) or total <= 0.0:
            # Nothing usable in the likelihoods; keep the Markov-predicted
            # weighting rather than propagating NaNs into the merged estimate.
            return self._merge()
        self.mode_probs = np.maximum(posterior / total, MIN_MODE_PROB)
        self.mode_probs /= self.mode_probs.sum()
        self._merge()

    def _mix(self):
        """IMM mixing: form each mode's prior from a blend of all modes."""
        # c_j = sum_i pi_ij mu_i ; mu_i|j = pi_ij mu_i / c_j
        c = self.transition.T @ self.mode_probs
        c = np.maximum(c, MIN_MODE_PROB)
        weights = (self.transition * self.mode_probs[:, None]) / c[None, :]

        states = [mode.x.copy() for mode in self.modes]
        covariances = [mode.P.copy() for mode in self.modes]
        for j, mode in enumerate(self.modes):
            mixed_x = sum(weights[i, j] * states[i] for i in range(N_MODES))
            mixed_P = np.zeros_like(covariances[0])
            for i in range(N_MODES):
                spread = (states[i] - mixed_x).reshape(-1, 1)
                mixed_P += weights[i, j] * (covariances[i] + spread @ spread.T)
            mode.x = mixed_x
            mode.P = symmetrize(mixed_P)
        self.mode_probs = c / c.sum()

    def _merge(self):
        """Collapse the mode bank into a single Gaussian for output and gating."""
        self.x = sum(self.mode_probs[j] * self.modes[j].x for j in range(N_MODES))
        P = np.zeros_like(self.modes[0].P)
        for j in range(N_MODES):
            spread = (self.modes[j].x - self.x).reshape(-1, 1)
            P += self.mode_probs[j] * (self.modes[j].P + spread @ spread.T)
        self.P = symmetrize(P)
