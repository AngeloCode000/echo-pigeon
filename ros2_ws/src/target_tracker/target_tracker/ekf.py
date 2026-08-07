"""Constant-velocity extended Kalman filter over a 6D Cartesian state.

State: x = [px, py, pz, vx, vy, vz]
Measurement: z = [range, azimuth, elevation, radial_velocity]

The measurement function is nonlinear in the state, hence the EKF.
Pure numpy module — must not import rclpy so tests run without ROS.
"""

import numpy as np

from target_tracker.coordinates import spherical_jacobian, wrap_angle

STATE_DIM = 6
MEAS_DIM = 4


def measurement_function(state):
    """h(x): predicted [r, az, el, v_r].

    Works for any state whose first six elements are [position, velocity];
    trailing elements (e.g. the acceleration block of a 9D motion model)
    do not affect the measurement.
    """
    p = state[:3]
    v = state[3:6]
    r = np.linalg.norm(p)
    return np.array([
        r,
        np.arctan2(p[1], p[0]),
        np.arctan2(p[2], np.hypot(p[0], p[1])),
        np.dot(p, v) / r,
    ])


def measurement_jacobian(state):
    """H = dh/dx, the 4xN Jacobian of the measurement function.

    N is the length of the state. Columns beyond the first six are zero:
    range, azimuth, elevation and radial velocity depend only on position
    and velocity, never on acceleration.
    """
    p = state[:3]
    v = state[3:6]
    r = np.linalg.norm(p)
    H = np.zeros((MEAS_DIM, len(state)))
    H[:3, :3] = spherical_jacobian(p)
    # d(v_r)/dp = (v r^2 - p (p . v)) / r^3 ; d(v_r)/dv = p / r
    H[3, :3] = (v * r * r - p * np.dot(p, v)) / r ** 3
    H[3, 3:6] = p / r
    return H


class ConstantVelocityEKF:
    """EKF with a constant-velocity motion model and spherical radar measurements."""

    def __init__(self, initial_state, initial_covariance, sigma_accel,
                 measurement_noise_diag):
        self.x = np.asarray(initial_state, dtype=float).copy()
        self.P = np.asarray(initial_covariance, dtype=float).copy()
        self.sigma_accel = float(sigma_accel)
        self.R = np.diag(np.asarray(measurement_noise_diag, dtype=float) ** 2)

    def predict(self, dt):
        """Propagate state and covariance forward by dt seconds."""
        F = np.eye(STATE_DIM)
        F[:3, 3:] = dt * np.eye(3)
        # White-noise-acceleration process noise, per axis.
        q = self.sigma_accel ** 2
        q_pp = q * dt ** 4 / 4.0
        q_pv = q * dt ** 3 / 2.0
        q_vv = q * dt ** 2
        Q = np.zeros((STATE_DIM, STATE_DIM))
        Q[:3, :3] = q_pp * np.eye(3)
        Q[:3, 3:] = q_pv * np.eye(3)
        Q[3:, :3] = q_pv * np.eye(3)
        Q[3:, 3:] = q_vv * np.eye(3)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def innovation(self, z):
        """Innovation nu = z - h(x) with azimuth/elevation angle-wrapped."""
        nu = np.asarray(z, dtype=float) - measurement_function(self.x)
        nu[1] = wrap_angle(nu[1])
        nu[2] = wrap_angle(nu[2])
        return nu

    def mahalanobis(self, z):
        """Squared Mahalanobis distance of measurement z from the predicted state."""
        H = measurement_jacobian(self.x)
        S = H @ self.P @ H.T + self.R
        nu = self.innovation(z)
        return float(nu @ np.linalg.solve(S, nu))

    def update(self, z):
        """Fuse measurement z = [r, az, el, v_r] into the state estimate."""
        H = measurement_jacobian(self.x)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ self.innovation(z)
        # Joseph form keeps P symmetric positive semi-definite.
        I_KH = np.eye(STATE_DIM) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
