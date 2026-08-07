"""Linear motion models and the per-mode filter used by the IMM bank.

Shared state: x = [px, py, pz, vx, vy, vz, ax, ay, az]

Both models are linear in the state, so each carries a constant transition
matrix F(dt) rather than a Jacobian. That is the whole reason this pair was
chosen over a coordinated-turn model: mixing two filters is trivial when they
live in the same state space and neither needs a nonlinear process Jacobian.

The measurement side is shared with ekf.py — range/azimuth/elevation/doppler
are nonlinear in the state, hence the extended update.
Pure numpy module — must not import rclpy so tests run without ROS.
"""

import numpy as np

from target_tracker.coordinates import wrap_angle
from target_tracker.ekf import (
    MEAS_DIM,
    measurement_function,
    measurement_jacobian,
)

FULL_STATE_DIM = 9

# The constant-velocity mode pins acceleration at zero, which would leave that
# block of P exactly singular. A small variance floor keeps P positive definite
# so Cholesky/eigenvalue checks and np.linalg.solve stay well behaved.
ACCEL_FLOOR_VAR = 1e-6


def constant_velocity_matrices(dt, sigma_accel):
    """F, Q for a constant-velocity model embedded in the 9D state.

    The acceleration rows of F are zero: this mode's hypothesis is a = 0.
    After predict the acceleration state is zero and its cross-covariance
    with position/velocity is zero, so the measurement update — whose H has
    zero acceleration columns — leaves it at zero.

    Q on the position/velocity block is the usual white-noise-acceleration
    (DWNA) form, identical to ekf.ConstantVelocityEKF.
    """
    F = np.zeros((FULL_STATE_DIM, FULL_STATE_DIM))
    F[:3, :3] = np.eye(3)
    F[:3, 3:6] = dt * np.eye(3)
    F[3:6, 3:6] = np.eye(3)

    q = sigma_accel ** 2
    Q = np.zeros((FULL_STATE_DIM, FULL_STATE_DIM))
    Q[:3, :3] = q * dt ** 4 / 4.0 * np.eye(3)
    Q[:3, 3:6] = q * dt ** 3 / 2.0 * np.eye(3)
    Q[3:6, :3] = q * dt ** 3 / 2.0 * np.eye(3)
    Q[3:6, 3:6] = q * dt ** 2 * np.eye(3)
    Q[6:, 6:] = ACCEL_FLOOR_VAR * np.eye(3)
    return F, Q


def constant_acceleration_matrices(dt, sigma_jerk):
    """F, Q for a constant-acceleration model over the 9D state.

    Kinematics: p += v dt + a dt^2 / 2, v += a dt, a held.
    Q is the continuous Wiener-process-acceleration form driven by sigma_jerk,
    i.e. acceleration performs a random walk with jerk as the driving noise.
    """
    eye = np.eye(3)
    F = np.zeros((FULL_STATE_DIM, FULL_STATE_DIM))
    F[:3, :3] = eye
    F[:3, 3:6] = dt * eye
    F[:3, 6:] = 0.5 * dt ** 2 * eye
    F[3:6, 3:6] = eye
    F[3:6, 6:] = dt * eye
    F[6:, 6:] = eye

    q = sigma_jerk ** 2
    Q = np.zeros((FULL_STATE_DIM, FULL_STATE_DIM))
    Q[:3, :3] = q * dt ** 5 / 20.0 * eye
    Q[:3, 3:6] = q * dt ** 4 / 8.0 * eye
    Q[:3, 6:] = q * dt ** 3 / 6.0 * eye
    Q[3:6, :3] = Q[:3, 3:6]
    Q[3:6, 3:6] = q * dt ** 3 / 3.0 * eye
    Q[3:6, 6:] = q * dt ** 2 / 2.0 * eye
    Q[6:, :3] = Q[:3, 6:]
    Q[6:, 3:6] = Q[3:6, 6:]
    Q[6:, 6:] = q * dt * eye
    return F, Q


def symmetrize(P):
    """Average a covariance with its transpose to kill accumulated asymmetry."""
    return 0.5 * (P + P.T)


class LinearModeEKF:
    """One IMM mode: a linear motion model with the shared nonlinear measurement.

    matrices_fn(dt) returns the (F, Q) pair for this mode, so the same class
    serves both the constant-velocity and constant-acceleration hypotheses.
    """

    def __init__(self, initial_state, initial_covariance, matrices_fn,
                 measurement_noise_diag):
        self.x = np.asarray(initial_state, dtype=float).copy()
        self.P = np.asarray(initial_covariance, dtype=float).copy()
        self.matrices_fn = matrices_fn
        self.R = np.diag(np.asarray(measurement_noise_diag, dtype=float) ** 2)

    def predict(self, dt):
        """Propagate state and covariance forward by dt seconds."""
        F, Q = self.matrices_fn(dt)
        self.x = F @ self.x
        self.P = symmetrize(F @ self.P @ F.T + Q)

    def innovation(self, z):
        """Innovation nu = z - h(x) with azimuth/elevation angle-wrapped."""
        nu = np.asarray(z, dtype=float) - measurement_function(self.x)
        nu[1] = wrap_angle(nu[1])
        nu[2] = wrap_angle(nu[2])
        return nu

    def innovation_covariance(self):
        """H and S = H P H^T + R at the current state."""
        H = measurement_jacobian(self.x)
        return H, H @ self.P @ H.T + self.R

    def mahalanobis(self, z):
        """Squared Mahalanobis distance of measurement z from this mode."""
        _, S = self.innovation_covariance()
        nu = self.innovation(z)
        return float(nu @ np.linalg.solve(S, nu))

    def update(self, z):
        """Fuse z and return this mode's Gaussian measurement LOG-likelihood.

        The log is what gets returned, not the likelihood itself: with angle
        sigmas of 0.02 rad, det(S) is around 1e-11, so a badly-fitting mode
        underflows a plain exp() straight to zero. The IMM normalises across
        modes, and two zeros make that normaliser 0/0.
        """
        H, S = self.innovation_covariance()
        nu = self.innovation(z)
        d2 = float(nu @ np.linalg.solve(S, nu))
        sign, logdet = np.linalg.slogdet(S)
        log_likelihood = -0.5 * (d2 + logdet + MEAS_DIM * np.log(2.0 * np.pi))

        K = self.P @ np.linalg.solve(S, H).T
        self.x = self.x + K @ nu
        # Joseph form keeps P symmetric positive semi-definite.
        I_KH = np.eye(len(self.x)) - K @ H
        self.P = symmetrize(I_KH @ self.P @ I_KH.T + K @ self.R @ K.T)
        return float(log_likelihood)
