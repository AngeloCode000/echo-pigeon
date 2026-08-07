import numpy as np

from target_tracker.motion_models import (
    ACCEL_FLOOR_VAR,
    FULL_STATE_DIM,
    LinearModeEKF,
    constant_acceleration_matrices,
    constant_velocity_matrices,
    symmetrize,
)

SIGMAS = (0.1, 0.02, 0.02, 0.1)


def test_constant_acceleration_propagates_quadratic_exactly():
    p0 = np.array([8.0, 2.0, 1.5])
    v0 = np.array([0.3, -0.2, 0.1])
    a0 = np.array([0.4, 0.5, -0.2])
    x = np.concatenate([p0, v0, a0])
    dt = 0.05
    F, _ = constant_acceleration_matrices(dt, sigma_jerk=1.0)

    for step in range(1, 101):
        x = F @ x
        t = step * dt
        # Velocity components cross zero on this trajectory, so compare on an
        # absolute tolerance rather than a relative one.
        np.testing.assert_allclose(x[:3], p0 + v0 * t + 0.5 * a0 * t * t, atol=1e-10)
        np.testing.assert_allclose(x[3:6], v0 + a0 * t, atol=1e-10)
        np.testing.assert_allclose(x[6:], a0, atol=1e-12)


def test_constant_acceleration_transition_is_a_semigroup():
    # F(a) @ F(b) == F(a + b) catches any dt^2/2 or dt placement typo.
    F_a, _ = constant_acceleration_matrices(0.3, sigma_jerk=1.0)
    F_b, _ = constant_acceleration_matrices(0.7, sigma_jerk=1.0)
    F_sum, _ = constant_acceleration_matrices(1.0, sigma_jerk=1.0)
    np.testing.assert_allclose(F_a @ F_b, F_sum, atol=1e-12)


def test_constant_velocity_zeroes_acceleration():
    x = np.concatenate([[8.0, 2.0, 1.5], [0.3, -0.2, 0.1], [9.0, -9.0, 9.0]])
    F, _ = constant_velocity_matrices(0.1, sigma_accel=2.0)
    out = F @ x
    np.testing.assert_allclose(out[6:], np.zeros(3), atol=1e-15)
    np.testing.assert_allclose(out[:3], x[:3] + x[3:6] * 0.1, rtol=1e-12)
    np.testing.assert_allclose(out[3:6], x[3:6], rtol=1e-12)


def test_constant_velocity_process_noise_matches_legacy_block():
    """The CV mode must be the legacy filter in the position/velocity subspace."""
    dt, sigma_accel = 0.1, 2.0
    _, Q = constant_velocity_matrices(dt, sigma_accel)
    q = sigma_accel ** 2
    expected = np.zeros((6, 6))
    expected[:3, :3] = q * dt ** 4 / 4.0 * np.eye(3)
    expected[:3, 3:] = q * dt ** 3 / 2.0 * np.eye(3)
    expected[3:, :3] = q * dt ** 3 / 2.0 * np.eye(3)
    expected[3:, 3:] = q * dt ** 2 * np.eye(3)
    np.testing.assert_allclose(Q[:6, :6], expected, rtol=1e-12)
    # Acceleration is pinned, so its only noise is the positive-definiteness floor.
    np.testing.assert_allclose(Q[6:, 6:], ACCEL_FLOOR_VAR * np.eye(3), rtol=1e-12)
    np.testing.assert_allclose(Q[:6, 6:], np.zeros((6, 3)), atol=1e-15)


def test_process_noise_matrices_are_symmetric_psd():
    """Both Q are symmetric PSD; the CA one is additionally full rank.

    The CV block is the discrete white-noise-acceleration form and is rank 3
    of 6 by construction (position and velocity noise share one driving term
    per axis) — exactly as in the legacy ConstantVelocityEKF. Only the CA
    mode's continuous Wiener-process form is strictly positive definite.
    """
    for dt in (0.01, 0.1, 0.5, 1.0):
        _, Q_cv = constant_velocity_matrices(dt, 2.0)
        _, Q_ca = constant_acceleration_matrices(dt, 4.0)
        for Q in (Q_cv, Q_ca):
            np.testing.assert_allclose(Q, Q.T, atol=1e-15)
            assert np.min(np.linalg.eigvalsh(Q)) > -1e-18
        assert np.linalg.matrix_rank(Q_ca) == FULL_STATE_DIM
        assert np.all(np.linalg.eigvalsh(Q_ca) > 0)


def test_symmetrize_averages_with_the_transpose():
    M = np.array([[1.0, 2.0], [0.0, 1.0]])
    np.testing.assert_allclose(symmetrize(M), [[1.0, 1.0], [1.0, 1.0]])


def test_cv_mode_keeps_acceleration_at_zero_through_updates():
    """H has zero acceleration columns, so the update cannot move a pinned state."""
    from target_tracker.coordinates import cartesian_to_spherical, radial_velocity

    pos = np.array([9.0, 1.0, 2.0])
    vel = np.array([0.5, 0.4, 0.0])
    P = np.zeros((FULL_STATE_DIM, FULL_STATE_DIM))
    P[:3, :3] = np.eye(3)
    P[3:6, 3:6] = 25.0 * np.eye(3)
    P[6:, 6:] = 4.0 * np.eye(3)
    mode = LinearModeEKF(
        np.concatenate([pos, vel, np.zeros(3)]), P,
        lambda dt: constant_velocity_matrices(dt, 2.0), SIGMAS)

    for _ in range(30):
        pos = pos + vel * 0.1
        mode.predict(0.1)
        r, az, el = cartesian_to_spherical(*pos)
        mode.update(np.array([r, az, el, radial_velocity(pos, vel)]))
        np.testing.assert_allclose(mode.x[6:], np.zeros(3), atol=1e-12)
        assert np.all(np.linalg.eigvalsh(mode.P) > 0)


def test_mode_update_returns_a_log_likelihood():
    """A far-off measurement must stay finite rather than underflowing to -inf."""
    from target_tracker.coordinates import cartesian_to_spherical, radial_velocity

    pos = np.array([9.0, 1.0, 2.0])
    vel = np.array([0.5, 0.0, 0.0])
    P = np.eye(FULL_STATE_DIM)
    mode = LinearModeEKF(
        np.concatenate([pos, vel, np.zeros(3)]), P,
        lambda dt: constant_acceleration_matrices(dt, 4.0), SIGMAS)

    r, az, el = cartesian_to_spherical(*(pos + np.array([50.0, 50.0, 0.0])))
    log_likelihood = mode.update(np.array([r, az, el, radial_velocity(pos, vel)]))
    assert np.isfinite(log_likelihood)
    assert log_likelihood < 0.0
