import numpy as np

from target_tracker.coordinates import cartesian_to_spherical, radial_velocity
from target_tracker.ekf import (
    ConstantVelocityEKF,
    measurement_function,
    measurement_jacobian,
)

SIGMAS = (0.1, 0.02, 0.02, 0.1)


def make_measurement(position, velocity, rng=None, sigmas=SIGMAS):
    r, az, el = cartesian_to_spherical(*position)
    z = np.array([r, az, el, radial_velocity(position, velocity)])
    if rng is not None:
        z += rng.normal(0.0, sigmas)
    return z


def make_ekf(position, velocity):
    state = np.concatenate([position, velocity])
    covariance = np.diag([1.0] * 3 + [25.0] * 3)
    return ConstantVelocityEKF(state, covariance, sigma_accel=1.0,
                               measurement_noise_diag=SIGMAS)


def test_measurement_jacobian_matches_numerical():
    rng = np.random.default_rng(3)
    eps = 1e-6
    for _ in range(50):
        state = rng.uniform(-10, 10, size=6)
        if np.hypot(state[0], state[1]) < 0.5:
            continue
        H = measurement_jacobian(state)
        for col in range(6):
            dx = np.zeros(6)
            dx[col] = eps
            numerical = (measurement_function(state + dx)
                         - measurement_function(state - dx)) / (2 * eps)
            np.testing.assert_allclose(H[:, col], numerical, atol=1e-4)


def test_converges_on_constant_velocity_target():
    rng = np.random.default_rng(11)
    dt = 0.1
    # Slow target that stays inside the project's 3-10 m test envelope,
    # where cross-range velocity is observable from angle rates.
    true_pos = np.array([8.0, 2.0, 1.5])
    true_vel = np.array([0.3, -0.2, 0.1])

    # Deliberately biased initial state.
    ekf = make_ekf(true_pos + [1.0, -1.0, 0.5], np.zeros(3))

    errors = []
    velocity_estimates = []
    for _ in range(200):
        true_pos = true_pos + true_vel * dt
        ekf.predict(dt)
        ekf.update(make_measurement(true_pos, true_vel, rng))
        errors.append(np.linalg.norm(ekf.x[:3] - true_pos))
        velocity_estimates.append(ekf.x[3:].copy())

    # At ~10 m range, azimuth noise contributes ~0.2 m cross-range error per
    # measurement; the filter must average below that and beat the initial
    # 1.5 m state bias. Velocity is judged on its time average — a single
    # end-point sample is one noisy draw from the steady-state distribution.
    assert np.mean(errors[-50:]) < 0.2
    assert np.mean(errors[-50:]) < errors[0]
    mean_velocity = np.mean(velocity_estimates[-50:], axis=0)
    assert np.linalg.norm(mean_velocity - true_vel) < 0.25


def test_covariance_shrinks_with_updates():
    rng = np.random.default_rng(5)
    dt = 0.1
    pos = np.array([10.0, 0.0, 2.0])
    vel = np.array([0.5, 1.0, 0.0])
    ekf = make_ekf(pos, vel)
    initial_trace = np.trace(ekf.P)
    for _ in range(50):
        pos = pos + vel * dt
        ekf.predict(dt)
        ekf.update(make_measurement(pos, vel, rng))
    assert np.trace(ekf.P) < initial_trace / 5


def test_tracks_figure_eight_maneuver():
    """The CV model must follow a maneuvering target within a loose bound."""
    rng = np.random.default_rng(23)
    dt = 0.1
    amplitude, period = 5.0, 20.0
    omega = 2 * np.pi / period
    center = np.array([12.0, 0.0, 3.0])

    def truth(t):
        pos = center + np.array([
            amplitude * np.sin(omega * t),
            amplitude * np.sin(omega * t) * np.cos(omega * t),
            0.0,
        ])
        vel = np.array([
            amplitude * omega * np.cos(omega * t),
            amplitude * omega * np.cos(2 * omega * t),
            0.0,
        ])
        return pos, vel

    pos0, vel0 = truth(0.0)
    ekf = make_ekf(pos0, vel0)
    ekf.sigma_accel = 3.0  # maneuvering target needs more process noise

    errors = []
    for step in range(1, 400):
        t = step * dt
        pos, vel = truth(t)
        ekf.predict(dt)
        ekf.update(make_measurement(pos, vel, rng))
        errors.append(np.linalg.norm(ekf.x[:3] - pos))

    assert np.mean(errors[50:]) < 0.5


def test_covariance_stays_symmetric():
    rng = np.random.default_rng(9)
    dt = 0.1
    pos = np.array([6.0, 1.0, 1.0])
    vel = np.array([0.2, 0.3, 0.1])
    ekf = make_ekf(pos, vel)
    for _ in range(100):
        pos = pos + vel * dt
        ekf.predict(dt)
        ekf.update(make_measurement(pos, vel, rng))
        np.testing.assert_allclose(ekf.P, ekf.P.T, atol=1e-10)
        assert np.all(np.linalg.eigvalsh(ekf.P) > 0)


def test_mahalanobis_distinguishes_near_and_far():
    pos = np.array([10.0, 0.0, 2.0])
    vel = np.array([1.0, 0.0, 0.0])
    ekf = make_ekf(pos, vel)
    near = make_measurement(pos, vel)
    far = make_measurement(pos + [5.0, 5.0, 0.0], vel)
    gate_chi2 = 9.488  # chi-square 95th percentile, 4 DOF
    assert ekf.mahalanobis(near) < gate_chi2
    assert ekf.mahalanobis(far) > gate_chi2
