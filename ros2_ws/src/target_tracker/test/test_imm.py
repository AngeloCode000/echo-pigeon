import numpy as np
import pytest

from target_tracker.coordinates import cartesian_to_spherical, radial_velocity
from target_tracker.ekf import (
    ConstantVelocityEKF,
    measurement_function,
    measurement_jacobian,
)
from target_tracker.imm import MIN_MODE_PROB, N_MODES, IMMFilter

SIGMAS = (0.1, 0.02, 0.02, 0.1)


def make_measurement(position, velocity, rng=None, sigmas=SIGMAS):
    r, az, el = cartesian_to_spherical(*position)
    z = np.array([r, az, el, radial_velocity(position, velocity)])
    if rng is not None:
        z += rng.normal(0.0, sigmas)
    return z


def birth_covariance(accel_sigma=2.0):
    P = np.zeros((9, 9))
    P[:3, :3] = np.eye(3)
    P[3:6, 3:6] = 25.0 * np.eye(3)
    P[6:, 6:] = accel_sigma ** 2 * np.eye(3)
    return P


def make_imm(position, velocity, **kwargs):
    state = np.concatenate([position, velocity, np.zeros(3)])
    return IMMFilter(state, birth_covariance(), SIGMAS, **kwargs)


def figure_eight(amplitude, period, center=(12.0, 0.0, 3.0)):
    """Gerono lemniscate, matching radar_simulator.trajectories.FigureEight."""
    center = np.asarray(center, dtype=float)
    omega = 2 * np.pi / period

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

    return truth


def measurement_stream(truth, seed, n_steps=400, dt=0.1):
    """Pre-generate one noisy stream so competing filters see identical data."""
    rng = np.random.default_rng(seed)
    return [(truth(step * dt)[0], make_measurement(*truth(step * dt), rng=rng))
            for step in range(1, n_steps)]


def replay(filt, stream, dt=0.1):
    errors, mode_probs = [], []
    for position, z in stream:
        filt.predict(dt)
        filt.update(z)
        errors.append(float(np.linalg.norm(filt.x[:3] - position)))
        if hasattr(filt, 'mode_probs'):
            mode_probs.append(filt.mode_probs.copy())
    return np.array(errors), np.array(mode_probs) if mode_probs else None


def test_measurement_jacobian_matches_numerical_9d():
    """Same central-difference harness as the 6D case, over the 9D state."""
    rng = np.random.default_rng(3)
    eps = 1e-6
    for _ in range(50):
        state = rng.uniform(-10, 10, size=9)
        if np.hypot(state[0], state[1]) < 0.5:
            continue
        H = measurement_jacobian(state)
        assert H.shape == (4, 9)
        for col in range(9):
            dx = np.zeros(9)
            dx[col] = eps
            numerical = (measurement_function(state + dx)
                         - measurement_function(state - dx)) / (2 * eps)
            np.testing.assert_allclose(H[:, col], numerical, atol=1e-4)


def test_measurement_jacobian_acceleration_columns_are_zero():
    state = np.array([9.0, 2.0, 1.0, 0.5, -0.3, 0.1, 1.0, -2.0, 3.0])
    np.testing.assert_allclose(measurement_jacobian(state)[:, 6:], np.zeros((4, 3)))


def test_measurement_jacobian_6d_shape_unchanged():
    state = np.array([9.0, 2.0, 1.0, 0.5, -0.3, 0.1])
    assert measurement_jacobian(state).shape == (4, 6)


def test_rejects_a_state_that_is_not_9d():
    with pytest.raises(ValueError):
        IMMFilter(np.zeros(6), np.eye(6), SIGMAS)


def test_converges_on_constant_velocity_target():
    rng = np.random.default_rng(11)
    true_pos = np.array([8.0, 2.0, 1.5])
    true_vel = np.array([0.3, -0.2, 0.1])
    imm = make_imm(true_pos + [1.0, -1.0, 0.5], np.zeros(3))

    errors = []
    for _ in range(200):
        true_pos = true_pos + true_vel * 0.1
        imm.predict(0.1)
        imm.update(make_measurement(true_pos, true_vel, rng))
        errors.append(np.linalg.norm(imm.x[:3] - true_pos))

    # Same bar as the CV filter's convergence test.
    assert np.mean(errors[-50:]) < 0.2
    # A target flying straight should leave the bank in the CV hypothesis.
    assert imm.mode_probs[0] > 0.5


def test_mode_probability_rises_with_maneuver_severity():
    """The CA mode must earn its weight from the data, not sit at a constant."""
    mean_mu_ca = []
    for amplitude, period in ((5.0, 20.0), (5.0, 12.0), (8.0, 8.0)):
        truth = figure_eight(amplitude, period)
        stream = measurement_stream(truth, seed=23)
        pos0, vel0 = truth(0.0)
        _, mode_probs = replay(make_imm(pos0, vel0), stream)
        mean_mu_ca.append(mode_probs[50:, 1].mean())

    assert mean_mu_ca[0] < mean_mu_ca[1] < mean_mu_ca[2]
    assert mean_mu_ca[0] < 0.35     # gentle: CV hypothesis dominates
    assert mean_mu_ca[2] > 0.6      # sharp: CA hypothesis dominates


def test_beats_cv_on_a_sharp_figure_eight():
    """The point of the whole filter: hold a hard turn a CV model cannot."""
    truth = figure_eight(amplitude=8.0, period=8.0)
    pos0, vel0 = truth(0.0)
    cv_total, imm_total = [], []
    for seed in (23, 24, 25):
        stream = measurement_stream(truth, seed)
        cv = ConstantVelocityEKF(np.concatenate([pos0, vel0]),
                                 np.diag([1.0] * 3 + [25.0] * 3),
                                 sigma_accel=3.0, measurement_noise_diag=SIGMAS)
        cv_errors, _ = replay(cv, stream)
        imm_errors, _ = replay(make_imm(pos0, vel0), stream)
        cv_total.append(cv_errors[50:].mean())
        imm_total.append(imm_errors[50:].mean())

    cv_mean, imm_mean = np.mean(cv_total), np.mean(imm_total)
    # Measured: CV ~0.49 m, IMM ~0.23 m at this maneuver severity.
    assert imm_mean < 0.65 * cv_mean
    assert imm_mean < 0.32


def test_costs_nothing_on_a_gentle_trajectory():
    """Where the CV model is already adequate, the bank must not make it worse."""
    truth = figure_eight(amplitude=5.0, period=20.0)
    pos0, vel0 = truth(0.0)
    ratios = []
    for seed in (23, 24, 25):
        stream = measurement_stream(truth, seed)
        cv = ConstantVelocityEKF(np.concatenate([pos0, vel0]),
                                 np.diag([1.0] * 3 + [25.0] * 3),
                                 sigma_accel=3.0, measurement_noise_diag=SIGMAS)
        cv_errors, _ = replay(cv, stream)
        imm_errors, _ = replay(make_imm(pos0, vel0), stream)
        ratios.append(imm_errors[50:].mean() / cv_errors[50:].mean())
    assert np.mean(ratios) < 1.05


def test_covariance_stays_symmetric_through_mixing():
    """Mixing and merging both add outer products; neither may break P."""
    truth = figure_eight(amplitude=8.0, period=8.0)
    stream = measurement_stream(truth, seed=9, n_steps=150)
    pos0, vel0 = truth(0.0)
    imm = make_imm(pos0, vel0)

    for _, z in stream:
        for stage in (lambda: imm.predict(0.1), lambda: imm.update(z)):
            stage()
            np.testing.assert_allclose(imm.P, imm.P.T, atol=1e-10)
            assert np.all(np.linalg.eigvalsh(imm.P) > 0)
            for mode in imm.modes:
                np.testing.assert_allclose(mode.P, mode.P.T, atol=1e-10)
                assert np.all(np.linalg.eigvalsh(mode.P) > 0)


def test_mode_probs_stay_a_normalized_distribution():
    truth = figure_eight(amplitude=8.0, period=8.0)
    stream = measurement_stream(truth, seed=4, n_steps=200)
    imm = make_imm(*truth(0.0))
    for _, z in stream:
        imm.predict(0.1)
        imm.update(z)
        assert np.all(np.isfinite(imm.mode_probs))
        # The floor is applied before the final renormalization, so the
        # smallest weight can land a hair under it; what matters is that no
        # mode reaches zero and can never recover.
        assert np.all(imm.mode_probs > 0.9 * MIN_MODE_PROB)
        np.testing.assert_allclose(imm.mode_probs.sum(), 1.0, atol=1e-12)


def test_survives_a_wild_outlier():
    """A measurement far outside any gate must not poison the filter."""
    pos = np.array([10.0, 0.0, 2.0])
    vel = np.array([1.0, 0.0, 0.0])
    imm = make_imm(pos, vel)
    imm.predict(0.1)
    imm.update(make_measurement(pos + np.array([50.0, 50.0, 20.0]), vel))
    assert np.all(np.isfinite(imm.x))
    assert np.all(np.isfinite(imm.P))
    np.testing.assert_allclose(imm.mode_probs.sum(), 1.0, atol=1e-12)


def test_predict_without_update_advances_modes_by_the_markov_chain():
    """A coasting track gets no likelihood evidence, only the transition prior."""
    imm = make_imm(np.array([10.0, 0.0, 2.0]), np.array([1.0, 0.0, 0.0]))
    expected = imm.mode_probs.copy()
    for _ in range(3):
        imm.predict(0.1)
        expected = imm.transition.T @ expected
    np.testing.assert_allclose(imm.mode_probs, expected, atol=1e-12)


def test_innovation_wraps_azimuth():
    """A target behind the radar straddles the +/-pi branch cut."""
    pos = np.array([-10.0, -0.05, 1.0])
    vel = np.array([0.0, 0.5, 0.0])
    imm = make_imm(pos, vel)
    z = make_measurement(np.array([-10.0, 0.05, 1.0]), vel)
    nu = imm.innovation(z)
    assert abs(nu[1]) < np.pi


def test_mahalanobis_distinguishes_near_and_far():
    """Gating on the merged estimate must keep the existing gate meaningful."""
    pos = np.array([10.0, 0.0, 2.0])
    vel = np.array([1.0, 0.0, 0.0])
    imm = make_imm(pos, vel)
    gate_chi2 = 9.488  # chi-square 95th percentile, 4 DOF
    assert imm.mahalanobis(make_measurement(pos, vel)) < gate_chi2
    assert imm.mahalanobis(make_measurement(pos + [5.0, 5.0, 0.0], vel)) > gate_chi2


def test_exposes_the_constant_velocity_ekf_surface():
    """TrackManager holds either filter behind one attribute, so both must match."""
    imm = make_imm(np.array([10.0, 0.0, 2.0]), np.array([1.0, 0.0, 0.0]))
    for name in ('x', 'P', 'R', 'predict', 'innovation', 'mahalanobis', 'update'):
        assert hasattr(imm, name)
    assert imm.x.shape == (9,)
    assert imm.P.shape == (9, 9)
    assert imm.R.shape == (4, 4)
    assert len(imm.modes) == N_MODES
