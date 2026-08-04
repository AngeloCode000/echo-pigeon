import numpy as np
import pytest

from target_tracker.coordinates import (
    cartesian_to_spherical,
    radial_velocity,
    spherical_covariance_to_cartesian,
    spherical_jacobian,
    spherical_to_cartesian,
    wrap_angle,
)


def test_round_trip_randomized():
    rng = np.random.default_rng(42)
    for _ in range(200):
        p = rng.uniform(-20, 20, size=3)
        if np.hypot(p[0], p[1]) < 1e-6:
            continue
        r, az, el = cartesian_to_spherical(*p)
        back = spherical_to_cartesian(r, az, el)
        np.testing.assert_allclose(back, p, atol=1e-9)


def test_known_points():
    r, az, el = cartesian_to_spherical(1.0, 0.0, 0.0)
    assert (r, az, el) == pytest.approx((1.0, 0.0, 0.0))
    r, az, el = cartesian_to_spherical(0.0, 2.0, 0.0)
    assert (r, az, el) == pytest.approx((2.0, np.pi / 2, 0.0))
    r, az, el = cartesian_to_spherical(3.0, 0.0, 3.0)
    assert (r, az, el) == pytest.approx((3.0 * np.sqrt(2), 0.0, np.pi / 4))


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(np.pi + 0.1) == pytest.approx(-np.pi + 0.1)
    assert wrap_angle(-np.pi - 0.1) == pytest.approx(np.pi - 0.1)
    assert wrap_angle(3 * np.pi) == pytest.approx(np.pi)


def test_radial_velocity_signs():
    # Receding target: positive doppler.
    assert radial_velocity([10, 0, 0], [1, 0, 0]) == pytest.approx(1.0)
    # Approaching target: negative doppler.
    assert radial_velocity([10, 0, 0], [-2, 0, 0]) == pytest.approx(-2.0)
    # Tangential motion: zero doppler.
    assert radial_velocity([10, 0, 0], [0, 3, 0]) == pytest.approx(0.0)


def test_spherical_jacobian_matches_numerical():
    rng = np.random.default_rng(7)
    eps = 1e-6
    for _ in range(50):
        p = rng.uniform(-10, 10, size=3)
        if np.hypot(p[0], p[1]) < 0.5:
            continue
        J = spherical_jacobian(p)
        for col in range(3):
            dp = np.zeros(3)
            dp[col] = eps
            f_plus = np.array(cartesian_to_spherical(*(p + dp)))
            f_minus = np.array(cartesian_to_spherical(*(p - dp)))
            numerical = (f_plus - f_minus) / (2 * eps)
            numerical[1] = wrap_angle(numerical[1] * 2 * eps) / (2 * eps)
            np.testing.assert_allclose(J[:, col], numerical, atol=1e-5)


def test_spherical_jacobian_singular_on_z_axis():
    with pytest.raises(ValueError):
        spherical_jacobian(np.array([0.0, 0.0, 5.0]))


def test_covariance_conversion_positive_definite():
    cov = spherical_covariance_to_cartesian(10.0, 0.3, 0.2, 0.1, 0.02, 0.02)
    assert cov.shape == (3, 3)
    np.testing.assert_allclose(cov, cov.T, atol=1e-12)
    eigenvalues = np.linalg.eigvalsh(cov)
    assert np.all(eigenvalues > 0)


def test_covariance_conversion_scales_with_range():
    near = spherical_covariance_to_cartesian(5.0, 0.0, 0.0, 0.1, 0.02, 0.02)
    far = spherical_covariance_to_cartesian(50.0, 0.0, 0.0, 0.1, 0.02, 0.02)
    # Cross-range uncertainty grows with range; range uncertainty does not.
    assert far[1, 1] > near[1, 1]
    assert far[0, 0] == pytest.approx(near[0, 0])
