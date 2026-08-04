import numpy as np
import pytest

from radar_simulator.trajectories import Circle, FigureEight, make_trajectory


def numerical_velocity(trajectory, t, eps=1e-6):
    return (trajectory.position(t + eps) - trajectory.position(t - eps)) / (2 * eps)


def test_circle_geometry():
    circle = Circle(center=(8.0, 0.0, 3.0), radius=3.0, period=20.0)
    for t in np.linspace(0, 40, 50):
        p = circle.position(t)
        assert np.hypot(p[0] - 8.0, p[1]) == pytest.approx(3.0)
        assert p[2] == pytest.approx(3.0)
    # Full period returns to start.
    np.testing.assert_allclose(circle.position(0.0), circle.position(20.0),
                               atol=1e-9)


def test_circle_velocity_is_analytic_derivative():
    circle = Circle(center=(5.0, 2.0, 1.0), radius=2.0, period=12.0)
    for t in np.linspace(0, 12, 25):
        np.testing.assert_allclose(circle.velocity(t),
                                   numerical_velocity(circle, t), atol=1e-5)


def test_circle_speed_is_constant():
    circle = Circle(radius=3.0, period=20.0)
    expected_speed = 2 * np.pi * 3.0 / 20.0
    for t in np.linspace(0, 20, 25):
        assert np.linalg.norm(circle.velocity(t)) == pytest.approx(expected_speed)


def test_figure_eight_geometry():
    fig8 = FigureEight(center=(8.0, 0.0, 3.0), amplitude=3.0, period=20.0)
    # Crosses its center at t = 0 and at each half period.
    np.testing.assert_allclose(fig8.position(0.0), [8.0, 0.0, 3.0], atol=1e-9)
    np.testing.assert_allclose(fig8.position(10.0), [8.0, 0.0, 3.0], atol=1e-9)
    # Stays within the amplitude box.
    for t in np.linspace(0, 20, 100):
        p = fig8.position(t)
        assert abs(p[0] - 8.0) <= 3.0 + 1e-9
        assert abs(p[1]) <= 3.0 + 1e-9
        assert p[2] == pytest.approx(3.0)


def test_figure_eight_velocity_is_analytic_derivative():
    fig8 = FigureEight(center=(6.0, 1.0, 2.0), amplitude=2.5, period=15.0)
    for t in np.linspace(0, 15, 40):
        np.testing.assert_allclose(fig8.velocity(t),
                                   numerical_velocity(fig8, t), atol=1e-5)


def test_factory():
    assert isinstance(make_trajectory('circle', (0, 0, 1), 2.0, 10.0), Circle)
    assert isinstance(
        make_trajectory('figure_eight', (0, 0, 1), 2.0, 10.0), FigureEight)
    with pytest.raises(ValueError):
        make_trajectory('zigzag', (0, 0, 1), 2.0, 10.0)
