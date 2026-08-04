"""Analytic target trajectories for the Phase 0 simulator.

Positions and velocities are exact closed-form functions of time so the
simulated radial velocity is ground truth, not a finite difference.
Pure numpy module — must not import rclpy so tests run without ROS.
"""

import numpy as np


class Circle:
    """Horizontal circle at constant altitude, period seconds per lap."""

    def __init__(self, center=(8.0, 0.0, 3.0), radius=3.0, period=20.0):
        self.center = np.asarray(center, dtype=float)
        self.radius = float(radius)
        self.omega = 2.0 * np.pi / float(period)

    def position(self, t):
        angle = self.omega * t
        return self.center + self.radius * np.array(
            [np.cos(angle), np.sin(angle), 0.0])

    def velocity(self, t):
        angle = self.omega * t
        return self.radius * self.omega * np.array(
            [-np.sin(angle), np.cos(angle), 0.0])


class FigureEight:
    """Gerono lemniscate in the horizontal plane at constant altitude.

    x = cx + A sin(wt),  y = cy + A sin(wt) cos(wt),  z = cz.
    """

    def __init__(self, center=(8.0, 0.0, 3.0), amplitude=3.0, period=20.0):
        self.center = np.asarray(center, dtype=float)
        self.amplitude = float(amplitude)
        self.omega = 2.0 * np.pi / float(period)

    def position(self, t):
        angle = self.omega * t
        return self.center + self.amplitude * np.array([
            np.sin(angle),
            np.sin(angle) * np.cos(angle),
            0.0,
        ])

    def velocity(self, t):
        angle = self.omega * t
        return self.amplitude * self.omega * np.array([
            np.cos(angle),
            np.cos(2.0 * angle),
            0.0,
        ])


def make_trajectory(kind, center, size, period):
    """Factory used by the node: kind is 'circle' or 'figure_eight'."""
    if kind == 'circle':
        return Circle(center=center, radius=size, period=period)
    if kind == 'figure_eight':
        return FigureEight(center=center, amplitude=size, period=period)
    raise ValueError(f"unknown trajectory type '{kind}' "
                     "(expected 'circle' or 'figure_eight')")
