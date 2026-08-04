"""Spherical/Cartesian coordinate conversions for the radar frame.

Frame convention (REP-103): x forward along boresight, y left, z up.
    range     r  = |p|
    azimuth   az = atan2(y, x)          (positive = target to the left)
    elevation el = atan2(z, hypot(x,y)) (positive = target above boresight)
    radial velocity v_r = (p . v) / r   (positive = receding)

Pure numpy module — must not import rclpy so tests run without ROS.
"""

import numpy as np


def wrap_angle(angle):
    """Wrap an angle (scalar or array) to (-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def spherical_to_cartesian(range_m, azimuth_rad, elevation_rad):
    """Convert spherical coordinates to a Cartesian (x, y, z) numpy array."""
    cos_el = np.cos(elevation_rad)
    return np.array([
        range_m * cos_el * np.cos(azimuth_rad),
        range_m * cos_el * np.sin(azimuth_rad),
        range_m * np.sin(elevation_rad),
    ])


def cartesian_to_spherical(x, y, z):
    """Convert Cartesian coordinates to (range, azimuth, elevation)."""
    range_m = np.sqrt(x * x + y * y + z * z)
    azimuth_rad = np.arctan2(y, x)
    elevation_rad = np.arctan2(z, np.hypot(x, y))
    return range_m, azimuth_rad, elevation_rad


def radial_velocity(position, velocity):
    """Radial velocity of a target given Cartesian position and velocity."""
    position = np.asarray(position, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    range_m = np.linalg.norm(position)
    if range_m == 0.0:
        return 0.0
    return float(np.dot(position, velocity) / range_m)


def spherical_jacobian(position):
    """Jacobian of (r, az, el) with respect to Cartesian position (3x3).

    Rows: d(r)/dp, d(az)/dp, d(el)/dp. Undefined on the z-axis (rho = 0).
    """
    x, y, z = position
    r = np.linalg.norm(position)
    rho_sq = x * x + y * y
    rho = np.sqrt(rho_sq)
    if r == 0.0 or rho == 0.0:
        raise ValueError('spherical Jacobian is singular at rho = 0')
    return np.array([
        [x / r, y / r, z / r],
        [-y / rho_sq, x / rho_sq, 0.0],
        [-x * z / (r * r * rho), -y * z / (r * r * rho), rho / (r * r)],
    ])


def spherical_covariance_to_cartesian(range_m, azimuth_rad, elevation_rad,
                                      sigma_range, sigma_azimuth, sigma_elevation):
    """Linearized Cartesian position covariance from spherical measurement noise.

    Uses P_xyz = J R J^T where J is the inverse-mapping Jacobian
    d(x,y,z)/d(r,az,el) evaluated at the measurement.
    """
    cos_az, sin_az = np.cos(azimuth_rad), np.sin(azimuth_rad)
    cos_el, sin_el = np.cos(elevation_rad), np.sin(elevation_rad)
    jacobian = np.array([
        [cos_el * cos_az, -range_m * cos_el * sin_az, -range_m * sin_el * cos_az],
        [cos_el * sin_az, range_m * cos_el * cos_az, -range_m * sin_el * sin_az],
        [sin_el, 0.0, range_m * cos_el],
    ])
    measurement_cov = np.diag([sigma_range ** 2, sigma_azimuth ** 2, sigma_elevation ** 2])
    return jacobian @ measurement_cov @ jacobian.T
