import numpy as np
import pytest

from radar_simulator.measurement_model import MeasurementModel
from target_tracker.coordinates import cartesian_to_spherical


POSITION = np.array([8.0, 1.0, 2.0])
VELOCITY = np.array([1.0, -0.5, 0.0])


def test_measurement_centered_on_truth():
    model = MeasurementModel(detection_probability=1.0, clutter_mean_count=0.0,
                             seed=1)
    r_true, az_true, el_true = cartesian_to_spherical(*POSITION)
    samples = [model.target_measurement(POSITION, VELOCITY) for _ in range(500)]
    ranges = np.array([s.range_m for s in samples])
    azimuths = np.array([s.azimuth_rad for s in samples])
    assert np.mean(ranges) == pytest.approx(r_true, abs=0.02)
    assert np.std(ranges) == pytest.approx(0.1, rel=0.2)
    assert np.mean(azimuths) == pytest.approx(az_true, abs=0.005)


def test_detection_probability_respected():
    model = MeasurementModel(detection_probability=0.7, clutter_mean_count=0.0,
                             seed=2)
    detected = sum(
        model.target_measurement(POSITION, VELOCITY) is not None
        for _ in range(2000))
    assert detected / 2000 == pytest.approx(0.7, abs=0.05)


def test_clutter_count_is_poisson():
    model = MeasurementModel(clutter_mean_count=2.0, seed=3)
    counts = [len(model.clutter_measurements()) for _ in range(2000)]
    assert np.mean(counts) == pytest.approx(2.0, abs=0.15)


def test_clutter_within_bounds_and_low_doppler():
    model = MeasurementModel(clutter_mean_count=5.0,
                             clutter_range_bounds=(1.0, 15.0),
                             clutter_azimuth_bounds=(-1.0, 1.0),
                             clutter_elevation_bounds=(-0.3, 0.8),
                             seed=4)
    for _ in range(100):
        for c in model.clutter_measurements():
            assert 1.0 <= c.range_m <= 15.0
            assert -1.0 <= c.azimuth_rad <= 1.0
            assert -0.3 <= c.elevation_rad <= 0.8
            assert abs(c.radial_velocity_mps) < 0.5


def test_snr_decreases_with_range():
    model = MeasurementModel(detection_probability=1.0, snr_noise_db=0.0,
                             seed=5)
    near = model.target_measurement(np.array([3.0, 0.0, 1.0]), VELOCITY)
    far = model.target_measurement(np.array([14.0, 0.0, 1.0]), VELOCITY)
    assert near.signal_strength > far.signal_strength


def test_clutter_snr_below_target_snr():
    model = MeasurementModel(detection_probability=1.0, snr_noise_db=0.0,
                             clutter_mean_count=5.0, seed=6)
    target = model.target_measurement(POSITION, VELOCITY)
    for c in model.clutter_measurements():
        assert c.signal_strength < target.signal_strength


def test_frame_combines_target_and_clutter():
    model = MeasurementModel(detection_probability=1.0, clutter_mean_count=1.0,
                             seed=7)
    sizes = [len(model.frame(POSITION, VELOCITY)) for _ in range(200)]
    assert min(sizes) >= 1  # target always present at p=1
    assert max(sizes) > 1   # clutter appears sometimes


def test_seed_reproducibility():
    a = MeasurementModel(seed=42).frame(POSITION, VELOCITY)
    b = MeasurementModel(seed=42).frame(POSITION, VELOCITY)
    assert len(a) == len(b)
    for ma, mb in zip(a, b):
        assert ma.range_m == mb.range_m
        assert ma.azimuth_rad == mb.azimuth_rad
