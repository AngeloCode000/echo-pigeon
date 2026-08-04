from types import SimpleNamespace

from radar_preprocessor.filters import (
    range_filter,
    snr_filter,
    static_clutter_filter,
)


def det(range_m=5.0, doppler=1.0, snr=20.0):
    return SimpleNamespace(range_m=range_m, azimuth_rad=0.0,
                           elevation_rad=0.0, radial_velocity_mps=doppler,
                           signal_strength=snr)


def test_snr_filter():
    dets = [det(snr=3.0), det(snr=5.0), det(snr=20.0)]
    kept = snr_filter(dets, min_snr_db=5.0)
    assert [d.signal_strength for d in kept] == [5.0, 20.0]


def test_range_filter():
    dets = [det(range_m=0.1), det(range_m=5.0), det(range_m=40.0)]
    kept = range_filter(dets, min_range_m=0.3, max_range_m=30.0)
    assert [d.range_m for d in kept] == [5.0]


def test_static_clutter_filter():
    dets = [det(doppler=0.0), det(doppler=0.03), det(doppler=-0.5),
            det(doppler=1.2)]
    kept = static_clutter_filter(dets, min_abs_radial_velocity_mps=0.05)
    assert [d.radial_velocity_mps for d in kept] == [-0.5, 1.2]


def test_filters_pass_empty_lists():
    assert snr_filter([], 5.0) == []
    assert range_filter([], 0.3, 30.0) == []
    assert static_clutter_filter([], 0.05) == []
