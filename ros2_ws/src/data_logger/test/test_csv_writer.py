import csv
from types import SimpleNamespace

from data_logger.csv_writer import (
    DETECTION_FIELDS,
    TRACK_FIELDS,
    CsvRunWriter,
)


def make_detection(range_m=5.0):
    return SimpleNamespace(range_m=range_m, azimuth_rad=0.1,
                           elevation_rad=0.2, radial_velocity_mps=1.5,
                           signal_strength=20.0)


def make_track(track_id=1):
    return SimpleNamespace(
        track_id=track_id,
        position_x=1.0, position_y=2.0, position_z=3.0,
        velocity_x=0.1, velocity_y=0.2, velocity_z=0.3,
        track_age=SimpleNamespace(sec=2, nanosec=500_000_000),
        detection_count=10, miss_count=1, confidence=0.9,
        covariance=[float(i) for i in range(36)],
    )


def read_csv(path):
    with open(path, newline='') as f:
        return list(csv.reader(f))


def test_run_directory_created(tmp_path):
    writer = CsvRunWriter(tmp_path, run_name='run_test')
    writer.close()
    assert (tmp_path / 'run_test' / 'detections.csv').exists()
    assert (tmp_path / 'run_test' / 'tracks.csv').exists()


def test_detection_round_trip(tmp_path):
    writer = CsvRunWriter(tmp_path, run_name='run_test')
    writer.write_scan(100, 500, [make_detection(4.0), make_detection(6.0)])
    writer.write_scan(101, 0, [])  # empty frame advances the sequence only
    writer.write_scan(102, 0, [make_detection(8.0)])
    writer.close()

    rows = read_csv(tmp_path / 'run_test' / 'detections.csv')
    assert rows[0] == DETECTION_FIELDS
    assert len(rows) == 4  # header + 3 detection rows
    assert rows[1][:4] == ['100', '500', '0', '0']
    assert rows[2][:4] == ['100', '500', '0', '1']
    # Frame 1 was empty, so the last detection belongs to frame 2.
    assert rows[3][:4] == ['102', '0', '2', '0']
    assert float(rows[3][4]) == 8.0


def test_track_round_trip(tmp_path):
    writer = CsvRunWriter(tmp_path, run_name='run_test')
    writer.write_track(200, 0, make_track(7))
    writer.close()

    rows = read_csv(tmp_path / 'run_test' / 'tracks.csv')
    assert rows[0] == TRACK_FIELDS
    assert len(rows) == 2
    row = dict(zip(TRACK_FIELDS, rows[1]))
    assert row['track_id'] == '7'
    assert float(row['position_x']) == 1.0
    assert float(row['track_age_s']) == 2.5
    # Covariance diagonals of the row-major 6x6: 0, 7, 14, 21, 28, 35.
    assert [float(row[k]) for k in
            ('cov_px', 'cov_py', 'cov_pz', 'cov_vx', 'cov_vy', 'cov_vz')] \
        == [0.0, 7.0, 14.0, 21.0, 28.0, 35.0]


def test_flush_interval(tmp_path):
    writer = CsvRunWriter(tmp_path, flush_every_n=5, run_name='run_test')
    for i in range(4):
        writer.write_track(i, 0, make_track())
    # Fewer rows than the flush threshold: file may be empty on disk.
    writer.write_track(4, 0, make_track())
    rows = read_csv(tmp_path / 'run_test' / 'tracks.csv')
    assert len(rows) == 6  # header + 5, flushed at the threshold
    writer.close()
