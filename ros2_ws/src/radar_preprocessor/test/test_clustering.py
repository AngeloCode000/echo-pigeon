from types import SimpleNamespace

import numpy as np

from radar_preprocessor.clustering import NOISE, dbscan, reduce_clusters


def det(doppler=1.0, snr=20.0):
    return SimpleNamespace(radial_velocity_mps=doppler, signal_strength=snr)


def test_dbscan_empty():
    assert len(dbscan([], eps=0.5, min_samples=2)) == 0


def test_dbscan_two_well_separated_clusters():
    points = np.array([
        [0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.1, 0.0],
        [5.0, 5.0, 0.0], [5.1, 5.0, 0.0], [5.0, 5.1, 0.0],
    ])
    labels = dbscan(points, eps=0.5, min_samples=2)
    assert len(set(labels[:3])) == 1
    assert len(set(labels[3:])) == 1
    assert labels[0] != labels[3]
    assert NOISE not in labels


def test_dbscan_isolated_point_is_noise():
    points = np.array([
        [0.0, 0.0, 0.0], [0.1, 0.0, 0.0],
        [10.0, 10.0, 10.0],
    ])
    labels = dbscan(points, eps=0.5, min_samples=2)
    assert labels[2] == NOISE
    assert labels[0] == labels[1] != NOISE


def test_dbscan_chain_connectivity():
    # Points 0.4 apart in a line: each neighbors the next, one cluster.
    points = np.array([[0.4 * i, 0.0, 0.0] for i in range(6)])
    labels = dbscan(points, eps=0.5, min_samples=2)
    assert len(set(labels)) == 1
    assert NOISE not in labels


def test_reduce_clusters_centroid_and_snr():
    points = np.array([
        [1.0, 0.0, 0.0], [1.2, 0.0, 0.0],
    ])
    dets = [det(doppler=1.0, snr=10.0), det(doppler=2.0, snr=10.0)]
    labels = np.array([0, 0])
    reduced = reduce_clusters(dets, points, labels)
    assert len(reduced) == 1
    centroid, doppler, snr = reduced[0]
    np.testing.assert_allclose(centroid, [1.1, 0.0, 0.0])
    assert doppler == 1.5
    assert snr == 10.0


def test_reduce_clusters_snr_weighting():
    points = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    dets = [det(snr=30.0), det(snr=10.0)]
    reduced = reduce_clusters(dets, points, np.array([0, 0]))
    centroid, _, snr = reduced[0]
    assert centroid[0] < 1.5  # pulled toward the stronger return
    assert snr == 30.0


def test_reduce_clusters_keeps_noise_by_default():
    points = np.array([
        [1.0, 0.0, 0.0], [1.1, 0.0, 0.0],
        [8.0, 0.0, 0.0],
    ])
    dets = [det(), det(), det(doppler=3.0, snr=7.0)]
    labels = np.array([0, 0, NOISE])
    reduced = reduce_clusters(dets, points, labels)
    assert len(reduced) == 2
    noise_entries = [r for r in reduced if r[2] == 7.0]
    assert len(noise_entries) == 1
    np.testing.assert_allclose(noise_entries[0][0], [8.0, 0.0, 0.0])


def test_reduce_clusters_can_drop_noise():
    points = np.array([[1.0, 0.0, 0.0], [1.1, 0.0, 0.0], [8.0, 0.0, 0.0]])
    dets = [det(), det(), det()]
    labels = np.array([0, 0, NOISE])
    reduced = reduce_clusters(dets, points, labels, keep_noise=False)
    assert len(reduced) == 1


def test_reduce_clusters_all_noise_passthrough():
    points = np.array([[1.0, 0.0, 0.0], [8.0, 0.0, 0.0]])
    dets = [det(snr=10.0), det(snr=12.0)]
    labels = np.array([NOISE, NOISE])
    reduced = reduce_clusters(dets, points, labels)
    assert len(reduced) == 2
