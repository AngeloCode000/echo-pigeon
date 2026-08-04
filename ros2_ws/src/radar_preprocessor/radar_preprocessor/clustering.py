"""Minimal DBSCAN over Cartesian points and cluster-to-detection reduction.

Hand-rolled with numpy to avoid a scipy/sklearn dependency; radar frames
are tens of points, so the O(n^2) distance matrix is irrelevant.
Pure numpy module — must not import rclpy so tests run without ROS.
"""

import numpy as np

NOISE = -1


def dbscan(points, eps, min_samples):
    """Label each row of points (n, d). Returns an int array; -1 = noise."""
    points = np.asarray(points, dtype=float)
    n = len(points)
    if n == 0:
        return np.array([], dtype=int)

    diff = points[:, None, :] - points[None, :, :]
    adjacency = np.linalg.norm(diff, axis=2) <= eps
    neighbor_counts = adjacency.sum(axis=1)
    is_core = neighbor_counts >= min_samples

    labels = np.full(n, NOISE, dtype=int)
    cluster = 0
    for i in range(n):
        if labels[i] != NOISE or not is_core[i]:
            continue
        # Grow a new cluster from this unvisited core point.
        labels[i] = cluster
        frontier = [i]
        while frontier:
            j = frontier.pop()
            for k in np.flatnonzero(adjacency[j]):
                if labels[k] == NOISE:
                    labels[k] = cluster
                    if is_core[k]:
                        frontier.append(k)
        cluster += 1
    return labels


def reduce_clusters(detections, cartesian_points, labels, keep_noise=True):
    """Collapse each cluster to one representative detection.

    The representative takes the SNR-weighted centroid position and
    doppler, and the cluster's max signal strength. Points labeled noise
    (-1) are kept as singletons by default — a weak target can return a
    single point per frame, and the SNR/doppler filters upstream are the
    primary clutter rejection. Set keep_noise=False to drop them.
    Returns a list of (centroid_xyz, radial_velocity, signal_strength).
    """
    labels = np.asarray(labels)
    cartesian_points = np.asarray(cartesian_points, dtype=float)
    cluster_ids = sorted(set(labels[labels != NOISE]))

    reduced = []
    for cid in cluster_ids:
        idx = np.flatnonzero(labels == cid)
        weights = np.array([max(detections[i].signal_strength, 1e-3)
                            for i in idx])
        weights = weights / weights.sum()
        centroid = (cartesian_points[idx] * weights[:, None]).sum(axis=0)
        doppler = float(sum(w * detections[i].radial_velocity_mps
                            for w, i in zip(weights, idx)))
        snr = float(max(detections[i].signal_strength for i in idx))
        reduced.append((centroid, doppler, snr))

    if keep_noise:
        for i in np.flatnonzero(labels == NOISE):
            reduced.append((cartesian_points[i],
                            detections[i].radial_velocity_mps,
                            detections[i].signal_strength))
    return reduced
