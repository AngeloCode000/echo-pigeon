import numpy as np

from target_tracker.coordinates import cartesian_to_spherical, radial_velocity
from target_tracker.ekf import ConstantVelocityEKF
from target_tracker.track_manager import Track, TrackManager, TrackState


def make_confirmed_track(track_id, position, hits=5, cov_scale=0.01):
    """A CONFIRMED track with a tight, converged-looking covariance."""
    ekf = ConstantVelocityEKF(
        initial_state=np.concatenate([np.asarray(position, dtype=float),
                                      [0.0, 0.0, 0.0]]),
        initial_covariance=np.eye(6) * cov_scale,
        sigma_accel=2.0,
        measurement_noise_diag=(0.1, 0.02, 0.02, 0.1))
    track = Track(track_id, ekf, start_time=0.0)
    track.state = TrackState.CONFIRMED
    track.hits = hits
    return track


def detection_from_cartesian(position, velocity):
    r, az, el = cartesian_to_spherical(*position)
    return (r, az, el, radial_velocity(position, velocity))


def make_manager(**overrides):
    kwargs = dict(confirm_hits=3, max_misses=5)
    kwargs.update(overrides)
    return TrackManager(**kwargs)


def run_target(manager, start, velocity, n_frames, dt=0.1, t0=0.0, drop=()):
    pos = np.asarray(start, dtype=float)
    for i in range(n_frames):
        t = t0 + i * dt
        detections = [] if i in drop else [detection_from_cartesian(pos, velocity)]
        manager.process_scan(t, detections)
        pos = pos + np.asarray(velocity) * dt
    return manager


def test_track_confirmed_after_n_hits():
    manager = make_manager()
    run_target(manager, [8.0, 1.0, 1.0], [1.0, 0.0, 0.0], n_frames=2)
    assert manager.tracks[0].state is TrackState.TENTATIVE
    run_target(manager, [8.2, 1.0, 1.0], [1.0, 0.0, 0.0], n_frames=1, t0=0.2)
    assert manager.tracks[0].state is TrackState.CONFIRMED


def test_track_deleted_after_max_misses():
    manager = make_manager()
    run_target(manager, [8.0, 1.0, 1.0], [1.0, 0.0, 0.0], n_frames=5)
    assert len(manager.tracks) == 1
    # 5 consecutive empty frames kill the track.
    for i in range(5):
        manager.process_scan(0.5 + i * 0.1, [])
    assert len(manager.tracks) == 0


def test_track_survives_brief_misses():
    manager = make_manager()
    run_target(manager, [8.0, 1.0, 1.0], [1.0, 0.0, 0.0],
               n_frames=30, drop={10, 11, 17, 24})
    assert len(manager.tracks) == 1
    assert manager.tracks[0].state is TrackState.CONFIRMED
    assert manager.tracks[0].track_id == 1


def test_far_detection_spawns_new_track_not_association():
    manager = make_manager()
    run_target(manager, [8.0, 0.0, 1.0], [1.0, 0.0, 0.0], n_frames=5)
    assert len(manager.tracks) == 1
    # A detection 10 m away must fail the gate and start its own track.
    far = detection_from_cartesian([8.5, 10.0, 1.0], [0.0, 0.0, 0.0])
    manager.process_scan(0.5, [far])
    assert len(manager.tracks) == 2
    assert manager.tracks[1].state is TrackState.TENTATIVE


def test_clutter_does_not_steal_confirmed_track():
    rng = np.random.default_rng(31)
    manager = make_manager()
    pos = np.array([8.0, 0.0, 1.0])
    vel = np.array([1.0, 0.0, 0.0])
    for i in range(50):
        detections = [detection_from_cartesian(pos, vel)]
        # One random clutter point per frame, well away from the target.
        clutter_pos = rng.uniform([2, -8, 0], [20, 8, 4])
        if np.linalg.norm(clutter_pos - pos) > 3.0:
            detections.append(detection_from_cartesian(clutter_pos, [0, 0, 0]))
        manager.process_scan(i * 0.1, detections)
        pos = pos + vel * 0.1

    main_track = manager.tracks[0]
    assert main_track.track_id == 1
    assert main_track.state is TrackState.CONFIRMED
    # The track must still be on the true target, not dragged onto clutter.
    assert np.linalg.norm(main_track.ekf.x[:3] - pos) < 0.5


def test_two_targets_tracked_independently():
    manager = make_manager()
    pos_a = np.array([8.0, 3.0, 1.0])
    pos_b = np.array([8.0, -3.0, 1.0])
    vel_a = np.array([0.5, 0.0, 0.0])
    vel_b = np.array([-0.5, 0.0, 0.0])
    for i in range(20):
        manager.process_scan(i * 0.1, [
            detection_from_cartesian(pos_a, vel_a),
            detection_from_cartesian(pos_b, vel_b),
        ])
        pos_a = pos_a + vel_a * 0.1
        pos_b = pos_b + vel_b * 0.1
    confirmed = [t for t in manager.tracks if t.state is TrackState.CONFIRMED]
    assert len(confirmed) == 2


def test_confidence_reflects_hit_ratio():
    manager = make_manager()
    run_target(manager, [8.0, 1.0, 1.0], [1.0, 0.0, 0.0],
               n_frames=20, drop={5, 10})
    track = manager.tracks[0]
    assert track.confidence == track.hits / (track.hits + track.misses)
    assert 0.8 < track.confidence < 1.0


def test_coast_gate_widens_with_consecutive_misses():
    """A track's gate should loosen only while it is coasting through misses."""

    class _StubEKF:
        """Reports a fixed Mahalanobis distance regardless of measurement."""

        def __init__(self, d2):
            self.d2 = d2

        def mahalanobis(self, z):
            return self.d2

        def update(self, z):
            pass

        def predict(self, dt):
            pass

    z = detection_from_cartesian([8.0, 0.0, 1.0], [0.0, 0.0, 0.0])
    # d2=11.0 clears the flat gate (9.488) but not a gate widened by two
    # consecutive misses at growth=0.15 (9.488 * 1.3 = 12.33).
    for growth, expect_match in ((0.0, False), (0.15, True)):
        manager = make_manager(coast_gate_growth=growth)
        track = Track(1, _StubEKF(11.0), start_time=0.0)
        track.state = TrackState.CONFIRMED
        track.consecutive_misses = 2
        manager.tracks = [track]
        manager._next_id = 2

        manager.process_scan(1.0, [z])

        if expect_match:
            assert len(manager.tracks) == 1
            assert manager.tracks[0].track_id == 1
            assert manager.tracks[0].consecutive_misses == 0
        else:
            assert len(manager.tracks) == 2
            assert manager.tracks[0].consecutive_misses == 3
            assert manager.tracks[1].track_id == 2


def test_stitching_reunites_track_after_full_miss_out():
    manager = make_manager()
    pos = np.array([8.0, 1.0, 1.0])
    vel = np.array([1.0, 0.0, 0.0])
    dt = 0.1

    for i in range(4):
        manager.process_scan(i * dt, [detection_from_cartesian(pos, vel)])
        pos = pos + vel * dt
    assert manager.tracks[0].state is TrackState.CONFIRMED
    original_id = manager.tracks[0].track_id

    # max_misses (5) consecutive empty scans kill the live track; since it
    # was confirmed, it gets tombstoned instead of forgotten outright.
    t = 4 * dt
    for _ in range(5):
        manager.process_scan(t, [])
        pos = pos + vel * dt
        t += dt
    assert len(manager.tracks) == 0

    # A detection consistent with where the target coasted to, well inside
    # the default 2.0s stitch_grace_s (only ~0.1s has elapsed since death).
    manager.process_scan(t, [detection_from_cartesian(pos, vel)])
    assert len(manager.tracks) == 1
    assert manager.tracks[0].track_id == original_id
    assert manager.tracks[0].state is TrackState.CONFIRMED


def test_stitching_expires_after_grace_window():
    manager = make_manager(stitch_grace_s=0.5)
    pos = np.array([8.0, 1.0, 1.0])
    vel = np.array([1.0, 0.0, 0.0])
    dt = 0.1

    for i in range(4):
        manager.process_scan(i * dt, [detection_from_cartesian(pos, vel)])
        pos = pos + vel * dt
    original_id = manager.tracks[0].track_id

    t = 4 * dt
    for _ in range(5):
        manager.process_scan(t, [])
        pos = pos + vel * dt
        t += dt
    assert len(manager.tracks) == 0

    # Wait beyond stitch_grace_s (0.5s) before the next detection arrives.
    t += 1.0
    pos = pos + vel * 1.0
    manager.process_scan(t, [detection_from_cartesian(pos, vel)])
    assert len(manager.tracks) == 1
    assert manager.tracks[0].track_id != original_id
    assert manager.tracks[0].state is TrackState.TENTATIVE


def test_stitching_does_not_merge_distinct_targets():
    manager = make_manager()
    pos_a = np.array([8.0, 3.0, 1.0])
    vel_a = np.array([0.5, 0.0, 0.0])
    dt = 0.1

    for i in range(4):
        manager.process_scan(i * dt, [detection_from_cartesian(pos_a, vel_a)])
        pos_a = pos_a + vel_a * dt
    assert manager.tracks[0].state is TrackState.CONFIRMED
    original_id = manager.tracks[0].track_id

    t = 4 * dt
    for _ in range(5):
        manager.process_scan(t, [])
        pos_a = pos_a + vel_a * dt
        t += dt
    assert len(manager.tracks) == 0

    # A detection from an unrelated target/clutter, far from where A's
    # tombstone would have coasted to, must not revive A's id.
    pos_b = np.array([8.0, -3.0, 1.0])
    manager.process_scan(t, [detection_from_cartesian(pos_b, [0.0, 0.0, 0.0])])
    assert len(manager.tracks) == 1
    assert manager.tracks[0].track_id != original_id
    assert manager.tracks[0].state is TrackState.TENTATIVE


def test_merge_removes_duplicate_confirmed_track_on_same_target():
    manager = make_manager()
    manager.tracks = [
        make_confirmed_track(1, [8.0, 0.0, 3.0], hits=5),
        make_confirmed_track(2, [8.02, 0.01, 3.0], hits=10),
    ]
    manager._next_id = 3

    manager.process_scan(0.1, [])

    assert len(manager.tracks) == 1
    assert manager.tracks[0].track_id == 2
    assert manager.tracks[0].hits == 10


def test_merge_does_not_merge_distinct_targets():
    manager = make_manager()
    manager.tracks = [
        make_confirmed_track(1, [8.0, 3.0, 1.0], hits=5),
        make_confirmed_track(2, [8.0, -3.0, 1.0], hits=5),
    ]
    manager._next_id = 3

    manager.process_scan(0.1, [])

    assert len(manager.tracks) == 2
    assert {t.track_id for t in manager.tracks} == {1, 2}


def test_merge_ignores_tentative_tracks():
    manager = make_manager()
    confirmed = make_confirmed_track(1, [8.0, 0.0, 3.0], hits=5)
    tentative = Track(2, ConstantVelocityEKF(
        initial_state=np.array([8.0, 0.0, 3.0, 0.0, 0.0, 0.0]),
        initial_covariance=np.eye(6) * 0.01,
        sigma_accel=2.0,
        measurement_noise_diag=(0.1, 0.02, 0.02, 0.1)), start_time=0.0)
    manager.tracks = [confirmed, tentative]
    manager._next_id = 3

    manager.process_scan(0.1, [])

    assert {t.track_id for t in manager.tracks} == {1, 2}
