import numpy as np

from target_tracker.coordinates import cartesian_to_spherical, radial_velocity
from target_tracker.track_manager import TrackManager, TrackState


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
