import numpy as np
import pytest

from target_tracker.coordinates import cartesian_to_spherical, radial_velocity
from target_tracker.ekf import ConstantVelocityEKF
from target_tracker.imm import IMMFilter
from target_tracker.track_manager import TrackManager, TrackState

# Every lifecycle test runs under both motion models: 'imm' is the default the
# tracker now ships with, and 'cv' is the legacy filter kept as a rollback.
MOTION_MODELS = ['cv', 'imm']


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


@pytest.mark.parametrize('motion_model', MOTION_MODELS)
def test_track_confirmed_after_n_hits(motion_model):
    manager = make_manager(motion_model=motion_model)
    run_target(manager, [8.0, 1.0, 1.0], [1.0, 0.0, 0.0], n_frames=2)
    assert manager.tracks[0].state is TrackState.TENTATIVE
    run_target(manager, [8.2, 1.0, 1.0], [1.0, 0.0, 0.0], n_frames=1, t0=0.2)
    assert manager.tracks[0].state is TrackState.CONFIRMED


@pytest.mark.parametrize('motion_model', MOTION_MODELS)
def test_track_deleted_after_max_misses(motion_model):
    manager = make_manager(motion_model=motion_model)
    run_target(manager, [8.0, 1.0, 1.0], [1.0, 0.0, 0.0], n_frames=5)
    assert len(manager.tracks) == 1
    # 5 consecutive empty frames kill the track.
    for i in range(5):
        manager.process_scan(0.5 + i * 0.1, [])
    assert len(manager.tracks) == 0


@pytest.mark.parametrize('motion_model', MOTION_MODELS)
def test_track_survives_brief_misses(motion_model):
    manager = make_manager(motion_model=motion_model)
    run_target(manager, [8.0, 1.0, 1.0], [1.0, 0.0, 0.0],
               n_frames=30, drop={10, 11, 17, 24})
    assert len(manager.tracks) == 1
    assert manager.tracks[0].state is TrackState.CONFIRMED
    assert manager.tracks[0].track_id == 1


@pytest.mark.parametrize('motion_model', MOTION_MODELS)
def test_far_detection_spawns_new_track_not_association(motion_model):
    manager = make_manager(motion_model=motion_model)
    run_target(manager, [8.0, 0.0, 1.0], [1.0, 0.0, 0.0], n_frames=5)
    assert len(manager.tracks) == 1
    # A detection 10 m away must fail the gate and start its own track.
    far = detection_from_cartesian([8.5, 10.0, 1.0], [0.0, 0.0, 0.0])
    manager.process_scan(0.5, [far])
    assert len(manager.tracks) == 2
    assert manager.tracks[1].state is TrackState.TENTATIVE


@pytest.mark.parametrize('motion_model', MOTION_MODELS)
def test_clutter_does_not_steal_confirmed_track(motion_model):
    rng = np.random.default_rng(31)
    manager = make_manager(motion_model=motion_model)
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


@pytest.mark.parametrize('motion_model', MOTION_MODELS)
def test_two_targets_tracked_independently(motion_model):
    manager = make_manager(motion_model=motion_model)
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


@pytest.mark.parametrize('motion_model', MOTION_MODELS)
def test_confidence_reflects_hit_ratio(motion_model):
    manager = make_manager(motion_model=motion_model)
    run_target(manager, [8.0, 1.0, 1.0], [1.0, 0.0, 0.0],
               n_frames=20, drop={5, 10})
    track = manager.tracks[0]
    assert track.confidence == track.hits / (track.hits + track.misses)
    assert 0.8 < track.confidence < 1.0


@pytest.mark.parametrize('motion_model,state_dim,filter_type', [
    ('cv', 6, ConstantVelocityEKF),
    ('imm', 9, IMMFilter),
])
def test_motion_model_selects_the_filter(motion_model, state_dim, filter_type):
    manager = make_manager(motion_model=motion_model)
    run_target(manager, [8.0, 1.0, 1.0], [1.0, 0.0, 0.0], n_frames=3)
    ekf = manager.tracks[0].ekf
    assert isinstance(ekf, filter_type)
    assert ekf.x.shape == (state_dim,)
    assert ekf.P.shape == (state_dim, state_dim)


def test_unknown_motion_model_raises():
    with pytest.raises(ValueError):
        make_manager(motion_model='coordinated_turn')


def figure_eight_truth(amplitude, period, center=(8.0, 0.0, 3.0)):
    """Gerono lemniscate matching radar_simulator.trajectories.FigureEight.

    Inlined rather than imported: radar_simulator already depends on
    target_tracker, so importing it here would make the package graph circular.
    """
    center = np.asarray(center, dtype=float)
    omega = 2 * np.pi / period

    def truth(t):
        pos = center + np.array([
            amplitude * np.sin(omega * t),
            amplitude * np.sin(omega * t) * np.cos(omega * t),
            0.0,
        ])
        vel = np.array([
            amplitude * omega * np.cos(omega * t),
            amplitude * omega * np.cos(2 * omega * t),
            0.0,
        ])
        return pos, vel

    return truth


def run_figure_eight(motion_model, amplitude, period, seed, n_frames=600):
    """Fly a noisy figure-eight and report which track was on the target."""
    truth = figure_eight_truth(amplitude, period)
    rng = np.random.default_rng(seed)
    sigmas = (0.1, 0.02, 0.02, 0.1)
    manager = TrackManager(motion_model=motion_model, confirm_hits=3,
                           max_misses=8, sigma_accel=3.0)

    target_ids = []
    for i in range(n_frames):
        t = i * 0.1
        pos, vel = truth(t)
        detections = []
        if rng.random() < 0.9:  # 10% detection dropout, as in sim_params.yaml
            detections.append(tuple(np.asarray(detection_from_cartesian(pos, vel))
                                    + rng.normal(0.0, sigmas)))
        tracks = manager.process_scan(t, detections)
        on_target = [tr for tr in tracks
                     if np.linalg.norm(tr.ekf.x[:3] - pos) < 1.0]
        if on_target:
            closest = min(on_target,
                          key=lambda tr: np.linalg.norm(tr.ekf.x[:3] - pos))
            target_ids.append(closest.track_id)
    return target_ids, manager


def test_sharp_figure_eight_holds_its_id_under_both_motion_models():
    """A maneuvering target keeps one ID whichever motion model is selected.

    This test used to assert the IMM churned through fewer IDs than 'cv'
    (~25 against ~52). Spawn suppression and merging removed that churn at
    its source, so at this trajectory both models now hold a single ID and
    the comparison no longer discriminates. The IMM's actual contribution —
    predicting through a turn instead of extrapolating past it — is measured
    directly in test_imm.py::test_beats_cv_on_a_sharp_figure_eight.

    Peak lateral acceleration here is ~3.1 m/s^2. A handful of IDs is
    tolerated because a deep detection dropout mid-turn can still legitimately
    end a track; what must not come back is the 25-50 ID churn. 'cv' gets the
    looser bound: it drifts further through a turn, so its track falls outside
    the suppression allowance more often. Measured over these seeds, cv peaks
    at 3 and imm at 2.
    """
    bounds = {'cv': 4, 'imm': 2}
    for motion_model, bound in bounds.items():
        counts = [
            len(set(run_figure_eight(motion_model, amplitude=5.0,
                                     period=8.0, seed=seed)[0]))
            for seed in (1, 2, 3, 4, 5)
        ]
        assert max(counts) <= bound, f'{motion_model}: {counts}'


def test_figure_eight_target_is_never_left_uncovered():
    """Some track is always on the target.

    Coverage was already essentially total when the tracker was still churning
    through IDs — the drone was never lost, only renamed. This test guards that
    property against the duplicate suppression added on top: refusing to spawn
    near a live track, and merging tracks that converge, both risk paying for
    stable IDs with dropouts. Coverage must stay total, not merely improve on
    the ID count.
    """
    n_frames = 600
    for seed in (1, 2, 3):
        target_ids, _ = run_figure_eight('imm', amplitude=3.0,
                                         period=20.0, seed=seed, n_frames=n_frames)
        assert len(target_ids) > 0.95 * n_frames


@pytest.mark.parametrize('motion_model', MOTION_MODELS)
def test_gated_out_detection_does_not_spawn_a_twin(motion_model):
    """The duplicate-ID mechanism itself: same input, suppression on vs off.

    gate_chi2 is the chi-square 95th percentile, so a target's own detections
    fail association about 5% of the time. Spawning on those births a second
    track on top of the object that just rejected it.
    """
    def run(spawn_gate_chi2):
        manager = make_manager(motion_model=motion_model,
                               spawn_gate_chi2=spawn_gate_chi2, merge_chi2=0.0)
        run_target(manager, [8.0, 0.0, 1.0], [1.0, 0.0, 0.0], n_frames=5)
        assert manager.tracks[0].state is TrackState.CONFIRMED
        # Offset far enough to fail the association gate, close enough that it
        # is plainly the same object.
        nearby = detection_from_cartesian([8.5, 0.9, 1.0], [1.0, 0.0, 0.0])
        assert manager.tracks[0].ekf.mahalanobis(np.asarray(nearby)) > manager.gate_chi2
        manager.process_scan(0.5, [nearby])
        return manager

    assert len(run(0.0).tracks) == 2      # disabled: the old twin-spawning
    assert len(run(30.0).tracks) == 1     # enabled: no twin


@pytest.mark.parametrize('motion_model', MOTION_MODELS)
def test_duplicate_tracks_merge_onto_the_lower_id(motion_model):
    """Twins born before either confirmed are collapsed by the merge pass.

    Spawn suppression cannot catch these — there was no confirmed track to
    suppress against yet — so the merge pass is the second line of defence.
    The surviving ID must be the older one, since that is the identity
    downstream consumers have already seen.
    """
    manager = make_manager(motion_model=motion_model,
                           spawn_gate_chi2=0.0, merge_chi2=25.0)
    pos = np.array([8.0, 0.0, 1.0])
    vel = np.array([1.0, 0.0, 0.0])
    for i in range(12):
        # Two detections per frame on essentially the same object: one feeds
        # the live track, the other spawns a competitor.
        manager.process_scan(i * 0.1, [
            detection_from_cartesian(pos, vel),
            detection_from_cartesian(pos + [0.0, 0.12, 0.0], vel),
        ])
        pos = pos + vel * 0.1

    confirmed = [t for t in manager.tracks if t.state is TrackState.CONFIRMED]
    assert len(confirmed) == 1
    assert confirmed[0].track_id == 1
    assert np.linalg.norm(confirmed[0].ekf.x[:3] - pos) < 1.0


@pytest.mark.parametrize('motion_model', MOTION_MODELS)
def test_two_targets_are_never_merged(motion_model):
    """Merging must not collapse genuinely distinct targets.

    The guard for the merge threshold: two targets 6 m apart measure a state
    distance in the thousands, against a merge_chi2 of 25.
    """
    manager = make_manager(motion_model=motion_model)
    pos_a = np.array([8.0, 3.0, 1.0])
    pos_b = np.array([8.0, -3.0, 1.0])
    for i in range(30):
        manager.process_scan(i * 0.1, [
            detection_from_cartesian(pos_a, [0.0, 0.0, 0.0]),
            detection_from_cartesian(pos_b, [0.0, 0.0, 0.0]),
        ])
    confirmed = [t for t in manager.tracks if t.state is TrackState.CONFIRMED]
    assert len(confirmed) == 2
    assert manager._state_distance(*confirmed) > 10 * manager.merge_chi2


def test_figure_eight_holds_a_single_track_id():
    """The headline: one drone keeps one ID for a whole 60 s run.

    Before spawn suppression and merging this run churned through ~5 IDs and
    spent a third of its frames showing two confirmed tracks on the one drone.
    """
    for motion_model in ('cv', 'imm'):
        for seed in (1, 2, 3):
            target_ids, _ = run_figure_eight(motion_model, amplitude=3.0,
                                             period=20.0, seed=seed)
            assert set(target_ids) == {1}


def test_duplicate_suppression_can_be_disabled():
    """Setting both thresholds to 0 restores the pre-suppression behaviour.

    The rollback path: if suppression misbehaves on real hardware it can be
    turned off from the YAML without a code change, exactly as motion_model
    can be dropped back to 'cv'.
    """
    manager = make_manager(motion_model='imm',
                           spawn_gate_chi2=0.0, merge_chi2=0.0)
    run_target(manager, [8.0, 0.0, 1.0], [1.0, 0.0, 0.0], n_frames=5)
    # The gated-out detection that the shipped defaults suppress spawns a
    # second track again once both mechanisms are off.
    nearby = detection_from_cartesian([8.5, 0.9, 1.0], [1.0, 0.0, 0.0])
    manager.process_scan(0.5, [nearby])
    assert len(manager.tracks) == 2
