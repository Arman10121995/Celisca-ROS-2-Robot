"""Moving-obstacle tracking, prediction and time conflicts (R5.11)."""
import math

import numpy as np
import pytest

from robot_lab_utils import aerial_tracking as at


def box_points(centre, half=.3, step=.1):
    axis = np.arange(-half, half + 1e-9, step)
    gx, gy, gz = np.meshgrid(axis, axis, axis)
    return np.column_stack([gx.ravel(), gy.ravel(), gz.ravel()]) + np.asarray(centre)


def test_clusters_separate_objects_and_ignore_the_floor():
    floor = np.column_stack([np.random.RandomState(0).uniform(-5, 5, (2000, 2)), np.zeros(2000)])
    points = np.vstack([box_points((0, 0, 1.5)), box_points((3, 0, 1.5)), floor])
    clusters = at.cluster_points(points, floor_z=0.)
    assert len(clusters) == 2
    centres = sorted(c[0][0] for c in clusters)
    assert centres[0] == pytest.approx(0, abs=.05) and centres[1] == pytest.approx(3, abs=.05)


def test_a_crossing_box_is_tracked_as_moving_with_its_velocity():
    tracker = at.Tracker()
    for k in range(8):
        t = k * .2
        clusters = at.cluster_points(np.vstack([box_points((-2 + 1. * t, 1, 1.5)), box_points((4, 4, 1.5))]))
        tracker.update(clusters, t)
    moving = tracker.moving()
    assert len(moving) == 1
    assert moving[0].velocity[0] == pytest.approx(1., abs=.15) and abs(moving[0].velocity[1]) < .15
    assert len(tracker.tracks) == 2                                   # the static box is tracked, not moving


def test_lost_tracks_expire():
    tracker = at.Tracker()
    tracker.update(at.cluster_points(box_points((0, 0, 1))), 0.)
    for k in range(1, at.MAX_MISSES + 2):
        tracker.update([], k * .2)
    assert tracker.tracks == []


def make_track(position, velocity, radius=.4):
    track = at.Track(np.asarray(position, float), radius, 0.)
    track.x[3:] = velocity
    track.frames = at.MIN_FRAMES
    return track


def test_time_conflict_finds_the_crossing_time():
    # Drone flies +x at 0.4 m/s from (0,0,2); an obstacle moves -y at 1 m/s and crosses x = 0.8 at t = 2 s.
    track = make_track((.8, 2., 2.), (0., -1., 0.))
    conflict = at.time_conflict((0, 0, 2), [(6, 0, 2)], .4, [track], radius=.5)
    assert conflict is not None and 0.8 < conflict[0] <= 2.0          # within radius before the crossing at 2 s
    behind = make_track((-3., 0., 2.), (0., 1., 0.))                 # moving away behind the drone
    assert at.time_conflict((0, 0, 2), [(6, 0, 2)], .4, [behind], radius=.5) is None


def test_swept_capsules_block_the_corridor_and_the_planner_detours():
    from robot_lab_utils.aerial_planning import plan
    track = make_track((3., -1.5, 2.), (0., 1., 0.), radius=.5)    # sweeps y -1.5 .. 1.5 at x = 3 over 3 s
    predicted = at.PredictedObstacles([track], horizon=3.)
    assert predicted.clearance((3., 0., 2.)) < 0
    assert not predicted.segment_clear((0., 0., 2.), (6., 0., 2.), radius=.7)

    class Open:
        def clearance(self, position):
            return math.inf

    from robot_lab_utils.observed_aerial_scene import ObservedScene
    scene = ObservedScene(Open(), predicted)
    route, details = plan(scene, (0., 0., 2.), (6., 0., 2.), radius=.7, timeout=20.)
    assert details['method'] != 'direct checked segment'
    for a, b in zip(route[:-1], route[1:]):
        assert predicted.segment_clear(a, b, radius=.7)


def test_tracks_are_columns_from_ground_to_head():
    # A walker 1.8 m tall: the drone passing 0.3 m over the head conflicts; 2 m over it does not.
    points = np.vstack([box_points((0., 0., z), half=.15, step=.1) for z in np.arange(.15, 1.8, .3)])
    (centre, radius, count, low, high), = at.cluster_points(points, voxel=.15, min_points=3)
    assert low < .1 and high > 1.75 and radius < .35
    track = at.Track(centre, radius, 0., low, high)
    track.frames = at.MIN_FRAMES
    track.x[3:] = (0., 1., 0.)
    predicted = at.PredictedObstacles([track], horizon=2.)
    assert predicted.clearance((0., 1., 2.1)) < .7                    # just over the head: blocked
    assert predicted.clearance((0., 1., 3.9)) > .7                    # well above: clear
    crossing = at.Track(np.array([0., -1.5, centre[2]]), radius, 0., low, high)
    crossing.frames = at.MIN_FRAMES
    crossing.x[3:] = (0., 1., 0.)                                   # reaches y = 0 at t = 1.5 s
    assert at.time_conflict((-.6, 0., 2.1), [(2., 0., 2.1)], .4, [crossing], radius=.5) is not None
    assert at.time_conflict((-.6, 0., 4.), [(2., 0., 4.)], .4, [crossing], radius=.5) is None


def test_moving_tracks_coast_out_of_view_with_growing_uncertainty():
    tracker = at.Tracker()
    for k in range(6):
        tracker.update(at.cluster_points(box_points((0., -3. + .2 * k, 1.))), k * .2)      # 1 m/s along +y
    (track,) = tracker.moving()
    radius = track.effective_radius
    for k in range(6, 6 + 10):                                         # 2 s unseen: still tracked
        tracker.update([], k * .2)
    assert tracker.moving() == [track] and track.effective_radius > radius + .4
    assert track.position[1] == pytest.approx(-3. + 1. * 3.0, abs=.4)  # coasted on its velocity
    for k in range(16, 16 + int(at.COAST_S / .2) + 2):
        tracker.update([], k * .2)
    assert tracker.tracks == []                                        # dropped after COAST_S unseen


def test_swept_memory_removes_a_walkers_trail_but_keeps_other_obstacles():
    tracker, memory = at.Tracker(), at.SweptMemory()
    for k in range(30):                                       # a person walks y = -3 .. 3 at 1 m/s
        t = k * .2
        tracker.update(at.cluster_points(box_points((0, -3 + t, 1.)), voxel=.3, min_points=3), t)
        memory.record(tracker.moving(), now=t)
    assert memory.version > 0
    trail = np.vstack([box_points((0, y, 1.)) for y in np.arange(-2., 2.5, .5)])   # the map's ghost wall
    pillar = box_points((3, 0, 1.))
    kept, removed = at.remove_swept(np.vstack([trail, pillar]), memory.snapshot()[1])
    assert removed >= .9 * len(trail)
    assert len(kept) >= len(pillar) and np.all(np.abs(kept[:, 0] - 3) < .5)


def test_swept_memory_forgets_and_is_bounded():
    memory = at.SweptMemory(memory_s=10., max_columns=5)
    track = type('T', (), dict(position=np.array([0., 0., 1.]), radius=.3, z_low=0., z_high=1.8))()
    for k in range(10):
        track.position = np.array([k * 1., 0., 1.])
        memory.record([track], now=k)
    assert len(memory.columns) == 5                           # the newest five
    memory.record([], now=30.)
    assert not memory.columns
    assert at.remove_swept(np.zeros((0, 3)), [])[1] == 0


def test_time_conflict_checks_a_hovering_drone():
    track = at.Track(np.array([-3., 0., 1.]), .3, 0., 0., 1.8)
    track.x[3:] = (1.5, 0., 0.)                               # walking at the drone at 1.5 m/s
    track.frames = at.MIN_FRAMES
    assert track.moving()
    conflict = at.time_conflict((0., 0., 1.2), [], 0., [track])
    assert conflict is not None and 1. < conflict[0] < 2.


def test_a_partly_seen_walker_on_the_ground_is_assumed_person_height():
    tracker = at.Tracker(ground_z=0.)
    tracker.update([(np.array([0., 0., .7]), .3, 3, .5, .9)], 0.)       # only the legs/torso in view
    tracker.update([(np.array([3., 3., 6.]), .3, 3, 5.8, 6.2)], 0.)     # an object in the air: unchanged
    grounded, airborne = sorted(tracker.tracks, key=lambda t: t.position[2])
    assert grounded.z_low == 0. and grounded.z_high == at.MIN_STANDING_HEIGHT_M
    assert airborne.z_low == pytest.approx(5.8) and airborne.z_high == pytest.approx(6.2)
