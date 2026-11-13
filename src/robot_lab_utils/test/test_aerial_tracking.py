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
