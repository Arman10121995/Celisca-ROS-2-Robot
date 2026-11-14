"""Flipper-aware terrain policy from depth and IMU (R10.7)."""
import math

import numpy as np
import pytest

from robot_lab_utils.tracked_flippers import (FlipperPolicy, edge_elevation, joints, terrain_ahead,
                                              REST_RAD, PRESS_RAD)

CAM, HFOV, GROUND = (.05, 0., .34), 1.25, -.0719


def render(height_at, w=80, h=60):
    """Depth image of a level forward camera over terrain z = height_at(x) (base frame, ground line GROUND)."""
    fx = (w / 2) / math.tan(HFOV / 2)
    depth = np.full((h, w), np.inf)
    for v in range(h):
        for u in range(w):
            ray = np.array([1., -(u - (w - 1) / 2) / fx, -(v - (h - 1) / 2) / fx])   # forward, left, up per metre
            for d in np.arange(.1, 8., .02):
                x, y, z = np.array(CAM) + d * ray
                if z <= GROUND + height_at(x):
                    depth[v, u] = d
                    break
    return depth


@pytest.fixture(scope='module')
def flat():
    return render(lambda x: 0.)


@pytest.fixture(scope='module')
def step():
    return render(lambda x: .18 if x > .9 else 0.)


@pytest.fixture(scope='module')
def ledge():
    return render(lambda x: -.3 if x > .9 else 0.)


def test_flat_ground_has_no_rise_or_drop(flat):
    terrain = terrain_ahead(flat, HFOV, CAM, GROUND)
    assert abs(terrain['rise_m']) < .02 and not terrain['drop']


def test_step_ahead_reads_its_height(step):
    terrain = terrain_ahead(step, HFOV, CAM, GROUND)
    assert terrain['rise_m'] == pytest.approx(.18, abs=.02) and 0.85 < terrain['nearest_rise_m'] < 1.0


def test_ledge_ahead_is_a_drop(ledge):
    assert terrain_ahead(ledge, HFOV, CAM, GROUND)['drop']


def test_joint_conversion_matches_source_poses():
    assert joints(REST_RAD, REST_RAD) == [0., 0., 0., 0.]
    assert joints(0., 0.) == [.5, -.5, .5, -.5]                  # both horizontal
    assert joints(-1., 2.) == [.8, .8, .8, .8]                   # clipped to the joint limits


def test_step_sequence_approach_climb_travel(step, flat):
    policy = FlipperPolicy()
    out = policy.update(terrain_ahead(step, HFOV, CAM, GROUND), 0., 0., 0.)
    assert out['state'] == 'approach' and out['elevations'][1] == PRESS_RAD
    assert out['elevations'][0] == pytest.approx(max(REST_RAD, edge_elevation(out['rise_m'])), abs=1e-3)
    out = policy.update(terrain_ahead(step, HFOV, CAM, GROUND), .25, 0., 1.)
    assert out['state'] == 'climb' and out['elevations'] == (REST_RAD, PRESS_RAD)
    assert policy.update(terrain_ahead(flat, HFOV, CAM, GROUND), .02, 0., 2.)['state'] == 'climb'   # level, not yet held
    assert policy.update(terrain_ahead(flat, HFOV, CAM, GROUND), .02, 0., 3.1)['state'] == 'travel'


def test_drop_descends_and_roll_holds(ledge, flat):
    policy = FlipperPolicy()
    out = policy.update(terrain_ahead(ledge, HFOV, CAM, GROUND), 0., 0., 0.)
    assert out['state'] == 'descend' and out['elevations'] == (PRESS_RAD, REST_RAD)
    out = policy.update(terrain_ahead(flat, HFOV, CAM, GROUND), 0., .4, 1.)
    assert out['state'] == 'hold' and out['stop_drive']
    assert policy.update(terrain_ahead(flat, HFOV, CAM, GROUND), 0., .2, 2.)['state'] == 'travel'


def test_missing_depth_falls_back_to_rest_on_level_ground():
    policy = FlipperPolicy()
    assert policy.update(None, 0., 0., 0.)['state'] == 'travel'
    with pytest.raises(ValueError):
        policy.update(None, float('nan'), 0., 1.)
