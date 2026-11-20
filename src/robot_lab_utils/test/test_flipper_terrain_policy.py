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


def test_full_press_climb_levers_with_both_pairs(step):
    policy = FlipperPolicy(full_press=True)
    assert policy.update(terrain_ahead(step, HFOV, CAM, GROUND), 0., 0., 0.)['elevations'][1] == PRESS_RAD
    out = policy.update(terrain_ahead(step, HFOV, CAM, GROUND), .25, 0., 1.)
    assert out['state'] == 'climb' and out['targets'] == [.8, -.8, .8, -.8]


def test_drop_descends_and_roll_holds(ledge, flat):
    policy = FlipperPolicy()
    # A 0.18 m stair down shows its lower tread in the band: descend. The
    # 0.3 m ledge's floor is out of view, which since 2026-11-20 stops forward
    # drive instead (the vehicle drove off an unseen 0.9 m drop and flipped).
    stair = render(lambda x: -.18 if x > .9 else 0.)
    out = policy.update(terrain_ahead(stair, HFOV, CAM, GROUND), 0., 0., 0.)
    assert out['state'] == 'descend' and out['elevations'] == (PRESS_RAD, REST_RAD)
    assert FlipperPolicy().update(terrain_ahead(ledge, HFOV, CAM, GROUND), 0., 0., 0.)['state'] == 'ledge'
    out = policy.update(terrain_ahead(flat, HFOV, CAM, GROUND), 0., .4, 1.)
    assert out['state'] == 'hold' and out['stop_drive']
    assert policy.update(terrain_ahead(flat, HFOV, CAM, GROUND), 0., .2, 2.)['state'] == 'travel'


def test_missing_depth_falls_back_to_rest_on_level_ground():
    policy = FlipperPolicy()
    assert policy.update(None, 0., 0., 0.)['state'] == 'travel'
    with pytest.raises(ValueError):
        policy.update(None, float('nan'), 0., 1.)


def test_level_tread_on_a_flight_ends_the_climb_toward_the_next_riser():
    # terrain_stairs_flight (2026-11-20): after the first riser the next one is
    # always in view; a level hull must still leave the fully pressed climb.
    policy = FlipperPolicy(full_press=True)
    riser = dict(rise_m=.18, drop=False, points=200, nearest_rise_m=.8)
    assert policy.update(riser, .3, 0., 0.)['state'] == 'climb'
    assert policy.update(riser, 0., 0., 1.)['state'] == 'climb'      # level, hold not yet elapsed
    out = policy.update(riser, 0., 0., 2.1)
    from robot_lab_utils.tracked_flippers import FULL_PRESS_RAD
    assert out["state"] == "approach" and out["elevations"][1] != FULL_PRESS_RAD


def test_a_ledge_deeper_than_a_stair_stops_forward_drive_but_a_stair_descends():
    policy = FlipperPolicy(full_press=True)
    unseen = FlipperPolicy().update(dict(rise_m=0., drop=True, drop_m=None, step_down_seen=False, points=12,
                                         nearest_rise_m=None), 0., 0., 0.)
    assert unseen['state'] == 'ledge' and unseen['stop_forward']   # floor below out of view
    cliff = policy.update(dict(rise_m=0., drop=True, drop_m=.9, step_down_seen=True, points=200, nearest_rise_m=None), 0., 0., 0.)
    assert cliff['state'] == 'ledge' and cliff['stop_forward'] and not cliff['stop_drive']
    assert policy.update(None, 0., 0., 1.)['state'] == 'travel'       # ledge out of view: free again
    stair = policy.update(dict(rise_m=0., drop=True, drop_m=.18, step_down_seen=True, points=200, nearest_rise_m=None), 0., 0., 2.)
    assert stair['state'] == 'descend' and not stair['stop_forward']


def test_a_descending_flight_reads_its_next_step_not_the_ones_beyond():
    flight = render(lambda x: 0. if x < .9 else -.18*min(3, 1+int((x-.9)/1.1)))   # 1.1 m treads
    terrain = terrain_ahead(flight, HFOV, CAM, GROUND)
    assert terrain['step_down_seen'] and terrain['drop_m'] == pytest.approx(.18, abs=.03)
    assert FlipperPolicy().update(terrain, 0., 0., 0.)['state'] == 'descend'


def test_a_measured_stair_edge_stays_a_step_when_the_tread_below_leaves_view():
    policy = FlipperPolicy()
    step = dict(rise_m=0., drop=True, drop_m=.18, step_down_seen=True, points=200, nearest_rise_m=None)
    unseen = dict(rise_m=0., drop=True, drop_m=None, step_down_seen=False, points=12, nearest_rise_m=None)
    assert policy.update(step, 0., 0., 0.)['state'] == 'descend'
    assert policy.update(unseen, 0., 0., 2.)['state'] == 'descend'    # tread below now too close to see
    assert policy.update(unseen, 0., 0., 3.5)['state'] == 'ledge'     # no step seen for 3.5 s: unknown drop


def test_deeper_treads_in_view_at_a_measured_edge_continue_the_descent():
    policy = FlipperPolicy()
    step = dict(rise_m=0., drop=True, drop_m=.18, step_down_seen=True, points=200, nearest_rise_m=None)
    beyond = dict(rise_m=0., drop=True, drop_m=.36, step_down_seen=True, points=12, nearest_rise_m=None)
    assert policy.update(step, 0., 0., 0.)['state'] == 'descend'
    assert policy.update(beyond, 0., 0., 1.)['state'] == 'descend'
    assert FlipperPolicy().update(beyond, 0., 0., 1.)['state'] == 'ledge'   # no step measured first
