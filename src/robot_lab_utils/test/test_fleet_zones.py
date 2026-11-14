"""Shared-zone reservations (R10.2): passages from a map, grants and holds."""
import math

import numpy as np
from PIL import Image
import yaml

from robot_lab_utils.fleet_session import SpawnGrid
from robot_lab_utils.fleet_zones import ZoneArbiter, declared_zones, narrow_passages, plan_entry


def corridor_map(tmp_path, width_m=.7):
    """Two 4 x 4 m rooms joined by a 3 m corridor of the given width (5 cm cells)."""
    res = .05
    free = np.zeros((100, 240), dtype=bool)          # 5 m x 12 m, rows = y
    free[10:90, 10:90] = True                         # west room x 0.5..4.5
    free[10:90, 150:230] = True                       # east room x 7.5..11.5
    half = int(round(width_m / res / 2))
    free[50 - half:50 + half, 90:150] = True          # corridor x 4.5..7.5 at y 2.5
    image = np.where(free, 254, 0).astype(np.uint8)[::-1]
    Image.fromarray(image).save(tmp_path / 'map.png')
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump(dict(image='map.png', resolution=res, origin=[0., 0., 0.],
                                                           negate=0, occupied_thresh=.65, free_thresh=.196)))
    return SpawnGrid(tmp_path / 'map.yaml')


def line(x0, x1, y=2.5, step=.05):
    n = int(abs(x1 - x0) / step) + 1
    return [(x0 + (x1 - x0) * i / (n - 1), y) for i in range(n)]


def test_corridor_becomes_one_single_lane_zone(tmp_path):
    grid = corridor_map(tmp_path)
    zones = narrow_passages(grid, .18)
    corridor = [z for z in zones if z.contains(6., 2.5)]
    assert len(corridor) == 1
    zone = corridor[0]
    assert zone.kind == 'passage' and not zone.contains(2.5, 2.5) and not zone.contains(9.5, 2.5)
    x0, x1, _, _ = zone.bounds
    assert x0 < 4.8 and x1 > 7.2


def test_wide_corridor_is_not_single_lane(tmp_path):
    grid = corridor_map(tmp_path, width_m=1.6)
    assert not [z for z in narrow_passages(grid, .18) if z.contains(6., 2.5)]


def test_declared_doors_docks_and_zones():
    config = dict(shared_zones=dict(ramp=dict(rects=[[0, 1, 0, 2]], holding=[dict(x=-1, y=1)])),
                  doors=dict(main=dict(sweep=[-.5, .5, -1, 1]), lift=dict(sweep=[2, 3, 0, 1], z_joint='car'),
                             landing=dict(sweep=[2, 3, 0, 1], landing_floor='L0')),
                  docks=dict(west=dict(x=4., y=0.)))
    zones = {z.name: z for z in declared_zones(config)}
    assert set(zones) == {'ramp', 'door:main', 'dock:west'}  # the lift car has its own reservation
    assert zones['ramp'].holding == [dict(x=-1., y=1., yaw=0.)]
    assert zones['dock:west'].contains(4.5, 0., .2) and not zones['dock:west'].contains(5.5, 0., .2)


def test_plan_entry_distance():
    zone = declared_zones(dict(shared_zones=dict(z=dict(rects=[[5, 7, 2, 3]]))))[0]
    assert abs(plan_entry(zone, line(2., 9.), (2., 2.5), .2) - 2.8) < .11
    assert plan_entry(zone, line(2., 9.), (2., 2.5), .2, lookahead=2.) is None
    assert plan_entry(zone, line(2., 4.), (2., 2.5), .2) is None


def rows(**members):
    out = {}
    for name, (x, plan, extra) in members.items():
        out[name] = dict(x=x, y=2.5, radius=.18, floor='', plan=plan, navigating=plan is not None, **extra)
    return out


def test_head_on_corridor_one_member_waits_then_proceeds(tmp_path):
    grid = corridor_map(tmp_path)
    arbiter = ZoneArbiter(narrow_passages(grid, .18))
    corridor = next(z.name for z in arbiter.zones.values() if z.contains(6., 2.5))
    # alpha is inside the corridor heading east; beta approaches from the east.
    state = arbiter.update(rows(alpha=(5.5, line(5.5, 10.), {}), beta=(8.3, line(8.3, 2.), {})))
    assert state['grants'][corridor] == 'alpha'
    assert state['waits']['beta']['zone'] == corridor and state['waits']['beta']['holder'] == 'alpha'
    # beta is now held (its task queued with a zone wait); alpha leaves the corridor.
    waiting = dict(waiting_for=corridor, waiting_since=1.)
    state = arbiter.update(rows(alpha=(7.0, line(7.0, 10.), {}), beta=(8.3, None, waiting)))
    assert 'beta' in state['waits']
    state = arbiter.update(rows(alpha=(9.5, line(9.5, 10.), {}), beta=(8.3, None, waiting)))
    assert state['grants'][corridor] == 'beta' and 'beta' not in state['waits']


def test_distant_claim_does_not_take_the_grant(tmp_path):
    arbiter = ZoneArbiter(narrow_passages(corridor_map(tmp_path), .18))
    corridor = next(z.name for z in arbiter.zones.values() if z.contains(6., 2.5))
    # alpha plans through the corridor from 4 m away; beta reaches the east entry first.
    state = arbiter.update(rows(alpha=(1.0, line(1.0, 10.), {}), beta=(8.2, line(8.2, 2.), {})))
    assert state['grants'][corridor] == 'beta' and not state['waits']


def test_parked_member_holds_its_zone_and_waiters_keep_order(tmp_path):
    arbiter = ZoneArbiter(narrow_passages(corridor_map(tmp_path), .18))
    corridor = next(z.name for z in arbiter.zones.values() if z.contains(6., 2.5))
    members = rows(parked=(6.0, None, {}), late=(8.3, line(8.3, 2.), {}),
                   early=(4.0, None, dict(waiting_for=corridor, waiting_since=0.)))
    members['early']['x'] = 3.8
    state = arbiter.update(members)
    assert state['grants'][corridor] == 'parked' and {'late', 'early'} <= set(state['waits'])
    del members['parked']
    state = arbiter.update(members)
    assert state['grants'][corridor] == 'early' and 'late' in state['waits']


def test_floors_separate_zones():
    zone = declared_zones(dict(shared_zones=dict(z=dict(rects=[[0, 2, 0, 2]], floor='L1'))))[0]
    arbiter = ZoneArbiter([zone])
    state = arbiter.update(dict(a=dict(x=1., y=1., radius=.2, floor='L0', plan=None, navigating=False)))
    assert not state['occupants'] and not state['grants']


def test_two_members_inside_are_reported_not_held():
    zone = declared_zones(dict(shared_zones=dict(z=dict(rects=[[0, 4, 0, 1]]))))[0]
    state = ZoneArbiter([zone]).update(dict(
        a=dict(x=1., y=.5, radius=.2, plan=[(1., .5), (3., .5)], navigating=True),
        b=dict(x=3., y=.5, radius=.2, plan=[(3., .5), (1., .5)], navigating=True)))
    assert state['shared'] == ['z'] and not state['waits']
    assert math.isfinite(len(state['occupants']['z']))
