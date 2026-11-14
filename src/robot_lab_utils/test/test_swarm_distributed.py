"""Distributed swarm behaviours, sensed coverage and member-loss replanning (R10.3)."""
import math
from types import MethodType, SimpleNamespace

import numpy as np
from PIL import Image
import pytest
import yaml

from robot_lab_utils.fleet_controller import FleetController, Member
from robot_lab_utils.fleet_missions import MissionHistory, mission
from robot_lab_utils.fleet_motion import Neighbor
from robot_lab_utils.fleet_session import SpawnGrid
from robot_lab_utils.swarm_behaviours import BURGER, LocalBehaviour, robots_in_scan, scan_rays
from robot_lab_utils.swarm_coverage import SensedCoverage, reachable_from, redistribute

INC = math.radians(1.)


def scan(objects=(), default=math.inf, n=360, angle_min=-math.pi):
    """objects: [(centre_deg, width_m, range_m)] as arcs in a 1° scan."""
    ranges = [default] * n
    for centre, width, distance in objects:
        half = math.degrees(math.atan2(width / 2, distance))
        for i in range(n):
            angle = math.degrees(angle_min + i * INC)
            if abs((angle - centre + 180) % 360 - 180) <= half:
                ranges[i] = distance
    return ranges, angle_min, INC


def test_robot_recognition_uses_physical_width():
    rays = scan_rays(*scan([(0., .3, 1.), (90., 3., 1.5)]))
    found = robots_in_scan(rays, BURGER['aggregation'])
    assert len(found) == 1 and abs(found[0][0] - 1.) < .05 and abs(found[0][1]) < .05  # the wall is not a robot


def test_dispersion_moves_away_from_a_close_peer_and_cruises_alone():
    behaviour = LocalBehaviour('local_dispersion', .25, .6)
    linear, angular, info = behaviour.step(*scan([(90., .3, .5)]), now=0.)
    assert angular < 0 and info['state'] == 'repelling'           # peer on the left: turn right
    linear, angular, info = behaviour.step(*scan(), now=1.)
    assert info['state'] == 'alone' and linear == pytest.approx(.2) and angular == 0.


def test_attraction_turns_towards_a_peer():
    behaviour = LocalBehaviour('local_attraction', .25, .6)
    _linear, angular, info = behaviour.step(*scan([(60., .3, 1.5)]), now=0.)
    assert angular > 0 and info['state'] == 'attracting'


def test_aggregation_explores_joins_stays_and_leaves():
    behaviour = LocalBehaviour('local_aggregation', .25, .6)
    linear, _, info = behaviour.step(*scan(), now=0.)
    assert info['state'] == 'explore' and linear == .25
    linear, angular, info = behaviour.step(*scan([(30., .3, 2.)]), now=1.)
    assert info['state'] == 'join' and linear > 0 and angular > 0
    linear, angular, info = behaviour.step(*scan([(0., .3, .6)]), now=2.)
    assert info['state'] == 'stay' and linear == angular == 0.
    stay = 2 * BURGER['aggregation']['time_scale_s'] + BURGER['aggregation']['base_stay_s']
    assert behaviour.step(*scan([(0., .3, .6)]), now=2. + stay - .1)[2]['state'] == 'stay'
    linear, _, info = behaviour.step(*scan([(0., .3, .6)]), now=2. + stay + .1)
    assert info['state'] == 'leave' and linear < 0                  # backs away from the group
    leave_end = 2. + stay + .1 + 2 * BURGER['aggregation']['base_stay_s']
    assert behaviour.step(*scan([(0., .3, .6)]), now=leave_end + .1)[2]['state'] == 'explore'   # bounded leave


def test_flocking_zones():
    behaviour = LocalBehaviour('local_flocking', .25, .6)
    _, angular, info = behaviour.step(*scan([(90., .3, 2.)]), now=0.)
    assert info['state'] == 'cohesion' and angular > 0              # peer on the left in zone 3
    behaviour = LocalBehaviour('local_flocking', .25, .6)
    _, angular, info = behaviour.step(*scan([(0., 1., .3)]), now=0.)
    assert info['state'] == 'avoid' and angular != 0
    behaviour = LocalBehaviour('local_flocking', .25, .6)
    linear, _, info = behaviour.step(*scan(), now=0.)
    assert info['state'] == 'forward' and linear == pytest.approx(.2)


def test_flocking_thresholds_scale_with_scan_resolution():
    coarse = LocalBehaviour('local_flocking', .25, .6)
    ranges, angle_min, _ = scan([(90., .3, 2.)])
    _, angular, info = coarse.step(ranges[::2], angle_min, 2 * INC, now=0.)
    assert info['state'] == 'cohesion' and angular > 0


def open_map(tmp_path, size=(120, 120), walls=()):
    free = np.ones(size, dtype=bool)
    free[0, :] = free[-1, :] = free[:, 0] = free[:, -1] = False
    for r0, r1, c0, c1 in walls:
        free[r0:r1, c0:c1] = False
    Image.fromarray(np.where(free, 254, 0).astype(np.uint8)[::-1]).save(tmp_path / 'map.png')
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump(dict(image='map.png', resolution=.05, origin=[0., 0., 0.],
                                                           negate=0, occupied_thresh=.65, free_thresh=.196)))
    return SpawnGrid(tmp_path / 'map.yaml')


def test_sensed_coverage_counts_cells_seen_by_rays(tmp_path):
    grid = open_map(tmp_path)
    area = SensedCoverage(grid, (3., 3.), 4., 4., sensor_range=1.)
    assert area.fraction() == 0.
    ranges = [math.inf] * 360
    area.integrate(3., 3., 0., ranges, -math.pi, INC, range_max=8.)
    seen = area.fraction()
    assert math.pi * 1 ** 2 / 16 * .8 < seen < math.pi * 1 ** 2 / 16 * 1.2   # a 1 m disc of a 4 x 4 m area
    # A short return stops the ray: the cells behind an obstacle stay unseen.
    blocked = SensedCoverage(grid, (3., 3.), 4., 4., sensor_range=1.)
    blocked.integrate(3., 3., 0., [.3] * 360, -math.pi, INC)
    assert blocked.fraction() < seen / 5


def test_next_goal_prefers_near_unseen_and_avoids_held_goals(tmp_path):
    grid = open_map(tmp_path)
    area = SensedCoverage(grid, (3., 3.), 4., 4., sensor_range=1.)
    area.integrate(3., 3., 0., [math.inf] * 360, -math.pi, INC, range_max=8.)
    goal = area.next_goal((3., 3.), .2)
    assert .9 <= math.hypot(goal['x'] - 3., goal['y'] - 3.) <= 1.3
    other = area.next_goal((3., 3.), .2, held=[(goal['x'], goal['y'])])
    assert math.hypot(other['x'] - goal['x'], other['y'] - goal['y']) >= 1.5


def test_reachability_and_redistribution(tmp_path):
    grid = open_map(tmp_path, walls=[(0, 120, 58, 62)])           # wall splits west/east at x ~3 m
    west = reachable_from(grid, (1.5, 3.), .2)
    col, row = grid.cell(4.5, 3.)
    assert not west[row, col]
    plan = redistribute([dict(x=0., y=0.), dict(x=5., y=5.)], dict(a=(0., 1.), b=(5., 4.)))
    assert plan == dict(a=[dict(x=0., y=0.)], b=[dict(x=5., y=5.)])


def controller(names=('alpha', 'beta', 'gamma')):
    node = SimpleNamespace(history=MissionHistory(), serial_traffic=False, events=[], lost_members=set(),
                           swarm=None, sensed=None, coverage_group=None)
    for name in ('_replan_loss', '_stop_member', '_clear_queue', '_cancel_handle'):
        setattr(node, name, MethodType(getattr(FleetController, name), node))
    node._event = lambda event, **data: node.events.append((event, data))
    node.members = {}
    for index, name in enumerate(names):
        member = Member(config=dict(id=name, radius=.2))
        member.publisher = SimpleNamespace(publish=lambda value: None)
        node.members[name] = member
    positions = {name: Neighbor(name, float(i), 0., 0., 0., .2) for i, name in enumerate(names)}
    node._neighbor = lambda member: positions[member.name]
    return node


def test_lost_coverage_member_goals_go_to_the_nearest_remaining_member():
    node = controller()
    node.coverage_group = dict(targets=['alpha', 'beta', 'gamma'], loss_policy='replan')
    for name in node.members:
        node.members[name].mode = 'waiting'
    node.members['gamma'].queue = [mission(dict(x=2.1, y=0., yaw=0.), 'coverage'),
                                   mission(dict(x=-.5, y=0., yaw=0.), 'coverage')]
    fresh = dict(alpha=True, beta=True, gamma=False)
    node.members['alpha'].mode = 'navigation'
    assert node._replan_loss(['gamma'], fresh, 10.)
    assert node.lost_members == {'gamma'} and not node.members['gamma'].included
    assert node.members['gamma'].lost_pose.x == 2.
    assert [i['goal']['x'] for i in node.members['beta'].queue] == [2.1]
    assert [i['goal']['x'] for i in node.members['alpha'].queue] == [-.5]
    assert node.coverage_group['targets'] == ['alpha', 'beta']
    assert node.events[-1][0] == 'member_lost_replanned'


def test_stop_policy_or_too_few_members_or_outside_motion_stop_everything():
    node = controller()
    node.coverage_group = dict(targets=['alpha', 'beta', 'gamma'], loss_policy='stop')
    assert not node._replan_loss(['gamma'], dict(alpha=True, beta=True, gamma=False), 0.)
    node = controller()
    node.swarm = dict(targets=['alpha', 'beta'], loss_policy='replan', local=None)
    assert not node._replan_loss(['beta'], dict(alpha=True, beta=False, gamma=True), 0.)   # a swarm needs two
    node = controller()
    node.swarm = dict(targets=['alpha', 'beta', 'gamma'], loss_policy='replan', local=None)
    node.members['alpha'].mode = node.members['beta'].mode = 'swarm'
    outside = controller(('delta',)).members['delta']
    outside.mode = 'manual'
    node.members['delta'] = outside
    assert not node._replan_loss(['gamma'], dict(alpha=True, beta=True, gamma=False, delta=True), 0.)
    del node.members['delta']
    assert node._replan_loss(['gamma'], dict(alpha=True, beta=True, gamma=False), 0.)
    assert node.swarm['targets'] == ['alpha', 'beta']


def test_protection_layer_overrides_a_behaviour_near_obstacles():
    behaviour = LocalBehaviour('local_aggregation', .25, .6, radius=.12)
    linear, angular, info = behaviour.step(*scan([(0., 1.5, .2)]), now=0.)   # wall 0.2 m ahead
    assert info.get('protected') and linear < .25
    free = LocalBehaviour('local_aggregation', .25, .6, radius=.12)
    assert not free.step(*scan(), now=0.)[2].get('protected')


def test_laser_yaw_follows_the_joint_chain():
    from robot_lab_utils.fleet_session import laser_yaw
    urdf = ('<robot name="r"><link name="base_footprint"/><link name="base_link"/><link name="mount"/><link name="laser_link"/>'
            '<joint name="a" type="fixed"><parent link="base_footprint"/><child link="base_link"/><origin rpy="0 0 0"/></joint>'
            '<joint name="b" type="fixed"><parent link="base_link"/><child link="mount"/><origin rpy="0 0 1.0"/></joint>'
            '<joint name="c" type="fixed"><parent link="mount"/><child link="laser_link"/><origin rpy="0 0 2.14"/></joint>'
            '<gazebo reference="laser_link"><sensor name="lidar" type="gpu_lidar"/></gazebo></robot>')
    assert laser_yaw(urdf) == pytest.approx(3.14)
    assert laser_yaw('<robot name="r"><link name="base_link"/></robot>') == 0.


def test_wall_fragments_are_not_robots():
    # A wall 1 m away along y=1: grazing rays jump in range, splitting it into short pieces.
    ranges = [math.inf] * 360
    for i in range(360):
        a = -math.pi + i * INC
        if math.sin(a) > .05:
            ranges[i] = min(12., 1. / math.sin(a))
    rays = scan_rays(ranges, -math.pi, INC)
    assert robots_in_scan(rays, BURGER['aggregation']) == []
    for i in range(163, 198):
        ranges[i] = .5                              # a 0.3 m peer ahead, well in front of the wall
    rays = scan_rays(ranges, -math.pi, INC)
    assert len(robots_in_scan(rays, BURGER['aggregation'])) == 1
