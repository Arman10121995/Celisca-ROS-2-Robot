"""Mission ownership and actual-arrival regressions; no simulation PASS claims."""
import math
from pathlib import Path
import sys
from types import MethodType, SimpleNamespace

import numpy as np
from PIL import Image
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.fleet_missions import MissionHistory, coverage_plan, enqueue, mission, settled
from robot_lab_utils.fleet_session import SpawnGrid


def test_sweep_avoids_unknown_and_walls_and_partitions_without_duplicates(tmp_path):
    image = np.full((100, 100), 255, dtype=np.uint8)
    image[30:45, 45:60] = 0
    image[65:75, 40:50] = 205
    Image.fromarray(image).save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.1,
        origin=[-5., -5., .2], free_thresh=.196, occupied_thresh=.65, negate=0)))
    grid = SpawnGrid(tmp_path / 'map.yaml')
    members = [dict(id='a', x=-2., y=-2., radius=.18), dict(id='b', x=-3., y=-2., radius=.18)]
    plan = coverage_plan(grid, members, (-1., -1.), 4., 4., 1.)
    points = [(g['x'], g['y']) for goals in plan.values() for g in goals]
    assert len(points) == len(set(points))
    assert all(plan.values())
    assert all(grid.clear(x, y, .28) for x, y in points)
    assert image[30, 45] == 0  # The planner does not repair an obstacle away.


def test_disconnected_members_and_unbounded_sweeps_are_rejected(tmp_path):
    image = np.full((100, 100), 255, dtype=np.uint8)
    image[:, 48:52] = 0
    Image.fromarray(image).save(tmp_path / 'map.pgm')
    (tmp_path / 'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.1,
        origin=[-5., -5., 0.], free_thresh=.196, occupied_thresh=.65, negate=0)))
    grid = SpawnGrid(tmp_path / 'map.yaml')
    members = [dict(id='a', x=-2., y=0., radius=.18), dict(id='b', x=2., y=0., radius=.18)]
    with pytest.raises(ValueError, match='connected'):
        coverage_plan(grid, members, (0., 0.), 4., 4., 1.)
    with pytest.raises(ValueError):
        coverage_plan(grid, members[:1], (0., 0.), 100., 100., .3)


def test_task_identity_capacity_and_measured_arrival():
    goal = dict(x=1., y=0., yaw=-math.pi + .01, timeout=60.)
    queue = []
    for _ in range(64):
        enqueue(queue, mission(goal))
    assert len({item['task'] for item in queue}) == 64
    with pytest.raises(ValueError, match='full'):
        enqueue(queue, mission(goal))
    assert settled(goal, (1., 0., math.pi - .01), 0., 0.)
    assert not settled(goal, (1., 0., math.pi - .01), 0., .4)
    assert not settled(goal, (1., 0., math.pi - .01), None, 0.)
    assert not settled(goal, (1., 0., math.nan), 0., 0.)


def harness():
    # Exercise the real callback methods with an action-result fixture; no
    # rclpy context or simulated robot is created by these regression checks.
    from robot_lab_utils.fleet_controller import FleetController, Member
    node = SimpleNamespace(history=MissionHistory(), serial_traffic=True, events=[])
    for name in ('_clear_queue', '_finish_task', '_stop_member', '_cancel_handle',
                 '_goal_finished', '_arrival_tick', '_schedule'):
        setattr(node, name, MethodType(getattr(FleetController, name), node))
    node._event = lambda event, **data: node.events.append((event, data))
    node._truth_row = lambda member: dict(x=1., y=0., yaw=0., linear=0., angular=.4)
    member = Member(config=dict(id='alpha'))
    member.publisher = SimpleNamespace(publish=lambda value: None)
    member.mode = 'navigation'
    member.goal = dict(x=1., y=0., yaw=0., timeout=60.)
    member.active_item = mission(member.goal)
    member.task = member.active_item['task']
    node.members = {'alpha': member}
    return node, member


def test_nav2_success_requires_physical_settling_before_queued_task():
    node, member = harness()
    queued = mission(member.goal)
    member.queue.append(queued)
    future = SimpleNamespace(result=lambda: SimpleNamespace(status=4))
    node._goal_finished(member, member.generation, None, future)
    assert member.mode == 'settling' and len(member.queue) == 1
    node._arrival_tick(member, 10., True)
    assert not node.history.rows  # Yaw is still moving despite Nav2 success.
    node._truth_row = lambda member: dict(x=1., y=0., yaw=0., linear=0., angular=0.)
    node._arrival_tick(member, 11., True)
    node._arrival_tick(member, 11.5, True)
    assert node.history.rows[-1]['state'] == 'succeeded'
    assert member.mode == 'waiting' and member.queue[0]['task'] == queued['task']


def test_stop_cancels_waiting_and_active_identity_without_late_restart():
    node, member = harness()
    member.queue.append(mission(member.goal))
    generation = member.generation
    node._stop_member(member, 'Operator Stop')
    assert not member.queue and member.active_item is None and member.mode == 'idle'
    assert len(node.history.rows) == 2
    future = SimpleNamespace(result=lambda: SimpleNamespace(status=4))
    node._goal_finished(member, generation, None, future)
    assert member.mode == 'idle' and not member.queue


def test_failed_goal_holds_remaining_queue_for_explicit_operator_decision():
    node, member = harness()
    member.queue.append(mission(member.goal))
    future = SimpleNamespace(result=lambda: SimpleNamespace(status=6))
    node._goal_finished(member, member.generation, None, future)
    assert node.history.rows[-1]['state'] == 'failed'
    assert member.queue_paused and len(member.queue) == 1
