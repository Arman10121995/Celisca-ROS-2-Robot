"""Owned Nav2 action transitions in the mobile pick-and-place node."""
from concurrent.futures import Future
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


@pytest.fixture
def pick_place():
    pytest.importorskip('rclpy')
    path = Path(__file__).resolve().parents[1]/'scripts/urdf_pick_place.py'
    spec = importlib.util.spec_from_file_location('urdf_pick_place', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_stop_waits_for_late_nav2_acceptance_and_terminal_cancel(pick_place):
    node = object.__new__(pick_place.PickPlace)
    nav = dict(future=Future(), result=None)
    node.stopping_nav, node.task = nav, None
    node._twist = Mock()
    node._advance()
    assert node.stopping_nav is nav
    action_result = Future()
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: action_result, cancel_goal_async=Mock())
    nav['future'].set_result(handle)
    node._advance()
    node._advance()
    handle.cancel_goal_async.assert_called_once_with()
    assert node.stopping_nav is nav
    action_result.set_result(SimpleNamespace(status=pick_place.GoalStatus.STATUS_CANCELED))
    node._advance()
    assert node.stopping_nav is None
    node._twist.assert_called_with(0.0, 0.0)


def test_docking_waits_for_nav2_result_and_measured_stop(pick_place):
    node = object.__new__(pick_place.PickPlace)
    node.amcl_pose, node.amcl_at = (2.01, 1.0, 0.0), pick_place._now()
    node.base_pose, node.base_twist = (2.02, 1.0, 0., 0.), (.07, 0.)
    node._twist, node._set_footprint = Mock(), Mock()
    nav = dict(sent=pick_place._now(), handle=Mock(), result=Future())
    phase = dict(nav_goal=nav, name='carry', waypoint=0)
    task, target = dict(log=[]), dict(x=2., y=1., yaw=0.)
    node._nav2_leg(task, phase, target)
    nav['handle'].cancel_goal_async.assert_called_once_with()
    node._twist.assert_not_called()
    nav['result'].set_result(SimpleNamespace(status=pick_place.GoalStatus.STATUS_CANCELED))
    node._nav2_leg(task, phase, target)
    node._twist.assert_called_with(0., 0.)
    assert phase['waypoint'] == 0 and not task['log']
    node.base_twist = (0., 0.)
    node._nav2_leg(task, phase, target)
    assert phase['waypoint'] == 1 and phase['nav_goal'] is None
    assert task['log'][0]['status'] == pick_place.GoalStatus.STATUS_CANCELED
    assert task['log'][0]['handed_over']


def test_loaded_nav2_action_requires_two_fresh_shape_receipts(pick_place):
    from builtin_interfaces.msg import Time
    from nav2_msgs.action import NavigateToPose
    from robot_lab_utils.footprint_receipts import FootprintReceipts

    node = object.__new__(pick_place.PickPlace)
    node.footprint_receipts = FootprintReceipts(padding=.03)
    node.footprint_pubs = [Mock(), Mock()]
    node._sim_seconds, node._twist = lambda: 10., Mock()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10)))
    node.nav_action, node.nav_client = NavigateToPose, Mock()
    node.carried_footprint = [(0.6, -0.15), (0.6, 0.15)]
    phase, task, target = dict(name='carry', carrying=True), dict(object='box', log=[]), dict(x=2., y=1., yaw=0.)
    node._nav2_leg(task, phase, target)
    node.nav_client.send_goal_async.assert_not_called()
    shape = node.footprint_receipts.expected
    assert node.footprint_receipts.observe('local', 'odom', 10., shape, 10.)
    node._nav2_leg(task, phase, target)
    node.nav_client.send_goal_async.assert_not_called()
    assert node.footprint_receipts.observe('global', 'map', 10., shape, 10.)
    node._nav2_leg(task, phase, target)
    node.nav_client.send_goal_async.assert_called_once()
    assert task['log'][0]['consumers'] == ['global', 'local']
    node._sim_seconds, node._finish = lambda: 13., Mock()
    node._nav2_leg(task, phase, target)
    node._finish.assert_called_once_with('failed', 'Loaded footprint acknowledgement was lost during Nav2 transit')


def test_stop_keeps_the_loaded_costmap_polygon(pick_place, tmp_path):
    node = object.__new__(pick_place.PickPlace)
    node.mobile, node.carried_footprint = True, [(0.6, 0.15)]
    node.state, node.receipts = {}, tmp_path
    node._set_footprint, node._twist, node.get_logger = Mock(), Mock(), Mock()
    node.stop_client = SimpleNamespace(service_is_ready=lambda: False)
    nav = dict(result=None, future=Future())
    node.task = dict(phase_index=0, phases=[dict(name='carry', nav_goal=nav)], transit='nav2',
                     object='box', support='table', object_start=[0., 0., .2],
                     request={}, plan={}, log=[], started=pick_place._now())
    node._finish('stopped', 'Stopped by request')
    assert node.stopping_nav is nav
    assert node.carried_footprint == [(0.6, 0.15)]
    node._set_footprint.assert_not_called()


def test_loaded_outline_encloses_robot_and_rotated_payload_in_base_frame(pick_place):
    from shapely.geometry import Point, Polygon

    node = object.__new__(pick_place.PickPlace)
    node.base_pose = (4., -3., 0., math.pi/2)
    node.truth = dict(box=[3.9, -2.3, 0.7, math.sqrt(.5), 0., 0., math.sqrt(.5)])
    node.object_sizes = dict(box=[.12, .04, .2])
    robot = [[-.3, 0.], [.5, -.14], [.4, .25]]
    task = dict(scene=SimpleNamespace(robot_collision_points=lambda: robot), object='box')
    polygon = Polygon(node._carry_outline(task))
    assert all(polygon.covers(Point(*point)) for point in robot)
    assert polygon.covers(Point(.76, .12))
    assert polygon.bounds[2] == pytest.approx(.76+pick_place.CARRY_DRIFT)
