"""Owned Nav2 action transitions in the mobile pick-and-place node."""
from concurrent.futures import Future
import importlib.util
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
