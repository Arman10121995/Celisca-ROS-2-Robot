"""Asynchronous mission cancellation and blocked-input regressions, no flight/route qualification."""
from concurrent.futures import Future, ThreadPoolExecutor
import math
from pathlib import Path
import sys
import threading
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/robot_lab_utils'))
from robot_lab_utils.marine_navigation import MarineNavigation


@pytest.fixture
def mission(monkeypatch):
    monkeypatch.setattr('robot_lab_utils.marine_navigation.time.monotonic', lambda: 10.)
    worker = ThreadPoolExecutor(max_workers=1)
    node = SimpleNamespace(generation=3, future_generation=3, future_kind='replan', future=None,
        status='replanning', reason='', stop_token=threading.Event(), route=[[0., 0., -1.], [2., 0., -1.]],
        position=np.array([0., 0., -1.]), native=dict(enabled=True, operator_fresh=True, input_owner='mission'),
        deadline=100., fresh=lambda: True, drive_pub=SimpleNamespace(publish=Mock()), control=Mock(),
        event=Mock(), index=1, replan_at=-math.inf, auv=False, shore=None, observed=None, scene=object(),
        surface_vessel=False, radius=.4, pose_std=.01, bottom=-6., surface=0., worker=worker)
    for name in ('cancel', '_poll_plan', '_try_replan'):
        setattr(node, name, MethodType(getattr(MarineNavigation, name), node))
    yield node
    node.stop_token.set()
    worker.shutdown(wait=True, cancel_futures=True)


def result(node):
    future = Future()
    future.set_result((node.generation, [[0., 0., -1.], [1., 1., -1.], [2., 0., -1.]], {}, [0., 0., -1.]))
    node.future = future


def test_late_replan_cannot_resume_after_operator_cancel(mission):
    result(mission)
    mission.cancel('Operator Stop')
    assert mission.stop_token.is_set()
    mission._poll_plan(10.)
    assert mission.status == 'canceled' and mission.route is None
    mission.control.assert_called_with('stop')
    mission.event.assert_called_with('planner_result_discarded', reason='Canceled/replaced generation')


def test_old_worker_exception_does_not_overwrite_new_operator_state(mission):
    mission.future = Future()
    mission.future.set_exception(RuntimeError('old geometry failed'))
    mission.cancel('Replaced')
    mission.status, mission.reason = 'planned', 'A newer intent'
    mission._poll_plan(10.)
    assert mission.status == 'planned' and mission.reason == 'A newer intent'


@pytest.mark.parametrize('fault', ['drift', 'operator', 'control', 'owner', 'deadline', 'sensor'])
def test_replan_acceptance_rechecks_live_mission_barriers(mission, fault):
    result(mission)
    if fault == 'drift': mission.position[0] = .26
    if fault == 'operator': mission.native['operator_fresh'] = False
    if fault == 'control': mission.native['enabled'] = False
    if fault == 'owner': mission.native['input_owner'] = 'gui'
    if fault == 'deadline': mission.deadline = 9.
    if fault == 'sensor': mission.fresh = lambda: False
    mission._poll_plan(10.)
    assert mission.status == 'canceled' and mission.route is None


def test_current_owned_replan_accepts_only_its_captured_generation(mission):
    result(mission)
    mission._poll_plan(10.)
    assert mission.status == 'executing' and mission.index == 1
    assert mission.route[1] == [1., 1., -1.]


def test_blocked_final_segment_is_neutral_and_uses_cancelable_worker(mission, monkeypatch):
    captured = {}
    def compute(scene, start, goal, **kwargs):
        captured.update(kwargs)
        return [start.tolist(), goal.tolist()], {}
    monkeypatch.setattr('robot_lab_utils.marine_navigation.plan', compute)
    mission.status = 'executing'
    mission._try_replan('Observed final segment blocked')
    assert mission.status == 'replanning'
    assert mission.drive_pub.publish.call_args.args[0].linear.x == 0.
    mission.future.result(timeout=2.)
    mission.cancel('Operator Stop while planning')
    assert captured['canceled']()
    mission._poll_plan(10.1)
    assert mission.status == 'canceled'


def test_retry_cooldown_still_publishes_neutral(mission):
    mission.replan_at = 9.
    mission._try_replan('Blocked again')
    assert mission.future is None
    message = mission.drive_pub.publish.call_args.args[0]
    assert message.linear.x == message.linear.y == message.linear.z == message.angular.z == 0.


def test_underactuated_auv_cannot_bypass_transect_constraint_by_replanning(mission):
    mission.auv = True
    mission._try_replan('Transect obstructed')
    assert mission.future is None and mission.status == 'canceled'
    assert 'forward-aligned' in mission.reason
    mission.control.assert_called_with('stop')
