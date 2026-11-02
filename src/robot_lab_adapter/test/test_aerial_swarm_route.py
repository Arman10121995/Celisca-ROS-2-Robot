"""Source route geometry and FCU-time barriers; no FCU or plant is launched."""
from concurrent.futures import Future
import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from robot_lab_adapter.aerial_swarm_route import FormationRoute, plan_route, slots
from robot_lab_adapter.aerial_swarm_controller import AerialSwarmController


def source_plan(positions=None, anchors=None, canceled=lambda: False, scene=None):
    return plan_route(scene or SimpleNamespace(segment=lambda *args: True),
        positions or dict(air1=[0., -1.5, 2.], air2=[0., 1.5, 2.]),
        dict(air1=[0., -1.5, 0.], air2=[0., 1.5, 0.]),
        [[1., 0., 2.]] if anchors is None else anchors, 3., canceled)


def test_source_corridor_and_swept_peer_crossing_refuse_a_route():
    with pytest.raises(ValueError, match='source geometry'):
        source_plan(scene=SimpleNamespace(segment=lambda *args: False))
    with pytest.raises(ValueError, match='separation envelope'):
        source_plan(positions=dict(air1=[0., 1.5, 2.], air2=[0., -1.5, 2.]))


@pytest.mark.parametrize('anchors', [[], [[1., 0., 20.]], [[1., 0., float('nan')]], [[1., 2.]], [[1., 0., 2.]]*17, [[40., 0., 2.]]])
def test_route_bounds_are_enforced(anchors):
    with pytest.raises(ValueError): source_plan(anchors=anchors)


def test_planning_cancellation_is_consumed_inside_source_checks():
    with pytest.raises(TimeoutError): source_plan(canceled=lambda: True)


def test_endpoint_completion_requires_advancing_fcu_time_and_measured_speed():
    plan = source_plan()
    mission = FormationRoute(plan, 10.)
    poses = slots(mission.anchor, plan['targets'], plan['spacing'])
    stamps = dict(air1=100., air2=200.)
    mission.tick(poses, dict(air1=None, air2=None), stamps, 11.)
    assert mission.phase == 'executing'
    mission.tick(poses, dict(air1=0., air2=0.), stamps, 12.)
    mission.tick(poses, dict(air1=0., air2=0.), stamps, 20.)
    assert mission.phase == 'executing'
    mission.tick(poses, dict(air1=0., air2=0.), dict(air1=101., air2=201.), 21.)
    assert mission.phase == 'completed' and 'FCU estimates' in mission.reason


def test_stop_is_terminal_and_deadline_cancels_instead_of_assigning_success():
    mission = FormationRoute(source_plan(), 0.)
    mission.tick({}, {}, {}, 121.)
    assert mission.phase == 'canceled'
    mission.tick({}, {}, {}, 122.)
    assert mission.phase == 'canceled'


def test_next_anchor_waits_for_all_member_epochs_and_resets_dwell():
    plan = source_plan(anchors=[[1., 0., 2.], [2., 0., 2.]])
    mission = FormationRoute(plan, 0.)
    poses = slots(mission.anchor, plan['targets'], plan['spacing'])
    speed = dict(air1=0., air2=0.)
    mission.tick(poses, speed, dict(air1=1., air2=1.), .1)
    mission.tick(poses, speed, dict(air1=2., air2=1.), 1.1)
    assert mission.index == 0
    mission.tick(poses, speed, dict(air1=2., air2=2.), 1.2)
    assert mission.index == 1 and mission.settled_stamps is None


def test_late_planner_result_cannot_rearm_a_stopped_controller():
    controller = object.__new__(AerialSwarmController)
    controller.generation = 5; controller.route_phase = 'canceled'
    controller.route_plan = controller.route_mission = None; controller.mode = 'idle'
    controller.fresh = lambda row: True
    controller.members = dict(air1=dict(status=dict(phase='flying')), air2=dict(status=dict(phase='flying')))
    future = Future(); future.set_result(source_plan())
    controller.route_job = (future, 4, ['air1', 'air2'])
    controller._route_tick(10., True)
    assert controller.route_plan is None and controller.mode == 'idle' and controller.route_job is None


def test_old_planner_loss_does_not_replace_a_new_control_intent():
    controller = object.__new__(AerialSwarmController)
    controller.generation = 5; controller.route_phase = 'canceled'
    controller.route_plan = controller.route_mission = None; controller.mode = 'manual'
    controller.hold = Mock()
    future = Future(); future.set_result(source_plan())
    controller.route_job = (future, 4, ['air1', 'air2'])
    controller._route_tick(10., False)
    controller.hold.assert_not_called()
    assert controller.mode == 'manual' and controller.route_job is None


def test_movement_rejects_stale_feedback_without_blocking_executor(monkeypatch):
    controller = object.__new__(AerialSwarmController)
    controller.session = dict(session_id='session'); controller.operator = 'op'; controller.hb_at = 10.
    controller.members = dict(air1=dict(status=None)); controller.fresh = lambda row: False; controller.event = Mock()
    monkeypatch.setattr('robot_lab_adapter.aerial_swarm_controller.time.monotonic', lambda: 10.)
    monkeypatch.setattr('robot_lab_adapter.aerial_swarm_controller.time.sleep', lambda *args: pytest.fail('Executor must not sleep for its own subscriptions'))
    controller.operation(SimpleNamespace(data=json.dumps(dict(session_id='session', operator_id='op', op='manual', targets=['air1']))))
    assert 'fresh and airborne' in controller.reason
