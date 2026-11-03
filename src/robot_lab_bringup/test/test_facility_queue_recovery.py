"""Facility admission/ownership failures; these checks do not qualify a mission."""
import json
import math
import sys
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'src/robot_lab_utils'))
from robot_lab_utils.fleet_transport_queue import TransportQueue
from robot_lab_utils.fleet_lift_recovery import recovery_passengers
from robot_lab_utils.fleet_infrastructure import Infrastructure


def test_itinerary_admission_is_atomic_and_predicts_each_members_floor():
    queue = TransportQueue(4)
    queue.append([dict(robot='alpha', floor='L1'), dict(robot='alpha', floor='L0')], 'op', {'alpha': 'L0'}, ['L0', 'L1'], 1.)
    before = list(queue.pending)
    with pytest.raises(ValueError):
        queue.append([dict(robot='alpha', floor='L1'), dict(robot='alpha', floor='L1')], 'op', {'alpha': 'L0'}, ['L0', 'L1'], 2.)
    assert queue.pending == before
    assert queue.take()['floor'] == 'L1'
    assert queue.take() is None
    queue.finish('succeeded', 'independent observer settled')
    assert queue.take()['floor'] == 'L0'
    queue.finish('failed', 'lost feedback')
    assert queue.history[-1]['phase'] == 'failed'


def test_queue_bound_and_targeted_stop_preserve_other_members():
    queue = TransportQueue(2)
    queue.append([dict(robot=name, floor='L1') for name in ('alpha', 'beta')], 'op', {'alpha': 'L0', 'beta': 'L0'}, ['L0', 'L1'], 1.)
    with pytest.raises(ValueError):
        queue.append([dict(robot='alpha', floor='L0')], 'op', {'alpha': 'L0'}, ['L0', 'L1'], 2.)
    queue.cancel('operator Stop', ['alpha'])
    assert [row['robot'] for row in queue.pending] == ['beta']
    assert queue.history[-1]['phase'] == 'canceled'


def test_profile_wait_budget_survives_valid_long_predecessor_without_renewing_intent():
    queue=TransportQueue(wait_timeout_s=1800.)
    queue.append([dict(robot='alpha',floor='L1'),dict(robot='alpha',floor='L0')],'op',{'alpha':'L0'},['L0','L1'],10.)
    original_deadline=queue.pending[1]['expires_at']
    queue.take();queue.finish('succeeded','observed predecessor completed after 630 seconds')
    assert not queue.expired(640.)
    assert queue.pending[0]['expires_at']==original_deadline==1810.
    assert queue.expired(1810.01)
    assert queue.state()['wait_timeout_s']==1800.
    default=TransportQueue();default.append([dict(robot='alpha',floor='L1')],'op',{'alpha':'L0'},['L0','L1'],10.)
    assert default.expired(610.01)  # Existing non-facility default remains bounded.


@pytest.mark.parametrize('budget',[True,None,0.,-1.,math.inf,math.nan,3600.01])
def test_invalid_queue_wait_duration_is_refused(budget):
    with pytest.raises(ValueError):TransportQueue(wait_timeout_s=budget)


@pytest.fixture
def facility(monkeypatch):
    monkeypatch.setattr('robot_lab_utils.fleet_infrastructure.time.monotonic', lambda: 100.)
    config = dict(lift=dict(name='car', joint='lift', door='gate', floors=dict(L0=0., L1=3.), level_tolerance_m=.003),
                  doors=dict(gate=dict(joint='gate'), landing=dict(joint='landing', landing_floor='L0')),
                  transport=dict(cabin=[3.4, 4.6, -.6, .6], boarding=dict(x=3.65, y=0., yaw=0.),
                      landings=dict(L0=dict(x=2.3, y=0., yaw=0.), L1=dict(x=2.2, y=0., yaw=0.))))
    members = [dict(id='alpha', radius=.25), dict(id='beta', radius=.3)]
    bodies = dict(alpha=dict(xyz=[3.9, 0., 1.], speed=0., angular=0., at=100.),
                  beta=dict(xyz=[-3., -3., 0.], speed=0., angular=0., at=100.))
    node = SimpleNamespace(config=config, session=dict(members=members, navigation=True), bodies=bodies,
        rates=dict(lift=0.), positions=dict(lift=1., gate=0., landing=0.), targets=dict(lift=None, gate=None, landing=None),
        lift_owner='', lift_at=-math.inf, lift_request=None, lift_gui=False, hold=True,
        recovery=None, rmf_active=False, transports={}, transport_queue=TransportQueue(),
        identity='session', runtime=dict(operator_id='op', operator_fresh=True, members=[dict(id=name, mode='idle', navigation_ready=True, included=True, queued=0) for name in ('alpha', 'beta')]),
        runtime_at=100., joint_at=100., clock_at=100., outputs={}, energy=dict(alpha=dict(floor='L0'), beta=dict(floor='L0')),
        last_rejection='', get_logger=lambda: SimpleNamespace(warning=lambda text: None))
    for name in ('request_lift', '_admit_transports', '_cancel_transports', '_transport_actions', '_stop_recovery', '_recovery_tick', '_prepare_transport'):
        setattr(node, name, MethodType(getattr(Infrastructure, name), node))
    return node


def command(node, **data):
    Infrastructure.command(node, SimpleNamespace(data=json.dumps(dict(session_id='session', operator_id='op', **data))))


def test_sdk_session_cannot_be_stolen_after_five_seconds(facility):
    facility.lift_owner = 'sdk-passenger'
    facility.lift_at = 1.
    peer = SimpleNamespace(lift_name='car', session_id='other-session', request_type=1, destination_floor='L1', door_state=2)
    assert facility.request_lift(peer) is False
    assert facility.lift_owner == 'sdk-passenger'
    assert facility.lift_request is None
    peer.session_id = 'sdk-passenger'
    peer.request_type = 0
    assert facility.request_lift(peer)
    peer.session_id = 'other-session'
    peer.request_type = 1
    assert facility.request_lift(peer)


def test_explicit_recovery_admission_and_observed_completion(facility):
    command(facility, op='recover_lift', floor='L0')
    assert not facility.last_rejection
    assert facility.recovery['passengers'] == ['alpha']
    assert facility.lift_request == ('L0', True)
    assert not facility.hold
    facility._recovery_tick(100.1, True, dict(gate=dict(mode=2), landing=dict(mode=2)))
    assert facility.recovery['phase'] == 'recovering'  # Open doors do not establish leveling.
    facility.positions['lift'] = 0.
    facility.bodies['alpha']['xyz'][2] = 0.
    facility._recovery_tick(100.2, True, dict(gate=dict(mode=2), landing=dict(mode=2)))
    assert facility.recovery['phase'] == 'succeeded'
    assert not facility.lift_owner


@pytest.mark.parametrize('fault', ['stale', 'boundary', 'moving', 'multiple', 'car_moving'])
def test_recovery_refuses_unobserved_or_unsafe_passengers(facility, fault):
    if fault == 'stale': facility.bodies['beta']['at'] = 98.
    if fault == 'boundary': facility.bodies['alpha']['xyz'][0] = 3.5
    if fault == 'moving': facility.bodies['alpha']['speed'] = .1
    if fault == 'multiple': facility.bodies['beta']['xyz'] = [4.2, 0., 1.]
    if fault == 'car_moving': facility.rates['lift'] = .02
    with pytest.raises(ValueError):
        recovery_passengers(facility.config, facility.session['members'], facility.bodies, facility.positions, facility.rates, 100.)


def test_recovery_loss_latches_hold_and_does_not_resume(facility):
    command(facility, op='recover_lift', floor='L0')
    facility._recovery_tick(100.1, False, {})
    assert facility.recovery['phase'] == 'failed'
    assert facility.hold and not facility.lift_owner and facility.lift_request is None
    facility._recovery_tick(100.2, True, {})
    assert facility.recovery['phase'] == 'failed'


def test_native_queue_cannot_bypass_hold_or_sdk_ownership(facility):
    command(facility, op='transport_queue', requests=[dict(robot='alpha', floor='L1')])
    assert facility.last_rejection and not facility.transport_queue.pending
    facility.hold = False
    facility.rmf_active = True
    command(facility, op='transport_queue', requests=[dict(robot='alpha', floor='L1')])
    assert facility.last_rejection and not facility.transport_queue.pending
    facility.rmf_active = False
    command(facility, op='transport_queue', requests=[dict(robot='alpha', floor='L1')])
    assert not facility.last_rejection and len(facility.transport_queue.pending) == 1


def test_parked_peer_blocks_landing_and_stale_peer_blocks_dispatch(facility):
    facility.bodies['alpha']['xyz'] = [-2., -2., 0.]
    facility.bodies['beta']['xyz'] = [2.2, 0., 3.]
    with pytest.raises(ValueError, match='clear L1 landing'):
        facility._prepare_transport('alpha', 'L1', 'op', 100.)
    facility.bodies['beta']['xyz'] = [-3., -3., 0.]
    facility.bodies['beta']['at'] = 98.
    with pytest.raises(ValueError, match='all fleet bodies'):
        facility._prepare_transport('alpha', 'L1', 'op', 100.)


def test_admitted_queue_waits_for_actual_nav2_before_any_dispatch(facility):
    facility.hold = False
    facility.runtime['members'][0]['navigation_ready'] = False
    command(facility, op='transport_queue', requests=[dict(robot='alpha', floor='L1')])
    assert not facility.last_rejection and len(facility.transport_queue.pending) == 1
    with pytest.raises(ValueError, match='fresh idle member Nav2'):
        facility._prepare_transport('alpha', 'L1', 'op', 100.)


def test_fleet_stop_service_also_requests_physical_facility_stop():
    from unittest.mock import Mock
    from robot_lab_utils.fleet_controller import FleetController
    node = SimpleNamespace(session_id='session', _stop_all=Mock(), _stop_pub=Mock())
    response = SimpleNamespace()
    FleetController._stop_service(node, None, response)
    node._stop_all.assert_called_once()
    assert json.loads(node._stop_pub.publish.call_args.args[0].data) == dict(session_id='session', op='all_stop')
    assert response.success and 'facility' in response.message
