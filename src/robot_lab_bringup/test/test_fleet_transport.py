"""Automatic lift intent/observation barriers; no simulated mission PASS."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.fleet_transport import LiftTransport,lift_door_admission,door_position_mode

CONFIG = dict(boarding=dict(x=3.65, y=0., yaw=0.),
              landings=dict(L0=dict(x=2.3, y=0., yaw=0.), L1=dict(x=2.2, y=0., yaw=0.)),
              cabin=[3.4, 4.6, -.6, .6])


def test_precise_opening_survives_small_loading_compliance_but_not_motion_or_a_new_floor():
    gate=lambda position,rate=0.,floor='L1',height=3.,admitted=None:lift_door_admission(position,rate,floor,height,admitted,.0001,.003)
    assert gate(3.002)==(None,False)  # No initial 2 mm sill admission.
    admitted,allowed=gate(3.00005)
    assert allowed and admitted=='L1'
    assert gate(2.9994,admitted=admitted)==('L1',True)  # Actual payload compliance.
    assert gate(2.996,admitted=admitted)==(None,False)
    assert gate(3.,rate=.02,admitted=admitted)==(None,False)
    assert gate(3.,floor='L0',height=0.,admitted=admitted)==(None,False)
    assert gate(3.002)==(None,False)  # A lost admission cannot silently return.


def test_clear_door_beyond_open_target_is_still_open_in_both_directions():
    assert door_position_mode(2.2,0.,2.15)==2
    assert door_position_mode(-2.2,0.,-2.15)==2
    assert door_position_mode(1.,0.,2.15)==1
    assert door_position_mode(0.,0.,2.15)==0


def context(task, xyz=(2.3, 0., 0.), clock=1.):
    return dict(fresh=True, operator_fresh=True, clock=clock,
        body=dict(xyz=xyz, yaw=0., speed=0., angular=0.),
        member=dict(mode='navigation', active_source='route:' + str(task.route_id)),
        history=[], floor='L0', car_floor='L0', door_open=False, map_ready=True)


def test_acknowledgement_and_success_do_not_replace_settled_body():
    task = LiftTransport('alpha', 'operator', 'L0', 'L1', CONFIG, 0.)
    assert task.tick(context(task), .1)[0]['kind'] == 'route'
    state = context(task)
    assert task.tick(state, 1.) == []  # Active route acknowledgement only.
    state['history'] = [dict(source='route:' + task.route_id, state='succeeded')]
    state['body']['xyz'] = [4., 0., 0.]
    assert task.tick(state, 2.) == []
    assert task.phase == 'approaching'
    state['body']['xyz'] = [2.3, 0., 0.]
    state['clock'] = 10.
    assert task.tick(state, 3.) == []
    state['clock'] = 11.1
    assert task.tick(state, 4.) == [dict(kind='lift', floor='L0')]


def test_riding_waits_for_observed_robot_floor_map_and_open_door():
    task = LiftTransport('alpha', 'operator', 'L0', 'L1', CONFIG, 0.)
    task.phase = 'riding'
    state = context(task, (3.65, 0., 3.))
    state.update(car_floor='L1', door_open=True)
    assert task.tick(state, 1.) == []  # Car moved, robot floor not established.
    state.update(floor='L1', map_ready=False)
    assert task.tick(state, 2.) == []
    state['map_ready'] = True
    result = task.tick(state, 3.)
    assert result[0]['kind'] == 'route'
    assert result[0]['goal']['x'] == 2.2
    assert task.phase == 'exiting'


def test_loss_cancels_once_and_cannot_automatically_rearm():
    task = LiftTransport('alpha', 'operator', 'L0', 'L1', CONFIG, 0.)
    state = context(task)
    state['operator_fresh'] = False
    actions = task.tick(state, 1.)
    assert [action['kind'] for action in actions] == ['stop', 'release']
    assert actions[1]['hold'] is True
    state['operator_fresh'] = True
    assert task.tick(state, 2.) == []
    assert task.phase == 'failed'


def test_failed_owned_route_prevents_boarding():
    task = LiftTransport('alpha', 'operator', 'L0', 'L1', CONFIG, 0.)
    task.tick(context(task), .1)
    state = context(task)
    state['history'] = [dict(source='route:' + task.route_id, state='failed')]
    assert task.tick(state, 1.)[0]['kind'] == 'stop'
    assert not task.active


def test_recovered_passenger_exit_waits_for_floor_map_and_door_then_settles_outside():
    task = LiftTransport.exit_only('alpha', 'operator', 'L1', CONFIG, 0., parking=dict(x=1.55, y=.55, yaw=0.))
    assert (task.origin, task.destination, task.phase) == ('L1', 'L1', 'exit_ready')
    state = context(task, (4., 0., 3.))
    state.update(floor='L1', car_floor='L1', map_ready=False, door_open=True)
    assert task.tick(state, .5) == []                     # Floor map still loading.
    state['map_ready'] = True
    action = task.tick(state, 1.)[0]
    assert action['kind'] == 'route' and action['goal']['x'] == 2.2 and task.phase == 'exiting'
    state.update(history=[dict(source='route:' + task.route_id, state='succeeded')], clock=5.)
    state['body']['xyz'] = (2.2, 0., 3.)
    task.tick(state, 2.)
    state['clock'] = 6.1
    assert task.tick(state, 3.)[0]['goal']['x'] == 1.55 and task.phase == 'parking'


def test_recovered_exit_refuses_a_passenger_outside_the_car():
    task = LiftTransport.exit_only('alpha', 'operator', 'L0', CONFIG, 0.)
    state = context(task, (1., 0., 0.))
    assert task.tick(state, 1.)[-1] == dict(kind='release', hold=True) and task.phase == 'failed'
