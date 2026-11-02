"""Automatic lift intent/observation barriers; no simulated mission PASS."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.fleet_transport import LiftTransport

CONFIG = dict(boarding=dict(x=3.65, y=0., yaw=0.),
              landings=dict(L0=dict(x=2.3, y=0., yaw=0.), L1=dict(x=2.2, y=0., yaw=0.)),
              cabin=[3.4, 4.6, -.6, .6])


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
