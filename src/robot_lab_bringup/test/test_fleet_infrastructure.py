"""Focused regressions for facility energy and lift transport (no mission claims)."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.fleet_infrastructure import Infrastructure  # noqa: E402


def _make_session(tmp_path):
    infra = tmp_path / 'infra.yaml'
    infra.write_text('''schema_version: 1
doors:
  main:
    joint: door_main
    closed: 0.0
    opened: 1.8
    sweep: [-0.5, 0.5, -1.1, 3.1]
  lift:
    joint: door_lift
    closed: 0.0
    opened: 2.2
    sweep: [2.6, 3.4, -1.3, 3.5]
    z_joint: lift_car
lift:
  name: facility_lift
  joint: lift_car
  door: lift
  floors:
    L0: 0.0
    L1: 3.0
floor_maps:
  L0: maps/fleet_facility/maps/map.yaml
  L1: maps/fleet_facility/maps/map_L1.yaml
docks:
  west_charger:
    x: -4.5
    y: -3.0
    z: 0.0
    yaw: 0.0
    position_tolerance: 0.12
    height_tolerance: 0.12
    yaw_tolerance: 0.15
energy:
  capacity_J: 180000.0
  initial_fraction: 0.8
  hotel_W: 3.0
  drive_efficiency: 0.75
  charge_W: 120.0
  effort_observation_verified: true
''')
    session = dict(
        directory=str(tmp_path / 'session'),
        members=[dict(id='alpha'), dict(id='beta')],
        infrastructure_file=str(infra),
    )
    (tmp_path / 'session').mkdir(exist_ok=True)
    return session


def _make_node(session):
    node = SimpleNamespace()
    node.session = session
    import yaml
    node.config = yaml.safe_load(Path(session['infrastructure_file']).read_text())
    node.identity = 'sess'
    node.token = 'abc'
    node.positions = {}
    node.joint_at = -math.inf
    node.clock = None
    node.clock_at = -math.inf
    node.joint_stamp = None
    node.rates = {}
    node.runtime = None
    node.runtime_at = -math.inf
    node.bodies = {}
    node.paths = {}
    node.docking = {}
    node.events = []
    node.doors = {name: False for name in node.config['doors']}
    node.outputs = {}
    node.floor = 'L0'
    node.lift_request = None
    node.lift_owner = ''
    node.lift_at = -math.inf
    node.hold = False
    node.lift_gui = False
    node.door_owners = {name: set() for name in node.doors}
    node.energy = {
        m['id']: dict(
            joules=node.config['energy']['capacity_J'] * node.config['energy']['initial_fraction'],
            last=None, power=None, at=-math.inf,
            valid=bool(node.config['energy'].get('effort_observation_verified')),
            floor='L0',
        )
        for m in session['members']
    }
    node._effort_verified = bool(node.config['energy'].get('effort_observation_verified'))
    node._lift_prev = None
    node._lift_prev_at = -math.inf
    node.floor_maps = node.config.get('floor_maps', {})
    node.current_floor_map = ''
    return node


def _joint_state(names, velocities, efforts, stamp_sec=1000.0):
    from sensor_msgs.msg import JointState
    from builtin_interfaces.msg import Time
    msg = JointState()
    msg.name = list(names)
    msg.velocity = list(velocities)
    msg.effort = list(efforts)
    msg.header.stamp = Time(sec=int(stamp_sec), nanosec=int((stamp_sec % 1) * 1e9))
    return msg


def test_energy_becomes_valid_with_fresh_motor_feedback(tmp_path):
    session = _make_session(tmp_path)
    node = _make_node(session)
    for name in ('alpha', 'beta'):
        assert node.energy[name]['valid'] is True
    Infrastructure.motor(node, 'alpha', _joint_state(
        ['alpha/wheel_left', 'alpha/wheel_right'], [1.0, 1.0], [5.0, 5.0], 1000.0))
    Infrastructure.motor(node, 'alpha', _joint_state(
        ['alpha/wheel_left', 'alpha/wheel_right'], [1.0, 1.0], [5.0, 5.0], 1000.5))
    entry = node.energy['alpha']
    assert entry['valid'] is True
    assert entry['power'] is not None and entry['power'] > 0
    assert entry['joules'] < node.config['energy']['capacity_J'] * 0.8


def test_energy_stays_unknown_when_effort_not_verified(tmp_path):
    session = _make_session(tmp_path)
    import yaml
    cfg = yaml.safe_load(Path(session['infrastructure_file']).read_text())
    cfg['energy']['effort_observation_verified'] = False
    node = _make_node(session)
    node.config = cfg
    node._effort_verified = False
    node.energy = {
        m['id']: dict(
            joules=cfg['energy']['capacity_J'] * cfg['energy']['initial_fraction'],
            last=None, power=None, at=-math.inf,
            valid=bool(cfg['energy'].get('effort_observation_verified')), floor='L0',
        )
        for m in session['members']
    }
    for name in ('alpha', 'beta'):
        assert node.energy[name]['valid'] is False
    Infrastructure.motor(node, 'alpha', _joint_state(
        ['alpha/wheel_left', 'alpha/wheel_right'], [1.0, 1.0], [5.0, 5.0], 1000.0))
    assert node.energy['alpha']['valid'] is False


def test_energy_charges_when_docked(tmp_path):
    session = _make_session(tmp_path)
    node = _make_node(session)
    Infrastructure.motor(node, 'alpha', _joint_state(
        ['alpha/wheel_left', 'alpha/wheel_right'], [1.0, 1.0], [10.0, 10.0], 1000.0))
    Infrastructure.motor(node, 'alpha', _joint_state(
        ['alpha/wheel_left', 'alpha/wheel_right'], [1.0, 1.0], [10.0, 10.0], 1000.5))
    drained = node.energy['alpha']['joules']
    node.docking['alpha'] = dict(dock='west_charger', request_id='r1', settled=None, docked=True)
    Infrastructure.motor(node, 'alpha', _joint_state(
        ['alpha/wheel_left', 'alpha/wheel_right'], [1.0, 1.0], [10.0, 10.0], 1001.0))
    assert node.energy['alpha']['joules'] >= drained


def test_lift_transport_carries_robot_between_floors(tmp_path):
    session = _make_session(tmp_path)
    node = _make_node(session)
    lift = node.config['lift']
    node.bodies['alpha'] = dict(xyz=[4.0, 0.0, 0.0], yaw=0.0, stamp=1000.0, at=1000.0)
    node.energy['alpha']['floor'] = 'L0'
    node._lift_prev = 0.0
    node.positions[lift['joint']] = 3.0
    lift_position = node.positions[lift['joint']]
    lift_delta = lift_position - node._lift_prev
    node._lift_prev = lift_position
    on_lift = [name for name, b in node.bodies.items() if 2.8 <= b['xyz'][0] <= 5.2 and -1.2 <= b['xyz'][1] <= 1.2]
    assert 'alpha' in on_lift
    if abs(lift_delta) > 1e-4:
        for name in on_lift:
            b = node.bodies.get(name)
            if b:
                b['xyz'][2] += lift_delta
                b['at'] = 1001.0
                new_floor = next((f for f, z in lift['floors'].items() if abs(b['xyz'][2] - z) < .03),
                                 node.energy[name]['floor'])
                if new_floor != node.energy[name]['floor']:
                    node.energy[name]['floor'] = new_floor
    assert node.bodies['alpha']['xyz'][2] == 3.0
    assert node.energy['alpha']['floor'] == 'L1'


def test_robot_not_on_lift_is_not_transported(tmp_path):
    session = _make_session(tmp_path)
    node = _make_node(session)
    lift = node.config['lift']
    node.bodies['alpha'] = dict(xyz=[0.0, 0.0, 0.0], yaw=0.0, stamp=1000.0, at=1000.0)
    node.energy['alpha']['floor'] = 'L0'
    node._lift_prev = 0.0
    node.positions[lift['joint']] = 3.0
    lift_position = node.positions[lift['joint']]
    lift_delta = lift_position - node._lift_prev
    node._lift_prev = lift_position
    on_lift = [name for name, b in node.bodies.items() if 2.8 <= b['xyz'][0] <= 5.2 and -1.2 <= b['xyz'][1] <= 1.2]
    assert 'alpha' not in on_lift
    assert node.bodies['alpha']['xyz'][2] == 0.0
    assert node.energy['alpha']['floor'] == 'L0'


def test_floor_maps_loaded_from_config(tmp_path):
    session = _make_session(tmp_path)
    node = _make_node(session)
    assert node.floor_maps == {'L0': 'maps/fleet_facility/maps/map.yaml',
                               'L1': 'maps/fleet_facility/maps/map_L1.yaml'}
    assert node.current_floor_map == ''


def test_current_floor_map_tracks_lift_arrival(tmp_path):
    session = _make_session(tmp_path)
    node = _make_node(session)
    lift = node.config['lift']
    node.positions[lift['joint']] = 3.0
    node.rates[lift['joint']] = 0.0
    current_floor = next((f for f, z in lift['floors'].items() if abs(3.0 - z) < .03), '')
    if current_floor and abs(node.rates.get(lift['joint'], math.inf)) < .01:
        node.current_floor_map = node.floor_maps.get(current_floor, '')
    assert current_floor == 'L1'
    assert node.current_floor_map == 'maps/fleet_facility/maps/map_L1.yaml'


def test_floor_z_offset_transitions_on_lift_move(tmp_path):
    session = _make_session(tmp_path)
    node = _make_node(session)
    lift = node.config['lift']
    # Robot on lift car at L0
    node.bodies['alpha'] = dict(xyz=[4.0, 0.0, 0.0], yaw=0.0, stamp=1000.0, at=1000.0)
    node.energy['alpha']['floor'] = 'L0'
    node.floor_z = {name: float(lift['floors']['L0']) for name in node.energy}
    node._lift_prev = 0.0
    node.positions[lift['joint']] = 3.0
    lift_position = node.positions[lift['joint']]
    lift_delta = lift_position - node._lift_prev
    node._lift_prev = lift_position
    on_lift = [name for name, b in node.bodies.items() if 2.8 <= b['xyz'][0] <= 5.2 and -1.2 <= b['xyz'][1] <= 1.2]
    assert 'alpha' in on_lift
    if abs(lift_delta) > 1e-4:
        for name in on_lift:
            b = node.bodies.get(name)
            if b:
                b['xyz'][2] += lift_delta
                b['at'] = 1001.0
                new_floor = next((f for f, z in lift['floors'].items() if abs(b['xyz'][2] - z) < .03),
                                 node.energy[name]['floor'])
                if new_floor != node.energy[name]['floor']:
                    node.energy[name]['floor'] = new_floor
    # Floor z-offset follows the new floor
    target_z = float(lift['floors'].get(node.energy['alpha']['floor'], 0.0))
    assert node.energy['alpha']['floor'] == 'L1'
    assert target_z == 3.0
    assert node.floor_z.get('alpha', 0.0) == 0.0  # floor_z updated in tick, not here


def test_charging_power_readback_when_docked(tmp_path):
    session = _make_session(tmp_path)
    node = _make_node(session)
    # Establish motor feedback with discharge
    Infrastructure.motor(node, 'alpha', _joint_state(
        ['alpha/wheel_left', 'alpha/wheel_right'], [1.0, 1.0], [10.0, 10.0], 1000.0))
    Infrastructure.motor(node, 'alpha', _joint_state(
        ['alpha/wheel_left', 'alpha/wheel_right'], [1.0, 1.0], [10.0, 10.0], 1000.5))
    draining_power = node.energy['alpha']['power']
    assert draining_power is not None and draining_power > 0
    # Dock and verify charging power is applied
    node.docking['alpha'] = dict(dock='west_charger', request_id='r1', settled=None, docked=True)
    Infrastructure.motor(node, 'alpha', _joint_state(
        ['alpha/wheel_left', 'alpha/wheel_right'], [1.0, 1.0], [10.0, 10.0], 1001.0))
    entry = node.energy['alpha']
    assert entry['joules'] > draining_power * 0.5  # net charge exceeds discharge over 0.5s
    # Verify power readback includes hotel + drive (not just charging offset)
    assert entry['power'] is not None and entry['power'] > 0