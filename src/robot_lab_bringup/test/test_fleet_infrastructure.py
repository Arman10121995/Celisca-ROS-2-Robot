"""Focused regressions for facility energy and lift transport (no mission claims)."""
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.fleet_infrastructure import Infrastructure  # noqa: E402
from robot_lab_utils.fleet_session import SpawnGrid, navigation_parameters  # noqa: E402


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
    node.floor_offsets = {}
    node.floor_maps = node.config.get('floor_maps', {})
    node.current_floor_map = ''
    node._observe_robot_floor = lambda name, xyz: Infrastructure._observe_robot_floor(node, name, xyz)
    node.robot_floor_maps = {name: node.floor_maps for name in ('alpha', 'beta')}
    node.active_robot_maps = {'alpha': node.floor_maps['L0'], 'beta': node.floor_maps['L0']}
    node.map_clients = {}
    node.map_futures = {}
    node.map_transitions = set()
    node.map_errors = {}
    node.map_status = {}
    node.map_attempt_at = {}
    node._map_loaded = lambda name, floor, path, future: Infrastructure._map_loaded(node, name, floor, path, future)
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


def _body_state(x, y, z, stamp_sec=1000.0):
    from builtin_interfaces.msg import Time
    from nav_msgs.msg import Odometry
    msg = Odometry()
    msg.header.stamp = Time(sec=int(stamp_sec), nanosec=int((stamp_sec % 1) * 1e9))
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.w = 1.0
    return msg


def test_energy_becomes_valid_with_fresh_motor_feedback(tmp_path):
    from robot_lab_utils.fleet_energy import EnergyEstimate
    node = _make_node(_make_session(tmp_path))
    model = EnergyEstimate(node.config['energy'], ['left', 'right'], True)
    model.observe(['left', 'right'], [1., 1.], [5., 5.], 1., 1., 1.)
    assert model.state(1.)['fraction'] is None
    model.observe(['left', 'right'], [1., 1.], [5., 5.], 1.5, 1.5, 1.5)
    assert model.state(1.5)['fraction'] < .8
    assert model.state(1.5)['power_W'] == pytest.approx(3 + 10 / .75)


def test_energy_stays_unknown_when_effort_not_verified(tmp_path):
    from robot_lab_utils.fleet_energy import EnergyEstimate
    node = _make_node(_make_session(tmp_path))
    model = EnergyEstimate(node.config['energy'], ['left', 'right'], False)
    for stamp in (1., 1.5):
        model.observe(['left', 'right'], [1., 1.], [5., 5.], stamp, stamp, stamp)
    assert model.state(1.5)['fraction'] is None
    assert model.joules == 144000.


def test_energy_charges_when_docked(tmp_path):
    from robot_lab_utils.fleet_energy import EnergyEstimate
    node = _make_node(_make_session(tmp_path))
    model = EnergyEstimate(node.config['energy'], ['left', 'right'], True)
    for stamp in (1., 1.5):
        model.observe(['left', 'right'], [0., 0.], [0., 0.], stamp, stamp, stamp, True)
    assert model.joules == pytest.approx(144000 + (120 - 3) * .5)
    assert model.state(1.5)['charging_W'] == 120.


def test_robot_floor_changes_only_after_observed_body_motion(tmp_path):
    session = _make_session(tmp_path)
    node = _make_node(session)
    lift_joint = node.config['lift']['joint']
    node.positions[lift_joint] = 0.0
    node.rates[lift_joint] = 0.0
    Infrastructure.body(node, 'alpha', _body_state(4.0, 0.0, 0.1, 1000.0))
    assert node.energy['alpha']['floor'] == 'L0'

    node.positions[lift_joint] = 3.0
    assert node.bodies['alpha']['xyz'][2] == 0.1
    assert node.energy['alpha']['floor'] == 'L0'

    Infrastructure.body(node, 'alpha', _body_state(4.0, 0.0, 3.1, 1001.0))
    assert node.bodies['alpha']['xyz'][2] == 3.1
    assert node.energy['alpha']['floor'] == 'L1'


def test_floor_map_transition_holds_until_load_succeeds(tmp_path):
    from nav2_msgs.srv import LoadMap

    class Future:
        def add_done_callback(self, callback):
            self.callback = callback

        def result(self):
            return SimpleNamespace(result=LoadMap.Response.RESULT_SUCCESS)

    class Client:
        def service_is_ready(self):
            return True

        def call_async(self, request):
            self.request = request
            self.future = Future()
            return self.future

    node = _make_node(_make_session(tmp_path))
    client = Client()
    node.map_clients['alpha'] = client
    Infrastructure._request_robot_map(node, 'alpha', 'L1', 10.0)
    assert client.request.map_url == node.floor_maps['L1']
    assert node.active_robot_maps['alpha'] == node.floor_maps['L0']
    assert 'alpha' in node.map_transitions

    client.future.callback(client.future)
    assert node.active_robot_maps['alpha'] == node.floor_maps['L1']
    assert 'alpha' not in node.map_transitions


def test_nav2_static_costmap_uses_member_map_topic():
    member = dict(id='alpha', model='bumperbot', spawn=dict(x=0.0, y=0.0, yaw=0.0),
                  max_linear=.25, max_angular=.5, radius=.118)
    parameters = navigation_parameters(
        ROOT / 'src/robot_lab_navigation/config',
        ROOT / 'src/robot_lab_localization/config',
        member,
        '/unused/floor.yaml',
    )['/alpha']
    topic = parameters['global_costmap']['global_costmap']['ros__parameters']['static_layer']['map_topic']
    assert topic == '/alpha/map'


def test_floor_maps_represent_the_lift_and_use_planar_origin():
    maps = ROOT / 'src/robot_lab_maps/maps/fleet_facility/maps'
    l0 = SpawnGrid(maps / 'map.yaml')
    l1 = SpawnGrid(maps / 'map_L1.yaml')
    import yaml
    assert yaml.safe_load((maps / 'map_L1.yaml').read_text())['origin'][2] == 0.0
    assert all(l0.clear(x, 0.0, .23) for x in (2.5, 3.5, 4.0))
    assert all(l1.clear(x, 0.0, .23) for x in (2.0, 3.0, 4.0))


def test_lift_door_panel_clears_platform_and_car_walls():
    world = ROOT / 'src/robot_lab_maps/maps/fleet_facility/worlds/fleet_facility.world'
    root = ET.parse(world).getroot()
    door = root.find(".//link[@name='lift_door']")
    pose = [float(value) for value in door.findtext('pose').split()]
    size = [float(value) for value in door.findtext('collision/geometry/box/size').split()]
    assert pose[0] - size[0] / 2 >= 3.0
    assert size[1] < 1.82
    plugins = root.findall(".//model[@name='facility']/plugin[@name='ignition::gazebo::systems::JointPositionController']")
    plugin = next(item for item in plugins if item.findtext('joint_name') == 'door_lift')
    assert float(plugin.findtext('p_gain')) == 1200.0
    assert float(plugin.findtext('cmd_max')) == 300.0
    joint = root.find(".//joint[@name='door_lift']")
    assert float(joint.findtext('axis/limit/velocity')) == .25
    config = yaml.safe_load((ROOT / 'src/robot_lab_maps/maps/fleet_facility/infrastructure.yaml').read_text())
    assert config['doors']['lift']['opened'] == 2.15


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


def test_charging_stops_on_departure_and_feedback_loss(tmp_path):
    from robot_lab_utils.fleet_energy import EnergyEstimate
    node = _make_node(_make_session(tmp_path))
    model = EnergyEstimate(node.config['energy'], ['left', 'right'], True)
    for stamp in (1., 1.5):
        model.observe(['left', 'right'], [0., 0.], [0., 0.], stamp, stamp, stamp, True)
    model.observe(['left', 'right'], [0., 0.], [0., 0.], 2., 2., 2., False)
    assert model.state(2.)['charging_W'] == 0.
    assert model.state(2.7)['fraction'] is None
    for stamp in (3., 3.5):
        model.observe(['left', 'right'], [0., 0.], [0., 0.], stamp, stamp, stamp, True)
    assert model.state(3.5)['fraction'] is None  # Unobserved energy cannot be recovered.
