"""Owned Nav2 action transitions in the mobile pick-and-place node."""
from concurrent.futures import Future
import importlib.util
import json
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


def test_calibrated_close_meters_only_the_shaft_during_closure(pick_place):
    node = object.__new__(pick_place.PickPlace)
    node.spec = SimpleNamespace(aperture=[(0., 0.), (1., .10)])
    node.measured = lambda: {'shaft': .36}
    close = {'name': 'close', 'fingers': {'shaft': .31}}
    outputs = [node._stage_fingers(close)['shaft'] for _ in range(7)]
    assert outputs == pytest.approx([.35, .34, .33, .32, .31, .31, .31])
    release = {'name': 'release', 'fingers': {'shaft': 1.}}
    assert node._stage_fingers(release) == release['fingers']


def test_pick_place_uses_authored_collision_pose_from_measured_model_pose(pick_place, tmp_path):
    node = object.__new__(pick_place.PickPlace)
    node.object_shapes, node.object_collision_frames = {}, {}
    world = tmp_path/'shifted_object.world'
    world.write_text('''<sdf version="1.7"><world name="lab">
      <model name="block"><static>false</static>
        <link name="link"><pose>0.1 0 0 0 0 1.5707963267948966</pose>
          <inertial><mass>0.04</mass></inertial>
          <collision name="c"><pose>0.02 0 0 0 0 0.4</pose>
            <geometry><box><size>0.04 0.03 0.08</size></box></geometry>
          </collision>
        </link>
      </model>
      <model name="ball"><static>false</static>
        <link name="link"><inertial><mass>0.04</mass></inertial>
          <collision name="c"><geometry><sphere><radius>0.02</radius></sphere></geometry></collision>
        </link>
      </model>
      <model name="drawing"><static>true</static><link name="link">
        <visual name="v"><geometry><box><size>0.3 0.3 0.1</size></box></geometry></visual>
      </link></model>
      <model name="ramp"><static>true</static><pose>0 0 0 0.3 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.3 0.3 0.1</size></box></geometry></collision>
      </link></model>
      <model name="table"><static>true</static><pose>0 0 0 0 0 0.3</pose><link name="link">
        <collision name="top"><pose>0 0 0.52 0 0 0</pose>
          <geometry><box><size>0.3 0.3 0.04</size></box></geometry>
        </collision>
        <collision name="leg"><geometry><box><size>0.03 0.03 0.5</size></box></geometry></collision>
      </link></model>
      <model name="floor"><static>true</static><pose>0 0 -0.05 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>8 8 0.1</size></box></geometry></collision>
      </link></model>
      <model name="wall"><static>true</static><pose>0 0 1 0 0 0</pose><link name="link">
        <collision name="c"><geometry><box><size>0.1 4 2</size></box></geometry></collision>
      </link></model>
      <model name="pillar"><static>true</static><link name="link">
        <collision name="c"><geometry><cylinder><radius>0.2</radius><length>1</length></cylinder></geometry></collision>
      </link></model>
    </world></sdf>''')
    node.get_parameter = lambda name: SimpleNamespace(value=str(world))
    supports, sizes = node._world_boxes()
    assert sizes == {'block': [.04, .03, .08]}
    assert set(supports) == {'table'}
    assert supports['table'].top == pytest.approx(.54)
    assert sorted(box.name.split('/')[-1] for box in node.arm_obstacles) == ['floor', 'pillar', 'ramp', 'table', 'table', 'wall']
    assert [box.name.split('/')[-1] for box in node.arm_obstacles if box.floor] == ['floor']
    pillar, = [geometry for geometry in node.arm_obstacles if geometry.shape == 'cylinder']
    assert pillar.size == [.4, .4, 1.]
    assert sorted(box.name for box in node.drive_obstacles) == ['pillar', 'ramp', 'table', 'table', 'wall']
    ramp, = [box for box in node.drive_obstacles if box.name == 'ramp']
    assert ramp.size[:2] == pytest.approx([.3, .3*math.cos(.3)+.1*math.sin(.3)])
    s = math.sqrt(.5)
    node._on_truth(SimpleNamespace(data=json.dumps({'block': [2., 1., .5, s, 0., 0., s],
                                                  'ball': [0., 0., 1., 1., 0., 0., 0.]})))
    assert node.truth['block'] == pytest.approx([1.98, 1.1, .5, -math.sin(.2), 0., 0., math.cos(.2)])
    assert 'ball' not in node.truth
    node._on_truth(SimpleNamespace(data=json.dumps({'block': [2., 1., .5, 0., 0., 0., 0.]})))
    assert not node.truth


def test_world_loads_scaled_mesh_parts_as_obstacles_and_reports_missing_meshes(pick_place, tmp_path):
    pytest.importorskip('trimesh')
    node = object.__new__(pick_place.PickPlace)
    node.object_shapes, node.object_collision_frames = {}, {}
    mesh = tmp_path/'tetra.obj'
    mesh.write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n')
    world = tmp_path/'lab.world'
    world.write_text('''<sdf version="1.7"><world name="lab">
      <model name="fixture"><static>true</static><link name="link">
        <collision name="side"><pose>1 2 0.4 0 0 1.5707963267948966</pose><geometry>
          <mesh><uri>tetra.obj</uri><scale>0.2 0.4 0.6</scale></mesh>
        </geometry></collision>
        <collision name="missing"><geometry><mesh><uri>missing.obj</uri></mesh></geometry></collision>
      </link></model></world></sdf>''')
    node.get_parameter = lambda name: SimpleNamespace(value=str(world))
    supports, _ = node._world_boxes()
    assert not supports and node.scene_error == 'unresolved mesh missing.obj'
    obstacle, = node.arm_obstacles
    assert obstacle.shape == 'mesh'
    assert obstacle.size == pytest.approx([.2, .4, .6])
    assert obstacle.position == pytest.approx([.8, 2.1, .7])
    assert node.drive_obstacles[0].size[:2] == pytest.approx([.2, .4])


def test_pad_settling_needs_independent_bilateral_load_and_stable_payload(pick_place, monkeypatch):
    node = object.__new__(pick_place.PickPlace)
    node.task = {'object': 'block'}
    node.min_pad_force = .1
    node.truth, node.truth_at = {'block': [0., 0., .5]}, 10.
    monkeypatch.setattr(pick_place, '_now', lambda: 10.)
    stage = {}
    contacts = {'sim_time_s': 2., 'normal_force_n': {'block': {'left': .9, 'right': 1.1}}}
    node.state = {'grip_contacts': contacts}
    for _ in range(4):
        assert not node._pads_settled(stage)
    contacts['sim_time_s'] = 2.1
    assert not node._pads_settled(stage)
    contacts['sim_time_s'] = 2.2
    node.truth['block'] = [.008, 0., .5]
    assert not node._pads_settled(stage)
    contacts['sim_time_s'] = 2.3
    assert not node._pads_settled(stage)
    contacts['sim_time_s'] = 2.4
    assert node._pads_settled(stage)
    contacts['normal_force_n']['block']['right'] = 0.
    assert not node._pads_settled(stage)
    assert not stage['pad_settle']
    assert not node._pads_lost(stage, 'block')
    monkeypatch.setattr(pick_place, '_now', lambda: 10.5)
    assert node._pads_lost(stage, 'block')
    contacts['normal_force_n']['block']['right'] = .5
    assert not node._pads_lost(stage, 'block')
    assert 'pads_lost_at' not in stage


def test_shifted_calibrated_grasp_reapproaches_the_original_support_once(pick_place):
    node = object.__new__(pick_place.PickPlace)
    node.config = {'retry_shifted_grasp': True}
    node.spec = SimpleNamespace(pad_geoms=('left_pad', 'right_pad'))
    node._pad_check = lambda name: {'pad_normal_force_n': {'left': 0., 'right': .3}}
    node._table_under = lambda name: SimpleNamespace(name='pickup')
    node._plan_arm = Mock()
    node.carried_footprint = None
    node.state = {'coupling_error': {'follower': .12}}
    node.truth = {'block': [.02, 0., .5, 1., 0., 0., 0.]}
    phase = {'name': 'pick'}
    task = dict(object='block', table='pickup', object_start=[0., 0., .5], pose_source='rgbd', log=[])
    assert not node._retry_after_close_timeout(task, phase)
    task['pose_source'] = 'truth'
    task['table'] = 'different support'
    assert not node._retry_after_close_timeout(task, phase)
    task['table'] = 'pickup'
    assert node._retry_after_close_timeout(task, phase)
    assert task['log'][0]['displacement_m'] == .02
    assert task['log'][0]['max_follower_error_rad'] == .12
    assert task['object_start'] == [0., 0., .5]
    assert task['reapproach_support'] == 'pickup'
    assert not node._retry_after_close_timeout(task, phase)
    node._plan_arm.assert_called_once_with(task, phase, reapproach=True)


def test_retry_uses_fresh_post_retract_pose_on_the_same_support(pick_place, monkeypatch):
    node = object.__new__(pick_place.PickPlace)
    node.mobile, node.base_twist = True, (0., 0.)
    task = dict(object='block', object_start=[0., 0., .5], reapproach_support='pickup')
    node.truth = {'block': [.02, 0., .5]}
    node.truth_at = 10.
    node._table_under = lambda name: SimpleNamespace(name='pickup')
    monkeypatch.setattr(pick_place, '_now', lambda: node.truth_at + .05)
    assert node._check(task, 'retract_grasp') is None
    assert node._check(task, 'retract_grasp') is None
    node.truth['block'][0] = .031
    node.truth_at += .1
    assert node._check(task, 'retract_grasp') is None
    for _ in range(2):
        node.truth_at += .1
        result = node._check(task, 'retract_grasp')
    assert result['passed'] and result['samples'] == 3
    node._table_under = lambda name: SimpleNamespace(name='another table')
    assert not node._check(task, 'retract_grasp')['passed']


def test_pregrasp_anchor_catches_heading_drift_after_base_stops(pick_place, monkeypatch):
    node = object.__new__(pick_place.PickPlace)
    node.base_pose, node.base_twist = (.8808, -.0004, -.0034, -.0146), (.0, .0)
    node.truth, node.base_at, node.truth_at = {'block': [1.36, 0., .545]}, 10., 10.
    node._table_under = lambda name: SimpleNamespace(name='pickup')
    monkeypatch.setattr(pick_place, '_now', lambda: 10.1)
    task = dict(object='block', table='pickup', pregrasp_still_since=9.,
                scene=SimpleNamespace(base=(.883, .0004, -.0036, -.0237)),
                obj=SimpleNamespace(position=[1.36, 0., .545]))
    first = node._check_pregrasp_anchor(task, [1.361, 0., .60])
    assert first['passed'] and first['replan']
    assert first['projected_tool_drift_m'] == pytest.approx(.00417, abs=.0001)
    task['pregrasp_replanned'] = True
    assert not node._check_pregrasp_anchor(task, [1.361, 0., .60])['passed']
    task['scene'].base = node.base_pose
    second = node._check_pregrasp_anchor(task, [1.361, 0., .60])
    assert second['passed'] and not second['replan']
    node.base_twist = (.05, .0)
    assert node._check_pregrasp_anchor(task, [1.361, 0., .60]) is None


def test_holonomic_dock_corrects_the_measured_pose_after_settling(pick_place, monkeypatch):
    node = object.__new__(pick_place.PickPlace)
    node.base_pose = (.8686, .01, -.002, -.0531)
    node.base_at, node.base_twist, node.state = 10., (0., 0.), {}
    node.drive_obstacles, node.mobile = [], {'holonomic_docking': True, 'base_radius': .37}
    node.dock_tolerance = .01
    node._twist, node._next_phase = Mock(), Mock()
    monkeypatch.setattr(pick_place, '_now', lambda: 10.)
    task = dict(interlock={'drive_ticks': 0}, log=[])
    goal = dict(x=.89, y=0., yaw=0.)
    phase = dict(started=9., waypoints=[goal], waypoint=0, at_point=True)
    node._drive(task, phase)
    args, kwargs = node._twist.call_args
    assert args[0] > 0 and args[1] > 0 and kwargs['vy'] < 0
    assert math.hypot(args[0], kwargs['vy']) <= pick_place.HOLONOMIC_DOCK_SPEED_MPS
    assert phase['precision_dock'] == goal
    phase.update(waypoint=1, still=9.)
    node._drive(task, phase)
    assert phase['waypoint'] == 0 and phase['dock_recaptures'] == 1
    node._next_phase.assert_not_called()
    node.base_pose = (.886, .003, -.002, .01)
    phase.update(waypoint=1, still=9.)
    phase['name'] = 'to_object'
    node._drive(task, phase)
    node._next_phase.assert_called_once_with(task)
    assert task['log'][0]['dock_recaptures'] == 1


def test_direct_mobile_legs_check_primitive_obstacles_and_keep_nav2_transit(pick_place):
    from robot_lab_mujoco.arm_pick_place import Box

    node = object.__new__(pick_place.PickPlace)
    node.mobile = dict(pick_place.MOBILE_DEFAULTS)
    # This wall blocks the transit to the run-up pose. Nav2 can route around it.
    node.drive_obstacles = [Box('hall wall', (1., 0., .8), (1., 0., 0., 0.), (.1, .3, 1.6))]
    start = (0., 0., 0., 0.)
    with pytest.raises(ValueError, match='clear of hall wall'):
        node._route({'transit': 'straight'}, (3., 0., 0.), start, 0., .35)
    assert node._route({'transit': 'nav2'}, (3., 0., 0.), start, 0., .35)[0]['nav2']

    node.drive_obstacles.append(Box('dock post', (2.9, 0., .5), (1., 0., 0., 0.), (.1, .1, 1.)))
    with pytest.raises(ValueError, match='clear of dock post'):
        node._route({'transit': 'nav2'}, (3., 0., 0.), start, 0., .35)

    node.base_pose = (1.6, 0., 0., 0.)
    with pytest.raises(ValueError, match='Retreat blocked by hall wall'):
        node._start_phase(dict(phases=[dict(kind='drive', name='clear')], phase_index=0))


def test_fixed_arm_starts_with_camera_and_plans_from_its_estimate(pick_place, monkeypatch):
    node = object.__new__(pick_place.PickPlace)
    monkeypatch.setattr(pick_place, '_now', lambda: 10.)
    node.mobile, node.look, node.config = None, {}, {}
    node.truth, node.truth_at = {'cube': [.5, -.25, .32, 1., 0., 0., 0.]}, 10.
    node.supports = {'drop': object()}
    node.rgbd_enabled = True
    node.rgbd_bindings = {'cube': object()}
    node.spec = SimpleNamespace(arm_joints=('arm',), finger_joints=('jaw',))
    node._feedback_blocker = lambda: None
    node.measured = lambda: {'arm': 0., 'jaw': 0.}
    node._plan_arm = Mock()
    task = node._plan({'object': 'cube', 'support': 'drop', 'pose_source': 'rgbd'})
    look, arm = task['phases']
    assert look['kind'] == 'look' and look['targets'] == {}
    node._plan_arm.assert_not_called()
    task['perceived'] = {'position': [.501, -.25, .32], 'orientation_wxyz': [1., 0., 0., 0.]}
    assert node._object_pose(task, arm) == [.501, -.25, .32, 1., 0., 0., 0.]
    node._next_phase(task)
    node._plan_arm.assert_called_once_with(task, arm)


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
    node.mobile = {'nav_base_footprint': [[.29, .205], [.29, -.205], [-.29, -.205], [-.29, .205]]}
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


def test_rejected_dock_goal_records_rejection_and_stops(pick_place):
    node = object.__new__(pick_place.PickPlace)
    node.amcl_pose, node.base_pose = None, (0., 0., 0., 0.)
    node._finish = Mock()
    future = Future()
    future.set_result(SimpleNamespace(accepted=False))
    phase = dict(nav_goal=dict(sent=pick_place._now(), future=future, result=None), name='to_object', waypoint=0)
    task = dict(log=[])

    node._nav2_leg(task, phase, dict(x=2., y=1., yaw=0.))

    assert phase['waypoint'] == 0
    assert task['log'][0]['status'] == 'rejected'
    node._finish.assert_called_once_with('failed', 'to_object: Nav2 rejected the goal')


def test_held_arm_interlock_leaves_existing_nav2_goal_owned(pick_place):
    node = object.__new__(pick_place.PickPlace)
    node.base_pose, node.base_at = (0., 0., 0., 0.), pick_place._now()
    node._nav2_leg, node._twist, node._finish = Mock(), Mock(), Mock()
    target = dict(x=2., y=1., yaw=0., nav2=True)
    phase = dict(started=node.base_at, waypoint=0, waypoints=[target])
    task = dict(interlock=dict(drive_ticks=0, drive_ticks_with_articulation_active=0))
    node.state = dict(base_inhibited=True, holding=True)

    node._drive(task, phase)
    node._twist.assert_called_once_with(0., 0.)
    node._nav2_leg.assert_not_called()
    node._twist.reset_mock()
    phase['nav_goal'] = dict()
    node._drive(task, phase)
    node._nav2_leg.assert_called_once_with(task, phase, target)
    node._twist.assert_not_called()

    node.state['holding'] = False
    node._drive(task, phase)
    node._finish.assert_called_once_with('failed', 'Articulation interlock during a Nav2 leg')


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
