"""Generic mobile-arm page: profile joints, real contract feedback and Tk input."""
import json
import math
import os
from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import ttk
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_lab_gui import molar_controls
from robot_lab_gui.molar_controls import SCHEMA
from robot_lab_gui.urdf_arm_controls import UrdfArmControls, arm_config, selectable, validate_arm_feedback

ROOT = Path(__file__).resolve().parents[3]
J1, J2 = 'pro630_joint1_to_base', 'pro630_joint2_to_joint1'
URDF = '''<robot name="arm">
 <link name="base_link"/><link name="pro630_base"/><link name="pro630_link1"/><link name="pro630_link2"/>
 <joint name="pro630_mount_joint" type="fixed"><parent link="base_link"/><child link="pro630_base"/></joint>
 <joint name="%s" type="revolute"><parent link="pro630_base"/><child link="pro630_link1"/>
  <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="5" velocity="1"/></joint>
 <joint name="%s" type="revolute"><parent link="pro630_link1"/><child link="pro630_link2"/>
  <axis xyz="0 0 -1"/><limit lower="-2.6" upper="2.6" effort="43.4" velocity="1"/></joint>
</robot>''' % (J1, J2)
CONFIG = dict(schema=SCHEMA, panel='arm', home={J1: 0., J2: 0.}, base_motion_limits=[.1, .2], joints={
    J1: dict(mode='position', lower=-3.14, upper=3.14, max_velocity=1., effort=5., kp=200., kd=4.47),
    J2: dict(mode='position', lower=-2.6, upper=2.6, max_velocity=1., effort=43.4, kp=1736., kd=13.18)})


def command(backend='isaac'):
    return ['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py',
            'robot_model:=asset_elephant_myagv_pro_pro630', 'simulator:=' + backend, 'mode:=display',
            'map_name:=nav_empty']


@pytest.fixture
def model(tmp_path):
    path = tmp_path / 'drive.urdf'
    path.write_text(URDF)
    return path


def contract_state(model, positions=(.1, -.2)):
    """Feedback from the real shared contract, not a hand-written state."""
    sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
    from robot_lab_utils.urdf_articulation import URDFArticulation
    contract = URDFArticulation(CONFIG, URDF, str(model))
    contract.step(dict(zip((J1, J2), positions)), {J1: 0., J2: 0.}, .01)
    return contract.state()


def test_selectable_only_for_arm_panel_profiles():
    profile = {'articulation': CONFIG}
    for backend in molar_controls.BACKENDS:
        assert selectable('asset_elephant_myagv_pro_pro630', backend, 'nav', 'simulation', profile)
    assert not selectable('x', 'mujoco', 'display', 'hardware', profile)
    assert not selectable('x', 'mujoco', 'gait', 'simulation', profile)
    assert not selectable('x', 'other', 'display', 'simulation', profile)
    molar = {'articulation': dict(CONFIG, panel=None)}
    assert not selectable('molar_hri', 'mujoco', 'display', 'simulation', molar)
    velocity = {'articulation': dict(CONFIG, joints={J1: dict(CONFIG['joints'][J1], mode='velocity')})}
    assert arm_config(velocity) is None
    assert arm_config({}) is None


def test_real_contract_feedback_validates_and_mismatches_are_rejected(model):
    state = contract_state(model)
    channels = validate_arm_feedback(state, model, CONFIG)
    assert set(channels) == {J1, J2} and channels[J2]['measured'] == -.2
    for mutate in (lambda s: s['channels'].pop(),
                   lambda s: s['channels'][1].update(name=J1),
                   lambda s: s['channels'][0].update(limits=[-4., 3.14]),
                   lambda s: s['channels'][0].update(mode='velocity'),
                   lambda s: s['channels'][0].update(coupled=True),
                   lambda s: s['channels'][0].update(measured=math.nan),
                   lambda s: s['channels'][0].update(max_velocity=0.),
                   lambda s: s.update(state_monotonic=time.monotonic() - 10.),
                   lambda s: s.update(model_sha256='b' * 64)):
        bad = json.loads(json.dumps(state))
        mutate(bad)
        with pytest.raises((ValueError, KeyError)):
            validate_arm_feedback(bad, model, CONFIG)


@pytest.fixture
def panel(model, monkeypatch):
    if not os.environ.get('DISPLAY'):
        pytest.skip('Actual Tk requires Xvfb or a display')
    app = tk.Tk()
    app.geometry('720x850')
    app.robot_var = tk.StringVar(value='asset_elephant_myagv_pro_pro630')
    app.simulator_var = tk.StringVar(value='isaac')
    app.mode_var = tk.StringVar(value='display')
    app.launch_kind_var = tk.StringVar(value='simulation')
    app.map_var = tk.StringVar(value='nav_empty')
    app._robot_config = lambda: {'articulation': CONFIG}
    app._resolve_robot_path = lambda: str(model)
    app.process = None
    app._launch_running = False
    app.notebook = ttk.Notebook(app)
    app.notebook.pack(fill='both', expand=True)
    app.launch_tab = ttk.Frame(app.notebook)
    app.notebook.add(app.launch_tab, text='Launch')
    app.other_tab = ttk.Frame(app.notebook)
    app.notebook.add(app.other_tab, text='Other')
    app.control_notebook = ttk.Notebook(app.launch_tab)
    app.control_notebook.pack(fill='both', expand=True)
    widget = UrdfArmControls(app.control_notebook, app)
    app.drive_page = ttk.Frame(app.control_notebook)
    app.control_notebook.add(app.drive_page, text='Drive')
    app.update()
    monkeypatch.setattr(molar_controls, 'String', lambda **kw: SimpleNamespace(**kw), raising=False)
    monkeypatch.setattr(molar_controls, 'Empty', lambda: SimpleNamespace(), raising=False)
    monkeypatch.setattr(molar_controls, 'Trigger', SimpleNamespace(Request=lambda: SimpleNamespace()), raising=False)
    try:
        yield widget
    finally:
        widget.close()
        for job in app.tk.call('after', 'info'):
            app.tk.call('after', 'cancel', job)
        app.destroy()


def activate(panel, model):
    panel.app.process = Mock(args=command(panel.app.simulator_var.get()))
    panel.app.process.poll.return_value = None
    panel.app._launch_running = True
    panel.node = Mock()
    panel.node.get_publishers_info_by_topic.return_value = [SimpleNamespace(endpoint_gid=b'arm-gid')]
    panel.node.count_subscribers.return_value = 1
    panel.pub, panel.heartbeat, panel.stop_client = Mock(), Mock(), Mock()
    panel.stop_client.service_is_ready.return_value = True
    panel.receive(SimpleNamespace(data=json.dumps(contract_state(model))),
                  SimpleNamespace(source_timestamp=time.time_ns(), publisher_gid=b'arm-gid'))


def sent(panel):
    return json.loads(panel.pub.publish.call_args.args[0].data)


def test_rows_follow_profile_and_only_explicit_actions_publish(panel, model):
    assert panel.joint_names == (J1, J2)
    assert [panel.joint_frame.winfo_children()[i].cget('text') for i in (0, 1)] == [
        'J1 · joint1_to_base', 'J2 · joint2_to_joint1']
    activate(panel, model)
    assert panel.ready()
    assert panel.target_vars[J2].get() == pytest.approx(-.2)       # measured, not zero
    panel.after_cancel(panel.job)
    panel.tick()
    assert 'J2 -0.200 rad (-11.5°)' in panel.feedback.get()
    panel.scales[J1].set(1.)
    panel.app.update()
    panel.pub.publish.assert_not_called()
    panel.apply_buttons[J1].invoke()
    assert sent(panel)['targets'] == {J1: 1.}
    panel.jog_buttons[J2, 1].invoke()
    assert sent(panel)['targets'][J2] == pytest.approx(-.15)
    panel.home_button.invoke()
    assert sent(panel)['targets'] == {J1: 0., J2: 0.}
    count = panel.pub.publish.call_count
    panel.target_vars[J2].set(2.7)                                   # outside the source limit
    panel.apply_buttons[J2].invoke()
    assert panel.pub.publish.call_count == count


def test_ownership_loss_holds_and_revokes(panel, model):
    activate(panel, model)
    panel.app.control_notebook.select(panel.app.drive_page)
    panel.app.update()
    assert panel.state is None and not panel.ready()
    panel.stop_client.call_async.assert_called_once()
    assert not panel.apply(J1)
    panel.pub.publish.assert_not_called()


def enable_planning(panel, model):
    panel.app.simulator_var.set('mujoco')
    panel.app._robot_config = lambda: {'articulation': dict(CONFIG,
        planning=dict(schema='robot_lab.articulation_plan.v1', backend='mujoco'))}
    activate(panel, model)
    panel.state['planning'] = dict(schema='robot_lab.articulation_plan.v1', available=True,
                                  phase='idle', message='Choose joint targets')


def test_tool_jog_edits_without_moving_then_requires_matching_plan(panel, model):
    enable_planning(panel, model)
    articulation = dict(panel.arm(), cartesian=dict(schema='robot_lab.cartesian_goal.v1',
                                                    reference_frame='base_link', tip_frame='tool'))
    panel.app._robot_config = lambda: {'articulation': articulation}
    state = panel.state['planning']
    state['tool'] = dict(schema='robot_lab.cartesian_goal.v1', frame='base_link', tip_frame='tool',
                         position=[.4, 0., .6], rpy_deg=[0., 30., 0.])
    panel.update_buttons()
    panel.jog_tool(2, 1)
    panel.pub.publish.assert_not_called()
    panel.plan_pose_button.invoke()
    request = sent(panel)
    assert request['operation'] == 'plan_cartesian'
    assert request['pose'] == dict(frame='base_link', position=[.4, 0., .62], orientation='keep')
    state.update(phase='ready', request_id=request['request_id'], plan_id='0'*32,
                 cartesian_request=request['pose'])
    panel.update_buttons()
    assert panel.execute_button.instate(['!disabled'])
    assert panel.execute_plan()
    assert sent(panel)['operation'] == 'execute_plan'
    panel.jog_tool(0, -1)
    assert not panel.execute_plan()


def test_plan_execute_are_separate_and_edits_revoke_local_execution(panel, model):
    enable_planning(panel, model)
    panel.target_vars[J1].set(.4)
    assert panel.plan_targets()
    request = sent(panel)
    assert request['operation'] == 'plan' and request['targets'][J1] == .4
    assert request['plant_id'] == panel.state['plant_id']
    assert request['model_sha256'] == panel.state['model_sha256']
    assert panel.pub.publish.call_count == 1
    panel.state['planning'].update(phase='ready', plan_id='a'*32,
        request_id=request['request_id'], targets=request['targets'])
    assert panel.plan_matches()
    panel.target_vars[J1].set(.5)
    assert not panel.execute_plan() and panel.pub.publish.call_count == 1
    panel.target_vars[J1].set(.4)
    assert panel.execute_plan()
    execute = sent(panel)
    assert execute['operation'] == 'execute_plan' and execute['plan_id'] == 'a'*32
    assert 'targets' not in execute
    panel.stop()
    assert not panel.plan_request and not panel.plan_matches()


def test_plan_home_builds_source_target_without_sending_motion(panel, model):
    enable_planning(panel, model)
    assert panel.plan_home()
    assert sent(panel)['operation'] == 'plan'
    assert sent(panel)['targets'] == {J1: 0., J2: 0.}
    assert panel.pub.publish.call_count == 1
    panel.app.simulator_var.set('isaac')
    assert not panel.planning_enabled() and not panel.execute_plan()


def test_a_mechanically_coupled_finger_is_reported_not_commanded(tmp_path):
    from robot_lab_utils.urdf_articulation import URDFArticulation
    urdf = '''<robot name="hand"><link name="base"/><link name="left"/><link name="right"/>
     <joint name="f1" type="prismatic"><parent link="base"/><child link="left"/><axis xyz="0 1 0"/>
      <limit lower="-0.001" upper="0.04" effort="20" velocity="0.3"/></joint>
     <joint name="f2" type="prismatic"><parent link="base"/><child link="right"/><axis xyz="0 -1 0"/>
      <limit lower="-0.001" upper="0.04" effort="20" velocity="0.3"/></joint></robot>'''
    path = tmp_path / 'hand.urdf'
    path.write_text(urdf)
    config = dict(schema=SCHEMA, panel='arm', home={'f1': .04}, couplings={
        'f2': dict(joint='f1', multiplier=1., offset=0., mechanical=True)}, joints={
        'f1': dict(mode='position', lower=-.001, upper=.04, max_velocity=.025, effort=20., kp=800., kd=20.)})
    contract = URDFArticulation(config, urdf, str(path))
    contract.step({'f1': .02, 'f2': .02}, {'f1': 0., 'f2': 0.}, .01)
    found = validate_arm_feedback(contract.state(), str(path), config)
    assert set(found) == {'f1', 'f2'} and found['f2']['coupled']
    assert arm_config({'articulation': config})['joints'].keys() == {'f1'}


def test_pick_place_runs_only_on_request_with_fresh_idle_node(panel, model):
    profile = {'articulation': CONFIG, 'pick_place': {'arm_joints': [J1, J2]}}
    panel.app._robot_config = lambda: profile
    activate(panel, model)
    panel.pick_pub = Mock()
    panel.update_pick_place()
    assert panel.pick_run_button.instate(['disabled'])              # no node state yet
    state = dict(schema='robot_lab.pick_place_state.v1', phase='idle', message='ready',
                 objects=['pickup_cube'], supports=['pickup_table', 'place_table'], last_result=None)
    panel.receive_pick_place(SimpleNamespace(data=json.dumps(state)))
    panel.update_pick_place()
    assert panel.pick_object.get() == 'pickup_cube' and panel.pick_run_button.instate(['!disabled'])
    panel.pick_support.set('place_table')
    panel.pick_pub.publish.assert_not_called()                      # choosing sends nothing
    assert panel.run_pick_place()
    request = json.loads(panel.pick_pub.publish.call_args.args[0].data)
    assert request == dict(object='pickup_cube', support='place_table', lift=.1, pose_source='truth')
    panel.receive_pick_place(SimpleNamespace(data=json.dumps(dict(state, phase='executing'))))
    panel.update_pick_place()
    assert not panel.run_pick_place() and panel.pick_stop_button.instate(['!disabled'])
    panel.pick_lift.set(.5)
    panel.receive_pick_place(SimpleNamespace(data=json.dumps(state)))
    assert not panel.run_pick_place()                               # lift outside 0.03-0.25 m


def test_camera_pose_source_is_offered_only_when_the_node_has_it(panel, model):
    profile = {'articulation': CONFIG, 'pick_place': {'arm_joints': [J1, J2], 'mobile': {}}}
    panel.app._robot_config = lambda: profile
    activate(panel, model)
    panel.pick_pub = Mock()
    state = dict(schema='robot_lab.pick_place_state.v1', phase='idle', message='ready', mobile=True,
                 objects=['fetch_block', 'other_block'], supports=['drop_table'], last_result=None)
    panel.receive_pick_place(SimpleNamespace(data=json.dumps(state)))
    panel.update_pick_place()
    assert list(panel.pick_source_box['values']) == ['Simulator truth']
    state.update(pose_sources=['truth', 'rgbd'], rgbd_objects=['fetch_block'])
    panel.receive_pick_place(SimpleNamespace(data=json.dumps(state)))
    panel.update_pick_place()
    assert list(panel.pick_source_box['values']) == ['Simulator truth', 'RGB-D camera']
    panel.pick_source.set('RGB-D camera')
    panel.pick_object.set('other_block')
    assert not panel.run_pick_place()                               # no declared colour and size
    panel.pick_pub.publish.assert_not_called()
    panel.pick_object.set('fetch_block')
    assert panel.run_pick_place()
    assert json.loads(panel.pick_pub.publish.call_args.args[0].data)['pose_source'] == 'rgbd'


def test_mobile_node_labels_the_section_as_fetch(panel, model):
    profile = {'articulation': CONFIG, 'pick_place': {'arm_joints': [J1, J2], 'mobile': {}}}
    panel.app._robot_config = lambda: profile
    activate(panel, model)
    state = dict(schema='robot_lab.pick_place_state.v1', phase='idle', message='ready', mobile=True,
                 objects=['fetch_block'], supports=['drop_table', 'pickup_table'], last_result=None)
    panel.receive_pick_place(SimpleNamespace(data=json.dumps(state)))
    panel.update_pick_place()
    assert panel.pick_frame.cget('text').startswith('Fetch') and panel.pick_run_button.cget('text') == 'Run fetch'
    panel.pick_pub = Mock()
    panel.receive_pick_place(SimpleNamespace(data=json.dumps(dict(state, restart_required=True,
        message='Loaded footprint retained. Restart the simulation before another fetch.'))))
    panel.update_pick_place()
    assert panel.pick_run_button.instate(['disabled'])
    assert not panel.run_pick_place()
    panel.pick_pub.publish.assert_not_called()
    panel.receive_pick_place(SimpleNamespace(data=json.dumps(dict(state, mobile=False))))
    panel.update_pick_place()
    assert panel.pick_run_button.cget('text') == 'Run pick / place'


def test_gripper_presets_are_explicit_and_held_joints_are_excluded_from_plans(panel, model):
    config = dict(CONFIG, planning={'joints': [J1]}, grip_presets={J2: {'open': .3, 'close': 0.}})
    panel.app._robot_config = lambda: {'articulation': config}
    panel.joint_names = ()
    panel.rebuild()
    assert [button.cget('text') for button in panel.grip_buttons] == ['Open', 'Close']
    assert not panel.apply_grip_preset(J2, .3)
    activate(panel, model)
    panel.apply_grip_preset(J2, .3)
    assert json.loads(panel.pub.publish.call_args.args[0].data)['targets'] == {J2: .3}
    assert panel.current_targets() == {J1: panel.target_vars[J1].get()}
