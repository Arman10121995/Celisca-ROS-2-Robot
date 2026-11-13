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
    panel.app.process = Mock(args=command())
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
