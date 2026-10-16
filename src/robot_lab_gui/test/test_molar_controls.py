"""MOLAR control ownership, source identity and actual Tk input contracts."""
import copy
import hashlib
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
from robot_lab_gui import molar_controls as controls
from robot_lab_gui.molar_controls import (
    BACKENDS, COUPLED, LIFT, LIMITS, ROLLER, SCHEMA, STAGE,
    MolarControls, bounded_targets, launch_matches, selectable, validate_feedback,
)


def command(backend='mujoco'):
    return ['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py',
            'robot_model:=molar_hri', 'simulator:=' + backend, 'mode:=display', 'map_name:=nav_empty']


@pytest.fixture
def model(tmp_path):
    path = tmp_path / 'molar_hri.urdf.xacro'
    path.write_text('<robot name="molar_hri"/>\n')
    return path


def feedback(path, plant='plant-one'):
    channels = []
    for name, measured in ((LIFT, .2), (STAGE, .03), (COUPLED, .03), (ROLLER, 12.)):
        channels.append(dict(name=name, mode='velocity' if name == ROLLER else 'position',
                             units='rad/s' if name == ROLLER else 'm', measured=measured,
                             target=.5 if name == ROLLER else measured, measured_velocity=.5 if name == ROLLER else 0.,
                             limits=list(LIMITS[name]), max_velocity=8. if name == ROLLER else .05,
                             coupled=name == COUPLED))
    return dict(schema=SCHEMA, plant_id=plant, model_path=str(path),
                model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), urdf_sha256='a' * 64,
                initialized=True, fault='', status='Holding measured articulation', active=False,
                base_inhibited=True, coupling_error={COUPLED: 0.}, channels=channels)


@pytest.mark.parametrize('backend', BACKENDS)
def test_all_four_backend_candidates_require_exact_owned_display_launch(backend):
    profile = {'articulation': {'schema': SCHEMA}}
    assert selectable('molar_hri', backend, 'display', 'simulation', profile)
    assert selectable('probot_l', backend, 'display', 'simulation', profile)
    assert launch_matches(command(backend), 'molar_hri', backend, 'nav_empty')
    for changed in ('robot_model:=probot_l', 'simulator:=other', 'mode:=nav', 'map_name:=nav_obstacle'):
        key = changed.split(':=')[0] + ':='
        args = [changed if arg.startswith(key) else arg for arg in command(backend)]
        assert not launch_matches(args, 'molar_hri', backend, 'nav_empty')
        assert not launch_matches(command(backend) + [changed], 'molar_hri', backend, 'nav_empty')
    assert not launch_matches(' '.join(command(backend)), 'molar_hri', backend, 'nav_empty')
    assert not selectable('molar_hri', backend, 'nav', 'simulation', profile)
    assert not selectable('molar_hri', backend, 'display', 'hardware', profile)
    assert not selectable('other', backend, 'display', 'simulation', profile)
    assert not selectable('molar_hri', backend, 'display', 'simulation', {})


def test_feedback_requires_exact_source_bytes_and_independent_coupled_channels(model):
    state = feedback(model)
    channels = validate_feedback(state, model)
    assert channels[COUPLED]['coupled']
    assert channels[ROLLER]['measured'] == 12.  # angle is not a velocity target
    assert bounded_targets({LIFT: .68, STAGE: .14, ROLLER: -8.}, channels) == {
        LIFT: .68, STAGE: .14, ROLLER: -8.}
    model.write_text('<robot name="different"/>\n')
    with pytest.raises(ValueError, match='does not match'):
        validate_feedback(state, model)


@pytest.mark.parametrize('mutate', [
    lambda s: s.update(schema='other'),
    lambda s: s.update(plant_id=''),
    lambda s: s.update(model_sha256='b' * 64),
    lambda s: s.update(model_path='/other/model.urdf'),
    lambda s: s.update(urdf_sha256=''),
    lambda s: s.update(base_inhibited='false'),
    lambda s: s['channels'].pop(),
    lambda s: s['channels'][1].update(name=LIFT),
    lambda s: s['channels'][1].update(limits=[0., .28]),
    lambda s: s['channels'][1].update(mode='velocity'),
    lambda s: s['channels'][2].update(coupled=False),
    lambda s: s['channels'][0].update(measured=math.nan),
    lambda s: s['channels'][0].update(measured=None),
    lambda s: s['channels'][0].update(target=.69),
    lambda s: s['channels'][0].update(max_velocity=0.),
    lambda s: s['channels'][3].update(measured_velocity=math.inf),
    lambda s: s.update(coupling_error={COUPLED: math.nan}),
])
def test_malformed_or_mismatched_feedback_never_enables_controls(model, mutate):
    state = feedback(model)
    mutate(state)
    with pytest.raises((ValueError, KeyError)):
        validate_feedback(state, model)


@pytest.mark.parametrize('targets', [{COUPLED: .01}, {LIFT: .681}, {LIFT: -.001},
                                   {STAGE: .141}, {ROLLER: 8.01}, {ROLLER: math.nan},
                                   {LIFT: True}, {LIFT: '0.1'}, {}])
def test_invalid_or_coupled_targets_rejected(model, targets):
    with pytest.raises(ValueError):
        bounded_targets(targets, validate_feedback(feedback(model), model))


@pytest.fixture
def panel(model, monkeypatch):
    if not os.environ.get('DISPLAY'):
        pytest.skip('Actual Tk requires Xvfb or a display')
    app = tk.Tk()
    app.geometry('720x850')
    app.robot_var = tk.StringVar(value='molar_hri')
    app.simulator_var = tk.StringVar(value='mujoco')
    app.mode_var = tk.StringVar(value='display')
    app.launch_kind_var = tk.StringVar(value='simulation')
    app.map_var = tk.StringVar(value='nav_empty')
    app._robot_config = lambda: {'articulation': {'schema': SCHEMA}}
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
    widget = MolarControls(app.control_notebook, app)
    app.drive_page = ttk.Frame(app.control_notebook)
    app.control_notebook.add(app.drive_page, text='Drive')
    app.update()
    monkeypatch.setattr(controls, 'String', lambda **kw: SimpleNamespace(**kw), raising=False)
    monkeypatch.setattr(controls, 'Empty', lambda: SimpleNamespace(), raising=False)
    monkeypatch.setattr(controls, 'Trigger', SimpleNamespace(Request=lambda: SimpleNamespace()), raising=False)
    try:
        yield widget
    finally:
        widget.close()
        for job in app.tk.call('after', 'info'):
            app.tk.call('after', 'cancel', job)
        app.destroy()


def activate(panel, model, plant='plant-one', age=0.):
    if panel.app.process is None:
        panel.app.process = Mock(args=command())
        panel.app.process.poll.return_value = None
    panel.app._launch_running = True
    if panel.node is None:
        panel.node = Mock()
        panel.node.get_publishers_info_by_topic.return_value = [SimpleNamespace(endpoint_gid=b'molar-gid')]
        panel.node.count_subscribers.return_value = 1
        panel.pub, panel.heartbeat, panel.stop_client = Mock(), Mock(), Mock()
        panel.stop_client.service_is_ready.return_value = True
    panel.receive(SimpleNamespace(data=json.dumps(feedback(model, plant))),
                  SimpleNamespace(source_timestamp=time.time_ns() - int(age * 1e9), publisher_gid=b'molar-gid'))


def tick(panel):
    panel.after_cancel(panel.job)
    panel.tick()


def test_real_tk_selection_dragging_and_measured_reset_never_publish_targets(panel, model):
    assert panel.node is None
    panel.target_vars[LIFT].set(.5)
    panel.scales[STAGE].set(.1)
    panel.app.update()
    assert panel.node is None
    activate(panel, model)
    assert panel.ready()
    assert panel.target_vars[LIFT].get() == .2
    assert panel.target_vars[ROLLER].get() == .5  # actual velocity, not the 12 rad angle
    tick(panel)
    panel.target_vars[LIFT].set(.3)
    panel.scales[STAGE].set(.08)
    panel.app.update()
    panel.pub.publish.assert_not_called()
    assert panel.heartbeat.publish.call_count == 1
    assert 'deck height 0.620 m' in panel.feedback.get()
    assert 'Slide 1 0.030 m · slide 2 0.030 m' in panel.feedback.get()
    panel.use_measured_button.invoke()
    panel.pub.publish.assert_not_called()


def test_explicit_apply_jog_limits_and_plant_reset(panel, model):
    activate(panel, model)
    tick(panel)
    panel.target_vars[STAGE].set(.07)
    panel.apply_buttons[STAGE].invoke()
    sent = json.loads(panel.pub.publish.call_args.args[0].data)
    assert sent == {'plant_id': 'plant-one', 'targets': {STAGE: .07}}
    panel.jog_buttons[LIFT, 1].invoke()
    assert json.loads(panel.pub.publish.call_args.args[0].data)['targets'][LIFT] == pytest.approx(.21)
    count = panel.pub.publish.call_count
    panel.target_vars[ROLLER].set(9.)
    panel.apply_buttons[ROLLER].invoke()
    assert panel.pub.publish.call_count == count
    panel.target_vars[STAGE].set(.13)
    activate(panel, model, plant='plant-two')
    assert panel.target_vars[STAGE].get() == .03
    assert panel.pub.publish.call_count == count
    panel.apply(STAGE)
    assert json.loads(panel.pub.publish.call_args.args[0].data)['plant_id'] == 'plant-two'


@pytest.mark.parametrize('loss', ['selection', 'tab', 'workspace', 'process', 'stale', 'publisher', 'subscriber'])
def test_loss_revokes_state_and_cannot_publish_heartbeat_or_stale_commands(panel, model, loss):
    activate(panel, model)
    tick(panel)
    if loss == 'selection':
        panel.app.robot_var.set('probot_l')
    elif loss == 'tab':
        panel.app.control_notebook.select(panel.app.drive_page)
        panel.app.update()
    elif loss == 'workspace':
        panel.app.notebook.select(panel.app.other_tab)
        panel.app.update()
    elif loss == 'process':
        panel.app.process.poll.return_value = 1
    elif loss == 'stale':
        panel.received = time.monotonic() - .501
    elif loss == 'publisher':
        panel.node.get_publishers_info_by_topic.return_value.append(SimpleNamespace(endpoint_gid=b'other'))
    elif loss == 'subscriber':
        panel.node.count_subscribers.return_value = 2
    heartbeats = panel.heartbeat.publish.call_count
    tick(panel)
    assert not panel.ready() and panel.state is None
    assert not panel.apply(LIFT)
    assert panel.heartbeat.publish.call_count == heartbeats
    panel.pub.publish.assert_not_called()
    if loss in ('selection', 'tab', 'workspace', 'stale'):
        panel.stop_client.call_async.assert_called_once()


def test_delayed_dds_state_and_wrong_source_gid_are_rejected(panel, model):
    activate(panel, model, age=.501)
    assert not panel.ready()
    activate(panel, model)
    panel.receive(SimpleNamespace(data=json.dumps(feedback(model))),
                  SimpleNamespace(source_timestamp=time.time_ns(), publisher_gid=b'other'))
    assert panel.state is None
    assert not panel.apply(LIFT)
    panel.pub.publish.assert_not_called()


def test_close_stops_owned_plant_and_removes_callbacks(panel, model):
    activate(panel, model)
    panel.close()
    assert panel.closed and panel.state is None
    panel.stop_client.call_async.assert_called_once()
    assert panel.node is None
    panel.close()
