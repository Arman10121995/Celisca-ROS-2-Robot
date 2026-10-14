"""Native PID semantics, physical contact guards and source counterexamples.

The small model is an explicit software fixture, not a DEX-EE grasp mission.
"""
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'robot_lab_utils'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'robot_lab_mujoco/python'))
mj = pytest.importorskip('mujoco')
pytest.importorskip('std_msgs.msg')
from std_msgs.msg import String  # noqa: E402
from robot_lab_utils.native_hand import hand_contract, hand_pose_targets  # noqa: E402
from robot_lab_mujoco.native_hand_control import NativeHandControl  # noqa: E402


@pytest.fixture
def hand(tmp_path):
    folder = tmp_path/'shadow_dexee'
    folder.mkdir()
    source = folder/'shadow_dexee.xml'
    pid, bodies, actuators = [], [], []
    for joint in range(4):
        pid.append('<instance name="pid%d">%s</instance>' % (joint,
            ''.join('<config key="%s" value="%s"/>' % pair for pair in (
                ('kp', 2.), ('ki', 3.), ('kd', .02), ('imax', .1), ('slewmax', 3.)))))
    for finger in range(3):
        for joint, limits in enumerate(('-0.8727 0.8727', '-1.3963 0.7854', '0 1.3963', '-0.5236 1.4835')):
            name = 'F%d/J%d' % (finger, joint)
            bodies.append('<body name="%s" pos="%s 0 .2"><joint name="%s" axis="0 0 1" range="%s"/>'
                '<geom type="sphere" pos=".03 0 0" size=".005" mass=".05"/></body>' % (
                    name, .1*(4*finger+joint), name, limits))
            actuators.append('<plugin name="%s_actuator" plugin="mujoco.pid" instance="pid%d" '
                'joint="%s" ctrlrange="%s" forcerange="-.9 .53" dyntype="none" actdim="2"/>' % (
                    name, joint, name, limits))
    source.write_text('<mujoco><compiler angle="radian" autolimits="true"/>'
        '<option gravity="0 0 0" timestep=".002"/><extension><plugin plugin="mujoco.pid">'
        + ''.join(pid)+'</plugin></extension><worldbody>'+''.join(bodies)
        +'</worldbody><actuator>'+''.join(actuators)+'</actuator></mujoco>')
    model = mj.MjModel.from_xml_path(str(source))
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    messages = []
    node = SimpleNamespace(model=model, data=data, mujoco=mj, mobile=None, mobile_tasks=None,
        robot_bodies=model.nbody, robot_joints=model.njnt,
        create_publisher=lambda *a: SimpleNamespace(publish=messages.append),
        create_subscription=lambda *a: None, create_service=lambda *a: None,
        get_parameter=lambda name: SimpleNamespace(value=str(source) if name == 'native_mjcf' else ''))
    contract = hand_contract(model, source)
    controller = NativeHandControl(node, contract)
    controller.heartbeat(None)
    return controller, source, messages


def send(controller, **overrides):
    payload = dict(operation='hand_pose', plant_id=controller.plant_id,
        source_sha256=controller.source_sha256, contract_sha256=controller.contract['contract_sha256'],
        pose='Curl', closure=.3, spread=0., finger='All', guarded=False)
    payload.update(overrides)
    controller.receive(String(data=json.dumps(payload)))


def test_authored_pid_runs_real_steps_without_motor_replacement_or_pose_writes(hand):
    control, _, _ = hand
    model, data = control.model, control.data
    gains = model.actuator_gainprm.copy()
    bounds = model.actuator_forcerange.copy()
    initial = data.qpos.copy()
    send(control)
    np.testing.assert_array_equal(data.qpos, initial)
    efforts = []
    for step in range(500):
        if step % 100 == 0:
            control.heartbeat(None)
        control.before_step()
        mj.mj_step(model, data)
        control.after_step()
        efforts.append(data.actuator_force.copy())
    assert np.max(np.abs(data.qpos-initial)) > .05
    np.testing.assert_allclose(control.measured_controls(), data.actuator_length)
    np.testing.assert_allclose(control.measured_rates(), data.actuator_velocity)
    assert np.all(np.asarray(efforts) >= bounds[:, 0]-1e-9)
    assert np.all(np.asarray(efforts) <= bounds[:, 1]+1e-9)
    np.testing.assert_array_equal(model.actuator_gainprm, gains)
    np.testing.assert_array_equal(model.actuator_forcerange, bounds)


@pytest.mark.parametrize('property', ['gear', 'force', 'plugin'])
def test_modified_source_motor_contract_is_rejected(hand, property):
    control, source, _ = hand
    if property == 'gear':
        control.model.actuator_gear[0, 0] = 2.
    elif property == 'force':
        control.model.actuator_forcerange[0, 1] = 1.
    else:
        control.model.plugin_attr[0] = ord('3')
    with pytest.raises(ValueError, match='contract'):
        hand_contract(control.model, source)


def test_stop_and_lost_heartbeat_reset_only_owned_pid_memory(hand):
    control, _, _ = hand
    control.data.qpos[:] = .12
    control.data.qvel[:] = .08
    mj.mj_forward(control.model, control.data)
    control.data.act[:] = .9
    qpos, qvel = control.data.qpos.copy(), control.data.qvel.copy()
    send(control)
    control.stop(None, SimpleNamespace())
    np.testing.assert_array_equal(control.data.qpos, qpos)
    np.testing.assert_array_equal(control.data.qvel, qvel)
    np.testing.assert_allclose(control.target, .12)
    np.testing.assert_allclose(control.data.act[::2], 0.)
    np.testing.assert_allclose(control.data.act[1::2], .12)
    send(control)
    control.last_heartbeat = time.monotonic()-1.
    control.before_step()
    assert control.guard is None
    assert 'heartbeat lost' in control.status
    np.testing.assert_allclose(control.target, .12)


def test_finger_target_selection_is_bounded_and_leaves_other_groups_unchanged(hand):
    control, _, _ = hand
    send(control, finger='F1', closure=1., spread=-1.)
    for slot, channel in enumerate(control.channels):
        if not channel['name'].startswith('F1/'):
            assert control.target[slot] == 0.
        assert channel['limits'][0] <= control.target[slot] <= channel['limits'][1]
    targets = hand_pose_targets(control.contract, 'Pinch F0 + F1', 1., .5)
    assert targets['F2/J2_actuator'] == 0.
    assert targets['F0/J2_actuator'] == .9


def test_foreign_and_bad_pose_cannot_start_hand_motion(hand):
    control, _, _ = hand
    before = control.target.copy()
    send(control, plant_id='foreign')
    np.testing.assert_array_equal(control.target, before)
    send(control, contract_sha256='foreign')
    assert 'Rejected hand pose' in control.status and control.guard is None
    send(control, closure=float('nan'), guarded=True)
    assert 'Rejected hand pose' in control.status and control.guard is None


def test_actual_world_contact_stops_finger_advancement_without_grasp_flag(hand):
    control, source, _ = hand
    spec = mj.MjSpec.from_file(str(source))
    spec.worldbody.add_geom(name='fixture_support', type=mj.mjtGeom.mjGEOM_PLANE,
        size=[2, 2, .1], pos=[0, 0, .1955], solref=[.001, 1.])
    model = spec.compile()
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    control.node.model, control.node.data = model, data
    guard = NativeHandControl(control.node, hand_contract(model, source))
    guard.heartbeat(None)
    send(guard, guarded=True, contact_limit_n=.2, effort_fraction=1.)
    assert data.ncon > 0
    assert guard.contacts()['F0']['normal_force_n'] > .2
    guard.before_step()
    assert guard.guard and guard.guard['blocked']['F0'].startswith('Measured external')
    assert guard.motion['state'] == 'contact_or_effort_limited'
    assert 'grasp' not in guard.motion['state']
    for slot, channel in enumerate(guard.channels):
        assert guard.target[slot] == pytest.approx(guard.measured_controls()[slot])
    send(guard, pose='Open')
    assert guard.guard is None


def test_state_publishes_actual_contacts_efforts_and_source_contract(hand):
    control, _, messages = hand
    send(control)
    for _ in range(50):
        control.before_step()
        mj.mj_step(control.model, control.data)
        control.after_step()
    qpos, qvel, act = control.data.qpos.copy(), control.data.qvel.copy(), control.data.act.copy()
    stamp = control.data.time
    control.publish()
    state = json.loads(messages[-1].data)
    assert state['hand']['contract_sha256'] == control.contract['contract_sha256']
    assert list(state['hand']['contacts']) == ['F0', 'F1', 'F2']
    assert all(c['actuator_force'] == control.data.actuator_force[c['index']] for c in state['channels'])
    assert 'pending' in state['hand']['qualification']
    np.testing.assert_array_equal(control.data.qpos, qpos)
    np.testing.assert_array_equal(control.data.qvel, qvel)
    np.testing.assert_array_equal(control.data.act, act)
    assert state['time'] == stamp
    np.testing.assert_allclose([c['measured'] for c in state['channels']], control.data.qpos, atol=1e-12)


def test_spawn_roundoff_preserves_closed_chain_but_changed_coupling_is_rejected(tmp_path):
    folder = tmp_path/'robotiq_2f85'
    folder.mkdir()
    source = folder/'2f85.xml'
    source.write_text('''<mujoco><compiler angle="radian" autolimits="true"/>
      <worldbody><body name="mount" pos="0 0 .007">
        <body name="right_driver" pos="0 .0306011 .054904"><joint name="right_driver_joint" range="0 .8"/>
          <geom type="sphere" size=".01"/><body name="right_follower" pos="0 .0060989 .047596">
            <joint name="follower" range="-1 1"/><geom type="sphere" size=".01"/></body></body>
        <body name="left_driver" pos="0 -.0306011 .054904"><joint name="left_driver_joint" range="0 .8"/>
          <geom type="sphere" size=".01"/></body>
      </body></worldbody><tendon><fixed name="split"><joint joint="right_driver_joint" coef=".5"/>
        <joint joint="left_driver_joint" coef=".5"/></fixed></tendon>
      <equality><connect body1="right_follower" body2="right_driver" anchor="0 0 0"/>
        <joint joint1="right_driver_joint" joint2="left_driver_joint" polycoef="0 1 0 0 0"/></equality>
      <actuator><general name="fingers_actuator" tendon="split" gainprm=".3137255" biastype="affine"
        biasprm="0 -100 -10" ctrlrange="0 255" forcerange="-5 5"/></actuator></mujoco>''')
    spec = mj.MjSpec.from_file(str(source))
    for body in spec.worldbody.bodies:
        body.pos = body.pos+[-7, -7, .1]
    model = spec.compile()
    contract = hand_contract(model, source)
    assert contract['channels'][0]['source_force_units'] == 'N m'
    assert hand_pose_targets(contract, 'Close', .5)['fingers_actuator'] == 127.5
    model.eq_data[1, 1] = .99
    with pytest.raises(ValueError, match='equality'):
        hand_contract(model, source)
    model.eq_data[1, 1] = 1.
    model.wrap_prm[0] = .6
    with pytest.raises(ValueError, match='coupling'):
        hand_contract(model, source)


def test_real_soft_limit_excursion_is_reported_but_hold_target_stays_bounded(hand):
    control, _, messages = hand
    control.data.qpos[2] = -.003
    mj.mj_forward(control.model, control.data)
    control.publish()
    state = json.loads(messages[-1].data)
    assert state['channels'][2]['measured'] == pytest.approx(-.003)
    qpos = control.data.qpos.copy()
    control.stop(None, SimpleNamespace())
    assert control.target[2] == 0. and control.commanded[2] == 0.
    np.testing.assert_array_equal(control.data.qpos, qpos)
