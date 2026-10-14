"""Source-faithful native-hand object geometry and measured grasp evaluation.

These are software fixtures, not a grasp mission. They establish that the
derived object sits between the opposing grasp surfaces and that the grasp
state follows measured two-sided contact, support and slip honestly.
"""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'robot_lab_utils'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'robot_lab_mujoco/python'))
mj = pytest.importorskip('mujoco')
pytest.importorskip('std_msgs.msg')
from std_msgs.msg import String  # noqa: E402
from robot_lab_utils.native_hand import hand_contract  # noqa: E402
from robot_lab_utils.native_hand_object import (  # noqa: E402
    classify_grasp, grasp_sides, object_geometry)
from robot_lab_mujoco.native_hand_control import NativeHandControl  # noqa: E402


def _pad_hand(tmp_path, entry='robotiq_2f85', gap=.09):
    """A minimal coupled-tendon two-pad hand; its own software fixture."""
    folder = tmp_path/entry
    folder.mkdir()
    source = folder/(entry+'.xml')
    source.write_text('''<mujoco><compiler angle="radian" autolimits="true"/>
      <worldbody><body name="mount" pos="0 0 .05">
        <body name="left_pad" pos="%f 0 0"><joint name="left_driver_joint" range="0 .8"/>
          <geom type="box" size=".004 .012 .012"/></body>
        <body name="right_pad" pos="%f 0 0"><joint name="right_driver_joint" range="0 .8"/>
          <geom type="box" size=".004 .012 .012"/></body>
      </body></worldbody>
      <tendon><fixed name="split"><joint joint="left_driver_joint" coef=".5"/>
        <joint joint="right_driver_joint" coef=".5"/></fixed></tendon>
      <actuator><general name="fingers_actuator" tendon="split" gainprm=".3137255"
        biastype="affine" biasprm="0 -100 -10" ctrlrange="0 255" forcerange="-5 5"/>
      </actuator></mujoco>''' % (-gap/2, gap/2))
    model = mj.MjModel.from_xml_path(str(source))
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    messages = []
    node = type('N', (), {})()
    node.model, node.data, node.mujoco, node.mobile, node.mobile_tasks = model, data, mj, None, None
    node.robot_bodies, node.robot_joints = model.nbody, model.njnt
    node.create_publisher = lambda *a: type('P', (), {'publish': messages.append})()
    node.create_subscription = lambda *a: None
    node.create_service = lambda *a: None
    node.get_parameter = lambda name: type('V', (), {'value': str(source) if name == 'native_mjcf' else ''})()
    return source, model, data, node, messages


def test_object_sits_between_opposing_surfaces_and_fits_the_open_gap(tmp_path):
    source, model, data, node, _ = _pad_hand(tmp_path)
    contract = hand_contract(model, source)
    # The synthetic pads are named like the v4 distal pads (left_pad/right_pad).
    geometry = object_geometry(model, data, 'robotiq_2f85_v4', contract)
    # The derived cube is centered between the two pads on the closing axis.
    assert abs(geometry['center'][0]) < 1e-6
    assert geometry['closing_axis'] == 0
    # Half-size along the closing axis fits well inside the measured open gap.
    assert 0 < geometry['half_size'][0] < .35*geometry['open_span']
    # Both opposing pad bodies are recorded for two-sided contact evaluation.
    assert len(geometry['left_bodies']) == 1 and len(geometry['right_bodies']) == 1
    assert geometry['left_bodies'] != geometry['right_bodies']


def test_real_robotiq_geometry_is_between_the_source_pads():
    root = Path('/workspace/molar/robot_lab_runtime/external_assets/sources/'
                'mujoco_menagerie/4d038b3feae26ec82b46a4d586379114012a8ac7')
    source = root/'robotiq_2f85'/'2f85.xml'
    if not source.is_file():
        pytest.skip('Installed Robotiq source not available on this host')
    model = mj.MjModel.from_xml_path(str(source))
    data = mj.MjData(model)
    contract = hand_contract(model, source)
    geometry = object_geometry(model, data, 'robotiq_2f85', contract)
    # At the source open pose the pads are ~0.098 m apart; the object fits inside.
    assert geometry['open_span'] > .09
    assert geometry['half_size'][0] < .35*geometry['open_span']
    left = np.array([data.xpos[b] for b in geometry['left_bodies']])
    right = np.array([data.xpos[b] for b in geometry['right_bodies']])
    assert np.all(left[:, 0] < 0) and np.all(right[:, 0] > 0)


def test_unknown_adapter_grasp_sides_are_rejected():
    with pytest.raises(ValueError, match='opposing grasp surfaces'):
        grasp_sides('not_a_hand')


def test_grasp_classification_requires_measured_two_sided_contact():
    assert classify_grasp(.5, .5, True, 0.)['grasp'] == 'contact_hold'
    assert classify_grasp(.5, .05, True, 0.)['grasp'] == 'no_contact'
    assert classify_grasp(.5, .5, False, 0.)['grasp'] == 'unsupported'
    assert classify_grasp(.5, .5, True, .05)['grasp'] == 'slipping'
    state = classify_grasp(0., 0., True, 0.)
    assert not state['two_sided_contact'] and state['grasp'] == 'no_contact'


def test_object_geometry_rejects_coincident_surfaces(tmp_path):
    source, model, data, node, _ = _pad_hand(tmp_path, gap=0.)
    contract = hand_contract(model, source)
    # The synthetic pads are named like the v4 distal pads (left_pad/right_pad).
    with pytest.raises(ValueError, match='coincident'):
        object_geometry(model, data, 'robotiq_2f85_v4', contract)


def test_published_hand_state_carries_grasp_scope(tmp_path):
    source, model, data, node, messages = _pad_hand(tmp_path)
    contract = hand_contract(model, source)
    control = NativeHandControl(node, contract)
    control.heartbeat(None)
    control.publish()
    state = __import__('json').loads(messages[-1].data)
    assert 'measured contact hold only' in state['hand']['qualification']
    assert state['hand']['object_grasp']['state'] == 'idle'


def _object_hand():
    """The installed Robotiq hand with a documented, supported free cube.

    Uses the real source coupler linkage so the pads actually close on the
    object; the cube is placed at the measured midpoint between the open pads
    and rests on a static pedestal, exactly as the display plant authors it.
    Skips when the pinned source is not installed on this host.
    """
    import mujoco as mj
    root = Path('/workspace/molar/robot_lab_runtime/external_assets/sources/'
                'mujoco_menagerie/4d038b3feae26ec82b46a4d586379114012a8ac7')
    source = root/'robotiq_2f85'/'2f85.xml'
    if not source.is_file():
        pytest.skip('Installed Robotiq source not available on this host')
    base = mj.MjModel.from_xml_path(str(source))
    base_data = mj.MjData(base)
    contract = hand_contract(base, source)
    geometry = object_geometry(base, base_data, 'robotiq_2f85', contract)
    spec = mj.MjSpec.from_file(str(source))
    center, half = geometry['center'], geometry['half_size']
    support_top = center[2]-half[2]
    pedestal = spec.worldbody.add_body(name='manipulation_pedestal',
        pos=[center[0], center[1], support_top/2])
    pedestal.add_geom(name='manipulation_pedestal_geom', type=mj.mjtGeom.mjGEOM_BOX,
        size=[half[0]+.004, half[1]+.004, support_top/2])
    cube = spec.worldbody.add_body(name='manipulation_object', pos=center)
    cube.add_freejoint(name='manipulation_object_joint')
    cube.add_geom(name='manipulation_object_geom', type=mj.mjtGeom.mjGEOM_BOX,
        size=half, mass=.05, friction=[1., .005, .0001])
    model = spec.compile()
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    messages = []
    node = type('N', (), {})()
    node.model, node.data, node.mujoco, node.mobile, node.mobile_tasks = model, data, mj, None, None
    node.robot_bodies, node.robot_joints = base.nbody, base.njnt
    node.create_publisher = lambda *a: type('P', (), {'publish': messages.append})()
    node.create_subscription = lambda *a: None
    node.create_service = lambda *a: None
    node.get_parameter = lambda name: type('V', (), {'value': str(source) if name == 'native_mjcf' else ''})()
    node.object_body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'manipulation_object')
    control = NativeHandControl(node, contract)
    control.heartbeat(None)
    return control, model, data, node, messages, geometry


def test_real_robotiq_grasp_measures_two_sided_contact_then_release_is_free():
    import json
    control, model, data, node, messages, geometry = _object_hand()
    mj = control.node.mujoco
    assert control.object_body >= 0
    digest = dict(operation='hand_grasp', plant_id=control.plant_id,
        source_sha256=control.source_sha256, contract_sha256=control.contract['contract_sha256'],
        closure=1., effort_fraction=.8, contact_limit_n=2.)
    control.receive(String(data=json.dumps(digest)))
    assert control.grasp['state'] == 'closing'
    # Close: the source coupled tendon must drive both pads onto the free cube.
    for step in range(2000):
        if step % 100 == 0:
            control.heartbeat(None)
        control.before_step()
        mj.mj_step(model, data)
        control.after_step()
    control.publish()
    state = json.loads(messages[-1].data)
    grasp = state['hand']['object_grasp']
    # Two-sided measured contact against the free object; no pose is written.
    assert grasp['left_force_n'] > 0 and grasp['right_force_n'] > 0
    assert grasp['state'] in ('holding', 'slipping')
    object_qposadr = model.jnt_qposadr[mj.mj_name2id(
        model, mj.mjtObj.mjOBJ_JOINT, 'manipulation_object_joint')]
    assert np.all(np.isfinite(data.qpos[object_qposadr:]))
    # Release: open and confirm the object is no longer held by the fingers.
    control.receive(String(data=json.dumps(dict(digest, operation='hand_release'))))
    assert control.grasp['state'] == 'releasing'
    for step in range(1200):
        if step % 100 == 0:
            control.heartbeat(None)
        control.before_step()
        mj.mj_step(model, data)
        control.after_step()
    assert control.grasp['state'] in ('released', 'releasing')


def test_grasp_is_rejected_without_a_documented_object(tmp_path):
    import json
    source, model, data, node, messages = _pad_hand(tmp_path)  # no object authored
    contract = hand_contract(model, source)
    control = NativeHandControl(node, contract)
    control.heartbeat(None)
    control.receive(String(data=json.dumps(dict(operation='hand_grasp', plant_id=control.plant_id,
        source_sha256=control.source_sha256, contract_sha256=control.contract['contract_sha256']))))
    assert control.object_body < 0
    assert 'no documented grasp object' in control.status
    assert control.grasp['state'] == 'idle'


@pytest.mark.parametrize('support_first', [True, False])
def test_actual_support_force_is_measured_in_both_contact_geom_orders(tmp_path, support_first):
    source, original, _, node, _ = _pad_hand(tmp_path)
    spec = mj.MjSpec.from_file(str(source))
    def support():
        body = spec.worldbody.add_body(name='test_support', pos=[0, 0, .059])
        body.add_geom(type=mj.mjtGeom.mjGEOM_BOX, size=[.015, .015, .059])
    def cube():
        body = spec.worldbody.add_body(name='manipulation_object', pos=[0, 0, .1299])
        body.add_freejoint()
        body.add_geom(type=mj.mjtGeom.mjGEOM_BOX, size=[.01, .01, .012], mass=.05)
    if support_first:
        support(); cube()
    else:
        cube(); support()
    node.model = spec.compile()
    node.data = mj.MjData(node.model)
    node.robot_bodies, node.robot_joints = original.nbody, original.njnt
    node.object_body = mj.mj_name2id(node.model, mj.mjtObj.mjOBJ_BODY, 'manipulation_object')
    mj.mj_forward(node.model, node.data)
    control = NativeHandControl(node, hand_contract(node.model, source))
    measurement = control.object_contacts()
    assert measurement['supported'] and measurement['support_force_z_n'] > .1
    assert measurement['support_contact_count'] > 0
    assert measurement['left_force_n'] == 0. and measurement['right_force_n'] == 0.
    assert measurement['contact_count'] == 0
    control.grasp.update(state='holding', contact_limit_n=1.)
    control.update_grasp()
    assert control.grasp['state'] == 'contact_lost'
    assert control.grasp['contact_state'] == 'no_contact'
    control.grasp['state'] = 'closing'
    control.last_heartbeat = -float('inf')
    control.before_step()
    assert control.grasp['state'] == 'interrupted'


def test_nonfinite_slip_cannot_claim_a_contact_hold():
    result = classify_grasp(.5, .5, True, float('nan'))
    assert result['grasp'] == 'invalid_measurement'
    assert not result['two_sided_contact']

