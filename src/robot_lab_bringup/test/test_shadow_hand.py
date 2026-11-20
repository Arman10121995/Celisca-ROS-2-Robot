"""Shadow Hand (E3M5 right): source contract, synthesized tripod and object cycle (R5.8, 2026-11-20).

The twenty authored position servos are bound without a new transmission;
each finger's J0 servo drives J1+J2 through its fixed tendon. The tripod
(thumb against first+middle) comes from the shared synthesis with the
underactuated balance; the object cycle runs the real hand controller and
support stage in-process on the executed MuJoCo plant.
"""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/robot_lab_utils'))
sys.path.insert(0, str(ROOT/'src/robot_lab_mujoco/python'))
mj = pytest.importorskip('mujoco')
pytest.importorskip('scipy')
pytest.importorskip('std_msgs.msg')
from std_msgs.msg import String  # noqa: E402
from robot_lab_utils.hand_grasp_synthesis import synthesize_tripod  # noqa: E402
from robot_lab_utils.native_hand import SYNTHESIZED_POSE, hand_contract, hand_pose_targets  # noqa: E402

SOURCE = Path('/workspace/molar/robot_lab_runtime/external_assets/sources/mujoco_menagerie/'
              '4d038b3feae26ec82b46a4d586379114012a8ac7/shadow_hand/right_hand.xml')


@pytest.fixture(scope='module')
def plan():
    if not SOURCE.is_file():
        pytest.skip('Installed Shadow Hand source not available on this host')
    model = mj.MjModel.from_xml_path(str(SOURCE))
    return synthesize_tripod(model, mj.MjData(model), hand_contract(model, SOURCE))


def test_contract_binds_the_twenty_servos_and_coupled_tendons():
    if not SOURCE.is_file():
        pytest.skip('Installed Shadow Hand source not available on this host')
    model = mj.MjModel.from_xml_path(str(SOURCE))
    contract = hand_contract(model, SOURCE)
    assert contract['adapter'] == 'shadow_hand' and len(contract['channels']) == 20
    assert [group['name'] for group in contract['groups']] == ['Wrist', 'Thumb', 'First', 'Middle', 'Ring', 'Little']
    tendon = next(c for c in contract['channels'] if c['name'] == 'rh_A_FFJ0')
    assert tendon['joint'] == '' and tendon['limits'] == [0., 3.1415] and tendon['source_force_limits'] == [-1., 1.]
    curl = hand_pose_targets(contract, 'Curl', 1.)
    assert curl['rh_A_FFJ0'] == pytest.approx(1.6) and curl['rh_A_WRJ1'] == 0.
    assert hand_pose_targets(contract, 'Open', 0.)['rh_A_THJ4'] == 0.


def test_synthesized_tripod_balances_the_coupled_fingers(plan):
    from robot_lab_utils.hand_grasp_synthesis import EQUILIBRIUM_TAN, HANDS, _Kinematics
    assert plan['force_closure_epsilon'] > .05
    assert set(plan['fit']) == {'th', 'ff', 'mf'}
    assert all(fit['error_mm'] < 1.5 for fit in plan['fit'].values())
    # J1/J2 home positions come from the joint-space approach; the tendon
    # channel's approach is their sum.
    q = plan['approach_qpos']
    assert plan['approach']['rh_A_FFJ0'] == pytest.approx(q['rh_FFJ1']+q['rh_FFJ2'], abs=1e-6)
    assert plan['centre'][2]-plan['half_size'][2] > .03
    assert plan['quat_wxyz'][1:3] == [0., 0.]
    assert EQUILIBRIUM_TAN <= .6 and HANDS['shadow_hand']['equilibrium']


def _plant(plan):
    from robot_lab_mujoco.native_asset_display import add_hand_object_fixture, author_grasp_home
    from robot_lab_mujoco.native_hand_control import NativeHandControl
    base = mj.MjModel.from_xml_path(str(SOURCE))
    spec = mj.MjSpec.from_file(str(SOURCE))
    add_hand_object_fixture(spec, 'shadow_hand', plan['centre'], plan['half_size'], plan['quat_wxyz'])
    author_grasp_home(spec, plan)
    model = spec.compile()
    data = mj.MjData(model)
    mj.mj_resetDataKeyframe(model, data, 0)
    mj.mj_forward(model, data)
    node = type('N', (), {})()
    node.model, node.data, node.mujoco, node.mobile, node.mobile_tasks = model, data, mj, None, None
    node.robot_bodies, node.robot_joints = base.nbody, base.njnt
    node.create_publisher = lambda *a: type('P', (), {'publish': lambda *_: None})()
    node.create_subscription = lambda *a: None
    node.create_service = lambda *a: None
    node.get_parameter = lambda name: type('V', (), {'value': str(SOURCE) if name == 'native_mjcf' else ''})()
    node.object_body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'manipulation_object')
    control = NativeHandControl(node, hand_contract(model, SOURCE, plan))
    control.heartbeat(None)
    return control, model, data


def _run(control, model, data, steps):
    for step in range(steps):
        if step % 100 == 0:
            control.heartbeat(None)
        control.before_step()
        mj.mj_step(model, data)
        control.after_step()


def test_synthesized_grasp_lifts_holds_and_releases_the_box(plan):
    control, model, data = _plant(plan)
    assert control.contract['poses'][0] == SYNTHESIZED_POSE
    ids = dict(plant_id=control.plant_id, source_sha256=control.source_sha256,
               contract_sha256=control.contract['contract_sha256'])
    _run(control, model, data, 250)
    control.receive(String(data=json.dumps(dict(ids, operation='hand_grasp', closure=1.,
                                                 effort_fraction=.8, contact_limit_n=2.))))
    _run(control, model, data, 2500)
    assert control.grasp['state'] == 'holding', control.grasp
    start = float(data.xpos[control.object_body, 2])
    control.receive(String(data=json.dumps(dict(ids, operation='hand_lift_object', height_m=.05))))
    _run(control, model, data, 5500)    # 5 cm at 2 cm/s, then a hold
    assert control.lift_state['state'] == 'holding', control.lift_state
    assert not control.grasp['supported'] and control.grasp['state'] == 'holding', control.grasp
    assert abs(float(data.xpos[control.object_body, 2])-start) < .005
    control.receive(String(data=json.dumps(dict(ids, operation='hand_lower_object'))))
    for _ in range(40):
        _run(control, model, data, 100)
        if control.lift_state['state'] != 'lowering':
            break
    assert control.lift_state['state'] == 'supported', control.lift_state
    control.receive(String(data=json.dumps(dict(ids, operation='hand_release'))))
    _run(control, model, data, 2500)
    assert control.grasp['left_force_n'] < .2 and control.grasp['right_force_n'] < .2
    assert np.isfinite(data.qpos).all()


def test_carried_weight_matches_the_fixture_box():
    import inspect
    from robot_lab_utils.hand_grasp_synthesis import OBJECT_MASS_KG
    from robot_lab_mujoco.native_asset_display import add_hand_object_fixture
    assert 'mass=%g' % OBJECT_MASS_KG in inspect.getsource(add_hand_object_fixture).replace('mass=.05', 'mass=0.05')
