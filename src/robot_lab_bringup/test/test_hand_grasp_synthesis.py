"""DEX-EE tripod grasp synthesis on the installed source hand (R5.8).

Software checks on the real source model and controller, not a GUI mission:
the synthesized contacts are reachable and in force closure, the hand starts
clear of the object, and the unchanged guarded grasp, support-stage lift
and release run on them.
"""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'robot_lab_utils'))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'robot_lab_mujoco/python'))
mj = pytest.importorskip('mujoco')
pytest.importorskip('scipy')
pytest.importorskip('std_msgs.msg')
from std_msgs.msg import String  # noqa: E402
from robot_lab_utils.hand_grasp_synthesis import epsilon_quality, synthesize_tripod  # noqa: E402
from robot_lab_utils.native_hand import SYNTHESIZED_POSE, hand_contract, hand_pose_targets  # noqa: E402

SOURCE = Path('/workspace/molar/robot_lab_runtime/external_assets/sources/mujoco_menagerie/'
              '4d038b3feae26ec82b46a4d586379114012a8ac7/shadow_dexee/shadow_dexee.xml')


@pytest.fixture(scope='module')
def plan():
    if not SOURCE.is_file():
        pytest.skip('Installed DEX-EE source not available on this host')
    model = mj.MjModel.from_xml_path(str(SOURCE))
    return synthesize_tripod(model, mj.MjData(model), hand_contract(model, SOURCE))


def test_epsilon_separates_force_closure_from_one_sided_contact():
    centre = np.zeros(3)
    tripod = epsilon_quality([[0, .02, 0], [.012, -.02, 0], [-.012, -.02, 0]],
                             [[0, -1, 0], [0, 1, 0], [0, 1, 0]], centre)
    one_side = epsilon_quality([[.012, -.02, 0], [-.012, -.02, 0], [0, -.02, .01]],
                               [[0, 1, 0], [0, 1, 0], [0, 1, 0]], centre)
    assert tripod > 0 > one_side


def test_synthesized_tripod_is_reachable_and_in_force_closure(plan):
    assert plan['force_closure_epsilon'] > .05
    for finger, fit in plan['fit'].items():
        assert fit['error_mm'] < 1.5 and fit['pad_normal_deg'] < 35., (finger, fit)
    # Squeeze closes inward from the approach opening on every finger's curl.
    assert any(abs(plan['grasp'][name]-plan['approach'][name]) > .05 for name in plan['grasp'])


def test_contract_adds_the_tripod_and_opens_to_its_approach(plan):
    model = mj.MjModel.from_xml_path(str(SOURCE))
    contract = hand_contract(model, SOURCE, plan)
    assert contract['poses'][0] == SYNTHESIZED_POSE
    assert hand_pose_targets(contract, 'Open', 0.) == pytest.approx(plan['approach'])
    assert hand_pose_targets(contract, SYNTHESIZED_POSE, 1.) == pytest.approx(plan['grasp'])
    assert contract['contract_sha256'] != hand_contract(model, SOURCE)['contract_sha256']
    with pytest.raises(ValueError):
        hand_contract(model, SOURCE, dict(plan, grasp={**plan['grasp'], 'F0/J1_actuator': 9.}))


def _plant(plan):
    from robot_lab_mujoco.native_asset_display import add_hand_object_fixture, author_grasp_home
    from robot_lab_mujoco.native_hand_control import NativeHandControl
    base = mj.MjModel.from_xml_path(str(SOURCE))
    spec = mj.MjSpec.from_file(str(SOURCE))
    add_hand_object_fixture(spec, 'shadow_dexee', plan['centre'], plan['half_size'], plan['quat_wxyz'])
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


def test_home_starts_clear_of_the_object(plan):
    control, model, data = _plant(plan)
    body = control.object_body
    touching = [k for k in range(data.ncon)
                if body in (model.geom_bodyid[data.contact[k].geom1], model.geom_bodyid[data.contact[k].geom2])
                and 'pedestal' not in (mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, data.contact[k].geom1) or '')
                + (mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, data.contact[k].geom2) or '')]
    assert not touching


def test_synthesized_grasp_lifts_holds_and_releases_the_box(plan):
    control, model, data = _plant(plan)
    ids = dict(plant_id=control.plant_id, source_sha256=control.source_sha256,
               contract_sha256=control.contract['contract_sha256'])
    _run(control, model, data, 250)
    control.receive(String(data=json.dumps(dict(ids, operation='hand_grasp', closure=1.,
                                                 effort_fraction=.8, contact_limit_n=2.))))
    _run(control, model, data, 2500)
    assert control.motion['pose'] == SYNTHESIZED_POSE
    assert control.grasp['state'] == 'holding', control.grasp
    start = float(data.xpos[control.object_body, 2])
    control.receive(String(data=json.dumps(dict(ids, operation='hand_lift_object', height_m=.05))))
    _run(control, model, data, 5500)    # 5 cm at 2 cm/s, then a hold
    assert control.lift_state['state'] == 'holding', control.lift_state
    assert not control.grasp['supported'] and control.grasp['state'] == 'holding', control.grasp
    assert abs(float(data.xpos[control.object_body, 2])-start) < .005
    control.receive(String(data=json.dumps(dict(ids, operation='hand_lower_object'))))
    for _ in range(40):     # 2.5 s down, then 1 s settled on the support
        _run(control, model, data, 100)
        if control.lift_state['state'] != 'lowering':
            break
    assert control.lift_state['state'] == 'supported', control.lift_state
    control.receive(String(data=json.dumps(dict(ids, operation='hand_release'))))
    _run(control, model, data, 2500)
    assert control.grasp['left_force_n'] < .2 and control.grasp['right_force_n'] < .2
