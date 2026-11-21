"""Pro630 arm composition onto the myAGV Pro: declared estimates, not missions."""
import copy
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from robot_lab_utils import myagv_pro630
from robot_lab_utils.urdf_articulation import (
    URDFArticulation, add_mujoco_articulation_actuators, base_motion_limits, DEFAULT_BASE_MOTION_LIMITS)

REVISION = 'd42ff61a78122c79246623391540d75738b03b23'
F100_REVISION = 'b8176c264e583b073e1c4293c2edb1241b20e1aa'


@pytest.fixture(scope='module')
def composed(tmp_path_factory):
    root = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))
    arm_root = root / 'external_assets/sources/elephant_mycobot_ros2' / REVISION
    if not (arm_root / myagv_pro630.ARM_URDF).is_file():
        pytest.skip('Official mycobot_ros2 source not provisioned on this host')
    pytest.importorskip('trimesh')
    pytest.importorskip('pybullet')
    base = ET.fromstring('<robot name="base"><link name="base_link"/></robot>')
    articulation, receipt = myagv_pro630.compose(base, arm_root, tmp_path_factory.mktemp('meshes'))
    return base, articulation, receipt


def test_arm_is_prefixed_and_fixed_to_the_deck(composed):
    base, articulation, _ = composed
    names = [link.get('name') for link in base.findall('link')]
    assert names[0] == 'base_link' and all(n.startswith('pro630_') for n in names[1:])
    mount = base.find("joint[@name='pro630_mount_joint']")
    assert mount.get('type') == 'fixed' and mount.find('parent').get('link') == 'base_link'
    assert mount.find('origin').get('xyz') == '0 0 0.22'
    assert len(articulation['joints']) == 6 and articulation['panel'] == 'arm'


def test_declared_masses_limits_and_effort_floor(composed):
    base, articulation, receipt = composed
    masses = [float(link.find('inertial/mass').get('value')) for link in base.findall('link')[1:]]
    assert sum(masses) == pytest.approx(myagv_pro630.ARM_MASS_KG, rel=1e-6) and min(masses) > 0
    for name, spec in articulation['joints'].items():
        joint = base.find("joint[@name='%s']" % name)
        assert float(joint.find('limit').get('velocity')) == myagv_pro630.MAX_JOINT_SPEED
        assert float(joint.find('limit').get('effort')) == spec['effort'] >= 5.
        gravity = receipt['estimates']['gravity_torque_nm'][name]
        assert spec['effort'] >= myagv_pro630.EFFORT_MARGIN * gravity    # carries the rated payload
    assert 'arm masses and inertias (vendor publishes no weight)' in receipt['declared_estimates']


def test_contract_accepts_the_composed_arm(composed, tmp_path):
    base, articulation, _ = composed
    path = tmp_path / 'composed.urdf'
    path.write_text(ET.tostring(base, encoding='unicode'))
    contract = URDFArticulation(articulation, path.read_text(), str(path))
    assert set(contract.joints) == set(articulation['joints'])
    assert base_motion_limits(articulation) == (.10, .20)


def test_vendor_f100_preserves_the_mount_and_normalizes_follower_coordinates(composed, tmp_path):
    root = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))
    source = root/'external_assets/sources/elephant_pro630_f100'/F100_REVISION
    if not (source/myagv_pro630.F100_URDF).is_file():
        pytest.skip('Vendor Pro630 F100 source not provisioned on this host')
    bare_model, bare_config, _ = composed
    model, config = copy.deepcopy(bare_model), copy.deepcopy(bare_config)
    receipt = myagv_pro630.attach_f100(model, config, source, tmp_path/'meshes')
    assert len(model.findall('link')) == len(bare_model.findall('link')) + 10
    assert len(config['couplings']) == 5 and len(receipt['reversed_follower_axes']) == 3
    for joint in bare_model.findall('joint'):
        assert ET.tostring(model.find(f"joint[@name='{joint.get('name')}']")) == ET.tostring(joint)
    mount = model.find("joint[@name='pro630_joint6_to_gripper_connect']")
    assert mount.find('parent').get('link') == 'pro630_link6'
    grasp = receipt['grasp_reference']
    reference = model.find(f"joint[@name='{grasp['frame']}_joint']")
    assert reference.find('parent').get('link') == 'pro630_gripper_base'
    assert grasp['nominal']['gap_m'] == pytest.approx(.04, abs=.005)
    gaps = [sample['gap_m'] for sample in grasp['aperture_samples']]
    assert gaps == sorted(set(gaps)) and gaps[-1] == pytest.approx(.109, abs=.001)
    for side in ('left', 'right'):
        jaw = f'gripper_{side}1'
        part = model.find(f"link[@name='pro630_{jaw}']")
        assert [collision.get('name') for collision in part.findall('collision')] == [
            f'pro630_{jaw}_{label}_contact' for label in ('mount', 'finger', 'pad')]
        assert receipt['jaw_collision'][jaw]['pad']['sha256']
    leader, = config['grip_joints']
    assert leader in config['joints'] and config['joints'][leader]['effort'] == .8
    for follower, spec in config['couplings'].items():
        urdf_joint = model.find(f"joint[@name='{follower}']")
        assert spec == dict(joint=leader, multiplier=1., offset=0., mechanical=True, passive=True)
        assert urdf_joint.find('mimic').get('multiplier') == '1'
        assert float(urdf_joint.find('axis').get('xyz').split()[-1]) == (-1. if follower in receipt['reversed_follower_axes'] else 1.)
    output = tmp_path/'pro630_f100.urdf'
    output.write_text(ET.tostring(model, encoding='unicode'))
    contract = URDFArticulation(config, output.read_text(), output)
    assert len(contract.masters) == 7 and len(contract.joints) == 12
    fixture = '<mujoco><worldbody><body>' + ''.join(
        f'<joint name="{name}"/>' for name in contract.joints) + '</body></worldbody></mujoco>'
    compiled = ET.fromstring(add_mujoco_articulation_actuators(fixture, contract))
    assert {item.get('joint') for item in compiled.findall('actuator/general')} == set(contract.masters)
    expected = {leader: .003, **dict.fromkeys(contract.couplings, .00015)}
    assert {item.get('name'): float(item.get('armature'))
            for item in compiled.findall('worldbody//joint[@armature]')} == expected


@pytest.mark.parametrize('limits', [[.2, .2], [.1, .5], [0, .1], [-.1, .1]])
def test_base_motion_limits_are_bounded(limits):
    with pytest.raises(ValueError):
        base_motion_limits({'base_motion_limits': limits})
    assert base_motion_limits({}) == DEFAULT_BASE_MOTION_LIMITS
