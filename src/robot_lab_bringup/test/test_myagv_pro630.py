"""Pro630 arm composition onto the myAGV Pro: declared estimates, not missions."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from robot_lab_utils import myagv_pro630
from robot_lab_utils.urdf_articulation import URDFArticulation, base_motion_limits, DEFAULT_BASE_MOTION_LIMITS

REVISION = 'd42ff61a78122c79246623391540d75738b03b23'


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


@pytest.mark.parametrize('limits', [[.2, .2], [.1, .5], [0, .1], [-.1, .1]])
def test_base_motion_limits_are_bounded(limits):
    with pytest.raises(ValueError):
        base_motion_limits({'base_motion_limits': limits})
    assert base_motion_limits({}) == DEFAULT_BASE_MOTION_LIMITS
