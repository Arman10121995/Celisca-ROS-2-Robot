"""Pinned complete TIAGo source/adapter checks, separate from robot missions."""
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from robot_lab_utils.tiago_source import contracts, portable_description, POSITION_JOINTS
from robot_lab_utils.urdf_articulation import URDFArticulation


@pytest.fixture
def original():
    store = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))/'external_assets'
    config = store/'installed/robots.yaml'
    profile = yaml.safe_load(config.read_text())['robots'].get('asset_pal_tiago') if config.exists() else None
    if not profile:
        pytest.skip('Complete official TIAGo source not provisioned')
    path = Path(profile['xacro']).parent/'source-expanded.urdf'
    return ET.parse(path).getroot()


def test_preserves_all_source_mechanics_and_independent_fingers(original, tmp_path):
    protected = [(e.tag, e.get('name'), ET.tostring(e)) for e in original
                 if e.tag in ('link', 'joint')]
    drive, config = contracts(original)
    sensors = portable_description(original)
    assert [(e.tag, e.get('name'), ET.tostring(e)) for e in original
            if e.tag in ('link', 'joint')] == protected
    assert drive['mujoco_expected_mass_kg'] == pytest.approx(84.66799184112)
    assert not config['couplings'] and set(config['joints']) == set(POSITION_JOINTS)
    assert config['joints']['arm_5_joint']['effort'] == 3.
    assert config['joints']['torso_lift_joint']['max_velocity'] <= .07
    assert sensors['camera_link_name'] == 'head_front_camera_color_frame'
    assert sensors['imu_link_name'] == 'base_footprint'
    assert not original.findall('.//plugin') and not original.findall('ros2_control')
    assert float(original.findtext("gazebo[@reference='caster_front_left_2_link']/mu1")) == .01
    model = tmp_path/'model.urdf'
    model.write_text(ET.tostring(original, encoding='unicode'))
    articulation = URDFArticulation(config, model.read_text(), model)
    assert len(articulation.masters) == 12


def test_isaac_tiago_pairs_jaw_targets_and_reports_measured_skew(original, tmp_path):
    _, config = contracts(original)
    overlay_path = Path(__file__).resolve().parents[2]/'robot_lab_robots/config/installed_overlays.yaml'
    overlay = yaml.safe_load(overlay_path.read_text())['robots']['asset_pal_tiago']['articulation']
    config.update(overlay)
    portable_description(original)
    model = tmp_path/'tiago.urdf'
    model.write_text(ET.tostring(original, encoding='unicode'))
    arm = URDFArticulation(config, model.read_text(), model, symmetric_grip=config['isaac_symmetric_grip'])
    left, right = arm.symmetric_grip
    measured = config['home'].copy()
    still = dict.fromkeys(measured, 0.)
    arm.step(measured, still, .02, now=1.)
    arm.heartbeat(now=1.)
    assert arm.command(dict(plant_id=arm.plant_id, targets={right: .02}), now=1.)
    targets = arm.step(measured, still, .02, now=1.02)['position']
    assert targets[left] == targets[right]
    assert arm.targets[left] == arm.targets[right] == .02
    measured[right] -= .02
    arm.step(measured, still, .02, now=1.04)
    state = arm.state()
    assert state['coupling_error'][right] == pytest.approx(-.02)
    assert state['coupling_inhibited'] and state['base_inhibited']
    generation = arm.plant_id
    assert not arm.command(dict(plant_id=generation, targets={left: .02, right: .01}), now=1.04)
    assert arm.holding and arm.plant_id != generation


@pytest.mark.parametrize('change', ['wheel_geometry', 'command_interface', 'mimic'])
def test_rejects_changed_source_contract(original, change):
    if change == 'wheel_geometry':
        original.find("joint[@name='wheel_left_joint']/origin").set('xyz', '0 .4 0')
    elif change == 'command_interface':
        original.find("ros2_control/joint[@name='arm_1_joint']/command_interface").set('name', 'effort')
    else:
        ET.SubElement(original.find("joint[@name='gripper_right_finger_joint']"),
                      'mimic', joint='gripper_left_finger_joint')
    with pytest.raises(ValueError):
        contracts(original)
