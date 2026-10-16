"""MOLAR authored mechanics and launcher contract, not mission qualification."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('variant', ['molar_hri', 'probot_l'])
def test_source_mechanics_and_interfaces(variant):
    folder = ROOT / 'src/robot_lab_robots' / variant
    source = folder / 'source/robot.urdf'
    provenance = json.loads((folder / 'provenance.json').read_text())
    assert hashlib.sha256(source.read_bytes()).hexdigest() == provenance['source_sha256']
    model = ET.parse(folder / ('urdf/' + variant + '.urdf.xacro')).getroot()
    joints = {joint.get('name'): joint for joint in model.findall('joint')}
    assert len(joints) == 14 and len(model.findall('link')) == 15
    for name, upper, velocity in [('conveyor_lift_joint', .68, .035),
                                  ('telescope_stage1_joint', .14, .1),
                                  ('telescope_stage2_joint', .14, .1)]:
        joint = joints[name]
        assert joint.get('type') == 'prismatic'
        assert float(joint.find('limit').get('upper')) == upper
        assert float(joint.find('limit').get('velocity')) == velocity
    original_mimic = ET.parse(source).find("joint[@name='telescope_stage2_joint']/mimic")
    assert original_mimic.get('joint') == 'telescope_stage1_joint'
    # A single owned controller implements the explicit physical coupling.
    contract = json.loads((folder / 'config/articulation.json').read_text())
    assert contract['couplings']['telescope_stage2_joint']['joint'] == 'telescope_stage1_joint'
    control = model.find('ros2_control')
    assert control.find('hardware/plugin').text == 'ign_ros2_control/IgnitionSystem'
    for joint in control.findall('joint'):
        commands = joint.findall('command_interface')
        if joint.get('name').startswith('caster_'):
            assert not commands, 'Passive caster was accidentally motorized'
        else:
            assert len(commands) == 1
    for mesh in model.findall('.//visual/geometry/mesh'):
        assert mesh.get('filename').startswith('package://robot_lab_robots/' + variant + '/meshes/')
        assert (folder / 'meshes' / Path(mesh.get('filename')).name).is_file()


@pytest.mark.parametrize('variant', ['molar_hri', 'probot_l'])
def test_scoped_profile_and_control_limits(variant):
    profiles = yaml.safe_load((ROOT / 'src/robot_lab_robots/config/robots.yaml').read_text())['robots']
    profile = profiles[variant]
    assert profile['supported_modes'] == ['display']
    assert set(profile['supported_simulators']) == {'mujoco', 'pybullet', 'gazebo', 'isaac'}
    assert profile['drive']['type'] == 'diff'
    assert profile['drive']['wheel_radius'] == .09
    assert profile['drive']['wheel_separation'] == .573
    assert profile['drive']['max_speed'] <= .25
    assert profile['articulation']['joints']['transfer_roller_joint']['max_velocity'] <= 8.
    assert 'molar_position_controller' in profile['controllers']
