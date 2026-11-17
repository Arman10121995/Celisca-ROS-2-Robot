"""Published descriptions must remain loadable away from their source folder."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from urllib.parse import unquote, urlparse

import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'robot_lab_utils'))
from robot_lab_utils.robot_description import load_description, resolve_relative_assets
from robot_lab_utils.gazebo_display import add_display_plugins, hold_command_topic


def test_h1_resources_are_absolute_and_exist():
    path = SRC / 'robot_lab_robots/unitree/h1_2_description/h1_2.urdf'
    root = ET.fromstring(load_description(path))
    meshes = root.findall('.//mesh')
    assert len(meshes) > 20
    for mesh in meshes:
        uri = urlparse(mesh.get('filename'))
        assert uri.scheme == 'file'
        assert Path(unquote(uri.path)).is_file()
    assert root.find("link[@name='pelvis']") is not None


def test_resource_with_spaces_and_missing_resource(tmp_path):
    (tmp_path / 'leg mesh.stl').write_text('solid leg\nendsolid leg\n')
    urdf = '<robot><link name="base"><visual><geometry><mesh filename="leg mesh.stl"/></geometry></visual></link></robot>'
    resolved = resolve_relative_assets(urdf, tmp_path / 'robot.urdf')
    assert '%20' in resolved
    with pytest.raises(ValueError, match='does not exist'):
        resolve_relative_assets(urdf.replace('leg mesh', 'missing'), tmp_path / 'robot.urdf')


def test_passive_gazebo_preview_disables_gravity_only_when_held():
    urdf = '<robot name="test"><link name="base"/><link name="leg"/></robot>'
    held = ET.fromstring(add_display_plugins(urdf, hold=True))
    assert {b.get('reference') for b in held.findall('gazebo') if b.findtext('gravity') == 'false'} == {'base', 'leg'}
    assert 'gravity' not in add_display_plugins(urdf, hold=False)


def test_display_hold_topics_are_bridgeable_and_cover_mimic_joints():
    from rclpy.validate_topic_name import validate_topic_name
    urdf = ('<robot name="test"><link name="base"/><link name="a"/><link name="b"/>'
            '<joint name="finger1" type="prismatic"><parent link="base"/><child link="a"/></joint>'
            '<joint name="finger2" type="prismatic"><parent link="base"/><child link="b"/>'
            '<mimic joint="finger1"/></joint></robot>')
    plugins = ET.fromstring(add_display_plugins(urdf, hold=True, model_name='arm-1')).findall('gazebo/plugin')
    held = {p.findtext('joint_name'): p.findtext('topic') for p in plugins if p.findtext('joint_name')}
    assert held == {'finger1': hold_command_topic('arm-1', 'finger1'), 'finger2': hold_command_topic('arm-1', 'finger2')}
    for topic in held.values():
        validate_topic_name(topic)
    assert hold_command_topic('arm-1', '2nd joint') == '/model/arm_1/joint/j2nd_joint/cmd_pos'
    # Without a model name: Gazebo's default topics and no mimic hold, as before.
    plain = ET.fromstring(add_display_plugins(urdf, hold=True)).findall('gazebo/plugin')
    assert [p.findtext('joint_name') for p in plain if p.findtext('joint_name')] == ['finger1']
    assert all(p.find('topic') is None for p in plain)
