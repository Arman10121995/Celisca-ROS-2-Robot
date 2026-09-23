"""Published descriptions must remain loadable away from their source folder."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from urllib.parse import unquote, urlparse

import pytest

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'robot_lab_utils'))
from robot_lab_utils.robot_description import load_description, resolve_relative_assets
from robot_lab_utils.gazebo_display import add_display_plugins


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
