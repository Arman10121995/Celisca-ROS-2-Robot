"""Installed profiles must preserve existing selections and missing-asset gates."""
from pathlib import Path

import pytest
import yaml

from robot_lab_utils.installed_assets import installed_profiles, merge_installed_profiles
from robot_lab_adapter.resolver import _world_arguments
from robot_lab_registry.catalog import Registry
from robot_lab_registry.validation import Composition, check_composition


@pytest.fixture
def store(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_LAB_RUNTIME_ROOT', str(tmp_path))
    root = tmp_path/'external_assets'/'installed'
    root.mkdir(parents=True)
    return root


def test_missing_native_plant_is_not_selectable(store):
    description = store/'robot.urdf'; description.write_text('<robot/>')
    native = store/'robot.xml'
    profile = {'xacro': str(description), 'native_mjcf': str(native)}
    (store/'robots.yaml').write_text(yaml.safe_dump({'robots': {'new_robot': profile}}))
    assert installed_profiles('robots') == {}
    native.write_text('<mujoco/>')
    assert installed_profiles('robots') == {'new_robot': profile}
    native.unlink()
    assert installed_profiles('robots') == {}


def test_extension_cannot_replace_legacy_robot(store):
    description = store/'robot.urdf'; description.touch()
    (store/'robots.yaml').write_text(yaml.safe_dump({'robots': {'bumperbot': {'xacro': str(description)}}}))
    with pytest.raises(ValueError, match='replace existing'):
        merge_installed_profiles({'robots': {'bumperbot': {'xacro': 'original'}}}, 'robots')


def test_absolute_world_preserves_selected_id_and_actual_map():
    world = '/workspace/assets/worlds/dataset_room2.world'
    occupancy = '/workspace/assets/maps/generated.yaml'
    arguments = _world_arguments({'id': 'dataset_room2', 'world_file': world, 'occupancy_yaml': occupancy})
    assert arguments == {'map_name': 'dataset_room2', 'world_package': 'robot_lab_maps',
                         'world_name': 'dataset_room2', 'world_path': world, 'map_yaml': occupancy}
    assert _world_arguments({'id':'dataset_room2', 'world_file':world}).get('map_yaml') is None


def test_legacy_world_arguments_remain_package_relative():
    arguments = _world_arguments({'id':'nav_empty','world_file':'robot_lab_maps/maps/nav_empty/worlds/nav_empty.world'})
    assert arguments['world_path'] == 'maps/nav_empty/worlds/nav_empty.world'
    assert arguments['map_yaml'] == 'maps/nav_empty/maps/map.yaml'


def test_passive_display_does_not_weaken_mission_or_backend_gates(store):
    config = Path(__file__).resolve().parents[2]/'robot_lab'/'robot_lab_registry'/'config'
    registry = Registry(config); assert registry.load(config)
    robot = dict(registry.robots.get('bumperbot'), id='test_arm', robot_class='manipulator',
                 supported_simulators=['mujoco'], capabilities=['display'], tags=['extension'])
    registry.robots.entities['test_arm'] = robot
    display = Composition('test_arm', 'small_office', 'mujoco', mode='display')
    assert check_composition(registry, display).valid
    assert not check_composition(registry, Composition('test_arm','small_office','gazebo',mode='display')).valid
    assert not check_composition(registry, Composition('test_arm','small_office','mujoco',mode='nav')).valid
    assert not check_composition(registry, Composition('test_arm','small_office','mujoco',mode='display',
                                                      algorithm_ids={'localization':'amcl'})).valid
