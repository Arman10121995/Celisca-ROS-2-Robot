"""World-step preparation must retain geometry, identity and local resources."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from robot_lab_utils.gazebo_physics_world import (
    COMPLIANT_STEP_BOUNDS,
    bounded_world_for_robot,
    cap_physics_step,
)


def test_compliant_robot_cap_preserves_selected_geometry_and_resource_closure(tmp_path):
    source = tmp_path/'original'; source.mkdir()
    (source/'terrain.png').write_bytes(b'Fixture resource, not generated robot metrics')
    world = source/'selected.world'
    world.write_text('''<sdf version="1.7"><world name="selected">
      <physics type="ode"><max_step_size>.01</max_step_size></physics>
      <model name="actual_floor"><static>true</static><link name="floor">
        <collision name="terrain"><geometry><heightmap><uri>terrain.png</uri>
        <size>12 10 2</size></heightmap></geometry></collision></link></model>
      <plugin filename="actor_system" name="selected_plugin"/></world></sdf>''')
    original = world.read_bytes()
    target = Path(cap_physics_step(world, .002, tmp_path/'runtime'))
    root = ET.parse(target).getroot()
    assert world.read_bytes() == original
    assert root.find('world').get('name') == 'selected'
    assert root.find('world/model').get('name') == 'actual_floor'
    assert root.find('.//heightmap/size').text == '12 10 2'
    assert root.find('.//heightmap/uri').text == (source/'terrain.png').as_uri()
    assert root.find('world/plugin').get('filename') == 'actor_system'
    assert root.find('world/physics/max_step_size').text == '0.002'
    assert cap_physics_step(world, .002, tmp_path/'runtime') == str(target)


@pytest.mark.parametrize('step', [.001, .002, None])
def test_smaller_or_default_world_step_is_not_increased(tmp_path, step):
    physics = '' if step is None else f'<physics><max_step_size>{step}</max_step_size></physics>'
    world = tmp_path/'world.sdf'; world.write_text(f'<sdf><world name="kept">{physics}</world></sdf>')
    assert cap_physics_step(world, .002, tmp_path/'runtime') == str(world)
    assert not (tmp_path/'runtime').exists()


@pytest.mark.parametrize('step', [0, -.001, float('nan'), float('inf'), .1])
def test_invalid_step_rejected(tmp_path, step):
    world = tmp_path/'world.sdf'; world.write_text('<sdf><world name="kept"/></sdf>')
    with pytest.raises(ValueError, match='maximum step'):
        cap_physics_step(world, step, tmp_path/'runtime')


def _coarse_world(tmp_path):
    world = tmp_path/'selected.world'
    world.write_text('<sdf version="1.7"><world name="selected">'
                     '<physics type="ode"><max_step_size>0.01</max_step_size></physics>'
                     '<model name="floor"><static>true</static></model></world></sdf>')
    return world


def test_bounded_robot_caps_only_the_gazebo_path(tmp_path):
    world = _coarse_world(tmp_path)
    profile = dict(name='compliant_base', gazebo_max_physics_step_s=.002)
    target = Path(bounded_world_for_robot(world, profile, 'gazebo', tmp_path/'runtime'))
    assert target != world.resolve()
    assert world.read_text().count('0.01') == 1
    assert ET.parse(target).find('world/physics/max_step_size').text == '0.002'
    # The same selection on another backend keeps the exact selected world.
    assert bounded_world_for_robot(world, profile, 'mujoco', tmp_path/'runtime') == str(world)


def test_robot_without_a_bound_launches_the_selected_world(tmp_path):
    world = _coarse_world(tmp_path)
    assert bounded_world_for_robot(world, dict(name='plain'), 'gazebo',
                                   tmp_path/'runtime') == str(world)
    assert bounded_world_for_robot(world, None, 'gazebo',
                                   tmp_path/'runtime') == str(world)
    assert not (tmp_path/'runtime').exists()


def test_world_already_inside_the_bound_is_not_rewritten(tmp_path):
    world = tmp_path/'fine.world'
    world.write_text('<sdf><world name="fine">'
                     '<physics><max_step_size>0.002</max_step_size></physics></world></sdf>')
    profile = dict(name='compliant_base', gazebo_max_physics_step_s=.002)
    assert bounded_world_for_robot(world, profile, 'gazebo', tmp_path/'runtime') == str(world)
    assert not (tmp_path/'runtime').exists()


def test_turtlebot4_profiles_carry_the_measured_step_bound():
    """Both installed TurtleBot4 profiles must keep the measured 2 ms bound."""
    assert COMPLIANT_STEP_BOUNDS == {
        'asset_turtlebot4_standard': .002,
        'asset_turtlebot4_lite': .002,
    }


def test_launch_and_provisioner_are_wired_to_the_bound():
    """The bound must reach the launch (via the profile) and the profile must
    come from the provisioner, not from a hand-edited catalog."""
    bringup = Path(__file__).resolve().parents[1]
    launch = (bringup/'launch'/'simulated_robot.launch.py').read_text()
    assert 'bounded_world_for_robot' in launch
    provisioner = (bringup.parents[1]/'scripts'/'provision_extension_assets.py').read_text()
    assert 'COMPLIANT_STEP_BOUNDS' in provisioner
    assert "profile['gazebo_max_physics_step_s']" in provisioner
