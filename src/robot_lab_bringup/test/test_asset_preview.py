"""Preview source poses/scales must agree with authored geometry."""
import math
from pathlib import Path

import numpy as np
import pytest

from robot_lab_utils.asset_preview import urdf_shapes, scene_bounds
from robot_lab_utils.sdf_world import extract_static_shapes, uri_resolver


def test_preview_composes_joint_visual_and_mimic_frames_in_metres():
    robot = '''<robot><link name="base"/><link name="slide"/><link name="tip">
      <visual><origin xyz="1 0 0"/><geometry><box size="2 4 6"/></geometry></visual></link>
      <joint name="extend" type="prismatic"><parent link="base"/><child link="slide"/>
        <origin xyz="2 0 0" rpy="0 0 1.5707963267948966"/><axis xyz="1 0 0"/></joint>
      <joint name="rotate" type="revolute"><parent link="slide"/><child link="tip"/>
        <axis xyz="0 0 1"/><mimic joint="extend" multiplier="0.5" offset="0"/></joint></robot>'''
    shapes, notes = urdf_shapes(robot, lambda *_: '', joint_positions={'extend': math.pi})
    assert not notes
    np.testing.assert_allclose(shapes[0]['position'], [1., math.pi, 0.], atol=1e-12)
    bounds = scene_bounds(shapes)
    np.testing.assert_allclose(bounds['min_xyz'], [0., math.pi-2., -3.], atol=1e-12)
    np.testing.assert_allclose(bounds['max_xyz'], [2., math.pi+2., 3.], atol=1e-12)


@pytest.mark.parametrize('robot', [
    '<robot><link name="a"/><link name="b"/></robot>',
    '<robot><link name="a"/><link name="b"/><joint name="j" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 0"/></joint></robot>',
    '<robot><link name="a"/><link name="b"/><joint name="j" type="floating"><parent link="a"/><child link="b"/></joint></robot>',
])
def test_invalid_or_unsupported_robot_tree_is_not_shown_as_a_successful_preview(robot):
    with pytest.raises(ValueError):
        urdf_shapes(robot, lambda *_: '')


def test_unresolved_visual_mesh_is_reported_without_a_placeholder():
    with pytest.raises(ValueError, match='Unresolved preview mesh'):
        urdf_shapes('<robot><link name="base"><visual><geometry><mesh filename="missing.stl"/></geometry></visual></link></robot>', lambda *_: '')


def test_world_preview_uses_visuals_recursively_and_leaves_physics_collision_first(tmp_path):
    models = tmp_path/'models'; model = models/'fixture'; model.mkdir(parents=True)
    (model/'model.sdf').write_text('''<sdf><model name="fixture"><link name="part">
      <collision><geometry><box><size>1 1 1</size></box></geometry></collision>
      <visual><pose>1 2 3 0 0 0</pose><geometry><box><size>2 3 4</size></box></geometry>
        <material><diffuse>0.1 0.2 0.3 1</diffuse></material></visual></link></model></sdf>''')
    world = tmp_path/'scene.world'
    world.write_text('<sdf><world name="scene"><include><uri>model://fixture</uri><pose>4 5 6 0 0 0</pose></include></world></sdf>')
    resolve = uri_resolver([str(models)])
    physical, _ = extract_static_shapes(str(world), resolve)
    visual, notes = extract_static_shapes(str(world), resolve, prefer_visual=True)
    assert not notes and physical[0]['position'] == [4., 5., 6.]
    assert physical[0]['size'] == [1., 1., 1.]
    assert visual[0]['position'] == [5., 7., 9.] and visual[0]['size'] == [2., 3., 4.]
    assert visual[0]['rgba'] == [.1, .2, .3, 1.]
    with pytest.raises(ValueError):
        extract_static_shapes(str(world), resolve, collision_only=True, prefer_visual=True)


def test_fit_uses_actual_geometry_without_rescaling_it_to_a_ground_plane():
    shapes = [dict(type='plane', size=[1000., 1000.], position=[0., 0., 0.], orientation=[1., 0., 0., 0.]),
              dict(type='box', size=[2., 4., 6.], position=[1., 2., 3.], orientation=[1., 0., 0., 0.])]
    bounds = scene_bounds(shapes)
    assert bounds['min_xyz'] == [0., 0., 0.] and bounds['max_xyz'] == [2., 4., 6.]
    assert shapes[0]['size'] == [1000., 1000.]


def test_remote_upstream_fixture_does_not_hide_building_or_change_authored_pose():
    shapes = [dict(type='box', size=[1., 1., 1.], position=[float(i), 0., 0.], orientation=[1., 0., 0., 0.])
              for i in range(20)]
    remote = dict(type='box', size=[1., 1., 1.], position=[1., 0., -754989000.], orientation=[1., 0., 0., 0.])
    shapes.append(remote)
    bounds = scene_bounds(shapes)
    assert bounds['remote_geometry_count'] == 1
    assert bounds['min_xyz'][2] == -.5 and bounds['raw_min_xyz'][2] == -754989000.5
    assert remote['position'][2] == -754989000.
