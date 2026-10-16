"""Textured world meshes for the non-Gazebo renderers (R7.2, 2026-10-16).

MuJoCo drew small_house's collision meshes in one grey, where Gazebo shows
the models' Collada textures; visual odometry had nothing to track.  A
Collada visual is now read into one part per material (corners, texture
coordinates, image) and staged as OBJ + MTL + PNG.
"""
import os
import sys

import numpy as np
import pytest

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_SRC, 'robot_lab_utils'))

QUAD = """<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="0.01" name="centimeter"/><up_axis>Z_UP</up_axis></asset>
  <library_images><image id="board-image"><init_from>board.png</init_from></image></library_images>
  <library_effects>
    <effect id="textured-fx"><profile_COMMON><technique sid="t"><phong>
      <diffuse><texture texture="board-image" texcoord="CHANNEL0"/></diffuse>
    </phong></technique></profile_COMMON></effect>
    <effect id="red-fx"><profile_COMMON><technique sid="t"><lambert>
      <diffuse><color>1 0 0 1</color></diffuse>
    </lambert></technique></profile_COMMON></effect>
  </library_effects>
  <library_materials>
    <material id="textured"><instance_effect url="#textured-fx"/></material>
    <material id="red"><instance_effect url="#red-fx"/></material>
  </library_materials>
  <library_geometries><geometry id="quad"><mesh>
    <source id="p"><float_array id="pa" count="12">0 0 0 100 0 0 100 100 0 0 100 0</float_array>
      <technique_common><accessor source="#pa" count="4" stride="3"/></technique_common></source>
    <source id="t"><float_array id="ta" count="8">0 0 1 0 1 1 0 1</float_array>
      <technique_common><accessor source="#ta" count="4" stride="2"/></technique_common></source>
    <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
    <triangles material="m0" count="2"><input semantic="VERTEX" source="#v" offset="0"/>
      <input semantic="TEXCOORD" source="#t" offset="1"/><p>0 0 1 1 2 2 0 0 2 2 3 3</p></triangles>
    <triangles material="m1" count="2"><input semantic="VERTEX" source="#v" offset="0"/>
      <input semantic="TEXCOORD" source="#t" offset="1"/><p>0 0 1 1 2 2 0 0 2 2 3 3</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="s"><node id="n">
    <translate>0 0 50</translate>
    <instance_geometry url="#quad"><bind_material><technique_common>
      <instance_material symbol="m0" target="#textured"/>
      <instance_material symbol="m1" target="#red"/>
    </technique_common></bind_material></instance_geometry>
  </node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#s"/></scene>
</COLLADA>
"""


@pytest.fixture
def quad(tmp_path, monkeypatch):
    from PIL import Image
    monkeypatch.setenv('TMPDIR', str(tmp_path / 'cache'))
    import tempfile
    monkeypatch.setattr(tempfile, 'tempdir', None)
    Image.fromarray((np.indices((64, 64)).sum(0) % 16 < 8).astype(np.uint8) * 255).save(tmp_path / 'board.png')
    path = tmp_path / 'quad.dae'
    path.write_text(QUAD)
    return str(path)


def test_collada_parts_carry_units_transforms_texture_and_colour(quad):
    from robot_lab_utils.textured_mesh import parse_collada_parts
    parts = {('image' if part['image'] else 'colour'): part for part in parse_collada_parts(quad)}
    textured, red = parts['image'], parts['colour']
    assert textured['image'].endswith('board.png')
    assert red['rgba'] == [1.0, 0.0, 0.0, 1.0] and red['uv'] is None
    # Centimetres, with the node's 50 cm lift.
    assert np.allclose(textured['positions'][:3], [[0, 0, .5], [1, 0, .5], [1, 1, .5]])
    assert np.allclose(textured['uv'][:3], [[0, 0], [1, 0], [1, 1]])


def test_staged_parts_are_obj_with_texture_coordinates_and_an_mtl(quad):
    from robot_lab_utils.textured_mesh import stage_textured_parts
    parts = stage_textured_parts(quad, scale=(2.0, 2.0, 2.0))
    textured = [part for part in parts if part['texture']]
    assert len(parts) == 2 and len(textured) == 1
    obj = open(textured[0]['obj']).read()
    assert 'vt ' in obj and 'f 1/1 2/2 3/3' in obj and 'v 2.000000 0.000000 1.000000' in obj
    mtl = open(textured[0]['obj'][:-4] + '.mtl').read()
    assert 'map_Kd ' + os.path.basename(textured[0]['texture']) in mtl
    assert stage_textured_parts(quad, scale=(2.0, 2.0, 2.0)) == parts   # cached


def test_staged_parts_compile_as_textured_mujoco_geoms(quad):
    mujoco = pytest.importorskip('mujoco')
    from robot_lab_utils.textured_mesh import stage_textured_parts
    textured = [part for part in stage_textured_parts(quad) if part['texture']][0]
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><asset><texture name="t" type="2d" file="%s"/><material name="m" texture="t"/>'
        '<mesh name="q" file="%s" inertia="shell"/></asset><worldbody>'
        '<geom type="mesh" mesh="q" material="m" contype="0" conaffinity="0"/></worldbody></mujoco>'
        % (textured['texture'], textured['obj']))
    assert model.ntex == 1 and model.mesh_texcoordnum[0] >= 3
