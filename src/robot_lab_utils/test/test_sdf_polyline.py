"""SDF <polyline> geometry as convex STL prisms (sdf_polyline, sdf_world)."""
import struct

import pytest

from robot_lab_utils.sdf_polyline import polyline_prisms, triangulate
from robot_lab_utils.sdf_world import extract_static_shapes


def _stl_vertices(path):
    data = open(path, 'rb').read()
    count = struct.unpack('<I', data[80:84])[0]
    vertices = []
    for i in range(count):
        values = struct.unpack('<12f', data[84 + 50 * i:84 + 50 * i + 48])
        vertices += [values[3:6], values[6:9], values[9:12]]
    return vertices


def test_concave_outline_is_triangulated_without_covering_the_notch():
    l_shape = [(0, 0), (2, 0), (2, 1), (1, 1), (1, 2), (0, 2)]
    points, triangles = triangulate(l_shape)
    assert len(triangles) == 4
    area = sum(abs((points[b][0] - points[a][0]) * (points[c][1] - points[a][1])
                   - (points[b][1] - points[a][1]) * (points[c][0] - points[a][0])) / 2 for a, b, c in triangles)
    assert area == pytest.approx(3.)                      # the 1 x 1 notch is not filled


def test_prisms_span_the_height_and_are_cached(tmp_path):
    paths = polyline_prisms([[(0, 0), (1, 0), (0, 1), (0, 0)]], .5, directory=tmp_path)
    assert len(paths) == 1
    zs = {round(v[2], 6) for v in _stl_vertices(paths[0])}
    assert zs == {0., .5}
    assert polyline_prisms([[(0, 0), (1, 0), (0, 1)]], .5, directory=tmp_path) == paths


def test_world_polyline_becomes_posed_mesh_records(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_LAB_RUNTIME_ROOT', str(tmp_path))
    world = tmp_path / 'w.world'
    world.write_text('''<sdf version="1.7"><world name="w"><model name="wall"><static>true</static>
      <pose>1 2 0 0 0 0</pose><link name="l"><collision name="c"><geometry><polyline>
      <point>0 0</point><point>2 0</point><point>2 1</point><point>1 1</point><point>1 2</point><point>0 2</point>
      <height>0.8</height></polyline></geometry></collision></link></model></world></sdf>''')
    shapes, skipped = extract_static_shapes(str(world), lambda uri, base: '')
    assert not skipped
    assert len(shapes) == 4 and all(s['type'] == 'mesh' and s['polyline'] for s in shapes)
    assert shapes[0]['position'] == [1., 2., 0.]


def test_degenerate_polyline_is_reported(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_LAB_RUNTIME_ROOT', str(tmp_path))
    world = tmp_path / 'w.world'
    world.write_text('''<sdf version="1.7"><world name="w"><model name="m"><static>true</static><link name="l">
      <collision name="c"><geometry><polyline><point>0 0</point><point>1 1</point><height>1</height></polyline>
      </geometry></collision></link></model></world></sdf>''')
    shapes, skipped = extract_static_shapes(str(world), lambda uri, base: '')
    assert not shapes and 'polyline' in skipped[0]
