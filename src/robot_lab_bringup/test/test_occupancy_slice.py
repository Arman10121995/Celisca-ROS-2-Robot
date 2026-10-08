"""Conservative rasterization preserves actual slice height, pose and cell overlap."""
import math
import importlib.util
from pathlib import Path

import pytest

import numpy as np

from robot_lab_utils.occupancy_slice import primitive_slice_cells, terrain_slice_polygons


def box(position, size, yaw=0.):
    return {'type':'box','size':size,'position':position,
            'orientation':[math.cos(yaw/2),0.,0.,math.sin(yaw/2)]}


def test_floor_below_slice_is_free():
    assert primitive_slice_cells(box([0,0,-.1],[20,20,.2]),[30,30],.1,.3) is None


def test_exact_cell_overlap_and_translation():
    assert primitive_slice_cells(box([1,0,.5],[1,1,1]),[10,10],.5,.3) == (10,8,13,11)


def test_rotated_wall_bounds_are_conservative():
    cells = primitive_slice_cells(box([0,0,.5],[4,.2,1],math.pi/4),[10,10],.1,.3)
    assert cells == (35,35,64,64)
    assert primitive_slice_cells(box([0,0,.5],[4,.2,1]),[10,10],.1,.3) == (29,48,70,50)


def test_sphere_and_cylinder_use_real_dimensions():
    common={'position':[0,0,1],'orientation':[1,0,0,0]}
    assert primitive_slice_cells(dict(common,type='sphere',size=[.2]),[10,10],.1,.3) is None
    assert primitive_slice_cells(dict(common,type='cylinder',size=[.2,2]),[10,10],.1,.3) == (47,47,52,52)


def test_unknown_geometry_cannot_silently_become_free():
    with pytest.raises(ValueError,match='Unsupported primitive'):
        primitive_slice_cells({'type':'mesh','size':[1,1,1]},[10,10],.1,.3)


def test_terrain_slope_is_clipped_at_actual_height():
    vertices = [[0,0,0], [2,0,2], [0,2,0]]
    polygons = terrain_slice_polygons(vertices, [[0,1,2]], 1.)
    assert len(polygons) == 1
    assert {tuple(p) for p in polygons[0]} == {(1.,0.), (2.,0.), (1.,1.)}


def test_disconnected_hills_do_not_create_a_barrier_between_them():
    vertices = [[-3,0,2],[-2,0,2],[-3,1,2], [2,0,2],[3,0,2],[3,1,2]]
    polygons = terrain_slice_polygons(vertices, [[0,1,2],[3,4,5]], 1.)
    assert len(polygons) == 2
    assert polygons[0][:,0].max() < -1
    assert polygons[1][:,0].min() > 1
    assert terrain_slice_polygons(vertices, [[0,1,2],[3,4,5]], 3.) == []


def test_terrain_projection_uses_world_height_after_rotation():
    # A rotated/translated triangle whose local elevation would all be zero.
    vertices = np.array([[4,5,0], [4,7,2], [3,5,0.]])
    polygon, = terrain_slice_polygons(vertices, [[0,1,2]], 1.)
    assert {tuple(p) for p in polygon} == {(4.,6.),(4.,7.),(3.5,6.)}


def test_invalid_terrain_cannot_silently_become_free():
    with pytest.raises(ValueError, match='finite XYZ'):
        terrain_slice_polygons([[0,0,float('nan')]], [[0,0,0]], .3)


def test_open_mesh_wall_remains_an_occupancy_barrier(tmp_path):
    """A non-watertight source wall must block the slice, including its ends."""
    pytest.importorskip('ament_index_python', reason='Generate Map tool needs sourced ROS package paths')
    trimesh = pytest.importorskip('trimesh')
    spec = importlib.util.spec_from_file_location('map_generator',
        Path(__file__).resolve().parents[2]/'robot_lab_maps/tools/generate_occupancy_map.py')
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    mesh = trimesh.Trimesh(vertices=[[-2,0,0],[2,0,0],[2,0,1],[-2,0,1]],
        faces=[[0,1,2],[0,2,3]], process=False)
    path = tmp_path/'open-wall.stl'
    mesh.export(path)
    shape = dict(type='mesh', mesh=str(path), scale=[1,1,1],
        position=[3,4,0], orientation=[math.sqrt(.5),0,0,math.sqrt(.5)])
    segments, bounds = generator.slice_mesh(shape, .3, .1)
    points = np.concatenate(segments)
    assert points[:,0] == pytest.approx(np.full(len(points),3.))
    assert points[:,1].min() == pytest.approx(2.)
    assert points[:,1].max() == pytest.approx(6.)
    assert points[:,2] == pytest.approx(np.full(len(points),.3))
    assert np.sum([np.linalg.norm(s[1]-s[0]) for s in segments]) == pytest.approx(4.)
    assert generator.slice_mesh(shape, 1.5, .1)[0] == []
