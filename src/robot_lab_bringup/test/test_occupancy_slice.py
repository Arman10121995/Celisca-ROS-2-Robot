"""Conservative rasterization preserves actual slice height, pose and cell overlap."""
import math

import pytest

from robot_lab_utils.occupancy_slice import primitive_slice_cells


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
