"""Swept footprint intersections: all covered cells, reverse and rotated maps."""
from types import SimpleNamespace as NS
import math
import numpy as np
import pytest
from robot_lab_utils.car_parking_clearance import footprint_points,footprint_clear,swept_clear


FOOTPRINT=[[-.2,-.1],[.2,-.1],[.2,.1],[-.2,.1]]


def grid():
    return NS(info=NS(width=100,height=100,resolution=.05,origin=NS(position=NS(x=-2.5,y=-2.5),
        orientation=NS(x=0.,y=0.,z=0.,w=1.))),data=np.zeros(10000,dtype=int))


def obstacle(g,x,y,value=100):
    g.data[int((y+2.5)/.05)*100+int((x+2.5)/.05)]=value


@pytest.mark.parametrize('value',[-1,99,100])
def test_cell_inside_polygon_is_blocking_even_with_clear_vertices(value):
    g=grid();assert footprint_clear(g,FOOTPRINT)
    obstacle(g,0.,0.,value)
    assert not footprint_clear(g,FOOTPRINT)


def test_forward_and_reverse_sweeps_check_the_future_not_only_the_current_pose():
    for direction in (-1.,1.):
        g=grid();obstacle(g,direction*.3,0.)
        assert footprint_clear(g,FOOTPRINT)
        assert not swept_clear(g,FOOTPRINT,(0.,0.,0.),direction*.15,0.)
        assert swept_clear(g,FOOTPRINT,(0.,0.,0.),-direction*.15,0.)


def test_swinging_end_and_source_rear_footprint_are_checked():
    front=np.array([[.42,.17],[.42,-.17],[-.10,-.17],[-.10,.17]])
    rear=-front
    g=grid();obstacle(g,.4,.3)
    assert footprint_clear(g,front)
    assert not swept_clear(g,front,(0.,0.,0.),.15,.5)
    assert swept_clear(g,rear,(0.,0.,0.),.15,.5)


def test_rotated_grid_origin_and_outside_unknown_space():
    g=grid();g.info.origin.orientation.z=math.sin(math.pi/4);g.info.origin.orientation.w=math.cos(math.pi/4)
    # The original footprint in grid coordinates, transformed into the world.
    polygon=(np.array(FOOTPRINT)+[2.5,2.5])@np.array([[0.,1.],[-1.,0.]])+[-2.5,-2.5]
    assert footprint_clear(g,polygon)
    obstacle(g,0.,0.)
    assert not footprint_clear(g,polygon)
    assert not footprint_clear(grid(),np.array(FOOTPRINT)+[2.45,0.])


@pytest.mark.parametrize('shape',[[0.,0.,0.,0.,0.,0.],[[0.,0.],[1.,0.],[0.,1.],[1.,1.]],[[0.,0.],[1.,0.],[math.nan,1.]]])
def test_unreviewed_degenerate_or_nonfinite_polygon_is_refused(shape):
    with pytest.raises(ValueError):footprint_points(shape)
