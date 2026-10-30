"""Source support and estimator contracts, separate from physical missions."""
from pathlib import Path
import math
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/robot_lab_utils'))
from robot_lab_utils.amphibious_planning import ShoreScene
from robot_lab_utils.marine_navigation_model import MarineOdometry


def scene():return ShoreScene(ROOT/'src/robot_lab_maps/maps/amphibious_shore/worlds/amphibious_shore.world')


def test_support_uses_source_slope_and_water_instead_of_flattening_the_route():
    s=scene();assert s.height(-6,0)==pytest.approx(.74)
    # Independent ray-plane intersection of the rotated ramp's top face.
    slope=1.9/8.;expected=-.65+3*slope+.1*math.sqrt(1+slope*slope)
    assert s.support_height(-2,0)==pytest.approx(expected)
    assert s.height(6,0)==pytest.approx(.134)
    with pytest.raises(ValueError):s.height(50,0)


def test_only_declared_support_points_are_filtered_unknowns_remain_obstacles():
    s=scene();points=np.array([[-6,0,.4],[-6,0,.45],[50,0,.4]])
    np.testing.assert_array_equal(s.above_support(points),points[1:])


def test_atmospheric_pressure_cannot_collapse_an_amphibious_land_height():
    state=MarineOdometry([0,0,.74]);state.update(1.,[.1,0,0],np.eye(3)*.0001,[0,0,0],101325.,75.**2,use_pressure=False)
    position=state.update(1.1,[.1,0,0],np.eye(3)*.0001,[0,0,0],101325.,75.**2,use_pressure=False)
    assert position[2]==pytest.approx(.74);assert position[0]>0
