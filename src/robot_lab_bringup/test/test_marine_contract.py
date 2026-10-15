"""Authored-source force/water contracts, separate from runtime missions."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/robot_lab_utils'))
from robot_lab_utils.marine_model import MarineModel
from robot_lab_utils.robot_description import load_description


@pytest.fixture(scope='module')
def model():
    return MarineModel(load_description(ROOT/'src/robot_lab_robots/lab_rov/urdf/lab_rov.urdf.xacro'))


def test_six_independent_source_wrenches_are_reconstructed(model):
    assert np.linalg.matrix_rank(model.matrix)==6
    for wrench in np.eye(6)*.5:
        forces,scale=model.allocate(wrench)
        assert scale==1.
        np.testing.assert_allclose(model.matrix@forces,wrench,atol=1e-12)


def test_saturation_preserves_requested_wrench_direction(model):
    desired=np.array([100.,-80.,70.,20.,-30.,40.])
    forces,scale=model.allocate(desired)
    assert 0<scale<1
    assert (forces>=model.bounds[:,0]-1e-12).all()
    assert (forces<=model.bounds[:,1]+1e-12).all()
    np.testing.assert_allclose(model.matrix@forces,desired*scale,atol=1e-10)


@pytest.mark.parametrize('wrench',[[0]*5,[0,0,0,0,0,float('nan')],[0,0,0,float('inf'),0,0]])
def test_invalid_force_request_cannot_reach_thrusters(model,wrench):
    with pytest.raises(ValueError):model.allocate(wrench)


def test_mass_volume_and_weight_compensation_come_from_source(model):
    assert abs(model.mass-18.5417)<1e-10
    assert abs(model.displacement-.017985475)<1e-12
    assert abs(model.gravity_trim-(model.mass-model.density*model.displacement)*9.81)<1e-12


def test_native_added_mass_opposes_acceleration_on_fortress_618(model):
    description=load_description(ROOT/'src/robot_lab_robots/lab_rov/urdf/lab_rov.urdf.xacro')
    hydro=next(p for p in ET.fromstring(description).findall('gazebo/plugin') if p.get('name').endswith('::Hydrodynamics'))
    assert all(float(hydro.findtext(key))>0. for key in ('xDotU','yDotV','zDotW','kDotP','mDotQ','nDotR'))
    assert all(float(hydro.findtext(key))<0. for key in ('xU','yV','zW'))


def test_water_is_a_density_boundary_not_a_solid_collision_box():
    for name in ('marine_tank','marine_reef','marine_harbor','marine_cave','marine_wreck'):
        world=ET.parse(ROOT/'src/robot_lab_maps/maps'/name/'worlds'/(name+'.world')).getroot().find('world')
        assert world.find("model[@name='water_surface']/link/collision") is None
        assert world.find("model[@name='seafloor']/link/collision") is not None
        buoyancy=next(p for p in world.findall('plugin') if p.get('name').endswith('::Buoyancy'))
        assert float(buoyancy.findtext('graded_buoyancy/default_density'))==1025.
        assert float(buoyancy.findtext('graded_buoyancy/density_change/above_depth'))==0.
        assert float(buoyancy.findtext('graded_buoyancy/density_change/density'))<2.
        assert buoyancy.findtext('enable')=='lab_rov'
