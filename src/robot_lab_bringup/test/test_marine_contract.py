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
    assert hydro.findtext('default_current') is not None
    assert all(float(hydro.findtext(key))<0. for key in ('xU','yV','zW'))
    assert all(float(hydro.findtext(key))<0. for key in ('xUabsU','yVabsV','zWabsW'))


def test_no_unsupported_current_or_light_plugins():
    for name in ('marine_tank','marine_tank_textured','marine_open_water','marine_reef','marine_harbor','marine_cave','marine_wreck'):
        world=ET.parse(ROOT/'src/robot_lab_maps/maps'/name/'worlds'/(name+'.world')).getroot().find('world')
        for plugin in world.findall('plugin'):
            assert 'current-system' not in plugin.get('filename',''), name
            assert 'light-system' not in plugin.get('filename',''), name


def test_water_is_a_density_boundary_not_a_solid_collision_box():
    for name in ('marine_tank','marine_tank_textured','marine_open_water','marine_reef','marine_harbor','marine_cave','marine_wreck'):
        world=ET.parse(ROOT/'src/robot_lab_maps/maps'/name/'worlds'/(name+'.world')).getroot().find('world')
        assert world.find("model[@name='water_surface']/link/collision") is None
        assert world.find("model[@name='seafloor']/link/collision") is not None
        buoyancy=next(p for p in world.findall('plugin') if p.get('name').endswith('::Buoyancy'))
        assert float(buoyancy.findtext('graded_buoyancy/default_density'))==1025.
        assert float(buoyancy.findtext('graded_buoyancy/density_change/above_depth'))==0.
        assert float(buoyancy.findtext('graded_buoyancy/density_change/density'))<2.
        assert buoyancy.findtext('enable')=='lab_rov'


def test_fortress_worlds_do_not_require_the_unavailable_harmonic_dvl_plugin():
    for name in ('marine_tank','marine_tank_textured','marine_open_water','marine_reef','marine_harbor','marine_cave','marine_wreck'):
        world=ET.parse(ROOT/'src/robot_lab_maps/maps'/name/'worlds'/(name+'.world')).getroot().find('world')
        dvl=[p for p in world.findall('plugin') if p.get('name','').endswith('DopplerVelocityLogSystem')]
        assert not dvl, name
        assert any(p.get('name','').endswith('::Buoyancy') for p in world.findall('plugin'))


def test_rov_keeps_native_imu_and_uses_separate_emulated_dvl():
    description=load_description(ROOT/'src/robot_lab_robots/lab_rov/urdf/lab_rov.urdf.xacro')
    root=ET.fromstring(description)
    sensors=[s for g in root.findall('gazebo') if g.get('reference')=='base_link'
             for s in g.findall('sensor') if s.get('name')=='marine_imu']
    assert len(sensors)==1
    sensor=sensors[0]
    assert sensor.get('type')=='imu'
    assert sensor.findtext('always_on')=='true'
    assert float(sensor.findtext('update_rate'))>0.
    assert sensor.findtext('gz_frame_id')=='base_link'


def test_textured_tank_structure_is_in_view_and_clear_of_the_measured_route():
    """Relocalization panels (R10.5) stay clear of the route, its braking corridor and the walls."""
    import math
    world=ET.parse(ROOT/'src/robot_lab_maps/maps/marine_tank_textured/worlds/marine_tank_textured.world').getroot().find('world')
    start,goal=np.array([-4.,-3.]),np.array([-.93,-1.32])
    heading=(goal-start)/np.linalg.norm(goal-start)
    end=goal+3.*heading                       # 3 m terminal braking corridor
    added=[m for m in world.findall('model') if m.get('name').startswith('relocalization_')]
    assert len(added)==4
    for model in added:
        x,y,z,_,_,yaw=map(float,model.findtext('pose').split())
        sx,sy,sz=map(float,model.findtext('link/visual/geometry/box/size').split())
        reach=math.hypot(sx,sy)/2
        p=np.array([x,y]); t=np.clip((p-start)@(end-start)/((end-start)@(end-start)),0,1)
        assert np.linalg.norm(p-(start+t*(end-start)))-reach>1.3+.5, model.get('name')   # planner radius + margin
        bearing=math.atan2(y-start[1],x-start[0])-.5
        assert abs(bearing)<math.radians(36) and np.linalg.norm(p-start)<8., model.get('name')   # in the camera view
        assert abs(x)+reach<5.85 and abs(y)+reach<4.85
        texture=model.findtext('link/visual/material/pbr/metal/albedo_map')
        assert (ROOT/'src/robot_lab_maps/maps/marine_tank_textured/worlds'/texture).resolve().is_file()


def test_tethys_slam_is_geometry_only_unless_visual_features_are_requested():
    from robot_lab_utils.marine_pipeline import depth_registration_parameters
    assert depth_registration_parameters(geometry_only=True)['Kp/MaxFeatures']=='-1'
    visual=depth_registration_parameters(geometry_only=False)
    assert 'Kp/MaxFeatures' not in visual and visual['Reg/Strategy']=='2'
    launch=(ROOT/'src/robot_lab_bringup/launch/auv_simulator.launch.py').read_text()
    assert "DeclareLaunchArgument('visual_features',default_value='false'" in launch


def test_geometry_only_tethys_localization_starts_at_the_mapped_origin():
    # No visual words means no appearance loop closure; ICP proximity needs the
    # start pose, which the spawn-aligned estimator shares with the mapping run.
    pipeline=(ROOT/'src/robot_lab_utils/robot_lab_utils/marine_pipeline.py').read_text()
    assert ("**({'RGBD/StartAtOrigin':'true'} if localization and robot=='tethys_auv' and not visual_features else {})"
            in pipeline)


def test_marine_launch_uses_the_explicit_sensor_pipeline_instead_of_a_missing_native_bridge():
    launch=(ROOT/'src/robot_lab_bringup/launch/marine_simulator.launch.py').read_text()
    assert "'/marine/dvl'" not in launch
    assert 'actions += pipeline(' in launch
    pipeline=(ROOT/'src/robot_lab_utils/robot_lab_utils/marine_pipeline.py').read_text()
    assert "executable='marine_sensors.py'" in pipeline
    assert "arguments=['simulate']" in pipeline and "arguments=['estimate']" in pipeline
