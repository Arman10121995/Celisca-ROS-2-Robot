"""Terrain measurement, envelope and source-coordinate regression checks."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/robot_lab_utils'))
from robot_lab_utils.gait_terrain import TerrainAdaptation,StaticTerrainRay,sample_probes


def observation(stamp=1.):
    return dict(clock_time=stamp,feet={
        'left':dict(height_offset_m=-.004,ahead_offset_m=.002,spread_m=.006),
        'right':dict(height_offset_m=.004,ahead_offset_m=.008,spread_m=.004)})


def test_adaptation_is_bounded_and_common_body_heave_is_not_a_terrain_step():
    adapter=TerrainAdaptation(['left','right']);adapter.step(observation(),1.)
    measured=observation(1.1)
    result=adapter.step(measured,1.1)
    assert result['left']['offset']==pytest.approx(-.002)
    assert result['right']['offset']==pytest.approx(.002)
    assert result['left']['swing_extra']==pytest.approx(.006)
    for foot in measured['feet'].values():
        foot['height_offset_m']+=.01;foot['ahead_offset_m']+=.01
    result=adapter.step(dict(measured,clock_time=1.2),1.2)
    assert result['left']['offset']==pytest.approx(-.004)
    assert result['right']['offset']==pytest.approx(.004)


@pytest.mark.parametrize('fault',['stale','reset','missing','nonfinite','step','relief','drop'])
def test_unobserved_or_unreachable_terrain_is_refused(fault):
    adapter=TerrainAdaptation(['left','right']);adapter.step(observation(),1.)
    message=observation(1.1);clock=1.1
    if fault=='stale':clock=2.
    elif fault=='reset':message['clock_time']=.9
    elif fault=='missing':del message['feet']['right']
    elif fault=='nonfinite':message['feet']['left']['height_offset_m']=float('nan')
    elif fault=='step':message['feet']['left']['spread_m']=.021
    elif fault=='relief':message['feet']['left']['height_offset_m']=-.05
    else:
        for foot in message['feet'].values():
            foot['height_offset_m']-=.05;foot['ahead_offset_m']-=.05
    with pytest.raises(ValueError):adapter.step(message,clock)


def test_nominal_probe_uses_native_transform_and_measured_ground():
    probes={'left':dict(root='root',foot='sole',home=[.2,0.,-.2])}
    samples=sample_probes(probes,lambda _:([1.,2.,.25],np.eye(3)),lambda _:-.02,
                          lambda origin,length:float(origin[2]-.032),1.)
    assert samples['feet']['left']['height_offset_m']==pytest.approx(.002)
    with pytest.raises(ValueError,match='Missing observed'):
        sample_probes(probes,lambda _:([1.,2.,.25],np.eye(3)),lambda _:-.02,lambda *_:None,1.)


def test_flat_ground_does_not_convert_body_pitch_into_positive_feedback():
    from scipy.spatial.transform import Rotation
    probes={name:dict(root='root',foot=name,home=[x,0.,-.2]) for name,x in [('front',.3),('rear',-.3)]}
    matrix=Rotation.from_euler('y',.08).as_matrix()
    measured=sample_probes(probes,lambda _:([0.,0.,.22],matrix),lambda _:-.02,
        lambda origin,length:float(origin[2]),1.)
    assert measured['feet']['front']['height_offset_m']==pytest.approx(measured['feet']['rear']['height_offset_m'])


def test_static_source_ray_respects_height_and_missing_support(tmp_path):
    world=tmp_path/'raised.world'
    world.write_text('<sdf version="1.7"><world name="raised"><model name="deck"><static>true</static><pose>0 0 .2 0 0 0</pose><link name="base"><collision name="deck"><geometry><box><size>2 2 .4</size></box></geometry></collision></link></model></world></sdf>')
    ray=StaticTerrainRay(world)
    assert ray([0.,0.,.7],1.)==pytest.approx(.3)
    assert ray([2.,0.,.7],1.) is None
    assert ray.batch([[0.,0.,.7],[2.,0.,.7]],1.)==pytest.approx([.3,None])


def test_isaac_keeps_low_relief_instead_of_treating_it_as_duplicate_floor():
    import importlib.util
    path=ROOT/'src/robot_lab_isaac/python/robot_lab_isaac/isaac_runtime.py'
    spec=importlib.util.spec_from_file_location('terrain_sdk_runtime',path);runtime=importlib.util.module_from_spec(spec);spec.loader.exec_module(runtime)
    vertices=[[0.,0.,.002],[.1,0.,.003],[0.,.1,.004]];triangles=[[0,1,2]]
    assert runtime._off_ground_plane(vertices,triangles,dict(source_geometry='heightmap'))==(triangles,0)
    assert runtime._off_ground_plane(vertices,triangles,{})==([],1)


def test_isaac_plant_diagnostics_cannot_replace_owner_or_clear_live_process():
    import json
    import threading
    from types import SimpleNamespace
    sys.path.insert(0,str(ROOT/'src/robot_lab_isaac/python'))
    from robot_lab_isaac.isaac_spawner import IsaacSpawner
    node=object.__new__(IsaacSpawner);node._lock=threading.RLock()
    node._state=dict(t=1.,pos=[1.,2.,.2],orn=[0.,0.,0.,1.])
    contact=dict(clock_time=1.,normal_forces_N=dict(left=12.,right=11.,body=0.))
    node._latest_gait=dict(contacts=contact,terrain=dict(clock_time=1.,feet={}))
    node._proc=object();process=node._proc;node._spawned=False;messages=[]
    node._gait_plant_pub=SimpleNamespace(publish=messages.append)
    node._publish_gait_plant_state()
    observed=json.loads(messages[0].data)
    assert observed['actual_contacts']==contact and observed['position']==[1.,2.,.2]
    assert not {'enabled','operator_id','session_id','ready'} & observed.keys()
    assert node._proc is process and not node._spawned


@pytest.mark.parametrize('robot',['hexapod_ros2','lab_octopod','lab_centipede'])
def test_native_zero_reference_preserves_source_fk_and_effort(robot):
    import PyKDL as kdl
    from urdf_parser_py.urdf import URDF
    from robot_lab_utils.gait_model import GaitModel
    from robot_lab_utils.robot_description import load_description
    from robot_lab_utils.gait_gazebo_description import shifted_description
    from robot_lab_utils.vendor.hexapod_kinematics import urdf_to_kdl_tree
    folder=ROOT/'src/robot_lab_robots'/robot
    description=load_description(folder/'urdf'/(robot+'.urdf'));source=GaitModel(description,folder/'config/gait.yaml')
    native=shifted_description(description,source);tree=urdf_to_kdl_tree(URDF.from_xml_string(native))
    original=ET.fromstring(description);shifted=ET.fromstring(native)
    for index,name in enumerate(source.names):
        before=original.find("joint[@name='%s']/limit"%name);after=shifted.find("joint[@name='%s']/limit"%name)
        assert after.get('effort')==before.get('effort')
        assert float(after.get('lower'))+source.initial[index]==pytest.approx(float(before.get('lower')))
    for leg in source.legs:
        chain=tree.getChain(leg['root'],leg['foot']);solver=kdl.ChainFkSolverPos_recursive(chain)
        offset=np.array(leg['initial']);source_q=offset+np.array([.01,-.02,.03])
        frame=kdl.Frame();assert solver.JntToCart(source.array(source_q-offset),frame)==0
        np.testing.assert_allclose([frame.p.x(),frame.p.y(),frame.p.z()],source.position(leg['id'],source_q),atol=1e-10)


def test_fortress_description_produces_force_servos_and_complete_contacts(tmp_path):
    import shutil
    if not shutil.which('ign'):pytest.skip('Fortress SDF conversion not installed')
    from robot_lab_utils.gait_model import GaitModel
    from robot_lab_utils.robot_description import load_description
    from robot_lab_utils.gait_gazebo_description import prepare
    folder=ROOT/'src/robot_lab_robots/lab_octopod'
    description=load_description(folder/'urdf/lab_octopod.urdf')
    model=GaitModel(description,folder/'config/gait.yaml')
    (tmp_path/'robot.urdf').write_text(description)
    result=prepare(description,model,ROOT/'src/robot_lab_maps/maps/gait_low_relief/worlds/gait_low_relief.world',tmp_path,ROOT/'src/robot_lab_robots',True,'/actual-sdk-plugin.so')
    native=ET.parse(result['model']);controllers=[p for p in native.findall('.//plugin') if p.get('name','').endswith('::JointPositionController')]
    assert len(controllers)==len(model.names)
    assert all(float(p.findtext('cmd_max'))<=8. for p in controllers)
    frame=native.find(".//plugin[@name='robot_lab::GaitContactFrame']")
    assert {e.get('name') for e in frame.findall('group')}=={leg['id'] for leg in model.legs}|{'body'}
    assert len(native.findall('.//sensor'))==3
    assert ET.parse(result['world']).find('.//heightmap/uri').text.startswith('/')


def test_bullet_source_mass_and_actual_support_survive_ray_filtering(tmp_path):
    p=pytest.importorskip('pybullet')
    sys.path.insert(0,str(ROOT/'src/robot_lab_pybullet/python'))
    from robot_lab_pybullet.gait_plant import GaitPlant
    from robot_lab_utils.gait_model import GaitModel
    from robot_lab_utils.robot_description import load_description
    folder=ROOT/'src/robot_lab_robots/lab_octopod'
    description=load_description(folder/'urdf/lab_octopod.urdf');model=GaitModel(description,folder/'config/gait.yaml')
    path=tmp_path/'robot.urdf';path.write_text(description)
    connection=p.connect(p.DIRECT)
    try:
        p.setTimeStep(.001);p.setGravity(0.,0.,-9.81)
        plane=p.createCollisionShape(p.GEOM_PLANE);p.createMultiBody(baseMass=0.,baseCollisionShapeIndex=plane)
        robot=p.loadURDF(str(path),[0.,0.,.2],flags=p.URDF_USE_INERTIA_FROM_FILE)
        probes={leg['id']:dict(root=leg['root'],foot=leg['foot'],home=model.home[leg['id']].tolist()) for leg in model.legs}
        plant=GaitPlant(p,robot,description,model.pd_config(),probes);plant.seed();plant.align(.1)
        source_mass=sum(float(v.get('value')) for v in ET.fromstring(description).findall('link/inertial/mass'))
        assert sum(p.getDynamicsInfo(robot,i)[0] for i in range(-1,p.getNumJoints(robot)))==pytest.approx(source_mass)
        for step in range(1000):plant.step(step*.001);p.stepSimulation()
        forces=plant.measure(1.)['normal_forces_N']
        assert sum(forces.values())==pytest.approx(source_mass*9.81,rel=.03)
        assert forces['body']==0.
        assert plant.ray(np.array([0.,0.,1.]),2.)==pytest.approx(1.)
    finally:p.disconnect(connection)
