"""Source/motor/contact contracts, separate from walking mission evidence."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import json
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/robot_lab_utils'),str(ROOT/'src/robot_lab_mujoco/python')]
from robot_lab_utils.gait_model import GaitModel
from robot_lab_utils.robot_description import load_description


@pytest.fixture(params=['hexapod_ros2','lab_octopod','lab_centipede'])
def source(request):
    folder=ROOT/'src/robot_lab_robots'/request.param
    description=load_description(folder/'urdf'/(request.param+'.urdf'))
    return request.param,description,GaitModel(description,folder/'config/gait.yaml')


def test_complete_leg_chains_and_initial_ik(source):
    name,description,model=source
    expected={'hexapod_ros2':6,'lab_octopod':8,'lab_centipede':12}[name]
    assert len(model.legs)==expected
    assert len(set(j for leg in model.legs for j in leg['joints']))==3*expected
    xml=ET.fromstring(description)
    for leg in model.legs:
        assert xml.find("link[@name='%s']/collision/geometry/sphere"%leg['foot']) is not None
        target=model.home[leg['id']]+np.array([.002,0.,.003])
        q,error=model.solve(leg['id'],target,leg['initial'])
        assert error<.0002
        assert (q>=model.lower[model.indices[leg['id']]]).all()
        assert (q<=model.upper[model.indices[leg['id']]]).all()


def test_compiled_plant_preserves_source_link_masses(source):
    import mujoco
    from robot_lab_mujoco.mujoco_spawner import _build_mjcf_from_urdf,_FALLBACK_MJCF
    name,description,model=source
    xml=_build_mjcf_from_urdf(description,{'robot_lab_robots':str(ROOT/'src/robot_lab_robots')},robot_name=name)
    assert xml!=_FALLBACK_MJCF
    plant=mujoco.MjModel.from_xml_string(xml)
    source_mass=sum(float(link.find('inertial/mass').get('value')) for link in ET.fromstring(description).findall('link') if link.find('inertial/mass') is not None)
    # MjSpec.to_xml serializes child-body masses to six significant digits.
    # Account for that known round-trip resolution, not a changed motor/body
    # design. This still rejects the massless-root 11.31 vs 4.64 kg failure.
    tolerance=max(2e-7,source_mass*5e-6) if name=='hexapod_ros2' else 2e-7
    assert float(plant.body_mass.sum())==pytest.approx(source_mass,abs=tolerance)
    for joint,limit in zip(model.names,model.torque):
        source_limit=ET.fromstring(description).find("joint[@name='%s']/limit"%joint)
        assert 0<limit<=float(source_limit.get('effort'))


def test_foot_feedback_measures_physics_not_gait_phase():
    import mujoco
    from robot_lab_mujoco.gait_contacts import GaitContacts
    plant=mujoco.MjModel.from_xml_string('''<mujoco><option timestep=".001"/><worldbody>
      <geom type="plane" size="2 2 .1"/>
      <body pos="0 0 .021"><freejoint/>
        <geom name="foot" type="sphere" size=".02" mass="1"/>
      </body></worldbody></mujoco>''')
    data=mujoco.MjData(plant);sensor=GaitContacts(plant,json.dumps({'foot':['foot']}))
    mujoco.mj_forward(plant,data)
    assert sensor.measure(plant,data)['normal_forces_N']['foot']==0.
    for _ in range(1000):mujoco.mj_step(plant,data)
    assert sensor.measure(plant,data)['normal_forces_N']['foot']==pytest.approx(9.81,abs=.03)


def test_initial_floor_placement_excludes_robot_surfaces():
    import mujoco
    from robot_lab_mujoco.gait_contacts import GaitContacts
    plant=mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <geom type="box" size="2 2 .1" pos="0 0 .1"/>
      <body pos="0 0 .5"><freejoint/>
        <geom name="body" type="box" size=".2 .2 .05" mass="1"/>
        <geom name="foot" type="sphere" size=".02" pos="0 0 -.2" mass=".1"/>
      </body></worldbody></mujoco>''')
    data=mujoco.MjData(plant);mujoco.mj_forward(plant,data)
    original=plant.geom_group.copy()
    sensor=GaitContacts(plant,json.dumps({'leg':['foot'],'body':['body']}))
    result=sensor.align_spawn(plant,data,0,.1)
    assert result['floor_min_m']==pytest.approx(.2)
    assert data.qpos[2]==pytest.approx(.422)
    np.testing.assert_array_equal(original,plant.geom_group)
