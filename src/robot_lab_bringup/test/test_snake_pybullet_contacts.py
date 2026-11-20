"""R10.7: the passive-wheel snake's rolling contacts on the PyBullet gait plant (2026-11-20)."""
import json
from pathlib import Path
import sys
import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/robot_lab_utils'),str(ROOT/'src/robot_lab_pybullet/python')]
pybullet=pytest.importorskip('pybullet')


def test_averaged_wheel_contacts_give_the_odometry_three_supports(tmp_path):
    import pybullet_data
    from robot_lab_utils.gait_model import load_model
    from robot_lab_utils.robot_description import load_description
    from robot_lab_pybullet.gait_plant import GaitPlant
    folder=ROOT/'src/robot_lab_robots/lab_snake'
    description=load_description(folder/'urdf/lab_snake.urdf');model=load_model(description,folder/'config/gait.yaml')
    contacts={leg['id']:leg['contact_geoms'] for leg in model.legs};contacts['body']=model.body_geoms
    urdf=tmp_path/'snake.urdf';urdf.write_text(description)
    client=pybullet.connect(pybullet.DIRECT)  # the only client, so module calls use it
    try:
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0,0,-9.81);pybullet.setTimeStep(.001)
        pybullet.loadURDF('plane.urdf')
        robot=pybullet.loadURDF(str(urdf),[0,0,float(model.config['standing_height'])+.005])
        plant=GaitPlant(pybullet,robot,description,json.dumps(model.pd_config()),{},json.dumps(contacts))
        assert set(plant.groups)=={leg['id'] for leg in model.legs}
        wheel=next(index for index,group in plant.foot_groups.items() if group!='body')
        assert pybullet.getDynamicsInfo(robot,wheel)[1]==1.
        for _ in range(1500):pybullet.stepSimulation();plant.sample()
        # A long first window covers only its last 50 substeps, not the drop.
        assert plant.window_steps<=50
        plant.measure(0.)
        supports=[]
        for _ in range(10):
            for _ in range(20):pybullet.stepSimulation();plant.sample()
            instantaneous=sum(value>.02 for name,value in plant.forces().items() if name!='body')
            forces=plant.measure(1.5)['normal_forces_N']
            supports.append((sum(value>.02 for name,value in forces.items() if name!='body'),instantaneous))
            assert forces['body']<1.
        # Every averaged window carries the odometry's three supports.
        assert min(averaged for averaged,_ in supports)>=3, supports
    finally:pybullet.disconnect(client)
