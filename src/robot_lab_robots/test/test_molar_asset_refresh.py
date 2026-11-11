"""R11.1 installed/source model contracts; no hardware or ROS command publishers."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT=Path(__file__).resolve().parents[3]

def test_robot_frames_and_rest_relative_kinematics():
    from ament_index_python.packages import get_package_share_directory
    from robot_lab_utils.sensor_config import sensor_parameters
    profiles=yaml.safe_load((ROOT/'src/robot_lab_robots/config/robots.yaml').read_text())['robots']
    share=Path(get_package_share_directory('robot_lab_robots'))
    for name in ('probot_l','molar_hri'):
        profile=profiles[name]
        path=share/profile['xacro'];robot=ET.parse(path).getroot()
        assert path.read_bytes()==(ROOT/'src/robot_lab_robots'/profile['xacro']).read_bytes()
        spec=profile['articulation']['joints']['conveyor_lift_joint']
        assert [spec['lower'],spec['upper']]==pytest.approx([-.310,.350])
        joint=robot.find("joint[@name='conveyor_lift_joint']")
        assert float(joint.find('limit').get('lower'))==pytest.approx(-.315)
        assert float(joint.find('origin').get('xyz').split()[2])+spec['lower']==pytest.approx(.440)
        assert robot.find("link[@name='telescope_stage2']/visual[@name='Robot_Carriage_TransferPlate']") is not None
        assert robot.find("link[@name='conveyor_carriage']/visual[@name='Robot_Carriage_TransferPlate']") is None
        params=sensor_parameters(profile['sensor_config'],ET.tostring(robot,encoding='unicode'))
        assert params['laser_link_name']=='laser_link'
        scanner=robot.find("joint[@name='laser_link_mount']/origin")
        assert list(map(float,scanner.get('xyz').split()))==pytest.approx([.340,0,.130])
        if name=='molar_hri':
            camera=robot.find("joint[@name='camera_link_mount']/origin")
            assert list(map(float,camera.get('xyz').split()))==pytest.approx([.2704,0,1.649])
            optical=robot.find("joint[@name='camera_optical_link_mount']/origin")
            r,p,y=map(float,optical.get('rpy').split())
            # Optical +Z points along body +X; +X right / +Y down convention.
            z=[math.cos(y)*math.sin(p)*math.cos(r)+math.sin(y)*math.sin(r),
               math.sin(y)*math.sin(p)*math.cos(r)-math.cos(y)*math.sin(r),math.cos(p)*math.cos(r)]
            assert z==pytest.approx([1,0,0],abs=1e-8)
        else:
            assert 'camera_link_name' not in params

def test_native_detailed_floors_preserve_spawns_and_add_chargers():
    from ament_index_python.packages import get_package_share_directory
    share=Path(get_package_share_directory('robot_lab_maps'))
    maps=yaml.safe_load((ROOT/'src/robot_lab_bringup/config/sim_maps.yaml').read_text())['maps']
    for floor,count in ((1,2),(2,1)):
        name=f'celisca_floor_{floor}_detailed'
        provenance=json.loads((share/'maps'/name/'PROVENANCE.json').read_text())
        assert provenance['source_manifest']['detailParts']==96+8*count
        assert provenance['projection']['spawn']==maps[name]['spawn']
        assert provenance['source_manifest']['sourceGeometryAndTransformsUnchanged']
        assert all(provenance['source_manifest']['glbVerification'].values())
        assert Path(provenance['visual']['path']).exists()


def test_native_pybullet_minimum_transfer_and_total_tip_travel(tmp_path):
    bullet=pytest.importorskip('pybullet')
    from ament_index_python.packages import get_package_share_directory
    share=Path(get_package_share_directory('robot_lab_robots'))
    source=share/'molar_hri/urdf/molar_hri.urdf.xacro'
    # Exercise the actual installed model and its real geometry, not a test stub.
    text=source.read_text().replace('package://robot_lab_robots/',str(share)+'/')
    path=tmp_path/'molar_minimum.urdf';path.write_text(text)
    client=bullet.connect(bullet.DIRECT)
    try:
        body=bullet.loadURDF(str(path),useFixedBase=True,physicsClientId=client)
        ids={bullet.getJointInfo(body,i,physicsClientId=client)[1].decode():i for i in range(bullet.getNumJoints(body,physicsClientId=client))}
        def pose(joint):
            return bullet.getLinkState(body,ids[joint],computeForwardKinematics=True,physicsClientId=client)[4]
        bullet.resetJointState(body,ids['conveyor_lift_joint'],-.310,physicsClientId=client)
        assert pose('conveyor_lift_joint')[2]==pytest.approx(.440,abs=1e-6)
        before=pose('telescope_stage2_joint')
        for joint in ('telescope_stage1_joint','telescope_stage2_joint'):
            bullet.resetJointState(body,ids[joint],.140,physicsClientId=client)
        after=pose('telescope_stage2_joint')
        assert after[0]-before[0]==pytest.approx(.280,abs=1e-6)
        assert after[2]==pytest.approx(.440,abs=1e-6)
        bullet.resetJointState(body,ids['conveyor_lift_joint'],.350,physicsClientId=client)
        assert pose('conveyor_lift_joint')[2]==pytest.approx(1.100,abs=1e-6)
    finally:
        bullet.disconnect(client)
