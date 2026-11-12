"""Source-coordinate conversion contracts; small fixtures are not robot missions."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/robot_lab_utils'))
from robot_lab_utils.bluerov_source import heavy_description
from robot_lab_utils.marine_model import rotation


def source(tmp_path):
    (tmp_path/'body.dae').write_text('fixture resource, not a renderable mesh')
    root=ET.fromstring('''<sdf version="1.6"><model name="bluerov2_heavy"><link name="base_link">
      <inertial><pose>0 0 .011 0 0 0</pose><mass>13</mass><inertia><ixx>.26</ixx><iyy>.23</iyy><izz>.37</izz></inertia></inertial>
      <visual name="hull"><geometry><mesh><uri>model://bluerov2_heavy/body.dae</uri><scale>.025 .025 .025</scale></mesh></geometry></visual>
      <collision name="volume"><pose>0 0 .06 0 0 0</pose><geometry><box><size>.457 .575 .05</size></box></geometry></collision>
    </link><plugin name="ArduPilotPlugin"/></model></sdf>''')
    model=root.find('model');firmware=model.find('plugin')
    for i in range(1,9):
        link=ET.SubElement(model,'link',name='thruster'+str(i))
        ET.SubElement(link,'pose').text='.14 -.092 0 -1.570796326795 1.570796326795 -.785398163397'
        link.append(ET.fromstring('<inertial><mass>.002</mass><inertia><ixx>.001</ixx><iyy>.001</iyy><izz>.001</izz></inertia></inertial>'))
        joint=ET.SubElement(model,'joint',name='thruster'+str(i)+'_joint',type='revolute')
        ET.SubElement(joint,'parent').text='base_link';ET.SubElement(joint,'child').text=link.get('name')
        ET.SubElement(ET.SubElement(joint,'axis'),'xyz').text='0 0 -1'
        control=ET.SubElement(firmware,'control')
        for key,text in dict(jointName=joint.get('name'),type='COMMAND',offset='-.5',multiplier='100').items():
            ET.SubElement(control,key).text=text
    path=tmp_path/'model.sdf';ET.ElementTree(root).write(path)
    return path


def test_source_mesh_scale_volume_and_joint_axis_frame_are_preserved(tmp_path):
    root,contract=heavy_description(source(tmp_path),'candidate')
    assert root.find('link/visual/geometry/mesh').get('scale')=='.025 .025 .025'
    assert root.find('link/collision/geometry/box').get('size')=='.457 .575 .05'
    assert root.find('link/inertial/origin').get('xyz')=='0 0 .011'
    joint=root.find('joint');pose=joint.find('origin')
    assert pose.get('xyz')=='.14 -.092 0'
    direction=rotation(list(map(float,pose.get('rpy').split())))@np.array([0.,0.,-1.])
    np.testing.assert_allclose(direction,[-2**-.5,-2**-.5,0.],atol=1e-10)
    assert contract['force_bounds_N']['thruster1_joint']==[-50.,50.]
    assert not root.findall("gazebo/plugin[@name='ArduPilotPlugin']")


def test_unreviewed_relative_pose_frame_is_rejected(tmp_path):
    path=source(tmp_path);document=ET.parse(path)
    document.getroot().find("model/link[@name='thruster1']/pose").set('relative_to','other')
    document.write(path)
    with pytest.raises(ValueError,match='pose frame'):heavy_description(path,'candidate')
