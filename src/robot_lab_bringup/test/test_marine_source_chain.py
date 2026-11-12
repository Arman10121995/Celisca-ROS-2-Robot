"""Allocation must preserve a vendor's nested zero-held azimuth lever arms."""
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/robot_lab_utils'))
from robot_lab_utils.marine_model import SurfaceModel


def description():
    root=ET.fromstring('''<robot name="nested">
      <link name="base_link"><inertial><mass value="10"/></inertial>
        <collision><geometry><cylinder radius="0.2" length="2"/></geometry></collision></link>
    </robot>''')
    for side,y in [('left',1),('right',-1)]:
        for name in (side+'_engine',side+'_propeller'):ET.SubElement(root,'link',name=name)
        mount=ET.SubElement(root,'joint',name=side+'_steer',type='revolute')
        ET.SubElement(mount,'parent',link='base_link');ET.SubElement(mount,'child',link=side+'_engine')
        ET.SubElement(mount,'origin',xyz=f'-2 {y} .3',rpy='0 0 0')
        joint=ET.SubElement(root,'joint',name=side+'_prop',type='continuous')
        ET.SubElement(joint,'parent',link=side+'_engine');ET.SubElement(joint,'child',link=side+'_propeller')
        ET.SubElement(joint,'origin',xyz='-.2 0 -.5',rpy='0 0 0');ET.SubElement(joint,'axis',xyz='1 0 0')
        for kind,values in [('JointPositionController',dict(joint_name=side+'_steer',initial_position='0')),
            ('Thruster',dict(joint_name=side+'_prop',min_thrust_cmd='-20',max_thrust_cmd='30'))]:
            plugin=ET.SubElement(ET.SubElement(root,'gazebo'),'plugin',name='ignition::gazebo::systems::'+kind)
            for key,text in values.items():ET.SubElement(plugin,key).text=text
    return root


def test_nested_propellers_reconstruct_surge_and_yaw_with_source_lever_arms():
    model=SurfaceModel(ET.tostring(description(),encoding='unicode'))
    np.testing.assert_allclose(model.matrix[[0,4,5]],[[1,1],[-.2,-.2],[-1,1]],atol=1e-12)
    forces,scale=model.allocate([10,0,0,0,0,4])
    np.testing.assert_allclose(forces,[3,7],atol=1e-12)
    assert scale==1.
    assert model.displacement==pytest.approx(math.pi*.2**2*2)


@pytest.mark.parametrize('angle',[None,'0.2'])
def test_unknown_or_nonzero_steering_refuses_a_fixed_allocation(angle):
    root=description()
    hold=root.find("gazebo/plugin[@name='ignition::gazebo::systems::JointPositionController']")
    if angle is None:hold.set('name','other')
    else:hold.find('initial_position').text=angle
    with pytest.raises(ValueError,match='zero-angle'):SurfaceModel(ET.tostring(root,encoding='unicode'))


def test_velocity_controlled_propellers_cannot_exceed_source_joint_speed():
    root=description()
    for joint in root.findall("joint[@type='continuous']"):
        ET.SubElement(joint,'limit',velocity='10')
    for plugin in root.findall('gazebo/plugin'):
        if plugin.get('name','').endswith('::Thruster'):
            for key,text in dict(velocity_control='true',thrust_coefficient='.01',
                                 propeller_diameter='.2',fluid_density='1000').items():
                ET.SubElement(plugin,key).text=text
    model=SurfaceModel(ET.tostring(root,encoding='unicode'))
    np.testing.assert_allclose(model.bounds,[[-1.6,1.6],[-1.6,1.6]])
