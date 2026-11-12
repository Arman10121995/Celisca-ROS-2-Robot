"""Vendor contact-frame and mounted-camera contracts; not mission qualification."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/robot_lab_utils'))
from robot_lab_utils.source_ground import ground_sensor_kit, summit_fortress_contacts
from robot_lab_utils.sensor_config import mounted_rgbd_sensor
from robot_lab_utils.robot_description import resolve_relative_assets


def chassis():
    root=ET.fromstring('''<robot name="source"><link name="base_footprint"/><link name="base_link"/>
      <joint name="base_footprint_joint" type="fixed"><parent link="base_footprint"/>
      <child link="base_link"/><origin xyz="0 0 .127" rpy="0 0 0"/></joint></robot>''')
    for name,vector in [('fl','1 -1 0'),('fr','1 1 0'),('rl','1 1 0'),('rr','1 -1 0')]:
        block=ET.SubElement(root,'gazebo',reference=name)
        direction=ET.SubElement(block,'fdir1',{'{http://gazebosim.org/schema}expressed_in':'base_footprint'})
        direction.text=vector
    return root


def test_friction_direction_stays_in_the_chassis_frame_as_wheels_rotate():
    root=chassis();ports=summit_fortress_contacts(root)
    assert len(ports)==4
    assert [p['source_direction'] for p in ports]==['1 -1 0','1 1 0','1 1 0','1 -1 0']
    text=ET.tostring(root,encoding='unicode')
    assert text.count('ignition:expressed_in="base_footprint"')==4
    assert 'gz:expressed_in' not in text
    assert root.find('joint/origin').get('xyz')=='0 0 .127'


@pytest.mark.parametrize('prefix,uri',[('ignition','http://ignitionrobotics.org/schema'),
                                      ('gz','http://gazebosim.org/schema')])
def test_description_asset_resolution_preserves_native_sdk_extension_spelling(tmp_path,prefix,uri):
    source=f'<robot name="native" xmlns:{prefix}="{uri}"><gazebo><fdir1 {prefix}:expressed_in="base_footprint">1 -1 0</fdir1></gazebo></robot>'
    result=resolve_relative_assets(source,tmp_path/'robot.urdf')
    assert prefix+':expressed_in="base_footprint"' in result
    assert 'ns0:' not in result


def test_unreviewed_rotated_friction_frame_is_not_silently_relabelled():
    root=chassis();root.find('joint/origin').set('rpy','0 0 1.57')
    with pytest.raises(ValueError,match='zero relative'):summit_fortress_contacts(root)


def test_massless_sensor_frames_bridge_the_physical_hull_sensor():
    root=chassis();settings=ground_sensor_kit(root)
    assert mounted_rgbd_sensor(ET.tostring(root,encoding='unicode'))==('base_footprint','rgbd_camera')
    assert settings['camera_link_name']=='oakd_rgb_camera_frame'
    assert list(map(float,root.find("gazebo[@reference='base_footprint']/sensor[@name='rgbd_camera']/pose").text.split()))==pytest.approx([.3585,0,.37762,0,0,0])
    assert root.find("joint[@name='oakd_rgb_camera_frame_lab_mount']/origin").get('xyz')=='.3585 0 .25062'
    assert not root.findall('link/inertial')  # The declared lab kit adds no chassis mass.
