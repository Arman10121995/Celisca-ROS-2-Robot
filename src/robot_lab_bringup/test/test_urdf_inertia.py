"""Welded root inertia survives empty URDF frame roots and fixed sensor mounts."""
import math

import pytest
from robot_lab_utils.urdf_inertia import fixed_root_inertial
from robot_lab_utils.urdf_inertia import bullet_frame_inertials
import xml.etree.ElementTree as ET


def test_bullet_empty_frames_retain_names_and_real_source_inertias():
    source='''<robot name="test"><link name="root"/><link name="body">
      <inertial><mass value="2"/><inertia ixx=".1" iyy=".2" izz=".3"/></inertial></link>
      <link name="imu"/><joint name="mount" type="fixed"><parent link="root"/><child link="body"/></joint>
      <joint name="sensor" type="fixed"><parent link="body"/><child link="imu"/></joint></robot>'''
    before=ET.fromstring(source);after=ET.fromstring(bullet_frame_inertials(source))
    assert [e.get('name') for e in after.findall('link')]==['root','body','imu']
    assert float(after.find("link[@name='root']/inertial/mass").get('value'))==1e-6
    assert float(after.find("link[@name='imu']/inertial/mass").get('value'))==0.
    assert ET.canonicalize(ET.tostring(before.find("link[@name='body']/inertial"),encoding='unicode'),strip_text=True)==ET.canonicalize(ET.tostring(after.find("link[@name='body']/inertial"),encoding='unicode'),strip_text=True)


def test_bullet_missing_geometry_inertia_is_not_silently_invented():
    with pytest.raises(ValueError,match='geometry'):
        bullet_frame_inertials('<robot><link name="body"><collision/></link></robot>')


def test_rotated_fixed_child_and_parallel_axis_preserve_root_component_only():
    source=f'''<robot name="test"><link name="footprint"/><link name="base">
      <inertial><mass value="1"/><inertia ixx=".1" iyy=".1" izz=".1"/></inertial></link>
      <joint name="root_mount" type="fixed"><parent link="footprint"/><child link="base"/></joint>
      <link name="sensor"><inertial><mass value="2"/><inertia ixx=".2" iyy=".3" izz=".4"/></inertial></link>
      <joint name="sensor_mount" type="fixed"><parent link="base"/><child link="sensor"/><origin xyz="1 0 0" rpy="0 0 {math.pi/2}"/></joint>
      <link name="wheel"><inertial><mass value="100"/><inertia ixx="1" iyy="1" izz="1"/></inertial></link>
      <joint name="drive" type="continuous"><parent link="base"/><child link="wheel"/></joint></robot>'''
    result=fixed_root_inertial(source)
    assert float(result['mass'])==pytest.approx(3.)
    assert list(map(float,result['pos'].split()))==pytest.approx([2/3,0.,0.])
    assert list(map(float,result['fullinertia'].split()))==pytest.approx([.4,.3+2/3,.5+2/3,0.,0.,0.])
