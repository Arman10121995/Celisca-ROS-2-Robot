"""Welded root inertia survives empty URDF frame roots and fixed sensor mounts."""
import math

import pytest
from robot_lab_utils.urdf_inertia import fixed_root_inertial


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
