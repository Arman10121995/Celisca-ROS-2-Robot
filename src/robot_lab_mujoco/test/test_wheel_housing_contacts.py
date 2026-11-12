"""Keep convex wheel housings from blocking their driven subtrees only."""
import xml.etree.ElementTree as ET

import pytest

mujoco=pytest.importorskip('mujoco')
pytest.importorskip('rclpy')
from robot_lab_mujoco.mujoco_spawner import _exclude_wheel_housing_contacts


def test_explicit_housing_exclusion_preserves_floor_contacts_and_geometry():
    text='''<mujoco><worldbody><body name="chassis" pos="0 0 .09"><freejoint/>
      <geom type="box" size=".2 .2 .1"/><geom name="visual" type="sphere" size=".3" contype="0" conaffinity="0"/>
      <body name="carrier"><joint name="spring" type="slide" axis="0 0 1"/>
        <inertial pos="0 0 0" mass=".005" diaginertia=".000001 .000001 .000001"/>
        <body name="hub" pos=".15 0 0"><joint name="drive" axis="0 1 0"/>
          <geom type="sphere" size=".05"/>
          <body name="roller" pos=".02 0 0"><joint name="passive" axis="1 0 0"/><geom type="sphere" size=".03"/></body>
        </body>
      </body></body></worldbody></mujoco>'''
    out=_exclude_wheel_housing_contacts(text,['drive'])
    before,after=ET.fromstring(text),ET.fromstring(out)
    assert ET.tostring(before.find('worldbody'))==ET.tostring(after.find('worldbody'))
    pairs={frozenset(e.attrib.values()) for e in after.findall('contact/exclude')}
    assert pairs=={frozenset(['chassis','hub']),frozenset(['chassis','roller'])}
    ET.SubElement(after.find('worldbody'),'geom',name='floor',type='plane',size='2 2 .1')
    model=mujoco.MjModel.from_xml_string(ET.tostring(after,encoding='unicode'))
    data=mujoco.MjData(model);mujoco.mj_forward(model,data)
    assert data.ncon>0
    assert all(0 in (int(model.geom_bodyid[c.geom1]),int(model.geom_bodyid[c.geom2])) for c in data.contact[:data.ncon])
