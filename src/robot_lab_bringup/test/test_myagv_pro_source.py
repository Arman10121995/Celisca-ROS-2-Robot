"""Source corner/frame and declared simulation-contact contracts, not missions."""
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import pytest

from robot_lab_utils.myagv_pro import ROLES, adapt_source, sensor_kit, lab_compliance
from robot_lab_utils.drive_kinematics import drive_from_config


@pytest.fixture
def original():
    root=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))
    source=root/'external_assets/sources/elephant_myagv_pro/d3851c768d2e887eef7ad79fc4dcb9a928b070ac/agv_pro_description/urdf/agv_pro.urdf'
    if not source.is_file():pytest.skip('Official myAGV Pro source not provisioned on this host')
    return ET.fromstring(subprocess.check_output(['xacro',str(source),'namespace:='],text=True))


def test_physical_corners_and_inertias_are_source_derived(original):
    before={link.get('name'):ET.tostring(link.find('inertial')) for link in original.findall('link') if link.find('inertial') is not None}
    adapt_source(original)
    for name, expected in before.items():
        assert ET.tostring(original.find(f"link[@name='{name}']/inertial"))==expected
    assert ROLES['front_right']=='left_rear_wheel'
    assert ROLES['rear_left']=='right_front_wheel'
    assert sum(float(m.get('value')) for m in original.findall('link/inertial/mass'))==pytest.approx(20.31590622149244)


def test_wrong_source_corner_is_rejected(original):
    joint=original.find("joint[@name='left_rear_wheel_joint']/origin")
    joint.set('xyz','-.17128 -.1799 .052')
    with pytest.raises(ValueError,match='corner contract'):
        adapt_source(original)


def test_contact_direction_frame_and_source_sensor_mounts(original):
    adapt_source(original)
    cfg=sensor_kit(original)
    directions=original.findall('gazebo/collision/surface/friction/ode/fdir1')
    assert len(directions)==4
    assert all(d.get('{http://ignitionrobotics.org/schema}expressed_in')=='base_footprint' for d in directions)
    camera=original.find("gazebo/sensor[@name='rgbd_camera']/pose")
    assert list(map(float,camera.text.split()))==pytest.approx([.23191,0.,.16928,0.,0.,0.])
    assert cfg['laser_link_name']=='laser_link'
    for name in ROLES.values():
        limit=original.find(f"joint[@name='{name}_joint']/limit")
        assert float(limit.get('effort'))==5.
        assert float(limit.get('velocity'))==8.


def test_sdformat_retains_every_direction_and_sensor_system(original, tmp_path):
    import re
    import shutil
    if not shutil.which('ign'):pytest.skip('Fortress converter not installed')
    adapt_source(original);sensor_kit(original);lab_compliance(original)
    model=tmp_path/'model.urdf';model.write_text(ET.tostring(original,encoding='unicode'))
    converted=subprocess.check_output(['ign','sdf','-p',str(model)],text=True)
    # sdformat prints its namespaced extension literally without the xmlns.
    parsed=ET.fromstring(re.sub(r'(<sdf\b)',r'\1 xmlns:ignition="http://ignitionrobotics.org/schema"',converted,count=1))
    assert len(parsed.findall('.//fdir1'))==4
    for name in ROLES.values():
        link=parsed.find(f"model/link[@name='{name}_link']")
        assert link.find('collision/surface/friction/ode/fdir1') is not None
    root=parsed.find('model')
    assert len(root.findall("link[@name='base_footprint']/sensor"))==3
    assert root.find("plugin[@name='ignition::gazebo::systems::Sensors']") is not None
    assert root.find("plugin[@name='ignition::gazebo::systems::Imu']") is not None
    assert len(root.findall("joint[@type='prismatic']"))==4
    assert all(float(j.findtext('axis/dynamics/spring_stiffness'))==10000.
               for j in root.findall("joint[@type='prismatic']"))


def test_added_compliance_preserves_source_wheel_rest_frames(original):
    before={n:original.find(f"joint[@name='{n}_joint']/origin").get('xyz') for n in ROLES.values()}
    protected=adapt_source(original)
    result=lab_compliance(original)
    assert len(result['joints'])==4
    for name in ROLES.values():
        parent=original.find(f"joint[@name='{name}_joint']/parent").get('link')
        carrier=original.find(f"joint[@name='{parent}_joint']")
        assert carrier.find('origin').get('xyz')==before[name]
        assert protected[name]['origin']['xyz']==before[name]
        assert original.find(f"joint[@name='{name}_joint']/origin").get('xyz')=='0 0 0'


def test_combined_mecanum_commands_respect_the_wheel_interface_envelope():
    cfg=dict(type='mecanum',front_left_wheel_joint='fl',front_right_wheel_joint='fr',
        rear_left_wheel_joint='rl',rear_right_wheel_joint='rr',wheel_radius=.0762,
        wheelbase=.3431,wheel_separation=.31345248,max_speed=.3,max_angular_speed=.4,max_wheel_speed=8.)
    limited=drive_from_config(cfg)
    targets=limited.targets(.3,.4,vy=.3)
    assert max(abs(v) for v in targets.velocity.values())==pytest.approx(8.)
    vx,vy,yaw=limited.body_twist([targets.velocity[n] for n in limited.wheel_joints])
    assert vy/vx==pytest.approx(1.)
    assert yaw/vx==pytest.approx(.4/.3)
    assert limited._vx==pytest.approx(vx)
