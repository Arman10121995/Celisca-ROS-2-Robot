#!/usr/bin/env python3
"""An explicit bounded-actuator derivative of the preserved tracked source."""
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
JOINTS=['front_left_flipper_j','rear_left_flipper_j','front_right_flipper_j','rear_right_flipper_j']


def generate():
    source=ROOT/'src/robot_lab_robots/lab_tracked';target=ROOT/'src/robot_lab_robots/lab_tracked_articulated'
    target.mkdir(parents=True,exist_ok=True)
    sdf=ET.parse(source/'model.sdf');model=sdf.getroot().find('model')
    urdf=ET.parse(source/'robot.urdf');urdf.getroot().set('name','lab_tracked_articulated')
    for name in JOINTS:
        limit=model.find("joint[@name='%s']/axis/limit"%name)
        for key,value in dict(lower='-.8',upper='.8',effort='4',velocity='.3').items():
            item=limit.find(key)
            if item is None:item=ET.SubElement(limit,key)
            item.text=value
        joint=urdf.getroot().find("joint[@name='%s']/limit"%name)
        joint.attrib.update(lower='-.8',upper='.8',effort='4',velocity='.3')
        controller=ET.SubElement(model,'plugin',name='ignition::gazebo::systems::JointPositionController',filename='ignition-gazebo-joint-position-controller-system')
        for key,value in dict(joint_name=name,topic='/tracked/flippers/'+name+'/target',p_gain='8',i_gain='1.2',i_min='-.75',i_max='.75',d_gain='.4',
                cmd_min='-4',cmd_max='4',use_velocity_commands='false',initial_position='0').items():ET.SubElement(controller,key).text=value
    for name,tree in [('model.sdf',sdf),('robot.urdf',urdf)]:
        ET.indent(tree);tree.write(target/name,encoding='utf-8',xml_declaration=True)
    shutil.copy2(source/'LICENSE',target/'LICENSE')
    (target/'SOURCE.json').write_text(json.dumps(dict(parent='robot_lab_robots/lab_tracked',
        parent_sdf_sha256=hashlib.sha256((source/'model.sdf').read_bytes()).hexdigest(),license='Apache-2.0',
        adaptations=['Four explicitly unlocked ±0.8 rad flipper joints','Native force PID capped at 4 Nm, never ideal joint velocity',
                     'ROS target slew limited to 0.2 rad/s; source geometry, masses and native belts retained'],
        scope='Derived actuator variant; original zero-limit source remains separate; terrain acceptance pending'),indent=2)+'\n')


if __name__=='__main__':generate()
