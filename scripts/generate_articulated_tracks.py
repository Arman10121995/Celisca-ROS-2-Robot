#!/usr/bin/env python3
"""Explicit bounded-actuator derivatives of the preserved tracked source.

``lab_tracked_articulated``: four unlocked flippers, native force PID capped
at 4 N m. ``lab_tracked_strong_flippers`` (R10.7): the same with a 40 N m
cap and PID gains scaled by the same factor, for step climbing; the 4 N m
flippers were back-driven by the hull at a 0.18 m step (2026-11-14).
"""
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
JOINTS=['front_left_flipper_j','rear_left_flipper_j','front_right_flipper_j','rear_right_flipper_j']


VARIANTS={'lab_tracked_articulated':4.,'lab_tracked_strong_flippers':40.}


def generate(name='lab_tracked_articulated'):
    effort=VARIANTS[name];scale=effort/4.
    source=ROOT/'src/robot_lab_robots/lab_tracked';target=ROOT/'src/robot_lab_robots'/name
    target.mkdir(parents=True,exist_ok=True)
    sdf=ET.parse(source/'model.sdf');model=sdf.getroot().find('model')
    urdf=ET.parse(source/'robot.urdf');urdf.getroot().set('name',name)
    def number(value):return ('%g'%value).replace('0.','.',1) if abs(value)<1 else '%g'%value   # '.75' as authored
    for joint_name in JOINTS:
        limit=model.find("joint[@name='%s']/axis/limit"%joint_name)
        for key,value in dict(lower='-.8',upper='.8',effort=number(effort),velocity='.3').items():
            item=limit.find(key)
            if item is None:item=ET.SubElement(limit,key)
            item.text=value
        joint=urdf.getroot().find("joint[@name='%s']/limit"%joint_name)
        joint.attrib.update(lower='-.8',upper='.8',effort=number(effort),velocity='.3')
        controller=ET.SubElement(model,'plugin',name='ignition::gazebo::systems::JointPositionController',filename='ignition-gazebo-joint-position-controller-system')
        for key,value in dict(joint_name=joint_name,topic='/tracked/flippers/'+joint_name+'/target',p_gain=number(8*scale),
                i_gain=number(1.2*scale),i_min=number(-.75*scale),i_max=number(.75*scale),d_gain=number(.4*scale),
                cmd_min=number(-effort),cmd_max=number(effort),use_velocity_commands='false',initial_position='0').items():
            ET.SubElement(controller,key).text=value
    for name,tree in [('model.sdf',sdf),('robot.urdf',urdf)]:
        ET.indent(tree);tree.write(target/name,encoding='utf-8',xml_declaration=True)
    shutil.copy2(source/'LICENSE',target/'LICENSE')
    (target/'SOURCE.json').write_text(json.dumps(dict(parent='robot_lab_robots/lab_tracked',
        parent_sdf_sha256=hashlib.sha256((source/'model.sdf').read_bytes()).hexdigest(),license='Apache-2.0',
        adaptations=['Four explicitly unlocked ±0.8 rad flipper joints','Native force PID capped at %g Nm, never ideal joint velocity'%effort,
                     'ROS target slew limited to 0.2 rad/s; source geometry, masses and native belts retained'],
        scope='Derived actuator variant; original zero-limit source remains separate; terrain acceptance pending'),indent=2)+'\n')


if __name__=='__main__':
    for variant in VARIANTS:generate(variant)
