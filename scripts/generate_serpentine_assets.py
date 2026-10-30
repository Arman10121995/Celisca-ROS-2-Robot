#!/usr/bin/env python3
"""A complete articulated snake with passive wheels and real spine motors.

Passive rolling contacts provide direction-dependent ground response through
joint mechanics. No body-force/pose locomotion shortcut is used.
"""
import json
import xml.etree.ElementTree as ET
import yaml
from generate_marine_assets import ROOT, child, write


def generate():
    count=10;root=ET.Element('robot',name='lab_snake')
    for i in range(count):
        name='base_link' if i==0 else 'segment_'+str(i)
        centre='0 0 0' if i==0 else '-.08 0 0'
        link=child(root,'link',name=name);inertial=child(link,'inertial')
        child(inertial,'origin',xyz=centre,rpy='0 0 0');child(inertial,'mass',value='.24')
        child(inertial,'inertia',ixx='.00018',iyy='.00056',izz='.00064',ixy='0',ixz='0',iyz='0')
        for kind in ('visual','collision'):
            element=child(link,kind,name=name+'_'+kind);child(element,'origin',xyz=centre,rpy='0 0 0')
            child(child(element,'geometry'),'box',size='.14 .072 .045')
            if kind=='visual':child(child(element,'material',name='segment_paint_'+str(i)),'color',rgba='.9 .55 .08 1' if i%2==0 else '.15 .22 .28 1')
        if i:
            joint=child(root,'joint',name='spine_'+str(i),type='revolute')
            child(joint,'parent',link='base_link' if i==1 else 'segment_'+str(i-1));child(joint,'child',link=name)
            child(joint,'origin',xyz='-.08 0 0' if i==1 else '-.16 0 0',rpy='0 0 0');child(joint,'axis',xyz='0 0 1')
            child(joint,'limit',lower='-.8',upper='.8',effort='.8',velocity='2')
            child(joint,'dynamics',damping='.025',friction='.001')
        for side,y in [('left',.05),('right',-.05)]:
            wheel='wheel_'+str(i)+'_'+side
            link=child(root,'link',name=wheel);inertial=child(link,'inertial');child(inertial,'mass',value='.015')
            child(inertial,'inertia',ixx='.0000049',iyy='.0000092',izz='.0000049',ixy='0',ixz='0',iyz='0')
            for kind in ('visual','collision'):
                element=child(link,kind,name=wheel+'_'+kind);child(element,'origin',xyz='0 0 0',rpy='1.57079632679 0 0')
                child(child(element,'geometry'),'cylinder',radius='.035',length='.018')
                if kind=='visual':child(child(element,'material',name='wheel_paint_'+str(i)+side),'color',rgba='.08 .08 .08 1')
            joint=child(root,'joint',name=wheel+'_joint',type='continuous')
            child(joint,'parent',link=name);child(joint,'child',link=wheel)
            child(joint,'origin',xyz=f'{0. if i==0 else -.08} {y} -.02',rpy='0 0 0')
            child(joint,'axis',xyz='0 1 0');child(joint,'limit',effort='.01',velocity='80')
            child(joint,'dynamics',damping='.0003',friction='0')
    target=ROOT/'src/robot_lab_robots/lab_snake';write(root,target/'urdf/lab_snake.urdf')
    config=dict(kind='serpentine',robot_id='lab_snake',segments=count,segment_length=.16,wheel_radius=.035,wheel_half_track=.05,
        standing_height=.055,forward_yaw=0.,physical_leg_count=0,default_gait='serpenoid',cycle_s=2.4,
        max_speed=.08,max_yaw=.1,kp=4.,kd=.15,torque_limit=.8,joint_velocity_limit=.8,wave_amplitude=.38,wave_cycles=1.2)
    (target/'config').mkdir(parents=True,exist_ok=True)
    (target/'config/gait.yaml').write_text(yaml.safe_dump(config))
    (target/'SOURCE.json').write_text(json.dumps(dict(author='Robot Lab contributors',license='Apache-2.0',
        model='First-party ten-segment wheeled snake, nine yaw motors and twenty unpowered wheels',
        calibration='Authored mass, inertia, gains and wave parameters; no hardware calibration',
        locomotion='Native joint torque/contact physics; no body-pose or velocity writes',
        estimator='Contact-checked passive-wheel FK/encoders + native IMU; no body-truth input'),indent=2)+'\n')


if __name__=='__main__':generate()
