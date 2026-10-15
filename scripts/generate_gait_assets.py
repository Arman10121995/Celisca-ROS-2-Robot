#!/usr/bin/env python3
"""Generate complete legged variants; preserve the pinned MIT hexapod source.

The imported CAD robot, authored octopod and authored segmented robot each
have their own measured geometry, initial pose and actuator/gait contract.
This generator never asserts physical walking or navigation qualification.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
import yaml

ROOT=Path(__file__).resolve().parents[1]
ROBOTS=ROOT/'src/robot_lab_robots'
PIN='b146f84d58123caabfcea8fe2a5e2a353acc33de'


def element(parent,tag,text=None,**attrs):
    item=ET.SubElement(parent,tag,{k:str(v) for k,v in attrs.items()})
    if text is not None:item.text=str(text)
    return item


def save(root,path):
    path.parent.mkdir(parents=True,exist_ok=True);ET.indent(root)
    path.write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True))


def joint(root,name,parent,child,xyz='0 0 0',rpy='0 0 0',axis='0 0 1',kind='revolute',bounds=(-1.,1.),effort=8.):
    item=element(root,'joint',name=name,type=kind)
    element(item,'parent',link=parent);element(item,'child',link=child)
    element(item,'origin',xyz=xyz,rpy=rpy)
    if kind!='fixed':
        element(item,'axis',xyz=axis)
        element(item,'limit',lower=bounds[0],upper=bounds[1],velocity=4.,effort=effort)
        element(item,'dynamics',damping=.02,friction=.01)


def link(root,name,mass,shape,size,origin='0 0 0',color='.2 .45 .65 1'):
    body=element(root,'link',name=name)
    inertial=element(body,'inertial');element(inertial,'origin',xyz=origin)
    element(inertial,'mass',value=mass)
    if shape=='box':
        x,y,z=map(float,size.split());inertia=[mass*(y*y+z*z)/12,mass*(x*x+z*z)/12,mass*(x*x+y*y)/12]
    else:
        radius=float(size);inertia=[.4*mass*radius**2]*3
    element(inertial,'inertia',ixx=inertia[0],iyy=inertia[1],izz=inertia[2],ixy=0,ixz=0,iyz=0)
    for tag in ('visual','collision'):
        item=element(body,tag,name=name+('_collision' if tag=='collision' else '_visual'))
        element(item,'origin',xyz=origin)
        geometry=element(item,'geometry')
        element(geometry,shape,**({'size':size} if shape=='box' else {'radius':size}))
        if tag=='visual':element(element(item,'material',name=name+'_color'),'color',rgba=color)
    gazebo=element(root,'gazebo',reference=name);element(gazebo,'mu1',1.);element(gazebo,'mu2',1.)


def authored(name,pairs,segmented=False):
    root=ET.Element('robot',name=name);element(root,'link',name='base_footprint')
    length=.18*pairs if not segmented else .15
    link(root,'base_link',2.4 if not segmented else .8,'box',f'{length} .18 .07')
    # A massive root lets the URDF->MuJoCo importer retain the declared
    # trunk inertia. A massless root fused into the world loses that mass.
    joint(root,'base_fixed','base_link','base_footprint',kind='fixed')
    result=dict(schema_version=1,robot_id=name,description=f'{name}/urdf/{name}.urdf',forward_yaw=0.,body_center=[0.,0.],
                standing_height=.196,cycle_s=1.6,step_height=.025,max_stride=.045,max_speed=.045,max_yaw=.15,
                default_gait='tetrapod' if not segmented else 'wave',legs=[],spines=[],kp=75.,kd=2.5,torque_limit=8.)
    for pair in range(pairs):
        parent='base_link'
        x=(pairs-1-pair)*.18-length/2+.09 if not segmented else 0.
        if segmented and pair>0:
            parent='segment_'+str(pair)
            previous='base_link' if pair==1 else 'segment_'+str(pair-1)
            link(root,parent,.8,'box','.15 .18 .07',color='.55 .4 .2 1')
            joint(root,'spine_'+str(pair),previous,parent,xyz='-.18 0 0',bounds=(-.22,.22),effort=6.)
            result['spines'].append(dict(joint='spine_'+str(pair),index=pair))
        for side,sign in (('l',1),('r',-1)):
            leg='leg_'+side+str(pair+1)
            yaw=sign*1.5707963267948966
            link(root,leg+'_coxa_link',.07,'box','.045 .025 .025',origin='.0225 0 0')
            joint(root,leg+'_coxa',parent,leg+'_coxa_link',xyz=f'{x} {sign*.09} 0',rpy=f'0 0 {yaw}',bounds=(-.8,.8))
            link(root,leg+'_femur_link',.1,'box','.11 .025 .025',origin='.055 0 0')
            joint(root,leg+'_femur',leg+'_coxa_link',leg+'_femur_link',xyz='.045 0 0',axis='0 1 0',bounds=(-.6,1.5))
            link(root,leg+'_tibia_link',.09,'box','.14 .02 .02',origin='.07 0 0')
            joint(root,leg+'_tibia',leg+'_femur_link',leg+'_tibia_link',xyz='.11 0 0',axis='0 1 0',bounds=(-.2,2.))
            link(root,leg+'_foot',.02,'sphere','.016',color='.1 .1 .1 1')
            joint(root,leg+'_foot_joint',leg+'_tibia_link',leg+'_foot',xyz='.14 0 0',kind='fixed')
            # Exactly source FK: .11*sin(.3)+.14*sin(1.4) = .17048 m.
            result['legs'].append(dict(id=leg,root=parent,joints=[leg+'_'+j for j in ('coxa','femur','tibia')],
                                      foot=leg+'_foot',initial=[0.,.3,1.1],pair=pair,side=side,
                                      contact_geoms=[leg+'_foot_collision'],body_offset=[-.18*pair if segmented else 0.,0.]))
    # A real sensing frame; the existing MuJoCo bridge publishes physical IMU
    # and cast range data. Higher modes stay disabled until estimation exists.
    element(root,'link',name='imu_link');joint(root,'imu_fixed','base_link','imu_link',kind='fixed')
    save(root,ROBOTS/name/'urdf'/f'{name}.urdf')
    return result


def imported_hexapod():
    source=ROBOTS/'hexapod_ros2/description'
    if not source.is_dir():raise RuntimeError('Provision the pinned upstream description before generating')
    # Xacro include paths are adapted in a temporary in-memory top-level
    # document only; the original CAD description remains byte-identical.
    raw=(source/'urdf/Hexapod_Robot.xacro').read_text().replace('$(find Hexapod_Robot_description)',str(source))
    document=xacro.parse(raw)
    for include in list(document.getElementsByTagName('xacro:include')):
        if include.getAttribute('filename').endswith(('.gazebo','.ros2control')):
            include.parentNode.removeChild(include)
    xacro.process_doc(document)
    root=ET.fromstring(document.toxml())
    for item in list(root):
        if item.tag=='ros2_control' or item.tag=='gazebo' and not item.get('reference'):root.remove(item)
    base_joint=root.find("joint[@name='base_footprint_joint']")
    base_joint.find('parent').set('link','base_link');base_joint.find('child').set('link','base_footprint')
    for mesh in root.iter('mesh'):
        mesh.set('filename',mesh.get('filename').replace('file://'+str(source), 'package://robot_lab_robots/hexapod_ros2/description'))
    # Explicitly name base friction so the imported collision is traceable
    # through fixed-link fusion in actual body-contact telemetry.
    gazebo=element(root,'gazebo',reference='base_link');element(gazebo,'mu1',1.);element(gazebo,'mu2',1.)
    # Keep original body/link/joint/mass/collision definitions. Only the
    # simulator-controller declarations are replaced by an explicit PD contract.
    save(root,ROBOTS/'hexapod_ros2/urdf/hexapod_ros2.urdf')
    initial={'leg_l1':[0.,-.5920,.2684],'leg_l2':[0.,-.5921,.2687],'leg_l3':[0.,-.4538,.2681],
             'leg_r1':[0.,.5557,-.4075],'leg_r2':[0.,.5557,-.4075],'leg_r3':[0.,.5556,-.4088]}
    return dict(schema_version=1,robot_id='hexapod_ros2',description='hexapod_ros2/urdf/hexapod_ros2.urdf',
                upstream_revision=PIN,forward_yaw=-1.5707963267948966,body_center=[.115,.115],
                standing_height=.143,cycle_s=1.6,step_height=.02,max_stride=.025,max_speed=.025,max_yaw=.12,
                default_gait='tripod',spines=[],kp=160.,kd=5.,torque_limit=20.,
                passive_targets={'face_pan':0.,'face_tilt':0.},
                legs=[dict(id=leg,root='base_footprint',joints=[leg+'_'+j for j in ('coxa','femur','tibia')],foot=leg+'_foot',
                           initial=values,pair=int(leg[-1])-1,side=leg[4],contact_geoms=[leg+'_foot_collision'],body_offset=[0.,0.])
                      for leg,values in initial.items()])


def main():
    configs=[imported_hexapod(),authored('lab_octopod',4),authored('lab_centipede',6,True)]
    for config in configs:
        folder=ROBOTS/config['robot_id'];(folder/'config').mkdir(exist_ok=True)
        (folder/'config/gait.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    print('Generated three complete source-bound legged descriptions and gait configurations; runtime qualification separate')


if __name__=='__main__':main()
