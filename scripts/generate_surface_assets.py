#!/usr/bin/env python3
"""Author a complete twin-hull vessel and a collision-equipped harbor.

Hull/inertia/thrust/drag are declared laboratory parameters, not a calibrated
commercial boat. Native Fortress buoyancy, thrusters and hydrodynamics act on
the free body. No meshes or external downloads are required at launch.
"""
import json
import xml.etree.ElementTree as ET
from generate_marine_assets import ROOT, child, model, write


def generate():
    ns='http://ros.org/wiki/xacro';ET.register_namespace('xacro',ns)
    root=ET.Element('robot',name='lab_usv')
    child(root,'{'+ns+'}arg',name='model_name',default='lab_usv')
    child(root,'{'+ns+'}arg',name='is_ignition',default='true')
    child(root,'{'+ns+'}include',filename='$(find robot_lab_robots)/description_common/urdf/oakd_camera.xacro')
    body=child(root,'link',name='base_link');inertia=child(body,'inertial')
    # Declared low ballast centre keeps this laboratory hull stable under
    # Fortress' graded-buoyancy surface approximation, without attitude locks.
    child(inertia,'origin',xyz='0 0 -.1',rpy='0 0 0');child(inertia,'mass',value='70')
    child(inertia,'inertia',ixx='13.1',iyy='11.8',izz='24.2',ixy='0',ixz='0',iyz='0')
    for name,xyz,size,color in [('port_hull','0 .42 0','1.4 .2 .24','.12 .25 .42 1'),
            ('starboard_hull','0 -.42 0','1.4 .2 .24','.12 .25 .42 1'),
            ('deck','0 0 .19','.95 .85 .08','.95 .68 .08 1')]:
        for kind in ('collision','visual'):
            node=child(body,kind,name=name+'_'+kind);child(node,'origin',xyz=xyz,rpy='0 0 0')
            child(child(node,'geometry'),'box',size=size)
            if kind=='visual':child(child(node,'material',name=name+'_paint'),'color',rgba=color)
    camera=child(root,'{'+ns+'}oakd',name='oakd',parent_link='base_link',model='pro',mesh_visual='false',rate='5')
    child(camera,'origin',xyz='.48 0 .43',rpy='0 0 0')
    sensor=child(child(root,'gazebo',reference='base_link'),'sensor',name='marine_imu',type='imu')
    for key,value in dict(always_on='true',update_rate='50',topic='/marine/imu',gz_frame_id='base_link').items():child(sensor,key,value)
    for side,y in [('port',.42),('starboard',-.42)]:
        name=side+'_propeller';link=child(root,'link',name=name+'_link');inertial=child(link,'inertial')
        child(inertial,'mass',value='.05');child(inertial,'inertia',ixx='.0001',iyy='.0001',izz='.0001',ixy='0',ixz='0',iyz='0')
        visual=child(link,'visual');child(visual,'origin',xyz='0 0 0',rpy='0 1.57079632679 0')
        child(child(visual,'geometry'),'cylinder',radius='.065',length='.02')
        joint=child(root,'joint',name=name+'_joint',type='continuous')
        child(joint,'parent',link='base_link');child(joint,'child',link=name+'_link')
        child(joint,'origin',xyz=f'-.68 {y} -.1',rpy='0 0 0');child(joint,'axis',xyz='1 0 0')
        child(joint,'limit',effort='5',velocity='400')
        plugin=child(child(root,'gazebo'),'plugin',filename='ignition-gazebo-thruster-system',name='ignition::gazebo::systems::Thruster')
        for key,value in dict(namespace='$(arg model_name)',joint_name=name+'_joint',thrust_coefficient='.0044',
                fluid_density='1025',propeller_diameter='.13',min_thrust_cmd='-35',max_thrust_cmd='35',velocity_control='true').items():child(plugin,key,value)
    hydro=child(child(root,'gazebo'),'plugin',filename='ignition-gazebo-hydrodynamics-system',name='ignition::gazebo::systems::Hydrodynamics')
    for key,value in dict(link_name='base_link',xDotU=5,yDotV=12,zDotW=18,kDotP=.4,mDotQ=.7,nDotR=1.,
            xU=-18,yV=-40,zW=-100,kP=-6,mQ=-8,nR=-6,
            xUabsU=-22,yVabsV=-65,zWabsW=-120,kPabsP=-5,mQabsQ=-6,nRabsR=-9).items():child(hydro,key,value)
    for system,filename,parameters in [('OdometryPublisher','odometry-publisher',dict(odom_frame='map',robot_base_frame='base_link',odom_topic='/marine/odometry',tf_topic='/marine/truth_pose',odom_publish_frequency='50',dimensions='3')),
            ('JointStatePublisher','joint-state-publisher',dict(topic='/marine/joint_states'))]:
        plugin=child(child(root,'gazebo'),'plugin',filename='ignition-gazebo-'+filename+'-system',name='ignition::gazebo::systems::'+system)
        for key,value in parameters.items():child(plugin,key,value)
    target=ROOT/'src/robot_lab_robots/lab_usv';write(root,target/'urdf/lab_usv.urdf.xacro')
    (target/'SOURCE.json').write_text(json.dumps(dict(author='Robot Lab contributors',license='Apache-2.0',
        plant='First-party 70 kg catamaran, two 35 N propellers, authored drag/inertia',
        native_systems='Gazebo Fortress 6.18 graded buoyancy, thruster and hydrodynamics',
        reference='https://github.com/gazebosim/gz-sim/blob/ignition-gazebo6_6.18.0/examples/worlds/graded_buoyancy.sdf',
        scope='Uncalibrated research simulator; actual native physics, no hardware performance claim'),indent=2)+'\n')
    source=ET.parse(ROOT/'src/robot_lab_maps/maps/marine_harbor/worlds/marine_harbor.world')
    world=source.getroot().find('world');world.set('name','surface_harbor')
    world.find("plugin[@name='ignition::gazebo::systems::Buoyancy']/enable").text='lab_usv'
    for i,(position,size) in enumerate([((9,0,-1),(.4,14,4)),((-9,0,-1),(.4,14,4)),((0,7,-1),(18,.4,4)),((0,-7,-1),(18,.4,4))]):
        model(world,'harbor_wall_'+str(i),position,size,'.55 .58 .6 1')
    for i,(x,y) in enumerate([(-1,3),(-5,3),(-5,-4)]):
        model(world,'channel_marker_'+str(i),(x,y,.4),(.4,.4,2),'.85 .25 .12 1')
    write(source.getroot(),ROOT/'src/robot_lab_maps/maps/surface_harbor/worlds/surface_harbor.world')


if __name__=='__main__':generate()
