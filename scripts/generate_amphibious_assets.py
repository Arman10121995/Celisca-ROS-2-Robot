#!/usr/bin/env python3
"""Complete first-party wheeled/propeller craft and a physical shore ramp."""
import json
import math
import xml.etree.ElementTree as ET
from generate_marine_assets import ROOT, child, model, write


def generate():
    ns='http://ros.org/wiki/xacro';ET.register_namespace('xacro',ns)
    root=ET.Element('robot',name='lab_amphibious')
    for name,value in [('model_name','lab_amphibious'),('is_ignition','true')]:child(root,'{'+ns+'}arg',name=name,default=value)
    child(root,'{'+ns+'}include',filename='$(find robot_lab_robots)/description_common/urdf/oakd_camera.xacro')
    body=child(root,'link',name='base_link');inertia=child(body,'inertial')
    child(inertia,'origin',xyz='0 0 -.12',rpy='0 0 0');child(inertia,'mass',value='50')
    child(inertia,'inertia',ixx='4.1',iyy='8.1',izz='10.2',ixy='0',ixz='0',iyz='0')
    for kind in ('collision','visual'):
        item=child(body,kind,name='sealed_hull_'+kind);child(child(item,'geometry'),'box',size='1.3 .82 .34')
        if kind=='visual':child(child(item,'material',name='hull_orange'),'color',rgba='.95 .4 .05 1')
    camera=child(root,'{'+ns+'}oakd',name='oakd',parent_link='base_link',model='pro',mesh_visual='false',rate='5')
    child(camera,'origin',xyz='.66 0 .3',rpy='0 0 0')
    sensor=child(child(root,'gazebo',reference='base_link'),'sensor',name='marine_imu',type='imu')
    for key,value in dict(always_on='true',update_rate='50',topic='/marine/imu',gz_frame_id='base_link').items():child(sensor,key,value)
    for side,y in [('left',.49),('right',-.49)]:
        for end,x in [('front',.43),('rear',-.43)]:
            name=end+'_'+side+'_wheel';carrier=end+'_'+side+'_carrier'
            mount=child(root,'link',name=carrier);inertial=child(mount,'inertial');child(inertial,'mass',value='.1')
            child(inertial,'inertia',ixx='.0001',iyy='.0001',izz='.0001',ixy='0',ixz='0',iyz='0')
            suspension=child(root,'joint',name=end+'_'+side+'_suspension_joint',type='prismatic')
            child(suspension,'parent',link='base_link');child(suspension,'child',link=carrier)
            child(suspension,'origin',xyz=f'{x} {y} -.12',rpy='0 0 0');child(suspension,'axis',xyz='0 0 1')
            child(suspension,'limit',lower='-.04',upper='.04',effort='600',velocity='.3')
            link=child(root,'link',name=name)
            inertial=child(link,'inertial');child(inertial,'mass',value='1')
            child(inertial,'inertia',ixx='.0126',iyy='.0242',izz='.0126',ixy='0',ixz='0',iyz='0')
            for kind in ('collision','visual'):
                item=child(link,kind,name=name+'_'+kind);child(item,'origin',xyz='0 0 0',rpy=f'{math.pi/2} 0 0')
                child(child(item,'geometry'),'cylinder',radius='.22',length='.075')
                if kind=='visual':child(child(item,'material',name='tire'),'color',rgba='.08 .08 .1 1')
            joint=child(root,'joint',name=name+'_joint',type='continuous');child(joint,'parent',link=carrier);child(joint,'child',link=name)
            child(joint,'origin',xyz='0 0 0',rpy='0 0 0');child(joint,'axis',xyz='0 1 0')
            child(joint,'limit',effort='8',velocity='3');child(joint,'dynamics',damping='.02',friction='.01')
            gz=child(root,'gazebo',reference=name)
            child(gz,'mu1','.9');child(gz,'mu2','.9')
    for side,y in [('port',.38),('starboard',-.38)]:
        name=side+'_propeller';link=child(root,'link',name=name+'_link');inertial=child(link,'inertial')
        child(inertial,'mass',value='.05');child(inertial,'inertia',ixx='.0001',iyy='.0001',izz='.0001',ixy='0',ixz='0',iyz='0')
        visual=child(link,'visual');child(visual,'origin',xyz='0 0 0',rpy='0 1.57079632679 0')
        child(child(visual,'geometry'),'cylinder',radius='.065',length='.02')
        joint=child(root,'joint',name=name+'_joint',type='continuous');child(joint,'parent',link='base_link');child(joint,'child',link=name+'_link')
        child(joint,'origin',xyz=f'-.64 {y} -.15',rpy='0 0 0');child(joint,'axis',xyz='1 0 0');child(joint,'limit',effort='5',velocity='400')
        plugin=child(child(root,'gazebo'),'plugin',filename='ignition-gazebo-thruster-system',name='ignition::gazebo::systems::Thruster')
        for key,value in dict(namespace='$(arg model_name)',joint_name=name+'_joint',thrust_coefficient='.0044',fluid_density='1025',propeller_diameter='.13',min_thrust_cmd='-35',max_thrust_cmd='35',velocity_control='true').items():child(plugin,key,value)
    for system,filename,parameters in [('OdometryPublisher','odometry-publisher',dict(odom_frame='map',robot_base_frame='base_link',odom_topic='/marine/odometry',odom_publish_frequency='50',dimensions='3')),
            ('JointStatePublisher','joint-state-publisher',dict(topic='/marine/joint_states'))]:
        plugin=child(child(root,'gazebo'),'plugin',filename='ignition-gazebo-'+filename+'-system',name='ignition::gazebo::systems::'+system)
        for key,value in parameters.items():child(plugin,key,value)
    target=ROOT/'src/robot_lab_robots/lab_amphibious';write(root,target/'urdf/lab_amphibious.urdf.xacro')
    (target/'SOURCE.json').write_text(json.dumps(dict(author='Robot Lab contributors',license='Apache-2.0',
        plant='Authored 54.5 kg sealed hull with four independently sprung torque-limited wheels and two native propellers',
        revision='Passive suspension derivative; prior rigid-wheel source and failed return retained on SSD',
        suspension='Four passive 10000 N/m springs, 200 Ns/m dampers, +/-0.04 m stroke and 600 N force bound',
        source_limits=dict(wheel_radius_m=.22,wheel_half_track_m=.49,wheel_motor_Nm=8,wheel_rate_rad_s=3,propeller_force_N=35),
        water='Native graded buoyancy; immersion-gated thrust and authored passive drag; no wave/added-mass calibration',
        controller='Native joint effort servo and wall-clock leases, ROS operator ownership; no pose or velocity writes'),indent=2)+'\n')
    world=ET.parse(ROOT/'src/robot_lab_maps/maps/surface_harbor/worlds/surface_harbor.world');w=world.getroot().find('world');w.set('name','amphibious_shore')
    for node in list(w.findall('model')):w.remove(node)
    w.find("plugin[@name='ignition::gazebo::systems::Buoyancy']/enable").text='lab_amphibious'
    model(w,'seafloor',(0,0,-1.7),(24,12,.4),'.35 .3 .2 1')
    model(w,'water_surface',(0,0,0),(24,12,.01),'0 .4 .7 .2');water=w.find("model[@name='water_surface']")
    water.find('link').remove(water.find('link/collision'))
    model(w,'west_bank',(-7.5,0,-.55),(9,12,1.9),'.45 .5 .25 1')
    model(w,'shore_ramp',(1,0,-.65),(8.25,12,.2),'.5 .48 .35 1');ramp=w.find("model[@name='shore_ramp']")
    ramp.find('pose').text=f'1 0 -.65 0 {math.atan(1.9/8)} 0'
    for i,(x,y) in enumerate([(8,-4),(8,4),(-8,-4),(-8,4)]):model(w,'marker_'+str(i),(x,y,.6),(.3,.3,1.8),'.8 .25 .1 1')
    for i,(position,size) in enumerate([((12,0,0),(.3,12,4)),((-12,0,0),(.3,12,4)),((0,6,0),(24,.3,4)),((0,-6,0),(24,.3,4))]):model(w,'boundary_'+str(i),position,size,'.6 .6 .6 1')
    target=ROOT/'src/robot_lab_maps/maps/amphibious_shore';write(world.getroot(),target/'worlds/amphibious_shore.world')
    (target/'SOURCE.json').write_text(json.dumps(dict(author='Robot Lab contributors',license='Apache-2.0',
        geometry='Complete bank, 13-degree shore ramp, water volume, seabed, markers and boundaries',
        water_surface_z=0,bank_surface_z=.4,seabed_surface_z=-1.5,
        scope='Authored transition environment, no surveyed bathymetry or hardware validation'),indent=2)+'\n')


if __name__=='__main__':generate()
