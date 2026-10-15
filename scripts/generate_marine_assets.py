#!/usr/bin/env python3
"""First-party laboratory ROV and complete marine collision worlds.

Authoring tool, not a mission validator. Gazebo's existing Fortress systems
provide physical buoyancy/thrust/hydrodynamics. No measured hardware model is
claimed by the authored hull, inertia, thrust curve or damping coefficients.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def child(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, **attributes)
    if text is not None:
        element.text = str(text)
    return element


def write(root, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    path.write_bytes(ET.tostring(root, encoding='utf-8', xml_declaration=True))


def robot():
    xacro = 'http://ros.org/wiki/xacro'
    ET.register_namespace('xacro', xacro)
    root = ET.Element('robot', name='lab_rov')
    child(root, '{'+xacro+'}arg', name='is_ignition', default='true')
    child(root, '{'+xacro+'}arg', name='model_name', default='lab_rov')
    child(root, '{'+xacro+'}include', filename='$(find robot_lab_robots)/description_common/urdf/oakd_camera.xacro')
    material = child(root, 'material', name='dark_grey')
    child(material, 'color', rgba='.15 .18 .2 1')
    material = child(root, 'material', name='rov_yellow')
    child(material, 'color', rgba='.95 .7 .1 1')
    hull = child(root, 'link', name='base_link')
    inertia = child(hull, 'inertial')
    child(inertia, 'origin', xyz='0 0 -.025', rpy='0 0 0')
    child(inertia, 'mass', value='18.4')
    child(inertia, 'inertia', ixx='.159466667', iyy='.284586667', izz='.365546667', ixy='0', ixz='0', iyz='0')
    for kind in ('visual', 'collision'):
        element = child(hull, kind, name='hull_'+kind)
        geometry = child(element, 'geometry')
        child(geometry, 'box', size='.4 .28 .16')
        if kind == 'visual':
            child(element, 'material', name='rov_yellow')
    camera = child(root, '{'+xacro+'}oakd', name='oakd', parent_link='base_link', model='pro', mesh_visual='false')
    child(camera, 'origin', xyz='.21 0 .06', rpy='0 0 0')
    sensors = child(root, 'gazebo', reference='base_link')
    imu = child(sensors, 'sensor', name='marine_imu', type='imu')
    child(imu, 'always_on', 'true')
    child(imu, 'update_rate', 50)
    child(imu, 'topic', '/marine/imu')
    child(imu, 'gz_frame_id', 'base_link')
    positions = [(.18,.15,0),(.18,-.15,0),(-.18,.15,0),(-.18,-.15,0)]
    # Diagonal directions provide independent surge/sway/yaw. Vertical
    # corners provide heave/roll/pitch; source-derived allocation is rank six.
    angles = [-math.pi/4,math.pi/4,math.pi/4,-math.pi/4]
    for index in range(8):
        name = 'thruster_'+str(index)
        link = child(root, 'link', name=name+'_link')
        inertia = child(link, 'inertial')
        child(inertia, 'mass', value='.01')
        child(inertia, 'inertia', ixx='.000004', iyy='.000004', izz='.000004', ixy='0', ixz='0', iyz='0')
        visual = child(link, 'visual')
        child(visual, 'origin', xyz='0 0 0', rpy='0 1.57079632679 0' if index < 4 else '0 0 0')
        child(child(visual, 'geometry'), 'cylinder', radius='.04', length='.008')
        child(visual, 'material', name='dark_grey')
        joint = child(root, 'joint', name=name+'_joint', type='continuous')
        child(joint, 'parent', link='base_link')
        child(joint, 'child', link=name+'_link')
        pos = positions[index % 4]
        child(joint, 'origin', xyz=' '.join(map(str,pos)), rpy='0 0 '+str(angles[index] if index < 4 else 0.))
        child(joint, 'axis', xyz='1 0 0' if index < 4 else '0 0 1')
        child(joint, 'limit', effort='2', velocity='300')
        plugin = child(child(root, 'gazebo'), 'plugin', filename='ignition-gazebo-thruster-system', name='ignition::gazebo::systems::Thruster')
        for tag, value in dict(namespace='$(arg model_name)', joint_name=name+'_joint', thrust_coefficient=.0044,
                               fluid_density=1025, propeller_diameter=.08, min_thrust_cmd=-12, max_thrust_cmd=12,
                               velocity_control='true').items():
            child(plugin, tag, value)
    gazebo = child(root, 'gazebo')
    hydro = child(gazebo, 'plugin', filename='ignition-gazebo-hydrodynamics-system', name='ignition::gazebo::systems::Hydrodynamics')
    parameters = dict(link_name='base_link', xDotU=-3,yDotV=-5,zDotW=-7,kDotP=-.04,mDotQ=-.08,nDotR=-.08,
                      xU=-8,yV=-12,zW=-15,kP=-.2,mQ=-.3,nR=-.3,
                      xUabsU=-20,yVabsV=-30,zWabsW=-35,kPabsP=-.3,mQabsQ=-.5,nRabsR=-.5)
    for tag,value in parameters.items():
        child(hydro,tag,value)
    odom = child(gazebo,'plugin',filename='ignition-gazebo-odometry-publisher-system',name='ignition::gazebo::systems::OdometryPublisher')
    for tag,value in dict(odom_frame='map',robot_base_frame='base_link',odom_topic='/marine/odometry',
                           tf_topic='/marine/truth_pose',odom_publish_frequency=50,dimensions=3).items():
        child(odom,tag,value)
    joint_states=child(gazebo,'plugin',filename='ignition-gazebo-joint-state-publisher-system',name='ignition::gazebo::systems::JointStatePublisher')
    child(joint_states,'topic','/marine/joint_states')
    write(root,ROOT/'src/robot_lab_robots/lab_rov/urdf/lab_rov.urdf.xacro')


def model(world,name,position,size,color='.3 .35 .3 1',sphere=False,collision=True):
    entity=child(world,'model',name=name)
    child(entity,'static','true')
    child(entity,'pose',' '.join(map(str,position))+' 0 0 0')
    link=child(entity,'link',name='link')
    for tag in (('visual','collision') if collision else ('visual',)):
        element=child(link,tag,name=tag)
        geometry=child(element,'geometry')
        if sphere:
            child(child(geometry,'sphere'),'radius',size[0])
        else:
            child(child(geometry,'box'),'size',' '.join(map(str,size)))
        if tag=='visual':
            material=child(element,'material')
            child(material,'ambient',color)
            child(material,'diffuse',color)


def worlds():
    for name in ('marine_tank','marine_reef','marine_harbor','marine_cave','marine_wreck'):
        root=ET.Element('sdf',version='1.7')
        world=child(root,'world',name=name)
        physics=child(world,'physics',name='marine_physics',type='ode')
        child(physics,'max_step_size',.001)
        child(physics,'real_time_factor',1)
        child(world,'gravity','0 0 -9.81')
        for short,system in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors'),('imu','Imu')]:
            plugin=child(world,'plugin',filename='ignition-gazebo-'+short+'-system',name='ignition::gazebo::systems::'+system)
            if system=='Sensors':child(plugin,'render_engine','ogre2')
        plugin=child(world,'plugin',filename='ignition-gazebo-buoyancy-system',name='ignition::gazebo::systems::Buoyancy')
        graded=child(plugin,'graded_buoyancy')
        child(graded,'default_density',1025)
        change=child(graded,'density_change');child(change,'above_depth',0);child(change,'density',1.225)
        child(plugin,'enable','lab_rov')
        scene=child(world,'scene');child(scene,'ambient','.3 .55 .6 1');child(scene,'background','.05 .18 .25 1')
        light=child(world,'light',name='sun',type='directional')
        child(light,'pose','0 0 10 0 0 0');child(light,'diffuse','.8 .9 1 1');child(light,'direction','-.3 .2 -1')
        model(world,'seafloor',(0,0,-6.2),(18,14,.4),'.35 .3 .2 1')
        model(world,'water_surface',(0,0,0),(18,14,.015),'0 .4 .6 .18',collision=False)
        if name=='marine_tank':
            for i,(position,size) in enumerate([((6,0,-3),(.3,10,6)),((-6,0,-3),(.3,10,6)),((0,5,-3),(12,.3,6)),((0,-5,-3),(12,.3,6))]):
                model(world,'tank_wall_'+str(i),position,size,'.45 .5 .55 1')
        elif name=='marine_reef':
            for i,(x,y,r) in enumerate([(2,1,.9),(3,-2,1.3),(-3,2,.7),(-2,-2,1.2),(4,3,.8),(-4,-3,.8)]):
                model(world,'reef_rock_'+str(i),(x,y,-6+r),(r,),'.3 .35 .25 1',sphere=True)
        elif name=='marine_harbor':
            model(world,'dock',(3,0,.15),(5,10,.3),'.35 .25 .15 1')
            for i,(x,y) in enumerate([(1,-3),(1,0),(1,3),(5,-3),(5,0),(5,3)]):
                model(world,'harbor_piling_'+str(i),(x,y,-3),(.3,.3,6),'.2 .22 .2 1')
        elif name=='marine_cave':
            for i,(p,s) in enumerate([((2,-1.7,-3),(8,.4,4)),((2,1.7,-3),(8,.4,4)),((2,0,-1),(8,3.8,.4))]):
                model(world,'tunnel_'+str(i),p,s,'.25 .25 .22 1')
            model(world,'cave_boulder',(-3,3,-4.5),(1.5,),'.3 .3 .27 1',sphere=True)
        else:
            model(world,'wreck_keel',(2,0,-5.8),(6,3,.3),'.3 .17 .1 1')
            for i in range(5):
                model(world,'wreck_rib_'+str(i),(.1+1.1*i,0,-4.8),(.15,3,2),'.28 .15 .09 1')
            model(world,'wreck_side',(2,-1.5,-4.8),(6,.15,2),'.28 .15 .09 1')
        write(root,ROOT/'src/robot_lab_maps/maps'/name/'worlds'/(name+'.world'))


if __name__=='__main__':
    robot();worlds()
