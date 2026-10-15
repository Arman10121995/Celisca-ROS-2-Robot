"""Exact-source multi-leg gait with reusable MuJoCo in-step bounded PD."""
import json
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,EmitEvent,IncludeLaunchDescription,OpaqueFunction,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    from robot_lab_utils.gait_model import GaitModel
    from robot_lab_utils.robot_description import load_description
    from robot_lab_utils.installed_assets import merge_installed_profiles
    from robot_lab_utils.fleet_session import finite
    def value(key):return LaunchConfiguration(key).perform(context)
    robots=Path(share('robot_lab_robots'));bringup=Path(share('robot_lab_bringup'))
    profiles=yaml.safe_load((robots/'config/robots.yaml').read_text())['robots']
    name=value('robot_model');profile=profiles[name]
    config=robots/profile['gait_model'];urdf=load_description(robots/profile['xacro']);model=GaitModel(urdf,config)
    maps=merge_installed_profiles(yaml.safe_load((bringup/'config/sim_maps.yaml').read_text()),'maps')['maps']
    selected=value('map_name');selected='empty' if selected.lower() in ('','none','no map') else selected
    world=maps[selected]
    if world.get('marine'):raise ValueError('Terrestrial gaits require a ground world')
    gz=world['gazebo'];source_world=str(Path(share(gz.get('world_package','robot_lab_maps')))/gz['world_path'])
    # No silent empty-world fallback for this new controlled workflow.
    explicit_mjcf=world.get('mujoco',{}).get('xml_path','')
    if not explicit_mjcf:
        xml_name=gz.get('world_name') or selected
        explicit_mjcf=str(Path(share('robot_lab_maps'))/'mjcf'/(xml_name+'.xml'))
    if not Path(explicit_mjcf).is_file():raise ValueError('Selected world has no installed MuJoCo collision conversion: '+explicit_mjcf)
    folder_root=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'gait_sessions'
    folder_root.mkdir(parents=True,exist_ok=True);folder=Path(tempfile.mkdtemp(prefix=name+'-',dir=folder_root))
    description=folder/'robot.urdf';description.write_text(urdf)
    pose={axis:finite(world.get('spawn',{}).get(axis,default),'map spawn '+axis)
          for axis,default in (('x',0.),('y',0.),('z',.1),('yaw',0.))}
    floor_reference=str(pose.get('z',.1))
    pose['z']=float(model.config['standing_height'])
    for axis in ('x','y','z','yaw'):
        item=value('spawn_'+axis).strip()
        if item.lower() not in ('','auto'):
            pose[axis]=finite(item,'gait spawn '+axis)
            if axis=='z':floor_reference=''
    # The imported robot's front is -Y in its original CAD root. Rotate the
    # complete plant at spawn; preserve source link/joint axes and geometry.
    pose['yaw']-=float(model.config['forward_yaw'])
    contacts={leg['id']:leg['contact_geoms'] for leg in model.legs}
    contacts['body']=['base_link_contact_0'] if name=='hexapod_ros2' else ['base_link_collision']+['segment_'+str(i)+'_collision' for i in range(1,len(model.config.get('spines',[]))+1)]
    args=dict(model=str(description),robot_name=name,robot_package='robot_lab_robots',robot_xacro=profile['xacro'],
              world_name=Path(explicit_mjcf).stem,world_path=source_world,gui=value('gui'),use_sim_time='true',
              hold_position='false',joint_pd_config=json.dumps(model.pd_config()),pd_joint_armature='.01',
              initial_joint_positions=json.dumps(dict(zip(model.names,model.initial.tolist()))),
              gait_contact_config=json.dumps(contacts),gait_floor_reference=floor_reference,physics_timestep='.001',max_catch_up_ticks='0',
              **{'spawn_'+key:str(item) for key,item in pose.items() if key in ('x','y','z','yaw')})
    manifest=dict(robot=name,map=selected,backend='MuJoCo',world_xml=explicit_mjcf,spawn=pose,
                  description=str(description),config=str(config),pd=model.pd_config(),contacts=contacts,
                  scope='Experimental exact-geometry gait with actual contact/tilt guards; independent estimation/SLAM/navigation pending')
    (folder/'session.json').write_text(json.dumps(manifest,indent=2)+'\n');print('[robot_lab gait] '+str(folder/'session.json'))
    controller=Node(package='robot_lab_utils',executable='gait_controller.py',parameters=[{
        'use_sim_time':True,'description_file':str(description),'gait_config':str(config),'control_allowed':value('control')=='true'}],output='screen')
    actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('robot_lab_mujoco'))/'launch/mujoco_simulator.launch.py')),launch_arguments=args.items()),
             controller,RegisterEventHandler(OnProcessExit(target_action=controller,on_exit=[EmitEvent(event=Shutdown(reason='Gait controller exited'))]))]
    if value('rviz')=='true':
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(Path(share('robot_lab_description'))/'rviz/display.rviz'),'-f','odom'],parameters=[{'use_sim_time':True}],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot_model',default_value='hexapod_ros2'),
        DeclareLaunchArgument('map_name',default_value='nav_empty'),DeclareLaunchArgument('control',default_value='true'),
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),
        *[DeclareLaunchArgument('spawn_'+axis,default_value='') for axis in ('x','y','z','yaw')],OpaqueFunction(function=setup)])
