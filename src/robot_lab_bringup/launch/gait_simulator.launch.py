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
    task=value('task')
    if task not in ('gait','loc','slam','3d_slam','nav'):raise ValueError('Choose gait/loc/slam/3d_slam/nav for the experimental gait workflow')
    if task!='gait':
        from robot_lab_utils.gait_pipeline import sensor_description
        urdf=sensor_description(urdf,model.config['forward_yaw'])
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
    if task!='gait':
        args['sensor_config']=json.dumps(dict(laser_link_name='gait_laser_link',camera_link_name='gait_camera_link',
            camera_optical_frame='gait_camera_optical_frame',scan_samples=360,scan_rate=10.,scan_range_min=.05,
            scan_range_max=8.,camera_width=320,camera_height=240,camera_rate=5. if task=='3d_slam' else 0.,camera_horizontal_fov=1.25,
            camera_near=.1,camera_far=8.))
    manifest=dict(robot=name,map=selected,backend='MuJoCo',world_xml=explicit_mjcf,spawn=pose,
                  task=task,sensor_kit=task!='gait',
                  description=str(description),config=str(config),pd=model.pd_config(),contacts=contacts,
                  scope='Experimental exact-geometry gait with actual contact/tilt guards; independent estimation/SLAM/navigation pending')
    (folder/'session.json').write_text(json.dumps(manifest,indent=2)+'\n');print('[robot_lab gait] '+str(folder/'session.json'))
    controller=Node(package='robot_lab_utils',executable='gait_controller.py',parameters=[{
        'use_sim_time':True,'description_file':str(description),'gait_config':str(config),'control_allowed':value('control')=='true',
        'navigation_enabled':task=='nav','task':task,'session_directory':str(folder)}],output='screen')
    actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('robot_lab_mujoco'))/'launch/mujoco_simulator.launch.py')),launch_arguments=args.items()),
             controller,RegisterEventHandler(OnProcessExit(target_action=controller,on_exit=[EmitEvent(event=Shutdown(reason='Gait controller exited'))]))]
    if task!='gait':
        from robot_lab_utils.gait_pipeline import pipeline
        actions+=pipeline(task,description,config,pose,world,folder,model,
            map_yaml=value('map_yaml'),database=value('database'),localization=value('localization')=='true',
            world_file=source_world)
    if value('rviz')=='true':
        rviz=Path(share('robot_lab_description'))/'rviz/display.rviz'
        frame='odom'
        if task!='gait':
            frame='map';rviz=folder/'gait.rviz'
            rviz.write_text(yaml.safe_dump({'Visualization Manager':{'Global Options':{'Fixed Frame':'map'},'Displays':[
                {'Class':'rviz_default_plugins/RobotModel','Name':'Actual joint model','Enabled':True,'Description Source':'Topic','Description Topic':'/robot_description'},
                {'Class':'rviz_default_plugins/Map','Name':'Gait map','Enabled':True,'Topic':{'Value':'/gait/map','Durability Policy':'Transient Local'}},
                {'Class':'rviz_default_plugins/LaserScan','Name':'Actual estimated-frame scan','Enabled':True,'Topic':{'Value':'/gait/scan','Reliability Policy':'Best Effort'}},
                {'Class':'rviz_default_plugins/TF','Name':'Truth visualization and independent estimated frames','Enabled':True}],
                'Tools':[{'Class':'rviz_default_plugins/Interact'},{'Class':'rviz_default_plugins/MoveCamera'},
                         {'Class':'rviz_default_plugins/SetInitialPose','Topic':'/gait/initialpose'},
                         {'Class':'rviz_default_plugins/SetGoal','Topic':'/gait/goal_pose'}]}}))
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(rviz),'-f',frame],parameters=[{'use_sim_time':True}],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('robot_model',default_value='hexapod_ros2'),
        DeclareLaunchArgument('task',default_value='gait'),
        DeclareLaunchArgument('map_yaml',default_value='',description='Acquired gait occupancy map for localization/navigation'),
        DeclareLaunchArgument('database',default_value='',description='Saved acquired RTAB-Map database; copied read-only'),
        DeclareLaunchArgument('localization',default_value='false',description='Localize in the selected saved 3D database'),
        DeclareLaunchArgument('map_name',default_value='nav_empty'),DeclareLaunchArgument('control',default_value='true'),
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),
        *[DeclareLaunchArgument('spawn_'+axis,default_value='') for axis in ('x','y','z','yaw')],OpaqueFunction(function=setup)])
