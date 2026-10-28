"""Complete native tracked/flipper vehicle in the actual selected world."""
import copy
import json
import os
from pathlib import Path
import tempfile
import uuid
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,SetEnvironmentVariable,TimerAction,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    def value(key):return LaunchConfiguration(key).perform(context)
    def include(package,file,arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share(package))/'launch'/file)),launch_arguments=arguments.items())
    from robot_lab_utils.installed_assets import merge_installed_profiles
    from robot_lab_utils.gazebo_physics_world import cap_physics_step
    from robot_lab_utils.fleet_session import finite
    task=value('task')
    if task not in ('display','loc','slam','3d_slam','nav'):raise ValueError('Choose display/loc/slam/3d_slam/nav')
    profiles=merge_installed_profiles(yaml.safe_load((Path(share('robot_lab_bringup'))/'config/sim_maps.yaml').read_text()),'maps')['maps']
    name=value('map_name');name='empty' if name.lower() in ('','none','no map') else name
    profile=profiles[name]
    engine_receipt=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'native_tracks/receipt.json'
    if not engine_receipt.is_file():raise ValueError('Provision the private native track engine: python3 scripts/provision_tracked_physics.py')
    engine=json.loads(engine_receipt.read_text())
    import hashlib
    if not Path(engine['engine']).is_file() or hashlib.sha256(Path(engine['engine']).read_bytes()).hexdigest()!=engine['engine_sha256']:
        raise ValueError('Private native track engine differs from its build receipt; provision again')
    if profile.get('marine'):raise ValueError('Tracks require a terrestrial collision world')
    gz=profile['gazebo'];source=Path(share(gz.get('world_package','robot_lab_maps')))/gz['world_path']
    world=ET.parse(cap_physics_step(source,.002)).getroot();element=world.find('world')
    base=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'tracked_sessions'
    base.mkdir(parents=True,exist_ok=True)
    if base.stat().st_dev==Path('/').stat().st_dev:raise ValueError('Tracked sessions must remain on SSD')
    folder=Path(tempfile.mkdtemp(prefix='tracks-',dir=base))
    # Preserve source-relative model/texture resources when cloning the world.
    for node in element.iter():
        if node.tag in ('uri','diffuse','normal','image') and node.text:
            text=node.text.strip()
            if text and '://' not in text and not Path(text).is_absolute() and (source.parent/text).exists():node.text=str((source.parent/text).resolve())
    def plugin(parent,name,filename):
        return ET.SubElement(parent,'plugin',name='ignition::gazebo::systems::'+name,filename='ignition-gazebo-'+filename+'-system')
    existing={p.get('name','').split('::')[-1] for p in element.findall('plugin')}
    for key,file in (('Physics','physics'),('UserCommands','user-commands'),('SceneBroadcaster','scene-broadcaster'),('Sensors','sensors'),('Imu','imu')):
        if key not in existing:
            p=plugin(element,key,file)
            if key=='Sensors':ET.SubElement(p,'render_engine').text='ogre2'
    model=ET.parse(Path(share('robot_lab_robots'))/'lab_tracked/model.sdf').getroot().find('model')
    pose={axis:finite(profile.get('spawn',{}).get(axis,default),'tracked spawn '+axis) for axis,default in (('x',0.),('y',0.),('z',.1),('yaw',0.))}
    for axis in pose:
        text=value('spawn_'+axis).strip()
        if text.lower() not in ('','auto'):pose[axis]=finite(text,'tracked spawn '+axis)
    control=next(p for p in model.findall('plugin') if p.get('name','').endswith('TrackedVehicle'))
    control.find('topic').text='/tracked/actuator_cmd'
    control.set('name','robot_lab::TrackedVehicle')
    control.set('filename',str(Path(get_package_prefix('robot_lab_maps'))/'lib/librobot_lab_tracked_vehicle.so'))
    control.find('steering_efficiency').text='1.0'
    guard=ET.SubElement(model,'plugin',name='robot_lab::CommandWatchdog',filename=str(Path(get_package_prefix('robot_lab_maps'))/'lib/librobot_lab_command_watchdog.so'))
    for key,text in dict(input='/tracked/cmd_vel',output='/tracked/actuator_cmd',timeout_s='.4',max_linear='.3',max_angular='.5').items():ET.SubElement(guard,key).text=text
    model.find('pose').text=f"{pose['x']} {pose['y']} {pose['z']} 0 0 {pose['yaw']}"
    base_link=model.find("link[@name='base_link']")
    lidar=ET.SubElement(base_link,'sensor',name='laser_link',type='gpu_lidar')
    ET.SubElement(lidar,'pose').text='0 0 .32 0 0 0';ET.SubElement(lidar,'topic').text='/tracked/scan';ET.SubElement(lidar,'update_rate').text='10'
    scan=ET.SubElement(ET.SubElement(lidar,'lidar'),'scan');horizontal=ET.SubElement(scan,'horizontal')
    for key,text in dict(samples='360',resolution='1',min_angle='-3.14159265',max_angle='3.14159265').items():ET.SubElement(horizontal,key).text=text
    distance=ET.SubElement(lidar.find('lidar'),'range')
    for key,text in dict(min='.08',max='12',resolution='.01').items():ET.SubElement(distance,key).text=text
    imu=ET.SubElement(base_link,'sensor',name='imu',type='imu');ET.SubElement(imu,'topic').text='/tracked/imu';ET.SubElement(imu,'update_rate').text='50'
    camera=ET.SubElement(base_link,'sensor',name='camera',type='rgbd_camera');ET.SubElement(camera,'pose').text='.05 0 .34 0 0 0'
    ET.SubElement(camera,'topic').text='/tracked/camera';ET.SubElement(camera,'update_rate').text='5'
    lens=ET.SubElement(camera,'camera');ET.SubElement(lens,'horizontal_fov').text='1.25'
    image=ET.SubElement(lens,'image');ET.SubElement(image,'width').text='320';ET.SubElement(image,'height').text='240'
    clip=ET.SubElement(lens,'clip');ET.SubElement(clip,'near').text='.1';ET.SubElement(clip,'far').text='10'
    truth=plugin(model,'OdometryPublisher','odometry-publisher')
    for key,text in dict(odom_frame='map',robot_base_frame='base_link',odom_topic='/tracked/truth',dimensions='3',odom_publish_frequency='30').items():ET.SubElement(truth,key).text=text
    joints=plugin(model,'JointStatePublisher','joint-state-publisher');ET.SubElement(joints,'topic').text='/tracked/joint_states'
    element.append(model);world_file=folder/'world.sdf';ET.indent(world);world_file.write_bytes(ET.tostring(world))
    urdf=ET.parse(Path(share('robot_lab_robots'))/'lab_tracked/robot.urdf').getroot()
    for child,parent,xyz,rpy in (('laser_link','base_link','0 0 .32','0 0 0'),('imu_link','base_link','0 0 0','0 0 0'),
        ('camera_link','base_link','.05 0 .34','0 0 0'),('camera_optical_frame','camera_link','0 0 0','-1.57079633 0 -1.57079633')):
        ET.SubElement(urdf,'link',name=child);joint=ET.SubElement(urdf,'joint',name=child+'_fixed',type='fixed')
        ET.SubElement(joint,'parent',link=parent);ET.SubElement(joint,'child',link=child);ET.SubElement(joint,'origin',xyz=xyz,rpy=rpy)
    description=ET.tostring(urdf,encoding='unicode')
    bridges=[]
    def bridge(ros,gz_topic,ros_type,gz_type,frame=None,direction='GZ_TO_ROS'):
        row=dict(ros_topic_name=ros,gz_topic_name=gz_topic,ros_type_name=ros_type,gz_type_name='ignition.msgs.'+gz_type,direction=direction)
        # tracked_control normalizes headers against the explicit URDF kit.
        bridges.append(row)
    bridge('/clock','/clock','rosgraph_msgs/msg/Clock','Clock')
    bridge('/tracked/cmd_vel','/tracked/cmd_vel','geometry_msgs/msg/Twist','Twist',direction='ROS_TO_GZ')
    bridge('/robot_lab_controller/odom','/tracked/odometry','nav_msgs/msg/Odometry','Odometry')
    bridge('/odom/ground_truth','/tracked/truth','nav_msgs/msg/Odometry','Odometry')
    bridge('/joint_states','/tracked/joint_states','sensor_msgs/msg/JointState','Model')
    bridge('/tracked/raw/scan','/tracked/scan','sensor_msgs/msg/LaserScan','LaserScan','laser_link')
    bridge('/tracked/raw/imu/out','/tracked/imu','sensor_msgs/msg/Imu','IMU','imu_link')
    for ros,gz_topic,ros_type,gz_type in (('/camera/color/image_raw','/tracked/camera/image','sensor_msgs/msg/Image','Image'),
        ('/camera/depth/image_raw','/tracked/camera/depth_image','sensor_msgs/msg/Image','Image'),
        ('/camera/color/camera_info','/tracked/camera/camera_info','sensor_msgs/msg/CameraInfo','CameraInfo')):
        bridge('/tracked/raw'+ros,gz_topic,ros_type,gz_type,'camera_optical_frame')
    bridge_file=folder/'bridge.yaml';bridge_file.write_text(yaml.safe_dump(bridges))
    partition=os.environ.get('IGN_PARTITION') or os.environ.get('GZ_PARTITION') or 'tracks_'+uuid.uuid4().hex
    actions=[SetEnvironmentVariable(key,partition) for key in ('IGN_PARTITION','GZ_PARTITION')]
    actions.append(SetEnvironmentVariable('LD_LIBRARY_PATH',os.pathsep.join(engine['library_paths']+[os.environ.get('LD_LIBRARY_PATH','')])))
    resource=os.pathsep.join([os.environ.get('IGN_GAZEBO_RESOURCE_PATH',''),str(source.parent),str(Path(share('robot_lab_maps'))),str(Path(share('robot_lab_robots')))])
    actions.extend(SetEnvironmentVariable(key,resource) for key in ('IGN_GAZEBO_RESOURCE_PATH','GZ_SIM_RESOURCE_PATH'))
    owner=Node(package='robot_lab_utils',executable='tracked_control.py',parameters=[dict(use_sim_time=True,navigation=task=='nav')])
    actions.append(RegisterEventHandler(OnProcessExit(target_action=owner,on_exit=[EmitEvent(event=Shutdown(reason='Native tracked command owner exited'))])))
    actions+=[include('ros_gz_sim','gz_sim.launch.py',dict(gz_args='-r -v 2 --physics-engine '+engine['engine']+' '+('' if value('gui')=='true' else '-s --headless-rendering ')+str(world_file),on_exit_shutdown='True')),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[dict(use_sim_time=True,robot_description=ParameterValue(description,value_type=str))]),
        Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[dict(use_sim_time=True,config_file=str(bridge_file))]),
        owner]
    common=dict(use_sim_time='true',robot_model='lab_tracked',base_frame='base_footprint')
    actions.append(include('robot_lab_localization','local_localization.launch.py',dict(common,odom0='/robot_lab_controller/odom')))
    pipeline=[]
    if task in ('loc','nav'):
        entry=profile.get('map',{})
        if not entry.get('has_2d_map'):raise ValueError('Selected tracked localization/navigation world needs an occupancy map; generate it in Worlds')
        map_yaml=str(Path(share(entry.get('package','robot_lab_maps')))/entry['path'])
        pipeline.append(include('robot_lab_localization','global_localization.launch.py',dict(common,map_yaml=map_yaml,map_name=name,
            initial_pose_x=str(pose['x']),initial_pose_y=str(pose['y']),initial_pose_yaw=str(pose['yaw']),motion_model='diff')))
    elif task=='slam':pipeline.append(include('robot_lab_mapping','slam.launch.py',dict(common,slam_backend='slam_toolbox')))
    elif task=='3d_slam':pipeline.append(include('robot_lab_mapping','rtabmap.launch.py',dict(use_sim_time='true',frame_id='base_footprint',robot_name='lab_tracked',map_name=name,start_rtabmap_viz='false',start_visual_odometry='false',odom_topic='/odometry/filtered',rgb_topic='/camera/color/image_raw',depth_topic='/camera/depth/image_raw',camera_info_topic='/camera/color/camera_info')))
    if task=='nav':pipeline.append(include('robot_lab_navigation','navigation.launch.py',dict(common,global_planner_plugin=value('global_planner_plugin'),local_planner_plugin=value('local_planner_plugin'))))
    if pipeline:actions.append(TimerAction(period=7.,actions=pipeline))
    if value('rviz')=='true':actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(Path(share('robot_lab_description'))/'rviz/display.rviz'),'-f','map' if task!='display' else 'odom'],parameters=[dict(use_sim_time=True)]))
    (folder/'session.json').write_text(json.dumps(dict(robot='lab_tracked',map=name,task=task,source_world=str(source),world=str(world_file),spawn=pose,
        odometry='Native track dead reckoning; independent truth remains /odom/ground_truth',physics_engine=engine,scope='Implemented experimental native tracks; runtime mission acceptance separate'),indent=2)+'\n')
    print('[robot_lab tracked] '+str(folder/'session.json'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map_name',default_value='nav_empty'),DeclareLaunchArgument('task',default_value='display'),
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('global_planner_plugin',default_value='auto'),DeclareLaunchArgument('local_planner_plugin',default_value='auto'),
        *[DeclareLaunchArgument('spawn_'+axis,default_value='') for axis in ('x','y','z','yaw')],OpaqueFunction(function=setup)])
