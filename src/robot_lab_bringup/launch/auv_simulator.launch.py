"""Native source Tethys AUV in selected installed marine collision world."""
import json
import math
import os
from pathlib import Path
import tempfile
import uuid
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,EmitEvent,ExecuteProcess,OpaqueFunction,RegisterEventHandler,SetEnvironmentVariable,TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    from robot_lab_utils.marine_runtime import require_fortress_mass_convention
    require_fortress_mass_convention()
    from robot_lab_utils.robot_description import load_description
    from robot_lab_utils.fleet_session import finite
    from robot_lab_utils.robot_spawn import map_spawn_override
    def value(key):return LaunchConfiguration(key).perform(context)
    robots=Path(share('robot_lab_robots'));bringup=Path(share('robot_lab_bringup'));assets=robots/'tethys_auv'
    calibration=json.loads((assets/'calibration.json').read_text())
    maps=yaml.safe_load((bringup/'config/sim_maps.yaml').read_text())['maps'];name=value('map_name')
    if name not in maps or not maps[name].get('marine'):raise ValueError('Tethys AUV needs an installed marine water world')
    profile=maps[name];water=profile['marine'];gz=profile['gazebo']
    if float(water['density'])!=calibration['fluid_density_kg_m3']:raise ValueError('Regenerate explicit AUV trim for the chosen fluid density')
    source_world=Path(share(gz['world_package']))/gz['world_path']
    root=ET.parse(source_world).getroot();world=root.find('world');world_name=world.get('name')
    buoyancy=next(plugin for plugin in world.findall('plugin') if plugin.get('name','').endswith('::Buoyancy'))
    # This session owns one source AUV. Preserve selected obstacles and replace
    # only the world buoyancy inclusion identity, not source geometry.
    for enabled in buoyancy.findall('enable'):buoyancy.remove(enabled)
    ET.SubElement(buoyancy,'enable').text='tethys_auv'
    runtime=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'marine_sessions';runtime.mkdir(parents=True,exist_ok=True)
    folder=Path(tempfile.mkdtemp(prefix='tethys-',dir=runtime));world_file=folder/'world.sdf';world_file.write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True))
    native=ET.parse(assets/'model.sdf');model=native.getroot().find('model')
    for item in model.iter():
        if item.tag in ('uri','albedo_map','normal_map','metalness_map','roughness_map') and item.text:
            resource=Path(item.text.strip())
            if not resource.is_absolute() and '://' not in item.text:item.text=str((assets/'source'/resource).resolve())
    model_file=folder/'model.sdf';native.write(model_file,encoding='utf-8',xml_declaration=True)
    description=load_description(assets/'urdf/tethys_auv.urdf')
    if value('slam')=='true':
        from robot_lab_utils.marine_pipeline import auv_camera_kit
        description=auv_camera_kit(model,description)
        native.write(model_file,encoding='utf-8',xml_declaration=True)
    description_file=folder/'robot.urdf';description_file.write_text(description)
    pose={axis:finite(profile['spawn'].get(axis,0.),'marine spawn '+axis) for axis in ('x','y','z','yaw')}
    robot_profile=yaml.safe_load((robots/'config/robots.yaml').read_text())['robots']['tethys_auv']
    pose.update(map_spawn_override(robot_profile,name))
    for axis in ('x','y','z','yaw'):
        supplied=value('spawn_'+axis).strip()
        if supplied.lower() not in ('','auto'):pose[axis]=finite(supplied,'AUV spawn '+axis)
    if not water['bottom_z']+.6<pose['z']<water['surface_z']-.6:raise ValueError('AUV spawn needs surface/seabed body clearance')
    pose['yaw']-=math.pi
    mappings=[('/clock','/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock','GZ_TO_ROS'),
        ('/marine/odometry','/marine/odometry','nav_msgs/msg/Odometry','gz.msgs.Odometry','GZ_TO_ROS'),
        ('/joint_states','/marine/joint_states','sensor_msgs/msg/JointState','gz.msgs.Model','GZ_TO_ROS'),
        ('/marine/imu','/marine/imu','sensor_msgs/msg/Imu','gz.msgs.IMU','GZ_TO_ROS'),
        ('/marine/auv/propeller_force','/model/tethys_auv/joint/propeller_joint/cmd_thrust','std_msgs/msg/Float64','gz.msgs.Double','ROS_TO_GZ'),
        ('/marine/current','/ocean_current','geometry_msgs/msg/Vector3','gz.msgs.Vector3d','ROS_TO_GZ')]
    for axis in ('horizontal','vertical'):
        mappings.append(('/marine/auv/'+axis+'_fin','/model/tethys_auv/joint/'+axis+'_fins_joint/0/cmd_pos','std_msgs/msg/Float64','gz.msgs.Double','ROS_TO_GZ'))
    if value('slam')=='true':
        for suffix,topic,ros_type,gz_type in [('image','/oakd/rgb/image_raw','sensor_msgs/msg/Image','gz.msgs.Image'),
                ('camera_info','/oakd/rgb/camera_info','sensor_msgs/msg/CameraInfo','gz.msgs.CameraInfo'),
                ('depth_image','/oakd/depth/image_raw','sensor_msgs/msg/Image','gz.msgs.Image')]:
            mappings.append((topic,'/marine/auv/rgbd/'+suffix,ros_type,gz_type,'GZ_TO_ROS'))
    bridge_file=folder/'bridge.yaml';bridge_file.write_text(yaml.safe_dump([dict(ros_topic_name=r,gz_topic_name=g,ros_type_name=rt,gz_type_name=gt,direction=d) for r,g,rt,gt,d in mappings]))
    (folder/'session.json').write_text(json.dumps(dict(robot='tethys_auv',map=name,backend='Gazebo Fortress 6',source_world=str(source_world),
        world=str(world_file),model=str(model_file),description=str(description_file),bridge=str(bridge_file),spawn=pose,calibration=calibration,
        sensor_kit='Authored ideal forward RGB-D kit; no source hull/mass/force changes' if value('slam')=='true' else None,
        scope='Pinned source AUV/native propeller/fins + explicit trim; experimental cruise guidance, independent underwater estimation/SLAM/navigation pending'),indent=2)+'\n')
    print('[robot_lab AUV] '+str(folder/'session.json'))
    partition=os.environ.get('IGN_PARTITION') or os.environ.get('GZ_PARTITION') or 'robot_lab_auv_'+uuid.uuid4().hex[:12]
    resources=os.pathsep.join(filter(None,[os.environ.get('IGN_GAZEBO_RESOURCE_PATH',''),str(robots.parent),str(Path(share('robot_lab_maps')).parent),str(assets/'source')]))
    env=dict(IGN_PARTITION=partition,GZ_PARTITION=partition,IGN_GAZEBO_RESOURCE_PATH=resources,GZ_SIM_RESOURCE_PATH=resources)
    actions=[]
    for key,item in env.items():os.environ[key]=item;actions.append(SetEnvironmentVariable(key,item))
    server=ExecuteProcess(cmd=['ign','gazebo','-r','-v','2',* ([] if value('gui')=='true' else ['-s','--headless-rendering']),str(world_file),'--force-version','6'],
        sigterm_timeout='12',sigkill_timeout='3',output='screen')
    actions += [server,RegisterEventHandler(OnProcessExit(target_action=server,on_exit=[EmitEvent(event=Shutdown(reason='AUV Fortress server exited'))])),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'use_sim_time':True,'robot_description':ParameterValue(description,value_type=str)}],output='screen'),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='auv_bridge',parameters=[{'use_sim_time':True,'config_file':str(bridge_file)}],output='screen'),
        TimerAction(period=2.,actions=[Node(package='ros_gz_sim',executable='create',arguments=['-world',world_name,'-file',str(model_file),'-name','tethys_auv','-allow_renaming=false',
            '-x',str(pose['x']),'-y',str(pose['y']),'-z',str(pose['z']),'-Y',str(pose['yaw'])],output='screen')])]
    controller=Node(package='robot_lab_utils',executable='auv_controller.py',parameters=[{'use_sim_time':True,'calibration_file':str(assets/'calibration.json'),
        'surface_z':float(water['surface_z']),'maximum_depth':float(water['surface_z'])-float(water['bottom_z'])-.6,'control_allowed':value('control')=='true'}],output='screen')
    actions += [controller,RegisterEventHandler(OnProcessExit(target_action=controller,on_exit=[EmitEvent(event=Shutdown(reason='AUV controller owner exited'))]))]
    from robot_lab_utils.marine_pipeline import pipeline
    actions += pipeline(world_file, description_file, pose, water, 'tethys_auv', folder, control=value('control')=='true',
                        slam=value('slam')=='true', database=value('database'), localization=value('localization')=='true')
    if value('rviz')=='true':actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(Path(share('robot_lab_description'))/'rviz/display.rviz'),'-f','map'],parameters=[{'use_sim_time':True}],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map_name',default_value='marine_tank'),DeclareLaunchArgument('control',default_value='true'),
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('slam',default_value='false'),DeclareLaunchArgument('database',default_value=''),
        DeclareLaunchArgument('localization',default_value='false'),
        *[DeclareLaunchArgument('spawn_'+axis,default_value='') for axis in ('x','y','z','yaw')],OpaqueFunction(function=setup)])
