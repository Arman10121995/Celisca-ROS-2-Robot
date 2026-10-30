"""Native laboratory ROV, physical water forces and real sensor/force bridges."""
import json
import os
from pathlib import Path
import tempfile
import uuid
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_share_directory as share, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,EmitEvent,IncludeLaunchDescription,OpaqueFunction,RegisterEventHandler,SetEnvironmentVariable,TimerAction
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _setup(context):
    from robot_lab_utils.marine_runtime import require_fortress_mass_convention
    require_fortress_mass_convention()
    from robot_lab_utils.robot_description import load_description
    from robot_lab_utils.marine_model import MarineModel, SurfaceModel, AmphibiousModel
    from robot_lab_utils.fleet_session import finite
    def value(key):return LaunchConfiguration(key).perform(context)
    def enabled(key):return value(key).lower() in ('true','1','yes')
    from robot_lab_utils.installed_assets import merge_installed_profiles
    maps=merge_installed_profiles(yaml.safe_load((Path(share('robot_lab_bringup'))/'config/sim_maps.yaml').read_text()),'maps')['maps']
    robot=value('robot_model');amphibious=robot=='lab_amphibious'
    name=value('map_name') or 'marine_tank'
    if name not in maps or (not maps[name].get('marine') and not amphibious):
        raise ValueError('ROV control requires an installed marine water world')
    profile=maps[name]; water=profile.get('marine',dict(surface_z=-1000.,bottom_z=-1006.,density=1025.))
    if robot not in ('lab_rov','lab_usv','lab_amphibious'):raise ValueError('Choose a declared native marine/amphibious plant')
    surface=robot in ('lab_usv','lab_amphibious')
    gz=profile['gazebo']
    world=str(Path(share(gz['world_package']))/gz['world_path'])
    world_name=ET.parse(world).getroot().find('world').get('name')
    urdf=load_description(str(Path(share('robot_lab_robots'))/robot/('urdf/'+robot+'.urdf.xacro')),'model_name:='+robot)
    model=(AmphibiousModel if amphibious else SurfaceModel if surface else MarineModel)(urdf,water['density'])
    runtime=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'marine_sessions'
    runtime.mkdir(parents=True,exist_ok=True)
    folder=Path(tempfile.mkdtemp(prefix='rov-',dir=runtime))
    if surface:
        document=ET.fromstring(urdf)
        for joint in ([] if amphibious else model.joints):
            original='/model/'+robot+'/joint/'+joint+'/cmd_thrust'
            requested='/surface/request/'+joint
            guard=ET.SubElement(ET.SubElement(document,'gazebo'),'plugin',name='robot_lab::ThrustWatchdog',
                filename=str(Path(get_package_prefix('robot_lab_maps'))/'lib/librobot_lab_command_watchdog.so'))
            for key,text in dict(input=requested,output=original,timeout_s='.4',max_force='35').items():ET.SubElement(guard,key).text=text
        if amphibious:
            plugin=ET.SubElement(ET.SubElement(document,'gazebo'),'plugin',name='robot_lab::AmphibiousDrive',
                filename=str(Path(get_package_prefix('robot_lab_maps'))/'lib/librobot_lab_amphibious_drive.so'))
            for key,text in dict(surface_z=str(water['surface_z']),wheel_radius='.22',half_track='.49',wheel_torque='8').items():ET.SubElement(plugin,key).text=text
        urdf=ET.tostring(document,encoding='unicode')
        document=ET.parse(world)
        element=document.getroot().find('world')
        if amphibious:
            for system,short in [('Physics','physics'),('UserCommands','user-commands'),('SceneBroadcaster','scene-broadcaster'),('Sensors','sensors'),('Imu','imu')]:
                if not any(p.get('name','').endswith('::'+system) for p in element.findall('plugin')):
                    plugin=ET.SubElement(element,'plugin',name='ignition::gazebo::systems::'+system,filename='ignition-gazebo-'+short+'-system')
                    if system=='Sensors':ET.SubElement(plugin,'render_engine').text='ogre2'
            if not any(p.get('name','').endswith('::Buoyancy') for p in element.findall('plugin')):
                plugin=ET.SubElement(element,'plugin',name='ignition::gazebo::systems::Buoyancy',filename='ignition-gazebo-buoyancy-system')
                graded=ET.SubElement(plugin,'graded_buoyancy');ET.SubElement(graded,'default_density').text=str(water['density'])
                change=ET.SubElement(graded,'density_change');ET.SubElement(change,'above_depth').text=str(water['surface_z']);ET.SubElement(change,'density').text='1.225'
        # Source marine worlds enable one owned model; surface sessions clone
        # them without changing the underwater source or absolute water plane.
        for p in document.getroot().findall('world/plugin'):
            if p.get('name','').endswith('::Buoyancy'):
                for enabled_model in p.findall('enable'):p.remove(enabled_model)
                ET.SubElement(p,'enable').text=robot
        for node in document.getroot().iter():
            if node.tag in ('uri','diffuse','normal','image') and node.text:
                text=node.text.strip()
                if '://' not in text and not Path(text).is_absolute() and (Path(world).parent/text).exists():node.text=str((Path(world).parent/text).resolve())
        surface_world=folder/'world.sdf';document.write(surface_world,encoding='utf-8',xml_declaration=True);world=str(surface_world)
    description=folder/(robot+'.urdf');description.write_text(urdf)
    mappings=[('/clock','/clock','rosgraph_msgs/msg/Clock','gz.msgs.Clock'),
              ('/marine/odometry','/marine/odometry','nav_msgs/msg/Odometry','gz.msgs.Odometry'),
              ('/joint_states','/marine/joint_states','sensor_msgs/msg/JointState','gz.msgs.Model'),
              ('/marine/imu','/marine/imu','sensor_msgs/msg/Imu','gz.msgs.IMU')]
    camera='/world/'+world_name+'/model/'+robot+'/link/oakd_rgb_camera_frame/sensor/rgbd_camera'
    for suffix,ros_topic,ros_type,gz_type in [('/image','/oakd/rgb/image_raw','sensor_msgs/msg/Image','gz.msgs.Image'),
            ('/camera_info','/oakd/rgb/camera_info','sensor_msgs/msg/CameraInfo','gz.msgs.CameraInfo'),
            ('/depth_image','/oakd/depth/image_raw','sensor_msgs/msg/Image','gz.msgs.Image'),
            ('/points','/oakd/points_gz','sensor_msgs/msg/PointCloud2','gz.msgs.PointCloudPacked')]:
        mappings.append((ros_topic,camera+suffix,ros_type,gz_type))
    bridge=[dict(ros_topic_name=r,gz_topic_name=g,ros_type_name=rt,gz_type_name=gt,direction='GZ_TO_ROS') for r,g,rt,gt in mappings]
    if amphibious:
        bridge += [dict(ros_topic_name='/amphibious/plant',gz_topic_name='/amphibious/plant',ros_type_name='std_msgs/msg/String',gz_type_name='gz.msgs.StringMsg',direction='GZ_TO_ROS'),
                   dict(ros_topic_name='/amphibious/wheels',gz_topic_name='/amphibious/wheels',ros_type_name='geometry_msgs/msg/Twist',gz_type_name='gz.msgs.Twist',direction='ROS_TO_GZ')]
    for joint in model.joints:
        bridge.append(dict(ros_topic_name='/marine/thrusters/'+joint+'/force',
                           gz_topic_name='/surface/request/'+joint if surface else '/model/lab_rov/joint/'+joint+'/cmd_thrust',
                           ros_type_name='std_msgs/msg/Float64',gz_type_name='gz.msgs.Double',direction='ROS_TO_GZ'))
    bridge.append(dict(ros_topic_name='/marine/current',gz_topic_name='/ocean_current',
                       ros_type_name='geometry_msgs/msg/Vector3',gz_type_name='gz.msgs.Vector3d',direction='ROS_TO_GZ'))
    bridge_file=folder/'bridge.yaml';bridge_file.write_text(yaml.safe_dump(bridge))
    spawn=dict(profile['spawn'])
    from robot_lab_utils.robot_spawn import map_spawn_override
    robot_profile=yaml.safe_load((Path(share('robot_lab_robots'))/'config/robots.yaml').read_text())['robots'][robot]
    spawn.update(map_spawn_override(robot_profile,name))
    if surface and not amphibious:
        spawn['z']=float(water['surface_z'])
    for axis in ('x','y','z','yaw'):
        supplied=value('spawn_'+axis).strip()
        if supplied.lower() not in ('','auto'):spawn[axis]=finite(supplied,'marine spawn '+axis)
    if amphibious and value('spawn_z').strip().lower() in ('','auto') and 'z' not in map_spawn_override(robot_profile,name):
        if profile.get('marine'):spawn['z']=float(water['surface_z'])+.15
        else:
            from robot_lab_utils.aerial_planning import StaticScene
            from robot_lab_utils.marine_navigation_model import sonar_range
            # Use the reviewed floor/spawn reference, below indoor ceilings;
            # a ray starting at 3 m could select the roof instead of the floor.
            probe=max(.75,float(spawn.get('z',0.))+.6)
            distance=sonar_range(StaticScene(world),(float(spawn['x']),float(spawn['y']),probe),(0.,0.,0.),maximum=200.)
            if not __import__('math').isfinite(distance):raise ValueError('No collision surface at amphibious land spawn')
            spawn['z']=probe-distance+.36
    if surface and not amphibious and abs(float(spawn['z'])-float(water['surface_z']))>.15:
        raise ValueError('Surface vessel spawn must be within 0.15 m of the physical water surface')
    if not surface and not water['bottom_z']+.4<float(spawn['z'])<water['surface_z']-.25:
        raise ValueError('Marine spawn must leave body clearance below the surface and above the seabed')
    manifest=dict(robot=robot,map=name,backend='Gazebo Fortress 6',water=water,spawn=spawn,
                  mass_kg=model.mass,displaced_volume_m3=model.displacement,source_world=world,
                  description=str(description),bridge_file=str(bridge_file),
                  scope='Authored primitive ROV/native marine forces; independent underwater estimation/SLAM/navigation pending')
    (folder/'session.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('[robot_lab marine] '+str(folder/'session.json'))
    resource=os.pathsep.join([os.environ.get('IGN_GAZEBO_RESOURCE_PATH',''),str(Path(share('robot_lab_robots')).parent),str(Path(share('robot_lab_maps')).parent)])
    partition=os.environ.get('IGN_PARTITION') or os.environ.get('GZ_PARTITION') or 'robot_lab_marine_'+uuid.uuid4().hex[:12]
    for key,item in dict(IGN_PARTITION=partition,GZ_PARTITION=partition,IGN_GAZEBO_RESOURCE_PATH=resource,GZ_SIM_RESOURCE_PATH=resource).items():
        os.environ[key]=item
    actions=[SetEnvironmentVariable(key,item) for key,item in dict(IGN_PARTITION=partition,GZ_PARTITION=partition,
              IGN_GAZEBO_RESOURCE_PATH=resource,GZ_SIM_RESOURCE_PATH=resource).items()]
    arguments='-r -v 2 '+('' if enabled('gui') else '-s --headless-rendering ') + world
    actions += [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('ros_gz_sim'))/'launch/gz_sim.launch.py')),
                    launch_arguments={'gz_args':arguments,'on_exit_shutdown':'True'}.items()),
                Node(package='robot_state_publisher',executable='robot_state_publisher',
                     parameters=[{'use_sim_time':True,'robot_description':ParameterValue(urdf,value_type=str)}],output='screen'),
                Node(package='ros_gz_bridge',executable='parameter_bridge',name='marine_bridge',
                     parameters=[{'config_file':str(bridge_file),'use_sim_time':True}],output='screen'),
                TimerAction(period=2.,actions=[Node(package='ros_gz_sim',executable='create',
                    arguments=['-world',world_name,'-file',str(description),'-name',robot,'-allow_renaming=false',
                        '-x',str(spawn['x']),'-y',str(spawn['y']),'-z',str(spawn['z']),'-Y',str(spawn['yaw'])],output='screen')]),
                Node(package='robot_lab_description',executable='gz_pointcloud_to_optical.py',
                     parameters=[{'use_sim_time':True}],output='screen')]
    executable='amphibious_controller.py' if amphibious else 'surface_controller.py' if surface else 'marine_controller.py'
    controller=Node(package='robot_lab_utils',executable=executable,parameters=[{
        'use_sim_time':True,'description_file':str(description),'density':float(water['density']),
        'surface_z':float(water['surface_z']),'robot_id':robot,'maximum_depth':float(water['surface_z'])-float(water['bottom_z'])-.6}],output='screen')
    # Display has no actuator ownership; the observer remains useful for TF,
    # ideal pressure and measured pose, and refuses arm/command operations.
    if not enabled('control'):
        controller=Node(package='robot_lab_utils',executable=executable,parameters=[{
            'use_sim_time':True,'description_file':str(description),'density':float(water['density']),
            'surface_z':float(water['surface_z']),'robot_id':robot,'maximum_depth':5.4,'control_allowed':False}],output='screen')
    actions += [controller,RegisterEventHandler(OnProcessExit(target_action=controller,
                on_exit=[EmitEvent(event=Shutdown(reason='Marine actuator/observer owner exited'))]))]
    from robot_lab_utils.marine_pipeline import pipeline
    actions += pipeline(world, description, spawn, water, robot, folder, control=enabled('control'),
                        slam=enabled('slam'), database=value('database'), localization=enabled('localization'))
    if enabled('rviz'):
        rviz=folder/'marine.rviz'
        rviz.write_text(yaml.safe_dump({'Visualization Manager':{'Global Options':{'Fixed Frame':'map'},'Displays':[
            {'Class':'rviz_default_plugins/RobotModel','Name':'Laboratory ROV','Enabled':True,
             'Description Source':'Topic','Description Topic':'/robot_description'},
            {'Class':'rviz_default_plugins/TF','Name':'Measured marine frames','Enabled':True}]}}))
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(rviz)],parameters=[{'use_sim_time':True}],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_model',default_value='lab_rov'),
        DeclareLaunchArgument('map_name',default_value='marine_tank'),DeclareLaunchArgument('control',default_value='true'),
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('slam',default_value='false'),DeclareLaunchArgument('database',default_value=''),
        DeclareLaunchArgument('localization',default_value='false'),
        *[DeclareLaunchArgument('spawn_'+axis,default_value='') for axis in ('x','y','z','yaw')],
        OpaqueFunction(function=_setup)])
