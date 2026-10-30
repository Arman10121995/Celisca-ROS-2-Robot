"""One Gazebo plant/clock with source Bumperbot/Labbot instance controllers.

This intentionally does not include simulated_robot once per member: that
would start independent worlds and collide on topics, frames and reset services.
"""
import os
from pathlib import Path
import uuid

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _setup(context):
    from robot_lab_utils.fleet_session import prepare_session
    def value(name):
        return LaunchConfiguration(name).perform(context)
    def enabled(name):
        return value(name).lower() in ('true', '1', 'yes')
    if value('simulator') != 'gazebo':
        raise ValueError('Shared-world adapters currently support Gazebo only; other backends are pending')
    session = prepare_session(value('session'), value('map_name'), enabled('navigation'))
    if enabled('rmf'):
        import importlib.util
        if not session['navigation']:
            raise ValueError('Open-RMF requires navigation:=true')
        if importlib.util.find_spec('rmf_adapter') is None:
            raise ValueError('Open-RMF runtime missing; provision scripts/provision_rmf_runtime.py on SSD and source scripts/rmf_env.sh')
    print('[robot_lab fleet] Resolved shared-world session: ' + session['resolved_file'])
    actions = []
    partition = os.environ.get('GZ_PARTITION') or os.environ.get('IGN_PARTITION') or 'robot_lab_fleet_' + uuid.uuid4().hex[:12]
    resource_paths = [os.environ.get('GZ_SIM_RESOURCE_PATH', ''), os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]
    for package in ('robot_lab_robots', 'robot_lab_maps', 'robot_lab_models', 'robot_lab_description'):
        directory = Path(share(package))
        resource_paths += [str(directory.parent), str(directory / 'maps'), str(directory / 'models')]
    environments = dict(GZ_PARTITION=partition, IGN_PARTITION=partition,
                        GZ_SIM_RESOURCE_PATH=os.pathsep.join(filter(None, resource_paths)),
                        IGN_GAZEBO_RESOURCE_PATH=os.pathsep.join(filter(None, resource_paths)),
                        GZ_SIM_SYSTEM_PLUGIN_PATH=os.pathsep.join(filter(None, [
                            '/opt/ros/humble/lib', os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')])),
                        IGN_GAZEBO_SYSTEM_PLUGIN_PATH=os.pathsep.join(filter(None, [
                            '/opt/ros/humble/lib', os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')])))
    for key, item in environments.items():
        os.environ[key] = item  # ros_gz_sim's OpaqueFunction also reads the environment.
        actions.append(SetEnvironmentVariable(key, item))
    server = ExecuteProcess(cmd=['ign', 'gazebo', '-r', '-v', '2',
        *([] if enabled('gui') else ['-s', '--headless-rendering']), session['world'], '--force-version', '6'],
        sigterm_timeout='12', sigkill_timeout='3', output='screen')
    actions += [server, RegisterEventHandler(OnProcessExit(target_action=server,
        on_exit=[EmitEvent(event=Shutdown(reason='Shared-world Fortress server exited'))]))]
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='fleet_sensor_bridge',
                        parameters=[{'config_file': session['bridge_file'], 'use_sim_time': True}], output='screen'))
    if session.get('infrastructure_file'):
        infrastructure=Node(package='robot_lab_utils',executable='fleet_infrastructure.py',
            arguments=['--session',session['resolved_file']],parameters=[dict(use_sim_time=True)],output='screen')
        actions += [infrastructure,RegisterEventHandler(OnProcessExit(target_action=infrastructure,
            on_exit=[EmitEvent(event=Shutdown(reason='Physical facility workflow owner exited'))]))]
    actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='fleet_pose_service_bridge',
                        arguments=['/world/' + session['world_name'] + '/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
                        output='screen'))
    if session['map_yaml']:
        actions += [Node(package='nav2_map_server', executable='map_server', name='map_server',
                         parameters=[{'yaml_filename': session['map_yaml'], 'use_sim_time': True}], output='screen'),
                    Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='fleet_map_lifecycle',
                         parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}], output='screen')]
    robot_displays = []
    controller_spawners = []
    for index, member in enumerate(session['members']):
        name, pose = member['id'], member['spawn']
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            name='robot_state_publisher', namespace=name,
                            parameters=[{'use_sim_time': True, 'frame_prefix': name + '/',
                                         'robot_description': ParameterValue(Path(member['description']).read_text(), value_type=str)}],
                            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')], output='screen'))
        actions.append(TimerAction(period=2.0 + index * .4, actions=[Node(package='ros_gz_sim', executable='create',
            name='spawn_' + name, namespace=name, arguments=['-world', session['world_name'], '-file', member['description'],
                '-name', name, '-allow_renaming=false', '-x', str(pose['x']), '-y', str(pose['y']),
                '-z', str(pose['z']), '-Y', str(pose['yaw'])], output='screen')]))
        controller_spawners.append(Node(package='controller_manager', executable='spawner',
            name='controllers_' + name, namespace=name,
            arguments=['joint_state_broadcaster', 'robot_lab_controller', '--controller-manager', '/' + name + '/controller_manager',
                       '--controller-manager-timeout', '60', '--param-file', member['controller_file']],
            parameters=[{'use_sim_time': True}], output='screen'))
        actions.append(Node(package='robot_lab_description', executable='gz_pointcloud_to_optical.py',
            name='pointcloud_to_optical', namespace=name,
            parameters=[{'use_sim_time': True, 'input_topic': '/' + name + '/oakd/points_gz',
                         'output_topic': '/' + name + '/oakd/points', 'frame_id': name + '/oakd_rgb_camera_optical_frame'}],
            output='screen'))
        if session['navigation']:
            servers = [('nav2_amcl', 'amcl'), ('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
                       ('nav2_smoother', 'smoother_server'), ('nav2_bt_navigator', 'bt_navigator'), ('nav2_behaviors', 'behavior_server')]
            navigation_actions = [Node(package=package, executable=node, name=node, namespace=name,
                parameters=[member['navigation_file']],
                remappings=[('tf', '/tf'), ('tf_static', '/tf_static'), ('map', '/map')],
                output='screen') for package, node in servers]
            navigation_actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager', namespace=name, parameters=[{'use_sim_time': True, 'autostart': True,
                    'node_names': [node for _, node in servers]}], output='screen'))
            actions.append(TimerAction(period=8. + index * .3, actions=navigation_actions))
        robot_displays += [dict(Class='rviz_default_plugins/RobotModel', Name=name + ' robot', Enabled=True,
                               **{'Description Source': 'Topic', 'Description Topic': '/' + name + '/robot_description', 'TF Prefix': name}),
                           dict(Class='rviz_default_plugins/LaserScan', Name=name + ' scan', Enabled=True,
                                Topic={'Value': '/' + name + '/scan', 'Reliability Policy': 'Best Effort'})]
    fleet_controller = Node(package='robot_lab_utils', executable='fleet_controller.py', name='fleet_controller',
                            parameters=[{'session_file': session['resolved_file'], 'use_sim_time': True}], output='screen')
    # Loading the same controller library concurrently in several embedded
    # managers can race class_loader. Configure/activate each manager in
    # sequence and explicitly pass its parameter file before on_init.
    for index, spawner in enumerate(controller_spawners):
        following = controller_spawners[index + 1] if index + 1 < len(controller_spawners) else fleet_controller
        def after_controller(event, _context, next_action=following):
            if event.returncode:
                return [EmitEvent(event=Shutdown(reason='Shared-world controller startup failed'))]
            return [next_action]
        actions.append(RegisterEventHandler(OnProcessExit(target_action=spawner, on_exit=after_controller)))
    actions.append(RegisterEventHandler(OnProcessExit(target_action=fleet_controller,
        on_exit=[EmitEvent(event=Shutdown(reason='Shared-world command owner exited'))])))
    actions.append(TimerAction(period=3., actions=[controller_spawners[0]]))
    if enabled('rmf'):
        actions.append(Node(package='rmf_traffic_ros2', executable='rmf_traffic_schedule',
                            name='robot_lab_rmf_schedule', parameters=[{'use_sim_time': True}], output='screen'))
        actions.append(TimerAction(period=5., actions=[Node(package='robot_lab_utils', executable='fleet_rmf_adapter.py',
            arguments=['--session', session['resolved_file']], output='screen')]))
    if enabled('rviz'):
        rviz = Path(session['directory']) / 'fleet.rviz'
        rviz.write_text(yaml.safe_dump({'Visualization Manager': {
            'Global Options': {'Fixed Frame': 'map'}, 'Displays': robot_displays + [
                {'Class': 'rviz_default_plugins/TF', 'Name': 'Fleet TF', 'Enabled': True},
                {'Class': 'rviz_default_plugins/Map', 'Name': 'Shared map', 'Enabled': True,
                 'Topic': {'Value': '/map', 'Durability Policy': 'Transient Local'}}]}}))
        actions.append(Node(package='rviz2', executable='rviz2', name='fleet_rviz', arguments=['-d', str(rviz)],
                            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session', default_value=str(Path(share('robot_lab_bringup')) / 'config/fleet_demo.yaml')),
        DeclareLaunchArgument('map_name', default_value='', description='Registered map; empty uses session map'),
        DeclareLaunchArgument('simulator', default_value='gazebo'),
        DeclareLaunchArgument('navigation', default_value='false', description='One AMCL/Nav2 stack per instance, one shared map'),
        DeclareLaunchArgument('rmf', default_value='false', description='Actual optional Open-RMF schedule and fleet adapter'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
