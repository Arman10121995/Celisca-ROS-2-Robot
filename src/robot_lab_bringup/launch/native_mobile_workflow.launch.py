"""Experimental actual Stretch AMCL/SLAM/RTAB-Map/Nav2 stack composition."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    task, robot, map_yaml = value('native_task'), value('robot_model'), value('map_yaml')
    if task not in ('loc', 'slam', '3d_slam', 'nav'):
        raise ValueError('Choose an implemented native workflow: loc/slam/3d_slam/nav')
    if task in ('loc', 'nav') and not Path(map_yaml).is_file():
        raise ValueError('Native localization/navigation requires the selected world occupancy map; generate it in Worlds first')
    common = dict(use_sim_time='true', robot_model='native_stretch', base_frame='native_body_1')
    def include(package, name, arguments):
        path = Path(get_package_share_directory(package))/'launch'/name
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())
    actions = [include('robot_lab_localization', 'local_localization.launch.py',
        {**common, 'odom0': '/odom/ground_truth'})]
    if task in ('loc', 'nav'):
        actions.append(include('robot_lab_localization', 'global_localization.launch.py',
            {**common, 'map_yaml': map_yaml, 'map_name': value('map_name'), 'initial_pose_x': value('spawn_x'),
             'initial_pose_y': value('spawn_y'), 'initial_pose_yaw': value('spawn_yaw'), 'motion_model': 'diff'}))
    elif task == 'slam':
        actions.append(include('robot_lab_mapping', 'slam.launch.py', {**common, 'slam_backend': 'slam_toolbox'}))
    else:
        actions.append(include('robot_lab_mapping', 'rtabmap.launch.py',
            dict(use_sim_time='true', frame_id='native_body_1', robot_name=robot,
                 map_name=value('map_name'), start_rtabmap_viz='false', start_visual_odometry='false')))
    if task == 'nav':
        actions.append(include('robot_lab_navigation', 'navigation.launch.py', common))
    gate = Node(package='robot_lab_bringup', executable='wait_native_sensors.py', output='screen')
    def after_ready(event, _context):
        return actions if event.returncode == 0 else [EmitEvent(event=Shutdown(reason='Native sensor readiness failed'))]
    return [RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=after_ready)), gate]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('native_task'), DeclareLaunchArgument('robot_model'),
        DeclareLaunchArgument('map_yaml', default_value=''), DeclareLaunchArgument('map_name', default_value=''),
        *[DeclareLaunchArgument('spawn_'+axis, default_value='0.') for axis in ('x', 'y', 'yaw')],
        OpaqueFunction(function=setup),
    ])
