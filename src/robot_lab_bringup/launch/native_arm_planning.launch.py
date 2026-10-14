"""Real upstream MoveIt planning for a source-derived native fixed arm."""
import json
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    for package in ('moveit_ros_move_group', 'moveit_kinematics', 'moveit_planners_ompl'):
        get_package_share_directory(package)
    configuration = LaunchConfiguration('planning_config').perform(context)
    model = json.loads(Path(configuration).read_text())
    use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    group = model['group']
    parameters = {
        'use_sim_time': use_sim,
        'robot_description': Path(model['urdf']).read_text(),
        'robot_description_semantic': Path(model['srdf']).read_text(),
        'robot_description_kinematics': {group: {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': .005,
            'kinematics_solver_timeout': .2, 'position_only_ik': model['position_only_ik']}},
        'robot_description_planning': {'joint_limits': {name: {
            'has_velocity_limits': True, 'max_velocity': float(rate),
            'has_acceleration_limits': True, 'max_acceleration': .06 if unit == 'm' else .6}
            for name, rate, unit in zip(model['joint_names'], model['max_velocity'], model['units'])}},
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': {'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': .01,
            'planner_configs': {'RRTConnect': {'type': 'geometric::RRTConnect', 'range': .1}},
            group: {'planner_configs': ['RRTConnect'], 'longest_valid_segment_fraction': .005}},
        'allow_trajectory_execution': False,
        'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction',
        'publish_robot_description': False, 'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True,
    }
    # Same Humble plugin-lifetime workaround as the existing Panda launch;
    # library residency affects teardown only, never physics/planning limits.
    resident = []
    for package, library in (
        ('moveit_ros_move_group', 'libmoveit_move_group_default_capabilities.so'),
        ('moveit_planners_ompl', 'libmoveit_ompl_planner_plugin.so'),
        ('moveit_kinematics', 'libmoveit_kdl_kinematics_plugin.so'),
        ('moveit_ros_planning', 'libmoveit_default_planning_request_adapter_plugins.so'),
        ('moveit_simple_controller_manager', 'libmoveit_simple_controller_manager.so'),
    ):
        path = Path(get_package_prefix(package))/'lib'/library
        if not path.is_file():
            raise ValueError('Missing native MoveIt plugin: '+str(path))
        resident.append(str(path))
    if os.environ.get('LD_PRELOAD'):
        resident.append(os.environ['LD_PRELOAD'])
    print('[robot_lab] Experimental native arm planning: '+model['source_frame']+' / '+model['urdf'])
    return [
        Node(package='moveit_ros_move_group', executable='move_group', name='move_group',
            output='screen', parameters=[parameters], additional_env={'LD_PRELOAD': ' '.join(resident)}),
        Node(package='robot_lab_bringup', executable='native_arm_scene.py', name='native_arm_scene',
            output='screen', parameters=[dict(use_sim_time=use_sim, planning_config=configuration,
                world_path=LaunchConfiguration('world_path').perform(context),
                spawn_z=float(LaunchConfiguration('spawn_z').perform(context)))]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planning_config'), DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('spawn_z', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='true'), OpaqueFunction(function=_setup),
    ])
