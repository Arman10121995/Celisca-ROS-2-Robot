"""Upstream MoveIt OMPL/KDL over the exact native Panda planning model."""
import os
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    get_package_share_directory('moveit_ros_move_group')
    get_package_share_directory('moveit_kinematics')
    get_package_share_directory('moveit_planners_ompl')
    from robot_lab_utils.native_arm_description import panda_planning_model
    runtime = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', os.environ.get('TMPDIR', str(Path.cwd()/'log'))))
    directory = runtime/'arm_planning'/str(time.time_ns())
    spawn = tuple(float(LaunchConfiguration('spawn_'+axis).perform(context)) for axis in ('x', 'y', 'z', 'yaw'))
    model = panda_planning_model(LaunchConfiguration('native_mjcf').perform(context), directory, spawn)
    use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    parameters = {
        'use_sim_time': use_sim,
        'robot_description': Path(model['urdf']).read_text(),
        'robot_description_semantic': Path(model['srdf']).read_text(),
        'robot_description_kinematics': {'panda_arm': {
            'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
            'kinematics_solver_search_resolution': .005,
            'kinematics_solver_timeout': .2}},
        'robot_description_planning': {'joint_limits': {name: {
            'has_velocity_limits': True, 'max_velocity': .35,
            'has_acceleration_limits': True, 'max_acceleration': .7} for name in model['joint_names']}},
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': .01,
            'planner_configs': {'RRTConnect': {'type': 'geometric::RRTConnect', 'range': .1}},
            'panda_arm': {'planner_configs': ['RRTConnect'], 'longest_valid_segment_fraction': .005}},
        # The GUI executes collision-checked path positions through the
        # existing bounded position-only native action, with its heartbeat,
        # Stop and cancel contracts. Do not send OMPL velocity fields to it.
        'allow_trajectory_execution': False,
        # No MoveIt action/controller client: the GUI exclusively owns native
        # execution. Humble's unused controller client also crashed in teardown.
        'disable_capabilities': 'move_group/MoveGroupExecuteTrajectoryAction move_group/MoveGroupMoveAction',
        'publish_robot_description': False,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    # Humble 2.5.x can unload plugin-created callback deleters before the
    # shared ROS node is destroyed (MoveIt issue #1597). Keep only this
    # planning process's real plugins resident until process exit. This
    # changes library lifetime, not planning, controllers or simulator state.
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
    print('[robot_lab] Native Panda MoveIt description: '+model['urdf'])
    return [
        Node(package='moveit_ros_move_group', executable='move_group', name='move_group',
             output='screen', parameters=[parameters], additional_env={'LD_PRELOAD': ' '.join(resident)}),
        Node(package='robot_lab_bringup', executable='native_arm_scene.py', name='native_arm_scene',
             output='screen', parameters=[{'use_sim_time': use_sim,
                'world_path': LaunchConfiguration('world_path').perform(context),
                'spawn_z': spawn[2], 'grasp_fixture': LaunchConfiguration('grasp_fixture').perform(context).lower() == 'true'}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('native_mjcf'),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('grasp_fixture', default_value='false'),
        *[DeclareLaunchArgument('spawn_'+axis, default_value='0.0') for axis in ('x', 'y', 'z', 'yaw')],
        OpaqueFunction(function=_setup),
    ])
