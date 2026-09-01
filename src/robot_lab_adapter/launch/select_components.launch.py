#!/usr/bin/env python3
"""
Launch file for selecting algorithm components.

This launch file demonstrates independent selection of perception,
localization, state estimation, global planning, local planning, and control
algorithms.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for component selection."""
    
    # Declare launch arguments for all algorithm categories
    algorithm_args = [
        DeclareLaunchArgument('perception', default_value='', description='Perception algorithm ID'),
        DeclareLaunchArgument('localization', default_value='', description='Localization algorithm ID'),
        DeclareLaunchArgument('state_estimation', default_value='', description='State estimation algorithm ID'),
        DeclareLaunchArgument('sensor_fusion', default_value='', description='Sensor fusion algorithm ID'),
        DeclareLaunchArgument('global_planning', default_value='', description='Global planning algorithm ID'),
        DeclareLaunchArgument('local_planning', default_value='', description='Local planning algorithm ID'),
        DeclareLaunchArgument('control', default_value='', description='Control algorithm ID'),
    ]
    
    # Log the selections
    log_action = LogInfo(
        msg="Selected components: "
            + "perception=" + LaunchConfiguration('perception') + ", "
            + "localization=" + LaunchConfiguration('localization') + ", "
            + "state_estimation=" + LaunchConfiguration('state_estimation') + ", "
            + "sensor_fusion=" + LaunchConfiguration('sensor_fusion') + ", "
            + "global_planning=" + LaunchConfiguration('global_planning') + ", "
            + "local_planning=" + LaunchConfiguration('local_planning') + ", "
            + "control=" + LaunchConfiguration('control')
    )
    
    # Validate composition
    validate_node = Node(
        package='robot_lab_adapter',
        executable='robot-lab-select',
        name='composition_validator',
        output='screen',
        arguments=[
            'validate',
            '--perception', LaunchConfiguration('perception'),
            '--localization', LaunchConfiguration('localization'),
            '--state-estimation', LaunchConfiguration('state_estimation'),
            '--sensor-fusion', LaunchConfiguration('sensor_fusion'),
            '--global-planning', LaunchConfiguration('global_planning'),
            '--local-planning', LaunchConfiguration('local_planning'),
            '--control', LaunchConfiguration('control'),
        ]
    )
    
    return LaunchDescription([
        *algorithm_args,
        log_action,
        validate_node,
    ])
