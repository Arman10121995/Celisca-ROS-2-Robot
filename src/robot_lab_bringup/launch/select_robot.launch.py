#!/usr/bin/env python3
"""
Launch file for selecting a robot.

This launch file uses the robot selector to choose and configure a robot.
It can be used independently or as part of a larger composition.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from robot_lab_bringup.selectors import RobotSelector


def generate_launch_description():
    """Generate launch description for robot selection."""
    
    # Declare launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='bumperbot',
        description='ID of the robot to launch'
    )
    
    # Get robot from selector
    selector = RobotSelector()
    robot_id = LaunchConfiguration('robot_id')
    
    # For now, just print the selection
    # In P2.3, this will be expanded to actually launch the robot
    print_node = Node(
        package='robot_lab_bringup',
        executable='robot-lab-select',
        name='robot_selector',
        output='screen',
        arguments=['describe', 'robot', LaunchConfiguration('robot_id')]
    )
    
    return LaunchDescription([
        robot_id_arg,
        print_node,
    ])
