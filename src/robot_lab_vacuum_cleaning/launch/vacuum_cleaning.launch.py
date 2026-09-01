from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_lab_vacuum_cleaning', executable='vacuum_cleaner', name='vacuum_cleaner'),
    ])
