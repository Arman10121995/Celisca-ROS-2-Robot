from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='vacuum_cleaning', executable='vacuum_cleaner', name='vacuum_cleaner'),
    ])
