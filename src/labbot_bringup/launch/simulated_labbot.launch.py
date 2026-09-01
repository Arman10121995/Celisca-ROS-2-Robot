from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_labbot = get_package_share_directory('labbot_bringup')
    pkg_robots = get_package_share_directory('robots')
    urdf = os.path.join(pkg_robots, 'labbot', 'urdf', 'labbot.urdf.xacro')
    return LaunchDescription([
        DeclareLaunchArgument('map_name', default_value='small_office'),
        DeclareLaunchArgument('mode', default_value='nav'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
    ])
