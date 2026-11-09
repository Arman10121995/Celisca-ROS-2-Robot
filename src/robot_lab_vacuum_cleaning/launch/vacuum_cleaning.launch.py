from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = {
        'robot_model': 'oomwoo',
        'map_key': 'default',
        'use_sim_time': 'true',
        'auto_start': '',
        'report_path': '',
        'battery_time_scale': '-1.0',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in args.items()],
        Node(
            package='robot_lab_vacuum_cleaning', executable='vacuum_cleaner',
            name='vacuum_cleaner', output='screen',
            parameters=[{
                'robot_model': LaunchConfiguration('robot_model'),
                'map_key': LaunchConfiguration('map_key'),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=str),
                'report_path': LaunchConfiguration('report_path'),
                'battery_time_scale': ParameterValue(LaunchConfiguration('battery_time_scale'), value_type=float),
            }],
        ),
    ])
