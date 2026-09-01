from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
import os

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )

    use_python_imu_republisher_arg = DeclareLaunchArgument(
        "use_python_imu_republisher",
        default_value="True",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_python_imu_republisher = LaunchConfiguration("use_python_imu_republisher")

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(get_package_share_directory("robot_lab_localization"), "config", "ekf.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    imu_republisher_py = Node(
        package="robot_lab_localization",
        executable="imu_republisher.py",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_python_imu_republisher),
    )

    imu_republisher_cpp = Node(
        package="robot_lab_localization",
        executable="imu_republisher",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_python_imu_republisher),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_python_imu_republisher_arg,
        robot_localization,
        imu_republisher_py,
        imu_republisher_cpp,   
    ])
