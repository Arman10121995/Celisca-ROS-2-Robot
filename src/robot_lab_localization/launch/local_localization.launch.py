"""Local localization (EKF + IMU republisher), parameterized per robot.

Loads the base ekf.yaml and, if present, a per-robot overlay from
config/robots/<robot_model>.yaml so frames match any robot profile.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    localization_share = get_package_share_directory("robot_lab_localization")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    use_python_imu_republisher = LaunchConfiguration("use_python_imu_republisher").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)

    parameters = [os.path.join(localization_share, "config", "ekf.yaml")]
    overlay = os.path.join(localization_share, "config", "robots", f"{robot_model}.yaml")
    if os.path.exists(overlay):
        parameters.append(overlay)
    parameters.append({"use_sim_time": use_sim_time})

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=parameters,
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

    return [robot_localization, imu_republisher_py, imu_republisher_cpp]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_python_imu_republisher", default_value="True"),
        DeclareLaunchArgument(
            "robot_model",
            default_value="bumperbot",
            description="Robot id; loads config/robots/<robot_model>.yaml overlay if present",
        ),
        OpaqueFunction(function=_setup),
    ])
