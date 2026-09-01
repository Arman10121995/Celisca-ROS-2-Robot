"""2D SLAM (slam_toolbox) launch, parameterized per robot.

Loads the base slam_toolbox.yaml and, if present, a per-robot overlay from
config/robots/<robot_model>.yaml so frames/topics match any robot profile.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    mapping_share = get_package_share_directory("robot_lab_mapping")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    slam_config = LaunchConfiguration("slam_config").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)

    parameters = [slam_config]
    overlay = os.path.join(mapping_share, "config", "robots", f"{robot_model}.yaml")
    if os.path.exists(overlay):
        parameters.append(overlay)
    parameters.append({"use_sim_time": use_sim_time})

    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65},
        ],
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=parameters,
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": ["map_saver_server"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
        ],
    )

    return [nav2_map_saver, slam_toolbox, nav2_lifecycle_manager]


def generate_launch_description():
    mapping_share = get_package_share_directory("robot_lab_mapping")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "slam_config",
            default_value=os.path.join(mapping_share, "config", "slam_toolbox.yaml"),
            description="Full path to slam yaml file to load",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="bumperbot",
            description="Robot id; loads config/robots/<robot_model>.yaml overlay if present",
        ),
        OpaqueFunction(function=_setup),
    ])
