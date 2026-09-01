"""Nav2 navigation launch, parameterized per robot.

Loads the base Nav2 server configs and, if present, a per-robot overlay from
config/robots/<robot_model>.yaml so frames/footprint/velocity match any robot profile.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("robot_lab_navigation")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    robot_model = LaunchConfiguration("robot_model").perform(context)

    overlay = os.path.join(pkg, "config", "robots", f"{robot_model}.yaml")
    overlay_params = [overlay] if os.path.exists(overlay) else []

    def server(exec_name, name, config_file):
        parameters = [os.path.join(pkg, "config", config_file)] + overlay_params
        parameters.append({"use_sim_time": use_sim_time})
        return Node(
            package="nav2_controller" if exec_name == "controller_server"
            else "nav2_planner" if exec_name == "planner_server"
            else "nav2_smoother" if exec_name == "smoother_server"
            else "nav2_bt_navigator" if exec_name == "bt_navigator"
            else "nav2_behaviors",
            executable=exec_name,
            name=name,
            output="screen",
            parameters=parameters,
        )

    controllers = [
        server("controller_server", "controller_server", "controller_server.yaml"),
        server("planner_server", "planner_server", "planner_server.yaml"),
        server("smoother_server", "smoother_server", "smoother_server.yaml"),
        server("bt_navigator", "bt_navigator", "bt_navigator.yaml"),
        server("behavior_server", "behavior_server", "behavior_server.yaml"),
    ]

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": ["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
        ],
    )

    return controllers + [lifecycle]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "robot_model",
            default_value="bumperbot",
            description="Robot id; loads config/robots/<robot_model>.yaml overlay if present",
        ),
        OpaqueFunction(function=_setup),
    ])
