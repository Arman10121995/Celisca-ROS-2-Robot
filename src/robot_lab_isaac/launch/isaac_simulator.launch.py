"""Isaac Sim launch adapter for Robot Lab.

This launch file provides the entry point for spawning a robot in
NVIDIA Isaac Sim (or the open-source isaacsim package).  It mirrors
the interface of ``robot_lab_description/launch/gazebo.launch.py``
so that :mod:`robot_lab_bringup` can dispatch to it transparently
based on the ``simulator`` launch argument.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_ISAAC_PKG = "robot_lab_isaac"


def _resolve_world_stage(context, pkg, world_name):
    """Locate the USD/Omniverse stage for the requested map."""
    world_name_str = world_name.perform(context)
    pkg_share = get_package_share_directory(pkg)
    # Prefer a world-specific USD stage under maps package
    candidate = os.path.join(pkg_share, "worlds", world_name_str + ".usd")
    if os.path.exists(candidate):
        return candidate
    # Fallback: look in robot_lab_maps
    try:
        maps_share = get_package_share_directory("robot_lab_maps")
        candidate = os.path.join(
            maps_share, "worlds", world_name_str + ".usd"
        )
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    # Ultimate fallback: empty stage
    description_share = get_package_share_directory("robot_lab_description")
    return os.path.join(description_share, "worlds", "empty.usd")


def _build_isaac_actions(context):
    """Construct Isaac Sim nodes and include the base description launch."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_package = LaunchConfiguration("robot_package")
    robot_xacro = LaunchConfiguration("robot_xacro")
    robot_name = LaunchConfiguration("robot_name")
    world_name = LaunchConfiguration("world_name")
    world_package = LaunchConfiguration("world_package")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    actions = []

    # Resolve the USD stage for the world
    stage_path = _resolve_world_stage(context, world_package.perform(context), world_name)

    # Robot description + state publisher (mirrors gazebo.launch.py).
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )
    actions.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        )
    )

    # Start the Isaac Sim core (carb/kit-based).  When isaacsim is not
    # installed the node exits gracefully with a clear log message.
    actions.append(
        Node(
            package="robot_lab_isaac",
            executable="isaac_spawner",
            name="isaac_spawner",
            output="screen",
            parameters=[{
                "world_stage": stage_path,
                "robot_name": robot_name,
                "robot_package": robot_package,
                "robot_xacro": robot_xacro,
                "spawn_x": spawn_x,
                "spawn_y": spawn_y,
                "spawn_z": spawn_z,
                "spawn_yaw": spawn_yaw,
                "use_sim_time": use_sim_time,
            }],
        )
    )

    # Sensor bridge: converts Isaac Omniverse sensor data to ROS 2 topics
    actions.append(
        Node(
            package="robot_lab_isaac",
            executable="sensor_bridge",
            name="isaac_sensor_bridge",
            output="screen",
            parameters=[{
                "robot_name": robot_name,
                "use_sim_time": use_sim_time,
            }],
        )
    )

    return actions


def generate_launch_description():
    description_share = get_package_share_directory("robot_lab_description")

    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="empty"),
        DeclareLaunchArgument("world_package", default_value="robot_lab_maps"),
        DeclareLaunchArgument("world_path", default_value=""),
        DeclareLaunchArgument("model"),
        DeclareLaunchArgument("robot_package", default_value="robot_lab_robots"),
        DeclareLaunchArgument("robot_xacro"),
        DeclareLaunchArgument("robot_name", default_value="bumperbot"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_build_isaac_actions),
    ])
