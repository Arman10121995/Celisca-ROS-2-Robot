"""MuJoCo launch adapter for Robot Lab."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_mujoco_xml(context, world_name):
    """Locate the MuJoCo MJCF/XML for the requested map."""
    world_name_str = world_name.perform(context)
    maps_share = get_package_share_directory("robot_lab_maps")
    candidate = os.path.join(maps_share, "mjcf", world_name_str + ".xml")
    if os.path.exists(candidate):
        return candidate
    return os.path.join(maps_share, "mjcf", "empty.xml")


def _build_mujoco_actions(context):
    """Construct MuJoCo nodes."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_package = LaunchConfiguration("robot_package")
    robot_xacro = LaunchConfiguration("robot_xacro")
    robot_name = LaunchConfiguration("robot_name")
    world_name = LaunchConfiguration("world_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    actions = []

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

    mujoco_xml = _resolve_mujoco_xml(context, world_name)

    # MuJoCo physics engine + robot spawn
    actions.append(
        Node(
            package="robot_lab_mujoco",
            executable="mujoco_spawner",
            name="mujoco_spawner",
            output="screen",
            parameters=[{
                "model": LaunchConfiguration("model"),
                "world_xml": mujoco_xml,
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

    # NOTE: The MuJoCo spawner already publishes joint_states, odom,
    # TF, scan, imu, and clock directly.  No separate sensor_bridge needed.

    return actions


def generate_launch_description():
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
        OpaqueFunction(function=_build_mujoco_actions),
    ])
