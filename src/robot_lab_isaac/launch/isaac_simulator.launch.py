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


def _resolve_world_sdf(context, pkg, world_name):
    """Locate the SDF ``.world`` file for the requested map."""
    world_name_str = world_name.perform(context)
    try:
        maps_share = get_package_share_directory("robot_lab_maps")
        candidate = os.path.join(
            maps_share, "maps", world_name_str, "worlds",
            world_name_str + ".world",
        )
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    try:
        pkg_share = get_package_share_directory(pkg)
        candidate = os.path.join(
            pkg_share, "maps", world_name_str, "worlds",
            world_name_str + ".world",
        )
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass
    return ""


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
    try:
        hold_position = LaunchConfiguration("hold_position")
    except Exception:
        hold_position = None

    actions = []

    # A robot-free display run (robot_model:=none) passes an empty model:
    # there is no description to publish and nothing to spawn, so only the
    # world is shown.  Building the xacro command anyway would fail the
    # whole launch (this was the "map-only display does nothing" bug:
    # xacro on an empty path aborts before the world ever loads).
    model_path = LaunchConfiguration("model").perform(context).strip()
    spawn_robot = bool(model_path) and model_path.lower() != "none"

    # Resolve the SDF .world file first (preferred for mesh-based maps).
    sdf_path = _resolve_world_sdf(context, world_package.perform(context), world_name)
    # Honour an explicit world_path override from bringup (the dispatcher
    # resolves map_name -> world file); fall back to a USD stage only when
    # no SDF world is available.
    explicit_world = LaunchConfiguration("world_path").perform(context).strip()
    if explicit_world and os.path.exists(explicit_world):
        sdf_path = explicit_world
    # Only fall back to a USD stage if no SDF world is available.
    stage_path = "" if sdf_path else _resolve_world_stage(context, world_package.perform(context), world_name)

    if spawn_robot:
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

    spawner_params = {
        "world_stage": stage_path,
        "world_path": sdf_path,
        "model": LaunchConfiguration("model"),
        "robot_name": robot_name,
        "robot_package": robot_package,
        "robot_xacro": robot_xacro,
        "spawn_x": ParameterValue(spawn_x, value_type=float),
        "spawn_y": ParameterValue(spawn_y, value_type=float),
        "spawn_z": ParameterValue(spawn_z, value_type=float),
        "spawn_yaw": ParameterValue(spawn_yaw, value_type=float),
        "use_sim_time": use_sim_time,
        "gui": LaunchConfiguration("gui"),
    }
    for drive_key in ("left_wheel_joint", "right_wheel_joint", "wheel_radius", "wheel_separation"):
        spawner_params[drive_key] = LaunchConfiguration(drive_key)
    if hold_position is not None:
        try:
            spawner_params["hold_position"] = ParameterValue(hold_position, value_type=str)
        except Exception:
            pass
    # Start the Isaac Sim core (carb/kit-based).  When isaacsim is not
    # installed the node exits gracefully with a clear log message.
    actions.append(
        Node(
            package="robot_lab_isaac",
            executable="isaac_spawner",
            name="isaac_spawner",
            output="screen",
            parameters=[spawner_params],
        )
    )

    if spawn_robot:
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
        DeclareLaunchArgument("left_wheel_joint", default_value="wheel_left_joint"),
        DeclareLaunchArgument("right_wheel_joint", default_value="wheel_right_joint"),
        DeclareLaunchArgument("wheel_radius", default_value="0.033"),
        DeclareLaunchArgument("wheel_separation", default_value="0.17"),
        DeclareLaunchArgument("world_name", default_value="empty"),
        DeclareLaunchArgument("world_package", default_value="robot_lab_maps"),
        DeclareLaunchArgument("world_path", default_value=""),
        DeclareLaunchArgument("model", default_value=""),
        DeclareLaunchArgument("robot_package", default_value="robot_lab_robots"),
        DeclareLaunchArgument("robot_xacro", default_value=""),
        DeclareLaunchArgument("robot_name", default_value="bumperbot"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument(
            "hold_position", default_value="false",
            description="Display-mode joint hold forwarded to the runtime.",
        ),
        OpaqueFunction(function=_build_isaac_actions),
    ])
