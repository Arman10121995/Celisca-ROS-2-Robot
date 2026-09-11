"""MuJoCo launch adapter for Robot Lab."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_mujoco_xml(context, world_name):
    """Locate the MuJoCo MJCF/XML for the requested map.

    MJCF worlds are generated from the Gazebo ``.world`` files by
    ``robot_lab_maps/tools/gen_mjcf_worlds.py`` and named after the world.
    The ``world_path`` basename is tried as well so a map whose sim_maps key
    differs from its world name (``outdoor_terrain`` -> ``terrain_rough``)
    still finds its world.  Falling back to the empty stage is reported, not
    silent, because an unnoticed fallback looks exactly like a map that
    failed to load.
    """
    maps_share = get_package_share_directory("robot_lab_maps")
    candidates = [world_name.perform(context)]
    world_path = LaunchConfiguration("world_path").perform(context)
    if world_path:
        candidates.append(
            os.path.splitext(os.path.basename(world_path))[0])
    for name in candidates:
        if not name or name == "none":
            continue
        candidate = os.path.join(maps_share, "mjcf", name + ".xml")
        if os.path.exists(candidate):
            return candidate
    print("[robot_lab_mujoco] no MJCF world for %s; using the empty stage. "
          "Run robot_lab_maps/tools/gen_mjcf_worlds.py to generate it."
          % (candidates[0] or "<unnamed>"))
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
    gui = LaunchConfiguration("gui")

    actions = []

    # A robot-free display run (robot_model:=none) passes an empty model:
    # there is no description to publish and nothing to spawn, so only the
    # world is shown.  Building the xacro command anyway would fail the
    # whole launch.
    model_path = LaunchConfiguration("model").perform(context).strip()
    spawn_robot = bool(model_path) and model_path.lower() != "none"

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
                "gui": gui,
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
        OpaqueFunction(function=_build_mujoco_actions),
    ])
