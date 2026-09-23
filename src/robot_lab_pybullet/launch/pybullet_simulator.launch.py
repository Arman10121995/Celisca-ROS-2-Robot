"""PyBullet launch adapter for Robot Lab."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _build_pybullet_actions(context):
    """Construct PyBullet nodes."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_package = LaunchConfiguration("robot_package")
    robot_xacro = LaunchConfiguration("robot_xacro")
    robot_name = LaunchConfiguration("robot_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    world_path = LaunchConfiguration("world_path")
    gui = LaunchConfiguration("gui")
    # Display-mode hold: freeze the robot in its spawn pose.  'auto' lets the
    # spawner decide (hold only in map-free display), 'true' forces the hold
    # (passive display of legged/humanoid robots), 'false' forces full
    # physics.  Bringup forwards mode:=display automatically.
    try:
        hold_position = LaunchConfiguration("hold_position")
    except Exception:
        hold_position = None

    actions = []

    # A robot-free display run (robot_model:=none) passes an empty model:
    # there is no description to publish and nothing to spawn, so only the
    # world is shown.  Building the xacro command anyway would fail the
    # whole launch.
    model_path = LaunchConfiguration("model").perform(context).strip()
    spawn_robot = bool(model_path) and model_path.lower() != "none"

    if spawn_robot:
        # Robot description + state publisher (mirrors gazebo.launch.py: process
        # the xacro model and publish robot_description + TF).
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

    # PyBullet physics engine + robot spawn
    spawner_params = {
        "model": LaunchConfiguration("model"),
        "robot_name": robot_name,
        "robot_package": robot_package,
        "robot_xacro": robot_xacro,
        "spawn_x": ParameterValue(spawn_x, value_type=float),
        "spawn_y": ParameterValue(spawn_y, value_type=float),
        "spawn_z": ParameterValue(spawn_z, value_type=float),
        "spawn_yaw": ParameterValue(spawn_yaw, value_type=float),
        "use_sim_time": use_sim_time,
        "world_path": world_path,
        "gui": gui,
    }
    for drive_key in ("left_wheel_joint", "right_wheel_joint", "wheel_radius", "wheel_separation"):
        spawner_params[drive_key] = LaunchConfiguration(drive_key)
    if hold_position is not None:
        spawner_params["hold_position"] = ParameterValue(hold_position, value_type=str)
    actions.append(
        Node(
            package="robot_lab_pybullet",
            executable="pybullet_spawner",
            name="pybullet_spawner",
            output="screen",
            parameters=[spawner_params],
        )
    )

    # NOTE: The PyBullet spawner already publishes joint_states, odom,
    # TF, scan, imu, and clock directly.  No separate sensor_bridge needed.

    return actions


def generate_launch_description():
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
            description="Display-mode joint hold: 'auto' holds only map-free "
                        "display, 'true' always holds joints at spawn, "
                        "'false' runs full physics.",
        ),
        OpaqueFunction(function=_build_pybullet_actions),
    ])
