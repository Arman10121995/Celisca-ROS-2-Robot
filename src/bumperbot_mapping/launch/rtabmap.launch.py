import os
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("true", "1", "yes", "on")


def _safe_name(value):
    return "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in str(value))


def _auto_database_path(map_name, robot_name):
    base_dir = os.environ.get("BUMPERBOT_RTABMAP_DIR", os.path.join(os.getcwd(), "log", "rtabmap"))
    Path(base_dir).mkdir(parents=True, exist_ok=True)
    return os.path.join(base_dir, f"{_safe_name(map_name)}_{_safe_name(robot_name)}.db")


def _rtabmap_remappings(rgb_topic, depth_topic, camera_info_topic, odom_topic):
    return [
        ("rgb/image", rgb_topic),
        ("depth/image", depth_topic),
        ("rgb/camera_info", camera_info_topic),
        ("odom", odom_topic),
    ]


def _build_rtabmap_actions(context):
    try:
        get_package_share_directory("rtabmap_slam")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "RTAB-Map is required for mode:=3d_slam. Install it with: "
            "sudo apt-get install ros-humble-rtabmap-ros"
        ) from exc

    mapping_share = get_package_share_directory("bumperbot_mapping")

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    rgb_topic = LaunchConfiguration("rgb_topic").perform(context)
    depth_topic = LaunchConfiguration("depth_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    map_frame_id = LaunchConfiguration("map_frame_id").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    map_name = LaunchConfiguration("map_name").perform(context)
    start_visual_odometry = _as_bool(LaunchConfiguration("start_visual_odometry").perform(context))
    start_rtabmap_viz = _as_bool(LaunchConfiguration("start_rtabmap_viz").perform(context))
    delete_db_on_start = _as_bool(LaunchConfiguration("delete_db_on_start").perform(context))
    configured_database_path = LaunchConfiguration("rtabmap_database_path").perform(context)

    database_path = configured_database_path
    if not database_path or database_path.lower() == "auto":
        database_path = _auto_database_path(map_name, robot_name)

    config_path = LaunchConfiguration("rtabmap_config").perform(context)
    if not config_path or config_path.lower() == "auto":
        config_path = os.path.join(mapping_share, "config", "rtabmap_rgbd.yaml")

    effective_odom_topic = "/rtabmap/odom" if start_visual_odometry else odom_topic
    remappings = _rtabmap_remappings(rgb_topic, depth_topic, camera_info_topic, effective_odom_topic)

    common_parameters = [
        config_path,
        {
            "use_sim_time": _as_bool(use_sim_time),
            "frame_id": frame_id,
            "map_frame_id": map_frame_id,
            "odom_frame_id": "odom",
            "database_path": database_path,
            "subscribe_depth": True,
            "subscribe_rgbd": False,
            "approx_sync": True,
        },
    ]

    actions = []

    if start_visual_odometry:
        actions.append(
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                output="screen",
                parameters=common_parameters,
                remappings=_rtabmap_remappings(rgb_topic, depth_topic, camera_info_topic, "/rtabmap/odom"),
            )
        )

    rtabmap_arguments = ["--delete_db_on_start"] if delete_db_on_start else []
    actions.append(
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=common_parameters,
            remappings=remappings,
            arguments=rtabmap_arguments,
        )
    )

    if start_rtabmap_viz:
        actions.append(
            Node(
                package="rtabmap_viz",
                executable="rtabmap_viz",
                name="rtabmap_viz",
                output="screen",
                parameters=common_parameters,
                remappings=remappings,
            )
        )

    return actions


def generate_launch_description():
    mapping_share = get_package_share_directory("bumperbot_mapping")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map_name", default_value="default"),
        DeclareLaunchArgument("robot_name", default_value="bumperbot"),
        DeclareLaunchArgument("rgb_topic", default_value="/oakd/rgb/image_raw"),
        DeclareLaunchArgument("depth_topic", default_value="/oakd/depth/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/oakd/rgb/camera_info"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("frame_id", default_value="base_footprint"),
        DeclareLaunchArgument("map_frame_id", default_value="map"),
        DeclareLaunchArgument(
            "rtabmap_config",
            default_value=os.path.join(mapping_share, "config", "rtabmap_rgbd.yaml"),
            description="RTAB-Map RGB-D configuration YAML.",
        ),
        DeclareLaunchArgument(
            "rtabmap_database_path",
            default_value="auto",
            description="RTAB-Map database output path. 'auto' writes under log/rtabmap.",
        ),
        DeclareLaunchArgument(
            "delete_db_on_start",
            default_value="true",
            choices=["true", "false"],
            description="Start each 3D SLAM run with an empty RTAB-Map database.",
        ),
        DeclareLaunchArgument(
            "start_visual_odometry",
            default_value="false",
            choices=["true", "false"],
            description="Use RTAB-Map RGB-D odometry instead of bumperbot wheel odometry.",
        ),
        DeclareLaunchArgument(
            "start_rtabmap_viz",
            default_value="false",
            choices=["true", "false"],
            description="Start the RTAB-Map native visualization GUI.",
        ),
        OpaqueFunction(function=_build_rtabmap_actions),
    ])
