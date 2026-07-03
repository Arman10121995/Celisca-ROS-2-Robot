import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:
    import yaml
except ImportError:
    yaml = None


MODE_ALIASES = {
    "nave": "nav",
}


def _package_file(package_name, relative_path):
    if not relative_path:
        return ""

    path = Path(str(relative_path))
    if path.is_absolute():
        return str(path)

    return os.path.join(get_package_share_directory(package_name), *path.parts)


def _strip_yaml_comment(line):
    quote = None
    escaped = False
    for index, char in enumerate(line):
        if escaped:
            escaped = False
            continue
        if char == "\\":
            escaped = True
            continue
        if char in ("'", '"'):
            if quote == char:
                quote = None
            elif quote is None:
                quote = char
            continue
        if char == "#" and quote is None:
            return line[:index]
    return line


def _split_inline_list(value):
    items = []
    current = []
    quote = None
    escaped = False
    for char in value:
        if escaped:
            current.append(char)
            escaped = False
            continue
        if char == "\\":
            current.append(char)
            escaped = True
            continue
        if char in ("'", '"'):
            current.append(char)
            if quote == char:
                quote = None
            elif quote is None:
                quote = char
            continue
        if char == "," and quote is None:
            items.append("".join(current).strip())
            current = []
            continue
        current.append(char)
    items.append("".join(current).strip())
    return items


def _parse_yaml_scalar(value):
    value = value.strip()
    if not value:
        return {}

    if len(value) >= 2 and value[0] == value[-1] and value[0] in ("'", '"'):
        return value[1:-1]

    if value.startswith("[") and value.endswith("]"):
        content = value[1:-1].strip()
        if not content:
            return []
        return [_parse_yaml_scalar(item) for item in _split_inline_list(content)]

    normalized = value.lower()
    if normalized in ("true", "yes", "on"):
        return True
    if normalized in ("false", "no", "off"):
        return False
    if normalized in ("null", "none", "~"):
        return None
    return value


def _load_simple_yaml(text, path):
    root = {}
    stack = [(-1, root)]

    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = _strip_yaml_comment(raw_line).rstrip()
        if not line.strip():
            continue

        if "\t" in line[:len(line) - len(line.lstrip())]:
            raise RuntimeError(f"{path}:{line_number}: tabs are not supported in YAML indentation")

        indent = len(line) - len(line.lstrip(" "))
        stripped = line.strip()
        if stripped.startswith("- "):
            raise RuntimeError(f"{path}:{line_number}: block lists require PyYAML")
        if ":" not in stripped:
            raise RuntimeError(f"{path}:{line_number}: expected 'key: value'")

        key, value = stripped.split(":", 1)
        key = key.strip()
        if len(key) >= 2 and key[0] == key[-1] and key[0] in ("'", '"'):
            key = key[1:-1]
        if not key:
            raise RuntimeError(f"{path}:{line_number}: empty YAML key")

        while stack and indent <= stack[-1][0]:
            stack.pop()
        if not stack:
            raise RuntimeError(f"{path}:{line_number}: invalid YAML indentation")

        parsed_value = _parse_yaml_scalar(value)
        stack[-1][1][key] = parsed_value
        if isinstance(parsed_value, dict) and not value.strip():
            stack.append((indent, parsed_value))

    return root


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as yaml_file:
        if yaml is not None:
            return yaml.safe_load(yaml_file) or {}
        return _load_simple_yaml(yaml_file.read(), path)



def _launch_file(package_share, *path_parts):
    return os.path.join(package_share, "launch", *path_parts)


def _launch_value(context, name):
    return LaunchConfiguration(name).perform(context)


def _is_auto(value):
    return str(value).strip().lower() in ("", "auto")


def _config_value(context, launch_argument, default):
    value = _launch_value(context, launch_argument)
    if _is_auto(value):
        return default
    return value


def _as_bool(value, default=False):
    if value is None:
        return default
    if isinstance(value, bool):
        return value

    normalized = str(value).strip().lower()
    if normalized in ("true", "1", "yes", "on"):
        return True
    if normalized in ("false", "0", "no", "off"):
        return False
    return default


def _section_enabled(section, default=False):
    if isinstance(section, dict):
        return _as_bool(section.get("enabled"), default)
    return _as_bool(section, default)


def _auto_bool(context, launch_argument, default):
    value = _launch_value(context, launch_argument)
    if _is_auto(value):
        return default
    return _as_bool(value, default)


def _resolve_mode_config(mode_configs, requested_mode):
    mode_name = MODE_ALIASES.get(requested_mode, requested_mode)
    modes = mode_configs.get("modes", {})
    if mode_name not in modes:
        valid_modes = sorted(set(modes.keys()) | set(MODE_ALIASES.keys()))
        raise RuntimeError(
            f"Unknown mode '{requested_mode}'. Add it to sim_modes.yaml or use one of: {valid_modes}"
        )
    return mode_name, modes[mode_name]


def _resolve_map_config(map_configs, map_name):
    maps = map_configs.get("maps", {})
    if map_name not in maps:
        raise RuntimeError(
            f"Unknown map '{map_name}'. Add it to sim_maps.yaml or use one of: {sorted(maps.keys())}"
        )
    return maps[map_name]


def _resolve_robot_config(robot_configs, robot_model, explicit_robot_xacro):
    robots = robot_configs.get("robots", {})
    if robot_model in robots:
        return robots[robot_model]

    if not _is_auto(explicit_robot_xacro):
        return {}

    raise RuntimeError(
        f"Unknown robot_model '{robot_model}'. Add it to robots/config/robots.yaml "
        f"or use one of: {sorted(robots.keys())}"
    )


def _validate_robot_for_mode(robot_model, robot_config, mode_name, mode_config):
    if not robot_config:
        return

    supported_modes = robot_config.get("supported_modes", [])
    if supported_modes and mode_name not in supported_modes:
        raise RuntimeError(
            f"Robot '{robot_model}' does not support mode '{mode_name}'. "
            f"Supported modes: {supported_modes}"
        )

    required_features = mode_config.get("required_features", [])
    robot_features = robot_config.get("features", [])
    missing = [feature for feature in required_features if feature not in robot_features]
    if missing:
        raise RuntimeError(
            f"Robot '{robot_model}' cannot run mode '{mode_name}'. "
            f"Missing required robot features: {missing}"
        )


def _resolve_world_path(gazebo_config):
    world_path = gazebo_config.get("world_path", "")
    if not world_path:
        return ""
    if Path(str(world_path)).is_absolute():
        return str(world_path)
    return _package_file(gazebo_config.get("world_package", "maps"), world_path)


def _resolve_map_yaml(map_name, map_config):
    map_file_config = map_config.get("map", {})
    map_path = map_file_config.get("path", "")
    if not map_path:
        map_path = f"maps/{map_name}/maps/map.yaml"
    if Path(str(map_path)).is_absolute():
        return str(map_path)
    return _package_file(map_file_config.get("package", "maps"), map_path)


def _map_has_2d_map(map_name, map_config, map_yaml):
    map_file_config = map_config.get("map", {})
    configured = map_file_config.get("has_2d_map")
    if configured is not None:
        return _as_bool(configured, False) and os.path.exists(map_yaml)
    return os.path.exists(map_yaml)


def _resolve_rviz_config(mode_config, rviz_override):
    if not _is_auto(rviz_override):
        return rviz_override

    rviz_config = mode_config.get("rviz", {})
    if not _section_enabled(rviz_config, True):
        return ""

    package_name = rviz_config.get("package")
    relative_path = rviz_config.get("path")
    if not package_name or not relative_path:
        return ""
    return _package_file(package_name, relative_path)


def _build_simulation_actions(context):
    description_share = get_package_share_directory("bumperbot_description")
    controller_share = get_package_share_directory("bumperbot_controller")
    localization_share = get_package_share_directory("bumperbot_localization")
    mapping_share = get_package_share_directory("bumperbot_mapping")
    navigation_share = get_package_share_directory("bumperbot_navigation")

    map_configs = _load_yaml(_launch_value(context, "sim_maps_config"))
    mode_configs = _load_yaml(_launch_value(context, "sim_modes_config"))
    robot_configs = _load_yaml(_launch_value(context, "sim_robots_config"))

    requested_mode = _launch_value(context, "mode")
    mode_name, mode_config = _resolve_mode_config(mode_configs, requested_mode)
    map_name = _launch_value(context, "map_name")
    map_config = _resolve_map_config(map_configs, map_name)

    use_sim_time = _launch_value(context, "use_sim_time")
    robot_model = _launch_value(context, "robot_model")
    robot_config = _resolve_robot_config(
        robot_configs,
        robot_model,
        _launch_value(context, "robot_xacro"),
    )
    _validate_robot_for_mode(robot_model, robot_config, mode_name, mode_config)
    robot_package = _config_value(context, "robot_package", robot_config.get("package", "robots"))
    robot_xacro = _config_value(context, "robot_xacro", robot_config.get("xacro", ""))
    robot_name = _config_value(context, "robot_name", robot_config.get("name", robot_model))
    model_path = _package_file(robot_package, robot_xacro)

    gazebo_config = map_config.get("gazebo", {})
    spawn_config = map_config.get("spawn", {})
    initial_pose_config = map_config.get("initial_pose", {})

    world_package = _config_value(context, "world_package", gazebo_config.get("world_package", "maps"))
    world_name = _config_value(context, "world_name", gazebo_config.get("world_name", map_name))
    configured_world_path = _resolve_world_path({**gazebo_config, "world_package": world_package})
    world_path = _config_value(context, "world_path", configured_world_path)
    configured_map_yaml = _resolve_map_yaml(map_name, map_config)
    map_yaml = _config_value(context, "map_yaml", configured_map_yaml)
    if _as_bool(mode_config.get("requires_2d_map"), False) and not _map_has_2d_map(map_name, map_config, map_yaml):
        raise RuntimeError(
            f"Mode '{mode_name}' requires a valid 2D map, but map '{map_name}' does not have one. "
            "Use mode:=slam to create one first, then save it into the maps package and set has_2d_map: true."
        )

    spawn_x = str(_config_value(context, "spawn_x", spawn_config.get("x", "0.0")))
    spawn_y = str(_config_value(context, "spawn_y", spawn_config.get("y", "0.0")))
    spawn_z = str(_config_value(context, "spawn_z", spawn_config.get("z", "0.0")))
    spawn_yaw = str(_config_value(context, "spawn_yaw", spawn_config.get("yaw", "0.0")))

    initial_pose_x = str(_config_value(context, "initial_pose_x", initial_pose_config.get("x", "0.0")))
    initial_pose_y = str(_config_value(context, "initial_pose_y", initial_pose_config.get("y", "0.0")))
    initial_pose_yaw = str(_config_value(context, "initial_pose_yaw", initial_pose_config.get("yaw", "0.0")))

    actions = []

    if mode_name == "display":
        rviz_config = _resolve_rviz_config(mode_config, _launch_value(context, "rviz_config"))
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(description_share, "display.launch.py")),
                launch_arguments={
                    "model": model_path,
                    "rviz_config": rviz_config,
                    "start_rviz": str(_auto_bool(context, "start_rviz", True)),
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        )
        return actions

    gazebo_enabled = _auto_bool(
        context,
        "start_gazebo",
        _section_enabled(mode_config.get("gazebo"), True),
    )
    if gazebo_enabled:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(description_share, "gazebo.launch.py")),
                launch_arguments={
                    "world_name": world_name,
                    "world_package": world_package,
                    "world_path": world_path,
                    "model": model_path,
                    "robot_package": robot_package,
                    "robot_name": robot_name,
                    "spawn_x": spawn_x,
                    "spawn_y": spawn_y,
                    "spawn_z": spawn_z,
                    "spawn_yaw": spawn_yaw,
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        )

    controller_config = mode_config.get("controller", {})
    if _section_enabled(controller_config):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(controller_share, "controller.launch.py")),
                launch_arguments={
                    "use_simple_controller": str(controller_config.get("use_simple_controller", "False")),
                    "use_python": str(controller_config.get("use_python", "False")),
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        )

    if _section_enabled(mode_config.get("joystick")):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(controller_share, "joystick_teleop.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        )

    if _section_enabled(mode_config.get("global_localization")):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(localization_share, "global_localization.launch.py")),
                launch_arguments={
                    "map_name": map_name,
                    "map_yaml": map_yaml,
                    "use_sim_time": use_sim_time,
                    "initial_pose_x": initial_pose_x,
                    "initial_pose_y": initial_pose_y,
                    "initial_pose_yaw": initial_pose_yaw,
                }.items(),
            )
        )

    if _section_enabled(mode_config.get("local_localization")):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(localization_share, "local_localization.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        )

    if _section_enabled(mode_config.get("slam")):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(mapping_share, "slam.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        )

    rtabmap_config = mode_config.get("rtabmap", {})
    if _section_enabled(rtabmap_config):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(mapping_share, "rtabmap.launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "map_name": map_name,
                    "robot_name": robot_name,
                    "rgb_topic": _config_value(context, "rgb_topic", rtabmap_config.get("rgb_topic", "/oakd/rgb/image_raw")),
                    "depth_topic": _config_value(context, "depth_topic", rtabmap_config.get("depth_topic", "/oakd/depth/image_raw")),
                    "camera_info_topic": _config_value(
                        context,
                        "camera_info_topic",
                        rtabmap_config.get("camera_info_topic", "/oakd/rgb/camera_info"),
                    ),
                    "odom_topic": _config_value(context, "odom_topic", rtabmap_config.get("odom_topic", "/odom")),
                    "frame_id": _config_value(context, "rtabmap_frame_id", rtabmap_config.get("frame_id", "base_footprint")),
                    "map_frame_id": _config_value(context, "rtabmap_map_frame_id", rtabmap_config.get("map_frame_id", "map")),
                    "rtabmap_config": _config_value(context, "rtabmap_config", "auto"),
                    "rtabmap_database_path": _config_value(context, "rtabmap_database_path", "auto"),
                    "delete_db_on_start": _config_value(context, "delete_db_on_start", "true"),
                    "start_visual_odometry": _config_value(
                        context,
                        "start_visual_odometry",
                        rtabmap_config.get("start_visual_odometry", "false"),
                    ),
                    "start_rtabmap_viz": _config_value(
                        context,
                        "start_rtabmap_viz",
                        rtabmap_config.get("start_rtabmap_viz", "false"),
                    ),
                }.items(),
            )
        )

    if _section_enabled(mode_config.get("navigation")):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(navigation_share, "navigation.launch.py")),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            )
        )

    rviz_enabled = _auto_bool(
        context,
        "start_rviz",
        _section_enabled(mode_config.get("rviz"), True),
    )
    rviz_config = _resolve_rviz_config(mode_config, _launch_value(context, "rviz_config"))
    if rviz_enabled and rviz_config:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                parameters=[{"use_sim_time": _as_bool(use_sim_time, True)}],
            )
        )

    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory("bumperbot_bringup")

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="nav",
            description="Bringup mode from sim_modes.yaml. 'nave' is accepted as an alias for nav.",
        ),
        DeclareLaunchArgument(
            "map_name",
            default_value="celisca_floor_1",
            description="Map profile name from sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "sim_modes_config",
            default_value=os.path.join(bringup_share, "config", "sim_modes.yaml"),
            description="YAML file defining mode-specific simulator, stack, and RViz behavior.",
        ),
        DeclareLaunchArgument(
            "sim_maps_config",
            default_value=os.path.join(bringup_share, "config", "sim_maps.yaml"),
            description="YAML file defining map-specific Gazebo, spawn, and localization defaults.",
        ),
        DeclareLaunchArgument(
            "sim_robots_config",
            default_value=os.path.join(get_package_share_directory("robots"), "config", "robots.yaml"),
            description="YAML file defining robot model profiles.",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="bumperbot",
            description="Robot profile name from robots/config/robots.yaml.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock.",
        ),
        DeclareLaunchArgument(
            "start_gazebo",
            default_value="auto",
            choices=["auto", "true", "false"],
            description="Override whether Gazebo starts. 'auto' uses sim_modes.yaml.",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="auto",
            choices=["auto", "true", "false"],
            description="Override whether RViz starts. 'auto' uses sim_modes.yaml.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value="auto",
            description="Full RViz config path override. 'auto' uses sim_modes.yaml.",
        ),
        DeclareLaunchArgument(
            "world_package",
            default_value="auto",
            description="Gazebo world package override. 'auto' uses sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="auto",
            description="Gazebo world name override. 'auto' uses sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "world_path",
            default_value="auto",
            description="Full Gazebo world path override. 'auto' uses sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value="auto",
            description="Full Nav2 map YAML path override. 'auto' uses sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "robot_package",
            default_value="auto",
            description="Package containing the robot URDF/xacro file. 'auto' uses robot_model.",
        ),
        DeclareLaunchArgument(
            "robot_xacro",
            default_value="auto",
            description="Robot URDF/xacro path relative to robot_package. 'auto' uses robot_model.",
        ),
        DeclareLaunchArgument(
            "robot_name",
            default_value="auto",
            description="Name for the spawned robot in Gazebo. 'auto' uses robot_model.",
        ),
        DeclareLaunchArgument(
            "spawn_x",
            default_value="auto",
            description="Initial robot spawn x coordinate. 'auto' uses sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "spawn_y",
            default_value="auto",
            description="Initial robot spawn y coordinate. 'auto' uses sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "spawn_z",
            default_value="auto",
            description="Initial robot spawn z coordinate. 'auto' uses sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "spawn_yaw",
            default_value="auto",
            description="Initial robot spawn yaw. 'auto' uses sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "initial_pose_x",
            default_value="auto",
            description="AMCL initial pose x. 'auto' uses sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "initial_pose_y",
            default_value="auto",
            description="AMCL initial pose y. 'auto' uses sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "initial_pose_yaw",
            default_value="auto",
            description="AMCL initial pose yaw. 'auto' uses sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "rgb_topic",
            default_value="auto",
            description="RTAB-Map RGB image topic override for mode:=3d_slam.",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="auto",
            description="RTAB-Map depth image topic override for mode:=3d_slam.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="auto",
            description="RTAB-Map camera info topic override for mode:=3d_slam.",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="auto",
            description="RTAB-Map odometry topic override for mode:=3d_slam.",
        ),
        DeclareLaunchArgument(
            "rtabmap_frame_id",
            default_value="auto",
            description="RTAB-Map robot base frame override for mode:=3d_slam.",
        ),
        DeclareLaunchArgument(
            "rtabmap_map_frame_id",
            default_value="auto",
            description="RTAB-Map map frame override for mode:=3d_slam.",
        ),
        DeclareLaunchArgument(
            "rtabmap_config",
            default_value="auto",
            description="RTAB-Map config YAML override for mode:=3d_slam.",
        ),
        DeclareLaunchArgument(
            "rtabmap_database_path",
            default_value="auto",
            description="RTAB-Map database path override for mode:=3d_slam.",
        ),
        DeclareLaunchArgument(
            "delete_db_on_start",
            default_value="true",
            choices=["true", "false"],
            description="Clear the RTAB-Map database when starting mode:=3d_slam.",
        ),
        DeclareLaunchArgument(
            "start_visual_odometry",
            default_value="auto",
            choices=["auto", "true", "false"],
            description="Use RTAB-Map RGB-D odometry in mode:=3d_slam.",
        ),
        DeclareLaunchArgument(
            "start_rtabmap_viz",
            default_value="auto",
            choices=["auto", "true", "false"],
            description="Start RTAB-Map's native GUI in mode:=3d_slam.",
        ),
        OpaqueFunction(function=_build_simulation_actions),
    ])
