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

# Maps simulator name → (package_share_key, launch_file_relative_path)
_SIMULATOR_DISPATCH = {
    "gazebo": ("robot_lab_description", "gazebo.launch.py"),
    "isaac": ("robot_lab_isaac", "isaac_simulator.launch.py"),
    "pybullet": ("robot_lab_pybullet", "pybullet_simulator.launch.py"),
    "mujoco": ("robot_lab_mujoco", "mujoco_simulator.launch.py"),
}

_VALID_SIMULATORS = frozenset(_SIMULATOR_DISPATCH)

# Canonical algorithm categories (identical to the registry taxonomy and to
# the `algorithm_category` of every selectable step in sim_modes.yaml).  Each
# is a launch argument: `<category>:=<algorithm id>`, or `auto` to take the
# mode's declared default, or `none` to run the mode without that stage.
# Stack defaults, used when neither an explicit *_plugin argument nor a
# planner selection pins the nav2 plugin class.
_DEFAULT_GLOBAL_PLANNER_PLUGIN = "nav2_smac_planner/SmacPlanner2D"
_DEFAULT_LOCAL_PLANNER_PLUGIN = (
    "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
)

ALGORITHM_CATEGORIES = (
    "perception",
    "localization",
    "state_estimation",
    "sensor_fusion",
    "global_planning",
    "local_planning",
    "control",
)

# Sentinel selections for the robot / map slots.  Display mode is allowed to
# run with only a robot (no world) or only a world (no robot), so the GUI can
# visualize either one on its own in any simulator backend.
_NONE_VALUES = ("none", "None", "— None —", "")


def _is_none(value):
    return str(value).strip() in _NONE_VALUES or str(value).strip().lower() == "none"


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
    return _package_file(gazebo_config.get("world_package", "robot_lab_maps"), world_path)


def _resolve_map_yaml(map_name, map_config):
    map_file_config = map_config.get("map", {})
    map_path = map_file_config.get("path", "")
    if not map_path:
        map_path = f"maps/{map_name}/maps/map.yaml"
    if Path(str(map_path)).is_absolute():
        return str(map_path)
    return _package_file(map_file_config.get("package", "robot_lab_maps"), map_path)


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


def _load_algorithm_dispatch(bringup_share):
    """Load config/algorithm_dispatch.yaml (category -> id -> how to apply)."""
    path = os.path.join(bringup_share, "config", "algorithm_dispatch.yaml")
    if not os.path.isfile(path):
        return {}
    return (_load_yaml(path) or {}).get("algorithms", {}) or {}


def _mode_step_defaults(mode_config):
    """category -> default algorithm id declared by the mode's steps."""
    defaults = {}
    for step in mode_config.get("steps") or []:
        if not isinstance(step, dict):
            continue
        category = step.get("algorithm_category")
        default = step.get("default_algorithm")
        if category in ALGORITHM_CATEGORIES and default:
            defaults.setdefault(category, str(default))
    return defaults


def _mode_step_categories(mode_config):
    """Categories the mode actually runs, in declaration order."""
    categories = []
    for step in mode_config.get("steps") or []:
        if not isinstance(step, dict):
            continue
        category = step.get("algorithm_category")
        if category in ALGORITHM_CATEGORIES and category not in categories:
            categories.append(category)
    return categories


def _resolve_algorithm_selection(context, mode_config, dispatch):
    """Resolve every algorithm launch argument into a concrete decision.

    Returns ``(selection, plugins, nodes, notes)``:

    * ``selection`` - category -> algorithm id actually in force ('' when the
      stage is switched off or the mode does not run that category);
    * ``plugins``   - category -> nav2 plugin class for stack plugin switches;
    * ``nodes``     - list of ``(category, algorithm id, node spec)`` to start;
    * ``notes``     - human-readable lines describing where each selection
      takes effect, logged so a launch never silently drops a choice.

    Raises RuntimeError when a selection cannot be honoured, rather than
    starting a simulation that quietly ignores it.
    """
    mode_categories = _mode_step_categories(mode_config)
    defaults = _mode_step_defaults(mode_config)
    selection, plugins, nodes, notes = {}, {}, [], []

    # Backwards compatibility: the deprecated single `algorithm:=` argument
    # fills the mode's first selectable category when that category was left
    # on 'auto'.
    legacy = str(_launch_value(context, "algorithm") or "auto").strip()
    legacy_category = mode_categories[0] if mode_categories else None

    for category in ALGORITHM_CATEGORIES:
        requested = str(_launch_value(context, category) or "auto").strip()
        if _is_auto(requested) and category == legacy_category \
                and not _is_auto(legacy) and not _is_none(legacy):
            requested = legacy
            notes.append("legacy algorithm:=%s applied to %s"
                         % (legacy, category))
        if _is_auto(requested):
            requested = defaults.get(category, "none")
        if _is_none(requested):
            selection[category] = ""
            continue
        if category not in mode_categories:
            raise RuntimeError(
                "Algorithm '%s' was selected for category '%s', but the "
                "current mode does not run that category. Mode categories: %s"
                % (requested, category, mode_categories or "(none)")
            )
        entry = (dispatch.get(category) or {}).get(requested)
        if entry is None:
            known = sorted((dispatch.get(category) or {}).keys())
            raise RuntimeError(
                "Unknown %s algorithm '%s'. Known: %s"
                % (category, requested, known)
            )
        if entry.get("unavailable"):
            raise RuntimeError(
                "%s algorithm '%s' cannot be launched: %s"
                % (category, requested, entry["unavailable"])
            )
        selection[category] = requested
        if entry.get("plugin"):
            plugins[category] = str(entry["plugin"])
            notes.append("%s=%s -> plugin %s"
                         % (category, requested, entry["plugin"]))
        elif entry.get("node"):
            nodes.append((category, requested, entry["node"]))
            notes.append("%s=%s -> node %s/%s"
                         % (category, requested,
                            entry["node"].get("package"),
                            entry["node"].get("executable")))
        elif entry.get("stack"):
            notes.append("%s=%s -> %s stack"
                         % (category, requested, entry["stack"]))
    return selection, plugins, nodes, notes


def _algorithm_nodes(nodes, use_sim_time):
    """Node actions for every selection that runs as its own process."""
    actions = []
    for category, algorithm_id, spec in nodes:
        package = spec.get("package")
        executable = spec.get("executable")
        if not package or not executable:
            continue
        parameters = [{"use_sim_time": _as_bool(use_sim_time, True)}]
        extra = spec.get("parameters")
        if isinstance(extra, dict) and extra:
            parameters.append(dict(extra))
        actions.append(
            Node(
                package=package,
                executable=executable,
                name=spec.get("name") or ("%s_%s" % (category, algorithm_id)),
                output="screen",
                parameters=parameters,
            )
        )
    return actions


def _build_simulation_actions(context):
    description_share = get_package_share_directory("robot_lab_description")
    controller_share = get_package_share_directory("robot_lab_controller")
    localization_share = get_package_share_directory("robot_lab_localization")
    mapping_share = get_package_share_directory("robot_lab_mapping")
    navigation_share = get_package_share_directory("robot_lab_navigation")

    map_configs = _load_yaml(_launch_value(context, "sim_maps_config"))
    mode_configs = _load_yaml(_launch_value(context, "sim_modes_config"))
    robot_configs = _load_yaml(_launch_value(context, "sim_robots_config"))

    bringup_share = get_package_share_directory("robot_lab_bringup")

    requested_mode = _launch_value(context, "mode")
    mode_name, mode_config = _resolve_mode_config(mode_configs, requested_mode)

    # A map-free run is only meaningful in display mode: the robot is shown
    # in an empty world of the selected simulator.  Every other mode needs a
    # world (and loc/nav additionally need its occupancy map).
    map_name = _launch_value(context, "map_name")
    map_free = _is_none(map_name)
    if map_free and mode_name != "display":
        raise RuntimeError(
            "map_name:=none is only supported in mode:=display; mode '%s' "
            "needs a world. Select a map or switch to display mode."
            % mode_name
        )
    map_config = {} if map_free else _resolve_map_config(map_configs, map_name)

    use_sim_time = _launch_value(context, "use_sim_time")

    # A robot-free run shows the world on its own (display mode only), so a
    # map can be inspected in any simulator without spawning a robot.
    robot_model = _launch_value(context, "robot_model")
    robot_free = _is_none(robot_model)
    if robot_free and mode_name != "display":
        raise RuntimeError(
            "robot_model:=none is only supported in mode:=display; mode '%s' "
            "needs a robot. Select a robot or switch to display mode."
            % mode_name
        )
    if robot_free and map_free:
        raise RuntimeError(
            "robot_model:=none together with map_name:=none leaves nothing "
            "to display. Select at least a robot or a map."
        )

    if robot_free:
        robot_config = {}
        robot_package = ""
        robot_xacro = ""
        robot_name = "none"
        model_path = ""
    else:
        robot_config = _resolve_robot_config(
            robot_configs,
            robot_model,
            _launch_value(context, "robot_xacro"),
        )
        _validate_robot_for_mode(robot_model, robot_config, mode_name, mode_config)
        robot_package = _config_value(context, "robot_package", robot_config.get("package", "robot_lab_robots"))
        robot_xacro = _config_value(context, "robot_xacro", robot_config.get("xacro", ""))
        robot_name = _config_value(context, "robot_name", robot_config.get("name", robot_model))
        model_path = _package_file(robot_package, robot_xacro)

    gazebo_config = map_config.get("gazebo", {})
    spawn_config = map_config.get("spawn", {})
    initial_pose_config = map_config.get("initial_pose", {})

    world_package = _config_value(context, "world_package", gazebo_config.get("world_package", "robot_lab_maps"))
    # A map-free display run still needs a ground plane to stand on, so it
    # falls back to the always-present 'empty' world of the maps package.
    default_world_name = "empty" if map_free else gazebo_config.get("world_name", map_name)
    world_name = _config_value(context, "world_name", default_world_name)
    if map_free:
        configured_world_path = _package_file(
            "robot_lab_maps", "maps/empty/worlds/empty.world")
    else:
        configured_world_path = _resolve_world_path(
            {**gazebo_config, "world_package": world_package})
    world_path = _config_value(context, "world_path", configured_world_path)
    configured_map_yaml = "" if map_free else _resolve_map_yaml(map_name, map_config)
    map_yaml = _config_value(context, "map_yaml", configured_map_yaml)
    if not map_free and _as_bool(mode_config.get("requires_2d_map"), False) \
            and not _map_has_2d_map(map_name, map_config, map_yaml):
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
        # Display mode visualizes what was selected in the chosen simulator's
        # own viewer, with RViz alongside it.  All three combinations are
        # supported in every backend:
        #   robot + map  - robot spawned in the world
        #   robot only   - robot in the simulator's empty world (map_name:=none)
        #   map only     - the world on its own (robot_model:=none)
        # No physics stack, controllers or localization run here.
        simulator = _launch_value(context, "simulator")
        if simulator not in _VALID_SIMULATORS:
            raise RuntimeError(
                f"Unknown simulator '{simulator}'. "
                f"Choose from: {sorted(_VALID_SIMULATORS)}"
            )
        gui_value = _launch_value(context, "gui")
        if _is_auto(gui_value):
            gui_value = "true" if os.environ.get("DISPLAY") else "false"
        rviz_config = _resolve_rviz_config(mode_config, _launch_value(context, "rviz_config"))
        start_rviz = _auto_bool(context, "start_rviz", True)

        if simulator == "gazebo":
            # Gazebo display: robot_state_publisher + RViz via display.launch.py,
            # plus gz-sim itself when a world was selected so the map is visible.
            if not robot_free:
                actions.append(
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            _launch_file(description_share, "display.launch.py")),
                        launch_arguments={
                            "model": model_path,
                            "rviz_config": rviz_config,
                            "start_rviz": str(start_rviz),
                            "use_sim_time": use_sim_time,
                        }.items(),
                    )
                )
            elif start_rviz and rviz_config:
                actions.append(
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        arguments=["-d", rviz_config],
                        output="screen",
                        parameters=[{"use_sim_time": _as_bool(use_sim_time, True)}],
                    )
                )
            if not map_free or robot_free:
                gazebo_args = {
                    "world_name": world_name,
                    "world_package": world_package,
                    "world_path": world_path,
                    "use_sim_time": use_sim_time,
                    "gui": gui_value,
                }
                if not robot_free:
                    gazebo_args.update({
                        "model": model_path,
                        "robot_package": robot_package,
                        "robot_xacro": robot_xacro,
                        "robot_name": robot_name,
                        "spawn_x": spawn_x,
                        "spawn_y": spawn_y,
                        "spawn_z": spawn_z,
                        "spawn_yaw": spawn_yaw,
                    })
                else:
                    gazebo_args["spawn_robot"] = "false"
                actions.append(
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            _launch_file(description_share, "gazebo.launch.py")),
                        launch_arguments=gazebo_args.items(),
                    )
                )
            return actions

        # PyBullet / MuJoCo / Isaac: their own viewer renders both the world
        # and the robot, so the simulator launch is included directly.
        sim_pkg, sim_launch = _SIMULATOR_DISPATCH[simulator]
        sim_share = get_package_share_directory(sim_pkg)
        display_args = {
            "world_name": world_name,
            "world_package": world_package,
            "world_path": world_path,
            "model": model_path,
            "robot_package": robot_package,
            "robot_xacro": robot_xacro,
            "robot_name": robot_name,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
            "use_sim_time": use_sim_time,
            "gui": gui_value,
        }
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(sim_share, sim_launch)),
                launch_arguments=display_args.items(),
            )
        )
        if start_rviz and rviz_config and not robot_free:
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

    # Resolve every algorithm selection BEFORE anything is started, so an
    # unrunnable choice fails loudly here instead of producing a simulation
    # that silently ignores it.
    dispatch = _load_algorithm_dispatch(bringup_share)
    algorithm_selection, algorithm_plugins, algorithm_nodes, algorithm_notes = \
        _resolve_algorithm_selection(context, mode_config, dispatch)
    for note in algorithm_notes:
        print("[robot_lab] algorithm %s" % note)

    gazebo_enabled = _auto_bool(
        context,
        "start_gazebo",
        _section_enabled(mode_config.get("gazebo"), True),
    )
    if gazebo_enabled:
        simulator = _launch_value(context, "simulator")
        if simulator not in _VALID_SIMULATORS:
            raise RuntimeError(
                f"Unknown simulator '{simulator}'. "
                f"Choose from: {sorted(_VALID_SIMULATORS)}"
            )

        # Determine the mode-config allowed simulators (if listed).
        allowed = mode_config.get("simulators")
        if allowed and simulator not in allowed:
            raise RuntimeError(
                f"Simulator '{simulator}' is not supported in mode '{mode_name}'. "
                f"Allowed: {allowed}"
            )

        sim_pkg, sim_launch = _SIMULATOR_DISPATCH[simulator]
        sim_share = get_package_share_directory(sim_pkg)

        # Forward the gui argument: auto→true if DISPLAY set, else false
        gui_value = _launch_value(context, "gui")
        if gui_value == "auto":
            gui_value = "true" if os.environ.get("DISPLAY") else "false"

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(sim_share, sim_launch)),
                launch_arguments={
                    "world_name": world_name,
                    "world_package": world_package,
                    "world_path": world_path,
                    "model": model_path,
                    "robot_package": robot_package,
                    "robot_xacro": robot_xacro,
                    "robot_name": robot_name,
                    "spawn_x": spawn_x,
                    "spawn_y": spawn_y,
                    "spawn_z": spawn_z,
                    "spawn_yaw": spawn_yaw,
                    "use_sim_time": use_sim_time,
                    "gui": gui_value,
                }.items(),
            )
        )

    controller_config = mode_config.get("controller", {})
    # ros2_control (controller_manager + robot_lab_controller + joint_state_broadcaster)
    # is only provided by the Gazebo backend (gz_ros2_control).  Alternate
    # backends (Isaac/PyBullet/MuJoCo) are self-contained: their spawners
    # subscribe /cmd_vel and publish /odom + /joint_states + /clock directly,
    # so launching a controller layer would hang forever waiting for
    # /controller_manager.  In display mode the Gazebo controller layer is
    # likewise skipped — there is nothing to drive yet.
    _sim_for_controller = _launch_value(context, "simulator")
    if _section_enabled(controller_config) and _sim_for_controller == "gazebo":
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
                    "robot_model": robot_model,
                    "initial_pose_x": initial_pose_x,
                    "initial_pose_y": initial_pose_y,
                    "initial_pose_yaw": initial_pose_yaw,
                }.items(),
            )
        )

    # EKF odom0: Gazebo uses the SimpleController wheel estimate; self-contained
    # spawners (PyBullet/MuJoCo/Isaac) publish perfect odometry on /odom/ground_truth.
    _sim = _launch_value(context, "simulator")
    _odom0_topic = "/robot_lab_controller/odom" if _sim == "gazebo" else "/odom/ground_truth"

    if _section_enabled(mode_config.get("local_localization")):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(localization_share, "local_localization.launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "robot_model": robot_model,
                    "odom0": _odom0_topic,
                }.items(),
            )
        )

    if _section_enabled(mode_config.get("slam")):
        slam_args = {
            "use_sim_time": use_sim_time,
            "robot_model": robot_model,
        }
        slam_backend = algorithm_selection.get("localization", "")
        if slam_backend:
            slam_args["slam_backend"] = slam_backend
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(mapping_share, "slam.launch.py")),
                launch_arguments=slam_args.items(),
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
        # The resolved planner selections decide the active nav2 plugins.
        # An explicit *_plugin argument still wins, so a caller can pin a
        # plugin class the registry does not model yet.
        global_plugin = _launch_value(context, "global_planner_plugin")
        local_plugin = _launch_value(context, "local_planner_plugin")
        if _is_auto(global_plugin):
            global_plugin = algorithm_plugins.get(
                "global_planning", _DEFAULT_GLOBAL_PLANNER_PLUGIN)
        if _is_auto(local_plugin):
            local_plugin = algorithm_plugins.get(
                "local_planning", _DEFAULT_LOCAL_PLANNER_PLUGIN)
        nav_args = {
            "use_sim_time": use_sim_time,
            "robot_model": robot_model,
            "global_planner_plugin": global_plugin,
            "local_planner_plugin": local_plugin,
        }
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(navigation_share, "navigation.launch.py")),
                launch_arguments=nav_args.items(),
            )
        )

    # Selections that run as their own process (planners, estimators,
    # perception pipelines and controllers that are not stack plugins).
    actions.extend(_algorithm_nodes(algorithm_nodes, use_sim_time))

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
    bringup_share = get_package_share_directory("robot_lab_bringup")

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="nav",
            description="Bringup mode from sim_modes.yaml. 'nave' is accepted as an alias for nav.",
        ),
        DeclareLaunchArgument(
            "global_planner_plugin",
            default_value="auto",
            description="Global planner plugin forwarded to the navigation "
                        "stack. 'auto' uses the global_planning selection.",
        ),
        DeclareLaunchArgument(
            "local_planner_plugin",
            default_value="auto",
            description="Local planner plugin forwarded to the navigation "
                        "stack. 'auto' uses the local_planning selection.",
        ),
        DeclareLaunchArgument(
            "map_name",
            default_value="celisca_floor_1",
            description="Map profile name from sim_maps.yaml, or 'none' to "
                        "display a robot without a world (display mode only).",
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
            default_value=os.path.join(get_package_share_directory("robot_lab_robots"), "config", "robots.yaml"),
            description="YAML file defining robot model profiles.",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="bumperbot",
            description="Robot profile name from robots/config/robots.yaml, "
                        "or 'none' to display a world without a robot "
                        "(display mode only).",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock.",
        ),
        DeclareLaunchArgument(
            "simulator",
            default_value="gazebo",
            choices=["gazebo", "isaac", "pybullet", "mujoco"],
            description="Which physics simulator backend to use.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="auto",
            choices=["auto", "true", "false"],
            description="Simulator GUI mode. 'auto' enables GUI if DISPLAY is set, 'true' forces GUI, 'false' forces headless.",
        ),
        DeclareLaunchArgument(
            "algorithm",
            default_value="auto",
            description="Deprecated single-algorithm selector. Prefer the "
                        "per-category arguments below; when set it fills the "
                        "first selectable category of the mode.",
        ),
    ] + [
        DeclareLaunchArgument(
            category,
            default_value="auto",
            description="Algorithm for the '%s' category. 'auto' uses the "
                        "mode's default_algorithm from sim_modes.yaml, "
                        "'none' switches the stage off." % category,
        )
        for category in ALGORITHM_CATEGORIES
    ] + [
        DeclareLaunchArgument(
            "start_gazebo",
            default_value="auto",
            choices=["auto", "true", "false"],
            description="Override whether the physics simulator starts. 'auto' uses sim_modes.yaml.",
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
