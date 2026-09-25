import json
import os
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
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

# How long the localization/mapping/navigation stack waits for the simulator
# before starting anyway (the wait is a gate, not a hard requirement).
SIMULATOR_READY_TIMEOUT = 180

# Maps simulator name → (package_share_key, launch_file_relative_path)
_SIMULATOR_DISPATCH = {
    "gazebo": ("robot_lab_description", "gazebo.launch.py"),
    "isaac": ("robot_lab_isaac", "isaac_simulator.launch.py"),
    "pybullet": ("robot_lab_pybullet", "pybullet_simulator.launch.py"),
    "mujoco": ("robot_lab_mujoco", "mujoco_simulator.launch.py"),
}

_VALID_SIMULATORS = frozenset(_SIMULATOR_DISPATCH)

# Drive keys the simulator launch files declare as arguments of their own.
_DIFF_DRIVE_KEYS = ("left_wheel_joint", "right_wheel_joint",
                    "wheel_radius", "wheel_separation")

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



_URDF_TEXT_CACHE = {}


def _xacro_urdf(model_path):
    """Expanded URDF text for *model_path* ('' when xacro fails).

    Cached: the same description is inspected for ros2_control, spawn
    clearance and the RViz fixed frame during one launch.
    """
    import subprocess
    if not model_path:
        return ""
    if model_path not in _URDF_TEXT_CACHE:
        try:
            _URDF_TEXT_CACHE[model_path] = subprocess.run(
                ["xacro", model_path], capture_output=True, text=True,
                check=True, timeout=60).stdout
        except Exception:
            _URDF_TEXT_CACHE[model_path] = ""
    return _URDF_TEXT_CACHE[model_path]


def _description_has_ros2_control(model_path):
    """Whether the robot description (after xacro) has a <ros2_control> block."""
    return "<ros2_control" in _xacro_urdf(model_path)


def _rviz_fixed_frame(model_path):
    """RViz fixed frame for a robot description ('' when unknown).

    The shipped RViz configs use ``base_footprint``, the root link of the
    wheeled robots.  A description rooted at ``base``/``trunk`` (Unitree, the
    Berkeley Humanoid Lite) has no such frame: TF never contains it, so RViz
    reports "Fixed Frame [base_footprint] does not exist" and draws neither
    the model nor its joint states.  Pass the description's own root link to
    RViz in that case.
    """
    urdf = _xacro_urdf(model_path)
    if not urdf:
        return ""
    try:
        root = ET.fromstring(urdf)
    except ET.ParseError:
        return ""
    links = [link.get("name") for link in root.findall("link") if link.get("name")]
    if "base_footprint" in links:
        return "base_footprint"
    children = set()
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None and child.get("link"):
            children.add(child.get("link"))
    roots = [name for name in links if name not in children]
    return roots[0] if roots else ""


def _clear_spawn_height(model_path, spawn_z):
    """Spawn height keeping the robot's rest-pose collision geometry above z=0."""
    from robot_lab_utils.urdf_extent import (
        clear_spawn_z, lowest_collision_z, package_resolver)
    try:
        urdf = _xacro_urdf(model_path)
        if not urdf:
            return spawn_z
        lowest = lowest_collision_z(urdf, package_resolver(
            get_package_share_directory, os.path.dirname(model_path)))
    except Exception as exc:
        print("[robot_lab] spawn clearance not computed (%s)" % exc)
        return spawn_z
    return clear_spawn_z(spawn_z, lowest)


def _urdf_root_link(model_path):
    """Root link of the robot description, or '' when it cannot be read."""
    import subprocess
    from robot_lab_utils.sim_frames import urdf_link_frames
    try:
        urdf = subprocess.run(["xacro", model_path], capture_output=True,
                              text=True, check=True, timeout=60).stdout
        return urdf_link_frames(urdf)[1] or ""
    except Exception:
        return ""


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


def _as_float(value, default=0.0):
    """Coerce a launch substitution/string value to float.

    Node parameters arrive as strings; declaring them as DOUBLE/DOUBLE_ARRAY
    would reject a plain string, so numeric recovery parameters are converted
    here instead of relying on ``declare_parameter`` type inference.
    """
    if value is None:
        return default
    try:
        return float(str(value).strip())
    except (TypeError, ValueError):
        return default


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


def _validate_robot_for_mode(robot_model, robot_config, mode_name, mode_config,
                             simulator=None):
    if not robot_config:
        return

    supported_modes = robot_config.get("supported_modes", [])
    if supported_modes and mode_name not in supported_modes:
        raise RuntimeError(
            f"Robot '{robot_model}' does not support mode '{mode_name}'. "
            f"Supported modes: {supported_modes}"
        )

    from robot_lab_utils.mode_capability import describe_missing, missing_features
    missing = missing_features(robot_config, mode_name, simulator, mode_config)
    if missing:
        raise RuntimeError(
            f"Robot '{robot_model}' cannot run mode '{mode_name}' in "
            f"{simulator or 'this simulator'}: {describe_missing(missing)}"
        )


def _resolve_asset_override(value, package):
    """Accept absolute/local files and the resolver's package-relative paths."""
    if not value or Path(str(value)).is_absolute():
        return value
    if Path(str(value)).is_file():
        return str(Path(str(value)).resolve())
    return _package_file(package, value)


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


def _resolve_algorithm_selection(context, mode_config, dispatch, robot_config=None):
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
    # A robot can replace a mode default it cannot use (a car's local
    # planner must not turn it on the spot): robots.yaml default_algorithms.
    for category, algorithm in ((robot_config or {}).get("default_algorithms") or {}).items():
        if category in defaults:
            defaults[category] = str(algorithm)
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
        unsuitable = set(entry.get("not_for_features") or []) \
            & set((robot_config or {}).get("features") or [])
        if unsuitable:
            raise RuntimeError(
                "%s algorithm '%s' cannot drive this robot (%s): %s"
                % (category, requested, ", ".join(sorted(unsuitable)),
                   entry.get("not_for_reason", "incompatible motion model")))
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
        _validate_robot_for_mode(robot_model, robot_config, mode_name, mode_config,
                                 _launch_value(context, "simulator"))
        robot_package = _config_value(context, "robot_package", robot_config.get("package", "robot_lab_robots"))
        robot_xacro = _config_value(context, "robot_xacro", robot_config.get("xacro", ""))
        robot_name = _config_value(context, "robot_name", robot_config.get("name", robot_model))
        model_path = _package_file(robot_package, robot_xacro)

    # The simulator bridges take the differential-drive keys as arguments
    # and the whole drive block (a car's steering too) as JSON; see
    # robot_lab_utils.drive_kinematics.
    drive_config = robot_config.get("drive", {}) or {}
    drive_args = {key: str(value) for key, value in drive_config.items()
                  if key in _DIFF_DRIVE_KEYS}
    drive_args["drive_config"] = json.dumps(drive_config, sort_keys=True)
    drive_type = str(drive_config.get("type", "diff"))
    # Localization runs in the robot's own root frame (base_footprint for
    # the wheeled bases, 'base' or 'pelvis' for legged and humanoid ones).
    base_frame = "" if robot_free or mode_name == "display" else _urdf_root_link(model_path)

    gazebo_config = map_config.get("gazebo", {})
    # Robot-level spawn overrides win over map defaults. Needed for robots
    # whose base frame does not sit at sole level (e.g. the R5.3 BHL biped,
    # whose foot soles are +0.076 m above the base origin at the URDF rest
    # pose): spawning it at the map default drops the model onto the ground
    # and the slam topples it before the balance loop can react.
    spawn_config = dict(map_config.get("spawn", {}))
    spawn_config.update(robot_config.get("spawn", {}))

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
    world_path = _resolve_asset_override(
        _config_value(context, "world_path", configured_world_path), world_package)
    configured_map_yaml = "" if map_free else _resolve_map_yaml(map_name, map_config)
    map_yaml = _resolve_asset_override(
        _config_value(context, "map_yaml", configured_map_yaml),
        map_config.get("map", {}).get("package", "robot_lab_maps"))
    if not map_free and _as_bool(mode_config.get("requires_2d_map"), False) \
            and not _map_has_2d_map(map_name, map_config, map_yaml):
        raise RuntimeError(
            f"Mode '{mode_name}' requires a valid 2D map, but map '{map_name}' does not have one. "
            "Use mode:=slam to create one first, then save it into the maps package and set has_2d_map: true."
        )

    spawn_x = str(float(_config_value(context, "spawn_x", spawn_config.get("x", "0.0"))))
    spawn_y = str(float(_config_value(context, "spawn_y", spawn_config.get("y", "0.0"))))
    spawn_z = str(float(_config_value(context, "spawn_z", spawn_config.get("z", "0.0"))))
    spawn_yaw = str(float(_config_value(context, "spawn_yaw", spawn_config.get("yaw", "0.0"))))

    if _launch_value(context, "simulator") == "gazebo" and not robot_free:
        # The other backends lift a robot whose legs start inside the floor
        # from their own collision bounds; Gazebo spawns blind, so the root
        # height is raised here from the description's collision geometry.
        cleared = _clear_spawn_height(model_path, float(spawn_z))
        if cleared > float(spawn_z) + 1e-4:
            print("[robot_lab] spawn z %.3f -> %.3f m: keeps %s's lowest "
                  "collision point above the floor" % (float(spawn_z), cleared, robot_model))
            spawn_z = str(cleared)

    initial_pose_x = str(float(_config_value(context, "initial_pose_x", spawn_x)))
    initial_pose_y = str(float(_config_value(context, "initial_pose_y", spawn_y)))
    initial_pose_yaw = str(float(_config_value(context, "initial_pose_yaw", spawn_yaw)))

    actions = []
    # Every process this launch starts carries a run id, and a detached
    # watcher stops whatever still carries it once the launch has exited
    # (see robot_lab_utils.partition_reaper): Gazebo servers, state
    # publishers or Nav2 left behind by a killed launch otherwise keep
    # publishing into the next run's ROS graph.
    run_id = "robot_lab_run_" + uuid.uuid4().hex
    actions.append(SetEnvironmentVariable("ROBOT_LAB_RUN_ID", run_id))
    import subprocess
    import sys
    subprocess.Popen(
        [sys.executable, "-m", "robot_lab_utils.partition_reaper",
         str(os.getpid()), "ROBOT_LAB_RUN_ID=" + run_id],
        start_new_session=True, stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if _launch_value(context, "simulator") == "gazebo":
        # GUI/server/bridges must share a transport partition, but must not
        # discover a previous launch's world. Respect explicitly supplied IDs.
        partition = (context.environment.get("IGN_PARTITION") or
                     context.environment.get("GZ_PARTITION") or
                     "robot_lab_" + uuid.uuid4().hex)
        actions.extend([SetEnvironmentVariable("IGN_PARTITION", partition),
                        SetEnvironmentVariable("GZ_PARTITION", partition)])

    # Older Cyclone builds exhaust their automatic participant range in a
    # full Nav2/SLAM graph. Preserve any operator-supplied discovery config.
    if not context.environment.get("CYCLONEDDS_URI"):
        actions.append(SetEnvironmentVariable(
            "CYCLONEDDS_URI", '<CycloneDDS><Domain><Discovery>'
            '<ParticipantIndex>auto</ParticipantIndex>'
            '<MaxAutoParticipantIndex>99</MaxAutoParticipantIndex>'
            '</Discovery></Domain></CycloneDDS>'))

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
        # RViz needs a fixed frame that exists in TF.  The configs ship with
        # 'base_footprint' (the wheeled robots' root link); a description
        # rooted at 'base'/'trunk' has no such frame, and RViz then reports
        # "Fixed Frame [base_footprint] does not exist" and draws nothing —
        # however fresh the simulator's joint states are.
        rviz_arguments = ["-d", rviz_config]
        rviz_frame = "" if robot_free else _rviz_fixed_frame(model_path)
        if rviz_frame:
            rviz_arguments += ["-f", rviz_frame]

        if simulator == "gazebo":
            # Gazebo runs for every display selection, like the other
            # backends: a robot without a map is spawned into the empty
            # world (it used to get only the RViz joint-slider preview, so
            # Gazebo never opened).  Joint states come from the simulation.
            if start_rviz and rviz_config and not robot_free:
                actions.append(
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        arguments=rviz_arguments,
                        output="screen",
                        parameters=[{"use_sim_time": _as_bool(use_sim_time, True)}],
                    )
                )
            gazebo_args = {
                "world_name": world_name,
                "world_package": world_package,
                "world_path": world_path,
                "use_sim_time": use_sim_time,
                "gui": gui_value,
            }
            if not robot_free:
                hold = _launch_value(context, "display_hold").strip().lower()
                gazebo_args.update({
                    # Joint states for robots without ros2_control, and
                    # the same display hold as the other backends
                    # ('auto' is harmless for ros2_control robots, whose
                    # controllers own their joints).
                    "display_plugins": "true",
                    "display_hold": "false" if hold == "false" else "true",
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
            # Robots that declare their own ros2_control stack (e.g. the
            # R5.3 Berkeley Humanoid Lite sim profile, whose controllers
            # are parameterized by the gz_ros2_control plugin inside the
            # description) get their controllers spawned here, on top of
            # the simulator bringup. The shared controller layer is
            # display-skipped, so without this the bringup reaches
            # controller_manager but never activates a controller.
            display_controllers = robot_config.get("controllers", [])
            # Decided by the description itself: a profile can list the
            # ros2_control feature for a model that carries no
            # <ros2_control> block, and a spawner for it only waits and fails.
            if not display_controllers and not robot_free \
                    and _description_has_ros2_control(model_path):
                display_controllers = ["joint_state_broadcaster"]
            for _robot_controller in display_controllers:
                actions.append(
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            _robot_controller,
                            "--controller-manager-timeout", "60",
                        ],
                        output="screen",
                    )
                )
            return actions

        # PyBullet / MuJoCo / Isaac: their own viewer renders both the world
        # and the robot, so the simulator launch is included directly.
        # Display is passive visualization: joints hold their spawn pose so
        # legged/humanoid robots do not collapse or jump under gravity while
        # RViz reflects the simulator's joint_states (the pre-regression
        # behaviour).  MuJoCo/Isaac honour the flag; PyBullet already has it.
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
        display_args.update(drive_args)
        if simulator in ("mujoco", "isaac"):
            display_args["hold_position"] = _launch_value(context, "display_hold")
        elif simulator == "pybullet":
            display_args["hold_position"] = _launch_value(context, "display_hold")
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
                    arguments=rviz_arguments,
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
        _resolve_algorithm_selection(context, mode_config, dispatch, robot_config)
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

        # The simulated BHL is an effort-driven biped. In localization mode
        # the GUI Drive pad publishes /key_vel, which twist_mux forwards to
        # the policy; the simulator's passive joint hold must be off so the
        # policy can move the legs. Start the controller before the backend
        # so it is ready for the first joint/IMU measurements.
        bhl_policy_active = (
            robot_model == "berkeley_humanoid_lite_sim"
            and simulator == "mujoco" and mode_name == "loc"
        )
        go2_controller_active = (
            robot_model == "unitree_go2"
            and simulator == "mujoco" and mode_name == "loc"
        )
        go2_mujoco = robot_model == "unitree_go2" and simulator == "mujoco"
        go2_policy_path = _launch_value(context, "go2_policy_path") if go2_controller_active else ""
        if go2_policy_path == "auto":
            go2_policy_path = os.path.join(
                get_package_share_directory("robot_lab_adapter"), "policies",
                "go2_velocity_flat", "policy.onnx")
        go2_recovery_path = (_launch_value(context, "go2_recovery_policy_path")
                             if go2_controller_active else "")
        if go2_recovery_path == "auto":
            go2_recovery_path = os.path.join(
                get_package_share_directory("robot_lab_adapter"), "policies",
                "go2_recovery_nju", "policy.onnx")
        go2_initial_positions = json.dumps({
            f"{leg}_{kind}_joint": value
            for leg in ("FL", "FR", "RL", "RR")
            for kind, value in (("hip", (-0.1 if leg.endswith("L") else 0.1)
                                  if go2_policy_path else 0.0),
                                ("thigh", 0.9 if go2_policy_path else 0.72),
                                ("calf", -1.8 if go2_policy_path else -1.45))
        }) if go2_mujoco else ""
        if bhl_policy_active:
            actions.append(
                Node(
                    package="robot_lab_adapter",
                    executable="humanoid-policy-controller",
                    output="screen",
                    parameters=[{
                        "use_sim_time": _as_bool(use_sim_time, True),
                        "cmd_vel_topic": "/robot_lab_controller/cmd_vel_unstamped",
                        "yaw_servo_gain": (
                            4.0 if _as_bool(
                                _launch_value(context, "bhl_enable_yaw_servo"))
                            else 0.0),
                        "torque_filter_enabled": _as_bool(
                            _launch_value(context, "bhl_enable_torque_filter")),
                    }],
                )
            )
        if go2_controller_active:
            actions.append(Node(
                package="robot_lab_adapter",
                executable="go2-stance-gait-controller",
                output="screen",
                parameters=[{
                    "use_sim_time": _as_bool(use_sim_time, True),
                    "cmd_vel_topic": "/robot_lab_controller/cmd_vel_unstamped",
                    "enable_experimental_gait": _as_bool(
                        _launch_value(context, "go2_enable_experimental_gait")),
                    "policy_path": go2_policy_path,
                    "recovery_policy_path": go2_recovery_path,
                    "reverse_command_map": _launch_value(
                        context, "go2_reverse_command_map"),
                    "enable_fall_recovery": _as_bool(
                        _launch_value(context, "enable_fall_recovery")),
                    "fall_recovery_timeout_s": _as_float(
                        _launch_value(context, "fall_recovery_timeout_s"), 4.0),
                    "fall_recovery_start_delay_s": _as_float(
                        _launch_value(context, "fall_recovery_start_delay_s"), 0.0),
                    "fall_recovery_gain_scale": _as_float(
                        _launch_value(context, "fall_recovery_gain_scale"), 0.5),
                    "fall_recovery_damping_scale": _as_float(
                        _launch_value(context, "fall_recovery_damping_scale"), 0.5),
                }],
            ))

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
                    # A robot without drive wheels (a humanoid or dog in
                    # loc) stands on the same joint hold as in display mode;
                    # otherwise it collapses and scans the floor.  Wheeled
                    # robots are never held ('auto' in the spawners).
                    "hold_position": ("false" if bhl_policy_active or go2_controller_active else
                                      _launch_value(context, "display_hold")),
                    **({"effort_controller_config": os.path.join(
                        get_package_share_directory("robot_lab_robots"),
                        "unitree", "go2_description", "config", "go2_controllers.yaml")}
                       if go2_controller_active else {}),
                    **({"initial_joint_positions": go2_initial_positions}
                       if go2_mujoco else {}),
                    **({"physics_timestep": "0.001"}
                       if go2_controller_active else {}),
                    **({"effort_joint_armature": "0.01"}
                       if go2_controller_active else {}),
                    **({"perturbation_force_n": _launch_value(
                         context, "go2_perturbation_force_n"),
                         "perturbation_start_s": _launch_value(
                             context, "go2_perturbation_start_s"),
                         "perturbation_duration_s": _launch_value(
                             context, "go2_perturbation_duration_s"),
                         "perturbation_axis": _launch_value(
                             context, "go2_perturbation_axis")}
                       if go2_controller_active else {}),
                    **({"physics_timestep": _launch_value(
                        context, "bhl_physics_timestep")}
                       if bhl_policy_active else {}),
                    **drive_args,
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
                    **{key: value for key, value in drive_args.items()
                       if key in ("wheel_radius", "wheel_separation")},
                    "drive_type": drive_type,
                    "drive_config": drive_args["drive_config"],
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

    # Localization, mapping and navigation all need the simulator running:
    # PyBullet, MuJoCo and Isaac build their model asynchronously (seconds to
    # tens of seconds) and only start publishing /clock, TF and joint states
    # once the robot exists.  A nav2 stack started first never recovers —
    # its costmaps time out on map->base_footprint forever and every goal is
    # rejected — so these sections are gated on the simulator's first
    # /joint_states instead of racing its load.  Gazebo loads in seconds and
    # starts everything at once, as before.
    gated_actions = []

    if _section_enabled(mode_config.get("global_localization")):
        gated_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(localization_share, "global_localization.launch.py")),
                launch_arguments={
                    "map_name": map_name,
                    "map_yaml": map_yaml,
                    "use_sim_time": use_sim_time,
                    "robot_model": robot_model,
                    "base_frame": base_frame,
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
        gated_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(localization_share, "local_localization.launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "robot_model": robot_model,
                    "base_frame": base_frame,
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
        gated_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(mapping_share, "slam.launch.py")),
                launch_arguments=slam_args.items(),
            )
        )

    rtabmap_config = mode_config.get("rtabmap", {})
    if _section_enabled(rtabmap_config):
        gated_actions.append(
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
            "motion_model": "ackermann" if drive_type == "ackermann" else "diff",
        }
        gated_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_launch_file(navigation_share, "navigation.launch.py")),
                launch_arguments=nav_args.items(),
            )
        )

    # Selections that run as their own process (planners, estimators,
    # perception pipelines and controllers that are not stack plugins).
    gated_actions.extend(_algorithm_nodes(algorithm_nodes, use_sim_time))

    if gated_actions and _launch_value(context, "simulator") != "gazebo":
        # Gate: the sections above start once the simulator has published its
        # first /joint_states (see the note where gated_actions is declared).
        # The timeout keeps a broken backend from hiding the whole stack
        # forever; a late start still runs, just without the race.
        ready_gate = ExecuteProcess(
            cmd=["bash", "-c",
                 "timeout %d ros2 topic echo --once /joint_states "
                 ">/dev/null 2>&1 || true" % SIMULATOR_READY_TIMEOUT],
            name="simulator_ready_gate",
            output="log",
        )
        actions.append(ready_gate)
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=ready_gate,
                          on_exit=list(gated_actions))))
    else:
        actions.extend(gated_actions)

    rviz_enabled = _auto_bool(
        context,
        "start_rviz",
        _section_enabled(mode_config.get("rviz"), True),
    )
    rviz_config = _resolve_rviz_config(mode_config, _launch_value(context, "rviz_config"))
    if rviz_enabled and rviz_config:
        # Same fixed-frame rule as display mode: a robot whose description has
        # no 'base_footprint' link must be shown on its own root link, or RViz
        # cannot resolve the frame and draws neither model nor joint states.
        nav_rviz_arguments = ["-d", rviz_config]
        nav_rviz_frame = "" if robot_free else _rviz_fixed_frame(model_path)
        if nav_rviz_frame:
            nav_rviz_arguments += ["-f", nav_rviz_frame]
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=nav_rviz_arguments,
                output="screen",
                parameters=[{"use_sim_time": _as_bool(use_sim_time, True)}],
            )
        )

    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_lab_bringup")

    return LaunchDescription([
        DeclareLaunchArgument(
            "bhl_enable_yaw_servo", default_value="false",
            description="Opt-in boost-only closed-loop yaw servo for the BHL "
                        "policy controller: raises the commanded yaw rate "
                        "toward the training limit while the measured body "
                        "yaw rate falls short. Unqualified experiment."),
        DeclareLaunchArgument(
            "bhl_enable_torque_filter", default_value="false",
            description="Opt-in BHL Recoil torque EMA. Uses the pinned motor "
                        "configuration alpha and preserves its 2 kHz time "
                        "constant at the 250 Hz effort-command rate. "
                        "Experimental; matched live A/B did not revive the "
                        "sustained turn."),
        DeclareLaunchArgument(
            "bhl_physics_timestep", default_value="0.0",
            description="Optional BHL MuJoCo physics timestep in seconds; "
                        "0.0 preserves the current merged-model default. "
                        "Use 0.0005 for the upstream 2 kHz qualification rate "
                        "as an opt-in experiment."),
        DeclareLaunchArgument(
            "go2_enable_experimental_gait", default_value="false",
            description="Allow unqualified Go2 stepping experiments; the "
                        "measured gait does not yet track forward/reverse commands."),
        DeclareLaunchArgument(
            "go2_policy_path", default_value="",
            description="Optional ONNX Go2 velocity policy ('auto' uses the "
                        "bundled flat-ground model); experimental until "
                        "the model and stop/turn/terrain behavior are qualified."),
        DeclareLaunchArgument(
            "go2_recovery_policy_path", default_value="",
            description="Optional NJU-RLC Go2 get-up ONNX actor ('auto' uses the "
                        "bundled model); requires enable_fall_recovery:=true and "
                        "remains experimental until a live get-up trial passes."),
        DeclareLaunchArgument(
            "go2_reverse_command_map", default_value="feedforward",
            description="Opt-in Go2 reverse observation map: 'feedforward' preserves "
                        "the measured dead-zone compensation; 'inverse' uses the "
                        "R5.2 sweep-fitted candidate and remains experimental."),
        DeclareLaunchArgument(
            "go2_perturbation_force_n", default_value="0.0",
            description="Opt-in Go2 MuJoCo body-frame force pulse in newtons; "
                        "0.0 disables the diagnostic perturbation."),
        DeclareLaunchArgument(
            "go2_perturbation_start_s", default_value="0.0",
            description="Simulation-time start of the Go2 force pulse in seconds."),
        DeclareLaunchArgument(
            "go2_perturbation_duration_s", default_value="0.0",
            description="Duration of the Go2 force pulse in seconds; 0.0 disables it."),
        DeclareLaunchArgument(
            "go2_perturbation_axis", default_value="1",
            description="Go2 body-frame force axis: 0=x, 1=y, 2=z."),
        DeclareLaunchArgument(
            "enable_fall_recovery", default_value="false",
            description="Opt-in experimental Go2 re-stand attempt after a "
                        "detected fall; off by default because it is not "
                        "qualified on this plant."),
        DeclareLaunchArgument(
            "fall_recovery_timeout_s", default_value="4.0",
            description="Bounded window for the Go2 re-stand attempt."),
        DeclareLaunchArgument(
            "fall_recovery_start_delay_s", default_value="0.0",
            description="Opt-in zero-effort settling delay before Go2 recovery "
                        "starts; timeout begins after this delay."),
        DeclareLaunchArgument(
            "fall_recovery_gain_scale", default_value="0.5",
            description="Stance gain scale used during the Go2 re-stand attempt."),
        DeclareLaunchArgument(
            "fall_recovery_damping_scale", default_value="0.5",
            description="Stance damping scale used during the Go2 re-stand attempt."),
        DeclareLaunchArgument("display_hold", default_value="auto",
                              description="Hold the joints of robots without drive wheels or "
                                          "their own controllers at their spawn pose (PyBullet, "
                                          "MuJoCo, Isaac; Gazebo in display); false enables free physics."),
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
