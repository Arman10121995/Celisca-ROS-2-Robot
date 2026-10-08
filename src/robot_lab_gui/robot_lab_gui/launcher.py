#!/usr/bin/env python3

import os
from pathlib import Path
import queue
import shlex
import shutil
import signal
import sqlite3
import subprocess
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, simpledialog, ttk

from ament_index_python.packages import get_package_share_directory
from .process_control import stop_group, stop_owned_launch
from .drive_control import LinuxJoystick, RampDrive, limits_from_drive
from robot_lab_utils.installed_assets import merge_installed_profiles
from robot_lab_utils.asset_groups import AssetGroups
from robot_lab_utils.robot_taxonomy import RobotTaxonomy, ALL_CATEGORIES, ALL_TYPES, CATEGORIES

try:
    from robot_lab_utils.mode_capability import (
        describe_missing as describe_missing_features,
        missing_features as missing_robot_features,
    )
except ImportError:  # pragma: no cover - robot_lab_utils is a dependency
    def missing_robot_features(_config, _mode, _simulator=None, _mode_config=None):
        return []

    def describe_missing_features(missing):
        return "robot lacks " + ", ".join(missing)

try:
    import rclpy
    from geometry_msgs.msg import Twist
except ImportError:
    rclpy = None
    Twist = None

try:
    import yaml
except ImportError:
    yaml = None

# ── Modern theme ─────────────────────────────────────────────────────────
try:
    from .themes.modern import (
        apply as apply_theme, tooltip as add_tooltip,
        STATUS_OK, STATUS_ERROR,
    )
    THEME_AVAILABLE = True
except ImportError:
    THEME_AVAILABLE = False
    STATUS_OK = STATUS_ERROR = "#555555"
    def apply_theme(_root): return {}
    def add_tooltip(_w, _t): pass

# ── Launch profiles ──────────────────────────────────────────────────────
try:
    from .launch_profiles import (
        list_profiles, save_profile, load_profile, delete_profile,
        ensure_defaults, save_manifest, is_manifest,
    )
    PROFILES_AVAILABLE = True
except ImportError:
    PROFILES_AVAILABLE = False
    def list_profiles(): return []
    def save_profile(_n, _c): pass
    def save_manifest(_n, _m): return ""
    def load_profile(_n): return None
    def delete_profile(_n): return False
    def ensure_defaults(): pass
    def is_manifest(_c): return False

# ── Headless composition logic (R3.4) ─────────────────────────────────────
try:
    from .gui_composition import (
        ALGORITHM_SLOT_LABELS,
        GuiCompositionSelection,
        environment_id_for_map_name,
        get_registry,
        migrate_legacy_selection,
        resolve_selection,
        validation_lines,
    )
    COMPOSITION_AVAILABLE = True
except ImportError as _import_error:
    COMPOSITION_AVAILABLE = False
    _COMPOSITION_IMPORT_ERROR = _import_error

# ── Simulator availability + compatibility gating (headless, testable) ────
try:
    from .simulator_compat import (
        SIMULATOR_FEATURE_GAPS,
        allowed_simulators as _allowed_simulators,
        available_simulators as _available_simulators,
        correction_for as _correction_for,
        mode_algorithm_categories as _mode_algorithm_categories,
        mode_category as _mode_category,
        mode_default_algorithms as _mode_default_algorithms,
        mode_steps as _mode_steps,
        simulator_supports_mode as _simulator_supports_mode,
    )
    SIMULATOR_COMPAT_AVAILABLE = True
except ImportError:  # pragma: no cover — defensive
    SIMULATOR_COMPAT_AVAILABLE = False
    SIMULATOR_FEATURE_GAPS = {}

    def _mode_steps(_mode, _profiles=None):
        return []

    def _mode_algorithm_categories(_mode, _profiles=None):
        return []

    def _mode_default_algorithms(_mode, _profiles=None):
        return {}

    def _mode_category(mode, _profiles=None):
        return str(mode).title()

    def _available_simulators(env=None):
        return {s: (True, "") for s in SIMULATOR_ORDER}

    def _allowed_simulators(mode, mode_profiles=None, env=None):
        return {s: (True, "") for s in SIMULATOR_ORDER}

    def _correction_for(current, allowed, fallback_order=None):
        if not allowed:
            return None, "no compatible option available"
        if current in allowed:
            return current, ""
        for candidate in (fallback_order or []):
            if candidate in allowed:
                return candidate, "corrected"
        return allowed[0], "corrected"

    def _simulator_supports_mode(simulator, mode, mode_profiles=None):
        return True


# Sentinel entry for the robot / map selectors.  Display mode can run with
# only a robot (no world) or only a world (no robot), so both selectors carry
# an explicit "nothing selected" choice that resolves to `none` on the command
# line rather than to an empty string.
NONE_LABEL = "— None —"
NONE_VALUE = "none"


def is_none_selection(value):
    """True when a selector holds the explicit 'nothing selected' entry."""
    return str(value).strip() in ("", NONE_LABEL, NONE_VALUE)


def selection_value(value):
    """Selector value as the launch layer expects it ('none' for the sentinel)."""
    return NONE_VALUE if is_none_selection(value) else str(value).strip()


MODE_ORDER = ["display", "loc", "slam", "3d_slam", "nav", "flight"]
MODE_TOOLTIPS = {
    "display": "Visualize a robot and/or a map in the selected simulator "
               "(unactuated legged robots hold their pose; no localization).",
    "loc": "Localization: localize against a known map.",
    "slam": "SLAM: build a 2D map while localizing.",
    "3d_slam": "3D SLAM: build a 3D map (RGB-D sensor required).",
    "nav": "Navigation: plan and follow paths (2D map required).",
    "flight": "PX4 X500: explicit takeoff, manual flight and 3D waypoint control. No obstacle avoidance.",
}
MODE_LABELS = {
    "display": "Display",
    "loc": "Localization",
    "slam": "SLAM",
    "3d_slam": "3D SLAM",
    "nav": "Navigation",
    "flight": "PX4 Flight",
}

SIMULATOR_ORDER = ["gazebo", "isaac", "pybullet", "mujoco"]
SIMULATOR_LABELS = {
    "gazebo": "Gazebo",
    "isaac": "Isaac Sim",
    "pybullet": "PyBullet",
    "mujoco": "MuJoCo",
}

# Algorithm categories from registry
ALGORITHM_CATEGORIES = [
    "perception",
    "localization",
    "state_estimation",
    "sensor_fusion",
    "global_planning",
    "local_planning",
    "control",
]

# Maps mode → default algorithm category
MODE_TO_ALGORITHM_CATEGORY = {
    "display": "perception",
    "loc": "localization",
    "slam": "localization",
    "3d_slam": "localization",
    "nav": "global_planning",
}


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


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as yaml_file:
        if yaml is not None:
            return yaml.safe_load(yaml_file) or {}
        return _load_simple_yaml(yaml_file.read(), path)


def package_path(package_name, relative_path):
    if not relative_path:
        return ""
    if os.path.isabs(str(relative_path)):
        return str(relative_path)
    try:
        return os.path.join(get_package_share_directory(package_name), *str(relative_path).split("/"))
    except Exception:
        # Installed package name may differ from the config key (e.g. "maps"
        # -> robot_lab_maps). Never let a missing optional asset crash the GUI;
        # callers treat a non-existent path as "asset unavailable".
        return ""


def bool_value(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ("1", "true", "yes", "on")


def subprocess_env():
    env = os.environ.copy()
    if not env.get("ROS_LOG_DIR"):
        log_dir = Path.cwd() / "log" / "ros"
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
        except OSError:
            log_dir = Path("/tmp/robot_lab_ros_logs")
            log_dir.mkdir(parents=True, exist_ok=True)
        env["ROS_LOG_DIR"] = str(log_dir)
    if not env.get("ROBOT_LAB_RTABMAP_DIR") and not env.get("BUMPERBOT_RTABMAP_DIR"):
        rtabmap_dir = Path.cwd() / "log" / "rtabmap"
        rtabmap_dir.mkdir(parents=True, exist_ok=True)
        env["ROBOT_LAB_RTABMAP_DIR"] = str(rtabmap_dir)
    return env


class SimulationLauncherGui(tk.Tk):
    def __init__(self):
        super().__init__()
                # Apply modern dark theme
        if THEME_AVAILABLE:
            self.fonts = apply_theme(self)
        else:
            self.fonts = {}

        self.title("Robot Lab Control Center")
        self.geometry("1600x980")
        self.minsize(1024, 768)

        # Keyboard shortcuts
        self.bind_all("<Control-Return>", lambda _e: self._start_launch())
        self.bind_all("<Control-r>", lambda _e: self._refresh_maps())
        self.bind_all("<Escape>", lambda _e: self._stop_launch())

        # Load saved launch profiles
        if PROFILES_AVAILABLE:
            ensure_defaults()

        self._profiles_menu = None
        self._profile_names = []

        self.bringup_share = get_package_share_directory("robot_lab_bringup")
        self.robots_share = get_package_share_directory("robot_lab_robots")
        self.maps_share = get_package_share_directory("robot_lab_maps")

        self.modes_config_path = os.path.join(self.bringup_share, "config", "sim_modes.yaml")
        self.maps_config_path = os.path.join(self.bringup_share, "config", "sim_maps.yaml")
        self.robots_config_path = os.path.join(self.robots_share, "config", "robots.yaml")

        self.mode_profiles = load_yaml(self.modes_config_path).get("modes", {})
        self.map_profiles = merge_installed_profiles(load_yaml(self.maps_config_path), 'maps').get('maps', {})
        self.robot_profiles = merge_installed_profiles(load_yaml(self.robots_config_path), 'robots').get('robots', {})
        self.robot_groups = AssetGroups('robots', self.robot_profiles)
        self.map_groups = AssetGroups('maps', self.map_profiles)
        self.robot_taxonomy = RobotTaxonomy(self.robot_groups)

        # Which simulators are actually usable on this host (installed
        # binaries / configured runtimes) — gates the Simulator combo.
        self.simulator_status = _available_simulators()

        # Load algorithms from registry
        self.algorithms_config_path = os.path.join(
            get_package_share_directory("robot_lab_registry"),
            "config",
            "algorithms.yaml",
        )
        self.algorithms = self._load_algorithms()
        self.algorithm_dispatch = self._load_algorithm_dispatch()
        self.slot_vars = {}
        self.slot_combos = {}
        self.slot_labels = {}
        self._mode_step_categories = {}
        self._last_algorithm_robot = None
        self._last_algorithm_mode = None
        self._pending_manifest_algos = {}
        self._compat_cache = {}
        self._cleared_selections = []
        self.compatibility_var = tk.StringVar(value="")
        self.validation_var = tk.StringVar(value="Valid")
        self.composition_registry = None
        if COMPOSITION_AVAILABLE:
            try:
                self.composition_registry = get_registry()
            except Exception as _reg_error:  # pragma: no cover — installed GUI path
                self._compo_error = str(_reg_error)

        self.process = None
        self.ros_node = None
        self.cmd_vel_pub = None
        self.drive_repeat_job = None
        self.drive_model = RampDrive()
        self.drive_strafe_model = RampDrive()
        self.drive_altitude_model = RampDrive()
        self.drive_altitude_buttons = set()
        self.drive_altitude_widgets = {}
        self.current_vertical = 0.0
        self.drive_buttons = set()
        self.drive_button_widgets = {}
        self.drive_button_mirrors = {}
        self.drive_strafe_buttons = set()
        self.drive_strafe_widgets = {}
        self.drive_strafe_mirrors = {}
        self.current_lateral = 0.0
        self.drive_keys = set()
        self.drive_stop_latched = False
        self.drive_joystick = LinuxJoystick()
        self.current_drive = (0.0, 0.0)
        self._launch_running = False
        self._last_fixes = []
        self.output_queue = queue.Queue()
        self._lifecycle_queue = queue.Queue()
        self._aux_processes = set()
        self._aux_lock = threading.Lock()
        self._aux_stopping = threading.Event()
        self._output_autoscroll = True  # follow-tail for Launch Output

        self.robot_var = tk.StringVar(
            value=self._first_key(self.robot_profiles, "bumperbot"))
        self.map_var = tk.StringVar(
            value=self._first_key(self.map_profiles, "celisca_floor_1"))
        self.robot_family_var = tk.StringVar(value=self.robot_groups.family(self.robot_var.get()))
        self.map_family_var = tk.StringVar(value=self.map_groups.family(self.map_var.get()))
        self.robot_category_var = tk.StringVar(value=ALL_CATEGORIES)
        self.robot_subtype_var = tk.StringVar(value=ALL_TYPES)
        self.robot_tags_var = tk.StringVar()
        self.mode_var = tk.StringVar(value="display")
        self.simulator_var = tk.StringVar(value="gazebo")
        self.launch_kind_var = tk.StringVar(value="simulation")
        self.drive_linear_var = tk.DoubleVar(value=0.025)
        self.drive_angular_var = tk.DoubleVar(value=0.08)
        self.drive_decel_linear_var = tk.DoubleVar(value=0.025)
        self.drive_decel_angular_var = tk.DoubleVar(value=0.08)
        self.drive_max_linear_var = tk.DoubleVar(value=1.0)
        self.drive_max_angular_var = tk.DoubleVar(value=2.0)
        self.drive_min_linear_var = tk.DoubleVar(value=-1.0)
        self.drive_min_angular_var = tk.DoubleVar(value=-2.0)
        self.drive_override_var = tk.BooleanVar(value=False)
        self.drive_limits_var = tk.StringVar(value="")
        self.drive_input_enabled = tk.BooleanVar(value=False)
        self.go2_policy_var = tk.BooleanVar(value=False)
        self.bhl_policy_var = tk.BooleanVar(value=True)
        self.steering_mode_var = tk.StringVar(value="")
        self.drive_status_var = tk.StringVar(value="Keyboard/joystick off")
        self.gui_var = tk.StringVar(value="auto")
        self.command_var = tk.StringVar()
        self._prepared_command = []
        self.summary_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Idle")

        self.mode_buttons = {}
        self._build_ui()
        self.bind_all("<KeyPress>", self._drive_key_press, add="+")
        self.bind_all("<KeyRelease>", self._drive_key_release, add="+")
        self.bind_all("<FocusOut>", lambda _event: self.drive_keys.clear(), add="+")
        self._update_from_selection()
        self.after(100, self._poll_output)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    @staticmethod
    def _first_key(mapping, preferred):
        if preferred in mapping:
            return preferred
        return next(iter(mapping), "")

    def _load_algorithms(self):
        """Load algorithms from the registry YAML."""
        raw = load_yaml(self.algorithms_config_path)
        if isinstance(raw, list):
            return raw
        if isinstance(raw, dict):
            return raw.get("algorithms", [])
        return []

    def _load_algorithm_dispatch(self):
        """Load how each algorithm is applied at launch (bringup config).

        The GUI must not offer an algorithm the launch layer would refuse:
        entries marked `unavailable` are cataloged only (not built in this
        workspace), so they are filtered out with their reason attached.
        """
        try:
            path = package_path(
                "robot_lab_bringup", "config/algorithm_dispatch.yaml")
        except Exception:
            return {}
        if not path or not os.path.exists(path):
            return {}
        raw = load_yaml(path)
        if not isinstance(raw, dict):
            return {}
        return raw.get("algorithms", {}) or {}

    def _algorithm_unavailable_reason(self, category, algorithm_id):
        """Why *algorithm_id* cannot be launched, or '' when it can."""
        entry = (self.algorithm_dispatch.get(category) or {}).get(algorithm_id)
        if entry is None:
            return ("not wired into the launch layer "
                    "(missing from algorithm_dispatch.yaml)")
        return entry.get("unavailable", "") or ""

    def _algorithm_unsuitable_reason(self, category, algorithm_id):
        """Why *algorithm_id* cannot drive the selected robot, or ''.

        algorithm_dispatch.yaml's ``not_for_features`` against the robot
        profile's features (e.g. DWB turns a robot on the spot, which a car
        with ``car_steering`` cannot do); the launch refuses the same pairs.
        """
        if category == 'local_planning' and self._four_wheel_steer_selectable() \
                and self.steering_mode_var.get() in ('crab', 'in_phase') \
                and algorithm_id != 'dwb_local_planner':
            return 'Parallel steering uses DWB with a translation/pivot constraint.'
        entry = (self.algorithm_dispatch.get(category) or {}).get(algorithm_id) or {}
        excluded = set(entry.get("not_for_features") or [])
        if not excluded or self._robot_free():
            return ""
        features = set((self._robot_config() or {}).get("features") or [])
        if excluded & features:
            return entry.get("not_for_reason", "incompatible with this robot")
        return ""

    def _algorithms_for_category(self, category):
        """Runnable algorithm IDs for *category*.

        Filtered by what the bringup layer can actually start, so a slot
        never lists an option that would fail the launch.
        """
        return [
            a["id"] for a in self.algorithms
            if a.get("category") == category
            and not self._algorithm_unavailable_reason(category, a["id"])
            and not self._algorithm_unsuitable_reason(category, a["id"])
        ]

    def _robot_default_algorithm(self, category):
        """The robot profile's replacement for a mode default, or ''."""
        defaults = (self._robot_config() or {}).get("default_algorithms") or {}
        return str(defaults.get(category, "") or "")

    def _algorithm_name(self, algorithm_id):
        """Return the human-readable name for an algorithm ID."""
        for a in self.algorithms:
            if a.get("id") == algorithm_id:
                return a.get("name", algorithm_id)
        return algorithm_id

    def _build_ui(self):
        from .workspace_ui import build_workspace, ScrollPanel
        controls, advanced = build_workspace(self)
        launch_tab = self.launch_tab
        output_frame = self.session_frame
        output_frame.columnconfigure(0, weight=1)
        output_frame.rowconfigure(3, weight=1)
        self.session_details = ttk.Notebook(output_frame)
        self.session_details.grid(row=3, column=0, sticky='nsew', pady=(6, 0))
        algorithms = ScrollPanel(self.session_details, width=430)
        self.session_details.add(algorithms, text='Algorithms & checks')
        algorithm_controls = algorithms.body
        diagnostics = ScrollPanel(self.session_details, width=430)
        self.session_details.add(diagnostics, text='Details')
        diagnostics_controls = diagnostics.body
        self.launch_log_page = ttk.Frame(self.session_details)
        self.launch_log_page.columnconfigure(0, weight=1)
        self.launch_log_page.rowconfigure(0, weight=1)
        self.session_details.add(self.launch_log_page, text='Launch log')
        self.session_action_frame = ttk.Frame(output_frame)
        self.session_action_frame.grid(row=2, column=0, sticky='ew')
        advanced.columnconfigure(0, weight=1)
        controls.columnconfigure(0, weight=1)

        ttk.Label(controls, text="Robot").grid(row=0, column=0, sticky="w")
        robot_frame = ttk.Frame(controls)
        robot_frame.grid(row=1, column=0, sticky="ew", pady=(2, 12))
        robot_frame.columnconfigure(0, weight=1)
        filters = ttk.Frame(robot_frame)
        filters.grid(row=0, column=0, sticky='ew', pady=(0, 8))
        filters.columnconfigure(0, weight=1); filters.columnconfigure(1, weight=1)
        self.robot_category_combo = ttk.Combobox(filters, textvariable=self.robot_category_var,
            values=(ALL_CATEGORIES, *CATEGORIES), state='readonly', width=18)
        self.robot_category_combo.grid(row=0, column=0, columnspan=2, sticky='ew')
        self.robot_subtype_combo = ttk.Combobox(filters, textvariable=self.robot_subtype_var,
            values=(ALL_TYPES, *self.robot_taxonomy.subtypes()), state='readonly', width=17)
        self.robot_subtype_combo.grid(row=1, column=0, columnspan=2, sticky='ew', pady=(4, 0))
        self.robot_category_combo.bind('<<ComboboxSelected>>', self._robot_filter_changed)
        self.robot_subtype_combo.bind('<<ComboboxSelected>>', self._robot_filter_changed)
        self.robot_combo = ttk.Combobox(
            robot_frame,
            textvariable=self.robot_family_var,
            values=[NONE_LABEL] + self.robot_groups.choices(),
            state="readonly",
            width=26,
        )
        self.robot_combo.grid(row=1, column=0, sticky="ew")
        self.robot_combo.bind("<<ComboboxSelected>>", self._robot_family_selected)
        add_tooltip(
            self.robot_combo,
            "Robot to simulate. '%s' shows the map with no robot "
            "(display mode only)." % NONE_LABEL)
        ttk.Label(robot_frame, text='Model / source variant').grid(row=2, column=0, sticky='w', pady=(4, 0))
        self.robot_variant_combo = ttk.Combobox(robot_frame, textvariable=self.robot_var, state='readonly', width=26)
        self.robot_variant_combo.grid(row=3, column=0, sticky='ew')
        self.robot_variant_combo.bind('<<ComboboxSelected>>', self._on_selection_changed)
        self.robot_info_var = tk.StringVar(value="")
        ttk.Label(
            robot_frame,
            textvariable=self.robot_info_var,
            foreground="#a6adc8" if THEME_AVAILABLE else "#555555",
            wraplength=295,
            justify="left",
        ).grid(row=5, column=0, sticky="ew", pady=(4, 0))

        ttk.Label(robot_frame, textvariable=self.robot_tags_var, style='Tag.TLabel', wraplength=300).grid(row=4, column=0, sticky='w', pady=(6, 0))
        ttk.Label(controls, text="Mode").grid(row=2, column=0, sticky="w")
        mode_frame = ttk.Frame(controls)
        mode_frame.grid(row=3, column=0, sticky="ew", pady=(2, 12))
        for index, mode in enumerate(MODE_ORDER):
            button = ttk.Radiobutton(
                mode_frame,
                text=MODE_LABELS.get(mode, mode),
                variable=self.mode_var,
                value=mode,
                command=self._update_from_selection,
            )
            button.grid(row=index//2, column=index%2, sticky="w", pady=2)
            self.mode_buttons[mode] = button
            add_tooltip(button, MODE_TOOLTIPS.get(mode, ""))

        ttk.Label(controls, text="Map").grid(row=4, column=0, sticky="w")
        map_frame = ttk.Frame(controls)
        map_frame.grid(row=5, column=0, sticky='ew', pady=(2, 12))
        map_frame.columnconfigure(0, weight=1)
        self.map_combo = ttk.Combobox(
            map_frame,
            textvariable=self.map_family_var,
            values=[NONE_LABEL] + self.map_groups.choices(),
            state="readonly",
            width=26,
        )
        self.map_combo.grid(row=0, column=0, sticky="ew")
        self.map_combo.bind("<<ComboboxSelected>>", self._map_family_selected)
        ttk.Label(map_frame, text='World variant').grid(row=1, column=0, sticky='w', pady=(4, 0))
        self.map_variant_combo = ttk.Combobox(map_frame, textvariable=self.map_var, state='readonly', width=26)
        self.map_variant_combo.grid(row=2, column=0, sticky='ew')
        self.map_variant_combo.bind('<<ComboboxSelected>>', self._on_selection_changed)
        add_tooltip(
            self.map_combo,
            "Environment to load. '%s' shows the robot with no world "
            "(display mode only)." % NONE_LABEL)

        ttk.Label(controls, text="Simulator").grid(row=6, column=0, sticky="w")
        self.simulator_combo = ttk.Combobox(
            controls,
            textvariable=self.simulator_var,
            values=[v for v in SIMULATOR_ORDER],
            state="readonly",
            width=26,
        )
        self.simulator_combo.grid(row=7, column=0, sticky="ew", pady=(2, 12))
        self.simulator_combo.bind("<<ComboboxSelected>>", self._simulator_selected)
        add_tooltip(self.simulator_combo, "Select the simulator backend.")

        ttk.Label(advanced, text="Launch").grid(row=8, column=0, sticky="w")
        launch_frame = ttk.Frame(advanced)
        launch_frame.grid(row=9, column=0, sticky="ew", pady=(2, 12))
        self.simulation_radio = ttk.Radiobutton(
            launch_frame,
            text="Simulation",
            variable=self.launch_kind_var,
            value="simulation",
            command=self._update_from_selection,
        )
        self.simulation_radio.grid(row=0, column=0, sticky="w", pady=2)
        add_tooltip(self.simulation_radio, "Run a full simulation (Gazebo/Isaac/PyBullet/MuJoCo).")
        self.vacuum_radio = ttk.Radiobutton(
            launch_frame,
            text="Room vacuum",
            variable=self.launch_kind_var,
            value="vacuum",
            command=self._update_from_selection,
        )
        self.vacuum_radio.grid(row=1, column=0, sticky="w", pady=2)
        add_tooltip(self.vacuum_radio, "Run a vacuum cleaning mission (robot must support it).")

        ttk.Label(advanced, text="GUI").grid(row=10, column=0, sticky="w", pady=(12, 0))
        gui_frame = ttk.Frame(advanced)
        gui_frame.grid(row=11, column=0, sticky="ew", pady=(2, 12))
        gui_tooltip = "Auto: GUI if DISPLAY is set. GUI: always launch the simulator UI. Headless: no UI."
        for column, text in enumerate(["Auto", "GUI", "Headless"]):
            rb = ttk.Radiobutton(
                gui_frame,
                text=text,
                variable=self.gui_var,
                value=["auto", "true", "false"][column],
                command=self._update_from_selection,
            )
            rb.grid(row=0, column=column, sticky="w", padx=(0, 12))
            add_tooltip(rb, gui_tooltip)

        # --- Simulator availability (host-detected; R3.4+ gating) ---------
        sim_panel = ttk.LabelFrame(
            advanced, text="Simulator availability", padding=(8, 4))
        sim_panel.grid(row=12, column=0, sticky="ew", pady=(0, 10))
        self.simulator_status_labels = {}
        for index, sim_id in enumerate(SIMULATOR_ORDER):
            row_label = ttk.Label(
                sim_panel,
                text="… %s" % SIMULATOR_LABELS.get(sim_id, sim_id),
                justify="left",
                wraplength=330,
            )
            row_label.grid(row=index, column=0, sticky="w")
            self.simulator_status_labels[sim_id] = row_label

        # --- Full composition controls (R3.4): dynamic algorithm slots ---
        # The slots are rebuilt when the mode changes (see _refresh_mode_steps).
        self.composition_frame = ttk.LabelFrame(
            algorithm_controls, text="Composition - algorithm slots", padding=(8, 6))
        self.composition_frame.grid(row=13, column=0, sticky="ew", pady=(12, 6))
        self.composition_frame.columnconfigure(1, weight=1)
        self._current_mode_steps = None  # Track when mode changes

        # Compatibility status label (color-coded)
        self.compatibility_label = ttk.Label(
            algorithm_controls,
            textvariable=self.compatibility_var,
            justify="left",
            wraplength=330,
            foreground="#a6adc8" if THEME_AVAILABLE else "#333333",
        )
        self.compatibility_label.grid(row=14, column=0, sticky="ew", pady=(0, 2))
        add_tooltip(self.compatibility_label,
                   "Shows whether the current robot/mode supports all slots.")

        # Separator between composition and validation
        ttk.Separator(algorithm_controls, orient="horizontal").grid(
            row=15, column=0, sticky="ew", pady=(2, 4))

        # Action buttons for composition management
        action_frame = ttk.Frame(algorithm_controls)
        action_frame.grid(row=16, column=0, sticky="ew", pady=(2, 4))
        action_frame.columnconfigure(0, weight=1)
        action_frame.columnconfigure(1, weight=1)
        action_frame.columnconfigure(2, weight=1)
        action_frame.columnconfigure(3, weight=1)

        ttk.Button(action_frame, text="Reset",
                   command=self._reset_composition,
                   style="Small.TButton").grid(row=0, column=0, sticky="ew", padx=(0, 3))
        ttk.Button(action_frame, text="Compatibility",
                   command=self._show_incompatible,
                   style="Small.TButton").grid(row=0, column=1, sticky="ew", padx=3)
        ttk.Button(action_frame, text="Presets",
                   command=self._quick_select,
                   style="Small.TButton").grid(row=0, column=2, sticky="ew", padx=3)
        ttk.Button(action_frame, text="Cleared choices",
                   command=self._show_cleared,
                   style="Small.TButton").grid(row=0, column=3, sticky="ew", padx=(3, 0))

        ttk.Label(diagnostics_controls, text="Validation", foreground="#a6adc8" if THEME_AVAILABLE else "#333333"
                  ).grid(row=17, column=0, sticky="w", pady=(4, 2))
        ttk.Label(
            diagnostics_controls,
            textvariable=self.validation_var,
            justify="left",
            wraplength=330,
            foreground="#a6adc8" if THEME_AVAILABLE else "#333333",
        ).grid(row=18, column=0, sticky="ew", pady=(2, 8))

        ttk.Label(diagnostics_controls, text="Resolved Configuration").grid(row=19, column=0, sticky="w")
        summary = ttk.Label(
            diagnostics_controls,
            textvariable=self.summary_var,
            justify="left",
            wraplength=330,
            foreground="#a6adc8" if THEME_AVAILABLE else "#333333",
        )
        summary.grid(row=20, column=0, sticky="ew", pady=(2, 12))

        button_frame = ttk.Frame(advanced)
        button_frame.grid(row=21, column=0, sticky="ew")
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)
        button_frame.columnconfigure(2, weight=1)

        # Launch profile buttons
        if PROFILES_AVAILABLE:
            ttk.Button(button_frame, text="Save Profile",
                       command=self._save_profile,
                       style="Small.TButton").grid(row=0, column=0, sticky="ew", padx=4)
            ttk.Button(button_frame, text="Load",
                       command=self._show_load_profile,
                       style="Small.TButton").grid(row=0, column=1, sticky="ew", padx=4)
            ttk.Button(button_frame, text="Delete/Profiles",
                       command=self._delete_profile,
                       style="Small.TButton").grid(row=0, column=2, sticky="ew", padx=(4, 0))

        self.bg_processes = {}

        self.go2_policy_checkbox = ttk.Checkbutton(
            advanced,
            text="Go2 flat-ground policy (experimental)",
            variable=self.go2_policy_var,
            command=self._update_validation_and_command,
        )
        self.go2_policy_checkbox.grid(row=23, column=0, sticky="w", pady=(6, 0))
        add_tooltip(self.go2_policy_checkbox,
                    "MuJoCo localization only. Forward and turn were measured; "
                    "slow reverse and stairs still fail qualification.")

        self.bhl_policy_checkbox = ttk.Checkbutton(
            advanced,
            text="BHL walking policy (experimental)",
            variable=self.bhl_policy_var,
            command=self._update_validation_and_command,
        )
        self.bhl_policy_checkbox.grid(row=22, column=0, sticky="w", pady=(10, 0))
        add_tooltip(self.bhl_policy_checkbox,
                    "MuJoCo localization only. Starts the ONNX effort policy "
                    "the Drive pad walks with: forward walk and stop measured; "
                    "held turning stalls (policy fixed point) and reverse is "
                    "unqualified. Uncheck for the passive spawn stance with no "
                    "policy node.")

        steering_frame = self.steering_frame = ttk.Frame(self.drive_pad_host)
        steering_frame.grid(row=0, column=0, sticky="ew", pady=(8, 0))
        ttk.Label(steering_frame, text="4WS pattern").pack(side="left")
        self.steering_mode_combo = ttk.Combobox(
            steering_frame, textvariable=self.steering_mode_var,
            values=("", "ackermann", "in_phase", "crab", "pivot"),
            state="disabled", width=11)
        self.steering_mode_combo.pack(side="left", padx=(6, 0))
        self.steering_mode_combo.bind(
            "<<ComboboxSelected>>",
            lambda _event: self._update_from_selection())
        add_tooltip(self.steering_mode_combo,
                    "Four-wheel-steer pattern for four_wheel_steer_car, passed "
                    "as steering_mode:= to the launch.  Empty keeps the "
                    "profile default (ackermann = opposite-phase).  Patterns "
                    "are measured per backend under R5.6; see the ledger.")

        drive_frame = ttk.Frame(self.drive_pad_host)
        drive_frame.grid(row=1, column=0, sticky="ew", pady=(2, 8))
        for column in range(3):
            drive_frame.columnconfigure(column, weight=1)

        forward_button = ttk.Button(drive_frame, text="Forward", width=8)
        forward_button.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
        self._bind_drive_button(forward_button, 1.0, 0.0)

        self.strafe_left_button = ttk.Button(drive_frame, text="Strafe L", width=8)
        self.strafe_left_button.grid(row=0, column=0, sticky="ew", padx=2, pady=2)
        self._bind_strafe_button(self.strafe_left_button, 1.0)

        self.strafe_right_button = ttk.Button(drive_frame, text="Strafe R", width=8)
        self.strafe_right_button.grid(row=0, column=2, sticky="ew", padx=2, pady=2)
        self._bind_strafe_button(self.strafe_right_button, -1.0)

        left_button = ttk.Button(drive_frame, text="Left", width=8)
        left_button.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
        self._bind_drive_button(left_button, 0.0, 1.0)

        stop_drive_button = ttk.Button(drive_frame, text="Stop", command=self._stop_drive, width=8)
        stop_drive_button.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

        right_button = ttk.Button(drive_frame, text="Right", width=8)
        right_button.grid(row=1, column=2, sticky="ew", padx=2, pady=2)
        self._bind_drive_button(right_button, 0.0, -1.0)

        reverse_button = ttk.Button(drive_frame, text="Reverse", width=8)
        reverse_button.grid(row=2, column=1, sticky="ew", padx=2, pady=2)
        self._bind_drive_button(reverse_button, -1.0, 0.0)

        ttk.Label(self.drone_actions_host, text='Take off, then use the seven direction buttons below.\nClick once to move; click again to slow down. Stop halts all manual motion.', wraplength=310).grid(row=0, column=0, columnspan=3, sticky='w', pady=(0, 8))
        drone_drive = ttk.Frame(self.drone_actions_host)
        drone_drive.grid(row=1, column=0, columnspan=3, sticky='ew', pady=(0, 8))
        self.drone_drive_widgets = {}
        for column in range(3):
            drone_drive.columnconfigure(column, weight=1)
        for label, row, column, direction in (
                ('Strafe L', 0, 0, 1.), ('Forward', 0, 1, (1., 0.)),
                ('Strafe R', 0, 2, -1.), ('Turn Left', 1, 0, (0., 1.)),
                ('Stop', 1, 1, None), ('Turn Right', 1, 2, (0., -1.)),
                ('Reverse', 2, 1, (-1., 0.))):
            button = ttk.Button(drone_drive, text=label, width=8)
            button.grid(row=row, column=column, sticky='ew', padx=2, pady=2)
            self.drone_drive_widgets[label] = button
            if direction is None:
                button.configure(command=self._stop_drive)
            elif isinstance(direction, tuple):
                self._bind_drive_button(button, *direction, mirror=True)
            else:
                self._bind_strafe_button(button, direction, mirror=True)
        self.flight_buttons = []
        for column, (label, action) in enumerate((('Takeoff', 'takeoff'), ('Hold', 'hold'), ('Land', 'land'))):
            button = ttk.Button(self.drone_actions_host, text=label, width=8,
                                command=lambda action=action: self._flight_action(action))
            button.grid(row=3, column=column, sticky='ew', padx=2, pady=2)
            self.flight_buttons.append(button)
        altitude = ttk.Frame(self.drone_actions_host)
        altitude.grid(row=4, column=0, columnspan=3, sticky='ew')
        for column, (label, sign) in enumerate((('Altitude Up',1.0),('Altitude Down',-1.0))):
            button = ttk.Button(altitude,text=label, width=12,
                                command=lambda sign=sign:self._start_altitude(sign))
            button.grid(row=0,column=column,sticky='ew',padx=2,pady=2)
            self.drive_altitude_widgets[sign] = button
            self.flight_buttons.append(button)
            add_tooltip(button,'Click to climb or descend; click again to slow to a hover. Stop/Space stops altitude changes.')

        speed_frame = ttk.Frame(self.drive_limits_host)
        speed_frame.grid(row=0, column=0, sticky='ew')
        speed_frame.columnconfigure(0, weight=1)
        ttk.Checkbutton(speed_frame, text='Override robot drive limits', variable=self.drive_override_var,
            command=self._update_drive_limits_label).grid(row=0, column=0, columnspan=2, sticky='w')
        ttk.Label(speed_frame, textvariable=self.drive_limits_var, wraplength=305).grid(
            row=1, column=0, columnspan=2, sticky='w', pady=6)
        self.drive_limit_widgets = []
        for row, (label, variable, low, high, step) in enumerate((
            ('Δ linear / 0.1 s (m/s)', self.drive_linear_var, .001, 1., .005),
            ('Δ angular / 0.1 s (rad/s)', self.drive_angular_var, .001, 2., .01),
            ('Δ linear brake', self.drive_decel_linear_var, .001, 1., .005),
            ('Δ angular brake', self.drive_decel_angular_var, .001, 2., .01),
            ('Max linear (m/s)', self.drive_max_linear_var, .01, 5., .05),
            ('Max angular (rad/s)', self.drive_max_angular_var, .01, 10., .1),
            ('Min linear (m/s)', self.drive_min_linear_var, -5., 0., .05),
            ('Min angular (rad/s)', self.drive_min_angular_var, -10., 0., .1)), start=2):
            ttk.Label(speed_frame, text=label).grid(row=row, column=0, sticky='w', pady=2)
            spin = ttk.Spinbox(speed_frame, textvariable=variable, from_=low, to=high,
                increment=step, width=8)
            spin.grid(row=row, column=1, sticky='w', padx=(6, 0))
            self.drive_limit_widgets.append(spin)
        input_frame = ttk.Frame(speed_frame)
        input_frame.grid(row=10, column=0, columnspan=2, sticky='ew', pady=(8, 0))
        self.drive_input_checkbox = ttk.Checkbutton(input_frame, text='Enable WASD + joystick',
            variable=self.drive_input_enabled, command=self._toggle_drive_input)
        self.drive_input_checkbox.grid(row=0, column=0, sticky='w')
        ttk.Label(input_frame, textvariable=self.drive_status_var, wraplength=305, style='Muted.TLabel').grid(row=1, column=0, sticky='w', pady=4)

        self.save_map_button = ttk.Button(self.session_action_frame, text="Save Map", command=self._save_map)
        self.save_map_button.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        self.reset_robot_button = ttk.Button(self.session_action_frame, text="Reset Robot",
                                             command=self._reset_robot, state='disabled')
        self.reset_robot_button.grid(row=0, column=1, sticky='ew', pady=(0, 4))
        add_tooltip(self.reset_robot_button,
                    'Stop and return to the initial pose in Display, Localization or SLAM. '
                    'Restart the launch to reset the entire world.')

        ttk.Label(self.drive_pad_host, text='Click once to latch; click again to release.\nWASD/joystick starts neutral. Space stops Drive.',
            wraplength=280, style='Muted.TLabel').grid(row=2, column=0, sticky='w', pady=8)
        ttk.Label(self.drone_actions_host, text='Left/right rotate gently; strafe moves sideways.\nEnable WASD + joystick in Drive for keyboard input.', wraplength=305,
                  style='Muted.TLabel').grid(row=5, column=0, columnspan=3, sticky='w', pady=8)
        ttk.Label(self.drone_limits_host, textvariable=self.drive_limits_var, wraplength=305).grid(row=0, column=0, columnspan=2, sticky='w')
        ttk.Checkbutton(self.drone_limits_host, text='Override manual velocity limits', variable=self.drive_override_var,
            command=self._update_drive_limits_label).grid(row=1, column=0, columnspan=2, sticky='w', pady=8)
        for row, (label, variable, ceiling) in enumerate((
            ('Max manual speed (m/s)', self.drive_max_linear_var, 5.),
            ('Max manual yaw (rad/s)', self.drive_max_angular_var, 10.),
            ('Linear increment / 0.1 s', self.drive_linear_var, 1.),
            ('Yaw increment / 0.1 s', self.drive_angular_var, 2.)), start=2):
            ttk.Label(self.drone_limits_host, text=label).grid(row=row, column=0, sticky='w', pady=4)
            spin = ttk.Spinbox(self.drone_limits_host, textvariable=variable, from_=.001, to=ceiling, increment=.01,
                width=8)
            spin.grid(row=row, column=1, sticky='w', padx=8)
            self.drive_limit_widgets.append(spin)
        ttk.Label(self.drone_limits_host, text='Overrides stay within robot-profile caps.\nAltitude uses the same bounded linear increment; PX4 enforces its flight limits.',
            wraplength=305, style='Muted.TLabel').grid(row=6, column=0, columnspan=2, sticky='w', pady=10)
        command_header = ttk.Frame(output_frame)
        command_header.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        ttk.Label(command_header, text="Run the selected experiment").grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))
        self.copy_command_button = ttk.Button(
            command_header, text="Copy command", command=self._copy_command)
        self.copy_command_button.grid(row=1, column=1, padx=4)
        self.start_button = ttk.Button(
            command_header, text="Run Command", command=self._start_launch,
            style="Accent.TButton")
        self.start_button.grid(row=1, column=0, sticky='w', padx=(0, 4))
        self.stop_button = ttk.Button(
            command_header, text="Stop", command=self._stop_launch,
            state="disabled", style="Danger.TButton", width=6)
        self.stop_button.grid(row=1, column=2, padx=(4, 0))
        def fit_command_actions(event):
            if event.width < 340:
                self.copy_command_button.grid(row=2, column=0, columnspan=3, sticky='w', pady=(4, 0))
                self.stop_button.grid(row=1, column=1, padx=4)
            else:
                self.copy_command_button.grid(row=1, column=1, columnspan=1, sticky='w', pady=0)
                self.stop_button.grid(row=1, column=2, padx=(4, 0))
        command_header.bind('<Configure>', fit_command_actions)
        self.command_preview = scrolledtext.ScrolledText(
            output_frame, wrap="word", height=4, width=1, state="disabled")
        self.command_preview.grid(row=1, column=0, sticky="ew", pady=(0, 12))
        add_tooltip(self.command_preview,
                    "Updates from the selected options. Run here or copy into a ROS 2 terminal.")
        self.output = scrolledtext.ScrolledText(self.launch_log_page, wrap="word", height=8)
        self.output.grid(row=0, column=0, sticky="nsew")
        self.output.configure(state="disabled")

        # Shared console: every control-center tab streams its output here
        console_frame = ttk.LabelFrame(self, text="Console", padding=(12, 2, 12, 6))
        console_frame.grid(row=2, column=1, sticky="ew")
        self.console_frame = console_frame
        self.console_visible = False
        console_frame.grid_remove()
        console_frame.columnconfigure(0, weight=1)
        self.console = scrolledtext.ScrolledText(console_frame, wrap="word", height=6)
        self.console.grid(row=0, column=0, sticky="ew", pady=(2, 0))
        self.console.configure(state="disabled")

        # Status bar spans the full window below the console
        if THEME_AVAILABLE:
            status_frame = ttk.Frame(self, style="Statusbar.TFrame")
        else:
            status_frame = ttk.Frame(self, relief="sunken")
        status_frame.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(4, 0))

        status_style = "Statusbar.TLabel" if THEME_AVAILABLE else None
        self._ros_status_dot = tk.Label(status_frame, text="*",
                                        fg="#ff6b6b", font=("Segoe UI", 10))
        self._ros_status_dot.grid(row=0, column=0, padx=(8, 4))

        ttk.Label(status_frame, text="ROS 2:",
                  style=status_style).grid(
            row=0, column=1, padx=(0, 0))
        self.ros_status_var = tk.StringVar(value="checking...")
        ttk.Label(status_frame, textvariable=self.ros_status_var,
                  style=status_style).grid(
            row=0, column=2, padx=(4, 16))

        ttk.Label(status_frame, text="Processes:",
                  style=status_style).grid(
            row=0, column=3, padx=(0, 0))
        self.proc_count_var = tk.StringVar(value="0")
        ttk.Label(status_frame, textvariable=self.proc_count_var,
                  style=status_style).grid(
            row=0, column=4, padx=(4, 16))

        ttk.Label(status_frame, textvariable=self.status_var,
                  style=status_style).grid(
            row=0, column=5, sticky="w", padx=(0, 0))

        ttk.Label(status_frame, text="Ctrl+Enter launch | Ctrl+R refresh | Esc stop",
                  style=status_style).grid(
            row=0, column=6, sticky="e", padx=(16, 8))

        status_frame.columnconfigure(5, weight=1)

        # Check ROS status
        self._check_ros_status()

        # Remaining control-center tabs (Registry, Vacuum, Benchmark, Tests, Health)
        from .lab_tabs import create_tabs
        create_tabs(self.notebook, self)
        from .workspace_ui import build_navigation
        build_navigation(self)

    def _robot_filter_changed(self, _event=None):
        category = self.robot_category_var.get()
        subtypes = self.robot_taxonomy.subtypes(category)
        self.robot_subtype_combo.configure(values=(ALL_TYPES, *subtypes))
        if self.robot_subtype_var.get() not in subtypes:
            self.robot_subtype_var.set(ALL_TYPES)
        allowed = [name for name in self._allowed_robots() if name in self.robot_profiles
                   and self.robot_taxonomy.matches(name, category, self.robot_subtype_var.get())
                   and self.robot_groups.member(name).get('role') not in ('component', 'simplified', 'reference')]
        if allowed and self.robot_var.get() not in allowed:
            family = self.robot_groups.choices(allowed)[0]
            self.robot_var.set(self.robot_groups.preferred(family, allowed))
        self._update_from_selection()

    def _refresh_workspace_selection(self):
        robot = self.robot_var.get()
        if is_none_selection(robot):
            self.robot_tags_var.set('World only')
            self.control_context.set('World-only display · no robot selected')
            return
        info = self.robot_taxonomy.classify(robot)
        self.robot_tags_var.set('  ·  '.join([info['category'], info['subtype'], *info['tags']]))
        self.steering_frame.grid() if self._four_wheel_steer_selectable() else self.steering_frame.grid_remove()
        self.control_context.set(robot+' · '+info['category']+' / '+info['subtype']+
            ' · Controls require a compatible controller and an owned running simulation.')
        if robot != getattr(self, '_control_robot', None):
            self._control_robot = robot
            if self._flight_selectable():
                self.control_notebook.select(self.drone_page)
            elif self._robot_config().get('arm_control') and hasattr(self, 'arm_tab'):
                self.control_notebook.select(self.arm_tab)
            else:
                # Select the control page without leaving an asset browser.
                self.control_notebook.select(self.drive_page)

    def _toggle_console(self):
        self.console_visible = not getattr(self, 'console_visible', False)
        self.console_frame.grid() if self.console_visible else self.console_frame.grid_remove()

    def _stop_all_motion(self):
        self._stop_drive()
        for name in ('arm_tab', 'hand_tab'):
            control = getattr(self, name, None)
            if control is not None:
                control.stop()

    def _on_selection_changed(self, _event):
        self._update_from_selection()

    def _robot_family_selected(self, _event=None):
        requested = self.robot_family_var.get()
        if is_none_selection(requested):
            self.robot_var.set(NONE_LABEL)
        elif requested in self.robot_profiles and requested not in self.robot_groups.groups:
            # Preserve programmatic selections and saved legacy profile IDs.
            self.robot_var.set(requested)
        else:
            members = self.robot_groups.members(requested, selectable=True)
            mode, backend = self.mode_var.get(), self.simulator_var.get()
            compatible = [member for member in members if mode in self.robot_profiles[member].get('supported_modes', ['display'])
                          and not missing_robot_features(self.robot_profiles[member], mode, backend, self.mode_profiles.get(mode))]
            selected = self.robot_groups.preferred(requested, compatible or members)
            if selected:
                self.robot_var.set(selected)
        self._update_from_selection()

    def _map_family_selected(self, _event=None):
        requested = self.map_family_var.get()
        if is_none_selection(requested):
            self.map_var.set(NONE_LABEL)
        elif requested in self.map_profiles and requested not in self.map_groups.groups:
            self.map_var.set(requested)
        else:
            selected = self.map_groups.preferred(requested, self._allowed_maps())
            if selected:
                self.map_var.set(selected)
        self._update_from_selection()

    def _sync_asset_selectors(self, allowed_maps):
        # Tests, profile restoration and Registry may set exact executable IDs
        # directly. The family controls follow them without replacing variants.
        if set(self.robot_groups.profiles) != set(self.robot_profiles):
            self.robot_groups = AssetGroups('robots', self.robot_profiles)
            self.robot_taxonomy = RobotTaxonomy(self.robot_groups)
        if set(self.map_groups.profiles) != set(self.map_profiles):
            self.map_groups = AssetGroups('maps', self.map_profiles)
        for kind, groups, selected, family_var, combo, variant_combo, allowed in (
            ('robots', self.robot_groups, self.robot_var, self.robot_family_var, self.robot_combo,
             self.robot_variant_combo, self._allowed_robots()),
            ('maps', self.map_groups, self.map_var, self.map_family_var, self.map_combo,
             self.map_variant_combo, allowed_maps)):
            family = NONE_LABEL if is_none_selection(selected.get()) else groups.family(selected.get())
            family_var.set(family)
            if kind == 'robots':
                allowed = [name for name in allowed if name == NONE_LABEL or self.robot_taxonomy.matches(
                    name, self.robot_category_var.get(), self.robot_subtype_var.get())]
            choices = groups.choices(allowed)
            if NONE_LABEL in allowed:
                choices.insert(0, NONE_LABEL)
            combo.configure(values=choices, state='readonly' if choices else 'disabled')
            variants = groups.members(family, selectable=True, allowed=allowed)
            variant_combo.configure(values=variants, state='readonly' if len(variants) > 1 else 'disabled')

    def preview_asset(self, kind, asset_id, entity=None):
        """Open an owned static inspector, preserving the active launch."""
        profiles = self.robot_profiles if kind == 'robots' else self.map_profiles
        profile = profiles.get(asset_id)
        if profile is None and entity:
            path = (entity.get('assets', {}).get('urdf') if kind == 'robots' else entity.get('world_file'))
            if path:
                path = path.removeprefix('package://')
                if os.path.isabs(path):
                    package, relative = entity.get('ros_package', 'robot_lab_robots'), path
                else:
                    package, _, relative = path.partition('/')
                profile = (dict(package=package, xacro=relative, name=entity.get('name', asset_id)) if kind == 'robots'
                           else dict(gazebo=dict(world_package=package, world_path=relative)))
        if profile is None:
            messagebox.showinfo('Preview 3D', 'This registry entry has no installed 3D description.')
            return
        if not hasattr(self, 'registry_tab'):
            from .lab_tabs import RegistryTab
            self.registry_tab = RegistryTab(self.notebook, self)
        try:
            self.registry_tab.show_preview(kind, asset_id, profile)
        except (OSError, ValueError) as exc:
            messagebox.showerror('Preview 3D', str(exc))

    def _robot_config(self):
        return self.robot_profiles.get(self.robot_var.get(), {})

    def _mode_config(self):
        return self.mode_profiles.get(self.mode_var.get(), {})

    def _map_config(self):
        return self.map_profiles.get(self.map_var.get(), {})

    def _map_yaml_path(self, map_id=None):
        map_id = map_id or self.map_var.get()
        if is_none_selection(map_id):
            return ""
        map_config = self.map_profiles.get(map_id, {})
        map_file_config = map_config.get("map", {})
        relative_path = map_file_config.get("path", "")
        if not relative_path:
            relative_path = f"maps/{map_id}/maps/map.yaml"
        return package_path(map_file_config.get("package", "maps"), relative_path)

    def _map_has_2d_map(self, map_id=None):
        map_id = map_id or self.map_var.get()
        if is_none_selection(map_id):
            return False
        map_file_config = self.map_profiles.get(map_id, {}).get("map", {})
        configured = map_file_config.get("has_2d_map")
        if configured is not None:
            return bool_value(configured) and os.path.exists(self._map_yaml_path(map_id))
        return os.path.exists(self._map_yaml_path(map_id))

    def _mode_requires_2d_map(self, mode):
        return bool_value(self.mode_profiles.get(mode, {}).get("requires_2d_map", False))

    def _mode_required_features(self, mode):
        return self.mode_profiles.get(mode, {}).get("required_features", [])

    def _supported_modes(self):
        """Modes selectable for the current robot+map+simulator combination."""
        simulator = self.simulator_var.get()
        return [mode for mode in self._robot_map_modes()
                if not simulator
                or (_simulator_supports_mode(simulator, mode, self.mode_profiles)
                    and self._robot_can_run(mode, simulator))]

    def _robot_can_run(self, mode, simulator):
        """Whether the selected robot has what *mode* needs in *simulator*.

        A robot-free run has nothing to check; the sensor/motion model is
        robot_lab_utils.mode_capability, shared with the bringup launch.
        """
        if self._robot_free():
            return True
        return not missing_robot_features(
            self._robot_config(), mode, simulator, self.mode_profiles.get(mode))

    def _robot_free(self):
        return is_none_selection(self.robot_var.get())

    def _map_free(self):
        return is_none_selection(self.map_var.get())

    def _mode_reasons(self):
        """mode -> '' when selectable, else why it is not.

        The reason is what the GUI shows on a disabled control, so every
        greyed-out option can say what would make it selectable.
        """
        simulator = self.simulator_var.get()
        robot_config = self._robot_config()
        robot_supported = robot_config.get("supported_modes", ["display"])
        robot_free = self._robot_free()
        map_free = self._map_free()

        reasons = {}
        for mode in MODE_ORDER:
            if mode not in self.mode_profiles:
                reasons[mode] = "not defined in sim_modes.yaml"
                continue
            # Robot-free / map-free runs only make sense for visualization.
            if (robot_free or map_free) and mode != "display":
                missing = "robot" if robot_free else "map"
                reasons[mode] = ("needs a %s; only Display runs without one"
                                 % missing)
                continue
            if not robot_free and mode not in robot_supported:
                reasons[mode] = ("robot '%s' does not declare this mode"
                                 % self.robot_var.get())
                continue
            if not map_free and self._mode_requires_2d_map(mode) \
                    and not self._map_has_2d_map():
                reasons[mode] = ("map '%s' has no 2D occupancy map; build one "
                                 "with SLAM first" % self.map_var.get())
                continue
            if not robot_free:
                missing_features = missing_robot_features(
                    robot_config, mode, simulator or None,
                    self.mode_profiles.get(mode))
                if missing_features and not simulator:
                    # No simulator chosen yet: fine if any backend supplies it.
                    if self._robot_runs_somewhere(mode):
                        missing_features = []
                if missing_features:
                    reasons[mode] = describe_missing_features(missing_features)
                    continue
            if simulator:
                supported, why = self._simulator_mode_support(simulator, mode)
                if not supported:
                    reasons[mode] = why
                    continue
            reasons[mode] = ""
        return reasons

    def _simulator_mode_support(self, simulator, mode):
        """(ok, reason) for one simulator/mode pair, installation included.

        Goes through the module-level compatibility helpers so the headless
        layer stays the single source of truth for what a backend can run.
        """
        allowed = _allowed_simulators(mode, self.mode_profiles)
        ok, why = allowed.get(simulator, (False, "unknown simulator"))
        if ok:
            return True, ""
        return False, "%s: %s" % (
            SIMULATOR_LABELS.get(simulator, simulator), why)

    def _robot_map_modes(self):
        """Modes the robot+map support, independent of the simulator choice."""
        robot_config = self._robot_config()
        robot_supported = robot_config.get("supported_modes", ["display"])
        robot_free = self._robot_free()
        map_free = self._map_free()
        map_has_2d_map = False if map_free else self._map_has_2d_map()
        supported = []
        for mode in MODE_ORDER:
            if mode not in self.mode_profiles:
                continue
            if (robot_free or map_free) and mode != "display":
                continue
            if not robot_free and mode not in robot_supported:
                continue
            if not map_free and self._mode_requires_2d_map(mode) \
                    and not map_has_2d_map:
                continue
            if not robot_free and not self._robot_runs_somewhere(mode):
                continue
            supported.append(mode)
        return supported

    def _robot_runs_somewhere(self, mode):
        """Whether some simulator gives the robot everything *mode* needs."""
        return any(not missing_robot_features(
            self._robot_config(), mode, simulator, self.mode_profiles.get(mode))
            for simulator in SIMULATOR_ORDER)

    def _resolve_compatibility(self):
        """Fix-point cascade correcting mode <-> simulator compatibility.

        Returns (supported_modes, fixes) after correcting self.mode_var and
        self.simulator_var in place, so every downstream decision (command,
        validation, algorithm slots) sees a compatible selection.
        """
        fixes = []
        simulator = self.simulator_var.get() or "gazebo"
        modes = self._robot_map_modes()
        previous = getattr(self, '_automatically_changed_mode', None)
        if previous:
            robot, fallback, requested = previous
            if robot != self.robot_var.get() or self.mode_var.get() != fallback:
                self._automatically_changed_mode = None
            elif requested in modes and self._robot_can_run(requested, simulator) \
                    and _simulator_supports_mode(simulator, requested, self.mode_profiles):
                self.mode_var.set(requested)
                self._automatically_changed_mode = None
                fixes.append('restored selected mode '+requested)
        if self._robot_free() or self._map_free():
            # A deliberately robot-free / map-free selection is display-only
            # by construction; nothing to correct beyond pinning the mode.
            if self.mode_var.get() != "display":
                fixes.append("mode '%s' not available without a %s -> "
                             "'display'" % (self.mode_var.get(),
                                            "robot" if self._robot_free()
                                            else "map"))
                self.mode_var.set("display")
            return ["display"], fixes
        for _ in range(3):
            modes = [mode for mode in self._robot_map_modes()
                     if _simulator_supports_mode(simulator, mode,
                                                 self.mode_profiles)
                     and self._robot_can_run(mode, simulator)]
            if self.mode_var.get() not in modes:
                new_mode, note = _correction_for(
                    self.mode_var.get(), modes,
                    [self._robot_config().get('default_mode', 'slam'),
                     "slam", "display", "loc", "nav", "3d_slam"])
                if new_mode:
                    fixes.append("mode %s" % note)
                    self._automatically_changed_mode = (self.robot_var.get(), new_mode,
                                                        self.mode_var.get())
                    self.mode_var.set(new_mode)
            sims = [sim for sim in SIMULATOR_ORDER
                    if _allowed_simulators(self.mode_var.get(),
                                           self.mode_profiles)
                       .get(sim, (False, ""))[0]
                    and self._robot_can_run(self.mode_var.get(), sim)]
            if simulator not in sims:
                new_sim, note = _correction_for(simulator, sims, SIMULATOR_ORDER)
                if new_sim and new_sim != simulator:
                    fixes.append("simulator %s" % note)
                    simulator = new_sim
                    self.simulator_var.set(new_sim)
                    continue
            break
        return modes, fixes

    def _allowed_maps(self):
        """Map profiles selectable for the active mode.

        Display mode accepts every registered environment in every backend
        (including the map-free sentinel); the map-dependent modes accept
        only environments with a real 2D occupancy map on disk.
        """
        maps = [map_id for map_id in sorted(self.map_profiles)
                if self._map_ok_for_mode(map_id)]
        if self.mode_var.get() == "display" and not self._robot_free():
            return [NONE_LABEL] + maps
        return maps

    def _allowed_robots(self):
        """Robot profiles selectable for the active mode.

        The robot-free sentinel is offered only in display mode, and only
        when a map is selected - there has to be something left to show.
        """
        robots = sorted(self.robot_profiles)
        if self.mode_var.get() == "display" and not self._map_free():
            return [NONE_LABEL] + robots
        return robots

    def _selectable_simulators(self):
        """Backends that can run the active mode and are installed here."""
        mode = self.mode_var.get()
        return [sim for sim in SIMULATOR_ORDER
                if self._simulator_mode_support(sim, mode)[0]
                and self._robot_can_run(mode, sim)]

    def _map_ok_for_mode(self, map_id):
        if self._mode_requires_2d_map(self.mode_var.get()):
            return self._map_has_2d_map(map_id)
        return True

    def _update_simulator_panel(self):
        """Refresh the per-simulator availability rows in the left panel."""
        rows = getattr(self, "simulator_status_labels", None)
        if not rows:
            return
        statuses = _available_simulators()
        mode = self.mode_var.get()
        for sim_id, row_label in rows.items():
            name = SIMULATOR_LABELS.get(sim_id, sim_id)
            installed, reason = statuses.get(sim_id, (False, "unknown"))
            ok_mode, why_mode = _simulator_supports_mode(
                sim_id, mode, self.mode_profiles)
            if installed and ok_mode:
                text = "● %s — available" % name
                color = "#a6e3a1" if THEME_AVAILABLE else "#2e7d32"
            elif installed:
                text = "● %s — cannot run %s: %s" % (name, mode, why_mode)
                color = "#fab387" if THEME_AVAILABLE else "#e65100"
            else:
                text = "○ %s — not available: %s" % (name, reason)
                color = "#6c7086" if THEME_AVAILABLE else "#777777"
            try:
                row_label.configure(text=text, foreground=color)
            except tk.TclError:  # pragma: no cover — widget destroyed
                pass

    def _fallback_mode(self, supported_modes):
        if "slam" in supported_modes:
            return "slam"
        if "display" in supported_modes:
            return "display"
        return supported_modes[0] if supported_modes else "display"

    def _mode_simulators(self):
        """Simulators the active mode declares in sim_modes.yaml.

        This is the mode's own declaration only; host availability is
        applied separately by :meth:`_selectable_simulators`.
        """
        return self._mode_config().get("simulators", SIMULATOR_ORDER)

    def _simulator_selected(self, _event=None):
        """Refresh summary/command after a simulator change."""
        self._update_from_selection()

    def _on_algorithm_category_changed(self, _event=None):
        """When an algorithm slot changes, re-resolve and refresh."""
        self._update_from_selection()

    def _refresh_mode_steps(self):
        """Rebuild the composition slots when the mode changes.

        Uses the mode's ``steps`` from sim_modes.yaml. Steps with an
        ``algorithm_category`` get a selectable dropdown; steps without are
        fixed pipeline labels. Only rebuilds when the mode actually changes.
        """
        mode = self.mode_var.get()
        steps = _mode_steps(mode, self.mode_profiles)
        steps_key = tuple(step["id"] for step in steps)
        if steps_key == self._current_mode_steps:
            return  # No change, skip rebuild
        self._current_mode_steps = steps_key

        # Clear existing widgets and the step->category map together: a
        # stale entry left from the previous mode has no matching variable,
        # so it would emit a second, contradictory `<category>:=none`
        # argument alongside the real selection.
        for widget in self.composition_frame.winfo_children():
            widget.destroy()
        self.slot_combos.clear()
        self.slot_labels.clear()
        self.slot_vars.clear()
        self._mode_step_categories.clear()

        # Update the frame title to show the mode category
        category = _mode_category(mode, self.mode_profiles)
        self.composition_frame.configure(text=f"Composition — {category}")

        # Build new slots from mode steps
        defaults = _mode_default_algorithms(mode, self.mode_profiles)
        for row, step in enumerate(steps):
            step_id = step["id"]
            label_text = step["label"]
            category_name = step.get("algorithm_category")
            label = ttk.Label(
                self.composition_frame,
                text=label_text,
            )
            label.grid(row=row, column=0, sticky="w", padx=(0, 6))
            self.slot_labels[step_id] = label

            if category_name:
                # Selectable algorithm slot
                var = tk.StringVar()
                default_algo = defaults.get(category_name, "")
                if default_algo:
                    var.set(default_algo)
                combo = ttk.Combobox(
                    self.composition_frame,
                    textvariable=var,
                    state="readonly",
                    width=26,
                )
                combo.grid(row=row, column=1, sticky="ew")
                combo.bind("<<ComboboxSelected>>", self._on_selection_changed)
                self.slot_combos[step_id] = combo
                self.slot_vars[step_id] = var
                self._mode_step_categories[step_id] = category_name
            else:
                # Fixed pipeline step (no selection)
                self._mode_step_categories[step_id] = None

    def _refresh_slot_combos(self):
        """Populate the dynamic algorithm slot dropdowns from the registry.

        Filters each slot's dropdown to algorithms compatible with the
        current robot/map/simulator, applies defaults from sim_modes.yaml,
        and attaches tooltips describing each algorithm.
        """
        defaults_changed = (self.robot_var.get() != self._last_algorithm_robot
                            or self.mode_var.get() != self._last_algorithm_mode)
        for step_id, combo in self.slot_combos.items():
            category_name = self._mode_step_categories.get(step_id)
            if category_name is None:
                continue  # Fixed pipeline step
            algorithms = self._slot_compatible_algorithms(category_name)
            var = self.slot_vars.get(step_id)
            if combo is None:
                continue
            # Every stage can also be switched off explicitly, which the
            # launch layer receives as `<category>:=none`.
            combo.configure(values=[NONE_LABEL] + algorithms)
            if var is not None:
                current = var.get()
                robot_default = self._robot_default_algorithm(category_name)
                mode_default = _mode_default_algorithms(
                    self.mode_var.get(), self.mode_profiles).get(category_name)
                if defaults_changed:
                    # Apply robot-specific defaults on robot/mode changes.
                    # A deliberate slot change made afterward is preserved
                    # even when it matches the mode's original default.
                    preferred = next(
                        (choice for choice in (robot_default, mode_default)
                         if choice in algorithms), NONE_LABEL)
                    var.set(preferred)
                elif current and not is_none_selection(current) \
                        and current not in algorithms:
                    if robot_default in algorithms:
                        # The robot names what replaces a mode default it
                        # cannot run (a car's path follower, say).
                        var.set(robot_default)
                    else:
                        self._cleared_selections.append(
                            (category_name, self._algorithm_name(current)))
                        var.set(NONE_LABEL)
            if algorithms:
                combo.configure(state="readonly")
                tip = ("%d algorithm(s) compatible with this robot/simulator."
                       % len(algorithms))
                if len(algorithms) == 1:
                    tip = self._algorithm_description(algorithms[0])
            else:
                # Nothing in this category fits the current selection; the
                # slot is locked with the reason rather than offering
                # choices that would fail at launch.
                combo.configure(state="disabled")
                tip = ("No %s algorithm is compatible with %s in %s."
                       % (ALGORITHM_SLOT_LABELS.get(category_name,
                                                    category_name).lower(),
                          self.robot_var.get() or "this robot",
                          SIMULATOR_LABELS.get(self.simulator_var.get(),
                                               self.simulator_var.get())))
            add_tooltip(combo, tip)
        self._last_algorithm_robot = self.robot_var.get()
        self._last_algorithm_mode = self.mode_var.get()

    def _slot_compatible_algorithms(self, slot):
        """Return algorithm IDs for *slot* compatible with the current robot.

        Uses the shared validator (validation_lines) to check each algorithm
        in the slot against the current robot/map/simulator selection. Falls
        back to the full category list if the registry is unavailable.
        """
        cache_key = (self.robot_var.get(), self.map_var.get(),
                     self.simulator_var.get(), slot)
        cached = self._compat_cache.get(cache_key)
        if cached is not None:
            return cached
        compatible = []
        if not COMPOSITION_AVAILABLE or self.composition_registry is None \
                or self._robot_free() or self._map_free():
            # Without a registry (or without both a robot and an
            # environment to validate against) the full category is offered
            # rather than an empty list that would look like a broken slot.
            compatible = self._algorithms_for_category(slot)
            self._compat_cache[cache_key] = compatible
            return compatible
        for algo_id in self._algorithms_for_category(slot):
            test_selection = GuiCompositionSelection(
                robot_id=self.robot_var.get() or None,
                simulator=self.simulator_var.get() or None,
                environment_id=self._environment_id(),
                algorithm_ids={slot: algo_id},
                reset=True,
            )
            try:
                errors, _ = validation_lines(
                    self.composition_registry, test_selection)
                if not errors:
                    compatible.append(algo_id)
            except Exception:
                compatible.append(algo_id)
        self._compat_cache[cache_key] = compatible
        return compatible

    def _slot_is_active(self, slot):
        """Whether *slot* should be editable in the current mode.

        With dynamic mode steps, every step that has an algorithm_category
        is selectable. This method is kept for backward compatibility but
        the dynamic slot building in _refresh_mode_steps handles activity.
        """
        return slot in self.slot_combos

    def _primary_algorithm_id(self):
        """Return the first non-empty algorithm ID from the dynamic slots."""
        for step_id, category_name in self._mode_step_categories.items():
            if category_name is None:
                continue
            var = self.slot_vars.get(step_id)
            if var and var.get():
                return var.get()
        return ""

    def _primary_algorithm_name(self):
        """Return the display name for the first selected algorithm, or 'auto'."""
        algo_id = self._primary_algorithm_id()
        if not algo_id:
            return "auto"
        return self._algorithm_name(algo_id)

    def _algorithm_description(self, algorithm_id):
        """Return the description for an algorithm ID (or its name)."""
        for a in self.algorithms:
            if a.get("id") == algorithm_id:
                return a.get("description", a.get("name", algorithm_id))
        return algorithm_id

    def _get_compatibility_summary(self):
        """Build a human-readable compatibility status string."""
        if not COMPOSITION_AVAILABLE or self.composition_registry is None:
            return "Composition resolver unavailable."
        robot_id = self.robot_var.get()
        if not robot_id:
            return "No robot selected."
        total = 0
        compatible = 0
        for slot in self._mode_step_categories:
            category_name = self._mode_step_categories[slot]
            if category_name is None:
                continue
            for algo_id in self._algorithms_for_category(category_name):
                total += 1
                test_selection = GuiCompositionSelection(
                    robot_id=robot_id or None,
                    simulator=self.simulator_var.get() or None,
                    environment_id=self._environment_id(),
                    algorithm_ids={category_name: algo_id},
                    reset=True,
                )
                try:
                    errors, _ = validation_lines(
                        self.composition_registry, test_selection)
                    if not errors:
                        compatible += 1
                except Exception:
                    compatible += 1
        if compatible == total:
            return f"All {total} algorithms compatible with {robot_id}."
        return (f"{compatible}/{total} algorithms compatible "
                f"with {robot_id}.")

    def _reset_composition(self):
        """Clear all algorithm slot selections back to empty."""
        for var in self.slot_vars.values():
            var.set("")
        self._cleared_selections.clear()
        self._compat_cache.clear()
        self._update_from_selection()
        self.status_var.set("Composition reset.")
        self.after(3000, lambda: self.status_var.set("Idle"))

    def _show_incompatible(self):
        """Show algorithms incompatible with the current robot."""
        if not COMPOSITION_AVAILABLE or self.composition_registry is None:
            messagebox.showinfo(
                "Incompatible Algorithms",
                "Composition resolver unavailable.")
            return
        robot_id = self.robot_var.get()
        if not robot_id:
            messagebox.showinfo(
                "Incompatible Algorithms",
                "No robot selected.")
            return
        lines = []
        for step_id in self._mode_step_categories:
            category_name = self._mode_step_categories[step_id]
            if category_name is None:
                continue
            bad = []
            for algo_id in self._algorithms_for_category(category_name):
                test_selection = GuiCompositionSelection(
                    robot_id=robot_id or None,
                    simulator=self.simulator_var.get() or None,
                    environment_id=self._environment_id(),
                    algorithm_ids={category_name: algo_id},
                    reset=True,
                )
                try:
                    errors, _ = validation_lines(
                        self.composition_registry, test_selection)
                    if errors:
                        bad.append(self._algorithm_name(algo_id))
                except Exception:
                    pass
            if bad:
                label = self.slot_labels.get(step_id)
                label_text = label.cget("text") if label else category_name
                lines.append(f"{label_text}: {', '.join(bad)}")
        if not lines:
            messagebox.showinfo(
                "Incompatible Algorithms",
                f"All algorithms are compatible with {robot_id}.")
            return
        dialog = tk.Toplevel(self)
        dialog.title("Incompatible Algorithms")
        dialog.transient(self)
        dialog.resizable(True, True)
        ttk.Label(
            dialog,
            text=f"Incompatible with '{robot_id}':",
            font=("Segoe UI", 10, "bold") if THEME_AVAILABLE else None,
        ).pack(padx=12, pady=(12, 4), anchor="w")
        text = tk.Text(dialog, width=48,
                       height=min(12, max(4, len(lines) + 2)),
                       wrap="word")
        text.pack(padx=12, pady=(0, 8), fill="both", expand=True)
        text.insert("1.0", "\n".join(lines))
        text.configure(state="disabled")
        ttk.Button(dialog, text="Close",
                   command=dialog.destroy).pack(pady=(0, 12))

    def _quick_select(self):
        """Pick the first compatible algorithm for every slot."""
        if not COMPOSITION_AVAILABLE or self.composition_registry is None:
            self.status_var.set("Composition resolver unavailable.")
            self.after(3000, lambda: self.status_var.set("Idle"))
            return
        for step_id, category_name in self._mode_step_categories.items():
            if category_name is None:
                continue
            compatible = self._slot_compatible_algorithms(category_name)
            if compatible:
                self.slot_vars[step_id].set(compatible[0])
        self._compat_cache.clear()
        self._update_from_selection()
        self.status_var.set("Quick select applied.")
        self.after(3000, lambda: self.status_var.set("Idle"))

    def _show_cleared(self):
        """Show algorithms cleared by the last compatibility filter."""
        if not self._cleared_selections:
            messagebox.showinfo(
                "Cleared Selections",
                "No selections were cleared by the last filter.")
            return
        lines = [
            f"{ALGORITHM_SLOT_LABELS.get(slot, slot.title())}: {name}"
            for slot, name in self._cleared_selections
        ]
        dialog = tk.Toplevel(self)
        dialog.title("Cleared Selections")
        dialog.transient(self)
        dialog.resizable(True, True)
        ttk.Label(
            dialog,
            text="Cleared by last filter:",
            font=("Segoe UI", 10, "bold") if THEME_AVAILABLE else None,
        ).pack(padx=12, pady=(12, 4), anchor="w")
        text = tk.Text(dialog, width=48,
                       height=min(12, max(4, len(lines) + 2)),
                       wrap="word")
        text.pack(padx=12, pady=(0, 8), fill="both", expand=True)
        text.insert("1.0", "\n".join(lines))
        text.configure(state="disabled")
        ttk.Button(dialog, text="Close",
                   command=dialog.destroy).pack(pady=(0, 12))

    def _environment_id(self):
        """Translate a map profile key to its registry environment ID."""
        map_name = self.map_var.get()
        if COMPOSITION_AVAILABLE and self.composition_registry is not None:
            return environment_id_for_map_name(
                map_name, self.composition_registry) or map_name or None
        return map_name or None

    def _composition_selection(self):
        """Build the shared resolver selection from the current GUI controls."""
        if not COMPOSITION_AVAILABLE:
            raise RuntimeError("gui_composition unavailable")
        # Build algorithm_ids from the dynamic mode steps, keyed by
        # algorithm_category so the resolver can apply them.
        algorithm_ids = {}
        for step_id, category_name in self._mode_step_categories.items():
            if category_name is None:
                continue
            var = self.slot_vars.get(step_id)
            if var and var.get() and not is_none_selection(var.get()):
                algorithm_ids[category_name] = var.get()
        return GuiCompositionSelection(
            robot_id=self.robot_var.get() or None,
            simulator=self.simulator_var.get() or None,
            environment_id=self._environment_id(),
            algorithm_ids=algorithm_ids,
            reset=True,
            mode=self.mode_var.get() or None,
            gui=self.gui_var.get() or None,
            steering_mode=(self.steering_mode_var.get() or None
                           if self._four_wheel_steer_selectable() else None),
        )

    def _update_validation_and_command(self):
        """Run the shared validator and refresh command + validation text."""
        self._update_reset_button()
        if not COMPOSITION_AVAILABLE or self.composition_registry is None:
            self.validation_var.set(
                "Composition resolver unavailable; direct launch used.")
            self._set_command(self._legacy_command())
            return
        if self._robot_free() or self._map_free():
            # The registry models an experiment as robot + environment, so a
            # deliberately robot-free or map-free display run is built
            # directly instead of being reported as an invalid composition.
            what = "map only" if self._robot_free() else "robot only"
            self.validation_var.set("Valid - display %s" % what)
            self._set_command(self._legacy_command())
            return
        try:
            selection = self._composition_selection()
            ok, manifest = resolve_selection(
                self.composition_registry, selection)
            if not ok:
                self.validation_var.set(
                    "Invalid: " + " | ".join(manifest.get("errors", [])))
                self._set_command([])
                return
            command = list(manifest["ros2_command"])
            if self.launch_kind_var.get() == "vacuum":
                command[3] = "simulated_room_vacuum.launch.py"
            # The manifest carries the algorithm choices, but only the two
            # nav2 planner slots become launch arguments there.  Appending
            # every selected category keeps the launch faithful to the panel
            # for perception, localization, estimation, fusion and control.
            for argument in self._algorithm_arguments():
                category = argument.split(":=", 1)[0]
                command = [part for part in command
                           if not part.startswith(category + ":=")]
                command.append(argument)
        except Exception as exc:  # pragma: no cover — defensive
            self.validation_var.set(f"Resolver error: {exc}")
            self._set_command([])
            return

        notes = []
        aliases = manifest.get("aliases_applied", [])
        if aliases:
            notes.append("Migrated: " + "; ".join(aliases))
        warnings = manifest.get("warnings", [])
        if warnings:
            notes.append("Warnings: " + "; ".join(warnings[:3]))
        self.validation_var.set("Valid" + (f" - {'; '.join(notes)}"
                                           if notes else ""))
        self._set_command(command)

    def _go2_policy_selectable(self):
        return (self.robot_var.get() == "unitree_go2"
                and self.simulator_var.get() == "mujoco"
                and self.mode_var.get() == "loc"
                and self.launch_kind_var.get() == "simulation")

    def _bhl_policy_selectable(self):
        return (self.robot_var.get() == "berkeley_humanoid_lite_sim"
                and self.simulator_var.get() == "mujoco"
                and self.mode_var.get() == "loc"
                and self.launch_kind_var.get() == "simulation")

    def _drive_type(self):
        profile = self.robot_profiles.get(self.robot_var.get(), {})
        return str(profile.get("drive", {}).get("type", "diff"))

    def _extension_drive_unavailable(self):
        profile = self._robot_config()
        return bool(profile.get('source_id') and not profile.get('drive'))

    def _four_wheel_steer_selectable(self):
        """The steering-pattern override is only meaningful for this drive."""
        return (self._drive_type() in ("four_wheel_steer", "4ws",
                                       "four_wheel_steering", "swerve")
                and self.launch_kind_var.get() == "simulation")

    def _mecanum_selectable(self):
        return (self._drive_type() in ("mecanum", "roller", "omni")
                and self.launch_kind_var.get() == "simulation")

    def _lateral_drive_selectable(self):
        return (self._flight_selectable() or self._mecanum_selectable() or
                (self._four_wheel_steer_selectable() and
                 self.steering_mode_var.get() in ("crab", "in_phase")))

    def _flight_selectable(self):
        return (self.robot_var.get() == 'px4_x500' and self.mode_var.get() == 'flight'
                and self.simulator_var.get() == 'gazebo'
                and self.launch_kind_var.get() == 'simulation')

    def _flight_action(self, action):
        if not self._flight_selectable():
            return
        self._stop_drive(keep_input_enabled=True)
        command = ['ros2', 'service', 'call', '/px4/'+action, 'std_srvs/srv/Trigger', '{}']
        threading.Thread(target=self._run_aux_command, args=(command, 'PX4 '+action), daemon=True).start()

    def _update_reset_button(self):
        profile = self.robot_profiles.get(self.robot_var.get(), {})
        native_arm = (profile.get('arm_control') == 'panda' and self.simulator_var.get() == 'mujoco'
                      and self.mode_var.get() == 'display')
        enabled = (self._launch_running and self.launch_kind_var.get() == 'simulation'
                   and self.mode_var.get() in ('display', 'loc', 'slam', '3d_slam')
                   and bool(self.robot_var.get()) and self.robot_var.get() != 'none'
                   and (native_arm or (bool(profile.get('drive'))
                   and self._drive_type() in ('diff', 'ackermann', 'four_wheel_steer', 'mecanum'))))
        self.reset_robot_button.state(['!disabled'] if enabled else ['disabled'])

    def _reset_robot(self):
        self._update_reset_button()
        if self.reset_robot_button.instate(['disabled']):
            return
        self._stop_drive()
        command = ['timeout', '30', 'ros2', 'service', 'call', '/robot_lab/reset',
                   'std_srvs/srv/Trigger', '{}']
        threading.Thread(target=self._run_aux_command, args=(command, 'Robot reset'), daemon=True).start()

    def _set_command(self, command):
        """Keep the preview, clipboard text and executable arguments in sync."""
        if command:
            from robot_lab_utils.robot_spawn import map_spawn_override
            for axis, value in map_spawn_override(self._robot_config(), self.map_var.get()).items():
                argument = 'spawn_'+axis+':='
                if not any(part.startswith(argument) for part in command):
                    command.append(argument+str(value))
        if command and self.go2_policy_var.get() and self._go2_policy_selectable():
            command = [part for part in command
                       if not part.startswith("go2_policy_path:=")]
            command.append("go2_policy_path:=auto")
        if command and self._bhl_policy_selectable():
            # The BHL policy defaults on, so the toggle is always written
            # explicitly: the preview says which policy path will run.
            command = [part for part in command
                       if not part.startswith("bhl_enable_policy:=")]
            command.append(
                "bhl_enable_policy:=%s"
                % ("true" if self.bhl_policy_var.get() else "false"))
        if command and self._four_wheel_steer_selectable() \
                and self.steering_mode_var.get():
            command = [part for part in command
                       if not part.startswith("steering_mode:=")]
            command.append("steering_mode:=%s" % self.steering_mode_var.get())
        if command and self._robot_config().get('arm_control') == 'panda' \
                and self.simulator_var.get() == 'mujoco' and self.mode_var.get() == 'display':
            command = [part for part in command if not part.startswith('arm_control:=')]
            command.append('arm_control:=panda')
            fixture = hasattr(self, 'hand_tab') and self.hand_tab.selected() and self.hand_tab.fixture_var.get()
            if fixture:
                command = [part for part in command if not part.startswith('grasp_fixture:=')]
                command.append('grasp_fixture:=true')
            if self._robot_config().get('arm_planning') == 'moveit':
                command = [part for part in command if not part.startswith('arm_planning:=')]
                command.append('arm_planning:='+('none' if fixture else 'moveit'))
        self._prepared_command = list(command)
        self.command_var.set(shlex.join(command))
        self.command_preview.configure(state="normal")
        self.command_preview.delete("1.0", "end")
        self.command_preview.insert("1.0", self.command_var.get())
        self.command_preview.configure(state="disabled")
        self.copy_command_button.state(["!disabled"] if command else ["disabled"])
        self.start_button.state(
            ["!disabled"] if command and not self._launch_running else ["disabled"])

    def _copy_command(self):
        self._update_validation_and_command()
        if self.command_var.get():
            self.clipboard_clear()
            self.clipboard_append(self.command_var.get())
            self.status_var.set("Command copied")

    def _update_from_selection(self):
        # 1. Maps: every map stays selectable in display mode (a map can be
        # visualized in any backend, with or without a robot); the
        # map-dependent modes keep only environments that have a real 2D
        # occupancy map on disk.
        allowed_maps = self._allowed_maps()
        if self.map_var.get() not in allowed_maps and allowed_maps:
            self.map_var.set(allowed_maps[0])

        # 2. Correct only what cannot be represented at all, then gate the
        # rest: an option the current selection cannot run is disabled with
        # the reason attached, instead of being silently switched.
        supported_modes, fixes = self._resolve_compatibility()
        if fixes and fixes != self._last_fixes:
            self._last_fixes = fixes
            self.status_var.set("Auto-corrected: " + "; ".join(fixes))
            self.after(6000, lambda: self.status_var.set("Idle"))

        mode_reasons = self._mode_reasons()
        for mode, button in self.mode_buttons.items():
            reason = mode_reasons.get(mode, "")
            if reason:
                button.state(["disabled"])
                add_tooltip(button, "Unavailable - %s" % reason)
            else:
                button.state(["!disabled"])
                add_tooltip(button, MODE_TOOLTIPS.get(mode, ""))

        # Display mode is exactly where the map selector matters most: a map
        # can be shown on its own in any simulator.  It is only locked when
        # no environment is valid for the mode at all.
        self._sync_asset_selectors(allowed_maps)

        # Simulators: every backend stays listed, and the panel underneath
        # says which are selectable and why the others are not, so the choice
        # is visible rather than silently narrowed.
        allowed_sims = self._selectable_simulators()
        self.simulator_combo.configure(
            values=allowed_sims or SIMULATOR_ORDER,
            state="readonly" if len(allowed_sims) > 1 else "disabled",
        )
        if allowed_sims and self.simulator_var.get() not in allowed_sims:
            self.simulator_var.set(allowed_sims[0])
        self._update_simulator_panel()

        supports_vacuum = bool_value(self._robot_config().get("supports_room_vacuum", False))
        if not supports_vacuum and self.launch_kind_var.get() == "vacuum":
            self.launch_kind_var.set("simulation")
        self.vacuum_radio.state(["!disabled"] if supports_vacuum else ["disabled"])
        self.save_map_button.state(["!disabled"] if self.mode_var.get() in ("slam", "3d_slam") else ["disabled"])
        self.go2_policy_checkbox.state(
            ["!disabled"] if self._go2_policy_selectable() else ["disabled"])
        self.bhl_policy_checkbox.state(
            ["!disabled"] if self._bhl_policy_selectable() else ["disabled"])
        self.steering_mode_combo.configure(
            state="readonly" if self._four_wheel_steer_selectable() else "disabled")
        for button in self._strafe_widgets():
            button.state(
                ["!disabled"] if self._lateral_drive_selectable() else ["disabled"])
        for button in self.flight_buttons:
            button.state(['!disabled'] if self._flight_selectable() else ['disabled'])
        drive_unavailable = self._extension_drive_unavailable()
        for button in self._drive_widgets():
            button.state(['disabled'] if drive_unavailable else ['!disabled'])
        for button in self.drone_drive_widgets.values():
            button.state(['!disabled'] if self._flight_selectable() else ['disabled'])
        self.drive_input_checkbox.state(['disabled'] if drive_unavailable else ['!disabled'])
        if drive_unavailable:
            if self.drive_input_enabled.get() or self.drive_repeat_job is not None:
                self._stop_drive()
            self.drive_status_var.set('Base Drive controller pending for this imported model')
        if not self._lateral_drive_selectable():
            self.drive_strafe_buttons.clear()

        # Clear cached compatibility results (robot/mode/map changed)
        self._compat_cache.clear()

        # Rebuild the composition slots when the mode changed, then populate
        # each selectable slot with compatible algorithms from the registry.
        self._refresh_mode_steps()
        self._refresh_slot_combos()

        # Update compatibility status label + color
        summary = self._get_compatibility_summary()
        self.compatibility_var.set(summary)
        if summary.startswith("All"):
            color = "#a6e3a1" if THEME_AVAILABLE else "#2e7d32"
        elif "unavailable" in summary or "not found" in summary:
            color = "#a6adc8" if THEME_AVAILABLE else "#333333"
        else:
            color = "#fab387" if THEME_AVAILABLE else "#e65100"
        self.compatibility_label.configure(foreground=color)

        self._update_validation_and_command()
        self.summary_var.set(self._summary_text(supported_modes, supports_vacuum))
        self.robot_info_var.set(self._robot_info_text(supported_modes, supports_vacuum))
        self._update_drive_limits_label()
        self._refresh_workspace_selection()
        if hasattr(self, 'arm_tab'):
            self.arm_tab.refresh_selection()
        if hasattr(self, 'hand_tab'):
            self.hand_tab.refresh_selection()

    def _robot_info_text(self, supported_modes, supports_vacuum):
        config = self._robot_config()
        features = config.get("features", [])
        lines = [
            f"Class profile: {', '.join(features) if features else 'display only'}",
            f"Modes: {', '.join(MODE_LABELS.get(m, m) for m in supported_modes) or 'none'}",
            f"Cleaning missions: {'yes' if supports_vacuum else 'no'}",
        ]
        if (self.robot_var.get() == "berkeley_humanoid_lite_sim"
                and self.simulator_var.get() == "mujoco"):
            lines.append(
                "Walk: ONNX policy in Localization (experimental); "
                "forward/stop measured, held turning stalls")
        if self.robot_var.get() == 'px4_x500':
            lines.append('PX4 Flight: Takeoff first; WASD/Drive controls XY and yaw, Strafe controls lateral flight. '
                         'Altitude Up/Down climb or descend; click again to slow to a hover. '
                         'Hold or Space stops manual travel; Land returns to the ground. '
                         '3D goals use /px4/goal; no obstacle avoidance. Health contains the flight guide and measured results.')
        if config.get('arm_control') == 'panda':
            lines.append('Arm tab: native Panda joint jogging, Home, Stop and bounded position trajectories on MuJoCo. '
                         'See Hand for qualified gripper controls; other arm backends remain pending.')
            lines.append('Cartesian: MoveIt Plan/Execute with the selected static collision scene. '
                         'Nav_empty targets are measured; other maps remain experiments and the grasp fixture uses joint controls.'
                         if config.get('arm_planning') == 'moveit' else 'Cartesian planning remains pending qualification.')
        if config.get('hand_control') == 'panda':
            lines.append('Hand tab: native coupled-finger Open/Close/Stop and bounded force. '
                         'The selected grasp fixture has a measured cube lift/release/reset on MuJoCo/nav_empty. '
                         'Dexterous hands and other gripper backends remain pending.')
        if config.get('drive_in_display'):
            lines.append('Drive/WASD: physical wheel control in Display with bounded speed and timeout. '
                         'Vendor docking/hazards remain pending.')
            screens = [screen for screen in config.get('runtime_screens', [])
                       if screen['backend'] == self.simulator_var.get()]
            if screens:
                lines.append('Recorded in this backend: '+ '; '.join(
                    MODE_LABELS.get(screen['mode'], screen['mode'])+' on '+', '.join(screen['maps'])
                    for screen in screens)+'. Other maps are experiments.')
            else:
                lines.append('Original sensor configuration is installed; mapping/navigation screens '
                             'for this backend remain pending.')
        return "\n".join(lines)

    def _resolve_rviz_path(self):
        rviz_config = self._mode_config().get("rviz", {})
        if not bool_value(rviz_config.get("enabled", True)):
            return "disabled"
        package_name = rviz_config.get("package", "")
        relative_path = rviz_config.get("path", "")
        if not package_name or not relative_path:
            return "not configured"
        return package_path(package_name, relative_path)

    def _resolve_world_path(self):
        if self._map_free():
            return "none"
        if self.mode_var.get() == "display":
            return "disabled"
        gazebo_config = self._map_config().get("gazebo", {})
        relative_path = gazebo_config.get("world_path", "")
        package_name = gazebo_config.get("world_package", "maps")
        if not relative_path:
            world_name = gazebo_config.get("world_name", self.map_var.get())
            relative_path = f"maps/{self.map_var.get()}/worlds/{world_name}.world"
        return package_path(package_name, relative_path)

    def _resolve_robot_path(self):
        if self._robot_free():
            return "none"
        robot_config = self._robot_config()
        return package_path(robot_config.get("package", "robots"), robot_config.get("xacro", ""))

    def _rtabmap_database_path(self):
        base_dir = os.environ.get(
            "ROBOT_LAB_RTABMAP_DIR",
            os.environ.get("BUMPERBOT_RTABMAP_DIR", str(Path.cwd() / "log" / "rtabmap")),
        )
        safe_map = "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in self.map_var.get())
        safe_robot = "".join(char if char.isalnum() or char in ("-", "_") else "_" for char in self.robot_var.get())
        return str(Path(base_dir) / f"{safe_map}_{safe_robot}.db")

    def _summary_text(self, supported_modes, supports_vacuum):
        unsupported = [MODE_LABELS.get(mode, mode) for mode in MODE_ORDER if mode not in supported_modes]
        robot_note = "full simulation stack" if len(supported_modes) > 1 else "description/display only"
        simulator = self.simulator_var.get()
        gui_label = {"auto": "Auto (GUI if DISPLAY)", "true": "GUI", "false": "Headless"}
        robot_free, map_free = self._robot_free(), self._map_free()
        lines = [
            "Robot: %s" % ("none (world only)" if robot_free
                           else "%s (%s)" % (self.robot_var.get(), robot_note)),
            f"Robot file: {'-' if robot_free else self._resolve_robot_path()}",
            f"Mode: {MODE_LABELS.get(self.mode_var.get(), self.mode_var.get())}",
        ]
        # Every category the mode runs is listed, because every one of them is
        # passed to the launch: the panel and the command cannot disagree.
        for step_id, category in self._mode_step_categories.items():
            if category is None:
                continue
            variable = self.slot_vars.get(step_id)
            value = variable.get() if variable else ""
            label = ALGORITHM_SLOT_LABELS.get(category, category.title())
            if is_none_selection(value):
                lines.append(f"  {label}: off")
            else:
                lines.append(f"  {label}: {self._algorithm_name(value)}")
        lines.extend([
            f"Simulator: {SIMULATOR_LABELS.get(simulator, simulator)}",
            f"GUI: {gui_label.get(self.gui_var.get(), self.gui_var.get())}",
            f"RViz: {self._resolve_rviz_path()}",
            "World: %s" % ("none (robot only)" if map_free
                           else self._resolve_world_path()),
            f"2D map: {self._map_yaml_path() if self._map_has_2d_map() else 'not available'}",
            f"Vacuum: {'available' if supports_vacuum else 'not available'}",
        ])
        if self.mode_var.get() == "3d_slam":
            rtabmap_config = self._mode_config().get("rtabmap", {})
            lines.extend([
                f"RGB-D: {rtabmap_config.get('rgb_topic', '/oakd/rgb/image_raw')} + {rtabmap_config.get('depth_topic', '/oakd/depth/image_raw')}",
                f"RTAB-Map DB: {self._rtabmap_database_path()}",
            ])
        if unsupported:
            lines.append(f"Unavailable modes: {', '.join(unsupported)}")
        return "\n".join(lines)

    def _command(self):
        """Return the executable arguments shown in the command preview."""
        return list(self._prepared_command)

    def _algorithm_arguments(self):
        """`<category>:=<algorithm>` for every selectable slot of the mode.

        Every category the mode runs is passed explicitly - including the
        ones left empty, which become `none` - so the launch runs exactly the
        composition shown in the panel instead of falling back to the mode's
        defaults for anything the user changed.
        """
        arguments = {}
        for step_id, category in self._mode_step_categories.items():
            if category is None:
                continue
            variable = self.slot_vars.get(step_id)
            if variable is None:
                continue
            value = variable.get().strip()
            # A mode may run one category in more than one step (SLAM uses
            # localization for its backend); the last real selection wins,
            # and an empty slot never overwrites a chosen one.
            if category in arguments and is_none_selection(value):
                continue
            arguments[category] = selection_value(value)
        return ["%s:=%s" % (category, value)
                for category, value in arguments.items()]

    def _legacy_command(self):
        """Direct launch command built from the panel selections.

        Used when the registry resolver is unavailable, and for the
        robot-free / map-free display runs the registry cannot express (it
        requires both a robot and an environment entity).
        """
        launch_file = (
            "simulated_room_vacuum.launch.py"
            if self.launch_kind_var.get() == "vacuum"
            else "simulated_robot.launch.py"
        )
        command = [
            "ros2",
            "launch",
            "robot_lab_bringup",
            launch_file,
            f"mode:={self.mode_var.get()}",
            f"map_name:={selection_value(self.map_var.get())}",
            f"robot_model:={selection_value(self.robot_var.get())}",
            f"simulator:={self.simulator_var.get()}",
            f"gui:={self.gui_var.get()}",
        ]
        command.extend(self._algorithm_arguments())
        return command

    def _save_profile(self):
        """Save current configuration as a named profile (resolved manifest)."""
        name = tk.simpledialog.askstring("Save Profile", "Profile name:")
        if not name:
            return
        if COMPOSITION_AVAILABLE and self.composition_registry is not None:
            try:
                selection = self._composition_selection()
                ok, manifest = resolve_selection(
                    self.composition_registry, selection)
                if ok:
                    save_manifest(name, manifest)
                    self.status_var.set(f"Profile '{name}' saved (manifest)")
                    self.after(3000, lambda: self.status_var.set("Idle"))
                    return
            except Exception as exc:  # pragma: no cover — defensive
                self.status_var.set(f"Save failed: {exc}")
                return
        config = {
            "mode": self.mode_var.get(),
            "simulator": self.simulator_var.get(),
            "robot": self.robot_var.get(),
            "map_name": self.map_var.get(),
            "gui": self.gui_var.get(),
            "algorithm": self._primary_algorithm_id(),
        }
        save_profile(name, config)
        self.status_var.set(f"Profile '{name}' saved")
        self.after(3000, lambda: self.status_var.set("Idle"))

    def _apply_manifest_to_controls(self, manifest):
        """Apply a resolved manifest back onto the composition controls."""
        self.robot_var.set(manifest.get("robot_id") or self.robot_var.get())
        self.simulator_var.set(manifest.get("simulator") or self.simulator_var.get())
        env = manifest.get("environment_id")
        if env:
            self.map_var.set(env)
        # Stash manifest algorithm_ids — they are keyed by
        # algorithm_category and will be re-applied to the rebuilt per-mode
        # slot widgets (see _show_load_profile / _pending_manifest_algos).
        self._pending_manifest_algos = manifest.get("algorithm_ids") or {}
        # Restore the applied mode and simulator-GUI choice (R3.4+).
        resolved_from = manifest.get("resolved_from") or {}
        mode = resolved_from.get("mode") or manifest.get("mode")
        if mode and mode in self.mode_profiles:
            self.mode_var.set(mode)
        self.steering_mode_var.set(resolved_from.get("steering_mode") or "")
        gui_value = resolved_from.get("gui") or manifest.get("gui")
        if gui_value in ("auto", "true", "false"):
            self.gui_var.set(gui_value)

    def _show_load_profile(self):
        """Show dialog to load a saved profile (manifest or migrated legacy)."""
        profiles = list_profiles()
        if not profiles:
            messagebox.showinfo("Load Profile", "No profiles saved yet.")
            return
        name = tk.simpledialog.askstring(
            "Load Profile", "Select profile to load:",
            initialvalue=profiles[0] if profiles else "")
        if not name or name not in profiles:
            return
        cfg = load_profile(name)
        if cfg is None:
            return

        if is_manifest(cfg):
            self._apply_manifest_to_controls(cfg)
        else:
            # Legacy JSON config dict: migrate through the shared selector
            # logic (best-effort map_name -> registry environment).
            selection = GuiCompositionSelection(
                robot_id=cfg.get("robot"),
                simulator=cfg.get("simulator"),
                environment_id=(cfg.get("map_name") or None),
                algorithm_ids={},
            )
            if COMPOSITION_AVAILABLE and self.composition_registry is not None:
                migrated = migrate_legacy_selection(
                    cfg, self.composition_registry, MODE_TO_ALGORITHM_CATEGORY)
                selection = migrated
                if migrated.environment_id:
                    self.map_var.set(migrated.environment_id)
                for slot, value in migrated.algorithm_ids.items():
                    if slot in self.slot_vars:
                        self.slot_vars[slot].set(value)
            self.robot_var.set(selection.robot_id or self.robot_var.get())
            self.simulator_var.set(selection.simulator or self.simulator_var.get())

        self._update_from_selection()

        # Apply stashed manifest algorithm_ids (or migrated legacy value)
        # to the rebuilt per-mode slot widgets AFTER _update_from_selection
        # has rebuilt them, so values survive the slot rebuild.
        if self._pending_manifest_algos:
            pending = self._pending_manifest_algos
            self._pending_manifest_algos = {}
        elif COMPOSITION_AVAILABLE and self.composition_registry is not None:
            pending = migrated.algorithm_ids or {}
        else:
            legacy_algo = cfg.get("algorithm", "") or ""
            pending = {}
            if legacy_algo:
                cat = MODE_TO_ALGORITHM_CATEGORY.get(cfg.get("mode", ""), "")
                if cat:
                    pending = {cat: legacy_algo}
        for step_id, category_name in self._mode_step_categories.items():
            if category_name is None:
                continue
            var = self.slot_vars.get(step_id)
            if var is not None:
                var.set(pending.get(category_name, ""))
        self._compat_cache.clear()
        self._refresh_slot_combos()
        self._update_validation_and_command()
        self.status_var.set(f"Profile '{name}' loaded")
        self.after(3000, lambda: self.status_var.set("Idle"))

    def _delete_profile(self):
        """Delete a saved profile."""
        profiles = list_profiles()
        if not profiles:
            messagebox.showinfo("Delete Profile", "No profiles to delete.")
            return
        name = tk.simpledialog.askstring(
            "Delete Profile", "Enter profile name to delete:",
            initialvalue=profiles[0] if profiles else "")
        if not name or name not in profiles:
            messagebox.showwarning("Delete Profile", f"Profile '{name}' not found.")
            return
        if not messagebox.askyesno("Confirm Delete",
                                    f"Delete profile '{name}'?"):
            return
        delete_profile(name)
        self.status_var.set(f"Profile '{name}' deleted")
        self.after(3000, lambda: self.status_var.set("Idle"))

    def _start_launch(self):
        if self._launch_running or (self.process is not None and self.process.poll() is None):
            return
        self._update_validation_and_command()
        command = self._command()
        if not command:
            messagebox.showerror("Launch blocked", self.validation_var.get())
            self.status_var.set("Launch blocked: invalid configuration")
            return
        self._append_output(f"$ {self.command_var.get()}\n")
        self._aux_stopping.clear()
        try:
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
                env=subprocess_env(),
            )
        except OSError as exc:
            messagebox.showerror("Failed to start launch", str(exc))
            return

        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self._launch_running = True
        self._update_reset_button()
        self.status_var.set(f"Running: {command[3]}")
        threading.Thread(target=self._read_process_output, daemon=True).start()

    def _read_process_output(self):
        assert self.process is not None
        process = self.process
        def read_lines():
            log_root = Path(subprocess_env().get('ROBOT_LAB_RUNTIME_ROOT',
                            '/workspace/molar/robot_lab_runtime'))/'gui'/'runs'
            log_root.mkdir(parents=True, exist_ok=True)
            log_path = log_root/f'launch-{time.time_ns()}-{process.pid}.log'
            self.output_queue.put(('line', f'[full launch log: {log_path}]\n'))
            dropped = 0
            with log_path.open('w', buffering=1) as log:
                for line in process.stdout:
                    log.write(line)
                    if self.output_queue.qsize() >= 4000:
                        dropped += 1
                        continue
                    if dropped:
                        self.output_queue.put(('line', f'[{dropped} console lines omitted; '
                            f'full output is in {log_path}]\n'))
                        dropped = 0
                    self.output_queue.put(("line", line))
        reader = threading.Thread(target=read_lines, daemon=True)
        reader.start()
        # Do not wait for pipe EOF first: an orphaned server can retain that
        # pipe indefinitely after ros2 launch has exited.
        return_code = process.wait()
        stop_group(process.pid, interrupt_timeout=0.0)
        reader.join(timeout=1.0)
        self._lifecycle_queue.put((process, return_code))

    def _poll_output(self):
        # A failing ROS graph can produce lines faster than Tk can draw them.
        # Bound each callback so Drive/Stop and window events still run.
        deadline = time.monotonic() + .02
        launch_lines, console_lines = [], []
        try:
            while True:
                process, return_code = self._lifecycle_queue.get_nowait()
                launch_lines.append(f'\n[launch exited with code {return_code}]\n')
                if process is self.process:
                    self._stop_aux_commands()
                    self._stop_drive()
                    self._launch_running = False
                    self._update_reset_button()
                    self._update_validation_and_command()
                    self.stop_button.configure(state='disabled')
                    self.status_var.set('Idle')
        except queue.Empty:
            pass
        try:
            for _ in range(200):
                kind, payload = self.output_queue.get_nowait()
                if kind == "line":
                    launch_lines.append(payload)
                elif kind in ("cline", "live_monitor"):
                    console_lines.append(payload)
                if time.monotonic() >= deadline:
                    break
        except queue.Empty:
            pass
        if launch_lines:
            self._append_output(''.join(launch_lines))
        if console_lines:
            self._console_append(''.join(console_lines))
        self.after(100, self._poll_output)

    def _refresh_maps(self):
        """Reload maps, robots and algorithms from the config files."""
        self.map_profiles = merge_installed_profiles(load_yaml(self.maps_config_path), 'maps').get('maps', {})
        self.robot_profiles = merge_installed_profiles(load_yaml(self.robots_config_path), 'robots').get('robots', {})
        self.algorithms = self._load_algorithms()
        if COMPOSITION_AVAILABLE:
            self.composition_registry = get_registry()
        # Update the dropdown values so new entries appear immediately
        self.robot_groups = AssetGroups('robots', self.robot_profiles)
        self.map_groups = AssetGroups('maps', self.map_profiles)
        self.robot_taxonomy = RobotTaxonomy(self.robot_groups)
        self._refresh_slot_combos()
        self._update_from_selection()
        self.status_var.set("Refreshed catalogs")
        self.after(3000, lambda: self.status_var.set("Idle"))

    def _check_ros_status(self):
        """Check if ROS 2 is available and show status in the status bar."""
        # Prefer the environment used for launching subprocesses, which
        # contains the sourced ROS 2 overlay even when the GUI itself
        # was started from a shell without ROS sourced.
        ros2_path = shutil.which("ros2")
        if not ros2_path:
            try:
                ros2_path = shutil.which(
                    "ros2", path=subprocess_env().get("PATH", ""))
            except Exception:
                ros2_path = None
        if ros2_path:
            self.ros_status_var.set("OK Available")
            if THEME_AVAILABLE:
                self._ros_status_dot.configure(fg=STATUS_OK)
        else:
            self.ros_status_var.set("X Not found")
            if THEME_AVAILABLE:
                self._ros_status_dot.configure(fg=STATUS_ERROR)

    def _update_proc_count(self):
        """Update the background process count in the status bar."""
        count = sum(1 for p in self.bg_processes.values()
                    if p.poll() is None)
        self.proc_count_var.set(str(count))

    def _append_output(self, text):
        self.output.configure(state="normal")
        self.output.insert("end", text)
        self._trim_console(self.output)
        self.output.see("end")
        self.output.configure(state="disabled")
        self._update_proc_count()

    def _console_append(self, text):
        """Append text to the shared bottom console (all lab tabs)."""
        self.console.configure(state="normal")
        self.console.insert("end", text)
        self._trim_console(self.console)
        self.console.see("end")
        self.console.configure(state="disabled")

    @staticmethod
    def _trim_console(widget):
        # Keep a useful recent history without making long trials redraw an
        # unbounded Text widget. ROS still writes its full logs separately.
        lines = int(widget.index('end-1c').split('.')[0])
        if lines > 10000:
            widget.delete('1.0', f'{lines-9999}.0')

    def _bind_drive_button(self, button, linear_scale, angular_scale, mirror=False):
        direction = (linear_scale, angular_scale)
        if mirror:
            self.drive_button_mirrors.setdefault(direction, []).append(button)
        else:
            self.drive_button_widgets[direction] = button
        button.configure(command=lambda: self._start_drive(*direction))

    def _drive_widgets(self, direction=None):
        directions = self.drive_button_widgets if direction is None else [direction]
        return [button for key in directions for button in
                ([self.drive_button_widgets[key]] if key in self.drive_button_widgets else [])
                + self.drive_button_mirrors.get(key, [])]

    def _strafe_widgets(self, sign=None):
        directions = self.drive_strafe_widgets if sign is None else [sign]
        return [button for key in directions for button in
                ([self.drive_strafe_widgets[key]] if key in self.drive_strafe_widgets else [])
                + self.drive_strafe_mirrors.get(key, [])]

    def _bind_strafe_button(self, button, sign, mirror=False):
        """Latch a lateral command (mecanum); only enabled for that drive."""
        if mirror:
            self.drive_strafe_mirrors.setdefault(sign, []).append(button)
        else:
            self.drive_strafe_widgets[sign] = button
        button.configure(command=lambda: self._start_strafe(sign))

    def _start_strafe(self, sign):
        if not self._lateral_drive_selectable():
            return
        self.drive_stop_latched = False
        if sign in self.drive_strafe_buttons:
            self.drive_strafe_buttons.remove(sign)
        else:
            self.drive_strafe_buttons.add(sign)
        for button in self._strafe_widgets(sign):
            button.state(["pressed"] if sign in self.drive_strafe_buttons
                         else ["!pressed"])
        self._schedule_drive()

    def _start_altitude(self, sign):
        if not self._flight_selectable():
            return
        self.drive_stop_latched = False
        if sign in self.drive_altitude_buttons:
            self.drive_altitude_buttons.remove(sign)
        else:
            self.drive_altitude_buttons.clear()
            self.drive_altitude_buttons.add(sign)
        for direction,button in self.drive_altitude_widgets.items():
            button.state(['pressed'] if direction in self.drive_altitude_buttons else ['!pressed'])
        self._schedule_drive()

    def _toggle_drive_input(self):
        if self._extension_drive_unavailable():
            self.drive_input_enabled.set(False)
            return
        if not self.drive_input_enabled.get():
            self.drive_keys.clear()
            self.drive_joystick.close()
            self.drive_status_var.set("Keyboard/joystick off")
        else:
            self.drive_status_var.set("WASD active; looking for joystick")
        self._schedule_drive()

    def _drive_key_press(self, event):
        if not self.drive_input_enabled.get():
            return
        if isinstance(event.widget, (tk.Entry, tk.Text, ttk.Entry,
                                     ttk.Spinbox, ttk.Combobox)):
            return
        key = event.keysym.lower()
        if key in ("space", "spacebar"):
            self._stop_drive(keep_input_enabled=True)
            return
        if key in ("w", "a", "s", "d"):
            self.drive_keys.add(key)
            self._schedule_drive()

    def _drive_key_release(self, event):
        self.drive_keys.discard(event.keysym.lower())

    def _schedule_drive(self):
        if self.drive_repeat_job is None:
            self._repeat_drive()

    def _ensure_ros_publisher(self):
        if rclpy is None or Twist is None:
            return False
        if not rclpy.ok():
            rclpy.init(args=None)
        if self.ros_node is None:
            self.ros_node = rclpy.create_node("robot_lab_gui")
        if self.cmd_vel_pub is None:
            self.cmd_vel_pub = self.ros_node.create_publisher(Twist, "/key_vel", 10)
        return True

    def _publish_drive(self, linear, angular, lateral=0.0, vertical=0.0):
        if self._ensure_ros_publisher():
            msg = Twist()
            msg.linear.x = float(linear)
            msg.linear.y = float(lateral)
            msg.linear.z = float(vertical) if self._flight_selectable() else 0.0
            msg.angular.z = float(angular)
            self.cmd_vel_pub.publish(msg)
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)
            return

        command = [
            "ros2",
            "topic",
            "pub",
            "--once",
            "/key_vel",
            "geometry_msgs/msg/Twist",
            f"{{linear: {{x: {float(linear):.3f}, y: {float(lateral):.3f}, "
            f"z: {float(vertical) if self._flight_selectable() else 0.0:.3f}}}, "
            f"angular: {{z: {float(angular):.3f}}}}}",
        ]
        subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=subprocess_env())

    def _start_drive(self, linear_scale, angular_scale):
        if self._extension_drive_unavailable():
            return
        self.drive_stop_latched = False
        direction = (linear_scale, angular_scale)
        if direction in self.drive_buttons:
            self.drive_buttons.remove(direction)
        else:
            self.drive_buttons.add(direction)
        for button in self._drive_widgets(direction):
            button.state(["pressed"] if direction in self.drive_buttons else ["!pressed"])
        self._schedule_drive()

    def _resolved_drive_limits(self):
        robot = self.robot_var.get()
        profile = self.robot_profiles.get(robot, {}).get("drive", {})
        max_linear, max_angular, linear_step, angular_step = limits_from_drive(profile)
        min_linear, min_angular = -max_linear, -max_angular
        linear_brake, angular_brake = linear_step, angular_step
        if self.drive_override_var.get():
            try:
                max_linear = min(max_linear, max(0.0, float(self.drive_max_linear_var.get())))
                max_angular = min(max_angular, max(0.0, float(self.drive_max_angular_var.get())))
                min_linear = max(min_linear, min(0.0, float(self.drive_min_linear_var.get())))
                min_angular = max(min_angular, min(0.0, float(self.drive_min_angular_var.get())))
                linear_step = max(0.0, float(self.drive_linear_var.get()))
                angular_step = max(0.0, float(self.drive_angular_var.get()))
                linear_brake = max(0.0, float(self.drive_decel_linear_var.get()))
                angular_brake = max(0.0, float(self.drive_decel_angular_var.get()))
            except (ValueError, tk.TclError):
                pass
        if robot == "berkeley_humanoid_lite_sim" and self.simulator_var.get() == "mujoco":
            max_angular = min_angular = 0.0
        return (min_linear, max_linear, min_angular, max_angular,
                linear_step, angular_step, linear_brake, angular_brake)

    def _update_drive_limits_label(self):
        (min_linear, max_linear, min_angular, max_angular,
         linear_step, angular_step, linear_brake, angular_brake) = self._resolved_drive_limits()
        robot = self.robot_var.get()
        source = "override" if self.drive_override_var.get() else "robot profile"
        self.drive_limits_var.set(
            f"{robot} ({source}): linear [{min_linear:.2f}, {max_linear:.2f}] m/s; "
            f"angular [{min_angular:.2f}, {max_angular:.2f}] rad/s\n"
            f"Δ accel/brake per 0.1 s: {linear_step:.3f}/{linear_brake:.3f} m/s, "
            f"{angular_step:.3f}/{angular_brake:.3f} rad/s")

        for widget in getattr(self, 'drive_limit_widgets', []):
            widget.state(['!disabled'] if self.drive_override_var.get() else ['disabled'])

    def _release_drive(self, linear_scale, angular_scale):
        direction = (linear_scale, angular_scale)
        self.drive_buttons.discard(direction)
        for button in self._drive_widgets(direction):
            button.state(["!pressed"])

    def _repeat_drive(self):
        self.drive_repeat_job = None
        was_moving = (any(abs(value) > 1e-9 for value in self.current_drive)
                      or abs(self.current_lateral) > 1e-9)
        was_vertical = abs(self.current_vertical)>1e-9
        was_moving = was_moving or was_vertical
        (min_linear, max_linear, min_angular, max_angular,
         linear_step, angular_step, linear_brake, angular_brake) = self._resolved_drive_limits()
        self.drive_model.max_linear = max_linear
        self.drive_model.min_linear = min_linear
        self.drive_model.min_angular = min_angular
        self.drive_model.max_angular = max_angular
        self.drive_strafe_model.max_linear = max_linear
        self.drive_strafe_model.min_linear = min_linear
        self.drive_altitude_model.max_linear = max_linear
        self.drive_altitude_model.min_linear = min_linear
        if self.drive_model.max_angular == 0.0:
            self.drive_model.angular = 0.0
            self.drive_model.min_angular = 0.0
        linear_input = sum(value[0] for value in self.drive_buttons)
        angular_input = sum(value[1] for value in self.drive_buttons)
        # Lateral input exists for mecanum and 4WS parallel steering; buttons are
        # disabled otherwise, so this also covers a stale latch.
        strafe_input = (sum(self.drive_strafe_buttons)
                        if self._lateral_drive_selectable() else 0.0)
        vertical_input = sum(self.drive_altitude_buttons) if self._flight_selectable() else 0.0
        if self.drive_input_enabled.get():
            linear_input += int("w" in self.drive_keys) - int("s" in self.drive_keys)
            angular_input += int("a" in self.drive_keys) - int("d" in self.drive_keys)
            joy_linear, joy_angular = self.drive_joystick.poll()
            linear_input += joy_linear
            angular_input += joy_angular
            if self.drive_joystick.path:
                self.drive_status_var.set(f"WASD + {self.drive_joystick.path}")
        if self.drive_stop_latched:
            if not self.drive_buttons and not self.drive_strafe_buttons and not self.drive_altitude_buttons \
                    and not self.drive_keys and \
                    abs(linear_input) < 1e-9 and abs(angular_input) < 1e-9:
                self.drive_stop_latched = False
            linear_input = angular_input = strafe_input = vertical_input = 0.0
        self._update_drive_limits_label()
        linear, angular = self.drive_model.step(
            linear_input, angular_input, linear_step, angular_step,
            linear_brake, angular_brake)
        lateral, _ = self.drive_strafe_model.step(
            strafe_input, 0.0, linear_step, angular_step,
            linear_brake, angular_brake)
        self.current_drive = (linear, angular)
        self.current_lateral = lateral
        vertical,_ = self.drive_altitude_model.step(vertical_input,0.0,linear_step,angular_step,
                                                   linear_brake,angular_brake)
        self.current_vertical = vertical
        # Arming WASD/joystick only starts polling. It must not put even a
        # zero Twist on /key_vel until a real input is made. Once moving,
        # keep publishing through the deceleration and its final zero.
        if (self.drive_buttons or self.drive_strafe_buttons or self.drive_keys or
                abs(linear_input) > 1e-9 or abs(angular_input) > 1e-9 or
                abs(strafe_input) > 1e-9 or abs(vertical_input)>1e-9 or was_moving or
                abs(linear) > 1e-9 or abs(angular) > 1e-9 or
                abs(lateral) > 1e-9):
            if abs(vertical)>1e-9 or was_vertical:
                self._publish_drive(linear, angular, lateral, vertical)
            else:
                self._publish_drive(linear, angular, lateral)
        if (self.drive_buttons or self.drive_strafe_buttons or self.drive_altitude_buttons or self.drive_keys
                or self.drive_input_enabled.get()
                or abs(linear) > 1e-9 or abs(angular) > 1e-9
                or abs(lateral) > 1e-9 or abs(vertical)>1e-9):
            self.drive_repeat_job = self.after(100, self._repeat_drive)

    def _stop_drive(self, keep_input_enabled=False):
        if self.drive_repeat_job is not None:
            self.after_cancel(self.drive_repeat_job)
            self.drive_repeat_job = None
        if not keep_input_enabled:
            self.drive_input_enabled.set(False)
            self.drive_joystick.close()
            self.drive_status_var.set("Keyboard/joystick off")
        self.drive_buttons.clear()
        self.drive_strafe_buttons.clear()
        self.drive_altitude_buttons.clear()
        for button in self.drive_altitude_widgets.values():
            button.state(['!pressed'])
        self.drive_stop_latched = keep_input_enabled
        for button in self._drive_widgets():
            button.state(["!pressed"])
        for button in self._strafe_widgets():
            button.state(["!pressed"])
        self.drive_keys.clear()
        self.current_drive = self.drive_model.stop()
        self.current_lateral = self.drive_strafe_model.stop()[0]
        self.current_vertical = self.drive_altitude_model.stop()[0]
        self._publish_drive(0.0, 0.0, 0.0)
        if keep_input_enabled:
            self._schedule_drive()

    def _default_map_save_dir(self):
        workspace_maps = Path.cwd() / "src" / "robot_lab_maps" / "maps" / self.map_var.get() / "maps"
        if workspace_maps.parent.exists():
            workspace_maps.mkdir(parents=True, exist_ok=True)
            return str(workspace_maps)

        package_maps = Path(self.maps_share) / "maps" / self.map_var.get() / "maps"
        package_maps.mkdir(parents=True, exist_ok=True)
        return str(package_maps)

    def _save_map(self):
        if self.mode_var.get() == "slam":
            self._save_2d_map()
        elif self.mode_var.get() == "3d_slam":
            self._save_3d_map()
        else:
            messagebox.showinfo("Save map", "Start a SLAM mode launch before saving a map.")

    def _save_2d_map(self):
        target = filedialog.asksaveasfilename(
            title="Save 2D map",
            initialdir=self._default_map_save_dir(),
            initialfile="map",
        )
        if not target:
            return

        prefix = str(Path(target))
        for suffix in (".yaml", ".pgm"):
            if prefix.endswith(suffix):
                prefix = prefix[:-len(suffix)]
                break
        Path(prefix).parent.mkdir(parents=True, exist_ok=True)

        command = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", prefix]
        self._append_output(f"$ {' '.join(command)}\n")
        threading.Thread(target=self._run_aux_command, args=(command, "map saver"), daemon=True).start()

    def _default_3d_map_save_dir(self):
        workspace_maps = Path.cwd() / "src" / "robot_lab_maps" / "maps" / self.map_var.get() / "rtabmap"
        if workspace_maps.parent.exists():
            workspace_maps.mkdir(parents=True, exist_ok=True)
            return str(workspace_maps)

        fallback = Path.cwd() / "log" / "rtabmap_exports"
        fallback.mkdir(parents=True, exist_ok=True)
        return str(fallback)

    def _save_3d_map(self):
        source = Path(self._rtabmap_database_path())
        if not source.exists():
            messagebox.showerror(
                "Save 3D map",
                f"RTAB-Map database does not exist yet:\n{source}\n\nStart 3D SLAM and move the robot first.",
            )
            return

        target = filedialog.asksaveasfilename(
            title="Save 3D map (database and point cloud)",
            initialdir=self._default_3d_map_save_dir(),
            initialfile=f"{self.map_var.get()}_{self.robot_var.get()}_rtabmap.db",
            defaultextension=".db",
            filetypes=[("RTAB-Map database", "*.db"), ("All files", "*")],
        )
        if not target:
            return

        target_path = Path(target)
        target_path.parent.mkdir(parents=True, exist_ok=True)
        if source.resolve() == target_path.resolve():
            messagebox.showerror('Save 3D map', 'Choose a file other than the active RTAB-Map database.')
            return
        self._stop_drive()
        threading.Thread(target=self._snapshot_3d_map,
                         args=(source, target_path), daemon=True).start()

    def _snapshot_3d_map(self, source, target):
        try:
            # RTAB-Map keeps recent keyframes in memory. Its acknowledged
            # backup service flushes them before creating a stable .back file.
            command = ['ros2', 'service', 'call', '/rtabmap/backup', 'std_srvs/srv/Empty', '{}']
            result = subprocess.run(command, env=subprocess_env(), capture_output=True,
                                    text=True, timeout=35)
            self.output_queue.put(('cline', result.stdout+result.stderr))
            if result.returncode != 0 or 'response:' not in result.stdout:
                raise RuntimeError('RTAB-Map backup was not acknowledged')
            backup = Path(str(source)+'.back')
            with sqlite3.connect(backup.resolve().as_uri()+'?mode=ro', uri=True) as src_db, \
                    sqlite3.connect(target) as target_db:
                src_db.backup(target_db)
            self.output_queue.put(('cline', f'[saved 3D map database to {target}]\n'))
            self._run_aux_command(['ros2', 'run', 'robot_lab_bringup', 'export_3d_map.py',
                '--cloud-topic', '/cloud_map', '--output', str(target.with_suffix('.pcd'))], '3D point cloud export')
        except (OSError, sqlite3.Error, RuntimeError, subprocess.TimeoutExpired) as exc:
            self.output_queue.put(('cline', f'[3D map save failed: {exc}]\n'))

    def _run_aux_command(self, command, label):
        if self._aux_stopping.is_set():
            return
        try:
            with self._aux_lock:
                if self._aux_stopping.is_set():
                    return
                process = subprocess.Popen(
                    command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, start_new_session=True, env=subprocess_env())
                self._aux_processes.add(process)
        except OSError as exc:
            self.output_queue.put(("cline", f"[{label} failed to start: {exc}]\n"))
            return

        try:
            try:
                output, _ = process.communicate(timeout=40)
            except subprocess.TimeoutExpired:
                self.output_queue.put(('cline', f'[{label} timed out after 40 seconds]\n'))
                stop_group(process.pid, interrupt_timeout=.5, terminate_timeout=.5)
                output, _ = process.communicate(timeout=3)
            self.output_queue.put(('cline', output))
            self.output_queue.put(('cline', f'[{label} exited with code {process.returncode}]\n'))
        finally:
            with self._aux_lock:
                self._aux_processes.discard(process)

    def _stop_aux_commands(self):
        self._aux_stopping.set()
        with self._aux_lock:
            processes = list(self._aux_processes)
        for process in processes:
            threading.Thread(target=stop_group, args=(process.pid,),
                             kwargs=dict(interrupt_timeout=.5, terminate_timeout=.5),
                             daemon=False).start()

    def _stop_launch(self):
        self._stop_aux_commands()
        if not self.process or not self._launch_running:
            return
        self._stop_drive()
        if hasattr(self, 'arm_tab'):
            self.arm_tab.stop()
        if hasattr(self, 'hand_tab'):
            self.hand_tab.stop()
        self.status_var.set("Stopping...")
        threading.Thread(target=stop_owned_launch, args=(self.process,), daemon=False).start()

    # ---- Control-center APIs used by the lab tabs ----
    def show_tab(self, title):
        """Raise a workspace or a page in the Launch control column."""
        controls = {'Drive': 'Drive & limits', 'Arm': 'Arm', 'Hand': 'Hand', 'Drone': 'Drone & limits'}
        if title in controls:
            self.notebook.select(self.launch_tab)
            for page in self.control_notebook.tabs():
                if self.control_notebook.tab(page, 'text') == controls[title]:
                    self.control_notebook.select(page)
                    return

        for tab_id in self.notebook.tabs():
            if self.notebook.tab(tab_id, "text") == title:
                self.notebook.select(tab_id)
                return

    def log(self, text):
        """Append text to the shared console (thread-safe via the queue)."""
        self.output_queue.put(("cline", text))

    def set_status(self, text):
        self.status_var.set(text)

    def start_launch_command(self, command):
        """Start an arbitrary launch command in the main launch slot."""
        if self._launch_running or (self.process and self.process.poll() is None):
            messagebox.showinfo(
                "Launch running",
                "Stop the current launch before starting another one.",
            )
            return
        self._append_output(f"$ {' '.join(str(part) for part in command)}\n")
        self._aux_stopping.clear()
        try:
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                start_new_session=True,
                env=subprocess_env(),
            )
        except OSError as exc:
            messagebox.showerror("Failed to start launch", str(exc))
            return
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self._launch_running = True
        self.status_var.set(f"Running: {command[3] if len(command) > 3 else command[0]}")
        self._update_reset_button()
        threading.Thread(target=self._read_process_output, daemon=True).start()

    def stop_launch(self):
        """Public stop hook used by the lab tabs."""
        self._stop_launch()

    def start_bg_process(self, command, key):
        """Run a background process, streaming output to the shared console."""
        existing = self.bg_processes.get(key)
        if existing and existing.poll() is None:
            self._append_output(f"[{key}] already running\n")
            return
        self._append_output(f"$ {' '.join(str(part) for part in command)}\n")
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=subprocess_env(),
            )
        except OSError as exc:
            self._append_output(f"[{key} failed to start: {exc}]\n")
            return
        self.bg_processes[key] = process
        threading.Thread(target=self._read_bg_output, args=(process, key), daemon=True).start()

    def _read_bg_output(self, process, key):
        for line in process.stdout:
            self.output_queue.put(("cline", line))
        return_code = process.wait()
        self.output_queue.put(("cline", f"[{key} exited with code {return_code}]\n"))

    def stop_bg_process(self, key):
        """Terminate a named background process if it is running."""
        process = self.bg_processes.get(key)
        if process and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()

    def stop_all_bg(self):
        """Terminate every tracked background process."""
        for key in list(self.bg_processes):
            self.stop_bg_process(key)

    def _on_close(self):
        preview = getattr(self, 'registry_preview', None)
        if preview is not None:
            preview.close()
        self._stop_aux_commands()
        if hasattr(self, 'arm_tab'):
            self.arm_tab.close()
        if hasattr(self, 'hand_tab'):
            self.hand_tab.close()
        self._stop_drive()
        self.stop_all_bg()
        # Stop the live monitor's ROS thread before shutting rclpy down
        monitor = getattr(self, "live_monitor_tab", None)
        if monitor is not None:
            monitor.shutdown()
        if self.process and self._launch_running:
            threading.Thread(target=stop_owned_launch, args=(self.process,), daemon=False).start()
        if self.ros_node is not None:
            self.ros_node.destroy_node()
            self.ros_node = None
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()
        self.destroy()


def main():
    app = SimulationLauncherGui()
    app.mainloop()


if __name__ == "__main__":
    main()
