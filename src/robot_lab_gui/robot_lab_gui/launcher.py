#!/usr/bin/env python3

import os
from pathlib import Path
import queue
import shlex
import shutil
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, simpledialog, ttk

from ament_index_python.packages import get_package_share_directory

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


MODE_ORDER = ["display", "loc", "slam", "3d_slam", "nav"]
MODE_TOOLTIPS = {
    "display": "Visualize a robot and/or a map in the selected simulator "
               "(no physics stack, controllers or localization).",
    "loc": "Localization: localize against a known map.",
    "slam": "SLAM: build a 2D map while localizing.",
    "3d_slam": "3D SLAM: build a 3D map (RGB-D sensor required).",
    "nav": "Navigation: plan and follow paths (2D map required).",
}
MODE_LABELS = {
    "display": "Display",
    "loc": "Localization",
    "slam": "SLAM",
    "3d_slam": "3D SLAM",
    "nav": "Navigation",
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
        self.geometry("1280x860")
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
        self.map_profiles = load_yaml(self.maps_config_path).get("maps", {})
        self.robot_profiles = load_yaml(self.robots_config_path).get("robots", {})

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
        self.current_drive = (0.0, 0.0)
        self._launch_running = False
        self._last_fixes = []
        self.output_queue = queue.Queue()
        self._output_autoscroll = True  # follow-tail for Launch Output

        self.robot_var = tk.StringVar(
            value=self._first_key(self.robot_profiles, "bumperbot"))
        self.map_var = tk.StringVar(
            value=self._first_key(self.map_profiles, "celisca_floor_1"))
        self.mode_var = tk.StringVar(value="display")
        self.simulator_var = tk.StringVar(value="gazebo")
        self.launch_kind_var = tk.StringVar(value="simulation")
        self.drive_linear_var = tk.DoubleVar(value=0.25)
        self.drive_angular_var = tk.DoubleVar(value=0.8)
        self.gui_var = tk.StringVar(value="auto")
        self.command_var = tk.StringVar()
        self._prepared_command = []
        self.summary_var = tk.StringVar()
        self.status_var = tk.StringVar(value="Idle")

        self.mode_buttons = {}
        self._build_ui()
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

    def _algorithms_for_category(self, category):
        """Runnable algorithm IDs for *category*.

        Filtered by what the bringup layer can actually start, so a slot
        never lists an option that would fail the launch.
        """
        return [
            a["id"] for a in self.algorithms
            if a.get("category") == category
            and not self._algorithm_unavailable_reason(category, a["id"])
        ]

    def _algorithm_name(self, algorithm_id):
        """Return the human-readable name for an algorithm ID."""
        for a in self.algorithms:
            if a.get("id") == algorithm_id:
                return a.get("name", algorithm_id)
        return algorithm_id

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        # Top-level notebook: one full-size tab per control-center area
        self.notebook = ttk.Notebook(self)
        self.notebook.grid(row=0, column=0, sticky="nsew")

        launch_tab = ttk.Frame(self.notebook)
        self.notebook.add(launch_tab, text="Launch")
        launch_tab.columnconfigure(0, weight=0)
        launch_tab.columnconfigure(1, weight=1)
        launch_tab.rowconfigure(0, weight=1)

        # Scrollable left controls panel (inside the Launch tab)
        left_container = ttk.Frame(launch_tab)
        left_container.grid(row=0, column=0, sticky="ns")
        left_container.columnconfigure(0, weight=1)
        left_container.rowconfigure(0, weight=1)

        scrollbar = ttk.Scrollbar(left_container, orient="vertical")
        scrollbar.grid(row=0, column=1, sticky="ns")

        canvas = tk.Canvas(
            left_container,
            borderwidth=0,
            highlightthickness=0,
            yscrollcommand=scrollbar.set,
            width=372,
        )
        try:
            canvas.configure(bg='#1e1e2e')
        except Exception:
            pass
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.configure(command=canvas.yview)

        controls = ttk.Frame(canvas, padding=12)
        canvas_window = canvas.create_window((0, 0), window=controls, anchor="nw")

        def _on_frame_configure(event):
            canvas.configure(scrollregion=canvas.bbox("all"))

        controls.bind("<Configure>", _on_frame_configure)

        def _on_canvas_configure(event):
            canvas.itemconfig(canvas_window, width=event.width)

        canvas.bind("<Configure>", _on_canvas_configure)

        def _on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        def _on_button4(event):
            canvas.yview_scroll(-1, "units")

        def _on_button5(event):
            canvas.yview_scroll(1, "units")

        for _seq, _fn in (("<MouseWheel>", _on_mousewheel),
                           ("<Button-4>", _on_button4),
                           ("<Button-5>", _on_button5)):
            canvas.bind(_seq, _fn)
            controls.bind(_seq, _fn)

        controls.columnconfigure(0, weight=1)

        ttk.Label(controls, text="Robot").grid(row=0, column=0, sticky="w")
        robot_frame = ttk.Frame(controls)
        robot_frame.grid(row=1, column=0, sticky="ew", pady=(2, 12))
        robot_frame.columnconfigure(0, weight=1)
        self.robot_combo = ttk.Combobox(
            robot_frame,
            textvariable=self.robot_var,
            values=[NONE_LABEL] + sorted(self.robot_profiles.keys()),
            state="readonly",
            width=34,
        )
        self.robot_combo.grid(row=0, column=0, sticky="ew")
        self.robot_combo.bind("<<ComboboxSelected>>", self._on_selection_changed)
        add_tooltip(
            self.robot_combo,
            "Robot to simulate. '%s' shows the map with no robot "
            "(display mode only)." % NONE_LABEL)
        self.robot_info_var = tk.StringVar(value="")
        ttk.Label(
            robot_frame,
            textvariable=self.robot_info_var,
            foreground="#a6adc8" if THEME_AVAILABLE else "#555555",
            wraplength=330,
            justify="left",
        ).grid(row=1, column=0, sticky="ew", pady=(4, 0))

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
            button.grid(row=index, column=0, sticky="w", pady=2)
            self.mode_buttons[mode] = button
            add_tooltip(button, MODE_TOOLTIPS.get(mode, ""))

        ttk.Label(controls, text="Map").grid(row=4, column=0, sticky="w")
        self.map_combo = ttk.Combobox(
            controls,
            textvariable=self.map_var,
            values=[NONE_LABEL] + sorted(self.map_profiles.keys()),
            state="readonly",
            width=34,
        )
        self.map_combo.grid(row=5, column=0, sticky="ew", pady=(2, 12))
        self.map_combo.bind("<<ComboboxSelected>>", self._on_selection_changed)
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
            width=34,
        )
        self.simulator_combo.grid(row=7, column=0, sticky="ew", pady=(2, 12))
        self.simulator_combo.bind("<<ComboboxSelected>>", self._simulator_selected)
        add_tooltip(self.simulator_combo, "Select the simulator backend.")

        ttk.Label(controls, text="Launch").grid(row=8, column=0, sticky="w")
        launch_frame = ttk.Frame(controls)
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

        ttk.Label(controls, text="GUI").grid(row=10, column=0, sticky="w", pady=(12, 0))
        gui_frame = ttk.Frame(controls)
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
            controls, text="Simulator availability", padding=(8, 4))
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
            controls, text="Composition - algorithm slots", padding=(8, 6))
        self.composition_frame.grid(row=13, column=0, sticky="ew", pady=(12, 6))
        self.composition_frame.columnconfigure(1, weight=1)
        self._current_mode_steps = None  # Track when mode changes

        # Compatibility status label (color-coded)
        self.compatibility_label = ttk.Label(
            controls,
            textvariable=self.compatibility_var,
            justify="left",
            wraplength=330,
            foreground="#a6adc8" if THEME_AVAILABLE else "#333333",
        )
        self.compatibility_label.grid(row=14, column=0, sticky="ew", pady=(0, 2))
        add_tooltip(self.compatibility_label,
                   "Shows whether the current robot/mode supports all slots.")

        # Separator between composition and validation
        ttk.Separator(controls, orient="horizontal").grid(
            row=15, column=0, sticky="ew", pady=(2, 4))

        # Action buttons for composition management
        action_frame = ttk.Frame(controls)
        action_frame.grid(row=16, column=0, sticky="ew", pady=(2, 4))
        action_frame.columnconfigure(0, weight=1)
        action_frame.columnconfigure(1, weight=1)
        action_frame.columnconfigure(2, weight=1)
        action_frame.columnconfigure(3, weight=1)

        ttk.Button(action_frame, text="Reset",
                   command=self._reset_composition,
                   style="Small.TButton").grid(row=0, column=0, sticky="ew", padx=(0, 3))
        ttk.Button(action_frame, text="Incompat",
                   command=self._show_incompatible,
                   style="Small.TButton").grid(row=0, column=1, sticky="ew", padx=3)
        ttk.Button(action_frame, text="Quick",
                   command=self._quick_select,
                   style="Small.TButton").grid(row=0, column=2, sticky="ew", padx=3)
        ttk.Button(action_frame, text="Cleared",
                   command=self._show_cleared,
                   style="Small.TButton").grid(row=0, column=3, sticky="ew", padx=(3, 0))

        ttk.Label(controls, text="Validation", foreground="#a6adc8" if THEME_AVAILABLE else "#333333"
                  ).grid(row=17, column=0, sticky="w", pady=(4, 2))
        ttk.Label(
            controls,
            textvariable=self.validation_var,
            justify="left",
            wraplength=330,
            foreground="#a6adc8" if THEME_AVAILABLE else "#333333",
        ).grid(row=18, column=0, sticky="ew", pady=(2, 8))

        ttk.Label(controls, text="Resolved Configuration").grid(row=19, column=0, sticky="w")
        summary = ttk.Label(
            controls,
            textvariable=self.summary_var,
            justify="left",
            wraplength=330,
            foreground="#a6adc8" if THEME_AVAILABLE else "#333333",
        )
        summary.grid(row=20, column=0, sticky="ew", pady=(2, 12))

        button_frame = ttk.Frame(controls)
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

        ttk.Label(controls, text="Drive").grid(row=24, column=0, sticky="w", pady=(12, 0))
        drive_frame = ttk.Frame(controls)
        drive_frame.grid(row=25, column=0, sticky="ew", pady=(2, 8))
        for column in range(3):
            drive_frame.columnconfigure(column, weight=1)

        forward_button = ttk.Button(drive_frame, text="Forward")
        forward_button.grid(row=0, column=1, sticky="ew", padx=2, pady=2)
        self._bind_drive_button(forward_button, 1.0, 0.0)

        left_button = ttk.Button(drive_frame, text="Left")
        left_button.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
        self._bind_drive_button(left_button, 0.0, 1.0)

        stop_drive_button = ttk.Button(drive_frame, text="Stop", command=self._stop_drive)
        stop_drive_button.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

        right_button = ttk.Button(drive_frame, text="Right")
        right_button.grid(row=1, column=2, sticky="ew", padx=2, pady=2)
        self._bind_drive_button(right_button, 0.0, -1.0)

        reverse_button = ttk.Button(drive_frame, text="Reverse")
        reverse_button.grid(row=2, column=1, sticky="ew", padx=2, pady=2)
        self._bind_drive_button(reverse_button, -1.0, 0.0)

        speed_frame = ttk.Frame(controls)
        speed_frame.grid(row=26, column=0, sticky="ew", pady=(0, 10))
        speed_frame.columnconfigure(1, weight=1)
        speed_frame.columnconfigure(3, weight=1)
        ttk.Label(speed_frame, text="Linear").grid(row=0, column=0, sticky="w", padx=(0, 4))
        ttk.Spinbox(
            speed_frame,
            from_=0.05,
            to=1.0,
            increment=0.05,
            textvariable=self.drive_linear_var,
            width=6,
        ).grid(row=0, column=1, sticky="ew", padx=(0, 8))
        ttk.Label(speed_frame, text="Angular").grid(row=0, column=2, sticky="w", padx=(0, 4))
        ttk.Spinbox(
            speed_frame,
            from_=0.1,
            to=2.0,
            increment=0.1,
            textvariable=self.drive_angular_var,
            width=6,
        ).grid(row=0, column=3, sticky="ew")

        self.save_map_button = ttk.Button(controls, text="Save Map", command=self._save_map)
        self.save_map_button.grid(row=27, column=0, sticky="ew", pady=(0, 4))

        output_frame = ttk.Frame(launch_tab, padding=(0, 12, 12, 12))
        output_frame.grid(row=0, column=1, sticky="nsew")
        output_frame.columnconfigure(0, weight=1)
        output_frame.rowconfigure(3, weight=1)
        command_header = ttk.Frame(output_frame)
        command_header.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        command_header.columnconfigure(0, weight=1)
        ttk.Label(command_header, text="Command (auto-filled)").grid(
            row=0, column=0, sticky="w")
        self.copy_command_button = ttk.Button(
            command_header, text="Copy Command", command=self._copy_command)
        self.copy_command_button.grid(row=0, column=1, padx=(8, 4))
        self.start_button = ttk.Button(
            command_header, text="Run Command", command=self._start_launch,
            style="Accent.TButton")
        self.start_button.grid(row=0, column=2, padx=4)
        self.stop_button = ttk.Button(
            command_header, text="Stop", command=self._stop_launch,
            state="disabled", style="Danger.TButton")
        self.stop_button.grid(row=0, column=3, padx=(4, 0))
        self.command_preview = scrolledtext.ScrolledText(
            output_frame, wrap="word", height=5, width=1, state="disabled")
        self.command_preview.grid(row=1, column=0, sticky="ew", pady=(0, 12))
        add_tooltip(self.command_preview,
                    "Updates from the selected options. Run here or copy into a ROS 2 terminal.")
        ttk.Label(output_frame, text="Launch Output").grid(row=2, column=0, sticky="w")
        self.output = scrolledtext.ScrolledText(output_frame, wrap="word", height=24)
        self.output.grid(row=3, column=0, sticky="nsew", pady=(2, 0))
        self.output.configure(state="disabled")

        # Shared console: every control-center tab streams its output here
        console_frame = ttk.LabelFrame(self, text="Console", padding=(12, 2, 12, 6))
        console_frame.grid(row=1, column=0, sticky="ew")
        console_frame.columnconfigure(0, weight=1)
        self.console = scrolledtext.ScrolledText(console_frame, wrap="word", height=10)
        self.console.grid(row=0, column=0, sticky="ew", pady=(2, 0))
        self.console.configure(state="disabled")

        # Status bar spans the full window below the console
        if THEME_AVAILABLE:
            status_frame = ttk.Frame(self, style="Statusbar.TFrame")
        else:
            status_frame = ttk.Frame(self, relief="sunken")
        status_frame.grid(row=2, column=0, sticky="ew", pady=(4, 0))

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

    def _on_selection_changed(self, _event):
        self._update_from_selection()

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
                or _simulator_supports_mode(simulator, mode, self.mode_profiles)]

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
        robot_features = robot_config.get("features", [])
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
            missing_features = [
                feature for feature in self._mode_required_features(mode)
                if not robot_free and feature not in robot_features
            ]
            if missing_features:
                reasons[mode] = ("robot lacks %s" % ", ".join(missing_features))
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
        robot_features = robot_config.get("features", [])
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
            if not robot_free and any(
                    feature not in robot_features
                    for feature in self._mode_required_features(mode)):
                continue
            supported.append(mode)
        return supported

    def _resolve_compatibility(self):
        """Fix-point cascade correcting mode <-> simulator compatibility.

        Returns (supported_modes, fixes) after correcting self.mode_var and
        self.simulator_var in place, so every downstream decision (command,
        validation, algorithm slots) sees a compatible selection.
        """
        fixes = []
        simulator = self.simulator_var.get() or "gazebo"
        modes = self._robot_map_modes()
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
                                                 self.mode_profiles)]
            if self.mode_var.get() not in modes:
                new_mode, note = _correction_for(
                    self.mode_var.get(), modes,
                    ["slam", "display", "loc", "nav", "3d_slam"])
                if new_mode:
                    fixes.append("mode %s" % note)
                    self.mode_var.set(new_mode)
            sims = [sim for sim in SIMULATOR_ORDER
                    if _allowed_simulators(self.mode_var.get(),
                                           self.mode_profiles)
                       .get(sim, (False, ""))[0]]
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
                if self._simulator_mode_support(sim, mode)[0]]

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

        # Clear existing widgets
        for widget in self.composition_frame.winfo_children():
            widget.destroy()
        self.slot_combos.clear()
        self.slot_labels.clear()
        self.slot_vars.clear()

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
                if current and not is_none_selection(current) \
                        and current not in algorithms:
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
        )

    def _update_validation_and_command(self):
        """Run the shared validator and refresh command + validation text."""
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

    def _set_command(self, command):
        """Keep the preview, clipboard text and executable arguments in sync."""
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
        self.map_combo.configure(
            values=allowed_maps,
            state="readonly" if allowed_maps else "disabled",
        )
        self.robot_combo.configure(values=self._allowed_robots())

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

    def _robot_info_text(self, supported_modes, supports_vacuum):
        config = self._robot_config()
        features = config.get("features", [])
        lines = [
            f"Class profile: {', '.join(features) if features else 'display only'}",
            f"Modes: {', '.join(MODE_LABELS.get(m, m) for m in supported_modes) or 'none'}",
            f"Cleaning missions: {'yes' if supports_vacuum else 'no'}",
        ]
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
        algorithm_name = self._primary_algorithm_name()
        lines = [
            f"Robot: {self.robot_var.get()} ({robot_note})",
            f"Robot file: {self._resolve_robot_path()}",
            f"Mode: {MODE_LABELS.get(self.mode_var.get(), self.mode_var.get())}",
            f"Algorithm: {algorithm_name}",
            f"Simulator: {SIMULATOR_LABELS.get(simulator, simulator)}",
            f"GUI: {gui_label.get(self.gui_var.get(), self.gui_var.get())}",
            f"RViz: {self._resolve_rviz_path()}",
            f"Gazebo world: {self._resolve_world_path()}",
            f"2D map: {self._map_yaml_path() if self._map_has_2d_map() else 'not available'}",
            f"Vacuum: {'available' if supports_vacuum else 'not available'}",
        ]
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
        arguments = []
        for step_id, category in self._mode_step_categories.items():
            if category is None:
                continue
            variable = self.slot_vars.get(step_id)
            value = variable.get().strip() if variable else ""
            arguments.append("%s:=%s" % (category, selection_value(value)))
        return arguments

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
        try:
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
                env=subprocess_env(),
            )
        except OSError as exc:
            messagebox.showerror("Failed to start launch", str(exc))
            return

        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self._launch_running = True
        self.status_var.set(f"Running: {command[3]}")
        threading.Thread(target=self._read_process_output, daemon=True).start()

    def _read_process_output(self):
        assert self.process is not None
        for line in self.process.stdout:
            self.output_queue.put(("line", line))
        return_code = self.process.wait()
        self._launch_running = False
        self.output_queue.put(("done", return_code))

    def _poll_output(self):
        try:
            while True:
                kind, payload = self.output_queue.get_nowait()
                if kind == "line":
                    self._append_output(payload)
                elif kind == "cline":
                    self._console_append(payload)
                elif kind == "done":
                    self._append_output(f"\n[launch exited with code {payload}]\n")
                    self._stop_drive()
                    self._launch_running = False
                    self._update_validation_and_command()
                    self.stop_button.configure(state="disabled")
                    self.status_var.set("Idle")
        except queue.Empty:
            pass
        self.after(100, self._poll_output)

    def _refresh_maps(self):
        """Reload maps, robots and algorithms from the config files."""
        self.map_profiles = load_yaml(self.maps_config_path).get("maps", {})
        self.robot_profiles = load_yaml(self.robots_config_path).get("robots", {})
        self.algorithms = self._load_algorithms()
        # Update the dropdown values so new entries appear immediately
        self.robot_combo.configure(values=sorted(self.robot_profiles.keys()))
        self.map_combo.configure(values=sorted(self.map_profiles.keys()))
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
        self.output.see("end")
        self.output.configure(state="disabled")
        self._update_proc_count()

    def _console_append(self, text):
        """Append text to the shared bottom console (all lab tabs)."""
        self.console.configure(state="normal")
        self.console.insert("end", text)
        self.console.see("end")
        self.console.configure(state="disabled")

    def _bind_drive_button(self, button, linear_scale, angular_scale):
        button.bind("<ButtonPress-1>", lambda _event: self._start_drive(linear_scale, angular_scale))
        button.bind("<ButtonRelease-1>", lambda _event: self._stop_drive())
        button.bind("<Leave>", lambda _event: self._stop_drive())

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

    def _publish_drive(self, linear, angular):
        if self._ensure_ros_publisher():
            msg = Twist()
            msg.linear.x = float(linear)
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
            f"{{linear: {{x: {float(linear):.3f}}}, angular: {{z: {float(angular):.3f}}}}}",
        ]
        subprocess.Popen(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=subprocess_env())

    def _start_drive(self, linear_scale, angular_scale):
        if self.drive_repeat_job is not None:
            self.after_cancel(self.drive_repeat_job)
            self.drive_repeat_job = None
        linear = float(self.drive_linear_var.get()) * linear_scale
        angular = float(self.drive_angular_var.get()) * angular_scale
        self.current_drive = (linear, angular)
        self._repeat_drive()

    def _repeat_drive(self):
        linear, angular = self.current_drive
        self._publish_drive(linear, angular)
        if linear != 0.0 or angular != 0.0:
            self.drive_repeat_job = self.after(100, self._repeat_drive)

    def _stop_drive(self):
        if self.drive_repeat_job is not None:
            self.after_cancel(self.drive_repeat_job)
            self.drive_repeat_job = None
        self.current_drive = (0.0, 0.0)
        self._publish_drive(0.0, 0.0)

    def _default_map_save_dir(self):
        workspace_maps = Path.cwd() / "src" / "maps" / "maps" / self.map_var.get() / "maps"
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
        workspace_maps = Path.cwd() / "src" / "maps" / "maps" / self.map_var.get() / "rtabmap"
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
            title="Save 3D RTAB-Map (will also export PCD + world)",
            initialdir=self._default_3d_map_save_dir(),
            initialfile=f"{self.map_var.get()}_{self.robot_var.get()}_rtabmap.db",
            defaultextension=".db",
            filetypes=[("RTAB-Map database", "*.db"), ("All files", "*")],
        )
        if not target:
            return

        target_path = Path(target)
        target_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target_path)
        for suffix in ("-wal", "-shm"):
            sidecar = Path(f"{source}{suffix}")
            if sidecar.exists():
                shutil.copy2(sidecar, Path(f"{target_path}{suffix}"))
        self._append_output(f"[saved 3D RTAB-Map database to {target_path}]\n")

        # Use the new exporter for PCD + optional OctoMap + world
        script = os.path.join(os.path.dirname(__file__), "export_3d_map.py")
        if os.path.exists(script):
            out_dir = str(target_path.parent)
            base_name = target_path.stem
            cmd = [
                "python3", script,
                "--db", str(target_path),
                "--output-dir", out_dir,
                "--map-name", base_name,
                "--pcd",
                "--mesh",
                "--world",
                "--octomap",
            ]
            self._append_output(f"$ {' '.join(cmd)}\n")
            threading.Thread(target=self._run_aux_command, args=(cmd, "3d map exporter"), daemon=True).start()
        else:
            self._append_output("[warn] export_3d_map.py not found next to GUI script. Only .db was saved.\n")

    def _run_aux_command(self, command, label):
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
            self.output_queue.put(("cline", f"[{label} failed to start: {exc}]\n"))
            return

        for line in process.stdout:
            self.output_queue.put(("cline", line))
        return_code = process.wait()
        self.output_queue.put(("cline", f"[{label} exited with code {return_code}]\n"))

    def _stop_launch(self):
        if not self.process or self.process.poll() is not None:
            return
        self._stop_drive()
        self.status_var.set("Stopping...")
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
        except OSError:
            return
        self.after(5000, self._terminate_if_running)

    def _terminate_if_running(self):
        if self.process and self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            except OSError:
                pass

    # ---- Control-center APIs used by the lab tabs ----
    def show_tab(self, title):
        """Raise the control-center tab with the given title."""
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
        if self.process and self.process.poll() is None:
            messagebox.showinfo(
                "Launch running",
                "Stop the current launch before starting another one.",
            )
            return
        self._append_output(f"$ {' '.join(str(part) for part in command)}\n")
        try:
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid,
                env=subprocess_env(),
            )
        except OSError as exc:
            messagebox.showerror("Failed to start launch", str(exc))
            return
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self._launch_running = True
        self.status_var.set(f"Running: {command[3] if len(command) > 3 else command[0]}")
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
        self._stop_drive()
        self.stop_all_bg()
        # Stop the live monitor's ROS thread before shutting rclpy down
        monitor = getattr(self, "live_monitor_tab", None)
        if monitor is not None:
            monitor.shutdown()
        if self.process and self.process.poll() is None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            except OSError:
                pass
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
