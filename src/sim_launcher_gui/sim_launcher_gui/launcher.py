#!/usr/bin/env python3

import os
from pathlib import Path
import queue
import shutil
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, scrolledtext, ttk

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


MODE_ORDER = ["display", "loc", "slam", "3d_slam", "nav"]
MODE_LABELS = {
    "display": "Display",
    "loc": "Localization",
    "slam": "SLAM",
    "3d_slam": "3D SLAM",
    "nav": "Navigation",
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
    return os.path.join(get_package_share_directory(package_name), *str(relative_path).split("/"))


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
        self.title("Robot Lab Control Center")
        self.geometry("1280x820")
        self.minsize(1000, 700)

        self.bringup_share = get_package_share_directory("robot_lab_bringup")
        self.robots_share = get_package_share_directory("robots")
        self.maps_share = get_package_share_directory("maps")

        self.modes_config_path = os.path.join(self.bringup_share, "config", "sim_modes.yaml")
        self.maps_config_path = os.path.join(self.bringup_share, "config", "sim_maps.yaml")
        self.robots_config_path = os.path.join(self.robots_share, "config", "robots.yaml")

        self.mode_profiles = load_yaml(self.modes_config_path).get("modes", {})
        self.map_profiles = load_yaml(self.maps_config_path).get("maps", {})
        self.robot_profiles = load_yaml(self.robots_config_path).get("robots", {})

        self.process = None
        self.ros_node = None
        self.cmd_vel_pub = None
        self.drive_repeat_job = None
        self.current_drive = (0.0, 0.0)
        self.output_queue = queue.Queue()

        self.robot_var = tk.StringVar(value=self._first_key(self.robot_profiles, "bumperbot"))
        self.map_var = tk.StringVar(value=self._first_key(self.map_profiles, "celisca_floor_1"))
        self.mode_var = tk.StringVar(value="nav")
        self.launch_kind_var = tk.StringVar(value="simulation")
        self.drive_linear_var = tk.DoubleVar(value=0.25)
        self.drive_angular_var = tk.DoubleVar(value=0.8)
        self.command_var = tk.StringVar()
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
            width=360,
        )
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

        canvas.bind_all("<MouseWheel>", _on_mousewheel)
        canvas.bind_all("<Button-4>", _on_button4)
        canvas.bind_all("<Button-5>", _on_button5)

        controls.columnconfigure(0, weight=1)

        ttk.Label(controls, text="Robot").grid(row=0, column=0, sticky="w")
        robot_frame = ttk.Frame(controls)
        robot_frame.grid(row=1, column=0, sticky="ew", pady=(2, 12))
        robot_frame.columnconfigure(0, weight=1)
        robot_combo = ttk.Combobox(
            robot_frame,
            textvariable=self.robot_var,
            values=sorted(self.robot_profiles.keys()),
            state="readonly",
            width=34,
        )
        robot_combo.grid(row=0, column=0, sticky="ew")
        robot_combo.bind("<<ComboboxSelected>>", self._on_selection_changed)
        self.robot_info_var = tk.StringVar(value="")
        ttk.Label(
            robot_frame,
            textvariable=self.robot_info_var,
            foreground="#555555",
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

        ttk.Label(controls, text="Map").grid(row=4, column=0, sticky="w")
        self.map_combo = ttk.Combobox(
            controls,
            textvariable=self.map_var,
            values=sorted(self.map_profiles.keys()),
            state="readonly",
            width=34,
        )
        self.map_combo.grid(row=5, column=0, sticky="ew", pady=(2, 12))
        self.map_combo.bind("<<ComboboxSelected>>", self._on_selection_changed)

        ttk.Label(controls, text="Launch").grid(row=6, column=0, sticky="w")
        launch_frame = ttk.Frame(controls)
        launch_frame.grid(row=7, column=0, sticky="ew", pady=(2, 12))
        self.simulation_radio = ttk.Radiobutton(
            launch_frame,
            text="Simulation",
            variable=self.launch_kind_var,
            value="simulation",
            command=self._update_from_selection,
        )
        self.simulation_radio.grid(row=0, column=0, sticky="w", pady=2)
        self.vacuum_radio = ttk.Radiobutton(
            launch_frame,
            text="Room vacuum",
            variable=self.launch_kind_var,
            value="vacuum",
            command=self._update_from_selection,
        )
        self.vacuum_radio.grid(row=1, column=0, sticky="w", pady=2)

        ttk.Label(controls, text="Resolved Configuration").grid(row=8, column=0, sticky="w")
        summary = ttk.Label(
            controls,
            textvariable=self.summary_var,
            justify="left",
            wraplength=330,
            foreground="#333333",
        )
        summary.grid(row=9, column=0, sticky="ew", pady=(2, 12))

        ttk.Label(controls, text="Command").grid(row=10, column=0, sticky="w")
        command = ttk.Entry(controls, textvariable=self.command_var, state="readonly", width=44)
        command.grid(row=11, column=0, sticky="ew", pady=(2, 12))

        button_frame = ttk.Frame(controls)
        button_frame.grid(row=12, column=0, sticky="ew")
        button_frame.columnconfigure(0, weight=1)
        button_frame.columnconfigure(1, weight=1)
        self.start_button = ttk.Button(button_frame, text="Start", command=self._start_launch)
        self.start_button.grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.stop_button = ttk.Button(button_frame, text="Stop", command=self._stop_launch, state="disabled")
        self.stop_button.grid(row=0, column=1, sticky="ew", padx=(4, 0))

        self.bg_processes = {}

        ttk.Label(controls, text="Drive").grid(row=13, column=0, sticky="w", pady=(12, 0))
        drive_frame = ttk.Frame(controls)
        drive_frame.grid(row=14, column=0, sticky="ew", pady=(2, 8))
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
        speed_frame.grid(row=15, column=0, sticky="ew", pady=(0, 10))
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
        self.save_map_button.grid(row=16, column=0, sticky="ew", pady=(0, 4))

        output_frame = ttk.Frame(launch_tab, padding=(0, 12, 12, 12))
        output_frame.grid(row=0, column=1, sticky="nsew")
        output_frame.columnconfigure(0, weight=1)
        output_frame.rowconfigure(1, weight=1)
        ttk.Label(output_frame, text="Launch Output").grid(row=0, column=0, sticky="w")
        self.output = scrolledtext.ScrolledText(output_frame, wrap="word", height=24)
        self.output.grid(row=1, column=0, sticky="nsew", pady=(2, 0))
        self.output.configure(state="disabled")

        # Shared console: every control-center tab streams its output here
        console_frame = ttk.LabelFrame(self, text="Console", padding=(12, 2, 12, 6))
        console_frame.grid(row=1, column=0, sticky="ew")
        console_frame.columnconfigure(0, weight=1)
        self.console = scrolledtext.ScrolledText(console_frame, wrap="word", height=10)
        self.console.grid(row=0, column=0, sticky="ew", pady=(2, 0))
        self.console.configure(state="disabled")

        # Status bar spans the full window below the console
        ttk.Label(
            self,
            textvariable=self.status_var,
            foreground="#555555",
            anchor="w",
            padding=(12, 2),
        ).grid(row=2, column=0, sticky="ew")

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

    def _map_yaml_path(self):
        map_config = self._map_config()
        map_file_config = map_config.get("map", {})
        relative_path = map_file_config.get("path", "")
        if not relative_path:
            relative_path = f"maps/{self.map_var.get()}/maps/map.yaml"
        return package_path(map_file_config.get("package", "maps"), relative_path)

    def _map_has_2d_map(self):
        map_file_config = self._map_config().get("map", {})
        configured = map_file_config.get("has_2d_map")
        if configured is not None:
            return bool_value(configured) and os.path.exists(self._map_yaml_path())
        return os.path.exists(self._map_yaml_path())

    def _mode_requires_2d_map(self, mode):
        return bool_value(self.mode_profiles.get(mode, {}).get("requires_2d_map", False))

    def _mode_required_features(self, mode):
        return self.mode_profiles.get(mode, {}).get("required_features", [])

    def _supported_modes(self):
        robot_config = self._robot_config()
        robot_supported = robot_config.get("supported_modes", ["display"])
        robot_features = robot_config.get("features", [])
        map_has_2d_map = self._map_has_2d_map()
        supported = []
        for mode in MODE_ORDER:
            if mode not in robot_supported or mode not in self.mode_profiles:
                continue
            if self._mode_requires_2d_map(mode) and not map_has_2d_map:
                continue
            required_features = self._mode_required_features(mode)
            if any(feature not in robot_features for feature in required_features):
                continue
            supported.append(mode)
        return supported

    def _fallback_mode(self, supported_modes):
        if "slam" in supported_modes:
            return "slam"
        if "display" in supported_modes:
            return "display"
        return supported_modes[0] if supported_modes else "display"

    def _update_from_selection(self):
        supported_modes = self._supported_modes()
        if self.mode_var.get() not in supported_modes:
            self.mode_var.set(self._fallback_mode(supported_modes))

        for mode, button in self.mode_buttons.items():
            if mode in supported_modes:
                button.state(["!disabled"])
            else:
                button.state(["disabled"])

        map_enabled = self.mode_var.get() != "display"
        self.map_combo.configure(state="readonly" if map_enabled else "disabled")

        supports_vacuum = bool_value(self._robot_config().get("supports_room_vacuum", False))
        if not supports_vacuum and self.launch_kind_var.get() == "vacuum":
            self.launch_kind_var.set("simulation")
        self.vacuum_radio.state(["!disabled"] if supports_vacuum else ["disabled"])
        self.save_map_button.state(["!disabled"] if self.mode_var.get() in ("slam", "3d_slam") else ["disabled"])

        self.command_var.set(" ".join(self._command()))
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
        lines = [
            f"Robot: {self.robot_var.get()} ({robot_note})",
            f"Robot file: {self._resolve_robot_path()}",
            f"RViz: {self._resolve_rviz_path()}",
            f"Gazebo: {self._resolve_world_path()}",
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
        launch_file = (
            "simulated_room_vacuum.launch.py"
            if self.launch_kind_var.get() == "vacuum"
            else "simulated_robot.launch.py"
        )
        return [
            "ros2",
            "launch",
            "robot_lab_bringup",
            launch_file,
            f"mode:={self.mode_var.get()}",
            f"map_name:={self.map_var.get()}",
            f"robot_model:={self.robot_var.get()}",
        ]

    def _start_launch(self):
        if self.process and self.process.poll() is None:
            messagebox.showinfo("Launch running", "Stop the current launch before starting another one.")
            return

        command = self._command()
        self._append_output(f"$ {' '.join(command)}\n")
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
        self.status_var.set(f"Running: {command[3]}")
        threading.Thread(target=self._read_process_output, daemon=True).start()

    def _read_process_output(self):
        assert self.process is not None
        for line in self.process.stdout:
            self.output_queue.put(("line", line))
        return_code = self.process.wait()
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
                    self.start_button.configure(state="normal")
                    self.stop_button.configure(state="disabled")
                    self.status_var.set("Idle")
        except queue.Empty:
            pass
        self.after(100, self._poll_output)

    def _append_output(self, text):
        self.output.configure(state="normal")
        self.output.insert("end", text)
        self.output.see("end")
        self.output.configure(state="disabled")

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
            self.ros_node = rclpy.create_node("sim_launcher_gui")
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
