"""Additional control-center tabs for the Robot Lab GUI.

Provides Registry (catalog browsing), Vacuum (cleaning control),
Benchmark (seeded runs + regression), Tests (suite runners),
Health (doctor / platform status / ROS graph), and Live Monitor
(real-time telemetry) tabs.
"""

import json
import threading
import os
import sys
import webbrowser
from pathlib import Path

import tkinter as tk
from tkinter import messagebox, scrolledtext, ttk

from .launcher import load_yaml, subprocess_env

try:
    from .live_monitor import LiveMonitorTab
    HAS_LIVE_MONITOR = True
except ImportError:
    HAS_LIVE_MONITOR = False

try:
    from .themes.modern import (
        BG_DARK, BG_CARD, FG_PRIMARY, FG_MUTED, ACCENT,
        ACCENT_GREEN, ACCENT_RED, ACCENT_YELLOW, STATUS_OK,
        STATUS_ERROR, STATUS_IDLE, STATUS_WARN,
    )
    THEME_AVAILABLE = True
except ImportError:
    THEME_AVAILABLE = False


def find_workspace_root():
    """Locate the workspace root (the directory containing src/robot_lab)."""
    try:
        base = Path(__file__).resolve().parent
    except NameError:
        base = Path.cwd()
    for candidate in [base, *base.parents]:
        if (candidate / "src" / "robot_lab").is_dir():
            return candidate
    return Path.cwd()


WORKSPACE_ROOT = find_workspace_root()
REGISTRY_CONFIG_DIR = WORKSPACE_ROOT / "src" / "robot_lab" / "robot_lab_registry" / "config"
BENCHMARK_SRC = WORKSPACE_ROOT / "src" / "robot_lab" / "robot_lab_benchmark"
STATUS_YAML = WORKSPACE_ROOT / "docs" / "status" / "platform-status.yaml"


def format_platform_status(data):
    """Display current patch and nested task states without historical logs."""
    lines = [f"Platform Status — updated {data.get('updated', 'unknown')}",
             f"Overall: {data.get('overall_state', 'unknown')}",
             data.get('assessment', '')]
    audit = data.get('completion_audit', {})
    if audit:
        lines.extend(["", f"Completion audit: {audit.get('date', '?')}",
                      audit.get('result', ''),
                      f"Report: {audit.get('report', '')}"])
    storage = data.get('storage_and_status_audit', {})
    if storage.get('result'):
        lines.extend(["", storage['result']])
    patches = data.get('priority_patches', {})
    if patches:
        lines.extend(["", "Stabilization patches"])
        for patch_id, patch in patches.items():
            lines.append(f"{patch_id}: {patch.get('state', '?')} — {patch.get('title', '')}")
            if patch.get('result'):
                lines.append(f"  {patch['result']}")
            if patch.get('evidence'):
                lines.append(f"  Evidence: {patch['evidence']}")
    lines.extend(["", "Roadmap tasks"])

    def append_tasks(tasks, depth=0):
        for task_id, task in tasks.items():
            if not isinstance(task, dict):
                continue
            lines.append(f"{'  ' * depth}{task_id}: {task.get('state', '?')} — "
                         f"{task.get('title', '')}")
            append_tasks(task.get('tasks', {}), depth + 1)

    append_tasks(data.get('tasks', {}))
    lines.extend(["", "Current verification"])
    for name, result in data.get('current_verification', {}).items():
        lines.append(f"{name}: {result}")
    return "\n".join(lines) + "\n"


class LabTab(ttk.Frame):
    """Base class for control-center tabs."""

    def __init__(self, notebook, app, title):
        super().__init__(notebook, padding=10)
        self.app = app
        notebook.add(self, text=title)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)


class WorldsTab(LabTab):
    """Generate actual occupancy files for the independently selected world."""
    def __init__(self,notebook,app):
        super().__init__(notebook,app,'Worlds')
        self.world_var=tk.StringVar(value=app.map_var.get())
        self.resolution_var=tk.DoubleVar(value=.1)
        self.height_var=tk.DoubleVar(value=.3)
        self.output_var=tk.StringVar(value=str(Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT',
            '/workspace/molar/robot_lab_runtime'))/'generated_maps'))
        controls=ttk.Frame(self);controls.grid(row=0,column=0,sticky='new')
        controls.columnconfigure(1,weight=1)
        ttk.Label(controls,text='World').grid(row=0,column=0,sticky='w')
        ttk.Combobox(controls,textvariable=self.world_var,values=sorted(app.map_profiles),
                     state='readonly').grid(row=0,column=1,sticky='ew')
        for row,label,var in [(1,'Resolution (m)',self.resolution_var),(2,'Slice height (m)',self.height_var),
                              (3,'Output directory',self.output_var)]:
            ttk.Label(controls,text=label).grid(row=row,column=0,sticky='w')
            ttk.Entry(controls,textvariable=var).grid(row=row,column=1,sticky='ew',pady=3)
        self.generate_button=ttk.Button(controls,text='Generate 2D Occupancy Grid',command=self.generate)
        self.generate_button.grid(row=4,column=0,columnspan=2,sticky='ew',pady=6)
        ttk.Button(controls,text='Stop Generation',command=lambda:app.stop_bg_process('world_maps')).grid(row=5,column=0,sticky='ew')
        ttk.Button(controls,text='World / Terrain Guide',command=lambda:self.open_guide()).grid(row=5,column=1,sticky='ew')
        ttk.Label(controls,text='Uses the pinned Fortress plugin in a separate headless world. Existing maps are preserved.\n'
            'Mesh collisions use their actual scale and height slice; heightfields need the terrain converter.\n'
            'Review the generated PGM/YAML and report before registering it for navigation.\n'
            'External world imports and terrain conversion are tracked as R6.6 / R6.7.',
            wraplength=720,justify='left').grid(row=6,column=0,columnspan=2,sticky='w',pady=12)

    def generation_command(self):
        from ament_index_python.packages import get_package_share_directory
        profile=self.app.map_profiles[self.world_var.get()]
        config=profile.get('gazebo',{})
        package=config.get('world_package','robot_lab_maps')
        if package=='maps':package='robot_lab_maps'
        path=Path(config.get('world_path',''))
        if not path.is_absolute():path=Path(get_package_share_directory(package))/path
        seed=profile.get('spawn',{})
        return ['ros2','run','robot_lab_maps','generate_occupancy_map.py','--world',str(path),
                '--output-dir',self.output_var.get(),'--resolution',str(self.resolution_var.get()),
                '--height',str(self.height_var.get()),'--seed-x',str(seed.get('x',0)),
                '--seed-y',str(seed.get('y',0))]

    def generate(self):
        try:self.app.start_bg_process(self.generation_command(),'world_maps')
        except (KeyError,ValueError,tk.TclError) as exc:
            messagebox.showerror('Generate occupancy grid',str(exc))

    def open_guide(self):
        messagebox.showinfo('World and robot extensions',(WORKSPACE_ROOT/'docs/ASSET_EXTENSION_GUIDE.md').read_text())


class AssetsTab(LabTab):
    """Installed extensions open through the same Launch controls as old assets."""
    def __init__(self, notebook, app):
        super().__init__(notebook, app, 'Installed Extensions')
        ttk.Label(self, text='Downloaded extension robots and worlds use the normal Launch selectors and run command.\n'
            'Select an installed entry below to open it in Launch. Modes follow its implemented model/controller support.',
            wraplength=880, justify='left').grid(row=0, column=0, sticky='w', pady=6)
        self.tree = ttk.Treeview(self, columns=('source', 'status'), show='tree headings', height=15)
        for column, title in [('#0', 'Robot / world'), ('source', 'Source pin'), ('status', 'Integration')]:
            self.tree.heading(column, text=title)
        self.tree.column('#0', width=450); self.tree.column('source', width=180); self.tree.column('status', width=230)
        self.tree.grid(row=1, column=0, sticky='nsew'); self.rowconfigure(1, weight=1)
        scrollbar = ttk.Scrollbar(self, orient='vertical', command=self.tree.yview)
        scrollbar.grid(row=1, column=1, sticky='ns'); self.tree.configure(yscrollcommand=scrollbar.set)
        actions = ttk.Frame(self); actions.grid(row=2, column=0, sticky='ew', pady=6)
        ttk.Button(actions, text='Open Selected in Launch', command=self.open_selected).pack(side='left')
        ttk.Button(actions, text='Refresh Installed Assets', command=self.refresh_local_status).pack(side='left', padx=6)
        ttk.Button(actions, text='Open Upstream Source', command=self.open_source).pack(side='left')
        self.detail_var = tk.StringVar()
        ttk.Label(self, textvariable=self.detail_var, wraplength=880, justify='left').grid(row=3, column=0, sticky='w')
        self.tree.bind('<<TreeviewSelect>>', self.show_details)
        self.tree.bind('<Double-1>', lambda _event: self.open_selected())
        self.refresh_local_status()

    def refresh_local_status(self):
        from robot_lab_utils.installed_assets import installation_root
        self.app._refresh_maps()
        self.tree.delete(*self.tree.get_children()); self.rows = {}; self.links = {}
        path = installation_root() / 'integration-report.json'
        report = json.loads(path.read_text()) if path.is_file() else {}
        catalog = load_yaml(WORKSPACE_ROOT / 'docs/status/asset-sources-2026-10-05.yaml') or {}
        for source in catalog.get('sources', []):
            parent = self.tree.insert('', 'end', text=source['id'], values=(source['revision'][:12], ''))
            self.links[parent] = source['repository']
            entries = {item['entry']: item for item in report.get('entries', []) if item['source_id'] == source['id']}
            for entry in source.get('entries', []):
                name = entry.get('name') if isinstance(entry, dict) else entry
                item = entries.get(name, {})
                profiles = item.get('profiles', [])
                if not profiles and isinstance(entry,dict):
                    profiles = [{'id':robot, 'kind':'robot', 'support':'Installed equivalent model',
                                 'notes':self.app.robot_profiles[robot].get('notes','')}
                                for robot in entry.get('installed_profiles',[])
                                if robot in self.app.robot_profiles]
                if profiles:
                    for profile in profiles:
                        row = self.tree.insert(parent, 'end', text=profile['id'], values=(source['revision'][:12], profile.get('support', 'installed')))
                        self.rows[row] = profile
                        self.links[row] = source['repository']
                else:
                    row = self.tree.insert(parent, 'end', text=name, values=(source['revision'][:12], item.get('status', 'integration pending')))
                    self.rows[row] = {'reason': item.get('reason', 'Source model integration is pending'), 'id': name}
                    self.links[row] = entry['source'] if isinstance(entry, dict) else source['repository']

    def show_details(self, _event=None):
        selected = self.tree.selection()
        profile = self.rows.get(selected[0], {}) if selected else {}
        self.detail_var.set(profile.get('reason') or profile.get('notes', 'Select a robot or world to see its available modes.'))

    def open_selected(self):
        selected = self.tree.selection()
        profile = self.rows.get(selected[0], {}) if selected else {}
        kind = profile.get('kind')
        choices = self.app.robot_profiles if kind == 'robot' else self.app.map_profiles
        if kind not in ('robot', 'world') or profile.get('id') not in choices:
            self.show_details()
            return
        if kind == 'robot': self.app.robot_var.set(profile['id'])
        else: self.app.map_var.set(profile['id'])
        self.app.mode_var.set('display')
        self.app._update_from_selection()
        self.app.show_tab('Launch')
        self.app.set_status('Selected ' + profile['id'] + '; run command filled')

    def open_source(self):
        selected = self.tree.selection()
        if selected: webbrowser.open(self.links[selected[0]])


ENTITY_TYPES = [
    ("robots", "Robots", ["id", "robot_class", "status", "name"]),
    ("environments", "Environments", ["id", "dimension", "simulator", "name"]),
    ("algorithms", "Algorithms", ["id", "category", "family", "name"]),
    ("scenarios", "Scenarios", ["id", "task_type", "status", "name"]),
    ("experiments", "Experiments", ["id", "status", "name"]),
]


class RegistryTab(LabTab):
    """Browse the Robot Lab registry with search and a details pane."""

    def __init__(self, notebook, app):
        super().__init__(notebook, app, "Registry")
        self.entities = {}
        self._build_ui()
        self._load_all()

    def _build_ui(self):
        top = ttk.Frame(self)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(3, weight=1)

        ttk.Label(top, text="Type").grid(row=0, column=0, sticky="w", padx=(0, 4))
        self.type_var = tk.StringVar(value=ENTITY_TYPES[0][1])
        type_combo = ttk.Combobox(
            top,
            textvariable=self.type_var,
            values=[label for _, label, _ in ENTITY_TYPES],
            state="readonly",
            width=16,
        )
        type_combo.grid(row=0, column=1, sticky="w", padx=(0, 12))
        type_combo.bind("<<ComboboxSelected>>", lambda _e: self._refresh_tree())

        ttk.Label(top, text="Search").grid(row=0, column=2, sticky="w", padx=(0, 4))
        self.search_var = tk.StringVar()
        search_entry = ttk.Entry(top, textvariable=self.search_var, width=30)
        search_entry.grid(row=0, column=3, sticky="ew", padx=(0, 12))
        search_entry.bind("<KeyRelease>", lambda _e: self._refresh_tree())

        ttk.Button(top, text="Reload", command=self._load_all).grid(row=0, column=4)

        body = ttk.Frame(self)
        body.grid(row=1, column=0, sticky="nsew", pady=(8, 0))
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=2)
        body.rowconfigure(0, weight=1)

        tree_frame = ttk.Frame(body)
        tree_frame.grid(row=0, column=0, sticky="nsew")
        tree_frame.columnconfigure(0, weight=1)
        tree_frame.rowconfigure(0, weight=1)

        self.tree = ttk.Treeview(tree_frame, columns=("a", "b", "c", "d"), show="headings")
        for index, heading in enumerate(("ID", "Class/Category", "Status", "Name")):
            self.tree.heading(index, text=heading)
        self.tree.column("a", width=200)
        self.tree.column("b", width=120)
        self.tree.column("c", width=90)
        self.tree.column("d", width=220)
        self.tree.grid(row=0, column=0, sticky="nsew")

        vsb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.tree.yview)
        vsb.grid(row=0, column=1, sticky="ns")
        self.tree.configure(yscrollcommand=vsb.set)
        self.tree.bind("<<TreeviewSelect>>", lambda _e: self._show_details())

        detail_frame = ttk.LabelFrame(body, text="Details", padding=6)
        detail_frame.grid(row=0, column=1, sticky="nsew", padx=(8, 0))
        detail_frame.columnconfigure(0, weight=1)
        detail_frame.rowconfigure(0, weight=1)
        self.details = scrolledtext.ScrolledText(detail_frame, wrap="word", width=50)
        self.details.grid(row=0, column=0, sticky="nsew")
        self.details.configure(state="disabled")

        self.count_var = tk.StringVar(value="")
        ttk.Label(self, textvariable=self.count_var, foreground="#555555").grid(
            row=2, column=0, sticky="w", pady=(6, 0)
        )

    def _load_all(self):
        self.entities = {}
        if not REGISTRY_CONFIG_DIR.is_dir():
            self.app.log(f"[registry] config dir not found: {REGISTRY_CONFIG_DIR}\n")
            return
        for entity_type, _label, _cols in ENTITY_TYPES:
            path = REGISTRY_CONFIG_DIR / f"{entity_type}.yaml"
            if not path.exists():
                continue
            try:
                data = load_yaml(str(path))
            except Exception as exc:  # noqa: BLE001
                self.app.log(f"[registry] failed to load {path.name}: {exc}\n")
                continue
            if isinstance(data, dict):
                data = next(iter(data.values()), [])
            if isinstance(data, list):
                self.entities[entity_type] = {
                    item.get("id", f"unnamed_{index}"): item
                    for index, item in enumerate(data)
                    if isinstance(item, dict)
                }
        from robot_lab_registry.catalog import Registry
        registry = Registry(REGISTRY_CONFIG_DIR)
        registry.load()
        self.entities['robots'] = registry.robots.get_all()
        self.entities['environments'] = registry.environments.get_all()
        self._refresh_tree()

    def _current_type(self):
        value = self.type_var.get()
        for name, label, _cols in ENTITY_TYPES:
            if label == value:
                return name
        return value

    def _current_columns(self):
        entity_type = self._current_type()
        for name, _label, cols in ENTITY_TYPES:
            if name == entity_type:
                return cols
        return ["id", "status", "name"]

    def _refresh_tree(self):
        entity_type = self._current_type()
        columns = self._current_columns()
        for index, column in enumerate(columns):
            self.tree.heading(index, text=column.replace("_", " ").title())

        query = self.search_var.get().strip().lower()
        items = self.entities.get(entity_type, {})
        self.tree.delete(*self.tree.get_children())

        shown = 0
        for entity_id, entity in sorted(items.items()):
            haystack = json.dumps(entity, default=str).lower()
            if query and query not in haystack and query not in entity_id.lower():
                continue
            values = [str(entity.get(column, "")) for column in columns]
            self.tree.insert("", "end", iid=entity_id, values=values)
            shown += 1

        self.count_var.set(f"{shown} of {len(items)} {entity_type} shown")

    def _show_details(self):
        selection = self.tree.selection()
        entity_type = self._current_type()
        entity = self.entities.get(entity_type, {}).get(
            selection[0] if selection else "", {}
        )
        self.details.configure(state="normal")
        self.details.delete("1.0", "end")
        if entity:
            try:
                import yaml  # noqa: PLC0415

                text = yaml.safe_dump(entity, sort_keys=False, default_flow_style=False)
            except ImportError:
                text = json.dumps(entity, indent=2, default=str)
            self.details.insert("1.0", text)
        self.details.configure(state="disabled")


class VacuumTab(LabTab):
    """Vacuum cleaning mission control: launch the room-vacuum simulation,
    run the vacuum_cleaner node, and manage the cleaning run."""

    def __init__(self, notebook, app):
        super().__init__(notebook, app, "Vacuum")
        self._build_ui()

    def _build_ui(self):
        info = ttk.LabelFrame(self, text="Vacuum Cleaning", padding=10)
        info.grid(row=0, column=0, sticky="ew")
        info.columnconfigure(0, weight=1)
        ttk.Label(
            info,
            text=(
                "Room vacuum coverage for any robot whose launch profile declares "
                "supports_room_vacuum: true (robot-agnostic: driven by "
                "src/robot_lab_robots/config/robots.yaml, no per-robot package needed).\n"
                "The room-vacuum launch brings up Gazebo + the vacuum room world; "
                "the cleaner node drives systematic coverage."
            ),
            wraplength=640,
            justify="left",
        ).grid(row=0, column=0, sticky="ew")

        robot_frame = ttk.LabelFrame(self, text="Robot (vacuum-capable profiles)", padding=10)
        robot_frame.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        robot_frame.columnconfigure(0, weight=1)
        robot_frame.columnconfigure(1, weight=0)
        capable = sorted(
            robot_id
            for robot_id, config in self.app.robot_profiles.items()
            if bool(config.get("supports_room_vacuum", False))
        )
        self.vacuum_robot_var = tk.StringVar(
            value=self.app.robot_var.get() if self.app.robot_var.get() in capable
            else (capable[0] if capable else "")
        )
        robot_combo = ttk.Combobox(
            robot_frame,
            textvariable=self.vacuum_robot_var,
            values=capable,
            state="readonly",
        )
        robot_combo.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        robot_combo.bind("<<ComboboxSelected>>", self._on_robot_selected)
        ttk.Button(
            robot_frame,
            text="Open Launch Tab (drive pad)",
            command=lambda: self.app.show_tab("Launch"),
        ).grid(row=0, column=1, sticky="ew")

        launch_frame = ttk.LabelFrame(self, text="Simulation", padding=10)
        launch_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        launch_frame.columnconfigure(0, weight=1)
        launch_frame.columnconfigure(1, weight=1)
        ttk.Button(
            launch_frame,
            text="Launch Room Vacuum Simulation",
            command=self._launch_vacuum_sim,
        ).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(
            launch_frame, text="Stop Simulation", command=self.app.stop_launch
        ).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        cleaner_frame = ttk.LabelFrame(self, text="Cleaner Node", padding=10)
        cleaner_frame.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        cleaner_frame.columnconfigure(0, weight=1)
        cleaner_frame.columnconfigure(1, weight=1)
        ttk.Button(
            cleaner_frame, text="Start Cleaner Node", command=self._start_cleaner
        ).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(
            cleaner_frame, text="Stop Cleaner Node", command=self._stop_cleaner
        ).grid(row=0, column=1, sticky="ew", padx=(4, 0))

        manual_frame = ttk.LabelFrame(self, text="Manual Cleaning Drive", padding=10)
        manual_frame.grid(row=4, column=0, sticky="nsew", pady=(10, 0))
        ttk.Label(
            manual_frame,
            text=(
                "Use the drive pad on the Launch tab to steer the robot while the "
                "vacuum simulation is running (teleop publishes on /key_vel). "
                "Switch to SLAM mode first if you want to build the coverage map, "
                "then save it with the Save Map button and switch to Navigation."
            ),
            wraplength=680,
            justify="left",
        ).grid(row=0, column=0, sticky="ew")

        self.rowconfigure(4, weight=1)

    def _on_robot_selected(self, _event):
        """Sync the Vacuum robot selection back into the Launch tab."""
        self.app.robot_var.set(self.vacuum_robot_var.get())
        self.app._update_from_selection()
        self.app.set_status(f"Vacuum robot: {self.vacuum_robot_var.get()}")

    def _launch_vacuum_sim(self):
        command = [
            "ros2",
            "launch",
            "robot_lab_bringup",
            "simulated_room_vacuum.launch.py",
            f"robot_model:={self.vacuum_robot_var.get()}",
        ]
        self.app.start_launch_command(command)

    def _start_cleaner(self):
        self.app.start_bg_process(
            ["ros2", "run", "robot_lab_vacuum_cleaning", "vacuum_cleaner"],
            "vacuum_cleaner",
        )
        self.app.set_status("Vacuum cleaner node running")

    def _stop_cleaner(self):
        self.app.stop_bg_process("vacuum_cleaner")
        self.app.set_status("Vacuum cleaner node stopped")


class BenchmarkTab(LabTab):
    """Seeded benchmark runs via the robot_lab_benchmark LaunchOrchestrator,
    plus regression checks against the checked-in reference results."""

    def __init__(self, notebook, app):
        super().__init__(notebook, app, "Benchmark")
        self.last_manifest = None
        self._build_ui()
        self._load_choices()

    def _build_ui(self):
        config = ttk.LabelFrame(self, text="Benchmark Run", padding=10)
        config.grid(row=0, column=0, sticky="ew")
        for column in (1, 3):
            config.columnconfigure(column, weight=1)

        ttk.Label(config, text="Robot").grid(row=0, column=0, sticky="w", padx=(0, 4))
        self.robot_var = tk.StringVar()
        self.robot_combo = ttk.Combobox(
            config, textvariable=self.robot_var, state="readonly", width=24
        )
        self.robot_combo.grid(row=0, column=1, sticky="ew", pady=2)

        ttk.Label(config, text="Environment").grid(row=0, column=2, sticky="w", padx=(12, 4))
        self.env_var = tk.StringVar()
        self.env_combo = ttk.Combobox(
            config, textvariable=self.env_var, state="readonly", width=24
        )
        self.env_combo.grid(row=0, column=3, sticky="ew", pady=2)

        ttk.Label(config, text="Scenario").grid(row=1, column=0, sticky="w", padx=(0, 4))
        self.scenario_var = tk.StringVar()
        self.scenario_combo = ttk.Combobox(
            config, textvariable=self.scenario_var, state="readonly", width=24
        )
        self.scenario_combo.grid(row=1, column=1, sticky="ew", pady=2)

        ttk.Label(config, text="Seed").grid(row=1, column=2, sticky="w", padx=(12, 4))
        self.seed_var = tk.IntVar(value=42)
        ttk.Spinbox(
            config, from_=0, to=99999, increment=1,
            textvariable=self.seed_var, width=10,
        ).grid(row=1, column=3, sticky="w", pady=2)

        ttk.Label(config, text="Duration (s)").grid(row=2, column=0, sticky="w", padx=(0, 4))
        self.duration_var = tk.DoubleVar(value=30.0)
        ttk.Spinbox(
            config, from_=5.0, to=600.0, increment=5.0,
            textvariable=self.duration_var, width=10,
        ).grid(row=2, column=1, sticky="w", pady=2)

        self.bag_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            config, text="Record rosbag", variable=self.bag_var
        ).grid(row=2, column=2, sticky="w", padx=(12, 4))

        ttk.Label(config, text="Output dir").grid(row=3, column=0, sticky="w", padx=(0, 4))
        self.outdir_var = tk.StringVar(value=str(WORKSPACE_ROOT / "benchmark_runs"))
        ttk.Entry(config, textvariable=self.outdir_var).grid(
            row=3, column=1, columnspan=3, sticky="ew", pady=2
        )

        buttons = ttk.Frame(self)
        buttons.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        for column in range(3):
            buttons.columnconfigure(column, weight=1)
        ttk.Button(
            buttons, text="Run Benchmark", command=self._run_benchmark
        ).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(
            buttons, text="Check Regression vs Reference",
            command=self._check_regression,
        ).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(
            buttons, text="Stop All Background",
            command=self._stop_all,
        ).grid(row=0, column=2, sticky="ew", padx=(4, 0))

        results_frame = ttk.LabelFrame(self, text="Latest Run Summary", padding=6)
        results_frame.grid(row=2, column=0, sticky="nsew", pady=(10, 0))
        results_frame.columnconfigure(0, weight=1)
        results_frame.rowconfigure(0, weight=1)
        self.results = scrolledtext.ScrolledText(results_frame, wrap="word", height=14)
        self.results.grid(row=0, column=0, sticky="nsew")
        self.results.configure(state="disabled")
        self.rowconfigure(2, weight=1)

    def _load_choices(self):
        def ids(filename):
            path = REGISTRY_CONFIG_DIR / filename
            if not path.exists():
                return []
            try:
                data = load_yaml(str(path))
            except Exception:  # noqa: BLE001
                return []
            if isinstance(data, dict):
                data = next(iter(data.values()), [])
            return sorted(
                item.get("id", "") for item in (data or []) if isinstance(item, dict)
            )

        for combo, filename, default in (
            (self.robot_combo, "robots.yaml", "bumperbot"),
            (self.env_combo, "environments.yaml", None),
            (self.scenario_combo, "scenarios.yaml", None),
        ):
            values = ids(filename) or [default or "default"]
            combo.configure(values=values)
            if default and default in values:
                combo.set(default)
            else:
                combo.current(0)

    def _run_benchmark(self):
        robot_id = self.robot_var.get()
        environment_id = self.env_var.get()
        scenario_id = self.scenario_var.get()
        seed = int(self.seed_var.get())
        duration = float(self.duration_var.get())
        bag = bool(self.bag_var.get())
        output_dir = self.outdir_var.get()

        self.app.log(
            f"\n[benchmark] {robot_id} / {environment_id} / {scenario_id} "
            f"seed={seed} duration={duration}s bag={bag}\n"
        )
        self.app.set_status("Benchmark running...")
        threading.Thread(
            target=self._benchmark_worker,
            args=(robot_id, environment_id, scenario_id, seed, duration, bag, output_dir),
            daemon=True,
        ).start()

    def _benchmark_worker(self, robot_id, environment_id, scenario_id, seed, duration, bag, output_dir):
        import sys

        added = str(BENCHMARK_SRC) not in sys.path
        if added:
            sys.path.insert(0, str(BENCHMARK_SRC))
        try:
            from robot_lab_benchmark.launch_orchestrator import LaunchOrchestrator

            orchestrator = LaunchOrchestrator(output_dir=output_dir)
            summary = orchestrator.execute_full_run(
                robot_id=robot_id,
                environment_id=environment_id,
                scenario_id=scenario_id,
                seed=seed,
                duration_sec=duration,
                bag_capture=bag,
            )
            self.last_manifest = summary
            text = json.dumps(summary, indent=2, default=str)
            for line in text.splitlines():
                self.app.log(line + "\n")
            self.app.log(
                f"[benchmark] manifest written to {summary.get('manifest_path')}\n"
            )
            self.app.set_status("Benchmark finished")
            self.app.after(0, self._show_summary, text)
        except Exception as exc:  # noqa: BLE001
            self.app.log(f"[benchmark] failed: {exc}\n")
            self.app.set_status("Benchmark failed")

    def _show_summary(self, text):
        self.results.configure(state="normal")
        self.results.delete("1.0", "end")
        self.results.insert("1.0", text)
        self.results.configure(state="disabled")

    def _check_regression(self):
        if not self.last_manifest:
            messagebox.showinfo(
                "Regression check",
                "Run a benchmark first - the check compares the latest run "
                "against the reference results.",
            )
            return
        threading.Thread(target=self._regression_worker, daemon=True).start()

    def _regression_worker(self):
        import sys

        if str(BENCHMARK_SRC) not in sys.path:
            sys.path.insert(0, str(BENCHMARK_SRC))
        try:
            from robot_lab_benchmark.reference import ReferenceRegistry

            registry_path = (
                BENCHMARK_SRC / "robot_lab_benchmark" / "reference_data" / "results.json"
            )
            registry = ReferenceRegistry(registry_path=registry_path)
            registry.load()
            report = registry.check_all_regressions([self.last_manifest])
            self.app.log(
                "\n[regression] " + json.dumps(report, indent=2, default=str) + "\n"
            )
            self.app.set_status("Regression check done")
        except Exception as exc:  # noqa: BLE001
            self.app.log(f"[regression] failed: {exc}\n")

    def _stop_all(self):
        self.app.stop_all_bg()
        self.app.set_status("Background processes stopped")


class TestsTab(LabTab):
    """Run the workspace test suites and see the output live."""

    def __init__(self, notebook, app):
        super().__init__(notebook, app, "Tests")
        self._build_ui()

    def _build_ui(self):
        frame = ttk.LabelFrame(self, text="Test Suites", padding=10)
        frame.grid(row=0, column=0, sticky="ew")
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)

        ttk.Button(
            frame, text="Fast Suite (P5 + P6 logic)",
            command=lambda: self.app.start_bg_process(
                ["bash", str(WORKSPACE_ROOT / "scripts" / "test_fast.sh")],
                "tests",
            ),
        ).grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=2)
        ttk.Button(
            frame, text="Full Suite (all tests)",
            command=self._run_full_suite,
        ).grid(row=0, column=1, sticky="ew", padx=(4, 0), pady=2)
        ttk.Button(
            frame, text="Registry Validation",
            command=self._run_registry_validation,
        ).grid(row=1, column=0, sticky="ew", padx=(0, 4), pady=2)
        ttk.Button(
            frame, text="Compile Algorithm Modules",
            command=self._run_compile,
        ).grid(row=1, column=1, sticky="ew", padx=(4, 0), pady=2)
        ttk.Button(
            frame, text="Stop Test Run",
            command=lambda: self.app.stop_bg_process("tests"),
        ).grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 2))

        info = ttk.Label(
            self,
            text=(
                "Fast suite runs in under a minute (no simulation needed). "
                "The full suite includes launch-orchestration tests that take "
                "longer and need the workspace sourced. Output is streamed to "
                "the shared console."
            ),
            wraplength=680,
            justify="left",
            foreground="#a6adc8" if THEME_AVAILABLE else "#555555",
        )
        info.grid(row=1, column=0, sticky="ew", pady=(10, 0))

    def _run_full_suite(self):
        command = (
            f"cd {WORKSPACE_ROOT} && "
            "PYTHONPATH=src/robot_lab/robot_lab_registry:"
            "src/robot_lab/robot_lab_benchmark:src/robot_lab_algorithms "
            "python3 -m unittest discover -s src/robot_lab/robot_lab_registry/test "
            "-p 'test_p*.py' -v"
        )
        self.app.start_bg_process(["bash", "-c", command], "tests")

    def _run_registry_validation(self):
        command = (
            f"cd {WORKSPACE_ROOT} && "
            "PYTHONPATH=src/robot_lab/robot_lab_registry python3 -c \""
            "from robot_lab_registry.catalog import Registry; "
            "reg = Registry('src/robot_lab/robot_lab_registry/config'); "
            "reg.load(); "
            "print('robots:', reg.robots.count()); "
            "print('environments:', reg.environments.count()); "
            "print('algorithms:', reg.algorithms.count()); "
            "print('scenarios:', reg.scenarios.count()); "
            "print('experiments:', reg.experiments.count())\""
        )
        self.app.start_bg_process(["bash", "-c", command], "tests")

    def _run_compile(self):
        command = (
            f"cd {WORKSPACE_ROOT} && for f in perception localization "
            "state_estimation sensor_fusion global_planning local_planning; do "
            "python3 -m py_compile "
            "src/robot_lab_algorithms/robot_lab_algorithms/$f.py && "
            "echo \"$f OK\"; done"
        )
        self.app.start_bg_process(["bash", "-c", command], "tests")


class HealthTab(LabTab):
    """Workspace health: doctor diagnostics, platform status and ROS graph."""

    def __init__(self, notebook, app):
        super().__init__(notebook, app, "Health")
        self._build_ui()

    def _build_ui(self):
        frame = ttk.LabelFrame(self, text="Diagnostics", padding=10)
        frame.grid(row=0, column=0, sticky="ew")
        frame.columnconfigure(0, weight=1)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=1)

        ttk.Button(
            frame, text="Run Doctor",
            command=lambda: self.app.start_bg_process(
                ["bash", str(WORKSPACE_ROOT / "scripts" / "doctor.sh")],
                "health",
            ),
        ).grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=2)
        ttk.Button(
            frame, text="Platform Status", command=self._show_platform_status
        ).grid(row=0, column=1, sticky="ew", padx=4, pady=2)
        ttk.Button(
            frame, text="ROS Nodes", command=self._ros_nodes
        ).grid(row=0, column=2, sticky="ew", padx=(4, 0), pady=2)
        ttk.Button(
            frame, text="ROS Topics", command=self._ros_topics
        ).grid(row=1, column=0, sticky="ew", padx=(0, 4), pady=2)
        ttk.Button(
            frame, text="ROS Packages", command=self._ros_packages
        ).grid(row=1, column=1, sticky="ew", padx=4, pady=2)
        ttk.Button(
            frame, text="Stop Diagnostics",
            command=lambda: self.app.stop_bg_process("health"),
        ).grid(row=1, column=2, sticky="ew", padx=(4, 0), pady=2)
        ttk.Button(
            frame, text="Patch Execution Guide", command=self._show_patch_guide,
        ).grid(row=2, column=0, columnspan=3, sticky="ew", pady=(6, 2))

        ttk.Button(
            frame, text="Completion Audit", command=self._show_completion_audit,
        ).grid(row=3, column=0, sticky="ew", padx=(0, 4), pady=2)
        ttk.Button(
            frame, text="Storage Guide", command=self._show_storage_guide,
        ).grid(row=3, column=1, columnspan=2, sticky="ew", padx=(4, 0), pady=2)
        ttk.Button(frame, text="PX4 Flight Guide", command=lambda: self._show_document(
            WORKSPACE_ROOT / 'docs/tutorials/px4_x500.md')).grid(
                row=4, column=0, sticky='ew', padx=(0, 4), pady=2)
        ttk.Button(frame, text="Verified Robot Trials", command=lambda: self._show_document(
            WORKSPACE_ROOT / 'docs/status/continuation-2026-10-05.md')).grid(
                row=4, column=1, columnspan=2, sticky='ew', padx=(4, 0), pady=2)

        self.summary = scrolledtext.ScrolledText(self, wrap="word", height=16)
        self.summary.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        self.summary.configure(state="disabled")
        self.rowconfigure(1, weight=1)
        self._show_platform_status()

    def _show_platform_status(self):
        if not STATUS_YAML.exists():
            self.app.log(f"[health] status file missing: {STATUS_YAML}\n")
            return
        try:
            data = load_yaml(str(STATUS_YAML))
        except Exception as exc:  # noqa: BLE001
            self.app.log(f"[health] failed to parse status yaml: {exc}\n")
            return
        self._set_summary(format_platform_status(data))

    def _show_patch_guide(self):
        path = WORKSPACE_ROOT / "docs" / "PATCH_EXECUTION_GUIDE.md"
        try:
            self._set_summary(path.read_text(encoding="utf-8"))
        except OSError as exc:
            self.app.log(f"[health] failed to read patch guide: {exc}\n")

    def _show_completion_audit(self):
        data = load_yaml(str(STATUS_YAML))
        report = data.get("completion_audit", {}).get("report")
        if report:
            self._show_document(WORKSPACE_ROOT / report)
        else:
            self._set_summary("No current completion audit is recorded.\n")

    def _show_storage_guide(self):
        self._show_document(WORKSPACE_ROOT / "docs" / "STORAGE.md")

    def _show_document(self, path):
        try:
            self._set_summary(path.read_text(encoding="utf-8"))
        except OSError as exc:
            self.app.log(f"[health] failed to read {path.name}: {exc}\n")

    def _set_summary(self, text):
        self.summary.configure(state="normal")
        self.summary.delete("1.0", "end")
        self.summary.insert("1.0", text)
        self.summary.configure(state="disabled")

    def _ros_nodes(self):
        self.app.start_bg_process(
            ["bash", "-c",
             "source /opt/ros/humble/setup.bash && timeout 5 ros2 node list"],
            "health",
        )

    def _ros_topics(self):
        self.app.start_bg_process(
            ["bash", "-c",
             "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic list"],
            "health",
        )

    def _ros_packages(self):
        self.app.start_bg_process(
            ["bash", "-c",
             "source /opt/ros/humble/setup.bash && ros2 pkg list | "
             "grep -E 'bumperbot|labbot|robot_lab|vacuum|sim_launcher'"],
            "health",
        )


def create_tabs(notebook, app):
    """Instantiate all control-center tabs and return them."""
    tabs = [
        RegistryTab(notebook, app),
        WorldsTab(notebook, app),
        AssetsTab(notebook, app),
        VacuumTab(notebook, app),
        BenchmarkTab(notebook, app),
        TestsTab(notebook, app),
        HealthTab(notebook, app),
    ]
    if HAS_LIVE_MONITOR:
        tabs.append(LiveMonitorTab(notebook, app))
    return tabs
