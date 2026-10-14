"""Native shared-world session editor, per-member controls and ground swarms."""
import json
import math
import os
from pathlib import Path
import shlex
import time
import uuid

import tkinter as tk
from tkinter import filedialog, scrolledtext, ttk
import yaml

from .drive_control import LinuxJoystick, RampDrive
from robot_lab_utils.fleet_session import MEMBER_MODELS, validate_session

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
except ImportError:
    rclpy = None


def session_command(path, map_name, navigation=False, gui=True, rviz=True):
    return ['ros2', 'launch', 'robot_lab_bringup', 'shared_world.launch.py',
            'session:=' + str(path), 'map_name:=' + map_name, 'simulator:=gazebo',
            'navigation:=' + str(bool(navigation)).lower(), 'gui:=' + str(bool(gui)).lower(),
            'rviz:=' + str(bool(rviz)).lower()]


class FleetTab(ttk.Frame):
    def __init__(self, notebook, app):
        super().__init__(notebook, padding=10)
        self.app, self.notebook = app, notebook
        notebook.add(self, text='Fleet & Swarm')
        self.columnconfigure(0, weight=3)
        self.columnconfigure(1, weight=2)
        self.rowconfigure(2, weight=1)
        self.operator_id = 'gui-' + uuid.uuid4().hex
        self.node = self.executor = self.command_pub = self.heartbeat_pub = None
        self.publishers = {}
        self.runtime = None
        self.runtime_at = -math.inf
        self.last_heartbeat = self.last_drive = 0.
        self.armed = []
        self.latches, self.keys, self.key_jobs = set(), set(), {}
        self.ramp, self.joystick = RampDrive(max_linear=.35, min_linear=-.35, max_angular=.8, min_angular=-.8), LinuxJoystick()
        self.driving = False
        self._closed = False
        self.members = [dict(id='alpha', model='bumperbot'), dict(id='beta', model='labbot')]
        base = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'gui_profiles'
        base.mkdir(parents=True, exist_ok=True)
        self.session_file = base / ('fleet-' + self.operator_id + '.yaml')
        selected = app.map_var.get()
        self.map_var = tk.StringVar(value=selected if selected in app.map_profiles else 'nav_empty')
        self.map_family = tk.StringVar(value=app.map_groups.family(self.map_var.get()))
        self.navigation, self.gui, self.rviz = tk.BooleanVar(value=False), tk.BooleanVar(value=True), tk.BooleanVar(value=True)
        self.status = tk.StringVar(value='Experimental Gazebo Bumperbot/Labbot shared-world control. Connect starts neutral.')
        ttk.Label(self, text='Shared world · one simulation · separate robot instances', font=('', 12, 'bold')).grid(
            row=0, column=0, columnspan=2, sticky='w')
        ttk.Label(self, text='Gazebo adapter implemented. Other backends, Open-RMF traffic, aerial swarms, marine and gait extensions remain pending.',
                  wraplength=980).grid(row=1, column=0, columnspan=2, sticky='ew', pady=(3, 8))
        left = ttk.LabelFrame(self, text='Session members and map', padding=8)
        left.grid(row=2, column=0, sticky='nsew', padx=(0, 8))
        left.columnconfigure(1, weight=1)
        left.rowconfigure(3, weight=1)
        ttk.Label(left, text='Map family').grid(row=0, column=0, sticky='w')
        families = ttk.Combobox(left, textvariable=self.map_family, values=app.map_groups.choices(), state='readonly')
        families.grid(row=0, column=1, sticky='ew')
        families.bind('<<ComboboxSelected>>', self._map_family_selected)
        ttk.Label(left, text='Map variant').grid(row=1, column=0, sticky='w')
        self.map_combo = ttk.Combobox(left, textvariable=self.map_var,
            values=app.map_groups.members(self.map_family.get(), selectable=True), state='readonly')
        self.map_combo.grid(row=1, column=1, sticky='ew', pady=3)
        options = ttk.Frame(left)
        options.grid(row=2, column=0, columnspan=2, sticky='w')
        ttk.Checkbutton(options, text='Per-robot AMCL + Nav2', variable=self.navigation, command=self._refresh_command).pack(side='left')
        ttk.Checkbutton(options, text='Gazebo window', variable=self.gui, command=self._refresh_command).pack(side='left')
        ttk.Checkbutton(options, text='RViz', variable=self.rviz, command=self._refresh_command).pack(side='left')
        self.tree = ttk.Treeview(left, columns=('model', 'x', 'y', 'z', 'yaw'), height=6)
        self.tree.heading('#0', text='Instance id')
        self.tree.column('#0', width=110, stretch=False)
        for column in self.tree['columns']:
            self.tree.heading(column, text=column.title())
            self.tree.column(column, width=70 if column != 'model' else 100)
        self.tree.grid(row=3, column=0, columnspan=2, sticky='nsew', pady=5)
        self.tree.bind('<<TreeviewSelect>>', self._edit_selected)
        self.member_id, self.member_model = tk.StringVar(value='gamma'), tk.StringVar(value='bumperbot')
        self.spawn = {key: tk.StringVar() for key in ('x', 'y', 'z', 'yaw')}
        editor = ttk.Frame(left)
        editor.grid(row=4, column=0, columnspan=2, sticky='ew')
        for col, (label, variable) in enumerate([('Id', self.member_id), ('Model', self.member_model), *[(k.upper(), v) for k, v in self.spawn.items()]]):
            ttk.Label(editor, text=label).grid(row=0, column=col, sticky='w')
            widget = ttk.Combobox(editor, textvariable=variable, values=MEMBER_MODELS, state='readonly', width=11) if label == 'Model' else ttk.Entry(editor, textvariable=variable, width=8)
            widget.grid(row=1, column=col, sticky='ew', padx=(0, 4))
            editor.columnconfigure(col, weight=1)
        buttons = ttk.Frame(left)
        buttons.grid(row=5, column=0, columnspan=2, sticky='w', pady=5)
        for label, callback in [('Add', self._add), ('Update selected', self._update), ('Remove', self._remove),
                                ('Load session', self._load), ('Save session', self._save)]:
            ttk.Button(buttons, text=label, command=callback).pack(side='left', padx=(0, 4))
        ttk.Label(left, text='Blank X/Y chooses a nearby free occupancy cell. Enter both for an explicit spawn.\n'
                  'Unknown cells, outside-map poses and overlapping members are refused. Maximum 8 members.',
                  wraplength=550).grid(row=6, column=0, columnspan=2, sticky='w')
        right = ttk.LabelFrame(self, text='Live member/group control', padding=8)
        right.grid(row=2, column=1, sticky='nsew')
        right.columnconfigure(1, weight=1)
        ttk.Button(right, text='Connect to shared session', command=self.connect).grid(row=0, column=0, columnspan=2, sticky='ew')
        self.target = tk.StringVar(value='alpha')
        ttk.Label(right, text='Target').grid(row=1, column=0, sticky='w')
        self.target_combo = ttk.Combobox(right, textvariable=self.target, values=('alpha', 'beta', 'All included'), state='readonly')
        self.target_combo.grid(row=1, column=1, sticky='ew', pady=5)
        self.target_combo.bind('<<ComboboxSelected>>', lambda _event: self.stop_manual())
        actions = ttk.Frame(right)
        actions.grid(row=2, column=0, columnspan=2, sticky='ew')
        ttk.Button(actions, text='Arm manual · neutral', command=self._arm).pack(side='left', fill='x', expand=True)
        ttk.Button(actions, text='ALL STOP', command=self.all_stop).pack(side='left', fill='x', expand=True, padx=(5, 0))
        control_pages = ttk.Notebook(right)
        control_pages.grid(row=3, column=0, columnspan=2, sticky='nsew', pady=5)
        right.rowconfigure(3, weight=1)
        drive_page, task_page, swarm_page = [ttk.Frame(control_pages, padding=5) for _ in range(3)]
        for page, label in ((drive_page, 'Drive'), (task_page, 'Tasks'), (swarm_page, 'Swarm')):
            control_pages.add(page, text=label)
            page.columnconfigure(0, weight=1)
        pad = ttk.Frame(drive_page)
        pad.grid(row=0, column=0, pady=5, sticky='ew')
        self.pad_buttons = {}
        for label, direction, row, col in [('↖', (1, 1), 0, 0), ('↑', (1, 0), 0, 1), ('↗', (1, -1), 0, 2),
                                           ('↶', (0, 1), 1, 0), ('↓', (-1, 0), 1, 1), ('↷', (0, -1), 1, 2)]:
            button = ttk.Button(pad, text=label, style='LatchedDrive.TButton', command=lambda d=direction: self._latch(d))
            button.grid(row=row, column=col, sticky='ew', padx=2, pady=2)
            self.pad_buttons[direction] = button
            for event in ('<ButtonRelease-1>', '<Leave>', '<Enter>', '<KeyRelease>', '<FocusOut>'):
                button.bind(event, lambda _event: self.after_idle(self._sync_buttons), add='+')
        ttk.Button(pad, text='Stop selected', command=self.stop_manual).grid(row=2, column=0, columnspan=3, sticky='ew', pady=3)
        for col in range(3):
            pad.columnconfigure(col, weight=1)
        self.keyboard = tk.BooleanVar(value=False)
        ttk.Checkbutton(drive_page, text='WASD + joystick · Space stops all', variable=self.keyboard, command=self._input_changed).grid(
            row=1, column=0, sticky='w')
        self.delta_linear, self.delta_angular = tk.DoubleVar(value=.02), tk.DoubleVar(value=.04)
        deltas = ttk.Frame(drive_page)
        deltas.grid(row=2, column=0, sticky='ew')
        ttk.Label(deltas, text='Δ linear / angular per 0.1 s').pack(side='left')
        for variable in (self.delta_linear, self.delta_angular):
            ttk.Entry(deltas, textvariable=variable, width=6).pack(side='left', padx=3)
        ttk.Label(drive_page, text='Click once to latch; click again to release and brake.\nArm/keyboard enable sends no movement.', wraplength=370).grid(
            row=3, column=0, sticky='w', pady=5)
        goal_frame = ttk.LabelFrame(task_page, text='Map goal / formation anchor', padding=5)
        goal_frame.grid(row=0, column=0, sticky='ew')
        self.goal = {key: tk.DoubleVar(value=0.) for key in ('x', 'y', 'yaw')}
        for col, (key, variable) in enumerate(self.goal.items()):
            ttk.Label(goal_frame, text=key.upper()).grid(row=0, column=col)
            ttk.Entry(goal_frame, textvariable=variable, width=9).grid(row=1, column=col, sticky='ew', padx=3)
            goal_frame.columnconfigure(col, weight=1)
        goal_buttons = ttk.Frame(task_page)
        goal_buttons.grid(row=1, column=0, sticky='ew', pady=4)
        for label, operation in [('Dispatch', 'dispatch'), ('Nearest idle', 'allocate'), ('Pause', 'pause'), ('Resume', 'resume'), ('Reset', 'reset')]:
            ttk.Button(goal_buttons, text=label, command=lambda op=operation: self._operation(op)).pack(side='left', expand=True, fill='x')
        ttk.Label(task_page, text='Dispatch selects one member. Nearest idle uses measured state.\n'
                  'Pause/cancel waits for the actual Nav2 result before replacement.\n'
                  'Reset restores only that entity; wheel odometry and world time continue.', wraplength=370).grid(row=2, column=0, sticky='w', pady=5)
        swarm_frame = ttk.LabelFrame(swarm_page, text='Ground swarm · experimental', padding=5)
        swarm_frame.grid(row=0, column=0, sticky='ew')
        self.swarm_mode, self.spacing = tk.StringVar(value='formation'), tk.DoubleVar(value=1.)
        ttk.Combobox(swarm_frame, textvariable=self.swarm_mode, values=('formation', 'rendezvous', 'dispersion', 'flocking'),
                     state='readonly', width=13).grid(row=0, column=0, sticky='ew')
        ttk.Label(swarm_frame, text='Spacing m').grid(row=0, column=1)
        ttk.Entry(swarm_frame, textvariable=self.spacing, width=6).grid(row=0, column=2)
        self.use_anchor = tk.BooleanVar(value=False)
        ttk.Checkbutton(swarm_frame, text='Use goal XY as anchor; yaw sets heading', variable=self.use_anchor).grid(row=1, column=0, columnspan=3, sticky='w')
        ttk.Button(swarm_frame, text='Start all included members', command=self._swarm).grid(row=2, column=0, columnspan=3, sticky='ew', pady=3)
        swarm_frame.columnconfigure(0, weight=1)
        membership = ttk.Frame(swarm_page)
        membership.grid(row=1, column=0, sticky='ew', pady=5)
        ttk.Button(membership, text='Leave group', command=lambda: self._operation('leave')).pack(side='left', fill='x', expand=True)
        ttk.Button(membership, text='Rejoin neutral', command=lambda: self._operation('join')).pack(side='left', fill='x', expand=True)
        ttk.Label(swarm_page, text='The physical robot stays in the shared world when it leaves.\n'
                  'Fresh state/scan and a live operator heartbeat are required.\n'
                  'These bounded ground kernels do not provide Open-RMF traffic scheduling.', wraplength=370).grid(row=2, column=0, sticky='w', pady=5)
        self.live = scrolledtext.ScrolledText(self, height=7, wrap='word', state='disabled')
        self.live.grid(row=3, column=0, columnspan=2, sticky='ew', pady=(8, 3))
        ttk.Label(self, textvariable=self.status, wraplength=1000).grid(row=4, column=0, columnspan=2, sticky='ew')
        command_frame = ttk.LabelFrame(self, text='Run command · automatically updated', padding=5)
        command_frame.grid(row=5, column=0, columnspan=2, sticky='ew', pady=5)
        command_frame.columnconfigure(0, weight=1)
        self.command_text = scrolledtext.ScrolledText(command_frame, height=3, wrap='word', state='disabled')
        self.command_text.grid(row=0, column=0, sticky='ew')
        command_buttons = ttk.Frame(command_frame)
        command_buttons.grid(row=0, column=1, sticky='ns', padx=5)
        for label, callback in [('Run shared world', self.run), ('Copy command', self.copy_command), ('Stop simulation', app.stop_launch)]:
            ttk.Button(command_buttons, text=label, command=callback).pack(fill='x', pady=2)
        self.map_var.trace_add('write', lambda *_args: self._refresh_command())
        app.bind('<KeyPress>', self._key_press, add='+')
        app.bind('<KeyRelease>', self._key_release, add='+')
        app.bind('<FocusOut>', self._focus_out, add='+')
        self._render_members()
        self._refresh_command()
        self._job = self.after(50, self._tick)

    def _map_family_selected(self, _event=None):
        choices = self.app.map_groups.members(self.map_family.get(), selectable=True)
        self.map_combo.configure(values=choices)
        if choices:
            self.map_var.set(choices[0])

    def _render_members(self):
        self.tree.delete(*self.tree.get_children())
        for member in self.members:
            pose = member.get('spawn') or {}
            self.tree.insert('', 'end', iid=member['id'], text=member['id'],
                             values=(member['model'], *[pose.get(key, 'auto') for key in ('x', 'y', 'z', 'yaw')]))

    def _edit_selected(self, _event=None):
        selected = self.tree.selection()
        if selected:
            member = next(m for m in self.members if m['id'] == selected[0])
            self.member_id.set(member['id'])
            self.member_model.set(member['model'])
            for key, variable in self.spawn.items():
                variable.set(str((member.get('spawn') or {}).get(key, '')))

    def _edited_member(self):
        result = dict(id=self.member_id.get().strip(), model=self.member_model.get())
        pose = {key: variable.get().strip() for key, variable in self.spawn.items() if variable.get().strip()}
        if pose:
            result['spawn'] = {key: float(value) for key, value in pose.items()}
        return result

    def _set_members(self, members):
        normalized = validate_session(dict(schema_version=1, map=self.map_var.get(), members=members))
        self.members = [{key: value for key, value in member.items() if value is not None} for member in normalized['members']]
        self._render_members()
        self._refresh_command()

    def _add(self):
        try:
            self._set_members([*self.members, self._edited_member()])
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))

    def _update(self):
        try:
            selected = self.tree.selection()
            if not selected:
                raise ValueError('Select a member to update')
            self._set_members([self._edited_member() if m['id'] == selected[0] else m for m in self.members])
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))

    def _remove(self):
        try:
            selected = set(self.tree.selection())
            self._set_members([m for m in self.members if m['id'] not in selected])
        except ValueError as exc:
            self.status.set(str(exc))

    def _load(self):
        path = filedialog.askopenfilename(title='Load shared-world session', filetypes=[('YAML session', '*.yaml *.yml')], initialdir=self.session_file.parent)
        if path:
            try:
                data = validate_session(yaml.safe_load(Path(path).read_text()))
                if data['map'] not in self.app.map_profiles:
                    raise ValueError('Session map is not installed')
                self.map_var.set(data['map'])
                self.map_family.set(self.app.map_groups.family(data['map']))
                self.map_combo.configure(values=self.app.map_groups.members(self.map_family.get(), selectable=True))
                self._set_members(data['members'])
            except (ValueError, OSError, yaml.YAMLError) as exc:
                self.status.set(str(exc))

    def _save(self):
        path = filedialog.asksaveasfilename(title='Save shared-world session', defaultextension='.yaml',
            filetypes=[('YAML session', '*.yaml')], initialdir=self.session_file.parent)
        if path:
            try:
                Path(path).write_text(self.session_file.read_text())
                self.status.set('Saved ' + path)
            except OSError as exc:
                self.status.set(str(exc))

    def _refresh_command(self):
        try:
            data = validate_session(dict(schema_version=1, map=self.map_var.get(), members=self.members))
            data['members'] = [{key: value for key, value in member.items() if value is not None} for member in data['members']]
            self.session_file.write_text(yaml.safe_dump(data, sort_keys=False))
            self.command = session_command(self.session_file, self.map_var.get(), self.navigation.get(), self.gui.get(), self.rviz.get())
            self.command_text.configure(state='normal')
            self.command_text.delete('1.0', 'end')
            self.command_text.insert('1.0', shlex.join(self.command))
            self.command_text.configure(state='disabled')
        except (ValueError, OSError, tk.TclError) as exc:
            self.status.set('Session configuration: ' + str(exc))

    def copy_command(self):
        self.clipboard_clear()
        self.clipboard_append(shlex.join(self.command))
        self.status.set('Shared-world command copied')

    def run(self):
        self._refresh_command()
        if self.app._launch_running:
            self.status.set('Stop the current simulation before starting a shared world')
            return
        self.stop_manual()
        self.app.start_launch_command(self.command)
        self.connect()
        self.status.set('Starting selected world and members; no movement until manual input or explicit task')

    def connect(self):
        if self.node is not None:
            self.status.set('Connected; command ownership stays neutral')
            return True
        if rclpy is None:
            self.status.set('ROS Python is unavailable; launch GUI from the sourced workspace')
            return False
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node('fleet_gui_' + self.operator_id[4:16])
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.command_pub = self.node.create_publisher(String, '/fleet/command', 10)
            self.heartbeat_pub = self.node.create_publisher(String, '/fleet/operator_heartbeat', 10)
            self.node.create_subscription(String, '/fleet/state', self._observe, 10)
            self.status.set('Connected; awaiting /fleet/state. No movement command sent.')
            return True
        except Exception as exc:
            self.status.set('ROS connection failed: ' + str(exc))
            return False

    def _observe(self, message):
        try:
            if len(message.data) > 262144:
                return
            data = json.loads(message.data)
            if data.get('schema_version') != 1 or data.get('backend') != 'gazebo':
                return
            previous = self.runtime
            if previous and previous['session_id'] != data['session_id']:
                self._clear_inputs()
                self.armed = []
                self.publishers.clear()
            self.runtime, self.runtime_at = data, time.monotonic()
            choices = [m['id'] for m in data['members']] + ['All included']
            self.target_combo.configure(values=choices)
            if self.target.get() not in choices:
                self.target.set(choices[0])
            lines = [f"{data['map']} · Gazebo · session {data['session_id']} · sim {data.get('sim_time')} s",
                     'Runtime artifacts: ' + data['directory']]
            for member in data['members']:
                pose, truth = member.get('estimate'), member.get('truth')
                position = f"({pose['x']:.2f}, {pose['y']:.2f})" if pose else 'waiting'
                measured = f"truth ({truth['x']:.2f}, {truth['y']:.2f})" if truth else 'truth waiting'
                command = member['command']
                lines.append(f"{member['id']} [{member['model']}] {member['mode']} · {position} · {measured} · "
                             f"v={command['linear']:.2f}, ω={command['angular']:.2f}\n  {member['reason']}")
            if data.get('last_rejection'):
                lines.append('Last rejected command: ' + data['last_rejection'])
            self.live.configure(state='normal')
            self.live.delete('1.0', 'end')
            self.live.insert('1.0', '\n'.join(lines))
            self.live.configure(state='disabled')
        except (ValueError, TypeError, KeyError):
            return

    def _ids(self, all_members=False):
        if not self.runtime or time.monotonic() - self.runtime_at > 1.:
            return []
        if all_members or self.target.get() == 'All included':
            return [m['id'] for m in self.runtime['members'] if m.get('included')]
        return [self.target.get()] if any(m['id'] == self.target.get() for m in self.runtime['members']) else []

    def _send(self, operation, targets=None, **extra):
        if self.node is None or not self.runtime or time.monotonic() - self.runtime_at > 1.:
            self.status.set('Connect to a live shared-world session first')
            return False
        message = dict(session_id=self.runtime['session_id'], operator_id=self.operator_id, op=operation,
                       targets=self._ids() if targets is None else targets, **extra)
        self.command_pub.publish(String(data=json.dumps(message, allow_nan=False)))
        self.status.set('Requested ' + operation + '; measured member status appears above')
        return True

    def _clear_inputs(self):
        self.latches.clear()
        self.keys.clear()
        self.ramp.linear = self.ramp.angular = 0.
        self.driving = False
        self._sync_buttons()
        self.joystick.close()

    def stop_manual(self):
        previous = list(self.armed)
        self._clear_inputs()
        self.armed = []
        if previous:
            self._send('stop', previous)

    def all_stop(self):
        self._clear_inputs()
        self.armed = []
        self._send('all_stop', [])

    def _arm(self):
        self.stop_manual()
        ids = self._ids()
        if ids and self._send('manual', ids):
            self.armed = ids

    def _latch(self, direction):
        if not self.armed:
            self.status.set('Arm selected manual controls before pressing a direction')
            return
        if direction in self.latches:
            self.latches.remove(direction)
        else:
            self.latches.add(direction)
        self._sync_buttons()

    def _sync_buttons(self):
        if not self._closed:
            for item, button in self.pad_buttons.items():
                button.state(['selected', 'pressed'] if item in self.latches else ['!selected', '!pressed'])

    def _input_changed(self):
        self._clear_inputs()
        self.status.set('Keyboard/joystick enabled neutral' if self.keyboard.get() else 'Keyboard/joystick disabled')

    def _operation(self, operation):
        self.stop_manual()
        try:
            data = {key: variable.get() for key, variable in self.goal.items()} if operation in ('dispatch', 'allocate') else {}
            if operation == 'dispatch' and len(self._ids()) != 1:
                raise ValueError('Select one instance for this goal, or use Nearest idle allocation')
            self._send(operation, **data)
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))

    def _swarm(self):
        self.stop_manual()
        try:
            extra = dict(controller=self.swarm_mode.get(), spacing=self.spacing.get(), heading=self.goal['yaw'].get())
            if self.use_anchor.get():
                extra.update(x=self.goal['x'].get(), y=self.goal['y'].get())
            self._send('swarm', self._ids(all_members=True), **extra)
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))

    def _can_keys(self):
        return (self.keyboard.get() and self.notebook.select() == str(self) and self.armed
                and not isinstance(self.focus_get(), (tk.Entry, ttk.Entry, tk.Text, ttk.Combobox)))

    def _key_press(self, event):
        key = event.keysym.lower()
        if (key == 'space' and self.keyboard.get() and self.notebook.select() == str(self)
                and not isinstance(self.focus_get(), (tk.Entry, ttk.Entry, tk.Text, ttk.Combobox))):
            self.all_stop()
            return 'break'
        if not self._can_keys():
            return
        if key in 'wasd' and len(key) == 1:
            job = self.key_jobs.pop(key, None)
            if job:
                self.after_cancel(job)
            self.keys.add(key)
            return 'break'

    def _key_release(self, event):
        key = event.keysym.lower()
        if key in self.keys:
            self.key_jobs[key] = self.after(35, lambda k=key: self.keys.discard(k))
            return 'break'

    def _focus_out(self, _event):
        # Moving focus to another widget is fine; leaving the application clears inputs.
        self.after_idle(lambda: self.stop_manual() if not self._closed and self.focus_get() is None else None)

    def _tick(self):
        if self._closed:
            return
        now = time.monotonic()
        if self.executor is not None and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.)
        if self.runtime and now - self.runtime_at <= 1.:
            if now - self.last_heartbeat >= .2:
                self.last_heartbeat = now
                self.heartbeat_pub.publish(String(data=json.dumps(dict(session_id=self.runtime['session_id'], operator_id=self.operator_id))))
            if self.armed and now - self.last_drive >= .1:
                self.last_drive = now
                rows = [m for m in self.runtime['members'] if m['id'] in self.armed]
                if self.runtime.get('operator_id') != self.operator_id or any(m['mode'] != 'manual' or not m['ready'] for m in rows):
                    # A rejected arm, stale feedback or runtime Stop cannot be
                    # revived by a direction held in the GUI.
                    if self.latches or self.keys or self.driving:
                        self.stop_manual()
                else:
                    linear_input = sum(v[0] for v in self.latches)
                    angular_input = sum(v[1] for v in self.latches)
                    if self.keyboard.get() and self.notebook.select() == str(self):
                        linear_input += int('w' in self.keys) - int('s' in self.keys)
                        angular_input += int('a' in self.keys) - int('d' in self.keys)
                        jl, ja = self.joystick.poll()
                        linear_input += jl
                        angular_input += ja
                    try:
                        dl, da = self.delta_linear.get(), self.delta_angular.get()
                        if not 0 < dl <= .05 or not 0 < da <= .1:
                            raise ValueError('Fleet increments: linear (0, .05], angular (0, .1]')
                        maximum_linear = min(m['limits']['linear'] for m in rows)
                        maximum_angular = min(m['limits']['angular'] for m in rows)
                        self.ramp.max_linear, self.ramp.min_linear = maximum_linear, -maximum_linear
                        self.ramp.max_angular, self.ramp.min_angular = maximum_angular, -maximum_angular
                        linear, angular = self.ramp.step(linear_input, angular_input, dl, da)
                        if linear_input or angular_input or self.driving or linear or angular:
                            self.driving = bool(linear_input or angular_input or linear or angular)
                            message = Twist()
                            message.linear.x, message.angular.z = linear, angular
                            for name in self.armed:
                                if name not in self.publishers:
                                    self.publishers[name] = self.node.create_publisher(Twist, '/' + name + '/key_vel', 10)
                                self.publishers[name].publish(message)
                    except (ValueError, tk.TclError) as exc:
                        self.stop_manual()
                        self.status.set(str(exc))
        elif self.armed:
            self._clear_inputs()
            self.armed = []
            self.status.set('Shared-world telemetry lost; inputs cleared and runtime watchdog stops manual/swarm motion')
        self._job = self.after(50, self._tick)

    def close(self):
        self.all_stop()
        self._closed = True
        self.after_cancel(self._job)
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=.1)
        if self.node is not None:
            self.node.destroy_node()
        self.node = self.executor = None
