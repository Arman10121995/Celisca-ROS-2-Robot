"""Native multi-X500 session editor and explicitly owned flight controls."""
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

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
except ImportError:
    rclpy = None


def session_command(path, map_name, gui=True):
    return ['ros2', 'launch', 'robot_lab_bringup', 'aerial_swarm.launch.py',
            'session:='+str(path), 'map_name:='+map_name, 'gui:='+str(bool(gui)).lower()]


class AerialSwarmTab(ttk.Frame):
    def __init__(self, notebook, app):
        super().__init__(notebook, padding=10)
        notebook.add(self, text='Aerial Swarm')
        self.app, self.notebook = app, notebook
        self.operator = 'air-gui-'+uuid.uuid4().hex
        base = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))/'gui_profiles'
        base.mkdir(parents=True, exist_ok=True)
        self.session_file = base/(self.operator+'.yaml')
        self.node = self.executor = None
        self.runtime = None
        self.runtime_at = -math.inf
        self.heartbeat_at = self.drive_at = 0.
        self.armed = False
        self.arm_at = 0.
        self.closed = False
        self.ramp = RampDrive(max_linear=.4, min_linear=-.4, max_angular=.2, min_angular=-.2)
        self.vertical = 0.
        self.joystick = LinuxJoystick()
        self.latches, self.keys, self.key_jobs = set(), set(), {}
        self.map_var = tk.StringVar(value=app.map_var.get() if app.map_var.get() in app.map_profiles else 'nav_empty')
        self.family_var = tk.StringVar(value=app.map_groups.family(self.map_var.get()))
        self.count = tk.IntVar(value=2)
        self.spacing = tk.DoubleVar(value=3.)
        self.height_reference=tk.StringVar(value='barometer')
        self.gui = tk.BooleanVar(value=True)
        self.keyboard = tk.BooleanVar(value=False)
        self.target = tk.StringVar(value='All vehicles')
        self.controller = tk.StringVar(value='formation')
        self.status = tk.StringVar(value='Implemented experimental multi-FCU workflow. Connect is neutral; flight qualification remains separate.')
        self.delta_linear, self.delta_yaw = tk.DoubleVar(value=.02), tk.DoubleVar(value=.015)
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(2, weight=1)
        ttk.Label(self, text='Aerial swarm · one world · one PX4 FCU per X500', font=('', 12, 'bold')).grid(
            row=0, column=0, columnspan=2, sticky='w')
        ttk.Label(self, text='Two to four independent PX4 instances. Selected-world spawn/envelope checks, separate ports/frames and operator leases. '
            'Check clearance before explicit Takeoff. A source check or an armed FCU does not establish flight.',
            wraplength=1000).grid(row=1, column=0, columnspan=2, sticky='ew', pady=5)
        editor = ttk.LabelFrame(self, text='New shared session', padding=8)
        editor.grid(row=2, column=0, sticky='nsew', padx=(0, 8))
        editor.columnconfigure(1, weight=1)
        ttk.Label(editor, text='Map family').grid(row=0, column=0, sticky='w')
        families = ttk.Combobox(editor, textvariable=self.family_var,
            values=app.map_groups.choices(), state='readonly')
        families.grid(row=0, column=1, sticky='ew')
        families.bind('<<ComboboxSelected>>', self._family_changed)
        self.map_combo = ttk.Combobox(editor, textvariable=self.map_var, state='readonly')
        self.map_combo.grid(row=1, column=1, sticky='ew', pady=4)
        ttk.Label(editor, text='Map variant').grid(row=1, column=0, sticky='w')
        self.map_combo.bind('<<ComboboxSelected>>', lambda _: self.refresh_command())
        options = ttk.Frame(editor)
        options.grid(row=2, column=0, columnspan=2, sticky='ew', pady=5)
        ttk.Label(options, text='Vehicles').pack(side='left')
        count = ttk.Combobox(options, textvariable=self.count, values=(2, 3, 4), width=3, state='readonly')
        count.pack(side='left', padx=5)
        count.bind('<<ComboboxSelected>>', lambda _: self.refresh_command())
        ttk.Label(options, text='Spacing m').pack(side='left')
        space = ttk.Entry(options, textvariable=self.spacing, width=5)
        space.pack(side='left', padx=5)
        space.bind('<FocusOut>', lambda _: self.refresh_command())
        ttk.Checkbutton(options, text='Gazebo window', variable=self.gui, command=self.refresh_command).pack(side='left')
        references=ttk.Combobox(options,textvariable=self.height_reference,values=('barometer','gnss'),state='readonly',width=11)
        references.pack(side='left',padx=4)
        references.bind('<<ComboboxSelected>>',lambda _:self.refresh_command())
        grid = ttk.Frame(editor)
        grid.grid(row=3, column=0, columnspan=2, sticky='ew')
        for col, title in enumerate(('Vehicle / FCU / port', 'World X', 'World Y', 'Model Z')):
            ttk.Label(grid, text=title).grid(row=0, column=col, sticky='w')
        self.spawns = []
        for index in range(4):
            ttk.Label(grid, text=f'air{index+1} / {index+1} / {14580+index}').grid(row=index+1, column=0, sticky='w')
            values = {}
            for col, axis in enumerate('xyz', 1):
                values[axis] = tk.StringVar()
                entry = ttk.Entry(grid, textvariable=values[axis], width=9)
                entry.grid(row=index+1, column=col, padx=3, pady=3, sticky='ew')
                entry.bind('<FocusOut>', lambda _: self.refresh_command())
            self.spawns.append(values)
        ttk.Label(editor, text='Blank X/Y selects separated free positions near the map spawn. '
            'Blank Z uses model height 0; enter explicit XYZ for elevated floors or platforms. '
            'Each actual vehicle must pass a fresh takeoff check after startup. '
            'Changing this editor applies to the next session.', wraplength=470).grid(
                row=4, column=0, columnspan=2, sticky='w', pady=8)
        files = ttk.Frame(editor)
        files.grid(row=5, column=0, columnspan=2, sticky='ew')
        ttk.Button(files, text='Load session', command=self.load_session).pack(side='left')
        ttk.Button(files, text='Save session', command=self.save_session).pack(side='left', padx=5)
        control = ttk.LabelFrame(self, text='Live per-vehicle / group controls', padding=8)
        control.grid(row=2, column=1, sticky='nsew')
        control.columnconfigure(0, weight=1)
        ttk.Button(control, text='Connect · neutral', command=self.connect).grid(row=0, column=0, sticky='ew')
        self.target_combo = ttk.Combobox(control, textvariable=self.target, values=('All vehicles',), state='readonly')
        self.target_combo.grid(row=1, column=0, sticky='ew', pady=5)
        self.target_combo.bind('<<ComboboxSelected>>', lambda _: self.all_stop())
        actions = ttk.Frame(control)
        actions.grid(row=2, column=0, sticky='ew')
        for label, op in [('Check clearance', 'check'), ('Takeoff', 'takeoff'), ('Hold', 'hold'), ('Land', 'land')]:
            ttk.Button(actions, text=label, command=lambda op=op: self.operation(op)).pack(side='left', expand=True, fill='x')
        ttk.Button(control, text='Arm manual · neutral until input', command=self.arm_manual).grid(row=3, column=0, sticky='ew', pady=5)
        pad = ttk.Frame(control)
        pad.grid(row=4, column=0, sticky='ew')
        self.buttons = {}
        for label, direction, row, col in [('↖', (1, 1, 0), 0, 0), ('↑', (1, 0, 0), 0, 1), ('↗', (1, -1, 0), 0, 2),
                ('↶', (0, 1, 0), 1, 0), ('↓', (-1, 0, 0), 1, 1), ('↷', (0, -1, 0), 1, 2),
                ('Altitude +', (0, 0, 1), 2, 0), ('Altitude −', (0, 0, -1), 2, 2)]:
            button = ttk.Button(pad, text=label, style='LatchedDrive.TButton', command=lambda d=direction: self.latch(d))
            button.grid(row=row, column=col, sticky='ew', padx=2, pady=2)
            self.buttons[direction] = button
            for event in ('<ButtonRelease-1>', '<Leave>', '<Enter>', '<FocusOut>'):
                button.bind(event, lambda _: self.after_idle(self.sync_buttons), add='+')
        for col in range(3):
            pad.columnconfigure(col, weight=1)
        ttk.Button(pad, text='STOP / HOLD', command=self.all_stop).grid(row=2, column=1, sticky='ew', padx=2)
        ttk.Checkbutton(control, text='WASD + joystick · arrows change altitude · Space holds',
            variable=self.keyboard, command=self.input_changed).grid(row=5, column=0, sticky='w', pady=5)
        increments = ttk.Frame(control)
        increments.grid(row=6, column=0, sticky='ew')
        for title, variable in [('Δ linear / 0.1 s', self.delta_linear), ('Δ yaw / 0.1 s', self.delta_yaw)]:
            ttk.Label(increments, text=title).pack(side='left')
            ttk.Entry(increments, textvariable=variable, width=5).pack(side='left', padx=4)
        ttk.Label(control, text='Click to latch; click again to release and brake. Yaw is capped at 0.2 rad/s. '
            'Enablement sends no movement. Operator loss requests landing in each worker.', wraplength=480).grid(row=7, column=0, sticky='w', pady=6)
        group = ttk.Frame(control)
        group.grid(row=8, column=0, sticky='ew')
        ttk.Combobox(group, textvariable=self.controller, values=('formation', 'rendezvous', 'dispersion', 'flocking'),
            state='readonly', width=14).pack(side='left')
        ttk.Button(group, text='Start selected group', command=lambda: self.operation(self.controller.get())).pack(side='left', padx=5)
        self.live = scrolledtext.ScrolledText(self, height=7, state='disabled', wrap='word')
        self.live.grid(row=3, column=0, columnspan=2, sticky='ew', pady=5)
        ttk.Label(self, textvariable=self.status, wraplength=1000).grid(row=4, column=0, columnspan=2, sticky='ew')
        self.preview = scrolledtext.ScrolledText(self, height=3, state='disabled', wrap='word')
        self.preview.grid(row=5, column=0, columnspan=2, sticky='ew', pady=5)
        run = ttk.Frame(self)
        run.grid(row=6, column=0, columnspan=2, sticky='ew')
        ttk.Button(run, text='Run shared aerial session', command=self.run).pack(side='left')
        ttk.Button(run, text='Copy command', command=self.copy_command).pack(side='left', padx=5)
        ttk.Button(run, text='Stop session', command=app._stop_launch).pack(side='left')
        self.bind_all('<KeyPress>', self.key_press, add='+')
        self.bind_all('<KeyRelease>', self.key_release, add='+')
        self.bind_all('<FocusOut>', lambda _: self.after_idle(self.focus_lost), add='+')
        self._family_changed()
        self.job = self.after(50, self.tick)

    def _family_changed(self, _event=None):
        values = self.app.map_groups.members(self.family_var.get(), selectable=True)
        self.map_combo.configure(values=values)
        if self.map_var.get() not in values and values:
            self.map_var.set(values[0])
        self.refresh_command()

    def configuration(self):
        if self.height_reference.get() not in ('barometer','gnss'):
            raise ValueError('Choose barometer or gnss height reference')
        count, spacing = self.count.get(), self.spacing.get()
        if count not in (2, 3, 4) or not math.isfinite(spacing) or not 3. <= spacing <= 8.:
            raise ValueError('Choose 2–4 vehicles and spawn spacing 3–8 m')
        members = []
        for index, fields in enumerate(self.spawns[:count]):
            spawn = {key: float(var.get()) for key, var in fields.items() if var.get().strip()}
            if not all(math.isfinite(v) for v in spawn.values()) or ('x' in spawn) != ('y' in spawn):
                raise ValueError('Enter finite X and Y together, or leave both automatic')
            members.append(dict(id='air'+str(index+1), spawn=spawn))
        return dict(schema_version=1, map=self.map_var.get(), spacing=spacing, members=members,
                    estimator=dict(height_reference=self.height_reference.get()))

    def refresh_command(self):
        try:
            self.session_file.write_text(yaml.safe_dump(self.configuration(), sort_keys=False))
            self.command = session_command(self.session_file, self.map_var.get(), self.gui.get())
            self.preview.configure(state='normal')
            self.preview.delete('1.0', 'end')
            self.preview.insert('end', shlex.join(self.command))
            self.preview.configure(state='disabled')
        except (ValueError, tk.TclError, OSError) as exc:
            self.command = []
            self.status.set(str(exc))

    def run(self):
        self.refresh_command()
        if self.command:
            self.all_stop()
            self.app.start_launch_command(self.command)

    def copy_command(self):
        self.refresh_command()
        if self.command:
            self.clipboard_clear()
            self.clipboard_append(shlex.join(self.command))

    def save_session(self):
        try:
            config = self.configuration()
            path = filedialog.asksaveasfilename(initialdir=str(self.session_file.parent), defaultextension='.yaml')
            if path:
                Path(path).write_text(yaml.safe_dump(config, sort_keys=False))
        except (ValueError, OSError, tk.TclError) as exc:
            self.status.set(str(exc))

    def load_session(self):
        path = filedialog.askopenfilename(initialdir=str(self.session_file.parent), filetypes=[('Session YAML', '*.yaml')])
        if not path:
            return
        try:
            data = yaml.safe_load(Path(path).read_text())
            if not 2 <= len(data['members']) <= 4:
                raise ValueError('Choose 2–4 members')
            self.count.set(len(data['members']))
            self.map_var.set(data['map'])
            self.family_var.set(self.app.map_groups.family(data['map']))
            self.spacing.set(data.get('spacing', 3.))
            self.height_reference.set((data.get('estimator') or {}).get('height_reference','barometer'))
            for index, fields in enumerate(self.spawns):
                spawn = data['members'][index].get('spawn', {}) if index < len(data['members']) else {}
                for key, var in fields.items():
                    var.set(str(spawn[key]) if key in spawn else '')
            self._family_changed()
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self.status.set(str(exc))

    def connect(self):
        if self.node is not None:
            return
        if rclpy is None:
            self.status.set('ROS Python is unavailable; source the workspace')
            return
        if not rclpy.ok():
            rclpy.init(args=[])
        self.node = rclpy.create_node('aerial_swarm_gui_'+self.operator[-10:])
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.node.create_subscription(String, '/aerial_swarm/state', self.observe, 10)
        self.command_pub = self.node.create_publisher(String, '/aerial_swarm/command', 10)
        self.heartbeat_pub = self.node.create_publisher(String, '/aerial_swarm/operator_heartbeat', 10)
        self.drive_pub = self.node.create_publisher(Twist, '/aerial_swarm/key_vel', 10)
        self.status.set('Connected neutral; waiting for the actual shared aerial session')

    def observe(self, msg):
        try:
            data = json.loads(msg.data)
            if self.runtime and self.runtime['session_id'] != data['session_id']:
                self.clear_inputs()
                self.armed = False
            self.runtime, self.runtime_at = data, time.monotonic()
            names = [m['id'] for m in data['members']]
            self.target_combo.configure(values=['All vehicles', *names])
            if self.target.get() not in ['All vehicles', *names]:
                self.target.set('All vehicles')
            lines = [f"{data['session_id']} · {data['map']} · {data['mode']} · {data['reason']}"]
            for member in data['members']:
                status = member.get('status') or {}
                lines.append(f"{member['id']}: {status.get('phase', 'waiting')} · ready={member['ready']} "
                    f"· FCU settling remaining={status.get('preflight_remaining_fcu_s', 0):.1f}s "
                    f"· FCU={member['system_id']} port={member['port']} · estimated={member['position']} "
                    f"· truth={member['truth']} · reply={member.get('last_service')}")
            self.live.configure(state='normal')
            self.live.delete('1.0', 'end')
            self.live.insert('end', '\n'.join(lines))
            self.live.configure(state='disabled')
        except (ValueError, TypeError, KeyError) as exc:
            self.status.set('Aerial state rejected: '+str(exc))

    def targets(self):
        return ([m['id'] for m in self.runtime['members']] if self.target.get() == 'All vehicles'
                else [self.target.get()])

    def operation(self, op):
        if not self.runtime or time.monotonic()-self.runtime_at > 1.:
            self.status.set('Connect to a live shared aerial session first')
            return
        self.clear_inputs()
        if op != 'check':
            self.armed = False
        try:
            data = dict(session_id=self.runtime['session_id'], operator_id=self.operator,
                        op=op, targets=self.targets(), spacing=self.spacing.get())
            self.command_pub.publish(String(data=json.dumps(data, allow_nan=False)))
            self.status.set(op+' requested; actual worker replies appear above')
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))

    def arm_manual(self):
        self.operation('manual')
        self.armed = bool(self.runtime and time.monotonic()-self.runtime_at < 1.)
        self.arm_at = time.monotonic()

    def clear_inputs(self):
        moving=bool(self.latches or self.keys or self.ramp.linear or self.ramp.angular or self.vertical)
        self.latches.clear()
        self.keys.clear()
        self.ramp.stop()
        self.vertical = 0.
        for job in self.key_jobs.values():
            self.after_cancel(job)
        self.key_jobs.clear()
        self.sync_buttons()
        if self.node is not None and moving:
            self.drive_pub.publish(Twist())

    def all_stop(self):
        self.clear_inputs()
        self.armed = False
        if self.node is not None and self.runtime:
            self.command_pub.publish(String(data=json.dumps(dict(session_id=self.runtime['session_id'], op='stop'))))

    def latch(self, direction):
        if not self.armed:
            self.status.set('Take off, then arm manual control before selecting a direction')
            return
        if direction in self.latches:
            self.latches.remove(direction)
        else:
            self.latches.add(direction)
        self.sync_buttons()

    def sync_buttons(self):
        if self.closed:return
        for direction, button in getattr(self, 'buttons', {}).items():
            button.state(['pressed' if direction in self.latches else '!pressed'])

    def input_changed(self):
        self.clear_inputs()
        self.joystick.close()

    def key_press(self, event):
        if self.closed or not self.keyboard.get() or self.notebook.select() != str(self):
            return
        if isinstance(self.focus_get(), (tk.Entry, ttk.Entry, ttk.Combobox, tk.Text)):
            return
        key = event.keysym.lower()
        if key == 'space':
            self.all_stop()
            return 'break'
        if self.armed and key in ('w', 's', 'a', 'd', 'up', 'down'):
            if key in self.key_jobs:
                self.after_cancel(self.key_jobs.pop(key))
            self.keys.add(key)
            return 'break'

    def key_release(self, event):
        key = event.keysym.lower()
        if key in self.keys:
            self.key_jobs[key] = self.after(35, lambda: (self.keys.discard(key), self.key_jobs.pop(key, None)))
            return 'break'

    def focus_lost(self):
        if not self.closed and self.focus_get() is None:
            self.clear_inputs()

    def tick(self):
        if self.closed:
            return
        now = time.monotonic()
        if self.executor is not None and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.)
        live = self.runtime and now-self.runtime_at < 1.
        if live and now-self.heartbeat_at > .2:
            self.heartbeat_at = now
            self.heartbeat_pub.publish(String(data=json.dumps(dict(
                session_id=self.runtime['session_id'], operator_id=self.operator))))
        if self.armed and now-self.drive_at >= .1:
            self.drive_at = now
            ready = (live and self.runtime['operator_id'] == self.operator and self.runtime['mode'] == 'manual'
                     and all(m['ready'] for m in self.runtime['members'] if m['id'] in self.targets()))
            if not ready:
                if now-self.arm_at > 2.:
                    self.clear_inputs()
                    self.armed = False
                    self.status.set('Manual ownership/feedback lost; inputs cleared')
            else:
                forward, yaw, vertical = (sum(d[i] for d in self.latches) for i in range(3))
                if self.keyboard.get() and self.notebook.select() == str(self):
                    forward += int('w' in self.keys)-int('s' in self.keys)
                    yaw += int('a' in self.keys)-int('d' in self.keys)
                    vertical += int('up' in self.keys)-int('down' in self.keys)
                    jl, ja = self.joystick.poll()
                    forward, yaw = forward+jl, yaw+ja
                try:
                    dl, da = self.delta_linear.get(), self.delta_yaw.get()
                    if not 0 < dl <= .05 or not 0 < da <= .05:
                        raise ValueError('Linear/yaw increments must be (0, .05]')
                    vx, wz = self.ramp.step(forward, yaw, dl, da)
                    target = max(-1., min(1., vertical))*.25
                    self.vertical += max(-.025, min(.025, target-self.vertical))
                    output = Twist()
                    output.linear.x, output.linear.z, output.angular.z = vx, self.vertical, wz
                    self.drive_pub.publish(output)
                except (ValueError, tk.TclError) as exc:
                    self.all_stop()
                    self.status.set(str(exc))
        self.job = self.after(50, self.tick)

    def close(self):
        self.all_stop()
        self.closed = True
        self.after_cancel(self.job)
        self.joystick.close()
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=.1)
        if self.node is not None:
            self.node.destroy_node()
