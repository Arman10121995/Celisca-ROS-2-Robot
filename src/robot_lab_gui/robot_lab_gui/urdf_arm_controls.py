"""Owned joint controls for an arm on the URDF articulation contract.

Any profile whose ``articulation`` block declares ``panel: arm`` (for example
the myAGV Pro + myCobot Pro630) gets one row per configured position joint.
Ownership, freshness, heartbeat and Stop are the MOLAR page's
(``molar_controls``); only the channels, targets and feedback differ.
"""
import math
import os
import json
import uuid
import time
import tkinter as tk
from tkinter import ttk

from .molar_controls import BACKENDS, MODES, SCHEMA, MolarControls, finite, validate_identity
from . import molar_controls
from .workspace_ui import ScrollPanel


def arm_config(profile):
    config = profile.get('articulation') if isinstance(profile, dict) else None
    if (isinstance(config, dict) and config.get('schema') == SCHEMA and config.get('panel') == 'arm'
            and isinstance(config.get('joints'), dict) and config['joints']
            and all(spec.get('mode') == 'position' for spec in config['joints'].values())):
        return config
    return None


def selectable(robot, backend, mode, kind, profile):
    return bool(backend in BACKENDS and mode in MODES and kind == 'simulation' and arm_config(profile))


def validate_arm_feedback(state, model_path, config):
    """Every configured joint, once, independent and inside its configured limits."""
    validate_identity(state, model_path)
    joints = config['joints']
    channels = state.get('channels')
    if not isinstance(channels, list) or len(channels) != len(joints):
        raise ValueError('Waiting for every configured arm joint.')
    found = {}
    for channel in channels:
        name = channel.get('name')
        if name not in joints or name in found:
            raise ValueError('Unexpected or duplicate arm joint.')
        if channel.get('mode') != 'position' or channel.get('units') not in ('rad', 'm') or channel.get('coupled'):
            raise ValueError('The arm page drives independent position joints only.')
        bounds = channel.get('limits')
        if not isinstance(bounds, list) or len(bounds) != 2:
            raise ValueError('Missing source joint limits.')
        low, high = map(finite, bounds)
        spec = joints[name]
        if low >= high or low < float(spec['lower']) - 1e-9 or high > float(spec['upper']) + 1e-9:
            raise ValueError('Joint limits exceed the configured arm envelope.')
        if finite(channel['max_velocity']) <= 0:
            raise ValueError('Missing bounded joint rate.')
        for field in ('measured', 'measured_velocity', 'target'):
            if channel.get(field) is None:
                if state['initialized']:
                    raise ValueError('Waiting for measured joint feedback.')
            else:
                finite(channel[field])
        if channel.get('target') is not None and not low <= channel['target'] <= high:
            raise ValueError('Reported target exceeds its source limits.')
        found[name] = channel
    return found


def short_name(name, prefix):
    return name[len(prefix):] if prefix and name.startswith(prefix) else name


class UrdfArmControls(MolarControls):
    """Arm page; inherits MOLAR's owned-launch, heartbeat and Stop logic."""
    page_label = 'Mobile arm'

    def __init__(self, notebook, app):
        ScrollPanel.__init__(self, notebook, width=320)
        notebook.add(self, text='Mobile arm')
        self.app = app
        self.node = self.executor = self.pub = self.heartbeat = self.stop_client = None
        self.state = None
        self.received = -math.inf
        self.bound_process = self.publisher_gid = None
        self.last_heartbeat = -math.inf
        self.closed = False
        self.transfer = None
        self.plan_request = ''
        self.joint_names = ()
        self.target_vars, self.apply_buttons, self.jog_buttons, self.scales, self.fields = {}, {}, {}, {}, []
        self.status = tk.StringVar(value='Select a mobile manipulator, a simulator and Run.')
        self.feedback = tk.StringVar(value='Waiting for measured joint state.')
        self.step = tk.DoubleVar(value=.05)
        body = self.body
        body.columnconfigure(0, weight=1)
        ttk.Label(body, text='Arm joints on a mobile base', style='Heading.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Label(body, textvariable=self.status, wraplength=295).grid(row=1, column=0, sticky='ew', pady=6)
        ttk.Label(body, textvariable=self.feedback, wraplength=295, justify='left').grid(row=2, column=0, sticky='ew', pady=6)
        self.joint_frame = ttk.Frame(body)
        self.joint_frame.grid(row=3, column=0, sticky='ew')
        self.joint_frame.columnconfigure(0, weight=1)
        step = ttk.Frame(body)
        step.grid(row=4, column=0, sticky='ew', pady=4)
        ttk.Label(step, text='Jog step (rad or m)').grid(row=0, column=0, sticky='w')
        self.step_entry = ttk.Entry(step, textvariable=self.step, width=8)
        self.step_entry.grid(row=0, column=1, sticky='w', padx=4)
        actions = ttk.Frame(body)
        actions.grid(row=5, column=0, sticky='ew', pady=6)
        actions.columnconfigure((0, 1, 2), weight=1)
        self.use_measured_button = ttk.Button(actions, text='Use measured', command=self.use_measured)
        self.use_measured_button.grid(row=0, column=0, sticky='ew', padx=2)
        self.home_button = ttk.Button(actions, text='Home (direct)', command=self.home)
        self.home_button.grid(row=0, column=1, sticky='ew', padx=2)
        self.stop_button = ttk.Button(actions, text='Stop / hold', command=self.stop, style='Danger.TButton')
        self.stop_button.grid(row=0, column=2, sticky='ew', padx=2)
        planning = ttk.LabelFrame(body, text='Collision-checked joint route', padding=4)
        planning.grid(row=6, column=0, sticky='ew', pady=5)
        planning.columnconfigure((0, 1, 2), weight=1)
        self.plan_button = ttk.Button(planning, text='Plan targets', command=self.plan_targets)
        self.plan_button.grid(row=0, column=0, sticky='ew', padx=2)
        self.plan_home_button = ttk.Button(planning, text='Plan Home', command=self.plan_home)
        self.plan_home_button.grid(row=0, column=1, sticky='ew', padx=2)
        self.execute_button = ttk.Button(planning, text='Execute plan', command=self.execute_plan)
        self.execute_button.grid(row=0, column=2, sticky='ew', padx=2)
        self.plan_feedback = tk.StringVar(value='Planning requires a supported live backend/profile.')
        ttk.Label(planning, textvariable=self.plan_feedback, wraplength=290, justify='left').grid(
            row=1, column=0, columnspan=3, sticky='ew', pady=4)
        ttk.Label(body, text='Apply, Jog and direct Home explicitly command motion; editing or dragging sends nothing.\n'
                  'Plan targets / Plan Home inspect a private source/world snapshot without moving the robot. '
                  'Execute is a separate command; edited targets or robot/scene changes require a new plan.\n'
                  'The base will not drive while a joint moves, and a command sent while the base '
                  'moves is refused and holds the arm.\n'
                  'Stop, heartbeat loss, leaving this page or a selection change hold the measured joints.\n'
                  'Experimental simulator controls; masses and limits are declared estimates where '
                  'the vendor publishes none.', wraplength=295, justify='left').grid(row=7, column=0, sticky='ew', pady=6)
        self.traces = [(var, var.trace_add('write', self.selection_changed)) for var in
                       (app.robot_var, app.simulator_var, app.mode_var, app.launch_kind_var, app.map_var)]
        self.bindings = [(widget, widget.bind('<<NotebookTabChanged>>', self.tab_changed, add='+'))
                         for widget in (app.notebook, notebook)]
        self.rebuild()
        self.job = self.after(50, self.tick)
        self.update_buttons()

    def arm(self):
        return arm_config(self.app._robot_config())

    def rebuild(self):
        """One row per configured joint of the selected profile."""
        config = self.arm()
        names = tuple(config['joints']) if config else ()
        if names == self.joint_names:
            return
        for child in self.joint_frame.winfo_children():
            child.destroy()
        self.joint_names = names
        self.target_vars, self.apply_buttons, self.jog_buttons, self.scales = {}, {}, {}, {}
        self.fields = [self.step_entry]
        prefix = os.path.commonprefix(names) if len(names) > 1 else ''
        prefix = prefix[:prefix.rfind('_') + 1]          # pro630_joint1_to_base -> joint1_to_base
        for row, name in enumerate(names):
            spec = config['joints'][name]
            frame = ttk.LabelFrame(self.joint_frame, text='J%d · %s' % (row + 1, short_name(name, prefix)), padding=4)
            frame.grid(row=row, column=0, sticky='ew', pady=3)
            frame.columnconfigure((0, 1, 2, 3), weight=1)
            self.target_vars[name] = tk.DoubleVar(value=0.)
            entry = ttk.Entry(frame, textvariable=self.target_vars[name], width=8)
            entry.grid(row=0, column=0, sticky='ew', padx=2)
            for column, sign in ((1, -1), (2, 1)):
                jog = ttk.Button(frame, text='−' if sign < 0 else '+', width=3,
                                 command=lambda n=name, s=sign: self.jog(n, s))
                jog.grid(row=0, column=column, sticky='ew', padx=2)
                self.jog_buttons[name, sign] = jog
            button = ttk.Button(frame, text='Apply', command=lambda n=name: self.apply(n))
            button.grid(row=0, column=3, sticky='ew', padx=2)
            self.apply_buttons[name] = button
            scale = ttk.Scale(frame, variable=self.target_vars[name], from_=float(spec['lower']), to=float(spec['upper']))
            scale.grid(row=1, column=0, columnspan=4, sticky='ew', pady=3)
            self.scales[name] = scale
            self.fields.extend((entry, scale))

    def selected(self):
        return selectable(self.app.robot_var.get(), self.app.simulator_var.get(),
                          self.app.mode_var.get(), self.app.launch_kind_var.get(), self.app._robot_config())

    def selection_changed(self, *_args):
        super().selection_changed()
        self.rebuild()

    def validate(self, state):
        config = self.arm()
        if not config:
            raise ValueError('The selected robot has no arm articulation.')
        return validate_arm_feedback(state, self.app._resolve_robot_path(), config)

    def bounded(self, targets):
        if not isinstance(targets, dict) or not targets:
            raise ValueError('Choose a joint target.')
        channels, result = self.channels(), {}
        for name, value in targets.items():
            if name not in channels:
                raise ValueError('Unknown arm joint.')
            value = finite(value)
            low, high = channels[name]['limits']
            if not low <= value <= high:
                raise ValueError('Choose a target inside the displayed joint limits.')
            result[name] = value
        return result

    def show_feedback(self):
        lines = []
        for index, name in enumerate(self.joint_names):
            channel = self.channels()[name]
            value = channel['measured']
            angle = ' (%.1f°)' % math.degrees(value) if channel['units'] == 'rad' else ''
            lines.append('J%d %.3f %s%s' % (index + 1, value, channel['units'], angle))
        lines.append('Base Drive %s' % ('inhibited (arm moving or settling)' if self.state['base_inhibited']
                                        else 'available'))
        self.feedback.set('\n'.join(lines))

    def use_measured(self):
        if not self.ready():
            return
        for name, variable in self.target_vars.items():
            channel = self.channels()[name]
            low, high = channel['limits']
            variable.set(max(low, min(high, channel['measured'])))
            self.scales[name].configure(from_=low, to=high)

    def planning(self):
        planning = (self.state or {}).get('planning')
        return planning if isinstance(planning, dict) and planning.get('schema') == 'robot_lab.articulation_plan.v1' else {}

    def planning_enabled(self):
        configured = (self.arm() or {}).get('planning', {})
        return (self.ready() and self.planning().get('available') is True
                and configured.get('schema') == 'robot_lab.articulation_plan.v1'
                and configured.get('backend') == self.app.simulator_var.get())

    def current_targets(self):
        return self.bounded({name: variable.get() for name, variable in self.target_vars.items()})

    def plan_matches(self):
        state = self.planning()
        try:
            targets = self.current_targets()
            frozen = state.get('targets', {})
            return bool(self.planning_enabled() and state.get('phase') == 'ready'
                        and self.plan_request and state.get('request_id') == self.plan_request
                        and isinstance(state.get('plan_id'), str) and len(state['plan_id']) == 32
                        and isinstance(frozen, dict) and targets.keys() == frozen.keys()
                        and all(abs(value-finite(frozen[name])) < 1e-9 for name, value in targets.items()))
        except (ValueError, TypeError, KeyError, tk.TclError):
            return False

    def planning_command(self, operation, **payload):
        if not self.planning_enabled():
            self.status.set('Wait for the owned source model with collision planning available.')
            return False
        request = {key: self.state[key] for key in ('plant_id', 'model_sha256', 'urdf_sha256')}
        request.update(operation=operation, **payload)
        self.heartbeat.publish(molar_controls.Empty())
        self.last_heartbeat = time.monotonic()
        self.pub.publish(molar_controls.String(data=json.dumps(request, allow_nan=False)))
        return True

    def plan_targets(self):
        try:
            targets = self.current_targets()
            identity = uuid.uuid4().hex
            if self.planning_command('plan', request_id=identity, targets=targets):
                self.plan_request = identity
                self.status.set('Plan requested; robot holds measured joints. Execute remains separate.')
                return True
        except (ValueError, TypeError, KeyError, tk.TclError) as exc:
            self.status.set(str(exc))
        return False

    def plan_home(self):
        if not self.planning_enabled():
            return False
        for name, variable in self.target_vars.items():
            variable.set(float(self.arm().get('home', {}).get(name, 0.)))
        return self.plan_targets()

    def execute_plan(self):
        if not self.plan_matches():
            self.status.set('Plan again after editing targets, moving the robot or expiry.')
            return False
        return self.planning_command('execute_plan', plan_id=self.planning()['plan_id'])

    def send(self, targets):
        self.plan_request = ''
        return super().send(targets)

    def stop(self, reason='Stop requested; holding measured arm and revoking its plan.'):
        self.plan_request = ''
        return super().stop(reason)

    def jog(self, name, sign):
        if not self.ready():
            return False
        try:
            step = finite(self.step.get())
            low, high = self.channels()[name]['limits']
            if not 0 < step <= (high - low) / 10:
                raise ValueError('Jog step must be positive and no larger than 10% of the range.')
            value = finite(self.target_vars[name].get()) + sign * step
            self.target_vars[name].set(max(low, min(high, value)))
            return self.apply(name)
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))
            return False

    def home(self):
        config = self.arm()
        if not config or not self.ready():
            return False
        home = {name: float(config.get('home', {}).get(name, 0.)) for name in self.joint_names}
        for name, value in home.items():
            self.target_vars[name].set(value)
        return self.send(home)

    def update_buttons(self):
        enabled = self.ready()
        for widget in [*self.apply_buttons.values(), *self.jog_buttons.values(), *self.fields,
                       self.use_measured_button, self.home_button]:
            widget.state(['!disabled'] if enabled else ['disabled'])
        self.stop_button.state(['!disabled'] if self.state else ['disabled'])
        planning = self.planning()
        can_plan = self.planning_enabled() and planning.get('phase') not in ('requested', 'planning', 'executing')
        for button in (self.plan_button, self.plan_home_button):
            button.state(['!disabled'] if can_plan else ['disabled'])
        self.execute_button.state(['!disabled'] if self.plan_matches() else ['disabled'])
        self.plan_feedback.set((planning.get('phase', '')+': '+planning.get('message', '')).strip(': ')
                              if planning else 'Collision planning is unavailable on this backend/profile. Direct joint controls remain available.')
