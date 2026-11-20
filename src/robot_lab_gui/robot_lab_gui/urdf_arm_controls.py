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

from robot_lab_utils.urdf_articulation import grip_aperture_interpolate, grip_aperture_samples

from .molar_controls import BACKENDS, MODES, SCHEMA, MolarControls, finite, validate_identity
from . import molar_controls
from .workspace_ui import ScrollPanel

try:
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
except ImportError:
    String = Trigger = None

# urdf_pick_place.py pose_source values and their labels.
POSE_SOURCE_LABELS = {'truth': 'Simulator truth', 'rgbd': 'RGB-D camera'}
CARTESIAN_SCHEMA = 'robot_lab.cartesian_goal.v1'
ORIENTATION_LABELS = {'Keep final angle': 'keep', 'Set roll / pitch / yaw': 'rpy', 'Position only': 'free'}


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
    """Every configured joint, once, inside its configured limits.

    Configured couplings (a gripper's second finger) are reported as coupled
    followers: measured and shown, never commanded from this page.
    """
    validate_identity(state, model_path)
    if 'grip_contacts' in state:
        contact = state['grip_contacts']
        readings = contact.get('normal_force_n') if isinstance(contact, dict) else None
        if (not isinstance(readings, dict) or any(
                not isinstance(load, dict) or set(load) != {'left', 'right'}
                or any(finite(force) < 0. for force in load.values()) for load in readings.values())):
            raise ValueError('Incomplete gripper pad force feedback.')
    joints = config['joints']
    couplings = config.get('couplings') or {}
    channels = state.get('channels')
    if not isinstance(channels, list) or len(channels) != len(joints) + len(couplings):
        raise ValueError('Waiting for every configured arm joint.')
    found = {}
    for channel in channels:
        name = channel.get('name')
        if name not in joints and name not in couplings or name in found:
            raise ValueError('Unexpected or duplicate arm joint.')
        if channel.get('mode') != 'position' or channel.get('units') not in ('rad', 'm') \
                or bool(channel.get('coupled')) != (name in couplings):
            raise ValueError('The arm page drives independent position joints only.')
        bounds = channel.get('limits')
        if not isinstance(bounds, list) or len(bounds) != 2:
            raise ValueError('Missing source joint limits.')
        low, high = map(finite, bounds)
        spec = joints.get(name) or joints[couplings[name]['joint']]
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
        self.tool_plant = None
        self.joint_names = ()
        self.target_vars, self.apply_buttons, self.jog_buttons, self.scales, self.fields = {}, {}, {}, {}, []
        self.grip_buttons = []
        self.grip_openings = {}
        self.status = tk.StringVar(value='Select a mobile manipulator, a simulator and Run.')
        self.feedback = tk.StringVar(value='Waiting for measured joint state.')
        self.step = tk.DoubleVar(value=.05)
        body = self.body
        body.columnconfigure(0, weight=1)
        ttk.Label(body, text='Arm joints on a mobile base', style='Heading.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Label(body, textvariable=self.status, wraplength=295).grid(row=1, column=0, sticky='ew', pady=6)
        ttk.Label(body, textvariable=self.feedback, wraplength=295, justify='left').grid(row=2, column=0, sticky='ew', pady=6)
        self.joint_frame = ttk.Frame(body)
        self.joint_frame.grid(row=6, column=0, sticky='ew')
        self.joint_frame.columnconfigure(0, weight=1)
        step = ttk.Frame(body)
        step.grid(row=7, column=0, sticky='ew', pady=4)
        ttk.Label(step, text='Jog step (rad or m)').grid(row=0, column=0, sticky='w')
        self.step_entry = ttk.Entry(step, textvariable=self.step, width=8)
        self.step_entry.grid(row=0, column=1, sticky='w', padx=4)
        actions = ttk.Frame(body)
        actions.grid(row=3, column=0, sticky='ew', pady=6)
        actions.columnconfigure((0, 1, 2), weight=1)
        self.use_measured_button = ttk.Button(actions, text='Use measured', command=self.use_measured)
        self.use_measured_button.grid(row=0, column=0, sticky='ew', padx=2)
        self.home_button = ttk.Button(actions, text='Home (direct)', command=self.home)
        self.home_button.grid(row=0, column=1, sticky='ew', padx=2)
        self.stop_button = ttk.Button(actions, text='Stop / hold', command=self.stop, style='Danger.TButton')
        self.stop_button.grid(row=0, column=2, sticky='ew', padx=2)
        self._build_pick_place(body)
        self._build_cartesian(body)
        planning = ttk.LabelFrame(body, text='Plan and execute', padding=4)
        planning.grid(row=5, column=0, sticky='ew', pady=5)
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
                  'the vendor publishes none.', wraplength=295, justify='left').grid(row=9, column=0, sticky='ew', pady=6)
        selection = (app.robot_var, app.simulator_var, app.mode_var, app.launch_kind_var, app.map_var,
                     getattr(app, 'experimental_assets_var', None))
        self.traces = [(var, var.trace_add('write', self.selection_changed)) for var in selection if var is not None]
        self.bindings = [(widget, widget.bind('<<NotebookTabChanged>>', self.tab_changed, add='+'))
                         for widget in (app.notebook, notebook)]
        self.rebuild()
        self.job = self.after(50, self.tick)
        self.update_buttons()

    def _build_cartesian(self, body):
        self.tool_frame = frame = ttk.LabelFrame(body, text='Tool pose in robot base', padding=4)
        frame.grid(row=4, column=0, sticky='ew', pady=5)
        frame.columnconfigure(1, weight=1)
        self.tool_feedback = tk.StringVar(value='Waiting for measured tool pose.')
        ttk.Label(frame, textvariable=self.tool_feedback, wraplength=285).grid(
            row=0, column=0, columnspan=4, sticky='ew')
        self.tool_xyz = [tk.DoubleVar(value=0.) for _ in range(3)]
        self.tool_rpy = [tk.DoubleVar(value=0.) for _ in range(3)]
        self.tool_step = tk.DoubleVar(value=.02)
        self.tool_orientation = tk.StringVar(value=next(iter(ORIENTATION_LABELS)))
        self.tool_fields = []
        for axis, (label, variable) in enumerate(zip(('X forward (m)', 'Y left (m)', 'Z up (m)'), self.tool_xyz)):
            row = axis+1
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky='w')
            entry = ttk.Entry(frame, textvariable=variable, width=9)
            entry.grid(row=row, column=1, sticky='ew')
            self.tool_fields.append(entry)
            for column, sign in ((2, -1), (3, 1)):
                jog = ttk.Button(frame, text='−' if sign < 0 else '+', width=3,
                                 command=lambda axis=axis, sign=sign: self.jog_tool(axis, sign))
                jog.grid(row=row, column=column)
                self.tool_fields.append(jog)
        ttk.Label(frame, text='XYZ step (m)').grid(row=4, column=0, sticky='w')
        step = ttk.Entry(frame, textvariable=self.tool_step, width=9)
        step.grid(row=4, column=1, sticky='ew')
        self.tool_fields.append(step)
        self.tool_orientation_box = ttk.Combobox(frame, textvariable=self.tool_orientation, state='readonly',
                                                 values=list(ORIENTATION_LABELS))
        self.tool_orientation_box.grid(row=5, column=0, columnspan=4, sticky='ew', pady=3)
        angles = ttk.Frame(frame)
        angles.grid(row=6, column=0, columnspan=4, sticky='ew')
        ttk.Label(angles, text='RPY (°)').pack(side='left')
        self.tool_angle_fields = []
        for variable in self.tool_rpy:
            entry = ttk.Entry(angles, textvariable=variable, width=7)
            entry.pack(side='left', expand=True, fill='x', padx=2)
            self.tool_angle_fields.append(entry)
        self.copy_tool_button = ttk.Button(frame, text='Use measured tool pose', command=self.use_tool_pose)
        self.copy_tool_button.grid(row=7, column=0, columnspan=2, sticky='ew', pady=4)
        self.plan_pose_button = ttk.Button(frame, text='Plan pose', command=self.plan_pose)
        self.plan_pose_button.grid(row=7, column=2, columnspan=2, sticky='ew', pady=4)
        ttk.Label(frame, text='XYZ buttons edit the target. Plan, then Execute below. '
                  'The tool can curve or rotate along the joint route.',
                  wraplength=285).grid(row=8, column=0, columnspan=4, sticky='ew')

    def tool_config(self):
        articulation = self.arm() or {}
        config = articulation.get('cartesian') or {}
        return config if (config.get('schema') == CARTESIAN_SCHEMA and
                          articulation.get('planning', {}).get('backend') == self.app.simulator_var.get()) else {}

    def measured_tool(self):
        config, pose = self.tool_config(), self.planning().get('tool')
        if (not config or not self.planning_enabled() or not isinstance(pose, dict)
                or pose.get('schema') != CARTESIAN_SCHEMA
                or pose.get('frame') != config['reference_frame'] or pose.get('tip_frame') != config['tip_frame']):
            return None
        try:
            if len(pose['position']) != 3 or len(pose['rpy_deg']) != 3:
                return None
            return dict(position=list(map(finite, pose['position'])), rpy_deg=list(map(finite, pose['rpy_deg'])))
        except (ValueError, TypeError, KeyError):
            return None

    def use_tool_pose(self):
        pose = self.measured_tool()
        if pose is None:
            return
        for variable, value in zip(self.tool_xyz+self.tool_rpy, pose['position']+pose['rpy_deg']):
            variable.set(round(value, 6))

    def jog_tool(self, axis, sign):
        if self.measured_tool() is None:
            return
        try:
            step = finite(self.tool_step.get())
            if not 0 < step <= .05:
                raise ValueError('XYZ step must be greater than zero and at most 0.05 m.')
            variable = self.tool_xyz[axis]
            variable.set(round(finite(variable.get())+sign*step, 6))
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))

    def current_pose(self):
        if self.measured_tool() is None:
            raise ValueError('Wait for the measured source tool frame.')
        orientation = ORIENTATION_LABELS[self.tool_orientation.get()]
        pose = dict(frame=self.tool_config()['reference_frame'], orientation=orientation,
                    position=[finite(field.get()) for field in self.tool_xyz])
        if orientation == 'rpy':
            pose['rpy_deg'] = [finite(field.get()) for field in self.tool_rpy]
        if any(abs(value) > 10. for value in pose['position']) or any(
                abs(value) > 180. for value in pose.get('rpy_deg', [])):
            raise ValueError('Enter bounded XYZ metres and angles within ±180°.')
        return pose

    def plan_pose(self):
        try:
            identity = uuid.uuid4().hex
            if self.planning_command('plan_cartesian', request_id=identity, pose=self.current_pose()):
                self.plan_request = identity
                self.status.set('Tool pose requested. Review the route before Execute.')
                return True
        except (ValueError, TypeError, KeyError, tk.TclError) as exc:
            self.status.set(str(exc))
        return False

    # --- pick and place (urdf_pick_place.py) ------------------------------------
    def _build_pick_place(self, body):
        """Object pick and place for profiles with a pick_place description."""
        frame = ttk.LabelFrame(body, text='Pick / place', padding=4)
        frame.grid(row=8, column=0, sticky='ew', pady=5)
        frame.columnconfigure(1, weight=1)
        self.pick_frame = frame
        self.pick_state, self.pick_received = {}, -math.inf
        self.pick_pub = self.pick_stop_client = None
        self.pick_object, self.pick_support = tk.StringVar(), tk.StringVar()
        self.pick_lift = tk.DoubleVar(value=.10)
        # Object pose for the grasp: simulator truth, or the head camera (mobile fetch).
        self.pick_source = tk.StringVar(value=POSE_SOURCE_LABELS['truth'])
        ttk.Label(frame, text='Object').grid(row=0, column=0, sticky='w')
        self.pick_object_box = ttk.Combobox(frame, textvariable=self.pick_object, state='readonly', width=18)
        self.pick_object_box.grid(row=0, column=1, sticky='ew', padx=2)
        ttk.Label(frame, text='Place on').grid(row=1, column=0, sticky='w')
        self.pick_support_box = ttk.Combobox(frame, textvariable=self.pick_support, state='readonly', width=18)
        self.pick_support_box.grid(row=1, column=1, sticky='ew', padx=2)
        ttk.Label(frame, text='Lift (m)').grid(row=2, column=0, sticky='w')
        self.pick_lift_entry = ttk.Entry(frame, textvariable=self.pick_lift, width=8)
        self.pick_lift_entry.grid(row=2, column=1, sticky='w', padx=2)
        ttk.Label(frame, text='Object pose').grid(row=3, column=0, sticky='w')
        self.pick_source_box = ttk.Combobox(frame, textvariable=self.pick_source, state='readonly', width=18,
                                            values=[POSE_SOURCE_LABELS['truth']])
        self.pick_source_box.grid(row=3, column=1, sticky='ew', padx=2)
        buttons = ttk.Frame(frame)
        buttons.grid(row=4, column=0, columnspan=2, sticky='ew', pady=3)
        buttons.columnconfigure((0, 1), weight=1)
        self.pick_run_button = ttk.Button(buttons, text='Run pick / place', command=self.run_pick_place)
        self.pick_run_button.grid(row=0, column=0, sticky='ew', padx=2)
        self.pick_stop_button = ttk.Button(buttons, text='Stop task', command=self.stop_pick_place,
                                           style='Danger.TButton')
        self.pick_stop_button.grid(row=0, column=1, sticky='ew', padx=2)
        self.pick_status = tk.StringVar(value='Available for fixed arms with a pick_place description.')
        ttk.Label(frame, textvariable=self.pick_status, wraplength=290, justify='left').grid(
            row=5, column=0, columnspan=2, sticky='ew')

    def pick_supported(self):
        return bool(self.arm() and self.app._robot_config().get('pick_place'))

    def connect(self):
        connected = super().connect()
        if connected and self.pick_pub is None:
            self.pick_pub = self.node.create_publisher(String, '/pick_place/request', 1)
            self.node.create_subscription(String, '/pick_place/state', self.receive_pick_place, 1)
            self.pick_stop_client = self.node.create_client(Trigger, '/pick_place/stop')
        return connected

    def receive_pick_place(self, message):
        try:
            state = json.loads(message.data)
        except ValueError:
            return
        if isinstance(state, dict) and state.get('schema') == 'robot_lab.pick_place_state.v1':
            self.pick_state, self.pick_received = state, time.monotonic()

    def pick_fresh(self):
        return self.pick_supported() and time.monotonic() - self.pick_received < 2.0

    def run_pick_place(self):
        """Explicitly start one pick and place; the node plans, executes and judges it."""
        if not (self.ready() and self.pick_fresh()) or self.pick_state.get('phase') != 'idle':
            return False
        if self.pick_state.get('restart_required'):
            self.pick_status.set(self.pick_state['message'])
            return False
        if self.pick_state.get('ready') is False:
            self.pick_status.set(self.pick_state.get('feedback_blocker', 'Waiting for measured pick feedback.'))
            return False
        lift = finite(self.pick_lift.get())
        if not .03 <= lift <= .25 or self.pick_object.get() not in self.pick_state.get('objects', []) \
                or self.pick_support.get() not in self.pick_state.get('supports', []):
            self.pick_status.set('Choose an object, a support and a 0.03-0.25 m lift.')
            return False
        source = next((k for k, v in POSE_SOURCE_LABELS.items() if v == self.pick_source.get()), 'truth')
        if source not in self.pick_state.get('pose_sources', ['truth']):
            self.pick_status.set('This robot and world have no %s object pose.' % self.pick_source.get())
            return False
        if source == 'rgbd' and self.pick_object.get() not in self.pick_state.get('rgbd_objects', []):
            self.pick_status.set('%s has no declared RGB-D colour and size in this world.' % self.pick_object.get())
            return False
        self.pick_pub.publish(String(data=json.dumps(dict(object=self.pick_object.get(),
                                                          support=self.pick_support.get(), lift=lift,
                                                          pose_source=source))))
        self.pick_status.set('Requested: %s onto %s.' % (self.pick_object.get(), self.pick_support.get()))
        return True

    def stop_pick_place(self):
        if self.pick_stop_client is not None and self.pick_stop_client.service_is_ready():
            self.pick_stop_client.call_async(Trigger.Request())
        self.stop(reason='Pick / place stopped; holding measured arm.')

    def update_pick_place(self):
        fresh = self.pick_fresh()
        state = self.pick_state if fresh else {}
        for box, variable, values in ((self.pick_object_box, self.pick_object, state.get('objects', [])),
                                      (self.pick_support_box, self.pick_support, state.get('supports', []))):
            if list(box['values']) != list(values):
                box['values'] = values
            if values and variable.get() not in values:
                variable.set(values[0])
        sources = [POSE_SOURCE_LABELS[k] for k in state.get('pose_sources', ['truth']) if k in POSE_SOURCE_LABELS]
        if list(self.pick_source_box['values']) != sources:
            self.pick_source_box['values'] = sources
        if sources and self.pick_source.get() not in sources:
            self.pick_source.set(sources[0])
        idle = state.get('phase') == 'idle'
        # A mobile profile fetches: drive, pick, carry, place (urdf_pick_place.py).
        mobile = bool(state.get('mobile'))
        title = 'Fetch: drive · pick · carry · place' if mobile else 'Pick / place'
        if self.pick_frame.cget('text') != title:
            self.pick_frame.configure(text=title)
            self.pick_run_button.configure(text='Run fetch' if mobile else 'Run pick / place')
        can_start = (fresh and idle and not state.get('restart_required')
                     and state.get('ready', True) and self.ready())
        self.pick_run_button.state(['!disabled'] if can_start else ['disabled'])
        self.pick_stop_button.state(['!disabled'] if fresh and not idle else ['disabled'])
        if not self.pick_supported():
            self.pick_status.set('This profile has no pick_place description.')
        elif not fresh:
            self.pick_status.set('Waiting for the pick-and-place node.')
        elif idle and state.get('feedback_blocker') and not state.get('restart_required'):
            self.pick_status.set(state['feedback_blocker'])
        else:
            text = '%s: %s' % (state.get('phase'), state.get('message'))
            result = state.get('last_result') or {}
            if idle and result:
                text += '\nLast: %s%s' % (result.get('outcome'), (' · ' + result['receipt']) if result.get('receipt') else '')
            self.pick_status.set(text)

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
        self.grip_buttons = []
        self.grip_openings = {}
        prefix = os.path.commonprefix(names) if len(names) > 1 else ''
        prefix = prefix[:prefix.rfind('_') + 1]          # pro630_joint1_to_base -> joint1_to_base
        for row, name in enumerate(names):
            spec = config['joints'][name]
            presets = config.get('grip_presets', {}).get(name, {})
            label = 'Gripper' if presets else 'J%d · %s' % (row + 1, short_name(name, prefix))
            frame = ttk.LabelFrame(self.joint_frame, text=label, padding=4)
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
            for column, action in enumerate(('open', 'close')):
                if action in presets:
                    button = ttk.Button(frame, text=action.title(),
                        command=lambda n=name, target=presets[action]: self.apply_grip_preset(n, target))
                    button.grid(row=2, column=2*column, columnspan=2, sticky='ew', padx=2)
                    self.grip_buttons.append(button)
            try:
                aperture = grip_aperture_samples(config, name)
            except ValueError as exc:
                self.status.set(str(exc))
                aperture = ()
            if aperture:
                self._build_grip_opening(frame, name, aperture)

    def _build_grip_opening(self, frame, name, aperture):
        feedback = tk.StringVar(value='Waiting for jaw feedback')
        maximum = math.floor(aperture[-1][1]*1000.)
        minimum = max(0, math.ceil(aperture[0][1]*1000.))
        opening = tk.DoubleVar(value=maximum)
        ttk.Label(frame, textvariable=feedback, wraplength=280).grid(
            row=3, column=0, columnspan=4, sticky='w', pady=3)
        field = ttk.Spinbox(frame, textvariable=opening, from_=minimum, to=maximum, increment=1, width=6)
        field.grid(row=4, column=0, sticky='ew', padx=2)
        button = ttk.Button(frame, text='Set opening (mm)', command=lambda: self.apply_grip_opening(name))
        button.grid(row=4, column=1, columnspan=3, sticky='ew', padx=2)
        self.grip_openings[name] = (opening, feedback, aperture, button)
        self.fields.extend((field, button))

    def selected(self):
        return selectable(self.app.robot_var.get(), self.app.simulator_var.get(),
                          self.app.mode_var.get(), self.app.launch_kind_var.get(), self.app._robot_config())

    def selection_changed(self, *_args):
        super().selection_changed()
        self.tool_plant = None
        self.plan_request = ''
        self.pick_lift.set(self.app._robot_config().get('pick_place', {}).get('default_lift', .10))
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
        if self.state.get('symmetric_grip'):
            lines.append('Gripper fingers move together. Apply either finger target.')
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
        return self.bounded({name: self.target_vars[name].get() for name in self.planning_joints()})

    def planning_joints(self):
        return (self.arm() or {}).get('planning', {}).get('joints', self.joint_names)

    def plan_matches(self):
        state = self.planning()
        try:
            if not (self.planning_enabled() and state.get('phase') == 'ready'
                    and self.plan_request and state.get('request_id') == self.plan_request
                    and isinstance(state.get('plan_id'), str) and len(state['plan_id']) == 32):
                return False
            if state.get('cartesian_request') is not None:
                return self.current_pose() == state['cartesian_request']
            targets = self.current_targets()
            frozen = state.get('targets', {})
            return bool(isinstance(frozen, dict) and targets.keys() == frozen.keys()
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
        for name in self.planning_joints():
            self.target_vars[name].set(float(self.arm().get('home', {}).get(name, 0.)))
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

    def apply_grip_preset(self, name, target):
        if not self.send({name: target}):
            return False
        self.target_vars[name].set(target)
        return True

    def apply_grip_opening(self, name):
        if not self.ready():
            return False
        opening, _feedback, aperture, _button = self.grip_openings[name]
        try:
            gap = finite(opening.get())/1000.
            if gap < 0.:
                raise ValueError('Opening must be zero or positive.')
            target = grip_aperture_interpolate(gap, aperture, by_gap=True)
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))
            return False
        return self.apply_grip_preset(name, target)

    def update_buttons(self):
        self.update_pick_place()
        enabled = self.ready()
        for widget in [*self.apply_buttons.values(), *self.jog_buttons.values(), *self.grip_buttons, *self.fields,
                       self.use_measured_button, self.home_button]:
            widget.state(['!disabled'] if enabled else ['disabled'])
        if self.grip_openings:
            channels = self.channels() if enabled else {}
            for name, (_opening, feedback, aperture, _button) in self.grip_openings.items():
                if name in channels:
                    angle = max(aperture[0][0], min(aperture[-1][0], channels[name]['measured']))
                    width = max(0., grip_aperture_interpolate(angle, aperture))*1000.
                    text = f'Estimated opening {width:.1f} mm'
                    contacts = self.state.get('grip_contacts', {}).get('normal_force_n', {})
                    if contacts:
                        target = max(contacts, key=lambda item: sum(contacts[item].values()))
                        left, right = (contacts[target][side] for side in ('left', 'right'))
                        text += f'\n{target} · left {left:.2f} N · right {right:.2f} N'
                    feedback.set(text)
                else:
                    feedback.set('Waiting for jaw feedback')
        self.stop_button.state(['!disabled'] if self.state else ['disabled'])
        planning = self.planning()
        can_plan = (self.planning_enabled() and not self.state['base_inhibited']
                    and planning.get('phase') not in ('requested', 'planning', 'executing'))
        for button in (self.plan_button, self.plan_home_button):
            button.state(['!disabled'] if can_plan else ['disabled'])
        if self.tool_config():
            self.tool_frame.grid()
        else:
            self.tool_frame.grid_remove()
        tool = self.measured_tool()
        if tool:
            plant = (self.bound_process, self.state['plant_id'])
            if self.tool_plant != plant:
                self.use_tool_pose()
                self.tool_plant = plant
            self.tool_feedback.set('%s · measured XYZ %.3f, %.3f, %.3f m' %
                                   (self.tool_config()['tip_frame'], *tool['position']))
        else:
            self.tool_feedback.set('Waiting for measured tool pose in a supported rigid scene.')
        for field in [*self.tool_fields, self.tool_orientation_box, self.copy_tool_button]:
            field.state(['!disabled'] if tool else ['disabled'])
        for field in self.tool_angle_fields:
            field.state(['!disabled'] if tool and ORIENTATION_LABELS[self.tool_orientation.get()] == 'rpy'
                        else ['disabled'])
        self.plan_pose_button.state(['!disabled'] if tool and can_plan else ['disabled'])
        self.execute_button.state(['!disabled'] if self.plan_matches() else ['disabled'])
        if planning:
            text = (planning.get('phase', '')+': '+planning.get('message', '')).strip(': ')
            if planning.get('waypoints'):
                text += '\n%d waypoints · %s · completed %d' % (
                    planning['waypoints'], planning.get('planner', {}).get('algorithm', ''), planning.get('waypoint', 0))
            summary = planning.get('planner', {})
            if summary.get('ik_candidates'):
                text += '\n%d feasible IK candidates' % summary['ik_candidates']
            if summary.get('measured_tcp_error'):
                error = summary['measured_tcp_error']
                text += '\nMeasured tool error %.1f mm' % (1000.*error['position_m'])
                if error['orientation_rad'] is not None:
                    text += ' · %.1f°' % math.degrees(error['orientation_rad'])
            self.plan_feedback.set(text)
        else:
            self.plan_feedback.set('Collision planning is unavailable on this backend/profile. Direct joint controls remain available.')
