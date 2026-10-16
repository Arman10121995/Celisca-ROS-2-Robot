"""Owned MOLAR simulator lift, coupled telescope and roller controls."""
import hashlib
import json
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk

from .workspace_ui import ScrollPanel

SCHEMA = 'robot_lab.urdf_articulation.v1'
BACKENDS = ('gazebo', 'mujoco', 'pybullet', 'isaac')
LIFT = 'conveyor_lift_joint'
STAGE = 'telescope_stage1_joint'
COUPLED = 'telescope_stage2_joint'
ROLLER = 'transfer_roller_joint'
LIMITS = {LIFT: (0., .68), STAGE: (0., .14), COUPLED: (0., .14), ROLLER: (-8., 8.)}
TOPIC = '/molar/articulation'
FRESH_SECONDS = .5

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Empty, String
    from std_srvs.srv import Trigger
except ImportError:
    rclpy = None


def selectable(robot, backend, mode, kind, profile):
    return (robot in ('probot_l', 'molar_hri') and backend in BACKENDS
            and mode == 'display' and kind == 'simulation'
            and isinstance(profile.get('articulation'), dict)
            and profile['articulation'].get('schema') == SCHEMA)


def launch_matches(args, robot, backend, map_name):
    """Only the actual main launch slot, with unambiguous selected arguments."""
    if not isinstance(args, (list, tuple)) or list(args[:4]) != [
            'ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py']:
        return False
    expected = dict(robot_model=robot, simulator=backend, mode='display', map_name=map_name)
    return all([arg for arg in args if str(arg).startswith(key + ':=')] == [key + ':=' + value]
               for key, value in expected.items())


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
        raise ValueError('Measured values and targets must be finite numbers.')
    return float(value)


def validate_feedback(state, model_path):
    """Bind actual feedback to the exact selected source, without trusting names."""
    if not isinstance(state, dict) or state.get('schema') != SCHEMA:
        raise ValueError('Waiting for the MOLAR articulation state schema.')
    published = finite(state.get('state_monotonic'))
    if not -.05 <= time.monotonic() - published < FRESH_SECONDS:
        raise ValueError('Waiting for fresh articulation feedback from the running plant.')
    expected = Path(model_path).resolve(strict=True)
    if (not isinstance(state.get('plant_id'), str) or not state['plant_id']
            or len(state['plant_id']) > 128
            or Path(state.get('model_path', '')).resolve() != expected
            or state.get('model_sha256') != hashlib.sha256(expected.read_bytes()).hexdigest()):
        raise ValueError('The articulation model does not match this launch selection.')
    digest = state.get('urdf_sha256', '')
    if not isinstance(digest, str) or len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest):
        raise ValueError('The executed URDF identity is missing.')
    if (not isinstance(state.get('initialized'), bool) or not isinstance(state.get('fault'), str)
            or not isinstance(state.get('base_inhibited'), bool)
            or not isinstance(state.get('status'), str)):
        raise ValueError('Incomplete articulation readiness feedback.')
    channels = state.get('channels')
    if not isinstance(channels, list) or len(channels) != len(LIMITS):
        raise ValueError('Waiting for all four measured articulation channels.')
    found = {}
    for channel in channels:
        name = channel['name']
        if name not in LIMITS or name in found:
            raise ValueError('Unexpected or duplicate articulation channel.')
        mode, units = ('velocity', 'rad/s') if name == ROLLER else ('position', 'm')
        if (channel.get('mode') != mode or channel.get('units') != units
                or channel.get('coupled') is not (name == COUPLED)):
            raise ValueError('Articulation mode, units or slide coupling do not match.')
        bounds = channel['limits']
        if not isinstance(bounds, list) or len(bounds) != 2:
            raise ValueError('Missing source articulation limits.')
        low, high = map(finite, bounds)
        if low >= high or low < LIMITS[name][0] or high > LIMITS[name][1]:
            raise ValueError('Articulation limits exceed the MOLAR control envelope.')
        if finite(channel['max_velocity']) <= 0:
            raise ValueError('Missing bounded articulation rate.')
        for field in ('measured', 'measured_velocity', 'target'):
            if channel.get(field) is None:
                if state['initialized']:
                    raise ValueError('Waiting for measured articulation feedback.')
            else:
                finite(channel[field])
        if channel.get('target') is not None and not low <= channel['target'] <= high:
            raise ValueError('Reported target exceeds its source limits.')
        found[name] = channel
    error = state.get('coupling_error', {})
    if not isinstance(error, dict) or COUPLED not in error:
        raise ValueError('Waiting for the measured telescope coupling error.')
    finite(error[COUPLED])
    return found


def bounded_targets(targets, channels):
    if not isinstance(targets, dict) or not targets:
        raise ValueError('Choose an articulation target.')
    result = {}
    for name, value in targets.items():
        if name not in (LIFT, STAGE, ROLLER):
            raise ValueError('Command the first telescope stage; the second stage is coupled.')
        value = finite(value)
        low, high = channels[name]['limits']
        if not low <= value <= high or not LIMITS[name][0] <= value <= LIMITS[name][1]:
            raise ValueError('Choose a target inside the displayed source limits.')
        result[name] = value
    return result


class MolarControls(ScrollPanel):
    def __init__(self, notebook, app):
        super().__init__(notebook, width=320)
        notebook.add(self, text='MOLAR')
        self.app = app
        self.node = self.executor = self.pub = self.heartbeat = self.stop_client = None
        self.state = None
        self.received = -math.inf
        self.bound_process = self.publisher_gid = None
        self.last_heartbeat = -math.inf
        self.closed = False
        self.status = tk.StringVar(value='Select MOLAR or ProBOT-L, Display and a simulator, then Run.')
        self.feedback = tk.StringVar(value='Waiting for measured lift, both slides and roller velocity.')
        self.target_vars = {name: tk.DoubleVar(value=0.) for name in (LIFT, STAGE, ROLLER)}
        self.step_vars = {name: tk.DoubleVar(value=step) for name, step in ((LIFT, .01), (STAGE, .005), (ROLLER, .5))}
        self.apply_buttons, self.jog_buttons, self.scales, self.fields = {}, {}, {}, []
        body = self.body
        body.columnconfigure(0, weight=1)
        ttk.Label(body, text='MOLAR · conveyor and telescope', style='Heading.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Label(body, textvariable=self.status, wraplength=295).grid(row=1, column=0, sticky='ew', pady=6)
        ttk.Label(body, textvariable=self.feedback, wraplength=295, justify='left').grid(row=2, column=0, sticky='ew', pady=6)
        for row, name, label in ((3, LIFT, 'Lift above 0.420 m deck (m)'),
                                 (4, STAGE, 'Each telescope stage (m) · tip = 2×'),
                                 (5, ROLLER, 'Transfer roller velocity (rad/s)')):
            frame = ttk.LabelFrame(body, text=label, padding=6)
            frame.grid(row=row, column=0, sticky='ew', pady=5)
            frame.columnconfigure((0, 1), weight=1)
            entry = ttk.Entry(frame, textvariable=self.target_vars[name], width=10)
            entry.grid(row=0, column=0, sticky='ew', padx=2)
            button = ttk.Button(frame, text='Apply', command=lambda n=name: self.apply(n))
            button.grid(row=0, column=1, sticky='ew', padx=2)
            self.apply_buttons[name] = button
            low, high = LIMITS[name]
            scale = ttk.Scale(frame, variable=self.target_vars[name], from_=low, to=high)
            scale.grid(row=1, column=0, columnspan=2, sticky='ew', pady=5)
            self.scales[name] = scale
            for column, sign in ((0, -1), (1, 1)):
                jog = ttk.Button(frame, text='− Jog' if sign < 0 else '+ Jog',
                                 command=lambda n=name, s=sign: self.jog(n, s))
                jog.grid(row=2, column=column, sticky='ew', padx=2)
                self.jog_buttons[name, sign] = jog
            ttk.Label(frame, text='Jog step').grid(row=3, column=0, sticky='w', pady=4)
            step = ttk.Entry(frame, textvariable=self.step_vars[name], width=8)
            step.grid(row=3, column=1, sticky='ew', padx=2)
            self.fields.extend((entry, scale, step))
        actions = ttk.Frame(body)
        actions.grid(row=6, column=0, sticky='ew', pady=6)
        actions.columnconfigure((0, 1), weight=1)
        self.use_measured_button = ttk.Button(actions, text='Use measured', command=self.use_measured)
        self.use_measured_button.grid(row=0, column=0, sticky='ew', padx=2)
        self.stop_button = ttk.Button(actions, text='Stop / hold', command=self.stop, style='Danger.TButton')
        self.stop_button.grid(row=0, column=1, sticky='ew', padx=2)
        ttk.Label(body, text='Apply and Jog explicitly command motion. Editing or dragging a target sends nothing.\n'
                  'Raise before extending; retract both stages before lowering or driving.\n'
                  'Stop holds measured lift/slides and stops the roller. Leaving this page or losing feedback stops control.\n'
                  'Experimental simulator controls; payload handling remains unvalidated.',
                  wraplength=295, justify='left').grid(row=7, column=0, sticky='ew', pady=6)
        self.traces = [(var, var.trace_add('write', self.selection_changed)) for var in
                       (app.robot_var, app.simulator_var, app.mode_var, app.launch_kind_var, app.map_var)]
        self.bindings = [(widget, widget.bind('<<NotebookTabChanged>>', self.tab_changed, add='+'))
                         for widget in (app.notebook, notebook)]
        self.job = self.after(50, self.tick)
        self.update_buttons()

    def selected(self):
        return selectable(self.app.robot_var.get(), self.app.simulator_var.get(),
                          self.app.mode_var.get(), self.app.launch_kind_var.get(), self.app._robot_config())

    def visible(self):
        return (self.app.notebook.select() == str(self.app.launch_tab)
                and self.app.control_notebook.select() == str(self))

    def owned(self):
        process = self.app.process
        selected_map = str(self.app.map_var.get()).strip()
        map_name = 'none' if selected_map in ('', 'none', '— None —') else selected_map
        return bool(not self.closed and self.selected() and self.visible() and self.app._launch_running
                    and process and process.poll() is None and launch_matches(
                        process.args, self.app.robot_var.get(), self.app.simulator_var.get(), map_name))

    def connect(self):
        if self.node:
            return True
        if rclpy is None:
            self.status.set('Start Robot Lab from the sourced ROS workspace.')
            return False
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node('molar_gui_' + str(id(self)))
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.pub = self.node.create_publisher(String, TOPIC + '/command', 1)
        self.heartbeat = self.node.create_publisher(Empty, TOPIC + '/heartbeat', 1)
        self.node.create_subscription(String, TOPIC + '/state', self.receive, 1)
        self.stop_client = self.node.create_client(Trigger, TOPIC + '/stop')
        return True

    def graph_gid(self):
        if not self.node:
            return None
        publishers = self.node.get_publishers_info_by_topic(TOPIC + '/state')
        if (len(publishers) != 1 or self.node.count_subscribers(TOPIC + '/command') != 1
                or self.node.count_subscribers(TOPIC + '/heartbeat') != 1):
            return None
        return bytes(publishers[0].endpoint_gid)

    def receive(self, message, info=None):
        try:
            if not self.owned() or len(message.data) > 65536:
                return
            gid = self.graph_gid()
            if info is not None:
                # Additional DDS check when the executor preserves metadata.
                age = (time.time_ns() - info.source_timestamp) / 1e9
                publisher_ok = gid is not None and gid == bytes(info.publisher_gid)
            else:
                # Humble calls subscriptions with only the message. Its actual
                # publication time is checked from the payload below, never
                # inferred from callback delivery time.
                age = None
                publisher_ok = gid is not None
            if gid is None or not publisher_ok or (age is not None and not -.5 <= age < FRESH_SECONDS):
                raise ValueError('Waiting for fresh, unambiguous articulation feedback.')
            state = json.loads(message.data)
            validate_feedback(state, self.app._resolve_robot_path())
            changed = (not self.state or (state['initialized'] and not self.state.get('initialized'))
                       or state['plant_id'] != self.state['plant_id']
                       or gid != self.publisher_gid or self.bound_process is not self.app.process)
            self.state, self.received = state, time.monotonic()
            self.publisher_gid, self.bound_process = gid, self.app.process
            if changed:
                self.use_measured()
        except (ValueError, TypeError, KeyError, OSError, AttributeError) as exc:
            self.stop(reason=str(exc))

    def ready(self):
        return bool(self.owned() and self.state and self.bound_process is self.app.process
                    and time.monotonic() - self.received < FRESH_SECONDS
                    and -.05 <= time.monotonic() - self.state['state_monotonic'] < FRESH_SECONDS
                    and self.state.get('initialized') and not self.state.get('fault')
                    and self.publisher_gid == self.graph_gid())

    def channels(self):
        return {channel['name']: channel for channel in (self.state or {}).get('channels', [])}

    def use_measured(self):
        if not self.ready():
            return
        for name, variable in self.target_vars.items():
            channel = self.channels()[name]
            low, high = channel['limits']
            measured = channel['measured_velocity'] if name == ROLLER else channel['measured']
            variable.set(max(low, min(high, measured)))
            self.scales[name].configure(from_=low, to=high)

    def send(self, targets):
        if not self.ready():
            self.status.set('Select the owned running model and wait for fresh measured state.')
            return False
        try:
            validate_feedback(self.state, self.app._resolve_robot_path())
            targets = bounded_targets(targets, self.channels())
            self.heartbeat.publish(Empty())
            self.last_heartbeat = time.monotonic()
            self.pub.publish(String(data=json.dumps(dict(plant_id=self.state['plant_id'], targets=targets), allow_nan=False)))
            self.status.set('Target requested; inspect actual feedback.')
            return True
        except (ValueError, TypeError, KeyError, OSError) as exc:
            self.status.set(str(exc))
            return False

    def apply(self, name):
        try:
            return self.send({name: self.target_vars[name].get()})
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))
            return False

    def jog(self, name, sign):
        if not self.ready():
            return False
        try:
            step = finite(self.step_vars[name].get())
            low, high = self.channels()[name]['limits']
            if not 0 < step <= (high - low) / 10:
                raise ValueError('Jog step must be positive and no larger than 10% of the range.')
            value = finite(self.target_vars[name].get()) + sign * step
            self.target_vars[name].set(max(low, min(high, value)))
            return self.apply(name)
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))
            return False

    def stop(self, reason='Stop requested; holding measured lift/slides and stopping roller.'):
        # Selection has already changed when its trace runs. Revoke only the
        # previous owned process/graph; do not require the new selection to match.
        if (self.state and self.bound_process is self.app.process and self.bound_process.poll() is None
                and self.publisher_gid == self.graph_gid() and self.stop_client
                and self.stop_client.service_is_ready()):
            self.stop_client.call_async(Trigger.Request())
        self.state = None
        self.received = -math.inf
        self.bound_process = self.publisher_gid = None
        self.status.set(reason)
        self.update_buttons()

    def selection_changed(self, *_args):
        self.stop(reason='Selection changed; wait for the matching owned launch.')

    def tab_changed(self, _event=None):
        if not self.visible():
            self.stop(reason='MOLAR control held while this page is inactive.')

    def update_buttons(self):
        enabled = self.ready()
        for widget in [*self.apply_buttons.values(), *self.jog_buttons.values(), *self.fields, self.use_measured_button]:
            widget.state(['!disabled'] if enabled else ['disabled'])
        self.stop_button.state(['!disabled'] if self.state else ['disabled'])

    def tick(self):
        if self.closed:
            return
        try:
            if self.owned() and self.connect():
                if self.executor:
                    for _ in range(4):
                        self.executor.spin_once(timeout_sec=0.)
                if self.ready():
                    if time.monotonic() - self.last_heartbeat >= .1:
                        self.heartbeat.publish(Empty())
                        self.last_heartbeat = time.monotonic()
                    channels = self.channels()
                    lift, first, second = (channels[name]['measured'] for name in (LIFT, STAGE, COUPLED))
                    roller = channels[ROLLER]
                    self.feedback.set('Lift %.3f m · deck height %.3f m\nSlide 1 %.3f m · slide 2 %.3f m\n'
                                      'Total tip extension %.3f m · coupling error %.4f m\n'
                                      'Roller velocity %.3f rad/s · angle %.3f rad\nBase Drive %s' %
                                      (lift, .420 + lift, first, second, first + second,
                                       self.state['coupling_error'][COUPLED], roller['measured_velocity'],
                                       roller['measured'], 'inhibited' if self.state['base_inhibited'] else 'available'))
                    self.status.set(self.state['status'])
                elif self.state:
                    self.stop(reason=self.state.get('fault') or 'Feedback lost or uninitialized; control held.')
                else:
                    self.status.set('Waiting for fresh feedback from the selected model.')
            elif self.state:
                self.stop(reason='Launch or page ownership lost; articulation held.')
            self.update_buttons()
        except (RuntimeError, ValueError, OSError) as exc:
            self.stop(reason='Articulation connection unavailable: ' + str(exc))
        self.job = self.after(50, self.tick)

    def close(self):
        if self.closed:
            return
        self.stop()
        self.closed = True
        self.after_cancel(self.job)
        for variable, trace in self.traces:
            variable.trace_remove('write', trace)
        for widget, binding in self.bindings:
            widget.unbind('<<NotebookTabChanged>>', binding)
        if self.executor:
            self.executor.shutdown(timeout_sec=.1)
        if self.node:
            self.node.destroy_node()
        self.node = self.executor = None
