"""Native position-actuator jogging from measured, owned simulator state."""
import json
import math
import time
from pathlib import Path
import tkinter as tk
from tkinter import ttk


class ArticulationControls(ttk.Frame):
    def __init__(self, parent, app):
        super().__init__(parent)
        self.app = app
        self.node = None
        self.state = None
        self.received = -math.inf
        self.channel = tk.StringVar()
        self.status = tk.StringVar(value='Enable native joint controls before Run.')
        self.feedback = tk.StringVar(value='Waiting for measured actuator state')
        self.target = tk.DoubleVar(value=0.)
        self.step = tk.DoubleVar(value=.01)
        ttk.Label(self, text='Native joint / coupled-hand controls', style='Heading.TLabel').grid(row=0, column=0, columnspan=4, sticky='w')
        ttk.Label(self, textvariable=self.status, wraplength=305).grid(row=1, column=0, columnspan=4, sticky='w', pady=5)
        self.selector = ttk.Combobox(self, textvariable=self.channel, state='readonly', width=30)
        self.selector.grid(row=2, column=0, columnspan=4, sticky='ew')
        self.selector.bind('<<ComboboxSelected>>', lambda _: self.select_channel())
        ttk.Label(self, textvariable=self.feedback, wraplength=305).grid(row=3, column=0, columnspan=4, sticky='w', pady=5)
        ttk.Label(self, text='Target (source units)').grid(row=4, column=0, sticky='w')
        self.entry = ttk.Entry(self, textvariable=self.target, width=10)
        self.entry.grid(row=4, column=1, sticky='w')
        self.scale = ttk.Scale(self, variable=self.target, from_=0., to=1.)
        self.scale.grid(row=5, column=0, columnspan=4, sticky='ew', pady=5)
        ttk.Label(self, text='Jog fraction of range').grid(row=6, column=0, sticky='w')
        ttk.Spinbox(self, textvariable=self.step, from_=.001, to=.1, increment=.005, width=8).grid(row=6, column=1)
        self.buttons = []
        for column, label, callback in (
                (0, '− Jog', lambda: self.jog(-1)), (1, '+ Jog', lambda: self.jog(1)),
                (2, 'Apply target', self.apply), (3, 'Use measured', self.select_channel)):
            button = ttk.Button(self, text=label, command=callback)
            button.grid(row=7, column=column, sticky='ew', padx=2, pady=6)
            self.buttons.append(button)
        self.home_button = ttk.Button(self, text='Source home', command=self.home)
        self.home_button.grid(row=8, column=0, columnspan=2, sticky='ew', padx=2)
        self.stop_button = ttk.Button(self, text='Stop / hold', command=self.stop)
        self.stop_button.grid(row=8, column=2, columnspan=2, sticky='ew', padx=2)
        ttk.Label(self, text='Targets use the source position actuators, including coupled tendons. '
            'Measured value, units and limits come from the running model. '
            'Dragging the target does not move the robot until Apply target. '
            'Heartbeat loss holds the measured articulation. Physical validation is pending.',
            wraplength=305).grid(row=9, column=0, columnspan=4, sticky='w', pady=8)
        from .servo_controls import ServoControls
        self.servo_controls = ServoControls(self, self, '/articulation/servo_command')
        self.servo_controls.grid(row=10, column=0, columnspan=4, sticky='ew', pady=8)
        from .articulation_sequences import ArticulationSequences
        self.sequences = ArticulationSequences(self, self)
        self.sequences.grid(row=11, column=0, columnspan=4, sticky='ew', pady=8)

    def selected(self):
        return (bool(self.app._robot_config().get('native_articulation'))
                and self.app.simulator_var.get() == 'mujoco'
                and self.app.mode_var.get() == 'display'
                and self.app.launch_kind_var.get() == 'simulation')

    def owned(self):
        process = self.app.process
        return bool(self.selected() and self.app._launch_running and process and process.poll() is None
            and 'robot_model:='+self.app.robot_var.get() in process.args
            and 'simulator:=mujoco' in process.args and 'mode:=display' in process.args
            and ('enable_native_articulation:=true' in process.args or 'enable_native_mobile:=true' in process.args))

    def connect(self):
        if self.node:
            return True
        if not self.app._ensure_ros_publisher():
            return False
        from std_msgs.msg import Empty, String
        from std_srvs.srv import Trigger
        self.node = self.app.ros_node
        self.pub = self.node.create_publisher(String, '/articulation/command', 1)
        self.heartbeat = self.node.create_publisher(Empty, '/articulation/heartbeat', 1)
        self.node.create_subscription(String, '/articulation/state', self.receive, 10)
        self.stop_client = self.node.create_client(Trigger, '/articulation/stop')
        return True

    def receive(self, message):
        try:
            state = json.loads(message.data)
            expected = self.app._robot_config().get('native_mjcf', '')
            if (not self.owned() or not state.get('plant_id') or not state.get('channels')
                    or Path(state['native_model']).resolve() != Path(expected).resolve()):
                return
            changed = not self.state or self.state['plant_id'] != state['plant_id']
            self.state, self.received = state, time.monotonic()
            names = [channel['name'] for channel in state['channels']]
            self.selector.configure(values=names)
            if changed or self.channel.get() not in names:
                self.channel.set(names[0])
                self.select_channel()
        except (ValueError, TypeError, KeyError):
            return

    def ready(self):
        return bool(self.owned() and self.state and time.monotonic()-self.received < .5
                    and not self.state.get('fault'))

    def current(self):
        return next((channel for channel in (self.state or {}).get('channels', [])
                     if channel['name'] == self.channel.get()), None)

    def select_channel(self):
        channel = self.current()
        if channel:
            self.target.set(channel['measured'])
            self.scale.configure(from_=channel['limits'][0], to=channel['limits'][1])

    def poll(self):
        if self.owned() and self.connect():
            import rclpy
            from std_msgs.msg import Empty
            for _ in range(12):
                rclpy.spin_once(self.node, timeout_sec=0.)
            if self.ready():
                self.heartbeat.publish(Empty())
                self.status.set(self.state['status'])
                if self.state.get('mobile_state'):
                    self.status.set(self.state['status']+' · Base: '+self.state['mobile_state']['status'])
                channel = self.current()
                if channel:
                    self.feedback.set('Measured %.4f %s · range %.4f … %.4f · force %.3f' %
                        (channel['measured'], channel['units'], *channel['limits'], channel['actuator_force']))
            else:
                self.status.set((self.state or {}).get('fault') or 'Waiting for fresh native actuator state…')
        else:
            self.state = None
            self.status.set('Enable Native joint controls in Launch, then Run the selected model.')
        for button in [*self.buttons, self.home_button, self.stop_button]:
            button.state(['!disabled'] if self.ready() else ['disabled'])
        self.servo_controls.poll()
        self.sequences.poll()

    def send(self, targets):
        if not self.ready():
            return
        from std_msgs.msg import Empty, String
        self.heartbeat.publish(Empty())
        self.pub.publish(String(data=json.dumps(dict(plant_id=self.state['plant_id'], targets=targets))))

    def apply(self):
        channel = self.current()
        if not channel:
            return
        try:
            value = float(self.target.get())
            if not math.isfinite(value) or not channel['limits'][0] <= value <= channel['limits'][1]:
                raise ValueError('Choose a finite target inside the source bounds.')
            self.send({channel['name']: value})
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))

    def jog(self, sign):
        channel = self.current()
        if not self.ready() or not channel:
            return
        try:
            fraction = float(self.step.get())
            if not math.isfinite(fraction) or not .001 <= fraction <= .1:
                raise ValueError('Use a jog fraction within 0.001–0.1.')
            span = channel['limits'][1]-channel['limits'][0]
            self.target.set(max(channel['limits'][0], min(channel['limits'][1], channel['target']+sign*fraction*span)))
            self.apply()
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))

    def home(self):
        if self.ready():
            self.send({channel['name']: channel['home'] for channel in self.state['channels']})

    def stop(self):
        self.servo_controls.stop()
        if self.node and self.owned() and self.stop_client.service_is_ready():
            from std_srvs.srv import Trigger
            self.stop_client.call_async(Trigger.Request())
