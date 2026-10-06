"""Measured gripper controls for an owned, qualified Panda launch."""
import json
import math
import time
import tkinter as tk
from tkinter import ttk


class HandTab(ttk.Frame):
    def __init__(self, notebook, app):
        super().__init__(notebook, padding=10)
        self.app = app
        notebook.add(self, text='Hand')
        self.status_var = tk.StringVar(value='Select a robot with a qualified hand controller in Launch.')
        ttk.Label(self, textvariable=self.status_var, wraplength=880).grid(row=0, column=0, columnspan=4, sticky='w')
        self.gap_var = tk.DoubleVar(value=80.)
        self.force_var = tk.DoubleVar(value=.5)
        self.actual_var = tk.StringVar(value='No measured finger state')
        ttk.Label(self, text='Opening (mm)').grid(row=1, column=0, sticky='w', pady=10)
        ttk.Spinbox(self, textvariable=self.gap_var, from_=0., to=80., increment=5., width=8).grid(row=1, column=1)
        ttk.Label(self, text='Force limit per finger (N)').grid(row=2, column=0, sticky='w', pady=10)
        ttk.Spinbox(self, textvariable=self.force_var, from_=.1, to=20., increment=.1, width=8).grid(row=2, column=1)
        ttk.Label(self, textvariable=self.actual_var).grid(row=3, column=0, columnspan=4, sticky='w', pady=10)
        self.open_button = ttk.Button(self, text='Open', command=lambda: self.command(.08))
        self.close_button = ttk.Button(self, text='Close', command=lambda: self.command(0.))
        self.set_button = ttk.Button(self, text='Set Opening', command=self.set_opening)
        for column, button in enumerate((self.open_button, self.close_button, self.set_button)):
            button.grid(row=4, column=column, padx=5, pady=8)
        self.cancel_button = ttk.Button(self, text='Cancel', command=self.cancel)
        self.stop_button = ttk.Button(self, text='Stop Hand', command=self.stop)
        self.cancel_button.grid(row=5, column=0, padx=5, pady=8)
        self.stop_button.grid(row=5, column=1, padx=5, pady=8)
        self.fixture_var = tk.BooleanVar(value=False)
        self.fixture_button = ttk.Checkbutton(self, text='Add a supported grasp object on next Run',
            variable=self.fixture_var, command=app._update_validation_and_command)
        self.fixture_button.grid(row=6, column=0, columnspan=4, sticky='w', pady=12)
        ttk.Label(self, text='Controls use measured finger motion and the source tendon coupling.\n'
            'The opening is the sum of the two finger-joint displacements.\n'
            'Stop or heartbeat loss holds the measured opening; select the fixture before Run.',
            justify='left', wraplength=880).grid(row=7, column=0, columnspan=4, sticky='w')
        self.node = None
        self.state = None
        self.received = -math.inf
        self.handle = None
        self.sending = False
        self.was_owned = False
        self.closed = False
        self.refresh_selection()
        self.job = self.after(100, self.poll)

    def selected(self):
        return (self.app.arm_tab.selected()
                and self.app.robot_profiles.get(self.app.robot_var.get(), {}).get('hand_control') == 'panda')

    def owned(self):
        return self.selected() and self.app.arm_tab.owned()

    def refresh_selection(self):
        for button in (self.open_button, self.close_button, self.set_button, self.cancel_button, self.stop_button):
            button.state(['disabled'])
        self.fixture_button.state(['!disabled'] if self.selected() and not self.app._launch_running else ['disabled'])
        if not self.selected():
            self.status_var.set('Select the native Panda, MuJoCo, Display with a qualified hand controller.')

    def connect(self):
        if self.node:
            return True
        if not self.app._ensure_ros_publisher():
            return False
        from control_msgs.action import GripperCommand
        from rclpy.action import ActionClient
        from std_msgs.msg import Empty, String
        from std_srvs.srv import Trigger
        self.node = self.app.ros_node
        self.heartbeat_pub = self.node.create_publisher(Empty, '/gripper/heartbeat', 10)
        self.subscription = self.node.create_subscription(String, '/gripper/status', self.receive, 10)
        self.action = ActionClient(self.node, GripperCommand, '/gripper/gripper_action')
        self.stop_client = self.node.create_client(Trigger, '/gripper/stop')
        return True

    def receive(self, message):
        try:
            state = json.loads(message.data)
            if state.get('controller') != 'panda_gripper_native' or len(state.get('positions', [])) != 2:
                return
            self.state, self.received = state, time.monotonic()
        except (ValueError, TypeError):
            return

    def ready(self):
        return bool(self.owned() and self.state and time.monotonic()-self.received < .5)

    def poll(self):
        if self.closed:
            return
        owned = self.owned()
        if self.was_owned and not owned:
            self.stop()
            self.state = None
        if owned and self.connect():
            import rclpy
            from std_msgs.msg import Empty
            for _ in range(12):
                rclpy.spin_once(self.node, timeout_sec=0.)
            if self.ready():
                self.heartbeat_pub.publish(Empty())
                self.status_var.set('Native Panda gripper / MuJoCo — '+self.state['status'])
                self.actual_var.set('Measured opening %.2f mm; actuator effort %.3f N per finger' %
                    (1000*self.state['opening_m'], self.state['actuator_effort_per_finger_n']))
            else:
                self.status_var.set('Waiting for measured gripper state…')
        elif self.selected():
            self.status_var.set('Run the selected Panda in Launch to enable hand controls.')
        ready = self.ready()
        idle = ready and not self.state['busy'] and not self.sending and not (self.app.arm_tab.state or {}).get('busy', True)
        for button in (self.open_button, self.close_button, self.set_button):
            button.state(['!disabled'] if idle else ['disabled'])
        self.cancel_button.state(['!disabled'] if ready and self.handle else ['disabled'])
        self.stop_button.state(['!disabled'] if ready else ['disabled'])
        self.fixture_button.state(['!disabled'] if self.selected() and not self.app._launch_running else ['disabled'])
        self.was_owned = owned
        self.job = self.after(100, self.poll)

    def set_opening(self):
        try:
            self.command(float(self.gap_var.get())/1000.)
        except (ValueError, tk.TclError):
            self.status_var.set('Opening must be a number in 0–80 mm')

    def command(self, position):
        if not self.ready() or self.sending or self.state['busy'] or not self.action.server_is_ready():
            return
        try:
            effort = float(self.force_var.get())
        except (ValueError, tk.TclError):
            effort = float('nan')
        if not math.isfinite(position) or not 0 <= position <= .08 or not math.isfinite(effort) or not .1 <= effort <= 20.:
            self.status_var.set('Opening must be 0–80 mm and force limit 0.1–20 N per finger')
            return
        from control_msgs.action import GripperCommand
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = position, effort
        self.sending = True
        self.action.send_goal_async(goal).add_done_callback(self.accepted)

    def accepted(self, future):
        if self.closed:
            return
        self.sending = False
        self.handle = future.result()
        if not self.handle.accepted:
            self.handle = None
            self.status_var.set('Gripper goal rejected; inspect the launch log')
            return
        self.handle.get_result_async().add_done_callback(self.finished)

    def finished(self, future):
        if not self.closed:
            result = future.result().result
            self.status_var.set('Opening reached' if result.reached_goal else
                                'Contact stall at force bound' if result.stalled else 'Gripper action stopped')
            self.handle = None

    def cancel(self):
        if self.handle:
            self.handle.cancel_goal_async()

    def stop(self):
        if self.node and self.stop_client.service_is_ready():
            from std_srvs.srv import Trigger
            self.stop_client.call_async(Trigger.Request())

    def close(self):
        self.stop()
        self.closed = True
        self.after_cancel(self.job)
