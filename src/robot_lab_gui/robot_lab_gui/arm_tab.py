"""Joint controls for an owned native Panda launch, with measured feedback."""
import json
import math
import time
import tkinter as tk
from tkinter import ttk


class ArmTab(ttk.Frame):
    def __init__(self, notebook, app):
        super().__init__(notebook, padding=10)
        self.app = app
        notebook.add(self, text='Arm')
        self.status_var = tk.StringVar(value='Select the native Panda in Launch, with MuJoCo and Display.')
        ttk.Label(self, textvariable=self.status_var, wraplength=880).grid(row=0, column=0, columnspan=4, sticky='w')
        self.step_var = tk.DoubleVar(value=.1)
        ttk.Label(self, text='Jog increment (rad)').grid(row=1, column=0, sticky='w', pady=8)
        ttk.Spinbox(self, textvariable=self.step_var, from_=.02, to=.2, increment=.02,
                    width=8).grid(row=1, column=1, sticky='w')
        self.values = []
        self.buttons = []
        self.jog_buttons = []
        for index in range(7):
            row = index+2
            ttk.Label(self, text='Joint '+str(index+1)).grid(row=row, column=0, sticky='w', pady=5)
            value = tk.StringVar(value='No measured state')
            self.values.append(value)
            ttk.Label(self, textvariable=value, width=18).grid(row=row, column=1, sticky='w')
            for column, sign, title in [(2, -1, '−'), (3, 1, '+')]:
                button = ttk.Button(self, text=title, width=5,
                    command=lambda i=index, s=sign: self.jog(i, s))
                button.grid(row=row, column=column, padx=5)
                self.buttons.append(button); self.jog_buttons.append(button)
        self.home_button = ttk.Button(self, text='Home', command=self.home)
        self.home_button.grid(row=9, column=0, sticky='ew', pady=10)
        self.buttons.append(self.home_button)
        self.cancel_button = ttk.Button(self, text='Cancel Trajectory', command=self.cancel)
        self.cancel_button.grid(row=9, column=1, sticky='ew', padx=5)
        self.stop_button = ttk.Button(self, text='Stop Arm', command=self.stop)
        self.stop_button.grid(row=9, column=2, columnspan=2, sticky='ew')
        ttk.Label(self, text='Joint trajectories use the actual native Panda actuators and model limits.\n'
            'Stop, cancellation or GUI heartbeat loss holds the measured arm position.\n'
            'Cartesian planning, predictive collision checking, grasp and other arm backends remain pending.',
            wraplength=880, justify='left').grid(row=10, column=0, columnspan=4, sticky='w', pady=12)
        self.node = None
        self.state = None
        self.received = -math.inf
        self.was_owned = False
        self.handle = None
        self.sending = False
        self.closed = False
        self.refresh_selection()
        self.job = self.after(100, self.poll)

    def selected(self):
        return (self.app.robot_var.get() == 'menagerie_franka_emika_panda'
                and self.app.simulator_var.get() == 'mujoco'
                and self.app.mode_var.get() == 'display'
                and self.app.launch_kind_var.get() == 'simulation'
                and self.app.robot_profiles.get(self.app.robot_var.get(), {}).get('arm_control') == 'panda')

    def owned(self):
        process = self.app.process
        return bool(self.selected() and self.app._launch_running and process and process.poll() is None
                    and 'robot_model:=menagerie_franka_emika_panda' in process.args
                    and 'simulator:=mujoco' in process.args
                    and 'mode:=display' in process.args
                    and 'arm_control:=panda' in process.args)

    def refresh_selection(self):
        if not self.selected():
            self.status_var.set('Select menagerie_franka_emika_panda, MuJoCo, Display in Launch.')
        for button in self.buttons+[self.cancel_button, self.stop_button]:
            button.state(['disabled'])

    def connect(self):
        if self.node:
            return True
        if not self.app._ensure_ros_publisher():
            self.status_var.set('ROS 2 is unavailable in this GUI environment')
            return False
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from std_msgs.msg import Empty, String
        from std_srvs.srv import Trigger
        self.node = self.app.ros_node
        self.heartbeat_pub = self.node.create_publisher(Empty, '/arm/heartbeat', 10)
        self.subscription = self.node.create_subscription(String, '/arm/status', self.receive, 10)
        self.action = ActionClient(self.node, FollowJointTrajectory, '/arm/follow_joint_trajectory')
        self.stop_client = self.node.create_client(Trigger, '/arm/stop')
        return True

    def receive(self, message):
        try:
            state = json.loads(message.data)
            if state.get('controller') != 'panda_native' or len(state.get('positions', [])) != 7:
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
            # The same Tk thread owns callbacks and controls; no ROS worker
            # touches widgets or spins this node concurrently.
            for _ in range(12):
                rclpy.spin_once(self.node, timeout_sec=0.)
            if self.ready():
                self.heartbeat_pub.publish(Empty())
                self.status_var.set('Native Panda / MuJoCo — '+self.state['status'])
                for variable, position in zip(self.values, self.state['positions']):
                    variable.set('%.4f rad' % position)
            else:
                self.status_var.set('Waiting for measured native Panda state…')
        elif self.selected():
            self.status_var.set('Run the selected Panda from Launch to enable joint controls.')
        ready = self.ready()
        idle = ready and not self.state['busy'] and not self.sending and not self.state['contact_blocked']
        for button in self.buttons:
            button.state(['!disabled'] if idle else ['disabled'])
        self.cancel_button.state(['!disabled'] if ready and self.handle else ['disabled'])
        self.stop_button.state(['!disabled'] if ready else ['disabled'])
        self.was_owned = owned
        self.job = self.after(100, self.poll)

    def send_positions(self, target):
        if not self.ready() or self.sending or self.state['busy'] or not self.action.server_is_ready():
            return
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        distance = max(abs(a-b) for a, b in zip(target, self.state['positions']))
        duration = max(.8, 3.1*distance)
        if duration > 15:
            self.status_var.set('Target requires a trajectory longer than the supported 15 seconds')
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.state['joint_names']
        point = JointTrajectoryPoint(positions=list(target))
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration-int(duration))*1e9)
        goal.trajectory.points = [point]
        self.sending = True
        future = self.action.send_goal_async(goal)
        future.add_done_callback(self.accepted)

    def accepted(self, future):
        if self.closed:
            return
        self.sending = False
        self.handle = future.result()
        if not self.handle.accepted:
            self.handle = None
            self.status_var.set('Arm goal rejected; inspect the launch log')
            return
        self.handle.get_result_async().add_done_callback(self.finished)

    def finished(self, future):
        if self.closed:
            return
        self.status_var.set(future.result().result.error_string)
        self.handle = None

    def jog(self, index, sign):
        if not self.ready():
            return
        try:
            increment = float(self.step_var.get())
        except (ValueError, tk.TclError):
            return
        if not math.isfinite(increment) or not .02 <= increment <= .2:
            self.status_var.set('Jog increment must be within 0.02–0.2 rad')
            return
        target = list(self.state['positions'])
        target[index] += sign*increment
        lower, upper = self.state['limits'][index]
        if not lower+.005 <= target[index] <= upper-.005:
            self.status_var.set('Jog would exceed the native joint limit')
            return
        self.send_positions(target)

    def home(self):
        if self.ready():
            self.send_positions(self.state['home'])

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
