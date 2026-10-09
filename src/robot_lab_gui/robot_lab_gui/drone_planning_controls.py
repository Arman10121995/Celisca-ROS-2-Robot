"""Measured FCU targets and separate static aerial Plan/Execute controls."""
import json
import math
import time
import tkinter as tk
from tkinter import ttk


class DronePlanningControls(ttk.LabelFrame):
    def __init__(self, parent, app):
        super().__init__(parent, text='3D target / static-world planning (experimental)', padding=8)
        self.app, self.node, self.state, self.position = app, None, {}, None
        self.received, self.pose_received = -math.inf, -math.inf
        self.status = tk.StringVar(value='Take off, use the measured position, edit target, then Plan and Execute.')
        self.target = [tk.DoubleVar(value=0.) for _ in range(3)]
        ttk.Button(self, text='Use current FCU position', command=self.use_measured).grid(row=0, column=0, columnspan=4, sticky='ew')
        for row, (axis, variable) in enumerate(zip('XYZ', self.target), 1):
            ttk.Label(self, text=axis+' (map metres)').grid(row=row, column=0, sticky='w')
            ttk.Entry(self, textvariable=variable, width=9).grid(row=row, column=1, sticky='ew')
            for column, sign in ((2, -1), (3, 1)):
                ttk.Button(self, text='−' if sign < 0 else '+', width=2,
                    command=lambda row=row, sign=sign: self.offset(row-1, sign)).grid(row=row, column=column, padx=2)
        self.plan_button = ttk.Button(self, text='Plan route', command=self.plan)
        self.execute_button = ttk.Button(self, text='Execute route', command=lambda: self.service('execute'))
        self.stop_button = ttk.Button(self, text='Cancel / Hold', command=lambda: self.service('cancel'))
        for column, button in enumerate((self.plan_button, self.execute_button, self.stop_button)):
            button.grid(row=4, column=column, sticky='ew', padx=2, pady=6)
        ttk.Label(self, textvariable=self.status, wraplength=305).grid(row=5, column=0, columnspan=4, sticky='w')
        ttk.Label(self, text='± changes a target by 0.25 m; no motion until Execute. '
            'The static planner uses a conservative 0.7 m spherical envelope. '
            'Moving actors, flight tracking and contact validation remain separate.',
            wraplength=305).grid(row=6, column=0, columnspan=4, sticky='w', pady=6)
        self.after(200, self.poll)

    def owned(self):
        process = self.app.process
        return bool(self.app._flight_selectable() and self.app._launch_running and process
            and process.poll() is None and 'robot_model:=px4_x500' in process.args and 'mode:=flight' in process.args)

    def connect(self):
        if self.node:
            return True
        if not self.app._ensure_ros_publisher():
            return False
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        self.node = self.app.ros_node
        self.goal_pub = self.node.create_publisher(PoseStamped, '/px4/plan_goal', 1)
        self.node.create_subscription(String, '/px4/planning/status', self.receive, 10)
        self.node.create_subscription(Odometry, '/px4/odometry', self.receive_pose, 10)
        self.clients = {name: self.node.create_client(Trigger, '/px4/plan_'+name) for name in ('execute', 'cancel')}
        return True

    def receive(self, message):
        try:
            if self.owned():
                self.state, self.received = json.loads(message.data), time.monotonic()
        except (TypeError, ValueError):
            pass

    def receive_pose(self, message):
        p = message.pose.pose.position
        values = [p.x, p.y, p.z]
        if self.owned() and all(map(math.isfinite, values)):
            self.position, self.pose_received = values, time.monotonic()

    def ready(self):
        return bool(self.owned() and self.position is not None and self.state.get('plant_id')
            and time.monotonic()-self.pose_received < .8 and time.monotonic()-self.received < 1.)

    def use_measured(self):
        if self.ready():
            for variable, value in zip(self.target, self.position):
                variable.set(round(value, 3))

    def offset(self, axis, sign):
        try:
            self.target[axis].set(round(float(self.target[axis].get())+.25*sign, 3))
        except (ValueError, tk.TclError):
            self.status.set('Enter a finite target in map metres.')

    def plan(self):
        if not self.ready():
            return
        try:
            from geometry_msgs.msg import PoseStamped
            values = [float(variable.get()) for variable in self.target]
            if not all(map(math.isfinite, values)):
                raise ValueError('Finite target required')
            message = PoseStamped()
            message.header.frame_id = 'map'
            message.pose.position.x, message.pose.position.y, message.pose.position.z = values
            message.pose.orientation.w = 1.
            self.goal_pub.publish(message)
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))

    def service(self, name):
        if name == 'execute' and not self.target_matches_plan():
            self.status.set('Target changed; Plan again before Execute.')
            return
        if self.owned() and self.connect() and self.clients[name].service_is_ready():
            from std_srvs.srv import Trigger
            def completed(future):
                try:
                    self.status.set(future.result().message)
                except Exception as exc:
                    self.status.set(str(exc))
            self.clients[name].call_async(Trigger.Request()).add_done_callback(completed)

    def target_matches_plan(self):
        try:
            values = [float(variable.get()) for variable in self.target]
            goal = self.state.get('goal_enu', [])
            return (len(goal) == 3 and all(map(math.isfinite, values))
                and math.dist(values, goal) < 1e-6)
        except (ValueError, TypeError, tk.TclError):
            return False

    def poll(self):
        if self.owned() and self.connect():
            import rclpy
            for _ in range(8):
                rclpy.spin_once(self.node, timeout_sec=0.)
            if self.ready():
                self.status.set(self.state.get('error') or 'Planner: '+self.state.get('state', '')+
                    ' · '+str(self.state.get('waypoints', 0))+' waypoints')
        else:
            self.state, self.position = {}, None
        self.plan_button.state(['!disabled'] if self.ready() and self.state.get('state') != 'planning' else ['disabled'])
        self.execute_button.state(['!disabled'] if self.ready() and self.state.get('state') == 'planned'
            and self.target_matches_plan() else ['disabled'])
        self.stop_button.state(['!disabled'] if self.owned() else ['disabled'])
        if self.winfo_exists():
            self.after(200, self.poll)
