"""MoveIt plans executed through the owned native Panda position action."""
import math
import time
import tkinter as tk
from tkinter import ttk


class CartesianControls(ttk.LabelFrame):
    def __init__(self, arm):
        super().__init__(arm.body, text='Cartesian motion — MoveIt', padding=8)
        self.arm = arm
        self.grid(row=10, column=0, columnspan=5, sticky='ew', pady=8)
        self.status = tk.StringVar(value='Cartesian planning is not enabled for this profile.')
        self.offset = []
        for i, axis in enumerate('xyz'):
            ttk.Label(self, text='Δ'+axis+' (m)').grid(row=i, column=0)
            value = tk.DoubleVar(value=.05 if axis == 'z' else 0.)
            self.offset.append(value)
            ttk.Spinbox(self, textvariable=value, from_=-.25, to=.25, increment=.01,
                        width=8).grid(row=i, column=1, padx=4)
            value.trace_add('write', lambda *_: self.invalidate())
        self.plan_button = ttk.Button(self, text='Plan', command=self.plan)
        self.execute_button = ttk.Button(self, text='Execute Plan', command=self.execute)
        self.plan_button.grid(row=3, column=0, sticky='ew', pady=8)
        self.execute_button.grid(row=3, column=1, sticky='ew', padx=4)
        ttk.Label(self, textvariable=self.status, wraplength=285).grid(row=4, column=0, columnspan=2, sticky='w')
        self.connected = False
        self.joints, self.received = {}, -math.inf
        self.scene, self.scene_received = {}, -math.inf
        self.generation, self.planning = 0, False
        self.points = None
        self.start = None
        self.was_owned = False
        self.refresh()

    def owned(self):
        process = self.arm.app.process
        return bool(self.arm.owned() and process and 'arm_planning:=moveit' in process.args)

    def invalidate(self):
        self.generation += 1
        self.planning = False
        self.points = None
        self.execute_button.state(['disabled'])

    def refresh(self):
        self.invalidate()
        self.plan_button.state(['disabled'])

    def connect(self):
        if self.connected:
            return True
        try:
            from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
        except ImportError:
            self.status.set('Install ROS 2 MoveIt messages in the GUI environment.')
            return False
        node = self.arm.node
        self.fk = node.create_client(GetPositionFK, '/compute_fk')
        self.ik = node.create_client(GetPositionIK, '/compute_ik')
        self.planner = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.joint_subscription = node.create_subscription(JointState, '/joint_states', self.receive_joints, 10)
        self.scene_subscription = node.create_subscription(String, '/arm/planning_scene_status', self.receive_scene, 10)
        self.connected = True
        return True

    def receive_joints(self, message):
        names = self.arm.state['joint_names'] if self.arm.state else ['joint'+str(i) for i in range(1, 8)]
        measured = dict(zip(message.name, message.position))
        if all(n in measured for n in names+['finger_joint1', 'finger_joint2']):
            self.joints, self.received = measured, time.monotonic()

    def receive_scene(self, message):
        import json
        try:
            self.scene = json.loads(message.data)
            self.scene_received = time.monotonic()
        except (ValueError, TypeError):
            return

    def ready(self):
        return bool(self.owned() and self.arm.ready() and self.connected and self.scene.get('ready')
                    and time.monotonic()-self.scene_received < 2.
                    and time.monotonic()-self.received < .5
                    and not self.arm.state['busy'] and not self.arm.sending
                    and not self.arm.state['contact_blocked'])

    def unchanged(self):
        return bool(self.start and all(abs(self.joints.get(n, math.inf)-q) < .01
                                     for n, q in self.start.items()))

    def poll(self):
        owned = self.owned()
        if self.was_owned and not owned:
            self.invalidate()
            self.joints, self.scene = {}, {}
        if owned:
            self.connect()
            if not self.scene.get('ready'):
                self.status.set(self.scene.get('error', 'Waiting for the selected-world collision scene…'))
        else:
            process = self.arm.app.process
            self.status.set('Cartesian planning is unavailable with the optional grasp fixture; use joint and hand controls.'
                if process and 'grasp_fixture:=true' in process.args else
                'Select a qualified Panda planning profile and Run from Launch.')
        if self.planning and time.monotonic()-self.planning_started > 8:
            self.invalidate()
            self.status.set('Planning timed out; plan again.')
        ready = self.ready()
        if self.points and (not ready or not self.unchanged() or time.monotonic()-self.planned_at > 10):
            self.invalidate()
            self.status.set('Plan expired or robot state changed; plan again.')
        self.plan_button.state(['!disabled'] if ready and not self.planning else ['disabled'])
        self.execute_button.state(['!disabled'] if ready and self.points and self.unchanged() else ['disabled'])
        self.was_owned = owned

    def plan(self):
        if (not self.ready() or self.planning or not self.fk.service_is_ready()
                or not self.ik.service_is_ready() or not self.planner.service_is_ready()):
            return
        try:
            offset = [float(v.get()) for v in self.offset]
            if not all(math.isfinite(v) and abs(v) <= .25 for v in offset) or max(abs(v) for v in offset) < .001:
                raise ValueError('Set a finite offset within ±0.25 m and at least 1 mm.')
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))
            return
        from moveit_msgs.srv import GetPositionFK
        self.invalidate()
        generation = self.generation
        self.planning = True
        self.planning_started = time.monotonic()
        self.start = dict(self.joints)
        request = GetPositionFK.Request()
        request.header.frame_id = 'native_world'
        request.fk_link_names = ['panda_tcp']
        request.robot_state.joint_state.name = list(self.start)
        request.robot_state.joint_state.position = list(self.start.values())
        self.status.set('Planning from measured joints with the selected-world collision scene…')
        self.fk.call_async(request).add_done_callback(lambda future: self.got_fk(future, generation, offset, request.robot_state))

    def got_fk(self, future, generation, offset, state):
        if generation != self.generation:
            return
        if not self.ready() or not self.unchanged():
            self.invalidate()
            self.status.set('Robot or scene state changed during planning; plan again.')
            return
        from geometry_msgs.msg import Pose
        from moveit_msgs.srv import GetPositionIK
        answer = future.result()
        if answer.error_code.val != 1 or not answer.pose_stamped:
            self.planning = False
            self.status.set('MoveIt could not compute the current native TCP pose.')
            return
        pose = answer.pose_stamped[0].pose
        for axis, delta in zip('xyz', offset):
            setattr(pose.position, axis, getattr(pose.position, axis)+delta)
        self.target = pose
        # A pose-only OMPL goal samples redundant IK branches, including far
        # elbow configurations for millimetre offsets. Seed KDL with measured
        # joints first, then plan to that collision-valid joint solution. The
        # path still goes through the real OMPL/FCL planning scene.
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name, ik.ik_link_name = 'panda_arm', 'panda_tcp'
        ik.robot_state, ik.avoid_collisions = state, True
        ik.pose_stamped.header.frame_id = 'native_world'
        ik.pose_stamped.pose = pose
        ik.timeout.sec = 1
        self.ik.call_async(request).add_done_callback(lambda f: self.got_ik(f, generation, state))

    def got_ik(self, future, generation, state):
        if generation != self.generation:
            return
        if not self.ready() or not self.unchanged():
            self.invalidate()
            self.status.set('Robot or scene state changed during planning; plan again.')
            return
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.srv import GetMotionPlan
        answer = future.result()
        if answer.error_code.val != 1:
            self.planning = False
            self.status.set('No collision-free reachable target (MoveIt code %d).' % answer.error_code.val)
            return
        solution = dict(zip(answer.solution.joint_state.name, answer.solution.joint_state.position))
        names = self.arm.state['joint_names']
        if not all(n in solution and math.isfinite(solution[n]) for n in names):
            self.planning = False
            self.status.set('MoveIt returned an incomplete IK solution.')
            return
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name, motion.pipeline_id, motion.planner_id = 'panda_arm', 'ompl', 'RRTConnect'
        motion.start_state = state
        motion.goal_constraints = [Constraints(joint_constraints=[JointConstraint(
            joint_name=name, position=solution[name], tolerance_above=.001,
            tolerance_below=.001, weight=1.) for name in names])]
        motion.allowed_planning_time, motion.num_planning_attempts = 3., 1
        self.planner.call_async(request).add_done_callback(lambda f: self.got_plan(f, generation))

    def got_plan(self, future, generation):
        if generation != self.generation:
            return
        if not self.ready() or not self.unchanged():
            self.invalidate()
            self.status.set('Robot or scene state changed during planning; plan again.')
            return
        self.planning = False
        response = future.result().motion_plan_response
        if response.error_code.val != 1 or not response.trajectory.joint_trajectory.points:
            self.status.set('No collision-free reachable plan (MoveIt code %d).' % response.error_code.val)
            return
        from robot_lab_utils.arm_trajectory import retime_collision_checked_path
        try:
            trajectory = response.trajectory.joint_trajectory
            self.points = retime_collision_checked_path(trajectory.joint_names, self.arm.state['joint_names'],
                [p.positions for p in trajectory.points], self.arm.state['positions'], self.arm.state['limits'])
        except ValueError as exc:
            self.points = None
            self.status.set('Plan exceeds the native execution contract: '+str(exc))
            return
        self.planned_at = time.monotonic()
        self.status.set('Collision-checked plan: %d waypoints, %.2f simulation seconds. Execute when ready.' %
                        (len(self.points), self.points[-1][0]))

    def execute(self):
        if not self.ready() or not self.points or not self.unchanged() or time.monotonic()-self.planned_at > 10:
            self.invalidate()
            return
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm.state['joint_names']
        for duration, positions in self.points:
            point = JointTrajectoryPoint(positions=positions)
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration-int(duration))*1e9)
            goal.trajectory.points.append(point)
        self.invalidate()
        self.arm.sending = True
        self.arm.action.send_goal_async(goal).add_done_callback(self.arm.accepted)
        self.status.set('Executing the checked path through the native Panda actuators.')
