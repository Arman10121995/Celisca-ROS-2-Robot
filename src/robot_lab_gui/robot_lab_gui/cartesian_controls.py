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
        self.pose_display = tk.StringVar(value='Tool position: use the measured pose below.')
        self.target_display = tk.StringVar(value='Target position: not yet measured.')
        self.reference = None
        self.reference_joints = {}
        self.reading_pose = False
        self.use_pose_button = ttk.Button(self, text='Use current tool pose', command=self.use_current_pose)
        self.use_pose_button.grid(row=0, column=0, columnspan=4, sticky='ew', pady=(0, 4))
        ttk.Label(self, textvariable=self.pose_display, wraplength=285).grid(
            row=1, column=0, columnspan=4, sticky='w')
        ttk.Label(self, textvariable=self.target_display, wraplength=285).grid(
            row=2, column=0, columnspan=4, sticky='w', pady=(0, 6))
        self.step = tk.DoubleVar(value=.01)
        ttk.Label(self, text='Target step (m)').grid(row=3, column=0, sticky='w')
        ttk.Spinbox(self, textvariable=self.step, from_=.001, to=.05, increment=.001,
                    width=8).grid(row=3, column=1, sticky='w', padx=4)
        self.offset = []
        self.target_buttons = []
        for i, axis in enumerate('xyz'):
            row = i+4
            ttk.Label(self, text='Δ'+axis.upper()+' (m)').grid(row=row, column=0, sticky='w')
            value = tk.DoubleVar(value=0.)
            self.offset.append(value)
            ttk.Spinbox(self, textvariable=value, from_=-.25, to=.25, increment=.01,
                        width=8).grid(row=row, column=1, padx=4)
            value.trace_add('write', self.target_edited)
            for column, sign, label in ((2, -1, '−'), (3, 1, '+')):
                button = ttk.Button(self, text=label, width=2,
                    command=lambda i=i, sign=sign: self.step_target(i, sign))
                button.grid(row=row, column=column, padx=2)
                self.target_buttons.append(button)
        self.plan_button = ttk.Button(self, text='Plan', command=self.plan)
        self.execute_button = ttk.Button(self, text='Execute Plan', command=self.execute)
        self.plan_button.grid(row=7, column=0, sticky='ew', pady=8)
        self.execute_button.grid(row=7, column=1, columnspan=3, sticky='ew', padx=4)
        ttk.Label(self, text='X/Y/Z follow native_world axes, in metres. Tool orientation stays fixed.\n'
                  'The ± buttons only edit the target. Review Plan, then Execute Plan to move.',
                  wraplength=285, style='Muted.TLabel').grid(row=8, column=0, columnspan=4, sticky='w')
        ttk.Label(self, textvariable=self.status, wraplength=285).grid(row=9, column=0, columnspan=4, sticky='w', pady=6)
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
        self.reading_pose = False
        self.points = None
        self.execute_button.state(['disabled'])

    def refresh(self):
        self.invalidate()
        self.clear_reference()
        self.plan_button.state(['disabled'])
        for button in [self.use_pose_button]+self.target_buttons:
            button.state(['disabled'])

    def clear_reference(self):
        self.reference = None
        self.reference_joints = {}
        self.pose_display.set('Tool position: fresh measured pose required.')
        self.target_display.set('Target position: use current tool pose.')

    def target_edited(self, *_):
        self.invalidate()
        self.update_target_display()

    def update_target_display(self):
        if self.reference is None:
            return
        try:
            offset = [float(value.get()) for value in self.offset]
            if not all(math.isfinite(value) and abs(value) <= .25 for value in offset):
                raise ValueError
            self.target_display.set('Target XYZ: '+', '.join('%.3f' % (p+d)
                for p, d in zip(self.reference, offset))+' m')
        except (ValueError, tk.TclError):
            self.target_display.set('Target offset must be finite and within ±0.25 m.')

    def step_target(self, index, sign):
        if not self.ready() or self.planning or self.reading_pose:
            self.status.set('Wait for fresh measured joints and an idle planning scene.')
            return
        try:
            step = float(self.step.get())
            target = float(self.offset[index].get())+sign*step
            if not math.isfinite(step) or not .001 <= step <= .05 or not math.isfinite(target) or abs(target) > .25:
                raise ValueError('Use a 1–50 mm step and keep the offset within ±0.25 m.')
            self.offset[index].set(round(target, 6))
            self.status.set('Target updated; Plan checks reachability and collisions before execution.')
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))

    @staticmethod
    def fk_request(joints):
        from moveit_msgs.srv import GetPositionFK
        request = GetPositionFK.Request()
        request.header.frame_id = 'native_world'
        request.fk_link_names = ['panda_tcp']
        request.robot_state.joint_state.name = list(joints)
        request.robot_state.joint_state.position = list(joints.values())
        return request

    def use_current_pose(self):
        if not self.ready() or self.planning or self.reading_pose or not self.fk.service_is_ready():
            self.status.set('Wait for fresh measured joints and the MoveIt FK service.')
            return
        self.invalidate()
        self.clear_reference()
        for value in self.offset:
            value.set(0.)
        snapshot = dict(self.joints)
        generation = self.generation
        self.reading_pose = True
        self.pose_started = time.monotonic()
        self.status.set('Reading the current tool pose; no motion is commanded.')
        self.fk.call_async(self.fk_request(snapshot)).add_done_callback(
            lambda future: self.got_current_pose(future, generation, snapshot))

    def set_reference(self, pose, joints):
        self.reference = tuple(getattr(pose.position, axis) for axis in 'xyz')
        self.reference_joints = dict(joints)
        self.pose_display.set('Tool XYZ: '+', '.join('%.3f' % p for p in self.reference)+' m')
        self.update_target_display()

    def got_current_pose(self, future, generation, snapshot):
        if generation != self.generation:
            return
        self.reading_pose = False
        if not self.ready() or not all(abs(self.joints.get(n, math.inf)-q) < .01 for n, q in snapshot.items()):
            self.clear_reference()
            self.status.set('Robot state changed; read the current tool pose again.')
            return
        try:
            answer = future.result()
            if answer.error_code.val != 1 or not answer.pose_stamped:
                raise ValueError('MoveIt could not compute the measured tool pose.')
            pose = answer.pose_stamped[0].pose
            if not all(math.isfinite(getattr(pose.position, axis)) for axis in 'xyz'):
                raise ValueError('MoveIt returned an invalid tool position.')
            self.set_reference(pose, snapshot)
            self.status.set('Current pose selected. Use small ± axis steps, then Plan and Execute Plan.')
        except Exception as exc:
            self.clear_reference()
            self.status.set('Could not read current tool pose: '+str(exc))

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
            self.clear_reference()
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
        if self.reading_pose and time.monotonic()-self.pose_started > 5:
            self.invalidate()
            self.status.set('Tool-pose request timed out; try again.')
        ready = self.ready()
        if self.reference is not None and (not ready or not all(
                abs(self.joints.get(n, math.inf)-q) < .01 for n, q in self.reference_joints.items())):
            self.clear_reference()
        if self.points and (not ready or not self.unchanged() or time.monotonic()-self.planned_at > 10):
            self.invalidate()
            self.status.set('Plan expired or robot state changed; plan again.')
        editable = ready and not self.planning and not self.reading_pose
        self.plan_button.state(['!disabled'] if editable else ['disabled'])
        for button in [self.use_pose_button]+self.target_buttons:
            button.state(['!disabled'] if editable else ['disabled'])
        self.execute_button.state(['!disabled'] if ready and self.points and self.unchanged() else ['disabled'])
        self.was_owned = owned

    def plan(self):
        if (not self.ready() or self.planning or self.reading_pose or not self.fk.service_is_ready()
                or not self.ik.service_is_ready() or not self.planner.service_is_ready()):
            return
        try:
            offset = [float(v.get()) for v in self.offset]
            if not all(math.isfinite(v) and abs(v) <= .25 for v in offset) or max(abs(v) for v in offset) < .001:
                raise ValueError('Choose a small ± axis step (at least 1 mm, within ±0.25 m).')
        except (ValueError, tk.TclError) as exc:
            self.status.set(str(exc))
            return
        self.invalidate()
        generation = self.generation
        self.planning = True
        self.planning_started = time.monotonic()
        self.start = dict(self.joints)
        request = self.fk_request(self.start)
        self.status.set('Planning from measured joints with the selected-world collision scene…')
        self.fk.call_async(request).add_done_callback(lambda future: self.got_fk(future, generation, offset, request.robot_state))

    def got_fk(self, future, generation, offset, state):
        if generation != self.generation:
            return
        if not self.ready() or not self.unchanged():
            self.invalidate()
            self.status.set('Robot or scene state changed during planning; plan again.')
            return
        from moveit_msgs.srv import GetPositionIK
        answer = future.result()
        if answer.error_code.val != 1 or not answer.pose_stamped:
            self.planning = False
            self.status.set('MoveIt could not compute the current native TCP pose.')
            return
        pose = answer.pose_stamped[0].pose
        self.set_reference(pose, self.start)
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
            self.status.set('Target is unreachable or colliding with the current tool orientation '
                '(MoveIt code %d). Try a smaller step or adjust a joint first.' % answer.error_code.val)
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
