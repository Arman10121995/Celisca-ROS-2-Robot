"""Experimental fixed-arm MoveIt panel using measured native articulation.

Reuse the existing target editor, scene acknowledgements and plan expiry;
keep Panda's controller/panel contracts separate from generic native arms.
"""
import math
import time
import tkinter as tk
from tkinter import ttk

from .cartesian_controls import CartesianControls


class NativeCartesianControls(CartesianControls):
    def __init__(self, arm):
        super().__init__(arm)
        self.configure(text='Cartesian planning — experimental native arm')
        self.grid(row=2, column=0, columnspan=4, sticky='ew', pady=8)
        self.attach_button.master.grid_remove()
        for widget in self.grid_slaves(row=11):
            widget.grid_remove()
        self.instructions = self.grid_slaves(row=8)[0]

    def owned(self):
        return self.arm.owned()

    def fk_request(self, joints):
        from moveit_msgs.srv import GetPositionFK
        request = GetPositionFK.Request()
        request.header.frame_id = 'native_world'
        request.fk_link_names = [self.arm.state['tip']]
        request.robot_state.joint_state.name = list(joints)
        request.robot_state.joint_state.position = list(joints.values())
        return request

    def receive_joints(self, message):
        state = self.arm.state
        if not state or not self.arm.owned():
            return
        measured = dict(zip(message.name, message.position))
        names = state['all_joint_names']
        if all(name in measured and math.isfinite(measured[name]) for name in names):
            self.joints = {name: measured[name] for name in names}
            self.received = time.monotonic()

    def ready(self):
        return bool(super().ready() and self.scene.get('export_sha256') == self.arm.state['export_sha256'])

    def poll(self):
        super().poll()
        if not self.owned():
            self.status.set('Enable Native arm planning before Run, then select a small tool offset.')
        if self.arm.state:
            state = self.arm.state
            orientation = 'Position-only IK: orientation follows the arm motion.' if state['position_only_ik'] else 'The current tool orientation is retained.'
            self.instructions.configure(text=state['source_frame']+' in native_world metres. '+orientation+
                '\nThe ± buttons edit the target. Plan checks the path; Execute Plan moves the original actuators.')

    def got_fk(self, future, generation, offset, state):
        if generation != self.generation:
            return
        if not self.ready() or not self.unchanged():
            self.invalidate()
            self.status.set('Robot or scene changed during planning; plan again.')
            return
        from moveit_msgs.srv import GetPositionIK
        answer = future.result()
        if answer.error_code.val != 1 or not answer.pose_stamped:
            self.planning = False
            self.status.set('MoveIt could not compute this authored frame pose.')
            return
        pose = answer.pose_stamped[0].pose
        self.set_reference(pose, self.start)
        for axis, delta in zip('xyz', offset):
            setattr(pose.position, axis, getattr(pose.position, axis)+delta)
        self.target = pose
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name, ik.ik_link_name = self.arm.state['group'], self.arm.state['tip']
        ik.robot_state, ik.avoid_collisions = state, True
        ik.pose_stamped.header.frame_id, ik.pose_stamped.pose = 'native_world', pose
        ik.timeout.sec = 1
        self.ik.call_async(request).add_done_callback(lambda f: self.got_ik(f, generation, state))

    def got_ik(self, future, generation, state):
        if generation != self.generation:
            return
        if not self.ready() or not self.unchanged():
            self.invalidate()
            self.status.set('Robot or scene changed during planning; plan again.')
            return
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.srv import GetMotionPlan
        answer = future.result()
        if answer.error_code.val != 1:
            self.planning = False
            self.status.set('This target is unreachable or colliding (MoveIt code %d). '
                'Try a smaller step or adjust a joint first.' % answer.error_code.val)
            return
        solution = dict(zip(answer.solution.joint_state.name, answer.solution.joint_state.position))
        names = self.arm.state['joint_names']
        if not all(name in solution and math.isfinite(solution[name]) for name in names):
            self.planning = False
            self.status.set('MoveIt returned an incomplete arm IK solution.')
            return
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name, motion.pipeline_id, motion.planner_id = self.arm.state['group'], 'ompl', 'RRTConnect'
        motion.start_state = state
        motion.goal_constraints = [Constraints(joint_constraints=[JointConstraint(joint_name=name,
            position=solution[name], tolerance_above=.001, tolerance_below=.001, weight=1.) for name in names])]
        motion.allowed_planning_time, motion.num_planning_attempts = 3., 1
        self.planner.call_async(request).add_done_callback(lambda f: self.got_plan(f, generation))

    def got_plan(self, future, generation):
        if generation != self.generation:
            return
        if not self.ready() or not self.unchanged():
            self.invalidate()
            self.status.set('Robot or scene changed during planning; plan again.')
            return
        self.planning = False
        response = future.result().motion_plan_response
        if response.error_code.val != 1 or not response.trajectory.joint_trajectory.points:
            self.status.set('No collision-free plan (MoveIt code %d).' % response.error_code.val)
            return
        from robot_lab_utils.native_arm_trajectory import retime_arm_path
        try:
            trajectory = response.trajectory.joint_trajectory
            self.points = retime_arm_path(trajectory.joint_names, self.arm.state,
                [point.positions for point in trajectory.points], self.arm.state['positions'])
        except ValueError as exc:
            self.points = None
            self.status.set('Plan exceeds the source execution contract: '+str(exc))
            return
        self.planned_at = time.monotonic()
        self.status.set('Collision-checked native path: %d waypoints, %.2f simulation seconds. '
            'Physical mission qualification is pending.' % (len(self.points), self.points[-1][0]))

    def execute(self):
        if not self.ready() or not self.points or not self.unchanged() or time.monotonic()-self.planned_at > 10.:
            self.invalidate()
            return
        points = self.points
        self.invalidate()
        self.arm.send_waypoints(points)


class NativePlanningControls(ttk.Frame):
    def __init__(self, parent, articulation):
        super().__init__(parent)
        self.articulation, self.app = articulation, articulation.app
        self.body = self
        self.sending = False
        self.handle = None
        self.generation = 0
        self.action = None
        self.was_owned = False
        self.status_var = tk.StringVar(value='Native arm planning is experimental; enable it before Run.')
        ttk.Label(self, textvariable=self.status_var, wraplength=305).grid(row=0, column=0, columnspan=4, sticky='w')
        self.cancel_button = ttk.Button(self, text='Cancel planned motion', command=self.stop)
        self.cancel_button.grid(row=1, column=0, columnspan=4, sticky='ew', pady=4)
        self.cartesian = NativeCartesianControls(self)

    @property
    def node(self):
        return self.articulation.node

    @property
    def state(self):
        return (self.articulation.state or {}).get('planning')

    def selected(self):
        return self.articulation.selected() and bool(self.app._robot_config().get('native_arm_planning'))

    def owned(self):
        return bool(self.selected() and self.articulation.owned() and self.app.process
                    and 'arm_planning:=native_moveit' in self.app.process.args)

    def ready(self):
        return bool(self.owned() and self.articulation.ready() and self.state
            and self.state.get('plant_id') == self.articulation.state['plant_id']
            and self.state.get('source_frame') == self.app._robot_config()['native_arm_planning']['source_frame'])

    def poll(self):
        if not self.selected():
            self.grid_remove()
        else:
            self.grid()
        owned = self.owned()
        if self.was_owned and not owned:
            self.stop()
        if owned and self.node and self.action is None:
            from control_msgs.action import FollowJointTrajectory
            from rclpy.action import ActionClient
            self.action = ActionClient(self.node, FollowJointTrajectory, '/native_arm/follow_joint_trajectory')
        self.cartesian.poll()
        if self.ready():
            self.status_var.set(self.state['source_frame']+' · '+self.state['status']+' · validation pending')
        elif self.selected():
            self.status_var.set('Enable Native arm planning in Launch and Run this arm on MuJoCo Display.')
        self.cancel_button.state(['!disabled'] if self.owned() else ['disabled'])
        self.was_owned = owned

    def send_waypoints(self, points):
        if (not self.ready() or self.state['busy'] or self.sending or self.action is None
                or not self.action.server_is_ready()):
            self.status_var.set('Wait for the measured native arm trajectory server.')
            return
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.state['joint_names']
        for duration, positions in points:
            point = JointTrajectoryPoint(positions=positions)
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int(round((duration-int(duration))*1e9))
            if point.time_from_start.nanosec == 1000000000:
                point.time_from_start.sec += 1
                point.time_from_start.nanosec = 0
            goal.trajectory.points.append(point)
        self.sending = True
        generation = self.generation
        self.action.send_goal_async(goal).add_done_callback(lambda f: self.accepted(f, generation))
        self.cartesian.status.set('Executing through the original native position actuators.')

    def accepted(self, future, generation):
        try:
            handle = future.result()
            if generation != self.generation or not self.owned():
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            self.sending = False
            if not handle.accepted:
                self.status_var.set('Native trajectory rejected; inspect the source/contact limits in the launch log.')
                self.cartesian.status.set(self.status_var.get())
                return
            self.handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.finished(f, generation))
        except Exception as exc:
            if generation == self.generation:
                self.sending = False
                self.status_var.set('Native trajectory request failed: '+str(exc))

    def finished(self, future, generation):
        if generation != self.generation:
            return
        self.handle = None
        try:
            message = future.result().result.error_string
            self.status_var.set(message)
            self.cartesian.status.set(message)
        except Exception as exc:
            self.status_var.set('Native trajectory result unavailable: '+str(exc))

    def stop(self):
        self.generation += 1
        self.sending = False
        self.cartesian.invalidate()
        self.cartesian.clear_reference()
        if self.handle:
            self.handle.cancel_goal_async()
            self.handle = None
        if self.owned() and self.node and self.articulation.stop_client.service_is_ready():
            from std_srvs.srv import Trigger
            self.articulation.stop_client.call_async(Trigger.Request())
