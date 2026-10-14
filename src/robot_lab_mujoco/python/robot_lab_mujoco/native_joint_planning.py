"""Real trajectory actions over an owned native articulation controller.

MuJoCo owns measured state and contact physics. Targets go through the original
bounded position actuators; only a private MjData is used for collision probes.
"""
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_lab_utils.native_arm_trajectory import checked_arm_path
from robot_lab_utils.native_planning_description import arm_contract, compiled_arm_sha256


def seconds(duration):
    return duration.sec+duration.nanosec*1e-9


class NativeJointPlanning:
    def __init__(self, articulation, configuration):
        self.owner = articulation
        self.node, self.model, self.data, self.mj = articulation.node, articulation.model, articulation.data, articulation.node.mujoco
        exported = json.loads(Path(configuration).read_text())
        contract = arm_contract(self.model, exported['source_frame'], self.node.robot_joints)
        for key in ('joint_names', 'actuator_indices', 'joint_indices', 'limits', 'margins', 'tip', 'max_velocity', 'units'):
            if contract[key] != exported[key]:
                raise ValueError('Executed native arm differs from the planning export: '+key)
        if (Path(exported['native_mjcf']).resolve() != Path(self.node.get_parameter('native_mjcf').value).resolve()
                or exported['source_sha256'] != articulation.source_sha256):
            raise ValueError('Native arm planner/controller source identity differs')
        if (self.node.robot_bodies != exported['robot_bodies'] or compiled_arm_sha256(self.model,
                self.node.robot_bodies, self.node.robot_joints, exported['robot_actuators']) != exported['compiled_sha256']):
            raise ValueError('Compiled native kinematics/collision/actuators differ from the planning export')
        self.contract = contract
        self.contract['source_sha256'] = exported['source_sha256']
        self.contract['export_sha256'] = hashlib.sha256(Path(configuration).read_bytes()).hexdigest()
        self.joints = np.asarray(contract['joint_indices'], dtype=int)
        self.qpos = self.model.jnt_qposadr[self.joints]
        self.dofs = self.model.jnt_dofadr[self.joints]
        self.actuators = np.asarray(contract['actuator_indices'], dtype=int)
        self.channels = np.asarray([int(np.flatnonzero(articulation.indices == actuator)[0]) for actuator in self.actuators])
        self.other_joints = np.asarray([j for j in range(self.node.robot_joints) if j not in self.joints], dtype=int)
        self.other_qpos = self.model.jnt_qposadr[self.other_joints]
        self.probe = self.mj.MjData(self.model)
        self.moving_bodies = set()
        for b in range(1, self.node.robot_bodies):
            body = b
            while body:
                start, count = int(self.model.body_jntadr[body]), int(self.model.body_jntnum[body])
                if set(range(start, start+count)).intersection(self.joints):
                    self.moving_bodies.add(b)
                    break
                body = int(self.model.body_parentid[body])
        self.goal = self.plan = self.future = None
        self.reserved = False
        self.offset = np.zeros(len(self.joints))
        self.server = ActionServer(self.node, FollowJointTrajectory, '/native_arm/follow_joint_trajectory',
            self.execute, goal_callback=self.accept, handle_accepted_callback=self.accepted,
            cancel_callback=lambda _: CancelResponse.ACCEPT)

    def collision(self, data):
        return any(contact.dist < -.0001 and any(int(self.model.geom_bodyid[g]) in self.moving_bodies
            for g in contact.geom) for contact in data.contact[:data.ncon])

    def validate(self, request):
        trajectory = request.trajectory
        if seconds(trajectory.header.stamp) or request.multi_dof_trajectory.points or request.multi_dof_trajectory.joint_names:
            raise ValueError('Use a zero timestamp and fixed joint positions only')
        if (request.path_tolerance or request.goal_tolerance or request.component_path_tolerance
                or request.component_goal_tolerance or seconds(request.goal_time_tolerance)):
            raise ValueError('Native arm uses measured source-unit tolerances; custom tolerances are unsupported')
        points = []
        for point in trajectory.points:
            if point.velocities or point.accelerations or point.effort:
                raise ValueError('Use position-only waypoints; the adapter bounds cubic targets')
            points.append((seconds(point.time_from_start), point.positions))
        plan = checked_arm_path(trajectory.joint_names, self.contract, points, self.data.qpos[self.qpos])
        # Check all requested edges against the current native collision plant,
        # including a caller that bypasses MoveIt. This does not predict moving
        # obstacles: fresh current/lookahead checks continue during execution.
        self.mj.mj_copyData(self.probe, self.model, self.data)
        checked, deadline = 0, time.monotonic()+.2
        for a, b in zip(plan.positions[:-1], plan.positions[1:]):
            step = np.where(np.asarray(self.contract['units']) == 'm', .002, .025)
            count = max(1, int(np.ceil(np.max(np.abs(b-a)/step))))
            if count > 400:
                raise ValueError('Arm collision edge exceeds the bounded inspection budget')
            for fraction in np.linspace(0., 1., count+1):
                checked += 1
                if checked > 5000 or time.monotonic() > deadline:
                    raise ValueError('Native collision inspection budget exceeded; use a shorter path or simpler scene')
                self.probe.qpos[self.qpos] = a+fraction*(b-a)
                self.mj.mj_forward(self.model, self.probe)
                if self.collision(self.probe):
                    raise ValueError('Native contact blocks the arm trajectory')
        return plan

    def accept(self, request):
        owner = self.owner
        if (self.reserved or owner.fault or owner.sequence is not None
                or owner.servo and owner.servo.active() or time.monotonic()-owner.last_heartbeat > .8
                or self.collision(self.data) or np.max(np.abs(owner.target-owner.measured_controls())) > .03):
            return GoalResponse.REJECT
        try:
            self.pending = self.validate(request)
        except (ValueError, TypeError, KeyError) as exc:
            self.node.get_logger().warn('Native arm trajectory rejected: '+str(exc))
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    def accepted(self, goal):
        self.goal, self.plan, self.start = goal, self.pending, float(self.data.time)
        self.other_start = self.data.qpos[self.other_qpos].copy()
        self.started_wall = self.last_progress_wall = time.monotonic()
        self.previous_time = float(self.data.time)
        self.checked_at = -float('inf')
        self.future = Future()
        goal._native_arm_future = self.future
        self.owner.status = 'executing collision-checked arm trajectory'
        goal.execute()

    async def execute(self, goal):
        return await goal._native_arm_future

    def finish(self, outcome='aborted', code=-4, reason='Stopped; holding measured articulation'):
        goal = self.goal
        self.goal = self.plan = None
        self.reserved = False
        if goal:
            {'succeeded': goal.succeed, 'canceled': goal.canceled, 'aborted': goal.abort}[outcome]()
            self.future.set_result(FollowJointTrajectory.Result(error_code=code, error_string=reason))
        if outcome != 'succeeded':
            self.owner.target = self.owner.measured_controls()
            self.owner.commanded = self.owner.target.copy()
        self.owner.status = reason

    def before_step(self):
        if not self.goal:
            return
        if self.goal.is_cancel_requested:
            self.finish('canceled', 0, 'Canceled; holding measured articulation')
            return
        if (self.owner.fault or time.monotonic()-self.owner.last_heartbeat > .8 or self.collision(self.data)
                or np.any(np.abs(self.data.qpos[self.other_qpos]-self.other_start) > .01)):
            self.finish(reason='Contact, heartbeat loss or other joint motion interrupted the arm path')
            return
        now = time.monotonic()
        if self.data.time > self.previous_time:
            self.last_progress_wall, self.previous_time = now, float(self.data.time)
        if now-self.last_progress_wall > 2. or now-self.started_wall > 600.:
            self.finish(reason='Native arm simulation/progress deadline exceeded')
            return
        target, _ = self.plan.sample(self.data.time-self.start)
        # A 50 ms target lookahead at bounded velocity checks the evolving
        # environment without touching the live qpos or adding body forces.
        if self.data.time-getattr(self, 'checked_at', -float('inf')) >= .02:
            self.checked_at = float(self.data.time)
            candidate, _ = self.plan.sample(self.data.time-self.start+.05)
            self.mj.mj_copyData(self.probe, self.model, self.data)
            self.probe.qpos[self.qpos] = candidate
            self.mj.mj_forward(self.model, self.probe)
            if self.collision(self.probe):
                self.finish(reason='Predicted native contact interrupted the arm path')
                return
        self.owner.target[self.channels] = target

    def apply_bias(self):
        # Bounded feedforward through authored unit-gear position actuators.
        # Original gain, passive gravity compensation and force clamps remain.
        bias = self.data.qfrc_bias[self.dofs]-self.data.qfrc_gravcomp[self.dofs]
        cap = np.where(np.asarray(self.contract['units']) == 'm', .003, .03)
        self.offset = np.clip(bias/self.model.actuator_gainprm[self.actuators, 0], -cap, cap)
        if not np.all(np.isfinite(self.offset)):
            self.offset[:] = 0.
        self.data.ctrl[self.actuators] = np.clip(self.owner.commanded[self.channels]+self.offset,
            self.model.actuator_ctrlrange[self.actuators, 0], self.model.actuator_ctrlrange[self.actuators, 1])

    def after_step(self):
        if not self.goal:
            return
        measured = self.data.qpos[self.qpos]
        target, _ = self.plan.sample(self.data.time-self.start)
        position_tolerance = np.where(np.asarray(self.contract['units']) == 'm', .003, .03)
        if self.collision(self.data) or np.any(np.abs(measured-target) > 4*position_tolerance):
            self.finish(reason='Native contact or measured path error interrupted the arm trajectory')
        elif self.data.time-self.start >= self.plan.times[-1]:
            rates = np.where(np.asarray(self.contract['units']) == 'm', .005, .03)
            if np.all(np.abs(measured-target) <= position_tolerance) and np.all(np.abs(self.data.qvel[self.dofs]) < rates):
                self.finish('succeeded', 0, 'Measured arm waypoint reached; object mission unassessed')
            elif self.data.time-self.start > self.plan.times[-1]+3.:
                self.finish(code=-5, reason='Measured arm goal did not settle within source-unit tolerances')

    def state(self):
        return dict(self.contract, plant_id=self.owner.plant_id, controller='native_arm_planning',
            positions=self.data.qpos[self.qpos].tolist(), velocities=self.data.qvel[self.dofs].tolist(),
            busy=bool(self.reserved or self.owner.sequence is not None or self.owner.servo and self.owner.servo.active()),
            contact_blocked=bool(self.collision(self.data)), status=self.owner.status,
            bias_compensation=self.offset.tolist())

    def publish_feedback(self):
        if self.goal:
            desired, velocity = self.plan.sample(self.data.time-self.start)
            actual = self.data.qpos[self.qpos]
            feedback = FollowJointTrajectory.Feedback(joint_names=self.contract['joint_names'],
                desired=JointTrajectoryPoint(positions=desired.tolist(), velocities=velocity.tolist()),
                actual=JointTrajectoryPoint(positions=actual.tolist(), velocities=self.data.qvel[self.dofs].tolist()),
                error=JointTrajectoryPoint(positions=(desired-actual).tolist()))
            feedback.header.stamp.sec = int(self.data.time)
            feedback.header.stamp.nanosec = int((self.data.time-int(self.data.time))*1e9)
            self.goal.publish_feedback(feedback)
