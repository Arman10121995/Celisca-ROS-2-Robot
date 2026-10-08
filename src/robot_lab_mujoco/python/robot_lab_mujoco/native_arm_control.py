"""Position-only Panda action over the pinned model's real PD actuators.

The simulator owns stepping and all state. ROS callbacks only replace bounded
actuator targets. Planning and predictive collision checking are separate.
"""
import json
import time

import numpy as np
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_lab_utils.arm_trajectory import position_trajectory


def seconds(duration):
    return duration.sec + duration.nanosec*1e-9


class NativePandaControl:
    def __init__(self, node):
        self.node, self.model, self.data, self.mj = node, node.model, node.data, node.mujoco
        self.names = ['joint'+str(i) for i in range(1, 8)]
        self.joints = np.array([self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_JOINT, n)
                                for n in self.names])
        self.actuators = np.array([self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_ACTUATOR,
                                                    'actuator'+str(i)) for i in range(1, 8)])
        if np.any(self.joints < 0) or np.any(self.actuators < 0):
            raise ValueError('Panda control requires the native Panda joint/actuator model')
        if not np.array_equal(self.model.actuator_trnid[self.actuators, 0], self.joints):
            raise ValueError('Panda actuator-to-joint mapping differs from the pinned model')
        self.position_gains = self.model.actuator_gainprm[self.actuators, 0].copy()
        if (not np.all(np.isfinite(self.position_gains)) or np.any(self.position_gains <= 0)
                or not np.allclose(self.model.actuator_biasprm[self.actuators, 1], -self.position_gains)
                or not np.allclose(self.model.actuator_gear[self.actuators, 0], 1.)
                or not np.allclose(self.model.actuator_gear[self.actuators, 1:], 0.)):
            raise ValueError('Panda feedforward requires the original unit-gear position actuators')
        self.control_offset = np.zeros(7)
        if np.any(self.model.jnt_type[:node.robot_joints] == int(self.mj.mjtJoint.mjJNT_FREE)):
            raise ValueError('This controller requires a fixed-base Panda')
        self.qpos = self.model.jnt_qposadr[self.joints]
        self.dofs = self.model.jnt_dofadr[self.joints]
        self.limits = self.model.jnt_range[self.joints].copy()
        self.home = self.data.qpos[self.qpos].copy()
        self.target = self.home.copy()
        self.plan = None
        self.goal = None
        self.future = None
        self.reserved = False
        self.last_heartbeat = -float('inf')
        self.status = 'holding'
        self.status_pub = node.create_publisher(String, '/arm/status', 10)
        node.create_subscription(Empty, '/arm/heartbeat', self.heartbeat, 10)
        node.create_service(Trigger, '/arm/stop', self.stop)
        self.server = ActionServer(node, FollowJointTrajectory, '/arm/follow_joint_trajectory',
                                   self.execute, goal_callback=self.accept,
                                   handle_accepted_callback=self.accepted,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT)

    def heartbeat(self, _message):
        self.last_heartbeat = time.monotonic()

    def contact_blocked(self):
        for contact in self.data.contact[:self.data.ncon]:
            bodies = [int(self.model.geom_bodyid[g]) for g in contact.geom]
            gripper = getattr(self.node, 'gripper', None)
            if (gripper is not None and getattr(self.node, 'object_body', -1) in bodies
                    and any(body in gripper.finger_bodies for body in bodies)):
                continue
            # The fixed base may rest on a floor. Moving arm/environment
            # contacts stop the trajectory; original native collision filters
            # retain Panda's self-contact exclusions and finger coupling.
            if any(2 <= body < self.node.robot_bodies for body in bodies) and any(
                    body == 0 or body >= self.node.robot_bodies for body in bodies):
                return True
        return False

    def validate(self, request):
        trajectory = request.trajectory
        if seconds(trajectory.header.stamp) != 0:
            raise ValueError('Scheduled timestamps are unsupported; use a zero header stamp')
        if request.multi_dof_trajectory.points or request.multi_dof_trajectory.joint_names:
            raise ValueError('Only fixed-base joint positions are supported')
        if request.component_path_tolerance or request.component_goal_tolerance:
            raise ValueError('Multi-component tolerances are unsupported')
        points = []
        for point in trajectory.points:
            if point.velocities or point.accelerations or point.effort:
                raise ValueError('Supply positions only; this server generates bounded cubic segments')
            points.append((seconds(point.time_from_start), point.positions))
        plan = position_trajectory(trajectory.joint_names, self.names, points,
                                   self.data.qpos[self.qpos], self.limits)
        path, goal = np.full(7, .12), np.full(7, .03)
        for source, destination in [(request.path_tolerance, path), (request.goal_tolerance, goal)]:
            seen = set()
            for tolerance in source:
                if tolerance.name not in self.names or tolerance.name in seen:
                    raise ValueError('Tolerance names must be unique arm joints')
                seen.add(tolerance.name)
                if tolerance.velocity != 0 or tolerance.acceleration != 0:
                    raise ValueError('Only position tolerances are supported')
                value = tolerance.position
                if not np.isfinite(value) or value < 0 or value > .2:
                    raise ValueError('Position tolerance must be finite and within [0, 0.2] rad')
                if value:
                    destination[self.names.index(tolerance.name)] = value
        grace = seconds(request.goal_time_tolerance)
        if not np.isfinite(grace) or not 0 <= grace <= 3:
            raise ValueError('Goal time tolerance must be within [0, 3] seconds')
        return plan, path, goal, grace or 1.

    def accept(self, request):
        if (self.reserved or getattr(getattr(self.node, 'gripper', None), 'reserved', False)
                or time.monotonic()-self.last_heartbeat > .8 or self.contact_blocked()):
            self.node.get_logger().warn('Arm goal rejected: busy, missing heartbeat or arm/environment contact')
            return GoalResponse.REJECT
        try:
            self.pending = self.validate(request)
        except ValueError as error:
            self.node.get_logger().warn('Arm goal rejected: '+str(error))
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    def accepted(self, goal):
        self.goal = goal
        self.plan, self.path_tolerance, self.goal_tolerance, self.grace = self.pending
        self.start = self.data.time
        self.future = Future()
        goal._arm_future = self.future
        self.status = 'executing'
        goal.execute()

    async def execute(self, goal):
        return await goal._arm_future

    def finish(self, outcome, code, message):
        if outcome != 'succeeded':
            self.target = self.data.qpos[self.qpos].copy()
        self.status = message
        if self.goal:
            {'succeeded': self.goal.succeed, 'canceled': self.goal.canceled,
             'aborted': self.goal.abort}[outcome]()
            result = FollowJointTrajectory.Result(error_code=code, error_string=message)
            self.future.set_result(result)
        self.goal = None
        self.plan = None
        self.reserved = False

    def stop(self, _request, response):
        self.finish('aborted', FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                    'stopped; holding measured position')
        response.success, response.message = True, self.status
        return response

    def before_step(self):
        if self.goal:
            if self.goal.is_cancel_requested:
                self.finish('canceled', 0, 'canceled; holding measured position')
            elif time.monotonic()-self.last_heartbeat > .8:
                self.finish('aborted', -4, 'heartbeat lost; holding measured position')
            elif self.contact_blocked():
                self.finish('aborted', -4, 'arm/environment contact; holding measured position')
            else:
                self.target, _ = self.plan.sample(self.data.time-self.start)
        # Feed model gravity/Coriolis bias through the original PD actuator:
        # gain * (target + bias/gain - measured), with unchanged force limits.
        # The bounded offset prevents gravity sag from consuming a 1 cm TCP
        # step. No generalized/body force or joint position is written. Avoid
        # double compensation if the model already supplies passive gravcomp.
        bias = self.data.qfrc_bias[self.dofs]-self.data.qfrc_gravcomp[self.dofs]
        self.control_offset = (np.clip(bias/self.position_gains, -.03, .03)
                               if np.all(np.isfinite(bias)) else np.zeros(7))
        self.data.ctrl[self.actuators] = np.clip(self.target+self.control_offset,
            self.model.actuator_ctrlrange[self.actuators, 0],
            self.model.actuator_ctrlrange[self.actuators, 1])

    def after_step(self):
        if not self.goal:
            return
        elapsed = self.data.time-self.start
        actual, velocity = self.data.qpos[self.qpos], self.data.qvel[self.dofs]
        if self.contact_blocked():
            self.finish('aborted', -4, 'arm/environment contact; holding measured position')
        elif np.any(np.abs(actual-self.target) > self.path_tolerance):
            self.finish('aborted', -4, 'path tolerance violated; holding measured position')
        elif elapsed >= self.plan.times[-1]:
            if np.all(np.abs(actual-self.target) <= self.goal_tolerance) and np.max(np.abs(velocity)) < .03:
                self.finish('succeeded', 0, 'target reached; holding target')
            elif elapsed > self.plan.times[-1]+self.grace:
                self.finish('aborted', -5, 'goal tolerance violated; holding measured position')

    def publish(self):
        actual, velocity = self.data.qpos[self.qpos], self.data.qvel[self.dofs]
        self.status_pub.publish(String(data=json.dumps({
            'controller': 'panda_native', 'status': self.status, 'joint_names': self.names,
            'positions': actual.tolist(), 'velocities': velocity.tolist(),
            'target': self.target.tolist(), 'home': self.home.tolist(), 'limits': self.limits.tolist(),
            'bias_compensation_rad': self.control_offset.tolist(),
            'busy': self.reserved, 'contact_blocked': self.contact_blocked(),
            'time': self.data.time})))
        if self.goal:
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = self.names
            feedback.actual = JointTrajectoryPoint(positions=actual.tolist(), velocities=velocity.tolist())
            desired, desired_velocity = self.plan.sample(self.data.time-self.start)
            feedback.desired = JointTrajectoryPoint(positions=desired.tolist(), velocities=desired_velocity.tolist())
            feedback.error = JointTrajectoryPoint(positions=(desired-actual).tolist(),
                                                  velocities=(desired_velocity-velocity).tolist())
            feedback.header.stamp.sec = int(self.data.time)
            feedback.header.stamp.nanosec = int((self.data.time-int(self.data.time))*1e9)
            self.goal.publish_feedback(feedback)
