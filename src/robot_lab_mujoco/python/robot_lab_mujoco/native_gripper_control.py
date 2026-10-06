"""GripperCommand on the pinned Panda's original tendon and equality.

The command position is the full opening in metres. Only actuator targets and
its force bound change; the physics engine owns joint and object motion.
"""
import json
import time

import numpy as np
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.task import Future
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger


class NativePandaGripper:
    def __init__(self, node):
        self.node, self.model, self.data, self.mj = node, node.model, node.data, node.mujoco
        self.names = ['finger_joint1', 'finger_joint2']
        self.joints = np.array([self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_JOINT, name)
                                for name in self.names])
        self.actuator = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_ACTUATOR, 'actuator8')
        self.tendon = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_TENDON, 'split')
        if np.any(self.joints < 0) or self.actuator < 0 or self.tendon < 0:
            raise ValueError('Panda gripper requires the original finger joints, actuator8 and split tendon')
        address, count = int(self.model.tendon_adr[self.tendon]), int(self.model.tendon_num[self.tendon])
        if (count != 2 or not np.array_equal(self.model.wrap_objid[address:address+count], self.joints)
                or not np.allclose(self.model.wrap_prm[address:address+count], .5)
                or int(self.model.actuator_trnid[self.actuator, 0]) != self.tendon
                or int(self.model.actuator_trntype[self.actuator]) != int(self.mj.mjtTrn.mjTRN_TENDON)
                or not np.allclose(self.model.actuator_ctrlrange[self.actuator], [0, 255])
                or not np.isclose(self.model.actuator_gainprm[self.actuator, 0], .01568627451)
                or not np.allclose(self.model.actuator_biasprm[self.actuator, :3], [0, -100, -10])
                or not self.model.actuator_forcelimited[self.actuator]
                or not np.allclose(self.model.jnt_range[self.joints], [[0, .04], [0, .04]])):
            raise ValueError('Panda gripper transmission differs from the pinned model')
        self.qpos = self.model.jnt_qposadr[self.joints]
        self.dofs = self.model.jnt_dofadr[self.joints]
        self.finger_bodies = self.model.jnt_bodyid[self.joints]
        self.target = self.opening()
        self.commanded = self.target
        self.force_limit = 10.
        self.goal = None
        self.reserved = False
        self.last_heartbeat = -float('inf')
        self.heartbeat_lost = False
        self.status = 'holding measured gap'
        self.stall_since = None
        self.status_pub = node.create_publisher(String, '/gripper/status', 10)
        node.create_subscription(Empty, '/gripper/heartbeat', self.heartbeat, 10)
        node.create_service(Trigger, '/gripper/stop', self.stop)
        self.server = ActionServer(node, GripperCommand, '/gripper/gripper_action', self.execute,
            goal_callback=self.accept, handle_accepted_callback=self.accepted,
            cancel_callback=lambda _: CancelResponse.ACCEPT)

    def opening(self):
        return float(np.sum(self.data.qpos[self.qpos]))

    def effort(self):
        return float(np.max(np.abs(self.data.qfrc_actuator[self.dofs])))

    def heartbeat(self, _message):
        self.last_heartbeat = time.monotonic()
        self.heartbeat_lost = False

    def contacts(self):
        result = {name: [] for name in self.names}
        for index, contact in enumerate(self.data.contact[:self.data.ncon]):
            bodies = [int(self.model.geom_bodyid[geom]) for geom in contact.geom]
            if bodies[0] == bodies[1]:
                continue
            force = np.zeros(6)
            self.mj.mj_contactForce(self.model, self.data, index, force)
            for name, body in zip(self.names, self.finger_bodies):
                if body in bodies:
                    other = bodies[1] if bodies[0] == body else bodies[0]
                    if other not in self.finger_bodies:
                        result[name].append(dict(body=self.mj.mj_id2name(self.model,
                            self.mj.mjtObj.mjOBJ_BODY, other) or 'world', normal_force_n=float(abs(force[0]))))
        return result

    def accept(self, request):
        position, effort = request.command.position, request.command.max_effort
        if (self.reserved or self.node.arm.reserved or time.monotonic()-self.last_heartbeat > .8
                or not np.isfinite(position) or not 0 <= position <= .08
                or not np.isfinite(effort) or not 0 <= effort <= 20):
            self.node.get_logger().warn('Gripper goal rejected: busy, heartbeat, gap or force limit')
            return GoalResponse.REJECT
        self.pending = (float(position), float(effort) or .5)
        self.reserved = True
        return GoalResponse.ACCEPT

    def accepted(self, goal):
        self.goal = goal
        self.target, self.force_limit = self.pending
        self.commanded = float(np.clip(self.opening(), 0, .08))
        self.start = self.data.time
        self.stall_since = None
        self.status = 'executing'
        goal._gripper_future = Future()
        goal.execute()

    async def execute(self, goal):
        return await goal._gripper_future

    def finish(self, outcome, message, reached=False, stalled=False):
        if outcome != 'succeeded':
            self.target = float(np.clip(self.opening(), 0, .08))
            self.commanded = self.target
        self.status = message
        if self.goal:
            {'succeeded': self.goal.succeed, 'canceled': self.goal.canceled,
             'aborted': self.goal.abort}[outcome]()
            self.goal._gripper_future.set_result(GripperCommand.Result(position=self.opening(),
                effort=self.effort(), reached_goal=reached, stalled=stalled))
        self.goal = None
        self.reserved = False
        self.stall_since = None

    def stop(self, _request, response):
        self.finish('aborted', 'stopped; holding measured gap')
        response.success, response.message = True, self.status
        return response

    def before_step(self):
        if time.monotonic()-self.last_heartbeat > .8 and not self.heartbeat_lost:
            self.finish('aborted', 'heartbeat lost; holding measured gap')
            self.heartbeat_lost = True
        elif self.goal and self.goal.is_cancel_requested:
            self.finish('canceled', 'canceled; holding measured gap')
        increment = .03*self.model.opt.timestep
        self.commanded += float(np.clip(self.target-self.commanded, -increment, increment))
        # The split tendon applies half the actuator force to each finger.
        # Bounds below retain the source gain, bias, transmission and coupling.
        self.model.actuator_forcerange[self.actuator] = [-2*self.force_limit, 2*self.force_limit]
        self.data.ctrl[self.actuator] = float(np.clip(self.commanded/.08*255., 0, 255))

    def after_step(self):
        if not self.goal:
            return
        opening = self.opening()
        speed = abs(float(np.sum(self.data.qvel[self.dofs])))
        reached = abs(opening-self.target) < .002 and speed < .001
        contacts = self.contacts()
        touching = all(sum(c['normal_force_n'] for c in contacts[name]) > .05 for name in self.names)
        stalled = (not reached and speed < .0008 and touching
                   and self.effort() >= .95*self.force_limit)
        if stalled:
            self.stall_since = self.data.time if self.stall_since is None else self.stall_since
        else:
            self.stall_since = None
        if reached:
            self.finish('succeeded', 'opening reached', reached=True)
        elif self.stall_since is not None and self.data.time-self.stall_since >= .3:
            self.finish('succeeded', 'contact stall; holding bounded target force', stalled=True)
        elif self.data.time-self.start > 8.:
            self.finish('aborted', 'opening did not converge before timeout')

    def publish(self):
        self.status_pub.publish(String(data=json.dumps(dict(controller='panda_gripper_native',
            status=self.status, joint_names=self.names, positions=self.data.qpos[self.qpos].tolist(),
            velocities=self.data.qvel[self.dofs].tolist(), opening_m=self.opening(),
            target_m=self.target, busy=self.reserved, force_limit_per_finger_n=self.force_limit,
            actuator_effort_per_finger_n=self.effort(), contacts=self.contacts(), time=self.data.time))))
        if self.goal:
            self.goal.publish_feedback(GripperCommand.Feedback(position=self.opening(), effort=self.effort(),
                                                               reached_goal=False, stalled=False))
