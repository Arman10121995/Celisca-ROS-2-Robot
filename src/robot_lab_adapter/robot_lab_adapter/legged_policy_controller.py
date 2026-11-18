#!/usr/bin/env python3
"""ROS node: velocity command -> model-matched legged policy -> joint PD targets.

Runs one provisioned rl_sar policy (robot_lab_adapter/legged_policy.py) at
its trained period (dt x decimation, in simulation time) from the
simulator's /joint_states and /imu/out, and publishes joint targets that
the simulator tracks with the policy's own PD law: the MuJoCo and Isaac
bridges take ``targets_topic`` (robot_lab_mujoco/joint_pd.py, PhysX
drives); Gazebo takes the same targets as a one-point JointTrajectory on
``trajectory_topic`` for an effort joint_trajectory_controller carrying the
policy gains (legged_policy.ros2_control_pd_parameters).

Safety: commands are clamped to the policy's command range and expire
after ``command_timeout_s`` (zero velocity: the policy stands); stale
joint/IMU state or a tilt beyond ``fall_tilt_rad`` latches SAFE_STOP,
which publishes the default standing pose until ``/legged_policy/reset``.
A policy trained to get up (``source.json`` task in ``GET_UP_TASKS``) acts
at any tilt: lying on its side or back is its starting state, so only
stale state and policy failures stop it.

Fall recovery: with ``recovery_policy`` (a get-up policy sharing the main
policy's joints, timing and PD gains, so the simulator's one PD law serves
both), a tilt beyond ``fall_tilt_rad`` hands control to it instead of
SAFE_STOP. Once the body has stayed within ``recovered_tilt_rad`` for
``recovered_hold_s`` of sim time, the main policy resumes from a reset
history. ``/legged_policy/status`` reports the active policy and switches.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

# Training tasks whose policies start from fallen poses (legged_training.py).
GET_UP_TASKS = ('get_up', 'get_up_roll')
# Contract fields two policies must share for one simulator PD law to serve both.
SHARED_PD_FIELDS = ('robot_joint_names', 'joint_mapping', 'dt', 'decimation', 'rl_kp', 'rl_kd',
                    'torque_limits', 'wheel_indices')


def recovery_mismatch(main, recovery):
    """Why ``recovery`` cannot take over from ``main`` (a reason string), or ''."""
    if recovery.source.get('task') not in GET_UP_TASKS:
        return 'recovery policy task %r is not a get-up task' % recovery.source.get('task')
    for name in SHARED_PD_FIELDS:
        if not np.array_equal(np.asarray(getattr(main, name)), np.asarray(getattr(recovery, name))):
            return 'recovery policy %s differs from the main policy' % name
    return ''
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_lab_adapter.legged_policy import (
    LeggedPolicy, LeggedState, PolicyContract, clamp_command, mujoco_pd_config,
)


def policy_directory(name):
    """Installed policy directory for an rl_sar '<robot>_<variant>' name or a path."""
    if os.path.isdir(name):
        return name
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('robot_lab_adapter'), 'policies', 'rl_sar', name)


# The bridge config lives with the pure contract module; kept under the
# old name so existing callers of this node module keep working.
pd_config = mujoco_pd_config


def trajectory_message(names, positions, velocities, velocity_joint):
    """One-point JointTrajectory for an effort joint_trajectory_controller.

    Position joints carry their target position and zero velocity (the
    policy's ``kd (0 - dq)`` term); velocity joints (wheels, controller
    gain p = 0) carry their target velocity.  An unstamped header and
    time_from_start 0 make the controller apply it immediately.
    """
    point = JointTrajectoryPoint()
    point.positions = [0.0 if wheel else float(p) for p, wheel in zip(positions, velocity_joint)]
    point.velocities = [float(v) if wheel else 0.0 for v, wheel in zip(velocities, velocity_joint)]
    message = JointTrajectory(joint_names=list(names))
    message.points = [point]
    return message


class LeggedPolicyController(Node):
    def __init__(self):
        super().__init__('legged_policy_controller')
        self.declare_parameter('policy', '')
        self.declare_parameter('targets_topic', '/legged_policy/joint_targets')
        self.declare_parameter('trajectory_topic', '')
        self.declare_parameter('command_topics', ['/robot_lab_controller/cmd_vel_unstamped', '/cmd_vel'])
        self.declare_parameter('command_limits', [0.8, 0.4, 1.0])
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('state_timeout_s', 0.5)
        self.declare_parameter('fall_tilt_rad', 1.0)
        self.declare_parameter('recovery_policy', '')
        self.declare_parameter('recovered_tilt_rad', 0.35)
        self.declare_parameter('recovered_hold_s', 1.0)
        directory = policy_directory(self.get_parameter('policy').value)
        self.contract = PolicyContract.load(directory)
        self.policy = LeggedPolicy(self.contract, model_path=os.path.join(directory, 'policy.onnx'))
        self.get_up = self.contract.source.get('task') in GET_UP_TASKS
        self.recovery, self.recovering, self.upright_since, self.switches = None, False, None, []
        recovery_name = str(self.get_parameter('recovery_policy').value or '')
        if recovery_name and not self.get_up:
            recovery_directory = policy_directory(recovery_name)
            recovery_contract = PolicyContract.load(recovery_directory)
            reason = recovery_mismatch(self.contract, recovery_contract)
            if reason:
                raise ValueError(reason)
            self.recovery = LeggedPolicy(recovery_contract,
                                         model_path=os.path.join(recovery_directory, 'policy.onnx'))
            self.recovery_name = recovery_contract.source.get('variant', recovery_name)
        self.limits = [float(v) for v in self.get_parameter('command_limits').value]
        self.command, self.command_time = [0.0, 0.0, 0.0], None
        self.joints, self.joint_time = None, None
        self.imu, self.imu_time = None, None
        # Terrain heights around the base, for policies that observe them
        # (the simulator's height-scan publisher; see legged_policy.HEIGHT_SCAN).
        self.scan, self.scan_time = None, None
        self.state, self.reason = 'waiting_for_state', ''
        self.steps = 0
        self.targets_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('targets_topic').value, 1)
        self.status_pub = self.create_publisher(String, '/legged_policy/status', 1)
        trajectory_topic = str(self.get_parameter('trajectory_topic').value or '')
        self.trajectory_pub = (self.create_publisher(JointTrajectory, trajectory_topic, 1)
                               if trajectory_topic else None)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 5)
        self.create_subscription(Imu, '/imu/out', self.on_imu, 5)
        if self.contract.height_scan is not None:
            from std_msgs.msg import Float32MultiArray
            from robot_lab_adapter.legged_policy import HEIGHT_SCAN_TOPIC
            self.create_subscription(Float32MultiArray, HEIGHT_SCAN_TOPIC, self.on_scan, 5)
        for topic in self.get_parameter('command_topics').value:
            self.create_subscription(Twist, topic, self.on_command, 5)
        self.create_service(Trigger, '/legged_policy/reset', self.on_reset)
        self.create_timer(self.contract.policy_period, self.tick)
        self.create_timer(0.5, self.report)
        self.get_logger().info('Legged policy %s/%s (%s, revision %s): %d joints, %.0f Hz%s'
                               % (self.contract.source.get('robot'), self.contract.source.get('variant'),
                                  self.contract.source.get('upstream'), self.contract.source.get('revision', '')[:8],
                                  self.contract.num_dofs, 1.0/self.contract.policy_period,
                                  '; get-up policy: no tilt stop' if self.get_up else ''))

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_joints(self, message):
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity)) if message.velocity else {}
        if self.joints is None:
            self.joints = ({}, {})
        self.joints[0].update(positions)
        self.joints[1].update({name: velocities.get(name, 0.0) for name in positions})
        self.joint_time = self.now()

    def on_imu(self, message):
        q, w = message.orientation, message.angular_velocity
        self.imu = ((q.w, q.x, q.y, q.z), (w.x, w.y, w.z))
        self.imu_time = self.now()

    def on_scan(self, message):
        self.scan = list(message.data)
        self.scan_time = self.now()

    def on_command(self, message):
        self.command = clamp_command((message.linear.x, message.linear.y, message.angular.z), self.limits)
        self.command_time = self.now()

    def on_reset(self, _request, response):
        self.policy.reset()
        self.state, self.reason = 'waiting_for_state', ''
        response.success, response.message = True, 'Policy state reset; standing pose until fresh state.'
        return response

    def publish(self, targets):
        velocity_joint = [m == 'velocity' for m in self._modes()]
        values = np.where(velocity_joint, targets.velocity, targets.position)
        self.targets_pub.publish(Float64MultiArray(data=[float(v) for v in values]))
        if self.trajectory_pub is not None:
            self.trajectory_pub.publish(trajectory_message(
                self.policy.names, targets.position, targets.velocity, velocity_joint))

    def _modes(self):
        wheels = set(self.contract.wheel_indices)
        return ['velocity' if i in wheels else 'position' for i in range(self.contract.num_dofs)]

    def standing(self):
        self.policy.actions[:] = 0.0
        self.publish(self.policy.targets())

    def tick(self):
        now = self.now()
        timeout = float(self.get_parameter('state_timeout_s').value)
        fresh = (self.joints is not None and self.imu is not None
                 and now-self.joint_time <= timeout and now-self.imu_time <= timeout
                 and (self.contract.height_scan is None
                      or (self.scan is not None and now-self.scan_time <= timeout)))
        if self.state == 'safe_stop':
            self.standing()
            return
        if not fresh:
            if self.state in ('running', 'recovering'):
                self.state, self.reason = 'safe_stop', 'joint/IMU/height-scan state went stale'
            self.standing()
            return
        names = self.policy.names
        if any(name not in self.joints[0] for name in names):
            self.state, self.reason = 'safe_stop', 'joint_states lacks policy joints'
            self.standing()
            return
        quaternion, gyro = self.imu
        gravity_z = 1.0-2.0*(quaternion[1]**2+quaternion[2]**2)   # body z in world z
        tilt = math.acos(max(-1.0, min(1.0, gravity_z)))
        fallen = tilt > float(self.get_parameter('fall_tilt_rad').value)
        if self.recovery is not None:
            if fallen and not self.recovering:
                self.recovering, self.upright_since = True, None
                self.recovery.reset()
                self.switches.append(dict(t=round(now, 3), to='recovery', tilt_rad=round(tilt, 3)))
                self.get_logger().warn('Fall (tilt %.2f rad): %s takes over' % (tilt, self.recovery_name))
            elif self.recovering:
                if tilt < float(self.get_parameter('recovered_tilt_rad').value):
                    self.upright_since = now if self.upright_since is None else self.upright_since
                    if now - self.upright_since >= float(self.get_parameter('recovered_hold_s').value):
                        self.recovering = False
                        self.policy.reset()
                        self.switches.append(dict(t=round(now, 3), to='main', tilt_rad=round(tilt, 3)))
                        self.get_logger().info('Upright again: main policy resumes')
                else:
                    self.upright_since = None
        elif not self.get_up and fallen:
            self.state, self.reason = 'safe_stop', 'tilt %.2f rad exceeds the fall limit' % tilt
            self.standing()
            return
        command = self.command
        if self.recovering or self.command_time is None or \
                now-self.command_time > float(self.get_parameter('command_timeout_s').value):
            command = [0.0, 0.0, 0.0]
        state = LeggedState(self.joints[0], self.joints[1], quaternion, gyro, height_scan=self.scan)
        try:
            targets = (self.recovery if self.recovering else self.policy).step(state, command)
        except (ValueError, RuntimeError) as exc:
            self.state, self.reason = 'safe_stop', 'policy failure: %s' % exc
            self.standing()
            return
        self.state = 'recovering' if self.recovering else 'running'
        self.steps += 1
        self.publish(targets)

    def report(self):
        self.status_pub.publish(String(data=json.dumps(dict(
            state=self.state, reason=self.reason, steps=self.steps, command=self.command,
            policy=self.contract.source.get('robot', '')+'/'+self.contract.source.get('variant', ''),
            recovery_policy=getattr(self, 'recovery_name', None), recovering=self.recovering,
            switches=self.switches[-20:],
            qualification='experimental model-matched policy; validation per robot/map pending'))))


def main():
    rclpy.init()
    node = LeggedPolicyController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
