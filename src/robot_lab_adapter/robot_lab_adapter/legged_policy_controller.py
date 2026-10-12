#!/usr/bin/env python3
"""ROS node: velocity command -> model-matched legged policy -> joint PD targets.

Runs one provisioned rl_sar policy (robot_lab_adapter/legged_policy.py) at
its trained period (dt x decimation, in simulation time) from the
simulator's /joint_states and /imu/out, and publishes joint targets that
the MuJoCo bridge tracks with the policy's own PD law at every physics
step (robot_lab_mujoco/joint_pd.py).

Safety: commands are clamped to the policy's command range and expire
after ``command_timeout_s`` (zero velocity: the policy stands); stale
joint/IMU state or a tilt beyond ``fall_tilt_rad`` latches SAFE_STOP,
which publishes the default standing pose until ``/legged_policy/reset``.
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


class LeggedPolicyController(Node):
    def __init__(self):
        super().__init__('legged_policy_controller')
        self.declare_parameter('policy', '')
        self.declare_parameter('targets_topic', '/legged_policy/joint_targets')
        self.declare_parameter('command_topics', ['/robot_lab_controller/cmd_vel_unstamped', '/cmd_vel'])
        self.declare_parameter('command_limits', [0.8, 0.4, 1.0])
        self.declare_parameter('command_timeout_s', 0.5)
        self.declare_parameter('state_timeout_s', 0.5)
        self.declare_parameter('fall_tilt_rad', 1.0)
        directory = policy_directory(self.get_parameter('policy').value)
        self.contract = PolicyContract.load(directory)
        self.policy = LeggedPolicy(self.contract, model_path=os.path.join(directory, 'policy.onnx'))
        self.limits = [float(v) for v in self.get_parameter('command_limits').value]
        self.command, self.command_time = [0.0, 0.0, 0.0], None
        self.joints, self.joint_time = None, None
        self.imu, self.imu_time = None, None
        self.state, self.reason = 'waiting_for_state', ''
        self.steps = 0
        self.targets_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('targets_topic').value, 1)
        self.status_pub = self.create_publisher(String, '/legged_policy/status', 1)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 5)
        self.create_subscription(Imu, '/imu/out', self.on_imu, 5)
        for topic in self.get_parameter('command_topics').value:
            self.create_subscription(Twist, topic, self.on_command, 5)
        self.create_service(Trigger, '/legged_policy/reset', self.on_reset)
        self.create_timer(self.contract.policy_period, self.tick)
        self.create_timer(0.5, self.report)
        self.get_logger().info('Legged policy %s/%s (%s, revision %s): %d joints, %.0f Hz'
                               % (self.contract.source.get('robot'), self.contract.source.get('variant'),
                                  self.contract.source.get('upstream'), self.contract.source.get('revision', '')[:8],
                                  self.contract.num_dofs, 1.0/self.contract.policy_period))

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

    def on_command(self, message):
        self.command = clamp_command((message.linear.x, message.linear.y, message.angular.z), self.limits)
        self.command_time = self.now()

    def on_reset(self, _request, response):
        self.policy.reset()
        self.state, self.reason = 'waiting_for_state', ''
        response.success, response.message = True, 'Policy state reset; standing pose until fresh state.'
        return response

    def publish(self, targets):
        values = np.where([m == 'velocity' for m in self._modes()], targets.velocity, targets.position)
        self.targets_pub.publish(Float64MultiArray(data=[float(v) for v in values]))

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
                 and now-self.joint_time <= timeout and now-self.imu_time <= timeout)
        if self.state == 'safe_stop':
            self.standing()
            return
        if not fresh:
            if self.state == 'running':
                self.state, self.reason = 'safe_stop', 'joint/IMU state went stale'
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
        if tilt > float(self.get_parameter('fall_tilt_rad').value):
            self.state, self.reason = 'safe_stop', 'tilt %.2f rad exceeds the fall limit' % tilt
            self.standing()
            return
        command = self.command
        if self.command_time is None or now-self.command_time > float(self.get_parameter('command_timeout_s').value):
            command = [0.0, 0.0, 0.0]
        state = LeggedState(self.joints[0], self.joints[1], quaternion, gyro)
        try:
            targets = self.policy.step(state, command)
        except (ValueError, RuntimeError) as exc:
            self.state, self.reason = 'safe_stop', 'policy failure: %s' % exc
            self.standing()
            return
        self.state = 'running'
        self.steps += 1
        self.publish(targets)

    def report(self):
        self.status_pub.publish(String(data=json.dumps(dict(
            state=self.state, reason=self.reason, steps=self.steps, command=self.command,
            policy=self.contract.source.get('robot', '')+'/'+self.contract.source.get('variant', ''),
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
