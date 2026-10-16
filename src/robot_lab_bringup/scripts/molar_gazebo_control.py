#!/usr/bin/env python3
"""Simulation-only MOLAR ROS control adapter; no physical robot API imports."""
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import xacro
from robot_lab_utils.urdf_articulation import ArticulationROS


class MolarGazeboControl(Node):
    def __init__(self):
        super().__init__('molar_gazebo_control')
        self.declare_parameter('model_path', '')
        self.declare_parameter('articulation_config', '{}')
        model = Path(self.get_parameter('model_path').value).resolve(strict=True)
        if not model.name.startswith(('molar_hri.', 'probot_l.')):
            raise ValueError('MOLAR simulation model required')
        text = xacro.process_file(str(model)).toxml()
        config = json.loads(self.get_parameter('articulation_config').value)
        self.articulation = ArticulationROS(self, config, text, str(model))
        self.positions, self.velocities = {}, {}
        self.feedback_at = -math.inf
        self.command_at = -math.inf
        self.command = Twist()
        self.last_sim = None
        self.pos_names = ['conveyor_lift_joint', 'telescope_stage1_joint', 'telescope_stage2_joint']
        self.position_pub = self.create_publisher(Float64MultiArray, '/molar_position_controller/commands', 1)
        self.roller_pub = self.create_publisher(Float64MultiArray, '/molar_roller_controller/commands', 1)
        self.drive_pub = self.create_publisher(Twist, '/molar_base_controller/cmd_vel_unstamped', 1)
        self.create_subscription(JointState, '/joint_states', self.feedback, 10)
        self.create_subscription(Twist, '/robot_lab_controller/cmd_vel_unstamped', self.drive, 1)
        from std_srvs.srv import Trigger
        self.create_service(Trigger, '/robot_lab/control_reset', self.reset_control)
        # Wall-clock watchdog must still fire if the simulator clock pauses.
        from rclpy.clock import Clock, ClockType
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def feedback(self, msg):
        values = dict(zip(msg.name, msg.position))
        if not all(name in values and math.isfinite(values[name]) for name in self.pos_names):
            return
        self.positions = values
        self.velocities = dict(zip(msg.name, msg.velocity))
        if not all(math.isfinite(v) for v in self.velocities.values()):
            return
        self.feedback_at = time.monotonic()

    def drive(self, msg):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            self.command, self.command_at = Twist(), -math.inf
            return
        # Do not retain a command suppressed by an extended/moving telescope.
        if self.articulation.base_inhibited:
            self.command, self.command_at = Twist(), -math.inf
            return
        self.command = Twist()
        self.command.linear.x = max(-.25, min(.25, msg.linear.x))
        self.command.angular.z = max(-.5, min(.5, msg.angular.z))
        self.command_at = time.monotonic()

    def tick(self):
        now = time.monotonic()
        sim = self.get_clock().now().nanoseconds / 1e9
        if self.last_sim is not None and sim < self.last_sim:
            self.articulation.reset()
            self.command, self.command_at = Twist(), -math.inf
        dt = 0. if self.last_sim is None else min(.05, max(0., sim-self.last_sim))
        self.last_sim = sim
        fresh = now-self.feedback_at < .3
        if fresh:
            moving = any(abs(self.velocities.get(name, 0.)) > .05 for name in ('drive_left_joint', 'drive_right_joint'))
            target = self.articulation.step(self.positions, self.velocities, dt, base_moving=moving)
            if all(name in target['position'] for name in self.pos_names):
                self.position_pub.publish(Float64MultiArray(data=[target['position'][name] for name in self.pos_names]))
            self.roller_pub.publish(Float64MultiArray(data=[target['velocity'].get('transfer_roller_joint', 0.)]))
        else:
            self.roller_pub.publish(Float64MultiArray(data=[0.]))
        allowed = fresh and not self.articulation.base_inhibited and now-self.command_at < .3
        self.drive_pub.publish(self.command if allowed else Twist())
        if not allowed:
            self.command, self.command_at = Twist(), -math.inf

    def stop_outputs(self):
        self.drive_pub.publish(Twist())
        self.roller_pub.publish(Float64MultiArray(data=[0.]))
        if all(name in self.positions for name in self.pos_names):
            self.position_pub.publish(Float64MultiArray(data=[self.positions[name] for name in self.pos_names]))

    def reset_control(self, _request, response):
        self.stop_outputs()
        self.command, self.command_at = Twist(), -math.inf
        self.articulation.reset()
        self.positions, self.velocities = {}, {}
        self.feedback_at, self.last_sim = -math.inf, None
        response.success = True
        response.message = 'MOLAR plant identity revoked; waiting for fresh measured hold'
        return response


def main():
    rclpy.init()
    node = None
    try:
        node = MolarGazeboControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            if rclpy.ok():
                node.stop_outputs()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
