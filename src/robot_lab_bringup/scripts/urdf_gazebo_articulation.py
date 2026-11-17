#!/usr/bin/env python3
"""Shared URDF articulation contract on Gazebo for URDF robots without ros2_control (R5.7).

Gazebo display runs give such robots a ``JointPositionController`` per
movable joint (``robot_lab_utils.gazebo_display``: velocity servo within the
joint's own effort limit). This node runs the same contract as the other
backends (``/molar/articulation/{state,command,heartbeat,stop}``) on the
bridged ``/joint_states`` and sends each joint's contract reference to its
controller topic ``hold_command_topic(robot, joint)`` (bridged by the launch
under the same name). Feedback loss holds the last
measured pose. MOLAR keeps its own ros2_control relay (molar_gazebo_control.py).
"""
import json
import math
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from robot_lab_utils.gazebo_display import hold_command_topic
from robot_lab_utils.urdf_articulation import ArticulationROS


class UrdfGazeboArticulation(Node):
    def __init__(self):
        super().__init__('urdf_gazebo_articulation')
        self.declare_parameter('model_path', '')
        self.declare_parameter('articulation_config', '{}')
        self.declare_parameter('robot_name', '')
        model = Path(self.get_parameter('model_path').value).resolve(strict=True)
        text = model.read_text()
        if model.suffix == '.xacro':
            import xacro
            text = xacro.process_file(str(model)).toxml()
        self.articulation = ArticulationROS(self, json.loads(self.get_parameter('articulation_config').value),
                                            text, str(model))
        joints = list(self.articulation.contract.joints)
        robot = self.get_parameter('robot_name').value
        self.publishers_ = {name: self.create_publisher(Float64, hold_command_topic(robot, name), 1) for name in joints}
        self.positions, self.velocities = {}, {}
        self.feedback_at, self.feedback_stale, self.last_sim = -math.inf, True, None
        self.create_subscription(JointState, '/joint_states', self.feedback, 10)
        from rclpy.clock import Clock, ClockType
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def feedback(self, message):
        values = dict(zip(message.name, message.position))
        rates = dict(zip(message.name, message.velocity)) if len(message.velocity) == len(message.name) else {}
        required = set(self.articulation.contract.joints)
        if not required <= set(values) or not all(math.isfinite(values[n]) for n in required):
            self.feedback_at = -math.inf
            return
        self.positions = {n: values[n] for n in required}
        self.velocities = {n: rates.get(n, 0.) if math.isfinite(rates.get(n, 0.)) else 0. for n in required}
        self.feedback_at = time.monotonic()

    def send(self, targets):
        for name, value in targets.items():
            if name in self.publishers_:
                self.publishers_[name].publish(Float64(data=float(value)))

    def tick(self):
        sim = self.get_clock().now().nanoseconds / 1e9
        if self.last_sim is not None and sim < self.last_sim:
            self.articulation.reset()
        dt = 0. if self.last_sim is None else min(.05, max(0., sim - self.last_sim))
        self.last_sim = sim
        if time.monotonic() - self.feedback_at < .3:
            self.feedback_stale = False
            target = self.articulation.step(self.positions, self.velocities, dt, base_moving=False)
            self.send(target['position'])
        else:
            if not self.feedback_stale:
                self.articulation.contract.stop('feedback lost; last finite measured hold')
                self.feedback_stale = True
            self.send(self.articulation.contract.references)


def main():
    rclpy.init()
    node = UrdfGazeboArticulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
