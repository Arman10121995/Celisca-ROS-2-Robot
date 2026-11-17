#!/usr/bin/env python3
"""Gazebo truth for loose world objects on /simulation/free_bodies (as MuJoCo, PyBullet, Isaac).

Reads the world's dynamic poses (``/world/<w>/dynamic_pose/info``, bridged
under the same name as tf2_msgs/TFMessage; model poses are in the world frame) and publishes
JSON {name: [x, y, z, qw, qx, qy, qz]} at 10 Hz for the loose single-link
models of the world file (``sdf_world.extract_dynamic_bodies``). Reports
only; nothing is written back.
"""
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def dynamic_pose_topic(world):
    return '/world/%s/dynamic_pose/info' % world


class GazeboFreeBodies(Node):
    def __init__(self):
        super().__init__('gazebo_free_bodies')
        self.declare_parameter('world_path', '')
        self.declare_parameter('world_name', '')
        from robot_lab_utils.sdf_world import extract_dynamic_bodies
        self.names = {body['name'] for body in extract_dynamic_bodies(self.get_parameter('world_path').value)}
        self.poses = {}
        self.pub = self.create_publisher(String, '/simulation/free_bodies', 10)
        self.create_subscription(TFMessage, dynamic_pose_topic(self.get_parameter('world_name').value),
                                 self.receive, 10)
        self.create_timer(.1, self.publish)

    def receive(self, message):
        for transform in message.transforms:
            name = transform.child_frame_id          # links are 'model::link'; models are bare names
            if name in self.names:
                t, q = transform.transform.translation, transform.transform.rotation
                self.poses[name] = [t.x, t.y, t.z, q.w, q.x, q.y, q.z]

    def publish(self):
        if self.poses:
            self.pub.publish(String(data=json.dumps(self.poses)))


def main():
    rclpy.init()
    node = GazeboFreeBodies()
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
