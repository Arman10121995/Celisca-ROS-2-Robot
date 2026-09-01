"""Vacuum cleaning controller node."""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class VacuumCleaner(Node):
    """Coverage-based vacuum cleaning controller."""

    def __init__(self):
        super().__init__('vacuum_cleaner')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.state = 'forward'
        self.linear_speed = 0.2
        self.angular_speed = 1.0
        self.min_clearance = 0.4
        self.timer = self.create_timer(0.1, self.control_loop)

    def scan_cb(self, msg):
        valid = [r for r in msg.ranges if 0.01 < r < 30.0]
        self.min_front = min(valid) if valid else 10.0

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def control_loop(self):
        cmd = Twist()
        if self.state == 'forward':
            if self.min_front < self.min_clearance:
                self.state = 'turn'
                self.turn_dir = 1
            else:
                cmd.linear.x = self.linear_speed
        elif self.state == 'turn':
            cmd.angular.z = self.angular_speed * self.turn_dir
            if self.min_front > self.min_clearance * 2:
                self.state = 'forward'
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = VacuumCleaner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
