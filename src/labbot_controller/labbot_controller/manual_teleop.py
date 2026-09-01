"""Manual teleoperation for Labbot."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, select


class ManualTeleop(Node):
    def __init__(self):
        super().__init__('labbot_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear = 0.3
        self.angular = 1.0
        self.get_logger().info('Labbot teleop: w/a/s/d/x to move, q to quit')

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            cmd = Twist()
            if key == 'w': cmd.linear.x = self.linear
            elif key == 'x': cmd.linear.x = -self.linear
            elif key == 'a': cmd.angular.z = self.angular
            elif key == 'd': cmd.angular.z = -self.angular
            elif key == 'q': break
            self.pub.publish(cmd)

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ''


def main(args=None):
    rclpy.init(args=args)
    node = ManualTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
