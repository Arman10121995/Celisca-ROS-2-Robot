import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """Log messages published on the tutorial ``chatter`` topic."""

    def __init__(self):
        super().__init__("simple_subscriber")
        self.subscription = self.create_subscription(
            String,
            "chatter",
            self.listener_callback,
            10,
        )

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main():
    rclpy.init()
    node = SimpleSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
