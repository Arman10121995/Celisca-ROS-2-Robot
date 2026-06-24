#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from threading import Lock
import sys

try:
    from pynput import keyboard
except ImportError as exc:
    raise SystemExit(
        'keyboard_teleop.py requires pynput for key release events. '
        'Install it with: sudo apt install python3-pynput'
    ) from exc

msg = """
Control Your Robot!
---------------------------
Moving around:
        w
   a    s    d

w/s : hold static forward/backward speed
a/d : hold static left/right turn speed
SHIFT + w/a/s/d : double speed
speed_modifier parameter scales all speeds

s/S : clear other keys and drive backward while pressed
release key or space key : force stop

CTRL-C to quit
"""

move_bindings = {
    'w': (0.1, 0.0),
    's': (-0.1, 0.0),
    'a': (0.0, 0.5),
    'd': (0.0, -0.5),
}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/key_vel', 10)
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.pressed_keys = {}
        self.declare_parameter('speed_modifier', 1.0)
        self.speed_modifier = self.get_parameter(
            'speed_modifier'
        ).get_parameter_value().double_value
        self.speed_modifier = self.get_speed_modifier_from_argv(
            self.speed_modifier
        )
        self.lock = Lock()
        self.timer = self.create_timer(0.05, self.publish_callback)
        self.listener = keyboard.Listener(
            on_press=self.on_key_press,
            on_release=self.on_key_release,
        )
        self.listener.start()
        self.get_logger().info(
            f'Using speed modifier: {self.speed_modifier}'
        )
        self.get_logger().info(msg)

    def get_speed_modifier_from_argv(self, default_value):
        for arg in sys.argv:
            if arg.startswith('speed-modifier:='):
                try:
                    return float(arg.split(':=', 1)[1])
                except ValueError:
                    self.get_logger().warn(
                        f'Ignoring invalid speed-modifier argument: {arg}'
                    )
        return default_value

    def on_key_press(self, key):
        if key == keyboard.Key.space:
            with self.lock:
                self.pressed_keys.clear()
                self.linear_x = 0.0
                self.angular_z = 0.0
            self.publish_twist()
            return

        try:
            key_char = key.char
        except AttributeError:
            return

        command_key = key_char.lower()
        if command_key in move_bindings:
            with self.lock:
                speed_multiplier = 2.0 if key_char.isupper() else 1.0
                if command_key == 's':
                    self.pressed_keys.clear()
                self.pressed_keys[command_key] = speed_multiplier
                self.update_velocity()
            self.publish_twist()

    def on_key_release(self, key):
        try:
            key_char = key.char
        except AttributeError:
            return

        command_key = key_char.lower()
        if command_key in move_bindings:
            with self.lock:
                self.pressed_keys.pop(command_key, None)
                self.update_velocity()
            self.publish_twist()

    def update_velocity(self):
        if not self.pressed_keys:
            self.linear_x = 0.0
            self.angular_z = 0.0
            return

        self.linear_x = sum(
            move_bindings[key][0] * multiplier * self.speed_modifier
            for key, multiplier in self.pressed_keys.items()
        )
        self.angular_z = sum(
            move_bindings[key][1] * multiplier * self.speed_modifier
            for key, multiplier in self.pressed_keys.items()
        )

    def publish_twist(self):
        twist = Twist()
        with self.lock:
            twist.linear.x = self.linear_x
            twist.angular.z = self.angular_z
        self.publisher_.publish(twist)

    def publish_callback(self):
        self.publish_twist()

    def stop(self):
        if self.listener.running:
            self.listener.stop()
        with self.lock:
            self.pressed_keys.clear()
            self.linear_x = 0.0
            self.angular_z = 0.0
        self.publish_twist()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
