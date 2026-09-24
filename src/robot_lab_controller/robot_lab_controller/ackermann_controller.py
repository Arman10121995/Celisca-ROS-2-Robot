#!/usr/bin/env python3
"""Car-like drive for ros2_control (Gazebo): /cmd_vel -> steering + wheels.

Turns the stamped twist from the teleop/navigation relay into the steering
angles and wheel rates of robot_lab_utils.drive_kinematics - the model the
PyBullet, MuJoCo and Isaac bridges apply directly - and publishes them to
two forward-command controllers (position for the steering joints, velocity
for the wheels).  Odometry comes from the fixed axle's wheel spin, which is
a differential drive: /robot_lab_controller/odom, as for the
differential-drive robots, so the EKF configuration is unchanged.

Commands stop after ``cmd_vel_timeout`` without a new twist.
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from robot_lab_utils.drive_kinematics import drive_from_config, parse_drive_config


class AckermannController(Node):
    def __init__(self):
        super().__init__("ackermann_controller")
        self.declare_parameter("drive_config", "")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("steering_controller", "car_steering_controller")
        self.declare_parameter("wheel_controller", "car_wheel_controller")
        config = parse_drive_config(self.get_parameter("drive_config").value)
        self._drive = drive_from_config(config)
        if self._drive.kind != "ackermann":
            raise ValueError("ackermann_controller needs an ackermann drive_config")
        # Joint order of the forward-command controllers (see the robot's
        # <steer_axle>_steer_controllers.yaml).
        self._steer_joints = list(self._drive.steer_joints)
        self._wheel_joints = [self._drive.left, self._drive.right,
                              self._drive.steered_left, self._drive.steered_right]
        self._timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self._command = (0.0, 0.0)
        self._command_time = None
        self._last_step = None
        self._rate = float(self.get_parameter("rate").value)

        steering = self.get_parameter("steering_controller").value
        wheels = self.get_parameter("wheel_controller").value
        self._steer_pub = self.create_publisher(
            Float64MultiArray, "/%s/commands" % steering, 10)
        self._wheel_pub = self.create_publisher(
            Float64MultiArray, "/%s/commands" % wheels, 10)
        self._odom_pub = self.create_publisher(Odometry, "/robot_lab_controller/odom", 10)
        self.create_subscription(
            TwistStamped, "/robot_lab_controller/cmd_vel", self._on_twist, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)
        self.create_timer(1.0 / self._rate, self._step)

        self._odom = [0.0, 0.0, 0.0]
        self._wheel_prev = None
        self._joint_time = None
        self.get_logger().info(
            "Ackermann drive: %s steering (%s), minimum turning radius %.2f m, "
            "steering %s, wheels %s"
            % (self._drive.steer_axle, self._drive.geometry,
               self._drive.min_turning_radius, self._steer_joints, self._wheel_joints))

    def _on_twist(self, msg):
        self._command = (msg.twist.linear.x, msg.twist.angular.z)
        self._command_time = self.get_clock().now()

    def _step(self):
        now = self.get_clock().now()
        stale = self._command_time is None or \
            (now - self._command_time).nanoseconds * 1e-9 > self._timeout
        vx, wz = (0.0, 0.0) if stale else self._command
        dt = 1.0 / self._rate
        if self._last_step is not None:
            elapsed = (now - self._last_step).nanoseconds * 1e-9
            if 0.0 < elapsed < 0.5:
                dt = elapsed
        self._last_step = now
        targets = self._drive.targets(vx, wz, dt=dt)
        self._steer_pub.publish(Float64MultiArray(
            data=[targets.position.get(j, 0.0) for j in self._steer_joints]))
        self._wheel_pub.publish(Float64MultiArray(
            data=[targets.velocity.get(j, 0.0) for j in self._wheel_joints]))

    def _on_joints(self, msg):
        try:
            left = msg.position[msg.name.index(self._drive.left)]
            right = msg.position[msg.name.index(self._drive.right)]
        except ValueError:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._wheel_prev is None:
            self._wheel_prev = (left, right)
            self._joint_time = stamp
            return
        dt = stamp - self._joint_time
        if dt <= 0.0:
            return
        d_left, d_right = left - self._wheel_prev[0], right - self._wheel_prev[1]
        self._wheel_prev = (left, right)
        self._joint_time = stamp
        ds, dtheta = self._drive.body_twist(d_left, d_right)
        vx, wz = ds / dt, dtheta / dt
        heading = self._odom[2] + dtheta / 2.0
        self._odom[0] += ds * math.cos(heading)
        self._odom[1] += ds * math.sin(heading)
        self._odom[2] += dtheta

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.get_parameter("base_frame").value
        odom.pose.pose.position.x = self._odom[0]
        odom.pose.pose.position.y = self._odom[1]
        odom.pose.pose.orientation.z = math.sin(self._odom[2] / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._odom[2] / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        # Same diagonal as the differential-drive controllers' configuration.
        odom.pose.covariance = [0.0] * 36
        odom.twist.covariance = [0.0] * 36
        for i, value in enumerate((0.001, 0.001, 1e-3, 1e-3, 1e-3, 0.01)):
            odom.pose.covariance[i * 7] = value
            odom.twist.covariance[i * 7] = value
        self._odom_pub.publish(odom)


def main():
    rclpy.init()
    node = AckermannController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
