#!/usr/bin/env python3
"""Gazebo joint-command and wheel-odometry bridge for 4WS and mecanum bases."""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from robot_lab_utils.drive_kinematics import drive_from_config, parse_drive_config


class HolonomicController(Node):
    def __init__(self):
        super().__init__("holonomic_controller")
        self.declare_parameter("drive_config", "")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("cmd_vel_timeout", 0.5)
        self.declare_parameter("base_frame", "base_footprint")
        config = parse_drive_config(self.get_parameter("drive_config").value)
        self._drive = drive_from_config(config)
        if self._drive.kind not in ("four_wheel_steer", "mecanum"):
            raise ValueError("holonomic_controller needs a 4WS or mecanum drive_config")
        self._wheel_joints = list(self._drive.wheel_joints)
        self._steer_joints = list(self._drive.steer_joints)
        name = ("four_wheel_wheel_controller" if self._steer_joints
                else "mecanum_wheel_controller")
        self._wheel_pub = self.create_publisher(
            Float64MultiArray, "/%s/commands" % name, 10)
        self._steer_pub = (self.create_publisher(
            Float64MultiArray, "/four_wheel_steer_controller/commands", 10)
            if self._steer_joints else None)
        self._odom_pub = self.create_publisher(
            Odometry, "/robot_lab_controller/odom", 10)
        self.create_subscription(
            TwistStamped, "/robot_lab_controller/cmd_vel", self._on_twist, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)
        self._rate = float(self.get_parameter("rate").value)
        self._timeout = float(self.get_parameter("cmd_vel_timeout").value)
        self._command = (0.0, 0.0, 0.0)
        self._command_time = None
        self._last_step = None
        self._wheel_prev = None
        self._joint_time = None
        self._odom = [0.0, 0.0, 0.0]
        self.create_service(Trigger, '/robot_lab/control_reset', self._reset_control)
        self.create_timer(1.0 / self._rate, self._step)
        self.get_logger().info(
            "%s drive: wheels %s; steering %s" %
            (self._drive.kind, self._wheel_joints, self._steer_joints))

    def _on_twist(self, msg):
        self._command = (msg.twist.linear.x, msg.twist.linear.y,
                         msg.twist.angular.z)
        self._command_time = self.get_clock().now()

    def _reset_control(self, _request, response):
        self._command = (0.0, 0.0, 0.0)
        self._command_time = None
        self._last_step = None
        self._wheel_prev = None
        self._joint_time = None
        self._drive.reset()
        # Keep the odom coordinate system continuous. AMCL resets map->odom
        # for the physical teleport; erasing wheel odom breaks the EKF history.
        self._step()
        response.success = True
        response.message = 'Drive stopped and control state reset; odom frame preserved'
        return response

    def _step(self):
        now = self.get_clock().now()
        stale = self._command_time is None or \
            (now - self._command_time).nanoseconds * 1e-9 > self._timeout
        vx, vy, wz = (0.0, 0.0, 0.0) if stale else self._command
        dt = 1.0 / self._rate
        if self._last_step is not None:
            elapsed = (now - self._last_step).nanoseconds * 1e-9
            if 0.0 < elapsed < 0.5:
                dt = elapsed
        self._last_step = now
        targets = self._drive.targets(vx, wz, dt=dt, vy=vy)
        if self._steer_pub is not None:
            self._steer_pub.publish(Float64MultiArray(
                data=[targets.position.get(j, 0.0) for j in self._steer_joints]))
        self._wheel_pub.publish(Float64MultiArray(
            data=[targets.velocity.get(j, 0.0) for j in self._wheel_joints]))

    def _on_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if any(j not in positions for j in self._wheel_joints):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        measured = [positions[j] for j in self._wheel_joints]
        if self._wheel_prev is None:
            self._wheel_prev = measured
            self._joint_time = stamp
            return
        dt = stamp - self._joint_time
        if dt <= 0.0 or dt > 1.0:
            self._wheel_prev = measured
            self._joint_time = stamp
            return
        rates = [(new - old) / dt for new, old in zip(measured, self._wheel_prev)]
        self._wheel_prev = measured
        self._joint_time = stamp
        if self._steer_joints:
            if any(j not in positions for j in self._steer_joints):
                return
            vx, vy, wz = self._drive.measured_twist(rates, positions)
        else:
            vx, vy, wz = self._drive.body_twist(rates)
        if not all(math.isfinite(v) for v in (vx, vy, wz)):
            return
        heading = self._odom[2] + wz * dt / 2.0
        self._odom[0] += (vx * math.cos(heading) - vy * math.sin(heading)) * dt
        self._odom[1] += (vx * math.sin(heading) + vy * math.cos(heading)) * dt
        self._odom[2] += wz * dt
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.get_parameter("base_frame").value
        odom.pose.pose.position.x, odom.pose.pose.position.y = self._odom[:2]
        odom.pose.pose.orientation.z = math.sin(self._odom[2] / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._odom[2] / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        for i, value in enumerate((0.001, 0.001, 1e-3, 1e-3, 1e-3, 0.01)):
            odom.pose.covariance[i * 7] = value
            odom.twist.covariance[i * 7] = value
        self._odom_pub.publish(odom)


def main():
    rclpy.init()
    node = HolonomicController()
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
