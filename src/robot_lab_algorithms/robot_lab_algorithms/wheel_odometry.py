"""Wheel odometry from /joint_states for any lab drive (diff, car, 4WS, mecanum).

The Gazebo controllers publish wheel odometry on /robot_lab_controller/odom;
the MuJoCo, PyBullet and Isaac bridges publish only joint states and the
simulator's truth.  This node computes the same wheel odometry from the
measured wheel spin (and steering angles) with the drive model of
robot_lab_utils.drive_kinematics, so estimators can run on wheel odometry
in every backend instead of truth.

* diff - (vx, wz) from the two wheels;
* ackermann - from the fixed axle's two wheels;
* four_wheel_steer - least-squares rolling and no-side-slip constraints over
  the four wheels and their measured steering angles;
* mecanum - (vx, vy, wz) from the four roller wheels.

Publishes nav_msgs/Odometry (twist in the base frame, pose integrated in
``odom``).  It does not publish TF.
"""
from __future__ import annotations

import math


class WheelOdometry:
    def __init__(self, drive):
        self.drive = drive
        self.x = self.y = self.yaw = 0.0
        self.twist = (0.0, 0.0, 0.0)
        self.last = None

    def body_twist(self, rates, positions):
        kind = self.drive.kind
        if kind == 'diff':
            vx, wz = self.drive.body_twist(rates.get(self.drive.left, 0.0), rates.get(self.drive.right, 0.0))
            return vx, 0.0, wz
        if kind == 'ackermann':
            vx, wz = self.drive.body_twist(rates.get(self.drive.left, 0.0), rates.get(self.drive.right, 0.0))
            return vx, 0.0, wz
        if kind == 'four_wheel_steer':
            wheels = [rates.get(j, 0.0) for j in self.drive.wheel_joints]
            angles = {j: positions.get(j, 0.0) for j in self.drive.steer_joints}
            return self.drive.measured_twist(wheels, angles)
        if kind == 'mecanum':
            return self.drive.body_twist([rates.get(j, 0.0) for j in self.drive.wheel_joints])
        vx, wz = self.drive.body_twist(rates.get(getattr(self.drive, 'left', ''), 0.0),
                                       rates.get(getattr(self.drive, 'right', ''), 0.0))
        return vx, 0.0, wz

    def update(self, stamp, rates, positions):
        vx, vy, wz = self.body_twist(rates, positions)
        if self.last is not None and stamp > self.last:
            dt = min(stamp - self.last, 0.2)
            heading = self.yaw + 0.5 * wz * dt  # midpoint heading
            self.x += (vx * math.cos(heading) - vy * math.sin(heading)) * dt
            self.y += (vx * math.sin(heading) + vy * math.cos(heading)) * dt
            self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))
        self.last = stamp
        self.twist = (vx, vy, wz)
        return self.twist


def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from robot_lab_utils.drive_kinematics import drive_from_config, parse_drive_config

    class WheelOdometryNode(Node):
        def __init__(self):
            super().__init__('wheel_odometry')
            self.declare_parameter('drive_config', '')
            self.declare_parameter('left_wheel_joint', 'wheel_left_joint')
            self.declare_parameter('right_wheel_joint', 'wheel_right_joint')
            self.declare_parameter('wheel_radius', 0.033)
            self.declare_parameter('wheel_separation', 0.17)
            self.declare_parameter('base_frame', 'base_footprint')
            self.declare_parameter('odom_topic', '/robot_lab_controller/odom')
            g = lambda name: self.get_parameter(name).value  # noqa: E731
            drive = drive_from_config(parse_drive_config(g('drive_config')), g('left_wheel_joint'),
                                      g('right_wheel_joint'), float(g('wheel_radius')),
                                      float(g('wheel_separation')))
            self.odometry = WheelOdometry(drive)
            self.pub = self.create_publisher(Odometry, g('odom_topic'), 20)
            self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
            self.get_logger().info('wheel odometry (%s drive, joints %s) -> %s' % (
                drive.kind, ', '.join(drive.wheel_joints), g('odom_topic')))

        def on_joints(self, msg):
            if not msg.velocity:
                return
            rates = dict(zip(msg.name, msg.velocity))
            positions = dict(zip(msg.name, msg.position))
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            vx, vy, wz = self.odometry.update(stamp, rates, positions)
            out = Odometry()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = 'odom'
            out.child_frame_id = self.get_parameter('base_frame').value
            out.pose.pose.position.x, out.pose.pose.position.y = self.odometry.x, self.odometry.y
            out.pose.pose.orientation.z = math.sin(self.odometry.yaw / 2)
            out.pose.pose.orientation.w = math.cos(self.odometry.yaw / 2)
            out.twist.twist.linear.x, out.twist.twist.linear.y = float(vx), float(vy)
            out.twist.twist.angular.z = float(wz)
            covariance = [0.0] * 36
            for i in (0, 7, 35):
                covariance[i] = 0.01
            out.twist.covariance = covariance
            self.pub.publish(out)

    rclpy.init(args=args)
    node = WheelOdometryNode()
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
