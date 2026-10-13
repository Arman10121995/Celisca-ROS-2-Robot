"""Leg kinematic odometry for walking robots (P7 mapping and navigation).

A foot in stance does not move in the world.  Its position in the base
frame, p_i(q), comes from the URDF kinematic chain, so

    0 = v_b + w_b x p_i + J_i(q) qdot      =>      v_b = -(J_i(q) qdot + w_b x p_i)

with v_b, w_b the base linear and angular velocity in the base frame (w_b
from the IMU gyro).  Each foot gives one such estimate; the stance feet are
chosen in two steps:

1. candidates: feet within ``stance_height`` of the lowest foot in the
   gravity-aligned base frame (IMU roll and pitch);
2. stance: candidates whose estimate lies within a gate of the previous
   velocity (``gate + gate_per_speed * |v|``) - a foot in swing moves at
   about twice the body speed, so its estimate is far off.  If none passes,
   the candidate closest to the previous velocity is used.

The estimate is the per-axis median over stance feet, low-passed with time
constant ``tau``.  Position is integrated in the plane with the IMU yaw.

Height alone is not enough: the Go2 himloco trot lifts its feet only a few
millimetres, so a 2 cm height window counted swing feet as stance and the
estimate read 34% short (MuJoCo, 2026-10-13); the velocity gate cut that to
3.7%, the remainder being foot slip in stance.

The node publishes ``nav_msgs/Odometry`` (twist in the base frame) for the
EKF's odom0, replacing the wheel odometry that a walking robot does not
have.  It uses only joint states, the IMU and the robot description - no
simulator truth.

Limits: no foot slip model, point feet (a flat humanoid foot is represented
by its ankle origin), and a wheeled foot (Go2W) violates the stance
assumption.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET

import numpy as np


# ---------------------------------------------------------------------------
# Kinematics (pure, unit-tested without ROS)
# ---------------------------------------------------------------------------

def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis = axis / max(np.linalg.norm(axis), 1e-12)
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return np.array([[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
                     [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
                     [t * x * z - s * y, t * y * z + s * x, t * z * z + c]])


def quaternion_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Joint:
    def __init__(self, element):
        self.name = element.get('name')
        self.type = element.get('type')
        self.parent = element.find('parent').get('link')
        self.child = element.find('child').get('link')
        origin = element.find('origin')
        xyz = [float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split()]
        rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None else '0 0 0').split()]
        self.translation = np.array(xyz)
        self.rotation = _rpy_matrix(*rpy)
        axis = element.find('axis')
        self.axis = np.array([float(v) for v in axis.get('xyz').split()]) if axis is not None else np.array([1.0, 0, 0])


class LegKinematics:
    """Base-frame foot positions and Jacobians from a URDF."""

    def __init__(self, urdf_xml, base_link=None, foot_links=None):
        root = ET.fromstring(urdf_xml)
        joints = [Joint(j) for j in root.findall('joint')]
        self.by_child = {j.child: j for j in joints}
        links = [link.get('name') for link in root.findall('link')]
        roots = [name for name in links if name not in self.by_child]
        self.base_link = base_link or (roots[0] if roots else links[0])
        if not foot_links:
            foot_links = [name for name in links if name.lower().endswith('foot')]
            if not foot_links:
                foot_links = [name for name in links if 'ankle_roll' in name.lower()]
        if not foot_links:
            raise ValueError('no foot links found; set foot_links')
        self.foot_links = list(foot_links)
        self.chains = {foot: self._chain(foot) for foot in self.foot_links}
        self.joint_names = sorted({j.name for chain in self.chains.values() for j in chain
                                   if j.type in ('revolute', 'continuous', 'prismatic')})

    def _chain(self, link):
        chain = []
        while link != self.base_link:
            if link not in self.by_child:
                raise ValueError('%s is not below %s' % (link, self.base_link))
            joint = self.by_child[link]
            chain.append(joint)
            link = joint.parent
        return list(reversed(chain))

    def frame(self, link, positions=None):
        """(R, p) of ``link`` in the base frame for fixed or given joint positions."""
        positions = positions or {}
        rotation, translation = np.eye(3), np.zeros(3)
        for joint in self._chain(link):
            translation = translation + rotation @ joint.translation
            rotation = rotation @ joint.rotation
            q = float(positions.get(joint.name, 0.0))
            if joint.type in ('revolute', 'continuous'):
                rotation = rotation @ _axis_angle(joint.axis, q)
            elif joint.type == 'prismatic':
                translation = translation + rotation @ (joint.axis * q)
        return rotation, translation

    def foot(self, foot, positions, velocities):
        """(p, v_rel): foot position and its velocity relative to the base, base frame."""
        rotation, translation = np.eye(3), np.zeros(3)
        columns = []  # (axis in base, joint origin in base, type, qdot)
        for joint in self.chains[foot]:
            translation = translation + rotation @ joint.translation
            rotation = rotation @ joint.rotation
            if joint.type in ('revolute', 'continuous', 'prismatic'):
                q = float(positions.get(joint.name, 0.0))
                axis = rotation @ (joint.axis / max(np.linalg.norm(joint.axis), 1e-12))
                columns.append((axis, translation.copy(), joint.type, float(velocities.get(joint.name, 0.0))))
                if joint.type == 'prismatic':
                    translation = translation + axis * q
                else:
                    rotation = rotation @ _axis_angle(joint.axis, q)
        velocity = np.zeros(3)
        for axis, origin, kind, qdot in columns:
            velocity += (axis if kind == 'prismatic' else np.cross(axis, translation - origin)) * qdot
        return translation, velocity


def tilt_matrix(orientation):
    """Roll/pitch part of a base orientation (the yaw removed)."""
    yaw = math.atan2(orientation[1, 0], orientation[0, 0])
    return _axis_angle((0, 0, 1), -yaw) @ orientation


def stance_velocity(feet, angular_velocity, orientation, stance_height=0.03, previous=None,
                    gate=0.15, gate_per_speed=0.3):
    """Base velocity (base frame) from stance feet -> (velocity or None, stance names).

    ``feet``: {name: (p, v_rel)} in the base frame; ``orientation``: base
    rotation in the world (only roll and pitch are used to find the ground);
    ``previous``: the last velocity estimate, for the swing-foot gate.
    """
    if not feet:
        return None, []
    tilt = tilt_matrix(orientation)
    heights = {name: float((tilt @ p)[2]) for name, (p, _) in feet.items()}
    lowest = min(heights.values())
    estimates = {name: -(v + np.cross(angular_velocity, p)) for name, (p, v) in feet.items()
                 if heights[name] <= lowest + stance_height}
    stance = sorted(estimates)
    if previous is not None:
        previous = np.asarray(previous, dtype=float)
        limit = gate + gate_per_speed * float(np.linalg.norm(previous[:2]))
        distance = {name: float(np.linalg.norm(e[:2] - previous[:2])) for name, e in estimates.items()}
        stance = sorted(name for name, d in distance.items() if d < limit) or [min(distance, key=distance.get)]
    return np.median(np.array([estimates[name] for name in stance]), axis=0), stance


class LegOdometry:
    """Integrates the stance velocity in the plane with the IMU yaw."""

    def __init__(self, kinematics, stance_height=0.03, imu_rotation=None, tau=0.02):
        self.kinematics = kinematics
        self.stance_height = stance_height
        self.tau = tau
        self.imu_rotation = np.eye(3) if imu_rotation is None else imu_rotation  # base <- imu
        self.x = self.y = 0.0
        self.last_time = None
        self.velocity = np.zeros(3)
        self.stance = []

    def update(self, stamp, positions, velocities, imu_orientation, imu_angular_velocity):
        """Advance to ``stamp`` (s); orientation as a 3x3 world <- imu matrix."""
        orientation = imu_orientation @ self.imu_rotation.T          # world <- base
        angular = self.imu_rotation @ np.asarray(imu_angular_velocity, dtype=float)
        feet = {name: self.kinematics.foot(name, positions, velocities) for name in self.kinematics.foot_links}
        velocity, self.stance = stance_velocity(feet, angular, orientation, self.stance_height,
                                                previous=self.velocity)
        dt = min(stamp - self.last_time, 0.2) if self.last_time is not None and stamp > self.last_time else 0.0
        if velocity is not None:
            blend = 1.0 if dt <= 0 or self.tau <= 0 else min(1.0, dt / self.tau)
            self.velocity = self.velocity + blend * (velocity - self.velocity)
        world = orientation @ self.velocity
        if dt > 0:
            self.x += float(world[0]) * dt
            self.y += float(world[1]) * dt
        self.last_time = stamp
        return orientation, angular


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState

    class LegOdometryNode(Node):
        def __init__(self):
            super().__init__('leg_odometry')
            self.declare_parameter('robot_description', '')
            self.declare_parameter('base_link', '')
            self.declare_parameter('foot_links', [''])
            self.declare_parameter('imu_link', 'imu_link')
            self.declare_parameter('imu_topic', '/imu/out')
            self.declare_parameter('joint_states_topic', '/joint_states')
            self.declare_parameter('odom_topic', '/leg_odometry')
            self.declare_parameter('odom_frame', 'odom')
            self.declare_parameter('stance_height', 0.03)
            self.declare_parameter('velocity_variance', 0.01)
            g = lambda name: self.get_parameter(name).value  # noqa: E731
            description = str(g('robot_description'))
            if not description.strip():
                raise RuntimeError('leg_odometry needs the robot_description parameter')
            feet = [f for f in (g('foot_links') or []) if f]
            self.kinematics = LegKinematics(description, base_link=str(g('base_link')) or None,
                                            foot_links=feet or None)
            imu_rotation = np.eye(3)
            imu_link = str(g('imu_link'))
            if imu_link and imu_link in self.kinematics.by_child:
                imu_rotation = self.kinematics.frame(imu_link)[0]
            self.odometry = LegOdometry(self.kinematics, float(g('stance_height')), imu_rotation)
            self.variance = float(g('velocity_variance'))
            self.imu = None
            self.pub = self.create_publisher(Odometry, str(g('odom_topic')), 20)
            self.create_subscription(Imu, str(g('imu_topic')), self.on_imu, qos_profile_sensor_data)
            self.create_subscription(JointState, str(g('joint_states_topic')), self.on_joints, 20)
            self.get_logger().info('leg odometry: base %s, feet %s' % (self.kinematics.base_link,
                                                                       ', '.join(self.kinematics.foot_links)))

        def on_imu(self, msg):
            q = msg.orientation
            self.imu = (quaternion_matrix(q.x, q.y, q.z, q.w),
                        (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))

        def on_joints(self, msg):
            if self.imu is None:
                return
            positions = dict(zip(msg.name, msg.position))
            velocities = dict(zip(msg.name, msg.velocity)) if len(msg.velocity) == len(msg.name) else {}
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            orientation, angular = self.odometry.update(stamp, positions, velocities, *self.imu)
            out = Odometry()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = str(self.get_parameter('odom_frame').value)
            out.child_frame_id = self.kinematics.base_link
            out.pose.pose.position.x, out.pose.pose.position.y = self.odometry.x, self.odometry.y
            yaw = math.atan2(orientation[1, 0], orientation[0, 0])
            out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            v = self.odometry.velocity
            out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = (float(v[0]), float(v[1]),
                                                                                           float(v[2]))
            out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = (
                float(angular[0]), float(angular[1]), float(angular[2]))
            variance = self.variance if self.odometry.stance else 1.0
            covariance = [0.0] * 36
            for i in range(6):
                covariance[i * 7] = variance
            out.twist.covariance = covariance
            out.pose.covariance = [c * 10 for c in covariance]
            self.pub.publish(out)

    rclpy.init(args=args)
    node = LegOdometryNode()
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
