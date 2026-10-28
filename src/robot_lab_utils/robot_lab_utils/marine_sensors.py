#!/usr/bin/env python3
"""Explicit pressure/Janus-DVL simulator and separate sensor-only odometry.

The DVL/pressure model derives noisy measurements from physical simulator
poses and static bottom-lock geometry. It is authored sensor emulation, not
a native calibrated DVL. The estimator subscribes to neither pose truth nor
the controller's ideal-pressure topic. Native IMU supplies attitude.
"""
import json
import copy
import math
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure, Imu, Range
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from robot_lab_utils.aerial_planning import StaticScene
from robot_lab_utils.marine_controller import rpy
from robot_lab_utils.marine_model import rotation
from robot_lab_utils.marine_navigation_model import MarineOdometry, bottom_ranges, dvl_velocity, sonar_range


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class MarineSensors(Node):
    def __init__(self):
        super().__init__('marine_sensor_simulator')
        for key, value in dict(world_file='', density=1025., surface_z=0., seed=41).items():
            self.declare_parameter(key, value)
        self.scene = StaticScene(self.get_parameter('world_file').value) if self.get_parameter('world_file').value else None
        self.density = float(self.get_parameter('density').value)
        self.surface = float(self.get_parameter('surface_z').value)
        self.seed = int(self.get_parameter('seed').value)
        self.rng = np.random.default_rng(self.seed)
        self.bias = self.rng.normal(0., .002, 3)
        self.previous,self.last_output=None,-math.inf
        self.state_pub=self.create_publisher(String,'/marine/sensors/state',10)
        self.motion_subscriber=self.create_subscription(Odometry,'/marine/odometry',self.observe,10)
        self.sonar_rng=np.random.default_rng(self.seed+1)
        self.sonar_previous = None
        self.sonar_last_output = -math.inf
        self.velocity_pub = self.create_publisher(TwistWithCovarianceStamped, '/marine/sensors/dvl', 10)
        self.pressure_pub = self.create_publisher(FluidPressure, '/marine/sensors/pressure', 10)
        self.sonar_pub = self.create_publisher(Range, '/marine/sensors/sonar', 10)
        self.sonar_subscriber = self.create_subscription(Odometry, '/marine/odometry', self.sonar_observe, 10)

    def sonar_observe(self, msg):
        p = msg.pose.pose.position
        current = stamp(msg)
        if msg.header.frame_id != 'map' or not np.isfinite([p.x, p.y, p.z]).all():
            return
        if self.sonar_previous is not None and current <= stamp(self.sonar_previous):
            self.sonar_previous = None
            return
        if self.sonar_previous is not None and current - stamp(self.sonar_previous) < .095:
            return
        self.sonar_previous = msg
        depth = max(0., self.surface - p.z)
        q=msg.pose.pose.orientation
        if q is None or not np.isfinite([q.x,q.y,q.z,q.w]).all() or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.05:
            return
        if depth > 0.:
            range_to_seabed = sonar_range(self.scene,(p.x,p.y,p.z),rpy(q))
        else:
            range_to_seabed = float('inf')
        sonar = Range()
        sonar.header = copy.copy(msg.header)
        sonar.header.frame_id = 'base_link'
        sonar.radiation_type = Range.ULTRASOUND
        sonar.field_of_view = .5
        sonar.min_range = .1
        sonar.max_range = 10.
        if depth <= 0. or not math.isfinite(range_to_seabed):
            sonar.range = float('inf')
        else:
            sonar.range = float(np.clip(range_to_seabed+self.sonar_rng.normal(0.,.01), sonar.min_range, sonar.max_range))
        self.sonar_pub.publish(sonar)


    def observe(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        current = stamp(msg)
        if msg.header.frame_id != 'map' or not np.isfinite([p.x, p.y, p.z, q.x, q.y, q.z, q.w, current]).all():
            return
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .05:
            return
        if self.previous is not None and current <= stamp(self.previous):
            self.previous, self.last_output = None, -math.inf
            return
        if current - self.last_output < .095:
            return
        previous, self.previous = self.previous, msg
        self.last_output = current
        pressure = FluidPressure()
        pressure.header = copy.copy(msg.header)
        pressure.header.frame_id = 'base_link'
        pressure.fluid_pressure = 101325. + self.density * 9.81 * max(0., self.surface - p.z) + float(self.rng.normal(0., 75.))
        pressure.variance = 75. ** 2
        self.pressure_pub.publish(pressure)
        if previous is None or not 0 < current - stamp(previous) < .5:
            return
        dt = current - stamp(previous)
        old = previous.pose.pose.position
        world_velocity = np.array([(p.x-old.x)/dt, (p.y-old.y)/dt, (p.z-old.z)/dt])
        orientation = rpy(q)
        ranges = bottom_ranges(self.scene, (p.x, p.y, p.z), orientation) if self.scene is not None else [math.inf]*4
        locked = all(math.isfinite(v) and .1 < v <= 10. for v in ranges)
        if locked:
            value, covariance, beams = dvl_velocity(rotation(orientation).T @ world_velocity + self.bias, self.rng)
            output = TwistWithCovarianceStamped()
            output.header = pressure.header
            for i, key in enumerate('xyz'):
                setattr(output.twist.twist.linear, key, float(value[i]))
                for j in range(3):
                    output.twist.covariance[i*6+j] = float(covariance[i, j] + (.002 ** 2 if i == j else 0.))
            self.velocity_pub.publish(output)
        self.state_pub.publish(String(data=json.dumps(dict(bottom_lock=locked,
            beam_ranges_m=[v if math.isfinite(v) else None for v in ranges], seed=self.seed,
            pressure_noise_Pa=75., beam_velocity_noise_m_s=.005, velocity_bias_std_m_s=.002,
            sonar_noise_m=.01,sonar_model='Body-down ray against selected static collision world; separate from DVL and pressure',
            source='Authored noisy hydrostatic/Janus-DVL simulator using actual plant motion and static world bottom intersections',
            native_sensor=False, dynamic_bottom_lock=False), allow_nan=False)))


class MarineEstimator(Node):
    def __init__(self):
        super().__init__('marine_sensor_odometry')
        for key, value in dict(initial_x=0., initial_y=0., initial_z=-1.,initial_yaw=0., density=1025., surface_z=0.).items():
            self.declare_parameter(key, value)
        self.filter = MarineOdometry([self.get_parameter('initial_'+k).value for k in 'xyz'],
                                     self.get_parameter('density').value)
        self.surface = float(self.get_parameter('surface_z').value)
        self.initial_yaw=float(self.get_parameter('initial_yaw').value)
        self.imu_yaw_offset=None
        self.imu = self.pressure = None
        self.count = 0
        self.odom_pub = self.create_publisher(Odometry, '/marine/estimated_odometry', 10)
        self.state_pub = self.create_publisher(String, '/marine/estimator/state', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, '/marine/imu', self.observe_imu, qos_profile_sensor_data)
        self.create_subscription(FluidPressure, '/marine/sensors/pressure', self.observe_pressure, 10)
        self.create_subscription(TwistWithCovarianceStamped, '/marine/sensors/dvl', self.observe_velocity, 10)

    def observe_imu(self, msg):
        q = msg.orientation
        if (msg.header.frame_id == 'base_link' and msg.orientation_covariance[0] != -1
                and np.isfinite([q.x, q.y, q.z, q.w]).all()
                and abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) < .05):
            # Fortress IMU heading starts at zero at spawn, including a hull
            # spawned at pi radians. Align it once to the configured prior;
            # subsequent changes remain native sensor observations. Without
            # this, AUV DVL travel and camera clouds were rotated by pi.
            if self.imu_yaw_offset is None:
                self.imu_yaw_offset=Rotation.from_euler('z',self.initial_yaw-rpy(q)[2])
            aligned=copy.deepcopy(msg)
            values=(self.imu_yaw_offset*Rotation.from_quat([q.x,q.y,q.z,q.w])).as_quat()
            for key,value in zip('xyzw',values):setattr(aligned.orientation,key,float(value))
            self.imu=aligned

    def observe_pressure(self, msg):
        if (msg.header.frame_id == 'base_link' and math.isfinite(msg.fluid_pressure)
                and math.isfinite(msg.variance) and msg.variance > 0):
            self.pressure = msg

    def observe_velocity(self, msg):
        if self.imu is None or self.pressure is None or msg.header.frame_id != 'base_link':
            return
        current = stamp(msg)
        if abs(current - stamp(self.imu)) > .2 or abs(current - stamp(self.pressure)) > .2:
            return
        velocity = [getattr(msg.twist.twist.linear, k) for k in 'xyz']
        covariance = np.asarray(msg.twist.covariance).reshape(6, 6)[:3, :3]
        try:
            position = self.filter.update(current, velocity, covariance, rpy(self.imu.orientation),
                self.pressure.fluid_pressure, self.pressure.variance, self.surface)
        except ValueError as exc:
            self.get_logger().warning('Marine odometry refused sample: ' + str(exc))
            return
        self.count += 1
        odom = Odometry()
        odom.header = copy.copy(msg.header)
        odom.header.frame_id = 'marine_sensor_odom'
        odom.child_frame_id = 'marine_estimated_base_link'
        for i, key in enumerate('xyz'):
            setattr(odom.pose.pose.position, key, float(position[i]))
            setattr(odom.twist.twist.linear, key, float(velocity[i]))
            for j in range(3):
                odom.pose.covariance[i*6+j] = float(self.filter.covariance[i, j])
        odom.pose.pose.orientation = self.imu.orientation
        if not all(math.isfinite(getattr(self.imu.angular_velocity,key)) for key in 'xyz'):return
        odom.twist.twist.angular = self.imu.angular_velocity
        odom.pose.covariance[21] = odom.pose.covariance[28] = odom.pose.covariance[35] = .005 ** 2
        odom.twist.covariance = msg.twist.covariance
        for i in range(3):
            for j in range(3):odom.twist.covariance[(i+3)*6+j+3]=self.imu.angular_velocity_covariance[i*3+j]
        self.odom_pub.publish(odom)
        transform = TransformStamped()
        transform.header, transform.child_frame_id = odom.header, odom.child_frame_id
        for key in 'xyz':
            setattr(transform.transform.translation, key, getattr(odom.pose.pose.position, key))
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)
        self.state_pub.publish(String(data=json.dumps(dict(samples=self.count, sim_time=current,
            position=position.tolist(), covariance=self.filter.covariance.tolist(),
            source='DVL + native IMU with one-time configured spawn-heading alignment + noisy pressure; no simulator pose correction',
            qualification='Experimental emulated sensors and drifting odometry, not calibrated DVL or SLAM'), allow_nan=False)))


def main(args=None):
    import sys
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent()
    kind = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in ('simulate', 'estimate') else 'simulate'
    if len(sys.argv) > 1 and sys.argv[1] in ('simulate', 'estimate'):
        del sys.argv[1]
    rclpy.init(args=args)
    node = MarineSensors() if kind == 'simulate' else MarineEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
