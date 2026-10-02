"""Live ROS 2 control of the local PX4 X500 SITL, with explicit takeoff.

20 Hz position/yaw setpoints run independently of operator input. Idle input
holds position; loss of this process invokes PX4's configured Land failsafe.
This is flight control, not obstacle-aware navigation or a Nav2 drone adapter.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from pymavlink import mavutil

from .px4_frames import attitude_enu_flu, enu_to_ned, ned_to_enu, yaw_enu
from .px4_mavlink import Flight, OFFBOARD_CUSTOM_MODE


class PX4Controller(Node):
    def __init__(self):
        super().__init__('px4_offboard_controller')
        for key, value in [('port', 'udpout:127.0.0.1:14580'), ('flight_enabled', True),
                           ('takeoff_alt', 3.0), ('origin_x', 0.0), ('origin_y', 0.0),
                           ('origin_z', 0.24)]:
            self.declare_parameter(key, value)
        self.enabled = self.get_parameter('flight_enabled').value
        self.origin = [self.get_parameter('origin_'+key).value for key in 'xyz']
        self.ground_z = self.origin[2]
        self.flight = Flight(mavutil.mavlink_connection(
            self.get_parameter('port').value, source_system=255, source_component=190))
        self.phase = 'waiting_for_fcu'
        self.target = None
        self.target_yaw = 0.0
        self.takeoff_z = None
        self.phase_since = self.last_tick = time.monotonic()
        self.last_manual = -math.inf
        self.manual = (0.0, 0.0, 0.0, 0.0)
        self.configured = False
        self.odom_pub = self.create_publisher(Odometry, '/px4/odometry', 10)
        self.status_pub = self.create_publisher(String, '/px4/status', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/key_vel', self.drive, 10)
        self.create_subscription(PoseStamped, '/px4/goal', self.goal, 10)
        self.create_service(Trigger, '/px4/takeoff', self.takeoff)
        self.create_service(Trigger, '/px4/land', self.land)
        self.create_service(Trigger, '/px4/hold', self.hold)
        self.create_timer(0.05, self.tick)
        self.create_timer(0.5, self.status)

    def position(self):
        p = self.flight.local_position
        if p is None:
            return None
        return [a+b for a, b in zip(ned_to_enu(p.x, p.y, p.z), self.origin)]

    def ready(self):
        return self.flight.state is not None and self.flight.position_ok() and time.monotonic()-self.flight.last_position_time < 1.0

    def transition(self, phase):
        self.phase, self.phase_since = phase, time.monotonic()
        self.get_logger().info('Flight state: '+phase)

    def takeoff(self, _request, response):
        if not self.enabled or not self.ready() or self.phase != 'idle' or self.flight.armed():
            response.success = False
            response.message = 'Takeoff requires Flight mode, a fresh FCU position and an idle, disarmed vehicle.'
            return response
        self.target = self.position()
        self.target_yaw = yaw_enu(self.flight.attitude.yaw) if self.flight.attitude else 0.0
        self.takeoff_z = self.target[2]+self.get_parameter('takeoff_alt').value
        self.last_manual = -math.inf
        self.transition('priming')
        response.success, response.message = True, 'Takeoff requested; priming the Offboard stream.'
        return response

    def command_mode(self, main, sub=0):
        f = self.flight
        f.master.mav.command_long_send(f.master.target_system, f.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, main, sub, 0, 0, 0, 0)

    def land(self, _request, response):
        response.success = self.enabled and self.ready() and self.flight.armed()
        response.message = 'Landing requested.' if response.success else 'No armed Flight vehicle with fresh telemetry.'
        if response.success:
            self.command_mode(4, 6)
            self.transition('landing')
        return response

    def hold(self, _request, response):
        response.success = self.phase in ('flying', 'ascending') and self.ready()
        response.message = 'Holding current position.' if response.success else 'Vehicle is not flying in Offboard.'
        if response.success:
            self.target = self.position()
            self.last_manual = -math.inf
            self.transition('flying')
        return response

    def drive(self, msg):
        if self.phase != 'flying' or not self.ready():
            return
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            return
        # A released GUI button publishes zero. Capture the current position
        # once when manual input ceases; keep streaming that hold thereafter.
        if not any(abs(v) > 1e-4 for v in values):
            if any(self.manual):
                self.target = self.position()
            self.manual = (0.0, 0.0, 0.0, 0.0)
        else:
            if time.monotonic()-self.last_manual > 0.5:
                self.target = self.position()
            self.manual = tuple(max(-1.0, min(1.0, v)) for v in values)
            norm = math.sqrt(sum(v*v for v in self.manual[:3]))
            if norm > 1.0:
                self.manual = tuple(v/norm for v in self.manual[:3])+(self.manual[3],)
        self.last_manual = time.monotonic()

    def goal(self, msg):
        if self.phase != 'flying' or msg.header.frame_id != 'map' or not self.ready():
            self.get_logger().warning('3D goal requires airborne Flight mode and frame_id map.')
            return
        p = msg.pose.position
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or not self.ground_z+0.5 <= p.z <= self.ground_z+10:
            self.get_logger().warning('Rejected invalid 3D goal/altitude (0.5 to 10 m above origin).')
            return
        self.target = [p.x, p.y, p.z]
        self.manual = (0.0, 0.0, 0.0, 0.0)
        self.last_manual = -math.inf
        q = msg.pose.orientation
        if sum(v*v for v in (q.x, q.y, q.z, q.w)) > 0.5:
            self.target_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def tick(self):
        f = self.flight
        f.poll()
        f.heartbeat()
        now = time.monotonic()
        dt, self.last_tick = min(0.1, now-self.last_tick), now
        if not self.ready():
            # Do not keep flying on stale state. Stop setpoints; FCU Land.
            if self.phase not in ('waiting_for_fcu', 'idle', 'landing', 'telemetry_lost'):
                self.transition('telemetry_lost')
            return
        if not self.configured:
            # Align PX4's local estimator origin once to the configured spawn.
            # Subsequent motion uses FCU telemetry only, never simulator truth.
            local = f.local_position
            self.origin = [a-b for a, b in zip(self.origin, ned_to_enu(local.x, local.y, local.z))]
            for name, value, ptype in [('COM_OBL_RC_ACT', 4, 6), ('COM_OF_LOSS_T', 0.5, 9),
                                       ('MPC_XY_VEL_MAX', 1.0, 9), ('MPC_XY_CRUISE', 1.0, 9),
                                       ('MPC_Z_VEL_MAX_UP', 1.0, 9), ('MPC_Z_VEL_MAX_DN', 1.0, 9),
                                       ('MPC_ACC_HOR', 1.0, 9)]:
                f.master.mav.param_set_send(f.master.target_system, f.master.target_component,
                                          name.encode(), value, ptype)
            self.configured = True
            self.transition('idle')
        if self.phase == 'landing':
            if not f.armed() and f.ext_sys.landed_state == 1:
                self.transition('idle')
            self.publish_pose()
            return
        if self.phase in ('ascending', 'flying') and not f.armed():
            self.transition('idle')
        elapsed = now-self.phase_since
        if self.phase == 'priming' and elapsed >= 2:
            self.command_mode(OFFBOARD_CUSTOM_MODE)
            self.transition('arming')
        elif self.phase == 'arming':
            if f.armed():
                self.transition('ascending')
            elif elapsed > 8:
                self.transition('idle')
                self.get_logger().error('PX4 refused arming; see FCU health in px4.log.')
            elif (f.state.custom_mode >> 16 & 0xff) == 6:
                f.master.mav.command_long_send(f.master.target_system, f.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        elif self.phase == 'ascending':
            self.target[2] = min(self.takeoff_z, self.target[2]+0.5*dt)
            if abs(self.position()[2]-self.takeoff_z) < 0.2:
                self.transition('flying')
        elif self.phase == 'flying':
            if (f.state.custom_mode >> 16 & 0xff) != 6:
                self.transition('landing' if f.armed() else 'idle')
            elif now-self.last_manual <= 0.5:
                vx, vy, vz, wz = self.manual
                yaw = yaw_enu(f.attitude.yaw) if f.attitude else self.target_yaw
                self.target[0] += (vx*math.cos(yaw)-vy*math.sin(yaw))*dt
                self.target[1] += (vx*math.sin(yaw)+vy*math.cos(yaw))*dt
                self.target[2] = max(self.ground_z+0.5, min(self.ground_z+10, self.target[2]+vz*dt))
                self.target_yaw += wz*dt
            elif any(self.manual):
                self.target = self.position()
                self.manual = (0.0, 0.0, 0.0, 0.0)
        if self.phase in ('priming', 'arming', 'ascending', 'flying'):
            xyz = enu_to_ned(*(a-b for a, b in zip(self.target, self.origin)))
            f.send_setpoint(*xyz, math.pi/2-self.target_yaw)
        self.publish_pose()

    def publish_pose(self):
        f = self.flight
        p = self.position()
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id, msg.child_frame_id = 'map', 'base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p
        if f.attitude:
            q = attitude_enu_flu(f.attitude.roll, f.attitude.pitch, f.attitude.yaw)
            msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        else:
            msg.pose.pose.orientation.w = 1.0
        yaw = yaw_enu(f.attitude.yaw) if f.attitude else 0.0
        vx, vy, vz = ned_to_enu(f.local_position.vx, f.local_position.vy, f.local_position.vz)
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = vx*math.cos(yaw)+vy*math.sin(yaw), -vx*math.sin(yaw)+vy*math.cos(yaw)
        msg.twist.twist.linear.z = vz
        self.odom_pub.publish(msg)
        tf = TransformStamped()
        tf.header, tf.child_frame_id = msg.header, msg.child_frame_id
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf.sendTransform(tf)

    def status(self):
        self.status_pub.publish(String(data=json.dumps({
            'phase': self.phase, 'armed': self.flight.armed(), 'position_ready': self.ready(),
            'flight_enabled': self.enabled, 'position_enu': self.position(), 'target_enu': self.target})))


def main(args=None):
    rclpy.init(args=args)
    node = PX4Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.flight.armed():
            node.command_mode(4, 6)
        node.flight.master.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
