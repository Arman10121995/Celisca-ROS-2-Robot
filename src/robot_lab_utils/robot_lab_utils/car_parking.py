#!/usr/bin/env python3
"""Park a car-like robot at a goal pose, heading included (ROADMAP R5.5).

Nav2's car stack (Hybrid-A* + Regulated Pure Pursuit) ends a goal at its
position; the heading is left to chance (8-23 degrees off in the car
course). This node runs the measured fix as a reusable runtime: the Nav2
goal, then the forward/reverse heading shuffle and the straight depth trim
of ``car_alignment`` (10 of 12 car-course cells parked within 0.003-0.079 m,
round 62). Pose is the localization estimate (map -> base frame), never
simulator truth.

Commands on ``/car_parking/command`` (JSON): ``{"op": "park", "x", "y",
"yaw", "timeout"}`` and ``{"op": "cancel"}``. ``/car_parking/state``
reports the phase (navigating, aligning, trimming, parked, failed,
canceled) and the estimated final errors.
"""
import json
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from robot_lab_utils.car_alignment import HeadingShuffle, longitudinal_error, trim_along_heading, wrap


class CarParking(Node):
    def __init__(self):
        super().__init__('car_parking')
        for key, value in dict(turning_radius=.5, base_frame='base_footprint', tolerance_deg=3., trim=True,
                               command_topic='/cmd_vel').items():
            self.declare_parameter(key, value)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.drive = self.create_publisher(Twist, self.get_parameter('command_topic').value, 10)
        self.state_pub = self.create_publisher(String, '/car_parking/state', 10)
        self.create_subscription(String, '/car_parking/command', self.command, 10)
        self.state = dict(phase='idle')
        self.cancel = threading.Event()
        self.worker = None
        self.goal_handle = None
        self.create_timer(.2, lambda: self.state_pub.publish(String(data=json.dumps(self.state))))

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', self.get_parameter('base_frame').value, rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001 - no transform yet
            return None
        q = t.rotation
        return (t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def publish(self, vx, wz):
        if self.cancel.is_set():
            vx = wz = 0.
        message = Twist()
        message.linear.x, message.angular.z = float(vx), float(wz)
        self.drive.publish(message)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not self.cancel.is_set():
            time.sleep(.02)

    def command(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('op') == 'cancel':
                self.cancel.set()
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.publish(0., 0.)
                return
            if data.get('op') != 'park':
                return
            goal = {k: float(data[k]) for k in ('x', 'y', 'yaw')}
            if not all(map(math.isfinite, goal.values())):
                raise ValueError('Finite goal x, y and yaw required')
            if self.worker is not None and self.worker.is_alive():
                raise ValueError('A parking manoeuvre is running; cancel it first')
            self.cancel = threading.Event()
            self.worker = threading.Thread(target=self.park, args=(goal, float(data.get('timeout', 180.))), daemon=True)
            self.worker.start()
        except (ValueError, KeyError, TypeError) as exc:
            self.state = dict(self.state, rejected=str(exc))

    def park(self, goal, timeout):
        started = time.monotonic()
        self.state = dict(phase='navigating', goal=goal, started=started)
        try:
            if not self.nav.wait_for_server(timeout_sec=5.):
                raise RuntimeError('Nav2 navigate_to_pose is not available')
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = 'map'
            request.pose.pose.position.x, request.pose.pose.position.y = goal['x'], goal['y']
            request.pose.pose.orientation.z, request.pose.pose.orientation.w = math.sin(goal['yaw'] / 2), math.cos(goal['yaw'] / 2)
            sent = self.nav.send_goal_async(request)
            while not sent.done():
                time.sleep(.05)
            self.goal_handle = sent.result()
            if not self.goal_handle.accepted:
                raise RuntimeError('Nav2 rejected the parking goal')
            result = self.goal_handle.get_result_async()
            while not result.done():
                if self.cancel.is_set():
                    raise InterruptedError('canceled')
                if time.monotonic() - started > timeout:
                    self.goal_handle.cancel_goal_async()
                    raise TimeoutError('Nav2 parking goal exceeded its budget')
                time.sleep(.05)
            status = result.result().status
            if status != 4:                      # GoalStatus.STATUS_SUCCEEDED
                raise RuntimeError('Nav2 ended with status %d' % status)
            arrival = self.pose()
            if arrival is None:
                raise RuntimeError('No localization estimate after Nav2 success')
            self.state.update(phase='aligning', nav2_heading_error_deg=round(math.degrees(wrap(goal['yaw'] - arrival[2])), 2))
            shuffle = HeadingShuffle(self.pose, self.publish, self.spin,
                                     turning_radius=self.get_parameter('turning_radius').value,
                                     tolerance=math.radians(self.get_parameter('tolerance_deg').value))
            self.state['alignment'] = {k: v for k, v in shuffle.run(goal['yaw']).items() if k != 'log'}
            if self.cancel.is_set():
                raise InterruptedError('canceled')
            if self.get_parameter('trim').value:
                self.state['phase'] = 'trimming'
                self.state['trim'] = trim_along_heading(self.pose, self.publish, self.spin, (goal['x'], goal['y']))
            if self.cancel.is_set():
                raise InterruptedError('canceled')
            end = self.pose()
            along, lateral = longitudinal_error(end, (goal['x'], goal['y']))
            self.state.update(phase='parked', position_error_m=round(math.hypot(end[0] - goal['x'], end[1] - goal['y']), 3),
                              heading_error_deg=round(math.degrees(wrap(goal['yaw'] - end[2])), 2),
                              along_m=round(along, 3), lateral_m=round(lateral, 3),
                              elapsed_s=round(time.monotonic() - started, 1),
                              observer='localization estimate (map -> base frame); not simulator truth')
        except InterruptedError:
            self.state.update(phase='canceled')
        except (RuntimeError, TimeoutError) as exc:
            self.state.update(phase='failed', reason=str(exc))
        finally:
            self.publish(0., 0.)
            self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = CarParking()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0., 0.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
