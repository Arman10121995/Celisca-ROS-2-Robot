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
"yaw", "timeout", "instance_id"}`` and ``{"op": "cancel", "instance_id"}``.
The current instance comes from ``/car_parking/state``; an operator lease on
``/car_parking/heartbeat`` is required throughout the operation. The final
correction checks the live local costmap before each nonzero command.
``/car_parking/state``
reports the phase (navigating, aligning, trimming, parked, failed,
canceled) and the estimated final errors.
"""
import json
import math
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Empty
from nav_msgs.msg import OccupancyGrid
from lifecycle_msgs.srv import GetState
from action_msgs.msg import GoalStatusArray
from tf2_ros import Buffer, TransformListener

from robot_lab_utils.car_alignment import HeadingShuffle, longitudinal_error, trim_along_heading, wrap
from robot_lab_utils.car_parking_clearance import footprint_points, swept_clear


class CarParking(Node):
    def __init__(self):
        super().__init__('car_parking')
        for key, value in dict(turning_radius=.5, base_frame='base_footprint', tolerance_deg=3., trim=True,
                               command_topic='cmd_vel',robot_model='',operator_timeout=.8,
                               footprint=[.42,.17,.42,-.17,-.10,-.17,-.10,.17]).items():
            self.declare_parameter(key, value)
        self.footprint=footprint_points(self.get_parameter('footprint').value)
        lease=float(self.get_parameter('operator_timeout').value)
        tolerance=float(self.get_parameter('tolerance_deg').value)
        radius=float(self.get_parameter('turning_radius').value)
        if not math.isfinite(lease) or not .2<=lease<=2.:raise ValueError('Operator lease must be 0.2–2 seconds')
        if not math.isfinite(tolerance) or not .1<=tolerance<=5.:raise ValueError('Heading tolerance must be 0.1–5 degrees')
        if not math.isfinite(radius) or radius<=0:raise ValueError('Positive finite source turning radius required')
        self.instance_id=uuid.uuid4().hex
        self.last_heartbeat=-math.inf;self.deadline=math.inf
        self.costmap=None;self.costmap_received=-math.inf
        self.active_nav_goals=set()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lifecycle=self.create_client(GetState,'bt_navigator/get_state')
        self.lifecycle_future=None;self.lifecycle_received=-math.inf;self.navigator_active=False
        self.drive = self.create_publisher(Twist, self.get_parameter('command_topic').value, 10)
        self.state_pub = self.create_publisher(String, 'car_parking/state', 10)
        self.create_subscription(String, 'car_parking/command', self.command, 10)
        self.create_subscription(Empty,'car_parking/heartbeat',lambda _:setattr(self,'last_heartbeat',time.monotonic()),10)
        self.create_subscription(OccupancyGrid,'local_costmap/costmap',self.receive_costmap,10)
        self.create_subscription(GoalStatusArray,'navigate_to_pose/_action/status',self.navigation_status,10)
        self.state = dict(phase='idle')
        self.cancel = threading.Event()
        self.worker = None
        self.goal_handle = None
        self.create_timer(.2,self.publish_state)

    def publish_state(self):
        if self.lifecycle.service_is_ready() and (self.lifecycle_future is None or self.lifecycle_future.done()):
            self.lifecycle_future=self.lifecycle.call_async(GetState.Request())
            def lifecycle_state(future):
                try:self.navigator_active=future.result().current_state.id==3
                except Exception:self.navigator_active=False
                self.lifecycle_received=time.monotonic()
            self.lifecycle_future.add_done_callback(lifecycle_state)
        state=dict(self.state,instance_id=self.instance_id,robot_model=self.get_parameter('robot_model').value,
                   pose=self.pose(),navigation_ready=self.navigator_active and time.monotonic()-self.lifecycle_received<1.)
        self.state_pub.publish(String(data=json.dumps(state)))

    def receive_costmap(self,message):
        self.costmap=message;self.costmap_received=time.monotonic()

    def navigation_status(self,message):
        self.active_nav_goals={bytes(item.goal_info.goal_id.uuid) for item in message.status_list if item.status in (1,2,3)}

    def check_budget(self):
        if self.cancel.is_set():raise InterruptedError('canceled')
        if time.monotonic()-self.last_heartbeat>self.get_parameter('operator_timeout').value:
            raise RuntimeError('Parking operator heartbeat lost')
        if time.monotonic()>self.deadline:raise TimeoutError('Parking exceeded its whole-operation budget')
        if not self.navigator_active or time.monotonic()-self.lifecycle_received>1.:
            raise RuntimeError('Nav2 navigator became inactive or its lifecycle feedback is stale')
        if self.state.get('phase') in ('aligning','trimming'):
            own=bytes(self.goal_handle.goal_id.uuid) if self.goal_handle is not None else None
            if any(goal!=own for goal in self.active_nav_goals):
                raise RuntimeError('Another navigation goal took ownership during parking correction')

    def fresh_transform(self,frame):
        transform=self.tf.lookup_transform(frame,self.get_parameter('base_frame').value,rclpy.time.Time())
        age=(self.get_clock().now()-rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds*1e-9
        if not -.1<=age<=.5:raise RuntimeError('Localization transform is stale or future-dated')
        t=transform.transform;q=t.rotation
        result=(t.translation.x,t.translation.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        if not all(map(math.isfinite,result)):raise RuntimeError('Nonfinite localization pose')
        return result

    def pose(self):
        try:
            return self.fresh_transform('map')
        except Exception:  # noqa: BLE001 - no transform yet
            return None

    def publish(self, vx, wz):
        if not self.context.ok():return
        if vx or wz:
            self.check_budget()
            if self.pose() is None:raise RuntimeError('Fresh localization is required during parking')
            grid=self.costmap
            if grid is None or time.monotonic()-self.costmap_received>1.:
                raise RuntimeError('Fresh local costmap required for parking correction')
            age=(self.get_clock().now()-rclpy.time.Time.from_msg(grid.header.stamp)).nanoseconds*1e-9
            if not -.1<=age<=1.:raise RuntimeError('Local costmap is stale or future-dated')
            try:pose=self.fresh_transform(grid.header.frame_id)
            except Exception as exc:raise RuntimeError('No fresh pose in local costmap frame') from exc
            if not swept_clear(grid,self.footprint,pose,vx,wz):
                raise RuntimeError('Parking correction crosses occupied, unknown or out-of-grid space')
        if self.cancel.is_set():
            vx = wz = 0.
        message = Twist()
        message.linear.x, message.angular.z = float(vx), float(wz)
        self.drive.publish(message)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not self.cancel.is_set():
            self.check_budget()
            time.sleep(.02)
        self.check_budget()

    def command(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data,dict):raise ValueError('Parking command must be an object')
            if data.get('instance_id')!=self.instance_id:raise ValueError('Parking instance changed; reconnect')
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
            timeout=float(data.get('timeout',180.))
            if not math.isfinite(timeout) or not 5.<=timeout<=600.:raise ValueError('Parking budget must be 5–600 seconds')
            if time.monotonic()-self.last_heartbeat>self.get_parameter('operator_timeout').value:
                raise ValueError('Fresh operator heartbeat required')
            if not self.navigator_active or time.monotonic()-self.lifecycle_received>1.:
                raise ValueError('Wait for the active Nav2 navigator')
            if self.worker is not None and self.worker.is_alive():
                raise ValueError('A parking manoeuvre is running; cancel it first')
            self.cancel = threading.Event()
            self.worker = threading.Thread(target=self.park, args=(goal, timeout), daemon=True)
            self.worker.start()
        except (ValueError, KeyError, TypeError) as exc:
            self.state = dict(self.state, rejected=str(exc))

    def park(self, goal, timeout):
        started = time.monotonic()
        self.deadline=started+timeout
        self.state = dict(phase='navigating', goal=goal, started=started)
        try:
            self.check_budget()
            if not self.nav.wait_for_server(timeout_sec=5.):
                raise RuntimeError('Nav2 navigate_to_pose is not available')
            self.check_budget()
            request = NavigateToPose.Goal()
            request.pose.header.frame_id = 'map'
            request.pose.pose.position.x, request.pose.pose.position.y = goal['x'], goal['y']
            request.pose.pose.orientation.z, request.pose.pose.orientation.w = math.sin(goal['yaw'] / 2), math.cos(goal['yaw'] / 2)
            sent = self.nav.send_goal_async(request)
            # A late acceptance after cancel/timeout must not leave an orphan
            # navigator driving after this worker has stopped waiting.
            cancellation=self.cancel
            def accepted(future):
                try:handle=future.result()
                except Exception:return  # Worker reports the failed transport.
                if cancellation.is_set() and handle and handle.accepted and self.context.ok():handle.cancel_goal_async()
            sent.add_done_callback(accepted)
            while not sent.done():
                self.check_budget()
                time.sleep(.05)
            self.goal_handle = sent.result()
            if not self.goal_handle.accepted:
                raise RuntimeError('Nav2 rejected the parking goal')
            result = self.goal_handle.get_result_async()
            while not result.done():
                self.check_budget()
                time.sleep(.05)
            self.check_budget()
            status = result.result().status
            if status != 4:                      # GoalStatus.STATUS_SUCCEEDED
                raise RuntimeError('Nav2 ended with status %d' % status)
            arrival = self.pose()
            if arrival is None:
                raise RuntimeError('No localization estimate after Nav2 success')
            self.state.update(phase='aligning', nav2_heading_error_deg=round(math.degrees(wrap(goal['yaw'] - arrival[2])), 2))
            def required_pose():
                self.check_budget()
                current=self.pose()
                if current is None:raise RuntimeError('Fresh localization lost during parking correction')
                return current
            shuffle = HeadingShuffle(required_pose, self.publish, self.spin,
                                     turning_radius=self.get_parameter('turning_radius').value,
                                     tolerance=math.radians(self.get_parameter('tolerance_deg').value))
            self.state['alignment'] = {k: v for k, v in shuffle.run(goal['yaw']).items() if k != 'log'}
            if self.cancel.is_set():
                raise InterruptedError('canceled')
            if self.get_parameter('trim').value:
                self.state['phase'] = 'trimming'
                self.state['trim'] = trim_along_heading(required_pose, self.publish, self.spin, (goal['x'], goal['y']))
            if self.cancel.is_set():
                raise InterruptedError('canceled')
            end = required_pose()
            along, lateral = longitudinal_error(end, (goal['x'], goal['y']))
            error=math.hypot(end[0]-goal['x'],end[1]-goal['y'])
            heading=abs(wrap(goal['yaw']-end[2]))
            within=error<=.1 and heading<=math.radians(self.get_parameter('tolerance_deg').value)
            self.state.update(phase='parked' if within else 'failed',position_error_m=round(error,3),
                              heading_error_deg=round(math.degrees(wrap(goal['yaw'] - end[2])), 2),
                              along_m=round(along, 3), lateral_m=round(lateral, 3),
                              elapsed_s=round(time.monotonic() - started, 1),
                              observer='localization estimate (map -> base frame); not simulator truth')
            if not within:self.state['reason']='Correction ended outside position/heading tolerance'
        except InterruptedError:
            self.state.update(phase='canceled')
        except Exception as exc:  # A failed ROS future/transform must also reach terminal Stop.
            self.state.update(phase='failed', reason=type(exc).__name__+': '+str(exc))
        finally:
            self.cancel.set()
            if self.goal_handle is not None and self.context.ok():self.goal_handle.cancel_goal_async()
            self.publish(0., 0.)
            self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = CarParking()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.cancel.set()
        if rclpy.ok():
            if node.goal_handle is not None:node.goal_handle.cancel_goal_async()
            node.publish(0., 0.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
