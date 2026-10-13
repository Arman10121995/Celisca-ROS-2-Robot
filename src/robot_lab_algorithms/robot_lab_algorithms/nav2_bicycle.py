"""Car-only Nav2 FollowPath with bounded bicycle tracking and full footprint.

This separate server leaves the qualified differential-controller adapter
unchanged. It does not rotate cars in place or silently drop reverse cusps.
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from action_msgs.srv import CancelGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import FollowPath
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

from .bicycle_control import BicycleTracker, LAWS, swept_footprint_blocked
from .grid_planners import GridView
from .tracking_control import wrap


def yaw(quaternion):
    return math.atan2(2*(quaternion.w*quaternion.z+quaternion.x*quaternion.y),
        1-2*(quaternion.y**2+quaternion.z**2))


class BicycleFollowPath(Node):
    def __init__(self):
        super().__init__('robot_lab_bicycle_controller')
        for name, default in [('planner', 'bicycle_pure_pursuit'), ('source_profile', ''),
                ('action_name', '/robot_lab/bicycle_follow_path'), ('max_linear', .3),
                ('rate', 20.), ('xy_goal_tolerance', .12), ('yaw_goal_tolerance', .15)]:
            self.declare_parameter(name, default)
        self.law = self.get_parameter('planner').value
        self.profile = json.loads(self.get_parameter('source_profile').value)
        if self.profile.get('drive', {}).get('type') != 'ackermann' or self.law not in LAWS:
            raise ValueError('Bicycle tracking requires an authored car drive and its launch source profile.')
        self.footprint = np.asarray(self.profile['footprint'], dtype=float)
        if self.footprint.shape != (4, 2) or not np.all(np.isfinite(self.footprint)):
            raise ValueError('Initial bicycle adapter requires the car overlay rectangle footprint.')
        self.frame = self.profile['base_frame']
        self.speed = min(float(self.get_parameter('max_linear').value), self.profile['drive']['max_speed'])
        self.period = 1/float(self.get_parameter('rate').value)
        if not .02 <= self.period <= .1:
            raise ValueError('Use a bounded 10–50 Hz bicycle control rate.')
        BicycleTracker(self.law, self.profile['drive'], max_speed=self.speed)
        self.lock = threading.Lock()
        self.active, self.odom, self.grid = None, None, None
        self.paused = False
        self.last_good, self.last_emit = -math.inf, -math.inf
        self.tf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf, self)
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 1)
        self.status = self.create_publisher(String, '/robot_lab/bicycle/status', 10)
        self.group = ReentrantCallbackGroup()
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data, callback_group=self.group)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_grid, 1, callback_group=self.group)
        self.parent_cancel = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal', callback_group=self.group)
        self.create_subscription(Twist, '/key_vel', self.manual_input, 1, callback_group=self.group)
        self.create_subscription(PoseStamped, '/goal_pose', self.new_operator_goal, 1, callback_group=self.group)
        self.create_service(Trigger, '/robot_lab/bicycle/stop', self.operator_stop, callback_group=self.group)
        self.create_service(Trigger, '/robot_lab/bicycle/resume', self.operator_resume, callback_group=self.group)
        self.server = ActionServer(self, FollowPath, self.get_parameter('action_name').value,
            execute_callback=self.execute, goal_callback=self.goal_response,
            handle_accepted_callback=self.accepted, cancel_callback=self.cancel,
            callback_group=self.group)
        self.create_timer(.05, self.watchdog, clock=Clock(clock_type=ClockType.SYSTEM_TIME), callback_group=self.group)

    def on_odom(self, message):
        values = (message.twist.twist.linear.x, message.twist.twist.angular.z)
        if all(map(math.isfinite, values)):
            with self.lock:
                self.odom = (values, time.monotonic(), message.header.stamp)

    def goal_response(self, _request):
        with self.lock:
            return GoalResponse.REJECT if self.paused else GoalResponse.ACCEPT

    def manual_input(self, _message):
        with self.lock:
            active = self.active is not None
        if active:
            self.request_operator_stop()

    def request_operator_stop(self):
        with self.lock:
            self.paused = True
            self.cmd.publish(Twist())
        # One robot is owned by this isolated ROS graph. Explicit operator
        # intervention cancels its NavigateToPose goal as well as FollowPath,
        # preventing the navigator from retrying after an inner-action abort.
        if self.parent_cancel.service_is_ready():
            request = CancelGoal.Request()
            request.goal_info.stamp = self.get_clock().now().to_msg()
            self.parent_cancel.call_async(request)

    def new_operator_goal(self, _message):
        with self.lock:
            self.paused = False

    def operator_stop(self, _request, response):
        self.request_operator_stop()
        response.success, response.message = True, 'Car tracking paused; local NavigateToPose cancel requested. New RViz goal or Resume releases pause.'
        return response

    def operator_resume(self, _request, response):
        with self.lock:
            self.paused = False
        response.success, response.message = True, 'Car controller accepts new paths; no movement was requested.'
        return response

    def on_grid(self, message):
        # Nav2's rolling costmap has an unrotated raster origin. Reject a
        # rotated/invalid representation rather than checking the wrong cells.
        if (message.info.resolution <= 0 or abs(yaw(message.info.origin.orientation)) > 1e-6
                or message.info.width*message.info.height != len(message.data)):
            return
        with self.lock:
            self.grid = (GridView.from_msg(message, allow_unknown=False), message.header, time.monotonic())

    @staticmethod
    def identity(handle):
        return bytes(handle.goal_id.uuid)

    def emit(self, identity, velocity=(0., 0.)):
        with self.lock:
            if self.active != identity:
                return
            if any(velocity) and (self.paused or time.monotonic()-self.last_good > .5):
                velocity = (0., 0.)
            message = Twist()
            message.linear.x, message.angular.z = map(float, velocity)
            self.cmd.publish(message)
            self.last_emit = time.monotonic()

    def accepted(self, handle):
        with self.lock:
            self.active = self.identity(handle)
            self.last_good = time.monotonic()
            self.cmd.publish(Twist())
        handle.execute()

    def cancel(self, handle):
        self.emit(self.identity(handle))
        return CancelResponse.ACCEPT

    def watchdog(self):
        with self.lock:
            if self.paused or (self.active is not None and (time.monotonic()-self.last_good > .5 or time.monotonic()-self.last_emit > .3)):
                self.cmd.publish(Twist())

    def observations(self, frame):
        transform = self.tf.lookup_transform(frame, self.frame, rclpy.time.Time())
        now = self.get_clock().now().nanoseconds*1e-9
        stamp = transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9
        if not -.1 <= now-stamp <= .5:
            raise ValueError('Car pose transform is stale.')
        p = transform.transform.translation
        pose = (p.x, p.y, yaw(transform.transform.rotation))
        with self.lock:
            odom, grid = self.odom, self.grid
        if odom is None or grid is None or time.monotonic()-odom[1] > 1. or time.monotonic()-grid[2] > 30.:
            raise ValueError('Measured odometry or local collision grid is missing/stale.')
        odom_stamp = odom[2].sec+odom[2].nanosec*1e-9
        grid_stamp = grid[1].stamp.sec+grid[1].stamp.nanosec*1e-9
        if not -.1 <= now-odom_stamp <= .5 or not -.1 <= now-grid_stamp <= 2.:
            raise ValueError('Odometry/collision grid simulation timestamp is stale.')
        if not all(map(math.isfinite, pose)):
            raise ValueError('Car pose is not finite.')
        grid_pose = pose
        if grid[1].frame_id != frame:
            conversion = self.tf.lookup_transform(grid[1].frame_id, frame, rclpy.time.Time()).transform
            angle = yaw(conversion.rotation)
            grid_pose = (conversion.translation.x+math.cos(angle)*pose[0]-math.sin(angle)*pose[1],
                conversion.translation.y+math.sin(angle)*pose[0]+math.cos(angle)*pose[1], pose[2]+angle)
        with self.lock:
            self.last_good = time.monotonic()
        return pose, odom[0], grid[0], grid_pose

    def finish(self, handle, outcome, reason):
        if not rclpy.ok():
            return FollowPath.Result()
        identity = self.identity(handle)
        self.emit(identity)
        with self.lock:
            if self.active == identity:
                self.active = None
        getattr(handle, outcome)()
        self.status.publish(String(data=json.dumps(dict(state=outcome, reason=reason,
            law=self.law, robot=self.profile['robot'], qualification='implemented bicycle candidate; physical validation pending'))))
        return FollowPath.Result()

    def execute(self, handle):
        path = handle.request.path
        identity = self.identity(handle)
        try:
            if not path.header.frame_id or len(path.poses) < 2:
                return self.finish(handle, 'abort', 'A finite framed forward car path is required.')
            points, headings = [], []
            for item in path.poses:
                p = item.pose.position
                q = item.pose.orientation
                if (not all(math.isfinite(value) for value in (q.x, q.y, q.z, q.w))
                        or not .99 <= q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w <= 1.01):
                    return self.finish(handle, 'abort', 'Path requires normalized body-heading quaternions.')
                heading = yaw(item.pose.orientation)
                if not all(map(math.isfinite, (p.x, p.y, heading))):
                    return self.finish(handle, 'abort', 'Non-finite path rejected.')
                points.append((p.x, p.y))
                headings.append(heading)
            for a, b, heading in zip(points, points[1:], headings):
                delta = np.asarray(b)-a
                if delta @ [math.cos(heading), math.sin(heading)] < -.01:
                    return self.finish(handle, 'abort', 'Reverse cusps need a reverse-aware adapter; use a forward Dubins car path.')
            tracker = BicycleTracker(self.law, self.profile['drive'], max_speed=self.speed)
            reference = tracker.reference(points)
            tolerance = float(self.get_parameter('xy_goal_tolerance').value)
            yaw_tolerance = float(self.get_parameter('yaw_goal_tolerance').value)
            started = time.monotonic()
            anchor, blocked, stale = None, None, None
            last_control_time = None
            while rclpy.ok() and time.monotonic()-started <= 600:
                cycle = time.monotonic()
                with self.lock:
                    current = self.active
                    paused = self.paused
                if current != identity:
                    # A preempted worker must never stop the new worker's car.
                    handle.abort()
                    return FollowPath.Result()
                if handle.is_cancel_requested:
                    return self.finish(handle, 'canceled', 'Operator canceled the car path.')
                if paused:
                    return self.finish(handle, 'abort', 'Manual Drive/Space/Stop interrupted car navigation.')
                try:
                    pose, measured, grid, grid_pose = self.observations(path.header.frame_id)
                    stale = None
                except (ValueError, tf2_ros.TransformException) as exc:
                    self.emit(identity)
                    stale = stale if stale is not None else cycle
                    if cycle-stale > 5:
                        return self.finish(handle, 'abort', str(exc))
                    time.sleep(self.period)
                    continue
                distance = math.dist(pose[:2], points[-1])
                if distance <= tolerance:
                    if abs(wrap(pose[2]-headings[-1])) <= yaw_tolerance:
                        return self.finish(handle, 'succeed', 'Car endpoint reached; body/contact mission acceptance is separate.')
                    return self.finish(handle, 'abort', 'Car reached XY with wrong heading; request a feasible approach, not an in-place turn.')
                sim_time = self.get_clock().now().nanoseconds*1e-9
                dt = .001 if last_control_time is None else sim_time-last_control_time
                if dt < .001:
                    self.emit(identity)
                    time.sleep(self.period)
                    continue
                last_control_time = sim_time
                if dt > .2:
                    self.emit(identity)
                    time.sleep(self.period)
                    continue
                command = tracker.command(pose, measured, reference, dt)
                unsafe = (swept_footprint_blocked(grid, self.footprint, grid_pose, command, tracker.acceleration)
                    or swept_footprint_blocked(grid, self.footprint, grid_pose, measured, tracker.acceleration))
                if unsafe:
                    self.emit(identity)
                    blocked = blocked if blocked is not None else cycle
                    if cycle-blocked > 5:
                        return self.finish(handle, 'abort', 'Full car stopping sweep is blocked; navigator must replan.')
                else:
                    blocked = None
                    self.emit(identity, command)
                if anchor is None or math.dist(pose[:2], anchor[1]) > .1:
                    anchor = (self.get_clock().now().nanoseconds*1e-9, pose[:2])
                elif self.get_clock().now().nanoseconds*1e-9-anchor[0] > 20:
                    return self.finish(handle, 'abort', 'Car failed to make measured progress.')
                feedback = FollowPath.Feedback()
                feedback.distance_to_goal, feedback.speed = float(distance), float(abs(measured[0]))
                handle.publish_feedback(feedback)
                self.status.publish(String(data=json.dumps(dict(law=self.law, state='tracking',
                    robot=self.profile['robot'], command=command, measured_velocity=measured,
                    virtual_steer_rad=tracker.delta, solver=tracker.solver_state,
                    compute_s=tracker.compute_s, deadline_misses=tracker.deadline_misses))))
                time.sleep(max(.001, self.period-(time.monotonic()-cycle)))
            return self.finish(handle, 'abort', 'Car path wall-clock deadline exceeded.')
        except Exception as exc:
            self.get_logger().error('Bicycle path failed: '+str(exc))
            return self.finish(handle, 'abort', str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = BicycleFollowPath()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.cmd.publish(Twist())
        executor.shutdown(timeout_sec=2.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
