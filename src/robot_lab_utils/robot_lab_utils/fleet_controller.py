#!/usr/bin/env python3
"""Actual shared-world instance/group command ownership and Nav2 dispatch.

Wheel feedback/AMCL provide estimates; Gazebo truth is reported separately.
Stop/loss closes the motor-command gate immediately and cancels owned Nav2
actions asynchronously. Late goal acknowledgements cannot reopen that gate.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.srv import SetEntityPose
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener

from robot_lab_utils.fleet_motion import Neighbor, angle, scan_guard, separation_guard, swarm_velocities
from robot_lab_utils.fleet_session import finite, ID_PATTERN, MEMBER_MODELS
from robot_lab_utils.fleet_missions import MissionHistory, coverage_plan, enqueue, mission, settled, traffic_yield_goal


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


@dataclass
class Member:
    config: dict
    publisher: object = None
    action: object = None
    initial_pub: object = None
    mode: str = 'idle'
    reason: str = 'Waiting for measured feedback'
    manual: tuple = (0., 0.)
    nav: tuple = (0., 0.)
    manual_at: float = -math.inf
    nav_at: float = -math.inf
    odom: object = None
    odom_at: float = -math.inf
    truth: object = None
    truth_at: float = -math.inf
    truth_velocity: tuple = None
    truth_yaw_rate: float = None
    scan: object = None
    scan_at: float = -math.inf
    estimate: tuple = None
    estimate_at: float = -math.inf
    localization_at: float = -math.inf
    generation: int = 0
    handle: object = None
    sending: bool = False
    cancel_pending: bool = False
    goal: dict = None
    task: str = ''
    deadline: float = 0.
    reset_at: float = 0.
    settled_at: float = 0.
    reset_future: object = None
    spawn_pose: object = None
    anchor: tuple = None
    reset_anchor_pending: tuple = None
    output: tuple = (0., 0.)
    included: bool = True
    queue: list = field(default_factory=list)
    active_item: dict = None
    queue_paused: bool = False
    arrival_at: float = 0.
    arrival_deadline: float = 0.
    lifecycle_client: object = None
    lifecycle_future: object = None
    lifecycle_active: bool = False
    lifecycle_at: float = -math.inf
    lifecycle_probe_at: float = -math.inf
    blocked_at: float = 0.
    recovery_checked_at: float = -math.inf
    sample_stamps: dict = field(default_factory=dict)
    message_counts: dict = field(default_factory=lambda: dict(odom=0, truth=0, scan=0, amcl=0))

    @property
    def name(self):
        return self.config['id']


class FleetController(Node):
    def __init__(self):
        super().__init__('fleet_controller')
        self.declare_parameter('session_file', '')
        self.session = json.loads(Path(self.get_parameter('session_file').value).read_text())
        self.session_id = Path(self.session['directory']).name
        self.members = {}
        self.operator = ''
        self.heartbeat_at = -math.inf
        self.clock_at = -math.inf
        self.last_clock = None
        self.swarm = None
        self.last_status = 0.
        self.history = MissionHistory()
        self.serial_traffic = True
        self._state_pub = self.create_publisher(String, '/fleet/state', 10)
        self._set_pose = self.create_client(SetEntityPose, '/world/' + self.session['world_name'] + '/set_pose')
        self._anchors = StaticTransformBroadcaster(self)
        self._tf = Buffer(node=self)
        self._listener = TransformListener(self._tf, self)
        self.last_rejection = ''
        self._journal = (Path(self.session['directory']) / 'events.jsonl').open('a', buffering=1)
        self._journal_size = 0
        for config in self.session['members']:
            name = config['id']
            if not ID_PATTERN.fullmatch(name) or name in self.members or config['model'] not in MEMBER_MODELS:
                raise ValueError('Invalid resolved fleet member')
            pose = config['spawn']
            member = Member(config=config, anchor=(float(pose['x']), float(pose['y']), float(pose['yaw'])))
            self.members[name] = member
            member.publisher = self.create_publisher(Twist, '/' + name + '/robot_lab_controller/cmd_vel_unstamped', 10)
            member.initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/' + name + '/initialpose', 10)
            member.action = ActionClient(self, NavigateToPose, '/' + name + '/navigate_to_pose')
            member.lifecycle_client = self.create_client(GetState, '/' + name + '/bt_navigator/get_state')
            def subscribe(kind, topic, callback, qos=10):
                self.create_subscription(kind, '/' + name + topic, lambda msg, m=member: callback(m, msg), qos)
            subscribe(Twist, '/key_vel', self._manual)
            subscribe(Twist, '/cmd_vel', self._nav)
            subscribe(Odometry, '/robot_lab_controller/odom', self._odom)
            subscribe(Odometry, '/odom/ground_truth', self._truth)
            subscribe(LaserScan, '/scan', self._scan, qos_profile_sensor_data)
            subscribe(PoseWithCovarianceStamped, '/amcl_pose', self._amcl)
            self.create_service(Trigger, '/' + name + '/reset', lambda _q, response, m=member: self._reset_service(m, response))
        self.create_subscription(String, '/fleet/command', self._command, 10)
        self.create_subscription(String, '/fleet/operator_heartbeat', self._heartbeat, 10)
        self.create_subscription(ClockMessage, '/clock', self._observe_clock, qos_profile_sensor_data)
        self.create_service(Trigger, '/fleet/stop', self._stop_service)
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self._tick, clock=self._steady)
        if not self.session['navigation']:
            self._publish_anchors()
        self._event('session_started', session_id=self.session_id, source=self.session)
        self.get_logger().info('One physical world; members=' + ', '.join(self.members) + '; session=' + self.session_id)

    def _event(self, operation, **data):
        # Bounded SSD journal; never create an unbounded prompt/telemetry log.
        row = json.dumps(dict(event=operation, wall=time.time(), monotonic=time.monotonic(),
                              sim_time=self.get_clock().now().nanoseconds * 1e-9, **data), allow_nan=False) + '\n'
        if self._journal_size + len(row) < 16 * 1024 * 1024:
            self._journal.write(row)
            self._journal_size += len(row)

    def _observe_clock(self, msg):
        stamp = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.last_clock is not None and stamp < self.last_clock:
            self._stop_all('World clock rewound: start a new session')
            for member in self.members.values():
                member.odom_at = member.truth_at = member.scan_at = member.estimate_at = member.localization_at = -math.inf
        self.clock_at, self.last_clock = time.monotonic(), stamp

    def _valid_pose(self, pose):
        values = [getattr(pose.position, key) for key in 'xyz'] + [getattr(pose.orientation, key) for key in 'xyzw']
        return all(math.isfinite(v) for v in values) and abs(sum(getattr(pose.orientation, key)**2 for key in 'xyzw') - 1) < .05

    def _sample_fresh(self, member, msg, kind):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_clock is None or not -1.5 <= stamp - self.last_clock <= .2:
            return False
        if stamp <= member.sample_stamps.get(kind, -math.inf):
            return False
        member.sample_stamps[kind] = stamp
        return True

    def _odom(self, member, msg):
        if (msg.header.frame_id != member.name + '/odom' or msg.child_frame_id != member.name + '/base_footprint'
                or not self._valid_pose(msg.pose.pose)
                or not all(math.isfinite(getattr(v, key)) for v in (msg.twist.twist.linear, msg.twist.twist.angular) for key in 'xyz')):
            member.odom_at = -math.inf
            return
        if not self._sample_fresh(member, msg, 'odom'):
            return
        member.odom, member.odom_at = msg, time.monotonic()
        member.message_counts['odom'] += 1

    def _truth(self, member, msg):
        if (msg.header.frame_id != 'map' or not self._valid_pose(msg.pose.pose)
                or not all(math.isfinite(getattr(vector, key)) for vector in (msg.twist.twist.linear, msg.twist.twist.angular) for key in 'xyz')):
            member.truth_at = -math.inf
            return
        if not self._sample_fresh(member, msg, 'truth'):
            return
        # Initial model-odometry diagnostics sometimes reported zero twist
        # while measured poses moved. Derive motion from two actual
        # stamped poses; never treat that zero field as a physical Stop.
        if member.truth is not None:
            previous = member.truth
            dt = (msg.header.stamp.sec - previous.header.stamp.sec
                  + (msg.header.stamp.nanosec - previous.header.stamp.nanosec) * 1e-9)
            if dt > 1e-9:
                member.truth_velocity = tuple((getattr(msg.pose.pose.position, k) - getattr(previous.pose.pose.position, k)) / dt for k in 'xyz')
                member.truth_yaw_rate = angle(yaw(msg.pose.pose.orientation) - yaw(previous.pose.pose.orientation)) / dt
        member.truth, member.truth_at = msg, time.monotonic()
        member.message_counts['truth'] += 1
        speed = math.sqrt(sum(v * v for v in member.truth_velocity)) if member.truth_velocity is not None else math.inf
        angular = member.truth_yaw_rate if member.truth_yaw_rate is not None else math.inf
        if member.spawn_pose is None and math.hypot(msg.pose.pose.position.x - member.config['spawn']['x'],
                                                  msg.pose.pose.position.y - member.config['spawn']['y']) < .2:
            if abs(speed) < .02 and abs(angular) < .04:
                member.settled_at = member.settled_at or time.monotonic()
                if time.monotonic() - member.settled_at > .3:
                    member.spawn_pose = copy.deepcopy(msg.pose.pose)
            else:
                member.settled_at = 0.

    def _scan(self, member, msg):
        if (msg.header.frame_id != member.name + '/laser_link' or not msg.ranges
                or not all(math.isfinite(v) for v in (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max))
                or msg.angle_increment <= 0 or msg.range_max <= msg.range_min):
            member.scan_at = -math.inf
            return
        if not self._sample_fresh(member, msg, 'scan'):
            return
        member.scan, member.scan_at = msg, time.monotonic()
        member.message_counts['scan'] += 1

    def _amcl(self, member, msg):
        if msg.header.frame_id != 'map' or not self._valid_pose(msg.pose.pose):
            member.estimate_at = -math.inf
            return
        p = msg.pose.pose
        member.estimate, member.estimate_at = (p.position.x, p.position.y, yaw(p.orientation)), time.monotonic()
        member.message_counts['amcl'] += 1

    def _bounded_twist(self, member, msg):
        values = [getattr(msg.linear, key) for key in 'xyz'] + [getattr(msg.angular, key) for key in 'xyz']
        if not all(math.isfinite(v) for v in values) or any(abs(v) > 1e-9 for v in (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)):
            return None
        return (max(-member.config['max_linear'], min(member.config['max_linear'], msg.linear.x)),
                max(-member.config['max_angular'], min(member.config['max_angular'], msg.angular.z)))

    def _manual(self, member, msg):
        if member.mode != 'manual':
            return  # Merely connecting/enabling a GUI cannot move a member.
        values = self._bounded_twist(member, msg)
        if values is None:
            self._stop_member(member, 'Invalid manual command')
        else:
            member.manual, member.manual_at = values, time.monotonic()

    def _nav(self, member, msg):
        if member.mode != 'navigation':
            return
        values = self._bounded_twist(member, msg)
        if values is None:
            self._stop_member(member, 'Invalid Nav2 command')
        else:
            member.nav, member.nav_at = values, time.monotonic()

    def _heartbeat(self, msg):
        try:
            data = json.loads(msg.data)
            operator = data['operator_id']
            if data['session_id'] != self.session_id or not isinstance(operator, str) or not 1 <= len(operator) <= 80:
                return
            now = time.monotonic()
            if self.operator and operator != self.operator and now - self.heartbeat_at <= .8:
                return
            if operator != self.operator:
                for member in self.members.values():
                    if member.mode in ('manual', 'swarm'):
                        self._stop_member(member, 'Operator ownership changed')
                self.swarm = None
            self.operator, self.heartbeat_at = operator, now
        except (ValueError, KeyError, TypeError):
            return

    def _fresh(self, member, now):
        return (now - self.clock_at < 1. and now - member.odom_at < 1.
                and now - member.truth_at < 1. and now - member.scan_at < 1.25
                and (not self.session['navigation'] or now - member.localization_at < .3))

    def _neighbor(self, member):
        if member.odom is not None:
            p = member.odom.pose.pose
            ax, ay, heading = member.anchor
            x = ax + math.cos(heading) * p.position.x - math.sin(heading) * p.position.y
            y = ay + math.sin(heading) * p.position.x + math.cos(heading) * p.position.y
            heading = angle(heading + yaw(p.orientation))
        else:
            return None
        speed = member.odom.twist.twist.linear.x if member.odom else 0.
        return Neighbor(member.name, x, y, heading, speed, member.config['radius'])

    def _command(self, msg):
        try:
            if len(msg.data) > 8192:
                raise ValueError('Fleet command exceeds 8 KiB')
            data = json.loads(msg.data)
            if data.get('session_id') != self.session_id:
                raise ValueError('Command belongs to a different/no shared-world session')
            op = data['op']
            if op == 'all_stop':
                self._stop_all('Operator all-stop')
                return
            if data.get('operator_id') != self.operator or time.monotonic() - self.heartbeat_at > .8:
                raise ValueError('Connect a live operator before commanding the fleet')
            ids = data.get('targets', [])
            if not isinstance(ids, list) or len(set(ids)) != len(ids) or any(name not in self.members for name in ids):
                raise ValueError('Select distinct session member ids')
            targets = [self.members[name] for name in ids]
            if op == 'swarm':
                self._start_swarm(data, targets)
            elif op in ('dispatch', 'allocate'):
                goal = self._goal(data)
                if op == 'allocate':
                    candidates = [m for m in self.members.values() if m.mode == 'idle' and m.included and self._fresh(m, time.monotonic())]
                    if not candidates:
                        raise ValueError('No idle member with fresh measured state')
                    targets = [min(candidates, key=lambda m: math.hypot(self._neighbor(m).x - goal['x'], self._neighbor(m).y - goal['y']))]
                if not targets:
                    raise ValueError('Select a member for dispatch')
                if len(targets) != 1:
                    raise ValueError('Dispatch one member; use coverage for a coordinated group mission')
                for member in targets:
                    if member.mode not in ('idle', 'navigation', 'dispatching', 'settling', 'waiting', 'paused'):
                        raise ValueError('Stop manual/swarm/reset ownership before queuing navigation')
                    item = mission(goal)
                    enqueue(member.queue, item)
                    if member.mode == 'idle' and not member.queue_paused:
                        member.mode = 'waiting'
                    self._event('goal_queued', member=member.name, task=item['task'], queued=len(member.queue))
            elif op == 'coverage':
                self._coverage(data, targets)
            elif op == 'route':
                if len(targets) != 1:
                    raise ValueError('A route must target exactly one member')
                route = data.get('goals')
                tag = data.get('request_id', '')
                if not isinstance(tag, str) or not 1 <= len(tag) <= 80:
                    raise ValueError('Route requires a bounded request identity')
                if not isinstance(route, list) or not 1 <= len(route) <= 64:
                    raise ValueError('A route requires 1–64 goals')
                items = [mission(self._goal(g), 'route:' + tag) for g in route]
                member = targets[0]
                if member.mode in ('manual', 'swarm', 'resetting'):
                    raise ValueError('Stop manual/group/reset ownership before accepting an external route')
                self._stop_member(member, 'Explicit external route replacement')
                member.queue.extend(items)
                member.mode = 'waiting'
                self._event('route_queued', member=member.name, request_id=tag, tasks=[i['task'] for i in items])
            elif op == 'traffic':
                if type(data.get('serial')) is not bool:
                    raise ValueError('Traffic mode requires a boolean serial value')
                if any(m.mode not in ('idle', 'waiting') or m.handle or m.sending for m in self.members.values()):
                    raise ValueError('Stop all active control before changing traffic mode')
                self.serial_traffic = data['serial']
            else:
                if not targets:
                    raise ValueError('Select at least one member')
                # Switching members out of a swarm stops that group as a unit.
                if self.swarm and any(m.name in self.swarm['targets'] for m in targets):
                    self._stop_swarm('Swarm membership/control changed')
                for member in targets:
                    if op == 'manual':
                        if member.handle is not None or member.sending or member.cancel_pending:
                            raise ValueError('Stop and wait for actual Nav2 cancellation before manual ownership')
                        self._stop_member(member, 'Manual control armed; neutral until input')
                        member.mode = 'manual'
                    elif op == 'stop':
                        self._stop_member(member, 'Operator stop')
                    elif op == 'pause':
                        goal = copy.deepcopy(member.goal)
                        self._stop_member(member, 'Paused by operator', clear_queue=False, preserve_active=True)
                        member.queue_paused = True
                        member.mode, member.goal = 'paused', goal
                    elif op == 'resume':
                        if member.mode != 'paused' or member.handle or member.sending or member.cancel_pending:
                            raise ValueError('Wait for the paused Nav2 action terminal result: ' + member.name)
                        if member.active_item is not None:
                            member.queue.insert(0, member.active_item)
                            member.active_item = None
                        member.goal = None
                        member.queue_paused, member.mode = False, 'waiting' if member.queue else 'idle'
                    elif op == 'clear_queue':
                        self._clear_queue(member, 'Operator cleared queued tasks')
                    elif op == 'queue_resume':
                        if member.mode not in ('idle', 'waiting'):
                            raise ValueError('Resume the active paused task or stop it first')
                        member.queue_paused = False
                        member.mode = 'waiting' if member.queue else 'idle'
                    elif op == 'reset':
                        self._request_reset(member)
                    elif op == 'leave':
                        self._stop_member(member, 'Left group; physical robot remains in the world')
                        member.included = False
                    elif op == 'join':
                        if not self._fresh(member, time.monotonic()):
                            raise ValueError('Member must have fresh measured feedback to rejoin')
                        member.included = True
                        self._stop_member(member, 'Rejoined; neutral until a new command')
                    else:
                        raise ValueError('Unknown fleet operation: ' + str(op))
            self._event('operator_command', command=data)
        except (ValueError, TypeError, KeyError) as exc:
            self.last_rejection = str(exc)
            self._event('command_rejected', reason=str(exc))
            self.get_logger().warning('Fleet command rejected: ' + str(exc))

    def _goal(self, data):
        if not self.session['navigation']:
            raise ValueError('Enable per-instance AMCL/Nav2 before Run')
        goal = {key: finite(data[key], 'goal ' + key) for key in ('x', 'y', 'yaw')}
        timeout = finite(data.get('timeout', 120.), 'goal timeout')
        if not 5 <= timeout <= 600:
            raise ValueError('Goal timeout must be between 5 and 600 wall seconds')
        goal['timeout'] = timeout
        not_before = finite(data.get('not_before', 0.), 'route departure time')
        if not 0 <= not_before <= (self.last_clock or 0.) + 600.:
            raise ValueError('Route departure must be within the next 600 simulation seconds')
        goal['not_before'] = not_before
        # A robot cannot be allocated to an unknown/occupied map cell.
        from robot_lab_utils.fleet_session import SpawnGrid
        if not hasattr(self, '_grid'):
            self._grid = SpawnGrid(self.session['map_yaml'])
        if not self._grid.clear(goal['x'], goal['y'], max(m.config['radius'] for m in self.members.values()) + .05):
            raise ValueError('Goal is not inside known free occupancy with body clearance')
        return goal

    def _dispatch(self, member, goal, item=None):
        if member.handle is not None or member.sending or member.cancel_pending:
            raise ValueError('Wait for the previous Nav2 action to finish/cancel: ' + member.name)
        if not self._fresh(member, time.monotonic()) or not self._nav_ready(member, time.monotonic()):
            raise ValueError('Fresh localization/sensors and actual Nav2 server required: ' + member.name)
        self._stop_member(member, 'Sending actual Nav2 goal', clear_queue=False)
        member.active_item = item or mission(goal)
        member.goal, member.task = copy.deepcopy(goal), member.active_item['task']
        generation = member.generation
        member.sending = True
        member.mode = 'dispatching'
        member.deadline = time.monotonic() + goal['timeout']
        request = NavigateToPose.Goal()
        request.pose.header.frame_id, request.pose.header.stamp = 'map', self.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = goal['x'], goal['y']
        request.pose.pose.orientation.z, request.pose.pose.orientation.w = math.sin(goal['yaw'] / 2), math.cos(goal['yaw'] / 2)
        try:
            future = member.action.send_goal_async(request)
        except Exception as exc:
            member.sending = False
            member.active_item, member.goal, member.task = None, None, ''
            member.mode = 'waiting'
            raise ValueError('Nav2 action submission failed: ' + str(exc)) from exc
        future.add_done_callback(lambda done, m=member, g=generation: self._goal_sent(m, g, done))
        self._event('nav2_goal_sent', member=member.name, task=member.task, goal=goal, generation=generation)

    def _goal_sent(self, member, generation, future):
        member.sending = False
        try:
            handle = future.result()
        except Exception as exc:
            if generation == member.generation:
                self._finish_task(member, 'failed', 'Nav2 send failed: ' + str(exc))
            return
        if not handle.accepted:
            if generation == member.generation:
                self._finish_task(member, 'failed', 'Nav2 rejected the goal')
            return
        member.handle = handle
        handle.get_result_async().add_done_callback(lambda done, m=member, g=generation, h=handle: self._goal_finished(m, g, h, done))
        if generation != member.generation or member.mode != 'dispatching':
            self._cancel_handle(member)
        else:
            member.mode, member.reason = 'navigation', 'Actual Nav2 goal accepted'
            member.nav_at = -math.inf

    def _goal_finished(self, member, generation, handle, future):
        if member.handle is handle:
            member.handle, member.cancel_pending = None, False
        try:
            status = int(future.result().status)
        except Exception:
            status = GoalStatus.STATUS_UNKNOWN
        self._event('nav2_result', member=member.name, generation=generation, status=status, task=member.task,
                    truth=self._truth_row(member))
        if generation == member.generation:
            if status == GoalStatus.STATUS_SUCCEEDED:
                member.mode, member.reason = 'settling', 'Nav2 succeeded; waiting for measured arrival and physical stop'
                member.arrival_at, member.arrival_deadline = 0., time.monotonic() + 12.
                member.nav, member.output = (0., 0.), (0., 0.)
                member.publisher.publish(Twist())
            else:
                self._finish_task(member, 'failed', 'Nav2 ended status=' + str(status))

    def _cancel_handle(self, member):
        if member.handle is None or member.cancel_pending:
            return
        member.cancel_pending = True
        try:
            member.handle.cancel_goal_async().add_done_callback(lambda done, m=member: self._cancel_ack(m, done))
        except Exception as exc:
            member.reason = 'Motor gate stopped; Nav2 cancellation failed: ' + str(exc)

    def _cancel_ack(self, member, future):
        # A cancel response is not a terminal result. Keep the handle until
        # get_result returns; replacement/manual commands must wait for it.
        try:
            accepted = bool(future.result().goals_canceling)
        except Exception:
            accepted = False
        self._event('nav2_cancel_ack', member=member.name, accepted=accepted)

    def _clear_queue(self, member, reason):
        for item in member.queue:
            self._event('task_finished', member=member.name,
                        **self.history.finish(item, 'canceled', reason))
        member.queue.clear()

    def _finish_task(self, member, state, reason):
        if member.active_item is not None:
            self._event('task_finished', member=member.name,
                        **self.history.finish(member.active_item, state, reason, self._truth_row(member)))
            member.active_item = None
        self._stop_member(member, reason, clear_queue=False)
        member.queue_paused = state != 'succeeded'
        if member.queue and not member.queue_paused:
            member.mode = 'waiting'

    def _stop_member(self, member, reason, clear_queue=True, preserve_active=False):
        if clear_queue:
            self._clear_queue(member, reason)
            member.queue_paused = False
        if member.active_item is not None and not preserve_active:
            self._event('task_finished', member=member.name,
                        **self.history.finish(member.active_item, 'canceled', reason, self._truth_row(member)))
            member.active_item = None
        member.generation += 1
        member.mode, member.reason = 'idle', reason
        member.manual = member.nav = (0., 0.)
        member.manual_at = member.nav_at = -math.inf
        member.goal = None
        if not preserve_active:
            member.task = ''
        member.output = (0., 0.)
        member.publisher.publish(Twist())
        self._cancel_handle(member)

    def _coverage(self, data, targets):
        if not self.session['navigation']:
            raise ValueError('Coverage requires per-instance AMCL/Nav2')
        if not targets or any(m.mode != 'idle' or m.queue or m.active_item or m.handle or m.sending
                              or not m.included or not self._fresh(m, time.monotonic()) for m in targets):
            raise ValueError('Coverage requires fresh included idle members with empty queues')
        from robot_lab_utils.fleet_session import SpawnGrid
        grid = SpawnGrid(self.session['map_yaml'])
        members = [dict(id=m.name, x=self._neighbor(m).x, y=self._neighbor(m).y,
                        radius=m.config['radius']) for m in targets]
        plan = coverage_plan(grid, members, (data['x'], data['y']), data['width'], data['height'], data['spacing'])
        # Validate the complete plan before changing any member's ownership.
        checked = {name: [mission(self._goal(goal), 'coverage') for goal in goals] for name, goals in plan.items()}
        for member in targets:
            member.queue.extend(checked[member.name])
            member.queue_paused = False
            member.mode = 'waiting' if member.queue else 'idle'
        self._event('coverage_planned', targets=plan, scope='Clearance-aware sweep waypoints; actual completion is reported per task')

    def _schedule(self, now, fresh):
        if not all(fresh.values()):
            return  # Every physical peer must be observable before dispatch.
        busy = any(m.mode in ('navigation', 'dispatching', 'settling', 'manual', 'swarm', 'resetting')
                   or m.handle or m.sending or m.cancel_pending for m in self.members.values())
        if self.serial_traffic and busy:
            return
        candidates = [m for m in self.members.values() if m.mode in ('idle', 'waiting') and m.included
                      and m.queue and not m.queue_paused and fresh[m.name]
                      and self.last_clock is not None and self.last_clock >= m.queue[0]['goal'].get('not_before', 0.)
                      and not m.handle and not m.sending and not m.cancel_pending and self._nav_ready(m, now)]
        candidates.sort(key=lambda m: (m.queue[0]['admitted'], m.name))
        for member in candidates[:1] if self.serial_traffic else candidates:
            item = member.queue[0]
            waiting_for = item.get('traffic_wait_for')
            if waiting_for:
                peer = self.members.get(waiting_for['member'])
                peer_items = ([peer.active_item] if peer and peer.active_item else []) + (peer.queue if peer else [])
                if any(p['task'] == waiting_for['task'] for p in peer_items):
                    if now - waiting_for['since'] > item['goal']['timeout']:
                        member.queue.pop(0)
                        self._event('task_finished', member=member.name,
                            **self.history.finish(item, 'failed', 'Traffic wait budget expired; queue held for operator'))
                        member.queue_paused = True
                    member.reason = 'Yielded; waiting for priority task on '+waiting_for['member']
                    continue
            try:
                self._dispatch(member, item['goal'], item)
                member.queue.pop(0)  # Only remove after successful action submission.
            except (ValueError, RuntimeError) as exc:
                member.reason = 'Queued task waiting: ' + str(exc)

    def _traffic_recovery_tick(self, now, fresh):
        """Bounded measured-peer recovery; retain the interrupted task identity."""
        for member in sorted(self.members.values(), key=lambda m: (
                (m.active_item or {}).get('admitted', 0.), m.name), reverse=True):
            blocked = (member.mode == 'navigation' and member.active_item and fresh[member.name]
                       and member.reason.startswith(('predicted member separation', 'member footprint proximity',
                                                     'scan stopping corridor', 'scan footprint proximity')))
            if not blocked:
                member.blocked_at = 0.
                continue
            member.blocked_at = member.blocked_at or now
            if now-member.blocked_at < 2. or now-member.recovery_checked_at < 1.:
                continue
            member.recovery_checked_at = now
            robot = self._neighbor(member)
            peers = [(other, self._neighbor(other)) for other in self.members.values()
                     if other is not member and fresh[other.name]]
            close = [(m, p) for m, p in peers if math.hypot(robot.x-p.x, robot.y-p.y) < 1.2]
            if not close:
                continue  # A wall is not a fleet deadlock.
            peer, _ = min(close, key=lambda item: math.hypot(robot.x-item[1].x, robot.y-item[1].y))
            original = member.active_item
            # Older work wins ties. Never interrupt a peer's existing yield.
            if peer.active_item and (peer.active_item.get('source') == 'traffic_yield' or
                    (peer.active_item['admitted'], peer.name) > (original['admitted'], member.name)):
                continue
            if original.get('traffic_retries', 0) >= 3:
                self._finish_task(member, 'failed', 'Traffic recovery limit reached; remaining queue held')
                continue
            try:
                if not hasattr(self, '_grid'):
                    from robot_lab_utils.fleet_session import SpawnGrid
                    self._grid = SpawnGrid(self.session['map_yaml'])
                target = traffic_yield_goal(self._grid, robot, [p for _, p in peers])
                if len(member.queue) > 62:
                    raise ValueError('No queue capacity for a bounded traffic recovery')
            except ValueError as exc:
                if now-member.blocked_at > 12.:
                    self._finish_task(member, 'failed', str(exc))
                continue
            member.active_item = None  # It is suspended, not canceled or succeeded.
            self._stop_member(member, 'Yielding to measured blocking peer', clear_queue=False)
            original['traffic_retries'] = original.get('traffic_retries', 0)+1
            if peer.active_item:
                original['traffic_wait_for'] = dict(member=peer.name, task=peer.active_item['task'], since=now)
            member.queue[:0] = [mission(target, 'traffic_yield'), original]
            member.queue_paused, member.mode = False, 'waiting'
            member.blocked_at = 0.
            self._event('traffic_yield_planned', member=member.name, peer=peer.name,
                        suspended_task=original['task'], target=target, attempt=original['traffic_retries'])
            break

    def _nav_ready(self, member, now):
        return member.lifecycle_active and now - member.lifecycle_at < 2. and member.action.server_is_ready()

    def _probe_navigation(self, member, now):
        future = member.lifecycle_future
        if future is not None:
            if future.done():
                try:
                    member.lifecycle_active = future.result().current_state.id == 3
                except Exception:
                    member.lifecycle_active = False
                member.lifecycle_at = now
                member.lifecycle_future = None
            elif now - member.lifecycle_probe_at > 2.:
                member.lifecycle_active = False
                future.cancel()
                member.lifecycle_future = None
        if member.lifecycle_future is None and now - member.lifecycle_probe_at > .5 and member.lifecycle_client.service_is_ready():
            member.lifecycle_probe_at = now
            member.lifecycle_future = member.lifecycle_client.call_async(GetState.Request())

    def _arrival_tick(self, member, now, fresh):
        truth = self._truth_row(member)
        observed = ((truth['x'], truth['y'], truth['yaw']) if truth else None)
        if fresh and truth and settled(member.goal, observed, truth['linear'], truth['angular']):
            member.arrival_at = member.arrival_at or now
            if now - member.arrival_at >= .4:
                self._finish_task(member, 'succeeded', 'Actual Nav2 success and measured settled arrival')
                return
        else:
            member.arrival_at = 0.
        if now > member.arrival_deadline:
            self._finish_task(member, 'failed', 'Nav2 returned success but measured arrival/settling did not pass')

    def _stop_swarm(self, reason):
        if self.swarm:
            for name in self.swarm['targets']:
                self._stop_member(self.members[name], reason)
            self._event('swarm_stopped', reason=reason)
        self.swarm = None

    def _stop_all(self, reason):
        self.swarm = None
        for member in self.members.values():
            self._stop_member(member, reason)
        self._event('all_stop', reason=reason)

    def _stop_service(self, _request, response):
        self._stop_all('Fleet stop service')
        response.success, response.message = True, 'Motor gates latched neutral; owned Nav2 cancellation requested'
        return response

    def _start_swarm(self, data, targets):
        if len(targets) < 2 or any(not m.included or not self._fresh(m, time.monotonic()) for m in targets):
            raise ValueError('Swarm requires at least two included members with fresh measured state')
        if any(m.handle is not None or m.sending or m.cancel_pending for m in targets):
            raise ValueError('Cancel Nav2 tasks and wait for terminal results before swarm control')
        mode, spacing = data['controller'], finite(data.get('spacing', 1.), 'swarm spacing')
        heading = finite(data.get('heading', 0.), 'swarm heading')
        neighbors = [self._neighbor(m) for m in targets]
        anchor = (finite(data['x'], 'anchor x'), finite(data['y'], 'anchor y')) if 'x' in data or 'y' in data else None
        swarm_velocities(mode, neighbors, .25, .6, spacing, anchor, heading)  # Validate before ownership changes.
        self._stop_swarm('New swarm controller')
        for member in targets:
            self._stop_member(member, 'Ground ' + mode + ' controller active')
            member.mode = 'swarm'
        self.swarm = dict(controller=mode, targets=[m.name for m in targets], spacing=spacing, anchor=anchor, heading=heading)

    def _request_reset(self, member):
        if member.spawn_pose is None:
            raise ValueError('No measured settled spawn pose recorded for ' + member.name)
        self._stop_member(member, 'Scoped reset: waiting for physical stop and Nav2 cancellation')
        member.mode, member.reset_at, member.settled_at = 'resetting', time.monotonic(), 0.
        member.reset_future = None

    def _reset_service(self, member, response):
        try:
            self._request_reset(member)
            response.success, response.message = True, 'Scoped reset accepted; observe /fleet/state for completion'
        except ValueError as exc:
            response.success, response.message = False, str(exc)
        return response

    def _publish_anchors(self):
        transforms = []
        for member in self.members.values():
            transform = TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', member.name + '/odom'
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.transform.translation.x, transform.transform.translation.y = member.anchor[:2]
            transform.transform.rotation.z = math.sin(member.anchor[2] / 2)
            transform.transform.rotation.w = math.cos(member.anchor[2] / 2)
            transforms.append(transform)
        self._anchors.sendTransform(transforms)

    def _reset_tick(self, member, now):
        if now - member.reset_at > 12.:
            self._stop_member(member, 'Scoped reset timed out; world clock preserved')
            return
        if member.handle is not None or member.sending or member.cancel_pending or now - member.truth_at > 1. or member.odom is None:
            return
        if member.reset_future is not None:
            if not member.reset_future.done():
                return
            try:
                accepted = member.reset_future.result().success
            except Exception:
                accepted = False
            member.reset_future = None
            if not accepted:
                self._stop_member(member, 'Gazebo rejected the scoped entity pose reset')
                return
            # Encoders keep their continuous history. Re-anchor that history
            # instead of resetting world time or another robot's controllers.
            p = member.odom.pose.pose
            heading = yaw(member.spawn_pose.orientation) - yaw(p.orientation)
            member.anchor = (member.spawn_pose.position.x - math.cos(heading) * p.position.x + math.sin(heading) * p.position.y,
                             member.spawn_pose.position.y - math.sin(heading) * p.position.x - math.cos(heading) * p.position.y, heading)
            if self.session['navigation']:
                member.reset_anchor_pending = member.anchor
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id, pose.header.stamp = 'map', self.get_clock().now().to_msg()
                pose.pose.pose = copy.deepcopy(member.spawn_pose)
                pose.pose.covariance[0] = pose.pose.covariance[7] = .01
                pose.pose.covariance[35] = .01
                member.initial_pub.publish(pose)
                member.estimate_at = member.localization_at = -math.inf
            else:
                self._publish_anchors()
            self._stop_member(member, 'Scoped entity pose reset acknowledged; commands neutral')
            # A deliberate pose reset is discontinuous; the next two samples
            # establish physical velocity rather than differentiating a teleport.
            member.truth, member.truth_at = None, -math.inf
            member.truth_velocity = member.truth_yaw_rate = None
            self._event('entity_reset_acknowledged', member=member.name, pose=member.config['spawn'])
            return
        p = member.spawn_pose.position
        for other in self.members.values():
            if other is member:
                continue
            if now - other.truth_at > 1. or math.hypot(other.truth.pose.pose.position.x - p.x, other.truth.pose.pose.position.y - p.y) < member.config['radius'] + other.config['radius'] + .15:
                member.reason = 'Scoped reset destination occupied or peer state stale'
                return
        if (member.truth_velocity is None or member.truth_yaw_rate is None
                or math.sqrt(sum(v * v for v in member.truth_velocity)) > .02 or abs(member.truth_yaw_rate) > .04):
            member.settled_at = 0.
            return
        member.settled_at = member.settled_at or now
        if now - member.settled_at < .3 or not self._set_pose.service_is_ready():
            return
        request = SetEntityPose.Request()
        request.entity.name, request.entity.type = member.name, request.entity.MODEL
        request.pose = copy.deepcopy(member.spawn_pose)
        member.reset_future = self._set_pose.call_async(request)
        member.reason = 'Setting this entity pose; peers and world clock unchanged'

    def _truth_row(self, member):
        if member.truth is None:
            return None
        pose, twist = member.truth.pose.pose, member.truth.twist.twist
        heading = yaw(pose.orientation)
        linear = (member.truth_velocity[0] * math.cos(heading) + member.truth_velocity[1] * math.sin(heading)) if member.truth_velocity is not None else None
        return dict(x=pose.position.x, y=pose.position.y, z=pose.position.z, yaw=yaw(pose.orientation),
                    linear=linear, angular=member.truth_yaw_rate,
                    velocity_source='finite difference of measured stamped Gazebo poses',
                    source_reported_twist=dict(linear=twist.linear.x, angular=twist.angular.z))

    def _tick(self):
        now = time.monotonic()
        if self.session['navigation']:
            for member in self.members.values():
                self._probe_navigation(member, now)
        if self.session['navigation'] and self.last_clock is not None:
            for member in self.members.values():
                try:
                    transform = self._tf.lookup_transform('map', member.name + '/odom', rclpy.time.Time())
                    stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                    if abs(stamp - self.last_clock) < 1.5:
                        t = transform.transform
                        candidate = (t.translation.x, t.translation.y, yaw(t.rotation))
                        expected = member.reset_anchor_pending
                        if expected is None or (math.hypot(candidate[0] - expected[0], candidate[1] - expected[1]) < .15
                                                and abs(angle(candidate[2] - expected[2])) < .15):
                            member.anchor = candidate
                            member.localization_at = now
                            member.reset_anchor_pending = None
                except TransformException:
                    pass
        if now - self.heartbeat_at > .8:
            for member in self.members.values():
                if member.mode in ('manual', 'swarm', 'navigation', 'dispatching', 'settling', 'waiting', 'paused') or member.queue:
                    self._stop_member(member, 'Operator heartbeat lost; explicit new ownership required')
            self.swarm = None
        fresh = {m.name: self._fresh(m, now) for m in self.members.values()}
        if not all(fresh.values()) and any(m.mode in ('manual','navigation','dispatching','settling','swarm')
                                           for m in self.members.values()):
            missing=', '.join(name for name,ready in fresh.items() if not ready)
            self._stop_all('Physical peer feedback lost: '+missing+'; new commands required after rejoin')
        if now - self.heartbeat_at <= .8:
            self._schedule(now, fresh)
        neighbors = [self._neighbor(m) for m in self.members.values() if fresh[m.name]]
        if self.session['navigation'] and now-self.heartbeat_at <= .8:
            self._traffic_recovery_tick(now, fresh)
        swarm_commands = {}
        if self.swarm:
            ids = self.swarm['targets']
            if any(not fresh[name] for name in ids):
                self._stop_swarm('Member feedback/scan lost')
            else:
                swarm_commands = swarm_velocities(self.swarm['controller'], [n for n in neighbors if n.name in ids],
                                                  .25, .6, self.swarm['spacing'], self.swarm['anchor'], self.swarm['heading'])
        for member in self.members.values():
            linear = angular = 0.
            if member.mode == 'resetting':
                self._reset_tick(member, now)
            elif member.mode == 'settling':
                self._arrival_tick(member, now, fresh[member.name])
            elif member.mode in ('navigation', 'dispatching') and now > member.deadline:
                self._finish_task(member, 'failed', 'Nav2 wall-time goal budget expired')
            elif member.mode in ('manual', 'navigation', 'swarm'):
                if not fresh[member.name]:
                    self._stop_member(member, 'Measured clock/odometry/truth/scan/localization stale')
                else:
                    if member.mode == 'manual' and now - member.manual_at < .35:
                        linear, angular = member.manual
                    elif member.mode == 'navigation' and now - member.nav_at < .35:
                        linear, angular = member.nav
                    elif member.mode == 'swarm':
                        linear, angular = swarm_commands.get(member.name, (0., 0.))
                    robot = self._neighbor(member)
                    linear, angular, reason = separation_guard(robot, neighbors, linear, angular)
                    if not reason:
                        scan = member.scan
                        linear, angular, reason = scan_guard(linear, angular, scan.ranges, scan.angle_min,
                                                             scan.angle_increment, member.config['radius'])
                    member.reason = reason or ('Neutral; waiting for input' if linear == angular == 0 and member.mode == 'manual'
                                               else member.mode + ' command active')
            message = Twist()
            message.linear.x, message.angular.z = float(linear), float(angular)
            member.publisher.publish(message)
            member.output = (linear, angular)
        if now - self.last_status >= .2:
            self.last_status = now
            rows = []
            for member in self.members.values():
                estimate = self._neighbor(member)
                def age(stamp):
                    return round(now - stamp, 3) if math.isfinite(stamp) else None
                rows.append(dict(id=member.name, model=member.config['model'], mode=member.mode, reason=member.reason,
                    included=member.included, ready=fresh[member.name], limits=dict(linear=member.config['max_linear'], angular=member.config['max_angular']),
                    navigation_ready=self.session['navigation'] and self._nav_ready(member, now),
                    estimate=dict(x=estimate.x, y=estimate.y, yaw=estimate.yaw) if estimate else None,
                    estimate_source=('AMCL transform + wheel feedback' if math.isfinite(member.localization_at) else 'waiting for AMCL; wheel feedback + spawn anchor')
                        if self.session['navigation'] else 'wheel feedback + spawn/reset anchor',
                    truth=self._truth_row(member), command=dict(linear=member.output[0], angular=member.output[1]),
                    wheel_twist=dict(linear=member.odom.twist.twist.linear.x, angular=member.odom.twist.twist.angular.z) if member.odom else None,
                    ages=dict(odom=age(member.odom_at), truth=age(member.truth_at), scan=age(member.scan_at),
                              amcl=age(member.estimate_at), localization_tf=age(member.localization_at)),
                    message_counts=member.message_counts, task=member.task, goal=member.goal,
                    nav2_pending=member.sending or member.handle is not None, cancel_pending=member.cancel_pending,
                    queued=len(member.queue),
                    queued_tasks=[dict(task=item['task'], goal=item['goal'], source=item['source']) for item in member.queue],
                    queue_paused=member.queue_paused,
                    settled_spawn_recorded=member.spawn_pose is not None, battery=None, charging=None))
            data = dict(schema_version=1, session_id=self.session_id, map=self.session['map'], backend='gazebo',
                        navigation=self.session['navigation'], directory=self.session['directory'],
                        sim_time=self.last_clock, clock_age=round(now - self.clock_at, 3) if math.isfinite(self.clock_at) else None,
                        operator_id=self.operator, operator_fresh=now - self.heartbeat_at < .8, swarm=self.swarm, members=rows,
                        last_rejection=self.last_rejection,
                        traffic=dict(serial=self.serial_traffic, policy='One autonomous route at a time; queued work waits for measured settling' if self.serial_traffic else 'Concurrent routes with local scan/separation guards'),
                        task_history=list(self.history.rows),
                        scope='Experimental shared-world runtime; contact and mission qualification pending')
            self._state_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    def close(self):
        self._stop_all('Fleet controller shutdown')
        self._journal.close()


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent()
    rclpy.init(args=args)
    node = FleetController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
