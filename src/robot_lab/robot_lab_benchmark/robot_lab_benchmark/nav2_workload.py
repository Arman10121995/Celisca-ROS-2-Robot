"""Run real Nav2 goals on an already launched isolated graph and measure truth.

Independent body truth must be in selected-world coordinates. Success here is
only the declared point-goal/settling contract; contacts and obstacle missions
remain unassessed. A simulator launch is owned by the concurrent parent.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

from .workload_contract import RESULT_SCHEMA, validate_workload


class Nav2Workload(Node):
    def __init__(self, specification, directory):
        super().__init__('robot_lab_nav2_workload', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.specification, self.directory = specification, directory
        self.trace = (directory/'mission-trace.jsonl').open('w', buffering=1)
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.stop_pub = self.create_publisher(Twist, '/key_vel', 1)
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom/ground_truth', self.truth, qos_profile_sensor_data)
        self.pose, self.velocity, self.previous, self.goal_handle = None, None, None, None
        self.sim_time, self.truth_stamp = None, None
        self.last_truth, self.last_clock, self.last_clock_record = -math.inf, -math.inf, -math.inf
        self.truth_count, self.clock_count, self.distance = 0, 0, 0.
        self.index, self.nav_status = 0, None
        self.state, self.reason = 'startup', ''
        self.interrupted, self.pending, self.complete = False, False, False
        self.started, self.phase_started, self.settled_since = time.monotonic(), time.monotonic(), None
        # bt_navigator's action server exists before the stack is active and
        # rejects goals until then: the first goal is retried every 2 s
        # within the startup deadline (2026-10-15, every queued workload
        # was rejected 1.5 s of sim time after readiness).
        self.retry_at, self.startup_rejections, self.ever_accepted = 0., 0, False
        self.measured_goals = []
        self.record(kind='workload_start', specification=specification,
            ros_domain_id=int(os.environ['ROS_DOMAIN_ID']), run_id=os.environ['ROBOT_LAB_EXPERIMENT_ID'])

    def record(self, **value):
        self.trace.write(json.dumps(dict(wall_monotonic=time.monotonic(), sim_time=self.sim_time,
            state=self.state, goal_index=self.index, **value), allow_nan=False)+'\n')

    def fail(self, reason):
        if self.state not in ('failed', 'measured_goals_reached'):
            self.state, self.reason, self.complete = 'failed', reason, True
            self.record(kind='failure', reason=reason)
            self.stop_pub.publish(Twist())
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()

    def clock(self, message):
        value = message.clock.sec+message.clock.nanosec*1e-9
        if self.sim_time is not None and value < self.sim_time-1e-6:
            self.fail('Simulation clock regressed; workload invalidated.')
        self.sim_time, self.last_clock = value, time.monotonic()
        self.clock_count += 1
        if self.last_clock-self.last_clock_record >= .2:
            self.last_clock_record = self.last_clock
            self.record(kind='clock', stamp=value)

    def truth(self, message):
        p, q, twist = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w, twist.linear.x, twist.linear.y, twist.angular.z]
        norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        if not all(map(math.isfinite, values)) or abs(norm-1.) > .01:
            self.fail('Independent body truth has invalid pose/quaternion/velocity.')
            return
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if self.truth_stamp is not None and stamp < self.truth_stamp-1e-6:
            self.fail('Independent truth time regressed; workload invalidated.')
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pose = [p.x, p.y, yaw]
        if self.previous:
            self.distance += math.dist(pose[:2], self.previous[:2])
        self.previous, self.pose = pose, pose
        self.velocity = [twist.linear.x, twist.linear.y, twist.angular.z]
        self.last_truth, self.truth_stamp = time.monotonic(), stamp
        self.truth_count += 1
        self.record(kind='truth', stamp=stamp, frame_id=message.header.frame_id,
            child_frame_id=message.child_frame_id, position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w],
            velocity=self.velocity, physical_pose=pose)

    def fresh(self):
        now = time.monotonic()
        return (self.pose is not None and self.sim_time is not None and self.truth_count >= 3
            and self.clock_count >= 2 and now-self.last_truth < 1. and now-self.last_clock < 1.)

    def send_goal(self):
        x, y, yaw = self.specification['goals'][self.index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.pending, self.nav_status, self.state, self.phase_started = True, None, 'navigating', time.monotonic()
        self.record(kind='goal_requested', requested=[x, y, yaw], physical_start=self.pose)
        self.client.send_goal_async(goal).add_done_callback(self.accepted)

    def accepted(self, future):
        self.pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                if not self.ever_accepted and time.monotonic()-self.started < 180:
                    self.startup_rejections += 1
                    self.state, self.phase_started = 'startup', self.started
                    self.retry_at = time.monotonic()+2.
                    self.record(kind='goal_rejected_before_nav2_active', rejections=self.startup_rejections)
                    return
                self.fail('Nav2 rejected the requested goal.')
                return
            self.ever_accepted = True
            self.goal_handle = handle
            self.record(kind='goal_accepted', goal_uuid=list(handle.goal_id.uuid))
            handle.get_result_async().add_done_callback(self.result)
            if self.complete or self.interrupted:
                handle.cancel_goal_async()
        except Exception as exc:
            self.fail('Nav2 request response failed: '+str(exc))

    def result(self, future):
        self.goal_handle = None
        try:
            self.nav_status = int(future.result().status)
            self.record(kind='nav2_result', nav2_status=self.nav_status)
            if self.complete:
                return
            if self.nav_status != GoalStatus.STATUS_SUCCEEDED:
                self.fail('Nav2 returned terminal status '+str(self.nav_status))
                return
            self.state, self.phase_started, self.settled_since = 'physical_settle', time.monotonic(), None
        except Exception as exc:
            self.fail('Nav2 result failed: '+str(exc))

    def tick(self):
        if self.complete:
            return
        now = time.monotonic()
        if self.interrupted:
            self.fail('Workload interrupted/canceled.')
            return
        if now-self.started > 3000 or now-self.phase_started > (180 if self.state == 'startup' else 600):
            self.fail('Workload/startup/goal wall-clock deadline exceeded.')
            return
        if self.state == 'startup':
            if self.fresh() and self.client.server_is_ready() and now >= self.retry_at and not self.pending:
                self.send_goal()
            return
        if not self.fresh():
            self.fail('Independent truth or simulation clock became stale.')
            return
        if self.state == 'physical_settle':
            requested = self.specification['goals'][self.index]
            error = math.dist(self.pose[:2], requested[:2])
            yaw_error = abs(math.atan2(math.sin(self.pose[2]-requested[2]), math.cos(self.pose[2]-requested[2])))
            stopped = math.hypot(*self.velocity[:2]) <= .025 and abs(self.velocity[2]) <= .05
            if not stopped:
                self.settled_since = None
                return
            if self.settled_since is None:
                self.settled_since = self.truth_stamp
            dwell = self.truth_stamp-self.settled_since
            if dwell < 1.:
                return
            measured = dict(requested=requested, physical_pose=list(self.pose), position_error_m=error,
                heading_error_rad=yaw_error, settled_sim_s=dwell, nav2_status=self.nav_status,
                truth_samples=self.truth_count, sim_time=self.sim_time)
            self.measured_goals.append(measured)
            self.record(kind='goal_measured', measurement=measured)
            if error > .15 or yaw_error > math.radians(5):
                self.fail('Physical endpoint exceeds 0.15 m / 5 degrees after Nav2 success.')
                return
            self.index += 1
            if self.index == len(self.specification['goals']):
                self.state, self.complete = 'measured_goals_reached', True
                self.record(kind='measured_goals_reached')
            else:
                self.send_goal()

    def finish(self, specification_path):
        if not self.complete:
            self.fail('Workload closed before completion.')
        deadline = time.monotonic()+5.
        while (self.pending or self.goal_handle is not None) and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
        self.trace.close()
        result = dict(schema=RESULT_SCHEMA, run_id=os.environ['ROBOT_LAB_EXPERIMENT_ID'],
            ros_domain_id=int(os.environ['ROS_DOMAIN_ID']), state=self.state, reason=self.reason,
            success=self.state == 'measured_goals_reached', goals=self.measured_goals,
            goal_contract=dict(position_m=.15, heading_rad=math.radians(5), settle_sim_s=1.),
            physical_path_length_m=self.distance, truth_samples=self.truth_count, clock_samples=self.clock_count,
            goal_rejections_before_nav2_active=self.startup_rejections,
            nav2_terminal=not self.pending and self.goal_handle is None, wall_duration_s=time.monotonic()-self.started,
            contacts=None, swept_footprint_clearance_m=None,
            scope='Actual Nav2 terminal statuses and selected-world independent body endpoints/settling only; full mission/contact qualification separate',
            seed_requested=os.environ.get('ROBOT_LAB_EXPERIMENT_SEED'), simulator_seed_applied=None,
            producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            workload_sha256=hashlib.sha256(specification_path.read_bytes()).hexdigest(),
            trace_sha256=hashlib.sha256((self.directory/'mission-trace.jsonl').read_bytes()).hexdigest())
        temporary = self.directory/'mission-result.new'
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        temporary.replace(self.directory/'mission-result.json')
        return result


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--spec', required=True)
    options = parser.parse_args(args)
    path = Path(options.spec).resolve()
    if path.stat().st_size > 64*1024:
        parser.error('Workload input exceeds 64 KB.')
    specification = validate_workload(json.loads(path.read_text()))
    output = os.environ.get('ROBOT_LAB_EXPERIMENT_OUTPUT')
    if not output or not os.environ.get('ROBOT_LAB_EXPERIMENT_ID') or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('Run this workload through the isolated concurrent runner.')
    directory = Path(output).resolve()
    if not str(directory).startswith('/workspace/') or directory.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Workload output must stay on the mounted SSD.')
    rclpy.init(args=[])
    node = None
    try:
        node = Nav2Workload(specification, directory)
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda _sig, _frame: setattr(node, 'interrupted', True))
        while rclpy.ok() and not node.complete:
            rclpy.spin_once(node, timeout_sec=.1)
            node.tick()
    except Exception as exc:
        if node:
            node.fail('Workload exception: '+str(exc))
        else:
            raise
    finally:
        if node:
            result = node.finish(path)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps(result), flush=True)
    return 0 if result['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
