#!/usr/bin/env python3
"""A navigation mission on a running nav launch: detour, cancel, resume, near-obstacle goal.

Uses the published /map to choose, from the localized start:

1. **Route goal A**: a free cell whose grid path is 4-12 m long and at
   least 1.25x the straight-line distance, so the robot must turn around
   obstacles (a multi-turn route).
2. **Cancellation**: A is sent, cancelled after the robot has travelled
   ``--cancel-after`` metres; the action must end CANCELED and the robot
   must come to rest (body speed below 0.05 m/s within 3 simulated s).
3. **Resume**: A is sent again and must succeed.
4. **Near-obstacle goal B**: a free cell close to an obstacle (clearance
   between robot radius + 0.05 and + 0.30 m), 2-8 m of path from A; it
   must succeed (a second goal in the same run).

Every success is measured against independent simulator truth
(/odom/ground_truth) after one simulated second of settling.  Prints one
JSON report and exits 0 only when every step passes.
"""
import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_nav_check import STATUS, Check  # noqa: E402


def grid_arrays(grid):
    info = grid.info
    data = np.asarray(grid.data, dtype=np.int16).reshape(info.height, info.width)
    return data, info.resolution, info.origin.position.x, info.origin.position.y


def clearance_map(data, resolution):
    """Distance (m) from each cell to the nearest occupied or unknown cell."""
    from scipy.ndimage import distance_transform_edt
    free = data == 0
    return distance_transform_edt(free) * resolution


def path_lengths(passable, start_cell, resolution):
    """8-connected grid path length (m) from start to every passable cell."""
    height, width = passable.shape
    distance = np.full(passable.shape, np.inf)
    sy, sx = start_cell
    if not passable[sy, sx]:
        return distance
    distance[sy, sx] = 0.0
    # Dijkstra with a bucket queue is overkill; BFS on 8-neighbours with
    # step costs 1/sqrt(2) visited in order of a simple deque is close
    # enough for choosing goals (lengths within a few percent).
    queue = deque([(sy, sx)])
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]
    while queue:
        y, x = queue.popleft()
        base = distance[y, x]
        for dy, dx, cost in steps:
            ny, nx = y+dy, x+dx
            if 0 <= ny < height and 0 <= nx < width and passable[ny, nx]:
                value = base+cost*resolution
                if value < distance[ny, nx]-1e-9:
                    distance[ny, nx] = value
                    queue.append((ny, nx))
    return distance


def choose_goals(grid, start, robot_radius, rng):
    data, resolution, ox, oy = grid_arrays(grid)
    clearance = clearance_map(data, resolution)
    passable = clearance > robot_radius+0.05
    sx, sy = int((start[0]-ox)/resolution), int((start[1]-oy)/resolution)
    # The localized start can sit in an inflated cell; walk to the nearest
    # passable cell (the robot is already there physically).
    if not (0 <= sy < data.shape[0] and 0 <= sx < data.shape[1]):
        return None, None, 'start outside the map'
    if not passable[sy, sx]:
        ys, xs = np.nonzero(passable)
        if not len(ys):
            return None, None, 'no passable cells'
        nearest = np.argmin((ys-sy)**2+(xs-sx)**2)
        sy, sx = int(ys[nearest]), int(xs[nearest])
    lengths = path_lengths(passable, (sy, sx), resolution)
    ys, xs = np.nonzero(np.isfinite(lengths))
    cx, cy = ox+(xs+0.5)*resolution, oy+(ys+0.5)*resolution
    straight = np.hypot(cx-start[0], cy-start[1])
    path = lengths[ys, xs]
    ratio = path/np.maximum(straight, 1e-6)
    roomy = clearance[ys, xs] > robot_radius+0.25
    candidates = np.flatnonzero((path >= 4.0) & (path <= 12.0) & (ratio >= 1.25) & roomy)
    detour = True
    if not len(candidates):
        candidates = np.flatnonzero((path >= 3.0) & (path <= 12.0) & roomy)
        detour = False
    if not len(candidates):
        return None, None, 'no reachable route goal 3-12 m away'
    pick = candidates[rng.integers(len(candidates))]
    goal_a = dict(x=float(cx[pick]), y=float(cy[pick]), path_m=float(path[pick]),
                  straight_m=float(straight[pick]), detour=detour)
    # Goal B: near an obstacle, reachable from A.
    ay, ax = int(ys[pick]), int(xs[pick])
    from_a = path_lengths(passable, (ay, ax), resolution)
    near = (clearance > robot_radius+0.05) & (clearance <= robot_radius+0.30) & passable
    bys, bxs = np.nonzero(near & (from_a >= 2.0) & (from_a <= 8.0))
    if not len(bys):
        return goal_a, None, 'no near-obstacle goal 2-8 m from A'
    k = rng.integers(len(bys))
    goal_b = dict(x=float(ox+(bxs[k]+0.5)*resolution), y=float(oy+(bys[k]+0.5)*resolution),
                  path_from_a_m=float(from_a[bys[k], bxs[k]]),
                  clearance_m=float(clearance[bys[k], bxs[k]]))
    return goal_a, goal_b, None


class Mission(Check):
    def truth_speed(self):
        if self.truth is None:
            return None
        twist = self.truth.twist.twist
        return math.hypot(twist.linear.x, twist.linear.y)

    def truth_stamp(self):
        stamp = self.truth.header.stamp
        return stamp.sec+stamp.nanosec*1e-9

    def travelled(self):
        return self.forward_distance_m+self.reverse_distance_m

    def send(self, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        future = self.action.send_goal_async(goal)
        self.spin_until(future.done, 30.0)
        handle = future.result() if future.done() else None
        return handle if handle is not None and handle.accepted else None

    def settle(self):
        if self.truth is None:
            return False
        start = self.truth_stamp()
        return self.spin_until(lambda: self.truth_stamp() >= start+1.0, 30.0)

    def measured(self, goal, yaw, base):
        report = {}
        report['settled_sim_second'] = self.settle()
        report.update(self.final_pose_report(base, (goal['x'], goal['y']), yaw))
        return report


def run_goal(node, goal, yaw, timeout, base):
    sent = time.monotonic()
    handle = node.send(goal['x'], goal['y'], yaw)
    if handle is None:
        return dict(outcome='rejected')
    result = handle.get_result_async()
    node.spin_until(result.done, timeout)
    status = result.result().status if result.done() else None
    report = dict(outcome=STATUS.get(status, 'timeout' if status is None else str(status)),
                  wall_s=round(time.monotonic()-sent, 1))
    report.update(node.measured(goal, yaw, base))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', default='base_footprint')
    parser.add_argument('--robot-radius', type=float, default=0.25)
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument('--cancel-after', type=float, default=1.0,
                        help='metres travelled toward A before cancelling')
    parser.add_argument('--max-position-error', type=float, default=0.30)
    parser.add_argument('--max-yaw-error-deg', type=float, default=15.0,
                        help='0 skips the heading check (position goals, e.g. cars)')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--drive-node', default='mujoco_spawner')
    parser.add_argument('--expected-steering-mode', choices=('ackermann', 'crab', 'in_phase', 'pivot'))
    args = parser.parse_args(argv)
    rclpy.init()
    node = Mission()
    rng = np.random.default_rng(args.seed)
    report = dict(steps={}, seed=args.seed)
    started = time.monotonic()
    for name, ready in (('map', lambda: node.map is not None),
                        ('localization_tf', lambda: node.pose(args.base) is not None),
                        ('action_server', lambda: node.action.server_is_ready()),
                        ('truth', lambda: node.truth is not None)):
        if not node.spin_until(ready, args.timeout):
            report['error'] = 'no %s after %.0f s' % (name, time.monotonic()-started)
            print(json.dumps(report))
            return 1
    report['stack_ready_wall_s'] = round(time.monotonic()-started, 1)
    if args.expected_steering_mode:
        from robot_lab_utils.qualification import inspect_drive_mode
        report['drive_configuration'] = inspect_drive_mode(node, args.drive_node, args.expected_steering_mode)
        if not report['drive_configuration']['passed']:
            report['error'] = 'Running steering mode differs from the requested trial'
            print(json.dumps(report))
            return 1
    node.spin_until(lambda: False, 5.0)
    start = node.pose(args.base)
    goal_a, goal_b, problem = choose_goals(node.map, start, args.robot_radius, rng)
    report.update(start=[round(v, 3) for v in start], goal_a=goal_a, goal_b=goal_b)
    if goal_a is None:
        report['error'] = problem
        print(json.dumps(report))
        return 1
    yaw_a = math.atan2(goal_a['y']-start[1], goal_a['x']-start[0])
    node.monitor_motion = True

    # 1-2: send A, cancel after some travel, require a stop.
    before = node.travelled()
    handle = node.send(goal_a['x'], goal_a['y'], yaw_a)
    cancel = dict(sent=handle is not None)
    if handle is not None:
        result = handle.get_result_async()
        moved = node.spin_until(lambda: node.travelled()-before >= args.cancel_after or result.done(),
                                args.timeout)
        cancel['travelled_before_cancel_m'] = round(node.travelled()-before, 3)
        cancel['finished_before_cancel'] = result.done()
        if moved and not result.done():
            future = handle.cancel_goal_async()
            node.spin_until(future.done, 30.0)
            node.spin_until(result.done, 60.0)
            status = result.result().status if result.done() else None
            cancel['outcome'] = STATUS.get(status, 'timeout' if status is None else str(status))
            stop_start = node.truth_stamp()
            stopped = node.spin_until(
                lambda: node.truth_speed() is not None and node.truth_speed() < 0.05
                and node.truth_stamp() >= stop_start, 60.0)
            cancel['stopped'] = stopped
            cancel['stop_sim_s'] = round(node.truth_stamp()-stop_start, 3)
            cancel['passed'] = (cancel['outcome'] == 'canceled' and stopped
                                and cancel['stop_sim_s'] <= 3.0)
        else:
            cancel['passed'] = False
            cancel['note'] = 'goal finished or robot did not move before the cancel point'
    else:
        cancel['passed'] = False
    report['steps']['cancel_a'] = cancel

    def accept(step, goal, yaw):
        passed = (step.get('outcome') == 'succeeded' and step.get('settled_sim_second')
                  and step.get('final_error_truth_m', math.inf) <= args.max_position_error)
        if args.max_yaw_error_deg > 0:
            passed = passed and abs(step.get('final_yaw_error_truth_deg', math.inf)) <= args.max_yaw_error_deg
        step['passed'] = bool(passed)
        return step

    # 3: resume A.
    report['steps']['resume_a'] = accept(run_goal(node, goal_a, yaw_a, args.timeout, args.base), goal_a, yaw_a)
    # 4: second goal near an obstacle.
    if goal_b is not None:
        here = node.pose(args.base) or (goal_a['x'], goal_a['y'])
        yaw_b = math.atan2(goal_b['y']-here[1], goal_b['x']-here[0])
        report['steps']['near_obstacle_b'] = accept(
            run_goal(node, goal_b, yaw_b, args.timeout, args.base), goal_b, yaw_b)
    else:
        report['steps']['near_obstacle_b'] = dict(passed=False, note=problem)
    report.update(node.motion_report())
    report['passed'] = all(step.get('passed') for step in report['steps'].values())
    print(json.dumps(report))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
