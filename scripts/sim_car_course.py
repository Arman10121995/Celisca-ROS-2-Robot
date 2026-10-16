#!/usr/bin/env python3
"""Car course mission on a running nav launch: cone slalom, then parking (R5.5).

The course comes from robot_lab_maps/maps/car_course/course.json
(gen_car_course.py).  With the robot localized at the nav_empty spawn:

1. **Slalom**: each gate goal in turn (alternately 0.5 m either side of the
   cone line, heading +x).  A gate passes on Nav2 success with the truth
   position within ``--max-position-error``.
2. **Parking**: the bay goal, heading +y (nose in between two parked boxes
   0.8 m apart).  It passes on success within ``--parking-position-error``
   and ``--parking-yaw-error-deg`` of the goal, measured against truth.

Throughout, the robot's footprint polygon (its Nav2 overlay footprint,
placed at the truth pose) is checked against the cones and boxes: the
**swept-footprint clearance** must stay above zero.  The driven
**curvature** (|yaw rate| / speed while moving) is reported against the
robot's turning radius.  Prints one JSON report; exits 0 only when every
step passes.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_nav_mission import Mission, run_goal  # noqa: E402


def footprint_of(robot):
    """The robot's global-costmap footprint polygon from its Nav2 overlay."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory('robot_lab_navigation')) / 'config' / 'robots' / (robot + '.yaml')
    params = yaml.safe_load(path.read_text())['global_costmap']['global_costmap']['ros__parameters']
    return np.array(yaml.safe_load(params['footprint']), dtype=float)


def boundary(polygon, step=0.01):
    points = []
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        n = max(2, int(np.ceil(np.linalg.norm(b - a) / step)))
        points.append(a + (b - a) * np.linspace(0, 1, n, endpoint=False)[:, None])
    return np.vstack(points)


def inside(points, polygon):
    """Even-odd test of points against a convex or concave polygon."""
    x, y = points[:, 0], points[:, 1]
    result = np.zeros(len(points), dtype=bool)
    for (x1, y1), (x2, y2) in zip(polygon, np.roll(polygon, -1, axis=0)):
        crosses = ((y1 > y) != (y2 > y)) & (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1)
        result ^= crosses
    return result


class Course:
    def __init__(self, course, footprint):
        self.footprint = footprint
        self.cones = [(c['x'], c['y'], c['radius']) for c in course['cones']]
        # Axis-aligned boxes: (name, centre x, y, half extents x, y).
        self.boxes = [(box['name'], box['x'], box['y'], box['size'][0] / 2, box['size'][1] / 2)
                      for box in course['boxes']]

    def clearance(self, x, y, yaw):
        """(metres, obstacle) from the footprint at this pose to the nearest
        cone or box; negative when they overlap."""
        c, s = math.cos(yaw), math.sin(yaw)
        polygon = self.footprint @ np.array([[c, s], [-s, c]]) + (x, y)
        edge = boundary(polygon)
        best = (math.inf, None)
        for index, (cx, cy, radius) in enumerate(self.cones):
            distance = np.hypot(edge[:, 0] - cx, edge[:, 1] - cy).min() - radius
            if inside(np.array([[cx, cy]]), polygon)[0]:
                distance = -radius
            best = min(best, (distance, 'cone_%d' % index))
        for name, bx, by, hx, hy in self.boxes:
            dx = np.maximum(np.abs(edge[:, 0] - bx) - hx, 0.0)
            dy = np.maximum(np.abs(edge[:, 1] - by) - hy, 0.0)
            gap = float(np.hypot(dx, dy).min())
            corners = np.array([[bx - hx, by - hy], [bx + hx, by - hy], [bx + hx, by + hy], [bx - hx, by + hy]])
            if gap == 0.0 or inside(corners, polygon).any():
                gap = -0.001   # touching or overlapping
            best = min(best, (gap, name))
        return best


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', required=True)
    parser.add_argument('--course', default='')
    parser.add_argument('--base', default='base_footprint')
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument('--max-position-error', type=float, default=0.30)
    parser.add_argument('--parking-position-error', type=float, default=0.15)
    parser.add_argument('--parking-yaw-error-deg', type=float, default=10.0)
    parser.add_argument('--turning-radius', type=float, default=0.5)
    parser.add_argument('--trace-out')
    args = parser.parse_args(argv)
    if not args.course:
        from ament_index_python.packages import get_package_share_directory
        args.course = str(Path(get_package_share_directory('robot_lab_maps')) / 'maps' / 'car_course' / 'course.json')
    course = json.loads(Path(args.course).read_text())
    geometry = Course(course, footprint_of(args.robot))

    rclpy.init()
    node = Mission()
    report = dict(steps={}, course=args.course, footprint=geometry.footprint.tolist())
    started = time.monotonic()
    for name, ready in (('map', lambda: node.map is not None),
                        ('localization_tf', lambda: node.pose(args.base) is not None),
                        ('action_server', lambda: node.action.server_is_ready()),
                        ('truth', lambda: node.truth is not None)):
        if not node.spin_until(ready, args.timeout):
            report['error'] = 'no %s after %.0f s' % (name, time.monotonic() - started)
            print(json.dumps(report))
            return 1
    report['stack_ready_wall_s'] = round(time.monotonic() - started, 1)
    node.spin_until(lambda: False, 5.0)
    node.monitor_motion = True

    watch = dict(clearance=math.inf, obstacle=None, curvature=0.0, samples=0)
    original = node.on_truth

    def on_truth(message):
        original(message)
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        watch['samples'] += 1
        if watch['samples'] % 5 == 0:
            gap, obstacle = geometry.clearance(p.x, p.y, yaw)
            if gap < watch['clearance']:
                watch['clearance'], watch['obstacle'] = gap, obstacle
        twist = message.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        if speed > 0.08:
            watch['curvature'] = max(watch['curvature'], abs(twist.angular.z) / speed)
    node.on_truth = on_truth
    # Rebind the subscription callback to the wrapper.
    for subscription in node.subscriptions:
        if subscription.topic_name == '/odom/ground_truth':
            subscription.callback = on_truth

    def section(name, goals, position_error, yaw_error_deg=0.0):
        watch.update(clearance=math.inf, obstacle=None, curvature=0.0)
        steps = []
        for goal in goals:
            step = run_goal(node, goal, goal['yaw'], args.timeout, args.base)
            passed = (step.get('outcome') == 'succeeded' and step.get('settled_sim_second')
                      and step.get('final_error_truth_m', math.inf) <= position_error)
            if yaw_error_deg > 0:
                passed = passed and abs(step.get('final_yaw_error_truth_deg', math.inf)) <= yaw_error_deg
            step['goal'] = goal
            step['passed'] = bool(passed)
            steps.append(step)
            if not passed:
                break
        result = dict(goals=steps, min_footprint_clearance_m=round(watch['clearance'], 3),
                      nearest_obstacle=watch['obstacle'],
                      max_curvature_per_m=round(watch['curvature'], 3),
                      curvature_limit_per_m=round(1.0 / args.turning_radius, 3))
        result['passed'] = (len(steps) == len(goals) and all(s['passed'] for s in steps)
                            and watch['clearance'] > 0.0)
        report['steps'][name] = result

    section('slalom', course['slalom_goals'], args.max_position_error)
    if report['steps']['slalom']['passed']:
        section('parking', [course['parking_goal']], args.parking_position_error, args.parking_yaw_error_deg)
    else:
        report['steps']['parking'] = dict(passed=False, note='not attempted after the slalom failed')
    report.update(node.motion_report())
    report['passed'] = all(step.get('passed') for step in report['steps'].values())
    if args.trace_out:
        Path(args.trace_out).write_text(json.dumps(dict(samples=node.trace, commands=node.command_trace)) + '\n')
        report['trace'] = args.trace_out
    print(json.dumps(report))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
