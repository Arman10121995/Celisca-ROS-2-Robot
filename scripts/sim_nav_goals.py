#!/usr/bin/env python3
"""Seeded multi-goal navigation trial with measured metrics (planner comparisons).

On a running nav launch, picks ``--goals`` reachable goals from the
published /map with a seeded generator (the same seed gives the same goals
on the same map, whatever planner runs), sends them one after another
through NavigateToPose, and measures each against independent simulator
truth (/odom/ground_truth):

* outcome (Nav2 terminal status), simulated duration;
* travelled path length and mean speed from the truth trace;
* minimum clearance of the truth trace to static map obstacles (distance
  transform of the map, minus the robot radius: <= 0 means the footprint
  overlapped a mapped obstacle);
* final position/heading error after one simulated second of settling.

Prints one JSON report; ``--trace-out`` saves the raw truth trace.
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
from sim_nav_check import STATUS  # noqa: E402
from sim_nav_mission import Mission, clearance_map, grid_arrays, path_lengths  # noqa: E402


def pick_goal(grid, start, rng, robot_radius, min_path, max_path):
    data, resolution, ox, oy = grid_arrays(grid)
    clearance = clearance_map(data, resolution)
    passable = clearance > robot_radius+0.05
    sx, sy = int((start[0]-ox)/resolution), int((start[1]-oy)/resolution)
    if not passable[sy, sx]:
        ys, xs = np.nonzero(passable)
        nearest = np.argmin((ys-sy)**2+(xs-sx)**2)
        sy, sx = int(ys[nearest]), int(xs[nearest])
    lengths = path_lengths(passable, (sy, sx), resolution)
    ys, xs = np.nonzero(np.isfinite(lengths) & (lengths >= min_path) & (lengths <= max_path)
                        & (clearance > robot_radius+0.25))
    if not len(ys):
        return None
    k = rng.integers(len(ys))
    return dict(x=float(ox+(xs[k]+0.5)*resolution), y=float(oy+(ys[k]+0.5)*resolution),
                grid_path_m=float(lengths[ys[k], xs[k]]))


class Trial(Mission):
    def path_metrics(self, samples, clearance, grid_meta, robot_radius):
        if len(samples) < 2:
            return {}
        xy = np.array([[s['x'], s['y']] for s in samples])
        length = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
        duration = samples[-1]['stamp_s']-samples[0]['stamp_s']
        resolution, ox, oy, height, width = grid_meta
        cols = np.clip(((xy[:, 0]-ox)/resolution).astype(int), 0, width-1)
        rows = np.clip(((xy[:, 1]-oy)/resolution).astype(int), 0, height-1)
        min_clear = float(np.min(clearance[rows, cols]))-robot_radius
        return dict(path_length_m=round(length, 3), duration_sim_s=round(duration, 3),
                    mean_speed_mps=round(length/duration, 3) if duration > 0 else None,
                    min_footprint_clearance_m=round(min_clear, 3))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', default='base_footprint')
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--goals', type=int, default=3)
    parser.add_argument('--min-path', type=float, default=3.0)
    parser.add_argument('--max-path', type=float, default=10.0)
    parser.add_argument('--robot-radius', type=float, default=0.2)
    parser.add_argument('--goal-timeout', type=float, default=300.0)
    parser.add_argument('--timeout', type=float, default=600.0)
    parser.add_argument('--trace-out')
    args = parser.parse_args(argv)
    rclpy.init()
    node = Trial()
    rng = np.random.default_rng(args.seed)
    report = dict(seed=args.seed, goals=[], robot_radius=args.robot_radius)
    started = time.monotonic()
    for name, ready in (('map', lambda: node.map is not None),
                        ('localization_tf', lambda: node.pose(args.base) is not None),
                        ('action_server', lambda: node.action.server_is_ready()),
                        ('truth', lambda: node.truth is not None)):
        if not node.spin_until(ready, args.timeout):
            report['error'] = 'no %s after %.0f s' % (name, time.monotonic()-started)
            print(json.dumps(report))
            return 1
    node.spin_until(lambda: False, 5.0)
    data, resolution, ox, oy = grid_arrays(node.map)
    clearance = clearance_map(data, resolution)
    grid_meta = (resolution, ox, oy, data.shape[0], data.shape[1])
    node.monitor_motion = True
    for index in range(args.goals):
        start = node.pose(args.base)
        goal = pick_goal(node.map, start, rng, args.robot_radius, args.min_path, args.max_path)
        if goal is None:
            report['goals'].append(dict(index=index, outcome='no_goal', start=start))
            break
        yaw = math.atan2(goal['y']-start[1], goal['x']-start[0])
        first_sample = len(node.trace)
        sent = time.monotonic()
        handle = node.send(goal['x'], goal['y'], yaw)
        entry = dict(index=index, start=[round(v, 3) for v in start], goal=goal)
        if handle is None:
            entry['outcome'] = 'rejected'
        else:
            result = handle.get_result_async()
            node.spin_until(result.done, args.goal_timeout)
            status = result.result().status if result.done() else None
            entry['outcome'] = STATUS.get(status, 'timeout' if status is None else str(status))
            if status is None:
                cancel = handle.cancel_goal_async()
                node.spin_until(cancel.done, 30.0)
            entry['wall_s'] = round(time.monotonic()-sent, 1)
            entry.update(node.measured(goal, yaw, args.base))
            entry.update(node.path_metrics(node.trace[first_sample:], clearance, grid_meta, args.robot_radius))
        report['goals'].append(entry)
    succeeded = [g for g in report['goals'] if g.get('outcome') == 'succeeded']
    report['success_rate'] = round(len(succeeded)/max(1, len(report['goals'])), 3)
    if args.trace_out:
        Path(args.trace_out).write_text(json.dumps(dict(samples=node.trace, commands=node.command_trace))+'\n')
        report['trace'] = args.trace_out
    print(json.dumps(report))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
