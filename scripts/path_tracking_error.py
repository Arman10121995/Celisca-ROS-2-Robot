#!/usr/bin/env python3
"""Cross-track error of the robot (Gazebo truth) from the latest global plan.

Reads each ``<planner>_seed<N>.bag`` of a planner_comparison.sh directory:
for every truth sample while a plan is active, the distance from the robot
to the most recent ``/plan`` polyline published before it.  Samples within
0.3 m of the plan's end are skipped (arrival, not tracking).  Prints JSON
per planner: RMS, 95th percentile and maximum cross-track error.

  python3 scripts/path_tracking_error.py <comparison dir> [--markdown out.md]
"""
import argparse
import bisect
import collections
import glob
import json
import math
import os
import sqlite3

import numpy as np


def _messages(db, topic, msg_type):
    from rclpy.serialization import deserialize_message
    connection = sqlite3.connect(db)
    ids = {name: i for i, name in connection.execute('select id, name from topics')}
    if topic not in ids:
        return []
    rows = connection.execute('select timestamp, data from messages where topic_id=? order by timestamp',
                              (ids[topic],)).fetchall()
    return [deserialize_message(data, msg_type) for _, data in rows]


def _stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _distance_to_polyline(point, xy):
    a, b = xy[:-1], xy[1:]
    ab = b - a
    length_sq = np.maximum((ab * ab).sum(axis=1), 1e-12)
    t = np.clip(((point - a) * ab).sum(axis=1) / length_sq, 0.0, 1.0)
    closest = a + ab * t[:, None]
    return float(np.min(np.hypot(*(closest - point).T)))


def trial_errors(bag_dir):
    from nav_msgs.msg import Odometry, Path
    dbs = glob.glob(os.path.join(bag_dir, '*.db3'))
    if not dbs:
        return []
    plans = [(_stamp(m.header), np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses]))
             for m in _messages(dbs[0], '/plan', Path) if len(m.poses) >= 2]
    truth = [(_stamp(m.header), m.pose.pose.position.x, m.pose.pose.position.y)
             for m in _messages(dbs[0], '/odom/ground_truth', Odometry)]
    if not plans or not truth:
        return []
    plan_times = [p[0] for p in plans]
    errors = []
    for t, x, y in truth[::5]:
        i = bisect.bisect_right(plan_times, t) - 1
        if i < 0 or t - plan_times[i] > 2.0:  # no plan, or the last one is stale (between goals)
            continue
        xy = plans[i][1]
        point = np.array([x, y])
        if math.hypot(*(xy[-1] - point)) < 0.3:
            continue
        errors.append(_distance_to_polyline(point, xy))
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('directory')
    parser.add_argument('--markdown')
    args = parser.parse_args()
    per = collections.OrderedDict()
    for bag in sorted(glob.glob(os.path.join(args.directory, '*_seed*.bag'))):
        planner = os.path.basename(bag).rsplit('_seed', 1)[0]
        per.setdefault(planner, []).extend(trial_errors(bag))
    result = {}
    for planner, errors in per.items():
        e = np.array(errors) if errors else np.zeros(1)
        result[planner] = {'samples': len(errors), 'rms_m': round(float(np.sqrt(np.mean(e ** 2))), 4),
                           'p95_m': round(float(np.percentile(e, 95)), 4), 'max_m': round(float(e.max()), 4)}
    print(json.dumps(result, indent=1))
    if args.markdown:
        lines = ['| Planner | Samples | Cross-track RMS (m) | p95 (m) | Max (m) |', '|---|---|---|---|---|']
        for planner, r in result.items():
            lines.append('| %s | %d | %.4f | %.4f | %.4f |' % (planner, r['samples'], r['rms_m'], r['p95_m'], r['max_m']))
        open(args.markdown, 'w').write('\n'.join(lines) + '\n')


if __name__ == '__main__':
    main()
