#!/usr/bin/env python3
"""R7.3 on simulator scans: replay recorded runs through the scan matchers.

Each bag (``planner_comparison.sh`` with ``EXTRA_BAG_TOPICS=/scan``) holds a
robot driving Nav2 goals with AMCL: /scan, /tf (the EKF's odom -> base),
/tf_static (base -> laser), /amcl_pose and simulator truth
(/odom/ground_truth).  Every localizer in robot_lab_algorithms.scan_matching
replays the same scans and odometry offline:

* it starts at the truth pose of the first scan (as the node does from
  /initialpose);
* each scan, it predicts with the odom -> base increment since the last
  one and aligns the scan to the map that Nav2 used.

AMCL, which ran live in the same run, is scored at the same scan times: its
last /amcl_pose moved forward by the odometry since that pose's stamp
(what map -> odom -> base gives a Nav2 server).  Dead reckoning is the
odometry alone.  Metrics: position and heading error against truth,
time "lost" (error > 0.5 m), compute per scan.

``--odom-bias`` corrupts the recorded increments the way the offline
benchmark does (distance +3%, rotation +2%, heading drift 0.5 deg/m and
seeded white noise), to test the matchers' corrections on real scans;
AMCL, which ran live on the true odometry, is then left out.

  python3 scripts/r7_3_live_scan_replay.py --map nav_maze --out <dir> <run.bag> ...
"""
import argparse
import bisect
import glob
import json
import math
import os
import platform
import sqlite3
import subprocess
import sys
import time

for _variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_variable, '1')

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot_lab_algorithms'))

from robot_lab_algorithms.scan_matching import (  # noqa: E402
    MATCHERS, OccupancyMap, ScanLocalizer, compose, relative, scan_points, wrap)

METHODS = [m for m in MATCHERS if m != 'dead_reckoning']


def read_map(name):
    folder = os.path.join(ROOT, 'src', 'robot_lab_maps', 'maps', name, 'maps')
    meta = yaml.safe_load(open(os.path.join(folder, 'map.yaml')))
    with open(os.path.join(folder, meta['image']), 'rb') as handle:
        assert handle.readline().strip() == b'P5'
        line = handle.readline()
        while line.startswith(b'#'):
            line = handle.readline()
        width, height = (int(v) for v in line.split())
        int(handle.readline())
        image = np.frombuffer(handle.read(), dtype=np.uint8).reshape(height, width)
    occupancy = image.astype(float) / 255.0 if meta.get('negate') else (255 - image.astype(float)) / 255.0
    grid = np.where(occupancy > meta['occupied_thresh'], 100, np.where(occupancy < meta['free_thresh'], 0, -1))
    grid = np.flipud(grid)  # image row 0 is the top (largest y)
    return OccupancyMap(grid.ravel(), width, height, meta['resolution'], meta['origin'])


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class Track:
    """Time-stamped planar poses with interpolation."""

    def __init__(self):
        self.t, self.p = [], []

    def add(self, t, pose):
        self.t.append(t)
        self.p.append(pose)

    def finish(self):
        order = np.argsort(self.t)
        self.t = [self.t[i] for i in order]
        self.p = [self.p[i] for i in order]

    def at(self, t, max_gap=0.2):
        i = bisect.bisect_left(self.t, t)
        if i == 0 or i >= len(self.t):
            return None
        t0, t1 = self.t[i - 1], self.t[i]
        if t1 - t0 > max_gap:
            return None
        a, b = self.p[i - 1], self.p[i]
        f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return np.array([a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), wrap(a[2] + f * wrap(b[2] - a[2]))])


def read_bag(path, base):
    files = glob.glob(os.path.join(path, '*.db3')) if os.path.isdir(path) else [path]
    db = sqlite3.connect(files[0])
    topics = {name: (tid, get_message(kind)) for tid, name, kind in
              db.execute('select id, name, type from topics')}
    by_id = {tid: (name, cls) for name, (tid, cls) in topics.items()}
    odom, truth, scans, amcl, static = Track(), Track(), [], [], {}
    for tid, data in db.execute('select topic_id, data from messages order by timestamp'):
        name, cls = by_id[tid]
        if name not in ('/tf', '/tf_static', '/scan', '/amcl_pose', '/odom/ground_truth'):
            continue
        msg = deserialize_message(data, cls)
        if name in ('/tf', '/tf_static'):
            for tf in msg.transforms:
                pose = (tf.transform.translation.x, tf.transform.translation.y, yaw_of(tf.transform.rotation))
                if name == '/tf_static':
                    static[tf.child_frame_id] = (tf.header.frame_id, pose)
                elif tf.header.frame_id == 'odom' and tf.child_frame_id == base:
                    odom.add(stamp_of(tf.header), pose)
        elif name == '/scan':
            scans.append(msg)
        elif name == '/amcl_pose':
            p = msg.pose.pose
            amcl.append((stamp_of(msg.header), np.array([p.position.x, p.position.y, yaw_of(p.orientation)])))
        else:
            p = msg.pose.pose
            truth.add(stamp_of(msg.header), (p.position.x, p.position.y, yaw_of(p.orientation)))
    odom.finish()
    truth.finish()
    amcl.sort(key=lambda row: row[0])
    return odom, truth, scans, amcl, static


def sensor_offset(static, frame, base):
    """Pose of ``frame`` in ``base`` from the static transforms."""
    pose = np.zeros(3)
    while frame != base:
        if frame not in static:
            raise SystemExit('no static transform chain from %s to %s' % (frame, base))
        parent, local = static[frame]
        pose = compose(np.array(local), pose)
        frame = parent
    return pose


def summary(offsets, heading, compute=None):
    offsets = np.array(offsets)
    e = np.hypot(offsets[:, 0], offsets[:, 1])
    out = {'scans': len(e), 'position_rmse_m': round(float(np.sqrt(np.mean(e ** 2))), 4),
           'mean_offset_m': [round(float(v), 4) for v in offsets.mean(axis=0)],
           'position_p95_m': round(float(np.percentile(e, 95)), 4), 'max_error_m': round(float(e.max()), 4),
           'heading_rmse_deg': round(math.degrees(float(np.sqrt(np.mean(np.square(heading))))), 3),
           'lost_fraction': round(float(np.mean(e > 0.5)), 4)}
    if compute:
        ms = np.array(compute) * 1e3
        out['compute_ms'] = {'p50': round(float(np.percentile(ms, 50)), 3), 'p99': round(float(np.percentile(ms, 99)), 3)}
    return out


def biased(delta, rng):
    """The offline benchmark's odometry error model applied to one increment."""
    dx, dy, dth = delta
    dist = math.hypot(dx, dy)
    return np.array([dx * 1.03 + rng.normal(0, 0.003), dy * 1.03 + rng.normal(0, 0.003),
                     dth * 1.02 + math.radians(0.5) * dist + rng.normal(0, 0.003)])


def replay(path, occupancy, base, rate, odom_bias=False, seed=1):
    odom, truth, scans, amcl, static = read_bag(path, base)
    if not scans:
        raise SystemExit('%s has no /scan' % path)
    offset = sensor_offset(static, scans[0].header.frame_id, base)
    # The usable scans: odometry and truth at the scan time, at most `rate` per second.
    usable, last = [], -1e9
    for msg in scans:
        t = stamp_of(msg.header)
        if t - last < 1.0 / rate:
            continue
        o, g = odom.at(t), truth.at(t)
        if o is None or g is None:
            continue
        usable.append((t, msg, o, g))
        last = t
    results = {}
    for method in ['dead_reckoning'] + METHODS:
        localizer = ScanLocalizer(method, occupancy, usable[0][3])
        previous = usable[0][2]
        rng = np.random.default_rng(seed)
        errors, heading, compute = [], [], []
        for t, msg, o, g in usable[1:]:
            points = scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                                 min(msg.range_max, 12.0), sensor_offset=tuple(offset))
            delta = relative(previous, o)
            if odom_bias:
                delta = biased(delta, rng)
            start = time.perf_counter()
            pose = localizer.step(delta, points)
            compute.append(time.perf_counter() - start)
            previous = o
            errors.append((pose[0] - g[0], pose[1] - g[1]))
            heading.append(abs(wrap(pose[2] - g[2])))
        results[method] = summary(errors, heading, compute)
        results[method]['rejected_matches'] = localizer.rejected
    if odom_bias:
        travelled = sum(math.hypot(b[3][0] - a[3][0], b[3][1] - a[3][1]) for a, b in zip(usable, usable[1:]))
        return {'bag': os.path.abspath(path), 'scans_used': len(usable), 'odom_bias': True,
                'travelled_m': round(travelled, 2)}, results
    # AMCL as it ran: the last /amcl_pose, moved forward by the odometry since.
    amcl_t = [row[0] for row in amcl]
    errors, heading = [], []
    for t, msg, o, g in usable[1:]:
        i = bisect.bisect_right(amcl_t, t) - 1
        if i < 0:
            continue
        ta, pose = amcl[i]
        oa = odom.at(ta)
        if oa is None:
            continue
        estimate = compose(pose, relative(oa, o))
        errors.append((estimate[0] - g[0], estimate[1] - g[1]))
        heading.append(abs(wrap(estimate[2] - g[2])))
    results['amcl_live'] = summary(errors, heading)
    travelled = sum(math.hypot(b[3][0] - a[3][0], b[3][1] - a[3][1]) for a, b in zip(usable, usable[1:]))
    meta = {'bag': os.path.abspath(path), 'scans_used': len(usable), 'scan_frame': scans[0].header.frame_id,
            'sensor_offset': [round(float(v), 4) for v in offset], 'beams': len(scans[0].ranges),
            'duration_s': round(usable[-1][0] - usable[0][0], 2), 'travelled_m': round(travelled, 2)}
    return meta, results


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bags', nargs='+')
    parser.add_argument('--map', default='nav_maze')
    parser.add_argument('--base', default='base_footprint')
    parser.add_argument('--rate', type=float, default=5.0, help='scans per second replayed')
    parser.add_argument('--label', default='')
    parser.add_argument('--odom-bias', action='store_true',
                        help="corrupt the recorded odometry with the offline benchmark's bias model")
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    occupancy = read_map(args.map)
    runs = {}
    for bag in args.bags:
        meta, results = replay(bag, occupancy, args.base, args.rate, args.odom_bias, args.seed)
        runs[os.path.basename(os.path.normpath(bag))] = {'meta': meta, 'results': results}
        print(os.path.basename(os.path.normpath(bag)), json.dumps({k: v['position_rmse_m'] for k, v in results.items()}))
    names = list(next(iter(runs.values()))['results'])
    mean = {}
    for name in names:
        rows = [run['results'][name] for run in runs.values()]
        mean[name] = {key: round(float(np.mean([r[key] for r in rows])), 4)
                      for key in ('position_rmse_m', 'position_p95_m', 'max_error_m', 'heading_rmse_deg',
                                  'lost_fraction')}
        if 'compute_ms' in rows[0]:
            mean[name]['compute_ms_p50'] = round(float(np.mean([r['compute_ms']['p50'] for r in rows])), 3)
            mean[name]['compute_ms_p99_max'] = round(float(max(r['compute_ms']['p99'] for r in rows)), 3)
    results = {
        'revision': subprocess.run(['git', '-C', HERE, 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip(),
        'host': '%s %s' % (platform.node(), platform.machine()), 'label': args.label, 'map': args.map,
        'rate_hz': args.rate, 'odom_bias': args.odom_bias, 'seed': args.seed, 'runs': runs, 'mean': mean,
    }
    lines = ['# R7.3 replay of simulator scans%s' % (' (%s)' % args.label if args.label else ''), '',
             'Revision `%s`; map %s; %d runs; %.0f scans/s replayed; localizers start at truth; '
             'odometry %s.' % (results['revision'][:7], args.map, len(runs), args.rate,
                               'with the benchmark bias model (seed %d)' % args.seed if args.odom_bias
                               else 'as recorded'), '',
             '| Localizer | Position RMSE (m) | p95 (m) | Max (m) | Heading RMSE (deg) | Lost (>0.5 m) | Compute p50 / max p99 (ms) |',
             '|---|---|---|---|---|---|---|']
    for name, r in mean.items():
        compute = '%.2f / %.2f' % (r['compute_ms_p50'], r['compute_ms_p99_max']) if 'compute_ms_p50' in r else '-'
        lines.append('| %s | %.3f | %.3f | %.3f | %.2f | %.1f%% | %s |' % (
            name, r['position_rmse_m'], r['position_p95_m'], r['max_error_m'], r['heading_rmse_deg'],
            100 * r['lost_fraction'], compute))
    report = '\n'.join(lines) + '\n'
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'results.json'), 'w') as handle:
        json.dump(results, handle, indent=1)
    with open(os.path.join(args.out, 'report.md'), 'w') as handle:
        handle.write(report)
    print(report)


if __name__ == '__main__':
    main()
