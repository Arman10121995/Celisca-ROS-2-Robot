#!/usr/bin/env python3
"""R7.4 replay: state estimators on the real AMCL fixes a simulator run recorded.

The offline benchmark feeds the estimators a modelled 10 Hz sensor with white
0.3 m noise.  Here the input is what AMCL actually published live: the
``/amcl_pose`` stream (``geometry_msgs/PoseWithCovarianceStamped``, frame
``map``) recorded with ``scripts/planner_comparison.sh`` during the R7.3
simulator runs.  Those fixes are low rate and bursty (~1.7 Hz, dt 0.2-7.4 s),
not 10 Hz, and they are the output of a scan matcher, so they are correlated
over time and carry the ~5-8 cm fixed map-registration offset that R7.3
measured - a real sensor, not white noise.

Each estimator (``robot_lab_algorithms.tracking_filters``) is replayed
offline over every run's fixes in time order:

- ``predict(dt)`` uses the wall-clock gap between consecutive fixes;
- ``update(z)`` takes the (x, y) of each fix;
- the estimate is scored against the simulator ground truth
  ``/odom/ground_truth`` interpolated to the fix's header stamp.

The raw AMCL fixes are the baseline; the first 10 s are skipped as warm-up.
This is the simulator-AMCL run the R7.4 benchmark listed as its next step.
It does not measure the live node timing in a running stack.

  python3 scripts/r7_4_live_replay.py --out <dir> <dir>/*.bag
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

sys.path.insert(0, '/opt/ros/humble/lib/python3.10/site-packages')  # rosbags need the ROS msgs
from rclpy.serialization import deserialize_message  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'robot_lab_algorithms'))

from robot_lab_algorithms.tracking_filters import ESTIMATORS, wrap  # noqa: E402

# Same defaults the state_estimation node uses (measurement_std 0.3, accel_std 0.5).
MEASUREMENT_STD = 0.3
ACCEL_STD = 0.5
WARMUP_S = 10.0  # skip the first fixes, as the offline benchmark skips its first 5 s


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw(orientation):
    sin = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cos = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(sin, cos)


def read_bag(path):
    """(fixes, truth) in time order: fixes = [(t, x, y)], truth = [(t, x, y, yaw)]."""
    files = glob.glob(os.path.join(path, '*.db3')) if os.path.isdir(path) else [path]
    db = sqlite3.connect(files[0])
    by_id = {tid: (name, kind) for tid, name, kind in db.execute('select id, name, type from topics')}
    fixes, truth = [], []
    for tid, data in db.execute('select topic_id, data from messages order by timestamp'):
        name, kind = by_id[tid]
        if name == '/amcl_pose' and kind == 'geometry_msgs/msg/PoseWithCovarianceStamped':
            m = deserialize_message(bytes(data), PoseWithCovarianceStamped)
            p = m.pose.pose
            fixes.append((stamp(m.header), p.position.x, p.position.y))
        elif name == '/odom/ground_truth' and kind == 'nav_msgs/msg/Odometry':
            m = deserialize_message(bytes(data), Odometry)
            p = m.pose.pose
            truth.append((stamp(m.header), p.position.x, p.position.y, yaw(p.orientation)))
    db.close()
    fixes.sort(key=lambda r: r[0])
    truth.sort(key=lambda r: r[0])
    return fixes, truth


def interpolate_truth(truth_t, truth_p, t):
    """Position and yaw of the ground truth at time ``t`` (linear, yaw wrapped)."""
    i = bisect.bisect_left(truth_t, t)
    if i <= 0 or i >= len(truth_t):
        return None
    t0, t1 = truth_t[i - 1], truth_t[i]
    a, b = truth_p[i - 1], truth_p[i]
    f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]),
            wrap(a[2] + f * wrap(b[2] - a[2])))


def run(name, fixes, truth_t, truth_p, start_index):
    """Replay one estimator over the fixes from ``start_index``; score vs truth."""
    flt = ESTIMATORS[name](measurement_std=MEASUREMENT_STD, accel_std=ACCEL_STD)
    pos_err, heading_err, gaps, compute = [], [], [], []
    previous_t = None
    for t, x, y in fixes[start_index:]:
        if previous_t is not None:
            flt.predict(t - previous_t)
            gaps.append(t - previous_t)
        start = time.perf_counter()
        flt.update((x, y))
        compute.append(time.perf_counter() - start)
        previous_t = t
        est = flt.estimate()
        if est is None:
            continue
        gt = interpolate_truth(truth_t, truth_p, t)
        if gt is None:
            continue
        pos_err.append(math.hypot(est[0] - gt[0], est[1] - gt[1]))
        heading_err.append(abs(wrap(est[2] - gt[2])))
    ms = np.array(compute) * 1e3 if compute else np.array([0.0])
    return {
        'fixes_scored': len(pos_err),
        'position_rmse_m': round(float(np.sqrt(np.mean(np.square(pos_err)))), 4) if pos_err else None,
        'position_p95_m': round(float(np.percentile(pos_err, 95)), 4) if pos_err else None,
        'max_error_m': round(float(np.max(pos_err)), 4) if pos_err else None,
        'mean_gap_s': round(float(np.mean(gaps)), 3) if gaps else None,
        'compute_ms': {'p50': round(float(np.percentile(ms, 50)), 4), 'p99': round(float(np.percentile(ms, 99)), 4)},
    }


def raw_fixes_summary(fixes, truth_t, truth_p, start_index):
    """Baseline: the raw AMCL (x, y) against truth at the same stamps."""
    errors, offsets = [], []
    for t, x, y in fixes[start_index:]:
        gt = interpolate_truth(truth_t, truth_p, t)
        if gt is None:
            continue
        errors.append(math.hypot(x - gt[0], y - gt[1]))
        offsets.append((x - gt[0], y - gt[1]))
    offsets = np.array(offsets)
    e = np.array(errors)
    return {
        'fixes_scored': len(e),
        'position_rmse_m': round(float(np.sqrt(np.mean(e ** 2))), 4),
        'position_p95_m': round(float(np.percentile(e, 95)), 4),
        'max_error_m': round(float(e.max()), 4),
        'mean_offset_m': [round(float(v), 4) for v in offsets.mean(axis=0)],
    }


def replay(path):
    fixes, truth = read_bag(path)
    if len(fixes) < 3 or len(truth) < 3:
        raise SystemExit('%s lacks /amcl_pose or /odom/ground_truth' % path)
    truth_t = [row[0] for row in truth]
    truth_p = [row[1:] for row in truth]
    t0 = fixes[0][0]
    start_index = next((i for i, row in enumerate(fixes) if row[0] - t0 >= WARMUP_S), len(fixes))
    gaps = [b[0] - a[0] for a, b in zip(fixes[start_index:], fixes[start_index + 1:])]
    meta = {
        'bag': os.path.abspath(path), 'amcl_fixes': len(fixes), 'fixes_scored': len(fixes) - start_index,
        'duration_s': round(fixes[-1][0] - t0, 1),
        'fix_rate_hz': round(len(fixes) / max(1e-9, fixes[-1][0] - t0), 2),
        'gap_min_s': round(min(gaps), 2) if gaps else None,
        'gap_median_s': round(float(np.median(gaps)), 2) if gaps else None,
        'gap_max_s': round(max(gaps), 2) if gaps else None,
        'warmup_s': WARMUP_S, 'measurement_std_m': MEASUREMENT_STD, 'accel_std': ACCEL_STD,
    }
    results = {'amcl_raw': raw_fixes_summary(fixes, truth_t, truth_p, start_index)}
    for name in ESTIMATORS:
        results[name] = run(name, fixes, truth_t, truth_p, start_index)
    return meta, results


def mean_dict(rows):
    out = {}
    for key, value in rows[0].items():
        if isinstance(value, dict):
            out[key] = {k: round(float(np.mean([r[key][k] for r in rows])), 4) for k in value}
        elif all(r[key] is not None for r in rows):
            out[key] = round(float(np.mean([r[key] for r in rows])), 4)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', required=True)
    parser.add_argument('bags', nargs='+')
    args = parser.parse_args()
    revision = subprocess.run(['git', '-C', HERE, 'rev-parse', 'HEAD'], capture_output=True,
                              text=True).stdout.strip()
    all_meta, all_results = {}, {}
    for path in args.bags:
        key = os.path.splitext(os.path.basename(path))[0]
        meta, results = replay(path)
        all_meta[key] = meta
        all_results[key] = results
    estimators = list(all_results.values())[0].keys()
    summary = {name: mean_dict([all_results[k][name] for k in all_results]) for name in estimators}
    results = {
        'revision': revision, 'host': '%s %s' % (platform.node(), platform.machine()),
        'python': platform.python_version(), 'numpy': np.__version__,
        'note': 'State estimators on the real AMCL fixes recorded in the R7.3 simulator runs.',
        'warmup_s': WARMUP_S, 'measurement_std_m': MEASUREMENT_STD, 'accel_std': ACCEL_STD,
        'runs': all_meta, 'summary': summary, 'per_run': all_results,
    }
    lines = ['# R7.4 live replay: state estimators on simulator AMCL fixes', '',
             'Revision `%s`, %s. Input: the `/amcl_pose` stream recorded in the R7.3 simulator '
             'runs, scored against `/odom/ground_truth`. First %.0f s skipped as warm-up.' % (
                 revision[:7], results['host'], WARMUP_S), '',
             '| Estimator | Position RMSE (m) | p95 (m) | max (m) | mean gap (s) | compute p50 (ms) |',
             '|---|---|---|---|---|---|']
    for name, r in summary.items():
        lines.append('| %s | %s | %s | %s | %s | %s |' % (
            name, r.get('position_rmse_m'), r.get('position_p95_m'), r.get('max_error_m'),
            r.get('mean_gap_s'), r.get('compute_ms', {}).get('p50')))
    report = '\n'.join(lines) + '\n'
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'results.json'), 'w') as handle:
        json.dump(results, handle, indent=1)
    with open(os.path.join(args.out, 'report.md'), 'w') as handle:
        handle.write(report)
    print(report)


if __name__ == '__main__':
    main()


