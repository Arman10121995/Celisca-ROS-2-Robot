#!/usr/bin/env python3
"""R7.2 feature detection: replay recorded RGB-D frames through the visual odometry.

Each bag (``EXTRA_BAG_TOPICS`` with ``scripts/slam_mapping_check.sh``) holds
a robot driving the nav_obstacle loop: the RGB-D camera
(/oakd/rgb/image_raw, /oakd/depth/image_raw, /oakd/rgb/camera_info), the
static TF and simulator truth (/odom/ground_truth), and wheel odometry
(/robot_lab_controller/odom) when the controller publishes it.

Every detector of robot_lab_algorithms.visual_odometry runs over the same
frames (RGB and depth paired by stamp; ``--every N`` keeps every N-th pair
to emulate a slower camera).  The base trajectory starts at the true pose
and is scored against truth in the plane: position RMSE, final error,
drift (final error over the distance driven), heading RMSE, frames lost,
features, PnP inliers and time per frame.  Wheel odometry, started at the
same pose, is the baseline.

  python3 scripts/r7_2_feature_odometry_replay.py --out <dir> <run.bag> ...
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
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'robot_lab_algorithms'))

from robot_lab_algorithms.perception import decode_depth  # noqa: E402
from robot_lab_algorithms.visual_odometry import (  # noqa: E402
    DETECTORS, RgbdOdometry, base_motion, decode_gray, transform_matrix)

RGB, DEPTH, INFO = '/oakd/rgb/image_raw', '/oakd/depth/image_raw', '/oakd/rgb/camera_info'
TRUTH, WHEELS, TF_STATIC = '/odom/ground_truth', '/robot_lab_controller/odom', '/tf_static'


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def yaw_of(rotation):
    return math.atan2(rotation[1, 0], rotation[0, 0])


def messages(path, wanted):
    files = glob.glob(os.path.join(path, '*.db3')) if os.path.isdir(path) else [path]
    db = sqlite3.connect(files[0])
    topics = {tid: (name, get_message(kind)) for tid, name, kind in db.execute('select id, name, type from topics')}
    ids = [tid for tid, (name, _) in topics.items() if name in wanted]
    query = 'select topic_id, data from messages where topic_id in (%s) order by timestamp' % ','.join(
        str(i) for i in ids)
    for tid, raw in (db.execute(query) if ids else []):
        name, cls = topics[tid]
        yield name, deserialize_message(raw, cls)


def static_chain(transforms, parent, child):
    """parent <- child matrix composed from tf_static edges."""
    up = {t.child_frame_id: t for t in transforms}
    matrix, frame = np.eye(4), child
    while frame != parent:
        if frame not in up:
            raise KeyError('no static path from %s to %s (stuck at %s)' % (child, parent, frame))
        t = up[frame]
        p, q = t.transform.translation, t.transform.rotation
        matrix = transform_matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)) @ matrix
        frame = t.header.frame_id
    return matrix


def pose_matrix(pose):
    p, q = pose.position, pose.orientation
    return transform_matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


class Track:
    """Nearest-in-time lookup of (t, 4x4 pose)."""

    def __init__(self, rows):
        rows.sort(key=lambda r: r[0])
        self.times = [r[0] for r in rows]
        self.poses = [r[1] for r in rows]

    def at(self, t, gap=0.2):
        i = bisect.bisect_left(self.times, t)
        best = min((j for j in (i - 1, i) if 0 <= j < len(self.times)), key=lambda j: abs(self.times[j] - t),
                   default=None)
        if best is None or abs(self.times[best] - t) > gap:
            return None
        return self.poses[best]


def errors(estimates, truth):
    """Planar errors of (t, 4x4) estimates against the truth track."""
    rows = []
    for t, pose in estimates:
        true = truth.at(t)
        if true is not None:
            rows.append((math.hypot(pose[0, 3] - true[0, 3], pose[1, 3] - true[1, 3]),
                         abs(wrap(yaw_of(pose[:3, :3]) - yaw_of(true[:3, :3])))))
    e = np.array(rows)
    return {'position_rmse_m': round(float(np.sqrt(np.mean(e[:, 0] ** 2))), 3),
            'position_max_m': round(float(e[:, 0].max()), 3),
            'final_error_m': round(float(e[-1, 0]), 3),
            'heading_rmse_deg': round(math.degrees(float(np.sqrt(np.mean(e[:, 1] ** 2)))), 2)}


def replay(bag, base_frame, detectors, every, max_features):
    transforms, info, truth_rows, wheel_rows = [], None, [], []
    pending, frames = {}, 0
    runs, start, start_time = {}, None, None
    for name, msg in messages(bag, {RGB, DEPTH, INFO, TRUTH, WHEELS, TF_STATIC}):
        if name == TF_STATIC:
            transforms.extend(msg.transforms)
        elif name == INFO and info is None:
            info = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        elif name == TRUTH:
            truth_rows.append((stamp(msg.header), pose_matrix(msg.pose.pose)))
        elif name == WHEELS:
            wheel_rows.append((stamp(msg.header), pose_matrix(msg.pose.pose)))
        elif name in (RGB, DEPTH):
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            entry = pending.setdefault(key, {})
            entry[name] = msg
            if len(entry) < 2:
                continue
            pending.pop(key)
            for old in [k for k in pending if k < key]:
                pending.pop(old)
            frames += 1
            if info is None or (frames - 1) % every:
                continue
            t = stamp(msg.header)
            if start is None:
                first = Track(list(truth_rows)).at(t)
                if first is None:
                    continue
                camera = static_chain(transforms, base_frame, entry[DEPTH].header.frame_id)
                start, start_time = (first, camera), t
                runs = {d: {'odometry': RgbdOdometry(info, d, max_features), 'poses': [], 'ms': [],
                            'features': [], 'inliers': []} for d in detectors}
            gray, depth = decode_gray(entry[RGB]), decode_depth(entry[DEPTH])
            for run in runs.values():
                began = time.perf_counter()
                camera_pose, stats = run['odometry'].step(gray, depth)
                run['ms'].append((time.perf_counter() - began) * 1e3)
                run['poses'].append((t, start[0] @ base_motion(camera_pose, start[1])))
                run['features'].append(stats['features'])
                if stats['inliers']:
                    run['inliers'].append(stats['inliers'])
    truth = Track(truth_rows)
    times = [t for t, _ in next(iter(runs.values()))['poses']] if runs else []
    driven = 0.0
    previous = None
    for t in times:
        pose = truth.at(t)
        if pose is not None and previous is not None:
            driven += math.hypot(pose[0, 3] - previous[0, 3], pose[1, 3] - previous[1, 3])
        previous = pose if pose is not None else previous
    out = {'frames_paired': frames, 'frames_used': len(times), 'every': every,
           'seconds': round(times[-1] - times[0], 1) if times else 0, 'driven_m': round(driven, 2),
           'intrinsics': info, 'methods': {}}
    for name, run in runs.items():
        result = errors(run['poses'], truth)
        result.update({
            'drift_percent': round(100.0 * result['final_error_m'] / driven, 2) if driven else None,
            'frames_lost': run['odometry'].lost,
            'features_mean': round(float(np.mean(run['features'])), 1),
            'inliers_median': int(np.median(run['inliers'])) if run['inliers'] else 0,
            'ms_p50': round(float(np.median(run['ms'])), 1)})
        out['methods'][name] = result
    if wheel_rows and start is not None:
        # Wheel odometry can begin after the first camera frame (MuJoCo,
        # PyBullet): it starts at the true pose of its first sample.
        wheels = Track(wheel_rows)
        first = next((i for i, t in enumerate(wheels.times) if t >= start_time), None)
        anchor = truth.at(wheels.times[first]) if first is not None else None
        if anchor is not None:
            w0 = wheels.poses[first]
            estimates = [(t, anchor @ np.linalg.inv(w0) @ w) for t, w in zip(wheels.times[first:], wheels.poses[first:])
                         if t <= times[-1]]
            result = errors(estimates, truth)
            result['drift_percent'] = round(100.0 * result['final_error_m'] / driven, 2) if driven else None
            out['methods']['wheel odometry (baseline)'] = result
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bags', nargs='+')
    parser.add_argument('--out', required=True)
    parser.add_argument('--base', default='base_footprint')
    parser.add_argument('--detectors', default=','.join(DETECTORS))
    parser.add_argument('--every', type=int, default=1)
    parser.add_argument('--max-features', type=int, default=1000)
    args = parser.parse_args()
    detectors = [d for d in args.detectors.split(',') if d]
    runs = {}
    for bag in args.bags:
        name = os.path.basename(os.path.normpath(bag))
        runs[name] = replay(bag, args.base, detectors, args.every, args.max_features)
        print(name, json.dumps(runs[name]['methods']), flush=True)
    results = {
        'revision': subprocess.run(['git', '-C', HERE, 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip(),
        'host': '%s %s' % (platform.node(), platform.machine()), 'runs': runs}
    lines = ['# R7.2 feature-based RGB-D visual odometry on recorded simulator frames', '',
             'Revision `%s`. Each method starts at the true pose; errors are planar, against simulator truth.'
             % results['revision'][:7], '']
    for run, r in runs.items():
        lines += ['## %s' % run, '',
                  '%d of %d RGB-D pairs (every %d), %.0f s, %.1f m driven.' % (
                      r['frames_used'], r['frames_paired'], r['every'], r['seconds'], r['driven_m']), '',
                  '| Method | Position RMSE (m) | Final (m) | Drift (%) | Heading RMSE (deg) | Lost | Features | '
                  'Inliers | ms |', '|---|---|---|---|---|---|---|---|---|']
        for name, m in r['methods'].items():
            lines.append('| %s | %.3f | %.3f | %s | %.2f | %s | %s | %s | %s |' % (
                name, m['position_rmse_m'], m['final_error_m'], m['drift_percent'], m['heading_rmse_deg'],
                m.get('frames_lost', '-'), m.get('features_mean', '-'), m.get('inliers_median', '-'),
                m.get('ms_p50', '-')))
        lines.append('')
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'results.json'), 'w') as handle:
        json.dump(results, handle, indent=1)
    with open(os.path.join(args.out, 'report.md'), 'w') as handle:
        handle.write('\n'.join(lines))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
