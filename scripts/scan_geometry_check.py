#!/usr/bin/env python3
"""Does a recorded 2D scan agree with the map and the robot's true pose?

For every /scan in a rosbag2 (sqlite) with /odom/ground_truth and
/tf_static, the scan is placed in the world with the true base pose and
the static base -> laser transform, and each return is compared with the
nearest occupied cell of the reference map.  Reported:

- short returns (< ``--self-range``) per beam index: a fixed set of short
  beams is the robot seeing itself;
- the median signed angular offset that best aligns each scan with the map
  (a search over +-3 deg, mean distance clipped at 0.3 m): a non-zero value is a laser mounting yaw that
  the TF does not carry;
- the lag of the odometry TF (odom -> base frame) behind truth, from the
  cross-correlation of their yaw rates: slam places each scan with that
  TF at the scan's stamp, so a lag rotates scans taken while turning.

  python3 scripts/scan_geometry_check.py <bag> <map.yaml> [--every 5]
"""
import argparse
import glob
import json
import math
import os
import sqlite3
import sys

import numpy as np
import yaml


def messages(path, wanted):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    files = glob.glob(os.path.join(path, '*.db3')) if os.path.isdir(path) else [path]
    db = sqlite3.connect(files[0])
    topics = {tid: (name, get_message(kind)) for tid, name, kind in db.execute('select id, name, type from topics')}
    ids = [tid for tid, (name, _) in topics.items() if name in wanted]
    query = 'select topic_id, data from messages where topic_id in (%s) order by timestamp' % ','.join(
        str(i) for i in ids)
    for tid, raw in (db.execute(query) if ids else []):
        name, cls = topics[tid]
        yield name, deserialize_message(raw, cls)


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_map(path):
    from PIL import Image
    from scipy.ndimage import distance_transform_edt
    meta = yaml.safe_load(open(path))
    image = np.array(Image.open(os.path.join(os.path.dirname(path), meta['image'])))
    occupied = (255 - image) / 255.0 > meta.get('occupied_thresh', 0.65)
    distance = distance_transform_edt(~occupied) * meta['resolution']
    return distance, meta['resolution'], meta['origin'][0], meta['origin'][1]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bag')
    parser.add_argument('map')
    parser.add_argument('--every', type=int, default=5)
    parser.add_argument('--self-range', type=float, default=0.3)
    parser.add_argument('--laser-frame', default='laser_link')
    parser.add_argument('--base-frame', default='base_footprint')
    args = parser.parse_args()
    distance, resolution, ox, oy = load_map(args.map)
    height = distance.shape[0]

    def map_distance(points):
        col = np.clip(((points[:, 0] - ox) / resolution).astype(int), 0, distance.shape[1] - 1)
        row = np.clip(height - 1 - ((points[:, 1] - oy) / resolution).astype(int), 0, height - 1)
        return distance[row, col]

    statics, truth, scans, odom = {}, [], [], []
    for name, msg in messages(args.bag, {'/tf_static', '/odom/ground_truth', '/scan', '/tf'}):
        if name == '/tf':
            for t in msg.transforms:
                if t.header.frame_id == 'odom' and t.child_frame_id == args.base_frame:
                    odom.append((stamp(t.header), yaw_of(t.transform.rotation)))
        elif name == '/tf_static':
            for t in msg.transforms:
                statics[t.child_frame_id] = (t.header.frame_id, t.transform)
        elif name == '/odom/ground_truth':
            p = msg.pose.pose.position
            truth.append((stamp(msg.header), p.x, p.y, yaw_of(msg.pose.pose.orientation),
                          msg.twist.twist.angular.z))
        else:
            scans.append(msg)
    # base -> laser by chaining static transforms (planar: x, y, yaw).
    chain, frame = [], args.laser_frame
    while frame in statics and frame != args.base_frame:
        parent, transform = statics[frame]
        chain.append(transform)
        frame = parent
    lx = ly = lyaw = 0.0
    for transform in reversed(chain):
        t, q = transform.translation, transform.rotation
        c, s = math.cos(lyaw), math.sin(lyaw)
        lx, ly = lx + c * t.x - s * t.y, ly + s * t.x + c * t.y
        lyaw += yaw_of(q)
    truth = np.array(truth)
    report = {'scans': len(scans), 'laser_in_base': {'x': round(lx, 4), 'y': round(ly, 4),
                                                      'yaw_rad': round(lyaw, 6)}}
    short = None
    yaw_offsets, offsets_turning, residuals = [], [], []
    for scan in scans[::args.every]:
        t = stamp(scan.header)
        i = int(np.clip(np.searchsorted(truth[:, 0], t), 1, len(truth) - 1))
        if abs(truth[i, 0] - t) > 0.05:
            continue
        x, y, yaw, wz = truth[i, 1:]
        ranges = np.asarray(scan.ranges, dtype=float)
        angles = scan.angle_min + scan.angle_increment * np.arange(len(ranges))
        valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < scan.range_max)
        if short is None:
            short = np.zeros(len(ranges))
        short += valid & (ranges < args.self_range)
        valid &= ranges >= args.self_range
        if valid.sum() < 50:
            continue
        cx, cy = x + math.cos(yaw) * lx - math.sin(yaw) * ly, y + math.sin(yaw) * lx + math.cos(yaw) * ly

        def score(dyaw):
            a = yaw + lyaw + dyaw + angles[valid]
            points = np.column_stack([cx + ranges[valid] * np.cos(a), cy + ranges[valid] * np.sin(a)])
            return float(np.mean(np.minimum(map_distance(points), 0.3)))
        candidates = np.radians(np.linspace(-3, 3, 121))
        values = [score(d) for d in candidates]
        best = float(candidates[int(np.argmin(values))])
        residuals.append(score(0.0))
        (offsets_turning if abs(wz) > 0.2 else yaw_offsets).append(best)
    report['short_returns'] = {'beams_ever_short': int((short > 0).sum()) if short is not None else 0,
                               'beams_short_in_half_the_scans': int((short > len(scans[::args.every]) / 2).sum())
                               if short is not None else 0}
    report['median_residual_m'] = round(float(np.median(residuals)), 4) if residuals else None
    report['best_yaw_offset_deg_still'] = round(math.degrees(float(np.median(yaw_offsets))), 3) if yaw_offsets else None
    report['best_yaw_offset_deg_turning'] = (round(math.degrees(float(np.median(offsets_turning))), 3)
                                             if offsets_turning else None)
    report['samples'] = {'still': len(yaw_offsets), 'turning': len(offsets_turning)}
    if len(odom) > 100 and len(truth) > 100:
        # Resample both yaw tracks at 200 Hz and correlate their rates.
        odom = np.array(sorted(odom))
        t0, t1 = max(odom[0, 0], truth[0, 0]), min(odom[-1, 0], truth[-1, 0])
        grid = np.arange(t0, t1, 0.005)
        track = lambda t, y: np.interp(grid, t, np.unwrap(y))
        rate_truth = np.gradient(track(truth[:, 0], truth[:, 3]), 0.005)
        rate_odom = np.gradient(track(odom[:, 0], odom[:, 1]), 0.005)
        lags = np.arange(-40, 41)   # +-200 ms
        scores = [np.corrcoef(rate_truth[max(0, -k):len(grid) - max(0, k)],
                              rate_odom[max(0, k):len(grid) - max(0, -k)])[0, 1] for k in lags]
        best = int(lags[int(np.nanargmax(scores))])
        report['odom_tf_lag_ms'] = best * 5
        report['odom_tf_rate_correlation'] = round(float(np.nanmax(scores)), 4)
        report['odom_tf_messages'] = int(len(odom))
    print(json.dumps(report, indent=1))


if __name__ == '__main__':
    sys.exit(main())
