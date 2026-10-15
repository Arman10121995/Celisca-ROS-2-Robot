#!/usr/bin/env python3
"""R7.2: check depth_image_to_pointcloud against the simulator's own cloud, live.

Runs beside a launched drive (``SIDECAR`` of ``scripts/slam_mapping_check.sh``)
while the node runs.  For every depth image it times the node: wall-clock
time from the image's arrival to the node's cloud with the same stamp, and
the images that got no cloud.  Every ``--period`` seconds of sim time it
pairs the node's cloud (``--topic``) with the simulator's cloud of the same
stamp (``--reference``: Gazebo's /oakd/points, already in the camera's
optical frame) and measures their agreement:

* distance from each node point to the nearest reference point (median,
  p95, max);
* coverage: the share of reference points within the node's depth range
  that have a node point within 1 cm (meaningful at stride 1).

MuJoCo, PyBullet and Isaac publish only the depth image, so there only the
timing and point counts are reported (``--reference ''``).

  python3 scripts/r7_2_depth_projection_check.py --out <file.json>
"""
import argparse
import json
import os
import signal
import time

for _variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_variable, '1')

import numpy as np  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402


def stamp_key(header):
    return (header.stamp.sec, header.stamp.nanosec)


def xyz(cloud):
    from sensor_msgs_py import point_cloud2
    points = point_cloud2.read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return points[np.isfinite(points).all(axis=1)]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--depth', default='/oakd/depth/image_raw')
    parser.add_argument('--topic', default='/perception/depth_points')
    parser.add_argument('--reference', default='/oakd/points', help="the simulator's cloud ('' for none)")
    parser.add_argument('--period', type=float, default=1.0, help='sim seconds between compared pairs')
    parser.add_argument('--min-range', type=float, default=0.05)
    parser.add_argument('--max-range', type=float, default=10.0)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, PointCloud2

    rclpy.init()
    node = Node('r7_2_depth_projection_check',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    arrivals, latencies, counts = {}, [], []
    clouds, references = {}, {}
    pairs = {'compared': 0, 'median_m': [], 'p95_m': [], 'max_m': [], 'coverage': [],
             'node_points': [], 'reference_points': []}
    seen = {'images': 0, 'clouds': 0, 'references': 0, 'last_compared': -1e9, 'frames': set()}

    def compare(key):
        ours, theirs = clouds.pop(key), references.pop(key)
        if len(ours) == 0 or len(theirs) == 0:
            return
        distance, _ = cKDTree(theirs).query(ours)
        depth = theirs[:, 2]
        in_range = theirs[(depth >= args.min_range) & (depth <= args.max_range)]
        covered, _ = cKDTree(ours).query(in_range, distance_upper_bound=0.01) if len(in_range) else ([], None)
        pairs['compared'] += 1
        pairs['median_m'].append(float(np.median(distance)))
        pairs['p95_m'].append(float(np.percentile(distance, 95)))
        pairs['max_m'].append(float(distance.max()))
        pairs['coverage'].append(float(np.mean(np.isfinite(covered))) if len(in_range) else None)
        pairs['node_points'].append(len(ours))
        pairs['reference_points'].append(len(in_range))

    def due(key):
        t = key[0] + key[1] * 1e-9
        if t - seen['last_compared'] < args.period:
            return False
        seen['last_compared'] = t
        return True

    def on_image(msg):
        seen['images'] += 1
        arrivals[stamp_key(msg.header)] = time.monotonic()
        for old in sorted(arrivals)[:-200]:
            arrivals.pop(old)

    def on_cloud(msg):
        now = time.monotonic()
        key = stamp_key(msg.header)
        seen['clouds'] += 1
        seen['frames'].add(msg.header.frame_id)
        counts.append(msg.width * msg.height)
        if key in arrivals:
            latencies.append((now - arrivals.pop(key)) * 1e3)
        if args.reference and (key in references or due(key)):
            clouds[key] = xyz(msg)
            if key in references:
                compare(key)
        for old in sorted(clouds)[:-20]:
            clouds.pop(old)

    def on_reference(msg):
        key = stamp_key(msg.header)
        seen['references'] += 1
        if key in clouds:
            references[key] = xyz(msg)
            compare(key)
        elif due(key):
            references[key] = xyz(msg)
        for old in sorted(references)[:-20]:
            references.pop(old)

    node.create_subscription(Image, args.depth, on_image, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, args.topic, on_cloud, qos_profile_sensor_data)
    if args.reference:
        node.create_subscription(PointCloud2, args.reference, on_reference, qos_profile_sensor_data)
    stop = {'now': False}
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__('now', True))
    while rclpy.ok() and not stop['now']:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    def summary(values, digits=4):
        values = [v for v in values if v is not None]
        if not values:
            return None
        return {'median': round(float(np.median(values)), digits), 'p95': round(float(np.percentile(values, 95)), digits),
                'max': round(float(np.max(values)), digits)}
    results = {
        'depth_topic': args.depth, 'topic': args.topic, 'reference': args.reference or None,
        'depth_images': seen['images'], 'clouds': seen['clouds'], 'reference_clouds': seen['references'],
        'cloud_frames': sorted(seen['frames']),
        'images_without_cloud': max(0, seen['images'] - seen['clouds']),
        'points_per_cloud': summary(counts, 0),
        'latency_ms': summary(latencies, 1),
        'agreement': None if not pairs['compared'] else {
            'pairs': pairs['compared'],
            'nearest_reference_median_m': summary(pairs['median_m'], 6),
            'nearest_reference_p95_m': summary(pairs['p95_m'], 6),
            'nearest_reference_max_m': summary(pairs['max_m'], 6),
            'reference_coverage_1cm': summary(pairs['coverage']),
            'node_points': summary(pairs['node_points'], 0),
            'reference_points_in_range': summary(pairs['reference_points'], 0)},
    }
    with open(args.out, 'w') as handle:
        json.dump(results, handle, indent=1)
    print(json.dumps(results, indent=1))


if __name__ == '__main__':
    main()
