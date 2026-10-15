#!/usr/bin/env python3
"""R7.2 on simulator depth clouds: score ground removal and clustering live.

Runs beside a launched Gazebo drive (``SIDECAR`` of
``scripts/slam_mapping_check.sh``).  Every ``--period`` seconds of sim time
it takes the latest RGB-D cloud (/oakd/points, camera optical frame), moves
it into the robot frame (static TF, as the perception nodes now do) and,
with the simulator's truth pose, into the world.  Each point is labelled:

* ground: world z within ``--ground-band`` of the floor (z = 0);
* obstacle: above that band and within 0.12 m (in x, y) of an occupied cell
  of the world's reference 2D map; its object is the connected component of
  that cell (8-connected occupied cells: a wall run, a block);
* other: anything else (left out of the scores).

Ground removal (robot_lab_algorithms.perception, in the robot frame): the
height threshold (0.1 m) and RANSAC (0.04 m, 200 iterations), scored per
point against the ground label.  Clustering of the labelled obstacle points:
Euclidean (0.12 m, 20 points) and DBSCAN (eps 0.12 m, 8 samples); an object
with at least 60 visible points counts as found when one cluster holds at
least 80% of them with at least 80% purity.

Writes JSON on SIGINT/SIGTERM or after ``--samples`` clouds.

  python3 scripts/r7_2_depth_live.py --map nav_obstacle --out <file.json>
"""
import argparse
import json
import os
import signal
import sys
import time

for _variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_variable, '1')

import numpy as np  # noqa: E402
import yaml  # noqa: E402
from scipy import ndimage  # noqa: E402
from scipy.spatial import cKDTree  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot_lab_algorithms'))

from robot_lab_algorithms.perception import (  # noqa: E402
    DBSCANClusterer, EuclideanClusterer, PointcloudSegmenter, RANSACGroundRemoval, quaternion_matrix,
    upright_points)


def reference_objects(name):
    """KD-tree over occupied cell centres and each cell's object id."""
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
    occ = image.astype(float) / 255.0 if meta.get('negate') else (255 - image.astype(float)) / 255.0
    occupied = np.flipud(occ > meta['occupied_thresh'])
    labels, count = ndimage.label(occupied, structure=np.ones((3, 3)))
    rows, cols = np.nonzero(occupied)
    res, origin = float(meta['resolution']), np.array(meta['origin'][:2], dtype=float)
    centres = np.column_stack([cols, rows]) * res + origin + res / 2
    return cKDTree(centres), labels[rows, cols], count


def found_objects(clusters, object_of_point, index, minimum=60):
    """(objects found, objects visible) for one clustering."""
    ids, sizes = np.unique(object_of_point, return_counts=True)
    visible = [int(i) for i, n in zip(ids, sizes) if n >= minimum]
    member_labels = [np.array([object_of_point[index[p]] for p in cluster]) for cluster in clusters]
    found = 0
    for obj in visible:
        size = int(np.sum(object_of_point == obj))
        for labels in member_labels:
            held = int(np.sum(labels == obj))
            if held >= 0.8 * size and held >= 0.8 * len(labels):
                found += 1
                break
    return found, len(visible)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--map', default='nav_obstacle')
    parser.add_argument('--topic', default='/oakd/points')
    parser.add_argument('--base', default='base_footprint')
    parser.add_argument('--period', type=float, default=2.0, help='sim seconds between scored clouds')
    parser.add_argument('--samples', type=int, default=60)
    parser.add_argument('--max-points', type=int, default=8000)
    parser.add_argument('--ground-band', type=float, default=0.03)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from tf2_ros import Buffer, TransformListener

    tree, cell_object, object_count = reference_objects(args.map)
    rclpy.init()
    from rclpy.parameter import Parameter
    node = Node('r7_2_depth_live', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    state = {'cloud': None, 'truth': None}
    node.create_subscription(PointCloud2, args.topic, lambda m: state.__setitem__('cloud', m),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom/ground_truth', lambda m: state.__setitem__('truth', m),
                             qos_profile_sensor_data)
    tf = Buffer()
    TransformListener(tf, node)
    methods = {'pointcloud_segmenter (height 0.1 m)': PointcloudSegmenter(ground_threshold=0.1),
               'ransac_ground_removal': RANSACGroundRemoval(distance_threshold=0.04, max_iterations=200)}
    clusterers = {'euclidean_clusterer': lambda p: EuclideanClusterer(tolerance=0.12, min_points=20).cluster(p),
                  'dbscan_clusterer': lambda p: DBSCANClusterer(eps=0.12, min_samples=8).cluster(p)[0]}
    totals = {name: {'tp': 0, 'fp': 0, 'fn': 0, 'ms': []} for name in methods}
    cluster_totals = {name: {'found': 0, 'visible': 0, 'clusters': 0, 'ms': []} for name in clusterers}
    meta = {'samples': 0, 'points_scored': 0, 'ground_points': 0, 'obstacle_points': 0, 'other_points': 0,
            'ground_z_median_m': []}
    stop = {'now': False}
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.__setitem__('now', True))

    last = -1e9
    while rclpy.ok() and not stop['now'] and meta['samples'] < args.samples:
        rclpy.spin_once(node, timeout_sec=0.05)
        cloud, truth = state['cloud'], state['truth']
        if cloud is None or truth is None:
            continue
        stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        if stamp - last < args.period:
            continue
        try:
            t = tf.lookup_transform(args.base, cloud.header.frame_id, Time())
        except Exception:
            continue
        last = stamp
        raw = np.array([[p[0], p[1], p[2]] for p in point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z'), skip_nans=True)], dtype=float)
        raw = raw[np.isfinite(raw).all(axis=1)]  # beyond the far plane: inf
        if len(raw) < 100:
            continue
        q, tr = t.transform.rotation, t.transform.translation
        base = upright_points(raw, quaternion_matrix(q.x, q.y, q.z, q.w), (tr.x, tr.y, tr.z), args.max_points)
        p, o = truth.pose.pose.position, truth.pose.pose.orientation
        world = base @ quaternion_matrix(o.x, o.y, o.z, o.w).T + np.array([p.x, p.y, p.z])
        ground = world[:, 2] <= args.ground_band
        distance, nearest = tree.query(world[:, :2])
        obstacle = (~ground) & (distance <= 0.12)
        meta['samples'] += 1
        meta['points_scored'] += int(np.sum(ground | obstacle))
        meta['ground_points'] += int(ground.sum())
        meta['obstacle_points'] += int(obstacle.sum())
        meta['other_points'] += int(np.sum(~(ground | obstacle)))
        if ground.any():
            meta['ground_z_median_m'].append(float(np.median(world[ground, 2])))
        tuples = [tuple(row) for row in base]
        index = {pt: i for i, pt in enumerate(tuples)}
        scored = ground | obstacle
        for name, method in methods.items():
            start = time.perf_counter()
            predicted_ground, _ = method.segment(tuples)
            totals[name]['ms'].append((time.perf_counter() - start) * 1e3)
            predicted = np.zeros(len(tuples), dtype=bool)
            predicted[[index[pt] for pt in predicted_ground]] = True
            totals[name]['tp'] += int(np.sum(predicted & ground & scored))
            totals[name]['fp'] += int(np.sum(predicted & obstacle))
            totals[name]['fn'] += int(np.sum(~predicted & ground))
        obstacle_tuples = [tuples[i] for i in np.nonzero(obstacle)[0]]
        object_of_point = np.full(len(tuples), -1)
        object_of_point[obstacle] = cell_object[nearest[obstacle]]
        if len(obstacle_tuples) >= 20:
            sub_index = {pt: i for i, pt in enumerate(obstacle_tuples)}
            sub_objects = object_of_point[obstacle]
            for name, cluster in clusterers.items():
                start = time.perf_counter()
                clusters = cluster(obstacle_tuples)
                cluster_totals[name]['ms'].append((time.perf_counter() - start) * 1e3)
                found, visible = found_objects(clusters, sub_objects, sub_index)
                cluster_totals[name]['found'] += found
                cluster_totals[name]['visible'] += visible
                cluster_totals[name]['clusters'] += len(clusters)
    node.destroy_node()
    rclpy.shutdown()

    def rate(a, b):
        return round(a / b, 4) if b else None
    results = {
        'map': args.map, 'topic': args.topic, 'reference_objects': int(object_count),
        'samples': meta['samples'], 'points_scored': meta['points_scored'],
        'ground_points': meta['ground_points'], 'obstacle_points': meta['obstacle_points'],
        'other_points': meta['other_points'],
        'ground_z_median_m': round(float(np.median(meta['ground_z_median_m'])), 4) if meta['ground_z_median_m'] else None,
        'ground_removal': {name: {'precision': rate(r['tp'], r['tp'] + r['fp']), 'recall': rate(r['tp'], r['tp'] + r['fn']),
                                  'ms_p50': round(float(np.median(r['ms'])), 1) if r['ms'] else None}
                           for name, r in totals.items()},
        'clustering': {name: {'objects_found': '%d/%d' % (r['found'], r['visible']),
                              'clusters_per_cloud': rate(r['clusters'], max(1, len(r['ms']))),
                              'ms_p50': round(float(np.median(r['ms'])), 1) if r['ms'] else None}
                       for name, r in cluster_totals.items()},
    }
    with open(args.out, 'w') as handle:
        json.dump(results, handle, indent=1)
    print(json.dumps(results, indent=1))


if __name__ == '__main__':
    main()
