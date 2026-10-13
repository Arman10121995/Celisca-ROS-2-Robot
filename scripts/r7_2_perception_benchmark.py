#!/usr/bin/env python3
"""R7.2 benchmark: perception kernels on synthetic scenes with known objects.

2D (laser): a 8 x 6 m room with six random round obstacles (radius
0.10-0.25 m) seen by a 360-beam scan with 0.01 m noise.  Clusters with at
least 3 points and an extent below 0.7 m are obstacle candidates; a
candidate is a true detection when its centroid lies within (radius +
0.15 m) of a visible obstacle (one hit by at least 3 beams).  Methods:
obstacle_detector (single linkage on points), scan_clusterer with a fixed
and with an adaptive breakpoint, DBSCAN and Euclidean clustering on the
scan points.

3D (depth cloud): a ground plane tilted 2 deg with 0.01 m noise, a wall,
and four boxes (0.2-0.5 m), about 6,000 points.  Ground removal: height
threshold (pointcloud_segmenter) vs RANSAC; scored per point.  Object
clustering on the non-ground points (ground truth labels): Euclidean vs
DBSCAN; an object counts as found when one cluster holds at least 80% of
its points with at least 80% purity.

Seeded scenes, the same for every method.  Synthetic data, not a simulator.

  python3 scripts/r7_2_perception_benchmark.py --out <dir>
"""
import argparse
import json
import math
import os
import platform
import subprocess
import sys
import time

for _variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_variable, '1')

import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'robot_lab_algorithms'))

from robot_lab_algorithms.perception import (  # noqa: E402
    DBSCANClusterer, EuclideanClusterer, ObstacleDetector, PointcloudSegmenter, RANSACGroundRemoval,
    ScanClusterer)


# ---------------------------------------------------------------------------
# 2D scenes
# ---------------------------------------------------------------------------

def scene_2d(rng):
    obstacles = []
    while len(obstacles) < 6:
        c = np.array([rng.uniform(0.6, 7.4), rng.uniform(0.6, 5.4)])
        r = rng.uniform(0.10, 0.25)
        if all(np.linalg.norm(c - o[0]) > o[1] + r + 0.4 for o in obstacles):
            obstacles.append((c, r))
    while True:
        robot = np.array([rng.uniform(1.0, 7.0), rng.uniform(1.0, 5.0)])
        if all(np.linalg.norm(robot - c) > r + 0.5 for c, r in obstacles):
            break
    return obstacles, robot


def scan_2d(obstacles, robot, rng, beams=360, max_range=10.0):
    angles = np.linspace(-math.pi, math.pi, beams, endpoint=False)
    d = np.column_stack([np.cos(angles), np.sin(angles)])
    ranges = np.full(beams, np.inf)
    hit_obstacle = np.full(beams, -1)
    # Walls of the 8 x 6 room.
    for axis, value in ((0, 0.0), (0, 8.0), (1, 0.0), (1, 6.0)):
        with np.errstate(divide='ignore', invalid='ignore'):
            t = (value - robot[axis]) / d[:, axis]
        t[t <= 0] = np.inf
        ranges = np.minimum(ranges, t)
    for k, (c, r) in enumerate(obstacles):
        oc = robot - c
        b = d @ oc
        disc = b * b - (oc @ oc - r * r)
        t = np.where(disc >= 0, -b - np.sqrt(np.maximum(disc, 0)), np.inf)
        t[t <= 0] = np.inf
        closer = t < ranges
        ranges[closer] = t[closer]
        hit_obstacle[closer] = k
    ranges = np.where(ranges < max_range, ranges + rng.normal(0, 0.01, beams), np.inf)
    return angles, ranges, hit_obstacle


def evaluate_2d(clusters, obstacles, visible, robot):
    candidates = []
    for cluster in clusters:
        pts = np.asarray(cluster, dtype=float)[:, :2]
        if len(pts) < 3:
            continue
        extent = float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)))
        if extent < 0.7:
            candidates.append(pts.mean(axis=0) + robot)
    matched, errors, true_positive = set(), [], 0
    for centroid in candidates:
        best = None
        for k in visible:
            c, r = obstacles[k]
            dist = float(np.linalg.norm(centroid - c))
            if dist < r + 0.15 and k not in matched and (best is None or dist < best[1]):
                best = (k, dist)
        if best is not None:
            matched.add(best[0])
            true_positive += 1
            errors.append(best[1] - obstacles[best[0]][1])  # centroid sits on the visible arc
    return true_positive, len(candidates), len(visible), errors


METHODS_2D = {
    'obstacle_detector': lambda pts, angles, ranges: ObstacleDetector(cluster_distance=0.15).detect(pts),
    'scan_clusterer_fixed': lambda pts, angles, ranges: ScanClusterer(cluster_distance=0.15).cluster_ranges(
        angles[0], angles[1] - angles[0], [r if np.isfinite(r) else 99.0 for r in ranges], 50.0),
    'scan_clusterer_adaptive': lambda pts, angles, ranges: ScanClusterer(cluster_distance=0.1, adaptive=True).cluster_ranges(
        angles[0], angles[1] - angles[0], [r if np.isfinite(r) else 99.0 for r in ranges], 50.0),
    'dbscan': lambda pts, angles, ranges: DBSCANClusterer(eps=0.15, min_samples=3).cluster([(x, y, 0.0) for x, y in pts])[0],
    'euclidean': lambda pts, angles, ranges: EuclideanClusterer(tolerance=0.15, min_points=3).cluster([(x, y, 0.0) for x, y in pts]),
}


def run_2d(seeds):
    results = {name: {'tp': 0, 'candidates': 0, 'visible': 0, 'errors': [], 'ms': []} for name in METHODS_2D}
    for seed in seeds:
        rng = np.random.default_rng(seed)
        obstacles, robot = scene_2d(rng)
        angles, ranges, hits = scan_2d(obstacles, robot, rng)
        visible = [k for k in range(len(obstacles)) if np.sum(hits == k) >= 3]
        ok = np.isfinite(ranges)
        pts = [(float(r * math.cos(a)), float(r * math.sin(a))) for a, r in zip(angles[ok], ranges[ok])]
        for name, method in METHODS_2D.items():
            start = time.perf_counter()
            clusters = method(pts, angles, ranges)
            results[name]['ms'].append((time.perf_counter() - start) * 1e3)
            tp, cand, vis, errors = evaluate_2d(clusters, obstacles, visible, robot)
            r = results[name]
            r['tp'] += tp
            r['candidates'] += cand
            r['visible'] += vis
            r['errors'] += errors
    summary = {}
    for name, r in results.items():
        summary[name] = {'precision': round(r['tp'] / max(1, r['candidates']), 4),
                         'recall': round(r['tp'] / max(1, r['visible']), 4),
                         'centroid_error_m': round(float(np.median(r['errors'])), 4) if r['errors'] else None,
                         'ms_p50': round(float(np.median(r['ms'])), 3), 'ms_max': round(float(np.max(r['ms'])), 3)}
    return summary


# ---------------------------------------------------------------------------
# 3D clouds
# ---------------------------------------------------------------------------

def cloud_3d(rng):
    tilt = math.radians(2.0)
    gx, gy = rng.uniform(0.5, 6.0, 4000), rng.uniform(-2.5, 2.5, 4000)
    gz = gx * math.tan(tilt) + rng.normal(0, 0.01, 4000)
    points = [np.column_stack([gx, gy, gz])]
    labels = [np.zeros(4000, dtype=int)]                       # 0 = ground
    wy, wz = rng.uniform(-2.5, 2.5, 600), rng.uniform(0.0, 1.5, 600)
    points.append(np.column_stack([np.full(600, 6.2) + rng.normal(0, 0.01, 600), wy, wz + 6.2 * math.tan(tilt)]))
    labels.append(np.full(600, 1))                             # 1 = wall
    placed = []
    for k in range(4):
        while True:
            c = np.array([rng.uniform(1.0, 5.5), rng.uniform(-2.0, 2.0)])
            if all(np.linalg.norm(c - p) > 1.0 for p in placed):
                placed.append(c)
                break
        size = rng.uniform(0.2, 0.5, 3)
        n = 350
        local = rng.uniform(-0.5, 0.5, (n, 3)) * size
        face = rng.integers(0, 3, n)
        local[np.arange(n), face] = np.sign(local[np.arange(n), face]) * size[face] / 2  # points on box faces
        base_z = c[0] * math.tan(tilt)
        box = local + np.array([c[0], c[1], base_z + size[2] / 2 + 0.02])
        points.append(box)
        labels.append(np.full(n, 2 + k))                        # 2.. = boxes
    return np.vstack(points), np.concatenate(labels)


def run_3d(seeds):
    ground_methods = {
        'pointcloud_segmenter (height 0.1 m)': lambda pts: PointcloudSegmenter(ground_threshold=0.1).segment(pts),
        'ransac_ground_removal': lambda pts: RANSACGroundRemoval(distance_threshold=0.04, max_iterations=200).segment(pts),
    }
    cluster_methods = {
        'euclidean_clusterer': lambda pts: EuclideanClusterer(tolerance=0.12, min_points=20).cluster(pts),
        'dbscan_clusterer': lambda pts: DBSCANClusterer(eps=0.12, min_samples=8).cluster(pts)[0],
    }
    g_res = {name: {'tp': 0, 'fp': 0, 'fn': 0, 'ms': []} for name in ground_methods}
    c_res = {name: {'found': 0, 'objects': 0, 'clusters': 0, 'ms': []} for name in cluster_methods}
    for seed in seeds:
        rng = np.random.default_rng(seed)
        pts, labels = cloud_3d(rng)
        tuples = [tuple(p) for p in pts]
        index = {p: i for i, p in enumerate(tuples)}
        for name, method in ground_methods.items():
            start = time.perf_counter()
            ground, _ = method(tuples)
            g_res[name]['ms'].append((time.perf_counter() - start) * 1e3)
            predicted = np.zeros(len(pts), dtype=bool)
            predicted[[index[p] for p in ground]] = True
            truth = labels == 0
            g_res[name]['tp'] += int(np.sum(predicted & truth))
            g_res[name]['fp'] += int(np.sum(predicted & ~truth))
            g_res[name]['fn'] += int(np.sum(~predicted & truth))
        objects = [tuples[i] for i in np.nonzero(labels >= 2)[0]] + [tuples[i] for i in np.nonzero(labels == 1)[0]]
        for name, method in cluster_methods.items():
            start = time.perf_counter()
            clusters = method(objects)
            c_res[name]['ms'].append((time.perf_counter() - start) * 1e3)
            c_res[name]['clusters'] += len(clusters)
            cluster_labels = [np.array([labels[index[p]] for p in cluster]) for cluster in clusters]
            for k in range(2, 6):
                size = int(np.sum(labels == k))
                c_res[name]['objects'] += 1
                for cl in cluster_labels:
                    held = int(np.sum(cl == k))
                    if held >= 0.8 * size and held >= 0.8 * len(cl):
                        c_res[name]['found'] += 1
                        break
    ground_summary = {name: {'precision': round(r['tp'] / max(1, r['tp'] + r['fp']), 4),
                             'recall': round(r['tp'] / max(1, r['tp'] + r['fn']), 4),
                             'ms_p50': round(float(np.median(r['ms'])), 2)} for name, r in g_res.items()}
    cluster_summary = {name: {'objects_found': '%d/%d' % (r['found'], r['objects']),
                              'clusters_per_scene': round(r['clusters'] / len(seeds), 2),
                              'ms_p50': round(float(np.median(r['ms'])), 2)} for name, r in c_res.items()}
    return ground_summary, cluster_summary


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', required=True)
    parser.add_argument('--scenes', type=int, default=50)
    args = parser.parse_args()
    seeds = list(range(1, args.scenes + 1))
    run_2d([0])  # warm-up: imports and first KD-tree build are not timed
    laser = run_2d(seeds)
    ground, clusters = run_3d(seeds[:max(5, args.scenes // 5)])
    results = {'revision': subprocess.run(['git', '-C', HERE, 'rev-parse', 'HEAD'], capture_output=True,
                                          text=True).stdout.strip(),
               'host': '%s %s' % (platform.node(), platform.machine()), 'scenes_2d': len(seeds),
               'scenes_3d': max(5, args.scenes // 5), 'laser': laser, 'ground': ground, 'clusters': clusters}
    lines = ['# R7.2 perception benchmark (synthetic scenes)', '',
             'Revision `%s`, %s; %d laser scenes, %d point clouds.' % (
                 results['revision'][:7], results['host'], results['scenes_2d'], results['scenes_3d']), '',
             '## Laser obstacle detection', '',
             '| Method | Precision | Recall | Median centroid offset (m) | ms p50 / max |', '|---|---|---|---|---|']
    for name, r in laser.items():
        lines.append('| %s | %.3f | %.3f | %s | %.2f / %.2f |' % (name, r['precision'], r['recall'],
                                                                r['centroid_error_m'], r['ms_p50'], r['ms_max']))
    lines += ['', '## Ground removal (per point)', '', '| Method | Precision | Recall | ms p50 |', '|---|---|---|---|']
    for name, r in ground.items():
        lines.append('| %s | %.3f | %.3f | %.1f |' % (name, r['precision'], r['recall'], r['ms_p50']))
    lines += ['', '## Object clustering (non-ground points)', '', '| Method | Boxes found | Clusters per scene | ms p50 |',
              '|---|---|---|---|']
    for name, r in clusters.items():
        lines.append('| %s | %s | %.1f | %.1f |' % (name, r['objects_found'], r['clusters_per_scene'], r['ms_p50']))
    report = '\n'.join(lines) + '\n'
    os.makedirs(args.out, exist_ok=True)
    json.dump(results, open(os.path.join(args.out, 'results.json'), 'w'), indent=1)
    open(os.path.join(args.out, 'report.md'), 'w').write(report)
    print(report)


if __name__ == '__main__':
    main()
