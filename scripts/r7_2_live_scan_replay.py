#!/usr/bin/env python3
"""R7.2 benchmark: scan segmentation on recorded simulator scans.

The offline benchmark (scripts/r7_2_perception_benchmark.py) clusters
synthetic scenes with round obstacles.  Here the same robot_lab_algorithms
perception kernels run on the real 360-beam ``/scan`` streams the R7.3
simulator runs recorded, and the task is the one a ground robot actually
faces in nav_maze: split the scan at the corners between wall segments.

Ground truth is the map itself.  For each scan the nav_maze occupancy grid
is ray-cast from the recorded ``/odom/ground_truth`` pose, giving a
noise-free reference scan; a *true* boundary is a pair of adjacent valid
reference beams whose endpoints are more than ``BOUNDARY_SEP_M`` apart in
the world (a corner), with max-range/open beams breaking adjacency exactly
as on the predicted side.  Each kernel clusters the real scan; a
*predicted* boundary is a pair of adjacent valid beams placed in different
clusters.  Boundaries are matched within ``MATCH_TOL_BEAMS`` beams and
scored as precision / recall / F1, with clusters-per-scan and compute time.

The five kernels are the offline benchmark's laser set: obstacle_detector
(single-linkage on points), scan_clusterer with a fixed and with an adaptive
breakpoint, and DBSCAN and Euclidean clustering on the scan points.  The
breakpoint/eps/tolerance is set to the wall-segment scale of the maze
(0.30 m), not the round-obstacle scale of the synthetic scene.

  python3 scripts/r7_2_live_scan_replay.py --map nav_maze --out <dir> <bag> [<bag> ...]

Real simulator data, replayed offline; the live nodes publish the same
clusters but their timing in a running stack is not measured here.
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

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'robot_lab_algorithms'))

import numpy as np  # noqa: E402

from r7_3_live_scan_replay import read_bag, read_map, sensor_offset, stamp_of  # noqa: E402
from robot_lab_algorithms.perception import (  # noqa: E402
    DBSCANClusterer, EuclideanClusterer, ObstacleDetector, ScanClusterer)

BASE = 'base_footprint'
BOUNDARY_SEP_M = 0.20      # a true boundary: adjacent valid reference beams > this apart
MATCH_TOL_BEAMS = 2        # predicted/true boundary match tolerance, in beams
CLUSTER_DIST = 0.30        # breakpoint / eps / tolerance: the maze wall-segment scale
RAY_STEP = 0.02            # occupancy ray-cast step (m)
LABELS = {'obstacle_detector': 'Obstacle detector (single linkage)',
          'scan_clusterer_fixed': 'Scan clusterer, fixed breakpoint',
          'scan_clusterer_adaptive': 'Scan clusterer, adaptive breakpoint',
          'dbscan': 'DBSCAN', 'euclidean': 'Euclidean clustering'}


def cast(grid, origin, res, x0, y0, theta, rmin, rmax):
    """Range to the first occupied cell along ``theta`` (occupancy >= 65)."""
    dx, dy = math.cos(theta), math.sin(theta)
    t = rmin
    rows, cols = grid.shape
    while t < rmax:
        col = int(math.floor((x0 + t * dx - origin[0]) / res))
        row = int(math.floor((y0 + t * dy - origin[1]) / res))
        if 0 <= row < rows and 0 <= col < cols and grid[row, col] >= 65:
            return t
        t += RAY_STEP
    return rmax


def true_boundaries(ranges, pose, laser_yaw, angle_min, angle_inc, rmin, rmax, grid, origin, res):
    """Boundary beam indices from a noise-free ray-cast of the map at ``pose``."""
    prev = None
    boundaries = set()
    for k, _ in enumerate(ranges):
        theta = pose[2] + laser_yaw + (angle_min + k * angle_inc)
        r = cast(grid, origin, res, pose[0], pose[1], theta, rmin, rmax)
        if r >= rmax - 1e-6:            # max-range / open beam: not an obstacle, break adjacency
            prev = None
            continue
        px, py = pose[0] + r * math.cos(theta), pose[1] + r * math.sin(theta)
        if prev is not None and math.hypot(px - prev[0], py - prev[1]) > BOUNDARY_SEP_M:
            boundaries.add(k - 1)
        prev = (px, py)
    return boundaries


def beam_of_point(x, y, angle_min, angle_inc, nbeam):
    ang = math.atan2(y, x)
    return int(round((ang - angle_min) / angle_inc)) % nbeam


def predicted_boundaries(clusters, angle_min, angle_inc, nbeam):
    """Boundary beam indices: adjacent valid beams assigned to different clusters."""
    cluster_of = {}
    for ci, cluster in enumerate(clusters):
        for p in cluster:
            cluster_of[beam_of_point(p[0], p[1], angle_min, angle_inc, nbeam)] = ci
    order = sorted(cluster_of)
    return {a for a, b in zip(order, order[1:]) if b == a + 1 and cluster_of[a] != cluster_of[b]}


def cluster_scan(ranges, angle_min, angle_inc, rmin, rmax, method):
    """Run one perception kernel on the real scan; returns a list of point clusters."""
    if method == 'scan_clusterer_fixed':
        return ScanClusterer(cluster_distance=CLUSTER_DIST).cluster_ranges(angle_min, angle_inc, list(ranges), rmax)
    if method == 'scan_clusterer_adaptive':
        return ScanClusterer(cluster_distance=CLUSTER_DIST, adaptive=True).cluster_ranges(
            angle_min, angle_inc, list(ranges), rmax)
    points = []
    for k, r in enumerate(ranges):
        if r <= rmin or r >= rmax:
            continue
        theta = angle_min + k * angle_inc
        points.append((r * math.cos(theta), r * math.sin(theta)))
    if method == 'obstacle_detector':
        return ObstacleDetector(cluster_distance=CLUSTER_DIST).detect(points)
    if method == 'dbscan':
        return DBSCANClusterer(eps=CLUSTER_DIST, min_samples=3).cluster([(x, y, 0.0) for x, y in points])[0]
    if method == 'euclidean':
        return EuclideanClusterer(tolerance=CLUSTER_DIST, min_points=3).cluster([(x, y, 0.0) for x, y in points])
    raise SystemExit('unknown method %s' % method)


def match_boundaries(truth, pred, tol=MATCH_TOL_BEAMS):
    used, tp = set(), 0
    for b in pred:
        candidates = [t for t in truth if abs(t - b) <= tol and t not in used]
        if candidates:
            used.add(min(candidates, key=lambda t: abs(t - b)))
            tp += 1
    return tp, len(pred) - tp, len(truth) - tp     # tp, fp, fn


def _prf(tp, fp, fn):
    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)
    return round(precision, 4), round(recall, 4), round(2 * precision * recall / max(1e-9, precision + recall), 4)


def replay_bag(path, occupancy, skip, stride):
    odom, truth, scans, _, static = read_bag(path, BASE)
    if not scans:
        raise SystemExit('%s has no /scan' % path)
    rmin = max(scans[0].range_min, 0.1)
    rmax = min(scans[0].range_max, 11.9)
    angle_min, angle_inc = scans[0].angle_min, scans[0].angle_increment
    nbeam = len(scans[0].ranges)
    laser_yaw = sensor_offset(static, scans[0].header.frame_id, BASE)[2]
    grid, origin, res = occupancy.grid, occupancy.origin, occupancy.resolution
    methods = sorted(LABELS)
    acc = {m: {'tp': 0, 'fp': 0, 'fn': 0, 'clusters': 0, 'segments': 0, 'ms': []} for m in methods}
    used = 0
    for i in range(skip, len(scans), stride):
        scan = scans[i]
        pose = truth.at(stamp_of(scan.header))
        if pose is None:
            continue
        truth_b = true_boundaries(scan.ranges, pose, laser_yaw, angle_min, angle_inc, rmin, rmax, grid, origin, res)
        used += 1
        for m in methods:
            start = time.perf_counter()
            clusters = cluster_scan(scan.ranges, angle_min, angle_inc, rmin, rmax, m)
            acc[m]['ms'].append((time.perf_counter() - start) * 1e3)
            pred_b = predicted_boundaries(clusters, angle_min, angle_inc, nbeam)
            tp, fp, fn = match_boundaries(truth_b, pred_b)
            acc[m]['tp'] += tp; acc[m]['fp'] += fp; acc[m]['fn'] += fn
            acc[m]['clusters'] += len(clusters); acc[m]['segments'] += len(truth_b) + 1
    per_method = {}
    for m in methods:
        r = acc[m]
        precision, recall, f1 = _prf(r['tp'], r['fp'], r['fn'])
        per_method[m] = {
            'precision': precision, 'recall': recall, 'f1': f1,
            'tp': r['tp'], 'fp': r['fp'], 'fn': r['fn'],
            'clusters_per_scan': round(r['clusters'] / max(1, used), 2),
            'true_segments_per_scan': round(r['segments'] / max(1, used), 2),
            'ms_p50': round(float(np.median(r['ms'])), 3) if r['ms'] else None,
            'ms_max': round(float(np.max(r['ms'])), 3) if r['ms'] else None}
    meta = {'bag': os.path.abspath(path), 'scans_used': used, 'beams': nbeam,
            'laser_yaw': round(float(laser_yaw), 4), 'range': [round(rmin, 3), round(rmax, 3)],
            'boundary_sep_m': BOUNDARY_SEP_M, 'cluster_dist_m': CLUSTER_DIST}
    return meta, per_method


def merge(runs):
    methods = sorted(LABELS)
    tot = {m: {'tp': 0, 'fp': 0, 'fn': 0, 'clusters': 0.0, 'segments': 0.0} for m in methods}
    for per in runs.values():
        for m in methods:
            tot[m]['clusters'] += per[m]['clusters_per_scan']
            tot[m]['segments'] += per[m]['true_segments_per_scan']
            tot[m]['tp'] += per[m]['tp']; tot[m]['fp'] += per[m]['fp']; tot[m]['fn'] += per[m]['fn']
    summary = {}
    for m in methods:
        precision, recall, f1 = _prf(tot[m]['tp'], tot[m]['fp'], tot[m]['fn'])
        summary[m] = {'precision': precision, 'recall': recall, 'f1': f1,
                      'clusters_per_scan': round(tot[m]['clusters'] / len(runs), 2),
                      'true_segments_per_scan': round(tot[m]['segments'] / len(runs), 2),
                      'tp': tot[m]['tp'], 'fp': tot[m]['fp'], 'fn': tot[m]['fn']}
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--map', default='nav_maze')
    parser.add_argument('--out', required=True)
    parser.add_argument('--skip', type=int, default=20, help='first scans to skip (warm-up)')
    parser.add_argument('--stride', type=int, default=40, help='keep every Nth scan')
    parser.add_argument('bags', nargs='+')
    args = parser.parse_args()
    occupancy = read_map(args.map)
    runs = {}
    scans_used = {}
    metas = {}
    for bag in args.bags:
        meta, per = replay_bag(bag, occupancy, args.skip, args.stride)
        key = os.path.basename(os.path.dirname(bag)) + '/' + os.path.basename(bag)
        runs[key] = per
        scans_used[key] = meta['scans_used']
        metas[key] = meta
        print('%s: %d scans' % (os.path.basename(bag), meta['scans_used']), file=sys.stderr)
    summary = merge(runs)
    revision = subprocess.run(['git', '-C', HERE, 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
    results = {'revision': revision, 'host': '%s %s' % (platform.node(), platform.machine()),
               'map': args.map, 'boundary_sep_m': BOUNDARY_SEP_M, 'cluster_dist_m': CLUSTER_DIST,
               'match_tol_beams': MATCH_TOL_BEAMS, 'bags': len(args.bags),
               'summary': summary, 'runs': runs, 'metas': metas}
    first = list(runs)[0]
    lines = ['# R7.2 perception on simulator scans (%s)' % args.map, '',
             'Revision `%s`, %s; %d bags, %d scans each (every %d after the first %d).' % (
                 revision[:7], results['host'], len(args.bags), scans_used[first],
                 args.stride, args.skip), '',
             'Boundaries: adjacent valid reference beams > %.2f m apart, matched within %d beams.' % (
                 BOUNDARY_SEP_M, MATCH_TOL_BEAMS), '',
             '| Method | Precision | Recall | F1 | Clusters / scan | True segments / scan | ms p50 |',
             '|---|---|---|---|---|---|---|']
    for m in sorted(summary):
        r = summary[m]
        lines.append('| %s | %.3f | %.3f | %.3f | %.1f | %.1f | %s |' % (
            LABELS[m], r['precision'], r['recall'], r['f1'], r['clusters_per_scan'],
            r['true_segments_per_scan'],
            ('%.2f' % runs[list(runs)[0]][m]['ms_p50']) if runs[list(runs)[0]][m]['ms_p50'] else '-'))
    os.makedirs(args.out, exist_ok=True)
    json.dump(results, open(os.path.join(args.out, 'results.json'), 'w'), indent=1)
    open(os.path.join(args.out, 'report.md'), 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
