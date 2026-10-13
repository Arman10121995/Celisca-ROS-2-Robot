#!/usr/bin/env python3
"""R7.3 benchmark: scan-to-map localizers on the nav_maze map with known truth.

Truth: Bumperbot trajectories recorded in Gazebo on nav_maze (the R4.3
evaluation traces, resampled to 5 Hz and stored in
``trajectories.json``).  Scans: 360 beams ray-cast on the same occupancy
map Nav2 uses, 0.02 m range noise, 3% dropouts, 12 m range.  Odometry:
the truth increments with +3% distance scale, +2% rotation scale, a
heading drift of 0.5 deg per metre and white noise - the same for every
localizer (seeded).

Each localizer predicts with the odometry increment and aligns the scan
to the map (robot_lab_algorithms.scan_matching); dead reckoning is the
baseline.  Metrics: position/heading RMSE, maximum error, time "lost"
(error > 0.5 m), compute per scan.

  python3 scripts/r7_3_localization_benchmark.py --out <dir> [--traces trace.json ...]
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
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot_lab_algorithms'))

from robot_lab_algorithms.scan_matching import (  # noqa: E402
    MATCHERS, OccupancyMap, ScanLocalizer, compose, relative, wrap)

MAP = os.path.join(ROOT, 'src', 'robot_lab_maps', 'maps', 'nav_maze', 'maps', 'map.pgm')
RATE = 5.0
BEAMS = 360
MAX_RANGE = 12.0


def read_map(path=MAP, resolution=0.05, origin=(-9.0, -9.0)):
    with open(path, 'rb') as handle:
        assert handle.readline().strip() == b'P5'
        line = handle.readline()
        while line.startswith(b'#'):
            line = handle.readline()
        width, height = (int(v) for v in line.split())
        int(handle.readline())
        image = np.frombuffer(handle.read(), dtype=np.uint8).reshape(height, width)
    occupancy = (255 - image.astype(float)) / 255.0
    grid = np.where(occupancy > 0.65, 100, np.where(occupancy < 0.196, 0, -1))
    grid = np.flipud(grid)  # image row 0 is the top (largest y)
    return OccupancyMap(grid.ravel(), width, height, resolution, origin)


def raycast(occ, pose, angles, rng, step=0.02):
    """Ranges from pose along each angle until an occupied cell (vectorised march)."""
    distances = np.arange(0.05, MAX_RANGE, step)
    a = pose[2] + angles
    xs = pose[0] + np.outer(np.cos(a), distances)
    ys = pose[1] + np.outer(np.sin(a), distances)
    cols = np.floor((xs - occ.origin[0]) / occ.resolution).astype(int)
    rows = np.floor((ys - occ.origin[1]) / occ.resolution).astype(int)
    inside = (rows >= 0) & (rows < occ.grid.shape[0]) & (cols >= 0) & (cols < occ.grid.shape[1])
    hit = np.zeros_like(inside)
    hit[inside] = occ.grid[rows[inside], cols[inside]] >= 65
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
    ranges = np.where(first >= 0, distances[np.maximum(first, 0)], np.inf)
    ranges = ranges + rng.normal(0, 0.02, len(ranges))
    ranges[rng.random(len(ranges)) < 0.03] = np.inf
    return ranges


def trajectories_from_traces(paths):
    out = []
    for path in paths:
        samples = json.load(open(path))['samples']
        t0, picked, last = samples[0]['stamp_s'], [], -1e9
        for s in samples:
            if s['stamp_s'] - last >= 1.0 / RATE - 1e-6:
                picked.append([round(s['stamp_s'] - t0, 3), s['x'], s['y'], s['yaw']])
                last = s['stamp_s']
        out.append({'source': os.path.basename(path), 'poses': picked})
    return out


def odometry_deltas(poses, rng):
    deltas = []
    for a, b in zip(poses[:-1], poses[1:]):
        dx, dy, dth = relative(np.array(a[1:]), np.array(b[1:]))
        dist = math.hypot(dx, dy)
        noisy = np.array([dx * 1.03 + rng.normal(0, 0.003), dy * 1.03 + rng.normal(0, 0.003),
                          dth * 1.02 + math.radians(0.5) * dist + rng.normal(0, 0.003)])
        deltas.append(noisy)
    return deltas


def run(method, occ, trajectory, seed):
    rng = np.random.default_rng(seed)
    poses = trajectory['poses']
    deltas = odometry_deltas(poses, rng)
    angles = np.linspace(-math.pi, math.pi, BEAMS, endpoint=False)
    localizer = ScanLocalizer(method, occ, poses[0][1:])
    errors, heading, compute = [], [], []
    for delta, truth in zip(deltas, poses[1:]):
        ranges = raycast(occ, np.array(truth[1:]), angles, rng)
        ok = np.isfinite(ranges)
        points = np.column_stack([ranges[ok] * np.cos(angles[ok]), ranges[ok] * np.sin(angles[ok])])[::2]
        start = time.perf_counter()
        pose = localizer.step(delta, points)
        compute.append(time.perf_counter() - start)
        errors.append(math.hypot(pose[0] - truth[1], pose[1] - truth[2]))
        heading.append(abs(wrap(pose[2] - truth[3])))
    e = np.array(errors)
    ms = np.array(compute) * 1e3
    return {'position_rmse_m': round(float(np.sqrt(np.mean(e ** 2))), 4), 'max_error_m': round(float(e.max()), 4),
            'final_error_m': round(float(e[-1]), 4),
            'heading_rmse_deg': round(math.degrees(float(np.sqrt(np.mean(np.square(heading))))), 3),
            'lost_fraction': round(float(np.mean(e > 0.5)), 4), 'rejected_matches': localizer.rejected,
            'compute_ms': {'p50': round(float(np.percentile(ms, 50)), 3), 'p99': round(float(np.percentile(ms, 99)), 3)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', required=True)
    parser.add_argument('--traces', nargs='*', help='rebuild trajectories.json from Gazebo trace files')
    parser.add_argument('--seeds', default='1,2')
    parser.add_argument('--max-steps', type=int, default=0, help='truncate trajectories (development)')
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, 'trajectories.json')
    if args.traces:
        json.dump(trajectories_from_traces(args.traces), open(path, 'w'))
    trajectories = json.load(open(path))
    if args.max_steps:
        for t in trajectories:
            t['poses'] = t['poses'][:args.max_steps]
    occ = read_map()
    seeds = [int(s) for s in args.seeds.split(',')]
    results = {name: [] for name in MATCHERS}
    for trajectory in trajectories:
        for seed in seeds:
            for name in MATCHERS:
                row = run(name, occ, trajectory, seed)
                row.update({'trajectory': trajectory['source'], 'seed': seed})
                results[name].append(row)

    def mean(rows, key):
        return round(float(np.mean([r[key] for r in rows])), 4)
    summary = {name: {key: mean(rows, key) for key in ('position_rmse_m', 'max_error_m', 'final_error_m',
                                                        'heading_rmse_deg', 'lost_fraction')}
               | {'compute_ms_p50': round(float(np.mean([r['compute_ms']['p50'] for r in rows])), 3),
                  'compute_ms_p99': round(float(np.max([r['compute_ms']['p99'] for r in rows])), 3)}
               for name, rows in results.items()}
    meta = {'revision': subprocess.run(['git', '-C', HERE, 'rev-parse', 'HEAD'], capture_output=True,
                                       text=True).stdout.strip(),
            'host': '%s %s' % (platform.node(), platform.machine()), 'seeds': seeds,
            'trajectories': [{'source': t['source'], 'scans': len(t['poses']) - 1,
                              'length_m': round(float(sum(math.hypot(b[1] - a[1], b[2] - a[2])
                                                          for a, b in zip(t['poses'], t['poses'][1:]))), 2)}
                             for t in trajectories]}
    json.dump({'meta': meta, 'summary': summary, 'runs': results}, open(os.path.join(args.out, 'results.json'), 'w'),
              indent=1)
    lines = ['# R7.3 localization benchmark (nav_maze, ray-cast scans)', '',
             'Revision `%s`, %s; %d trajectories (%s m), seeds %s.' % (
                 meta['revision'][:7], meta['host'], len(trajectories),
                 ', '.join(str(t['length_m']) for t in meta['trajectories']), seeds), '',
             '| Localizer | Position RMSE (m) | Max error (m) | Final error (m) | Heading RMSE (deg) | Lost (>0.5 m) | Compute p50 / max p99 (ms) |',
             '|---|---|---|---|---|---|---|']
    for name, r in summary.items():
        lines.append('| %s | %.3f | %.3f | %.3f | %.2f | %.1f%% | %.2f / %.2f |' % (
            name, r['position_rmse_m'], r['max_error_m'], r['final_error_m'], r['heading_rmse_deg'],
            100 * r['lost_fraction'], r['compute_ms_p50'], r['compute_ms_p99']))
    report = '\n'.join(lines) + '\n'
    open(os.path.join(args.out, 'report.md'), 'w').write(report)
    print(report)


if __name__ == '__main__':
    main()
