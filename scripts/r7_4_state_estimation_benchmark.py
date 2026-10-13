#!/usr/bin/env python3
"""R7.4 benchmark: state estimators on noisy position fixes with known truth.

A ground robot drives a CTRV trajectory for 300 s: straight runs, left and
right turns of 0.2-0.6 rad/s, speed changes between 0.15 and 0.8 m/s and a
stop.  A sensor reports its position at 10 Hz with 0.3 m noise.  Each
estimator (robot_lab_algorithms.tracking_filters) estimates position,
heading, speed and turn rate from the fixes alone.

The process-noise gain (accel_std) of each estimator is chosen on tuning
seed 0 by position RMSE; results are means over evaluation seeds.  The raw
fixes are the baseline.  Modelled sensor, not a simulator.

  python3 scripts/r7_4_state_estimation_benchmark.py --out <dir>
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

from robot_lab_algorithms.tracking_filters import ESTIMATORS, ctrv, wrap  # noqa: E402

DT = 0.1
NOISE = 0.3
GRID = [0.1, 0.2, 0.35, 0.5, 1.0, 2.0]


def profile(t):
    """(speed, turn rate) along the 300 s route; repeats every 100 s."""
    s = t % 100.0
    if s < 15:
        return 0.5, 0.0
    if s < 25:
        return 0.5, 0.4
    if s < 35:
        return 0.8, 0.0
    if s < 45:
        return 0.3, -0.6
    if s < 55:
        return 0.15, 0.0
    if s < 60:
        return 0.0, 0.0
    if s < 75:
        return 0.6, 0.2
    if s < 85:
        return 0.6, -0.3
    return 0.4, 0.0


def data(seed, duration=300.0):
    rng = np.random.default_rng(seed)
    state = np.array([0.0, 0.0, rng.uniform(-math.pi, math.pi), 0.0, 0.0])
    rows = []
    for k in range(int(duration / DT)):
        v, w = profile(k * DT)
        state = ctrv(np.array([state[0], state[1], state[2], v, w]), DT)
        rows.append((state.copy(), state[:2] + rng.normal(0, NOISE, 2)))
    return rows


def run(name, rows, accel_std):
    flt = ESTIMATORS[name](measurement_std=NOISE, accel_std=accel_std)
    est, compute = [], []
    for truth, z in rows:
        start = time.perf_counter()
        flt.predict(DT)
        flt.update(z)
        compute.append(time.perf_counter() - start)
        est.append(flt.estimate())
    truth = np.array([r[0] for r in rows])[50:]
    e = np.array(est)[50:]
    moving = truth[:, 3] > 0.1
    fast = truth[:, 3] > 0.3
    ms = np.array(compute) * 1e3
    return {
        'position_rmse_m': round(float(np.sqrt(np.mean(np.sum((e[:, :2] - truth[:, :2]) ** 2, axis=1)))), 4),
        'speed_rmse_mps': round(float(np.sqrt(np.mean((e[:, 3] - truth[:, 3]) ** 2))), 4),
        'heading_rmse_deg': round(math.degrees(float(np.sqrt(np.mean(
            [wrap(a - b) ** 2 for a, b in zip(e[moving, 2], truth[moving, 2])])))), 2),
        'heading_rmse_fast_deg': round(math.degrees(float(np.sqrt(np.mean(
            [wrap(a - b) ** 2 for a, b in zip(e[fast, 2], truth[fast, 2])])))), 2),
        'turn_rate_rmse': round(float(np.sqrt(np.mean((e[:, 4] - truth[:, 4]) ** 2))), 4),
        'compute_ms': {'p50': round(float(np.percentile(ms, 50)), 4), 'p99': round(float(np.percentile(ms, 99)), 4)},
    }


def mean(rows):
    out = {}
    for key, value in rows[0].items():
        out[key] = mean([r[key] for r in rows]) if isinstance(value, dict) else round(float(np.mean([r[key] for r in rows])), 4)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', required=True)
    parser.add_argument('--seeds', default='1,2,3')
    args = parser.parse_args()
    seeds = [int(s) for s in args.seeds.split(',')]
    tuning_rows = data(0)
    tuning, gains = {}, {}
    for name in ESTIMATORS:
        scores = {g: run(name, tuning_rows, g)['position_rmse_m'] for g in GRID}
        tuning[name] = scores
        gains[name] = min(scores, key=scores.get)
    per_seed = {name: [] for name in ESTIMATORS}
    raw = []
    for seed in seeds:
        rows = data(seed)
        truth = np.array([r[0] for r in rows])[50:]
        fixes = np.array([r[1] for r in rows])[50:]
        raw.append(float(np.sqrt(np.mean(np.sum((fixes - truth[:, :2]) ** 2, axis=1)))))
        for name in ESTIMATORS:
            per_seed[name].append(run(name, rows, gains[name]))
    results = {
        'revision': subprocess.run(['git', '-C', HERE, 'rev-parse', 'HEAD'], capture_output=True,
                                   text=True).stdout.strip(),
        'host': '%s %s' % (platform.node(), platform.machine()), 'python': platform.python_version(),
        'numpy': np.__version__, 'seeds': seeds, 'tune_seed': 0, 'measurement_rate_hz': 1.0 / DT,
        'measurement_std_m': NOISE, 'raw_fix_rmse_m': round(float(np.mean(raw)), 4),
        'tuning_position_rmse': tuning, 'gains_accel_std': gains,
        'estimators': {name: mean(rows) for name, rows in per_seed.items()}, 'per_seed': per_seed,
    }
    lines = ['# R7.4 state estimation benchmark (noisy position fixes)', '',
             'Revision `%s`, %s; seeds %s; fixes at %.0f Hz with %.1f m noise (raw fix RMSE %.3f m).' % (
                 results['revision'][:7], results['host'], seeds, 1.0 / DT, NOISE, results['raw_fix_rmse_m']), '',
             'accel_std chosen on seed 0: %s.' % ', '.join('%s %g' % kv for kv in gains.items()), '',
             '| Estimator | Position RMSE (m) | Speed RMSE (m/s) | Heading RMSE, v > 0.1 / v > 0.3 (deg) | Turn-rate RMSE (rad/s) | Compute p50 / p99 (ms) |',
             '|---|---|---|---|---|---|']
    for name, r in results['estimators'].items():
        lines.append('| %s | %.3f | %.3f | %.1f / %.1f | %.3f | %.3f / %.3f |' % (
            name, r['position_rmse_m'], r['speed_rmse_mps'], r['heading_rmse_deg'], r['heading_rmse_fast_deg'],
            r['turn_rate_rmse'],
            r['compute_ms']['p50'], r['compute_ms']['p99']))
    report = '\n'.join(lines) + '\n'
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'results.json'), 'w') as handle:
        json.dump(results, handle, indent=1)
    with open(os.path.join(args.out, 'report.md'), 'w') as handle:
        handle.write(report)
    print(report)


if __name__ == '__main__':
    main()
