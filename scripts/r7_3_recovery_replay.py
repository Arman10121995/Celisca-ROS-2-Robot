#!/usr/bin/env python3
"""R7.3 recovery: seed each scan matcher at a WRONG pose and see if it recovers.

The main R7.3 replay (scripts/r7_3_live_scan_replay.py) starts every localizer
at the true pose of the first scan, exactly as the node is seeded from
/initialpose.  This script tests the other half of the ROADMAP R7.3 acceptance
- "initialization and relocalization success" - by seeding each matcher at a
wrong pose and replaying the recorded scans and odometry, scoring whether it
converges back to truth and where it ends up.

The matchers in robot_lab_algorithms.scan_matching are local: ICP and NDT
descend to the nearest minimum and CorrelativeMatcher searches only a +/-0.3 m
window, and ScanLocalizer refuses any correction further than max_correction
(0.5 m by default) from the odometry guess.  Recovery is therefore bounded.
It is measured two ways:

* as shipped (max_correction 0.5 m): the tracker a run actually uses;
* gate relaxed (max_correction 5 m): the matcher's own basin of convergence,
  with the safety gate lifted so a large jump back to truth is allowed.

Offsets are applied to the truth pose of the first scan in the map frame:
translations of 0.3-2.0 m along +x (the full basin sweep), two perpendicular
offsets (0.7, 1.5 m in y) to check anisotropy, and pure heading offsets of
15-180 deg.  AMCL, which does global localization live, is not replayed here
(it is scored in the main replay).  Dead reckoning is kept as the
no-correction baseline - it never recovers.

Metrics per (method, offset): final and minimum position/heading error, the
scan at which the minimum is reached, whether it recovered (converged to
truth: position < 0.25 m and heading < 10 deg at some scan), whether it held
there (never lost again afterwards), the fraction of scans "lost" (error >
0.5 m), and whether it settled onto a wrong but stable pose (ended < 0.5 m
without ever converging).

  python3 scripts/r7_3_recovery_replay.py --map nav_maze --out <dir> <run.bag> ...
"""
import argparse
import json
import math
import os
import platform
import subprocess
import sys

for _v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_v, '1')

import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'robot_lab_algorithms'))
sys.path.insert(0, HERE)

from robot_lab_algorithms.scan_matching import ScanLocalizer, wrap  # noqa: E402
from r7_3_live_scan_replay import (  # noqa: E402
    read_bag, read_map, relative, scan_points, sensor_offset, stamp_of)

METHODS = ['dead_reckoning', 'icp_point_to_point', 'icp_point_to_line', 'ndt', 'correlative']
# The heading limit is below the smallest heading offset (15 deg), so a
# seed is only "recovered" once the matcher has actually corrected it; at
# 15 deg, dead reckoning counted as recovering from a 15 deg seed.
RECOVER_POS, RECOVER_HEAD, LOST = 0.25, math.radians(10), 0.5
GATES = [0.5, 5.0]  # max_correction: as shipped, and relaxed (the matcher's own basin)
# A focused, honest grid: translation along +x at the full distance sweep (the
# basin of convergence), two perpendicular offsets to check anisotropy, and pure
# heading offsets.  Full 4-direction sweep is 28 cells; this is 12.
TRANSLATIONS = [(d, 0.0) for d in (0.3, 0.5, 0.7, 1.0, 1.5, 2.0)] + [(0.0, 0.7), (0.0, 1.5)]
HEADINGS = [15, 30, 90, 180]


def offsets(truth0):
    """(label, wrong_start) test seeds around the first truth pose."""
    for dx, dy in TRANSLATIONS:
        yield ('x%+.1f_y%+.1f' % (dx, dy), np.array([truth0[0] + dx, truth0[1] + dy, truth0[2]]))
    for deg in HEADINGS:
        yield ('yaw%+d' % deg, np.array([truth0[0], truth0[1], wrap(truth0[2] + math.radians(deg))]))


def usable_scans(odom, truth, scans, rate):
    out, last = [], -1e9
    for msg in scans:
        t = stamp_of(msg.header)
        if t - last < 1.0 / rate:
            continue
        o, g = odom.at(t), truth.at(t)
        if o is None or g is None:
            continue
        out.append((t, msg, o, g))
        last = t
    return out


def prepare(path, occupancy, base, rate):
    """Read a bag once and precompute scan points for every usable scan.

    Points are shared across methods, offsets and both gate settings, so the
    per-scan range -> cartesian conversion is paid once per run, not once per
    (method, offset, gate) cell.
    """
    odom, truth, scans, _amcl, static = read_bag(path, base)
    if not scans:
        raise SystemExit('%s has no /scan' % path)
    offset = sensor_offset(static, scans[0].header.frame_id, base)
    usable = usable_scans(odom, truth, scans, rate)
    prepared = []
    for t, msg, o, g in usable:
        points = scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                             min(msg.range_max, 12.0), sensor_offset=tuple(offset))
        prepared.append((t, points, o, g))
    travelled = sum(math.hypot(b[3][0] - a[3][0], b[3][1] - a[3][1]) for a, b in zip(prepared, prepared[1:]))
    meta = {'bag': os.path.abspath(path), 'scans_used': len(prepared), 'beams': len(scans[0].ranges),
            'duration_s': round(prepared[-1][0] - prepared[0][0], 2), 'travelled_m': round(travelled, 2),
            'truth_start': [round(float(v), 3) for v in prepared[0][3]]}
    return prepared, prepared[0][3], meta


def recover(prepared, occupancy, method, wrong_start, max_correction):
    """Replay one method seeded at ``wrong_start``; return per-scan errors."""
    localizer = ScanLocalizer(method, occupancy, wrong_start, max_correction=max_correction)
    previous = prepared[0][2]
    pos_err, head_err = [], []
    for _t, points, o, g in prepared[1:]:
        delta = relative(previous, o)
        pose = localizer.step(delta, points)
        previous = o
        pos_err.append(math.hypot(pose[0] - g[0], pose[1] - g[1]))
        head_err.append(abs(wrap(pose[2] - g[2])))
    return pos_err, head_err, localizer.rejected


def score(label, wrong_start, pos_err, head_err, rejected):
    """Recovery facts for one (method, offset, gate) replay."""
    pos = np.asarray(pos_err)
    head = np.asarray(head_err)
    imin = int(np.argmin(pos))
    converged = [i for i in range(len(pos)) if pos[i] < RECOVER_POS and head[i] < RECOVER_HEAD]
    return {
        'offset_label': label, 'wrong_start': [round(float(v), 3) for v in wrong_start],
        'scans': len(pos), 'final_pos_m': round(float(pos[-1]), 3), 'min_pos_m': round(float(pos[imin]), 3),
        'min_head_deg': round(math.degrees(float(head[imin])), 2), 'scan_at_min': imin,
        'recovered': bool(converged), 'scan_recovered': (converged[0] if converged else None),
        'held': bool(converged) and bool(np.all(pos[converged[0]:] <= LOST)),
        'lost_fraction': round(float(np.mean(pos > LOST)), 3), 'rejected_matches': rejected,
        'wrong_stable': bool(not converged and pos[-1] < LOST)}


def replay_bag(path, occupancy, base, rate, gates):
    """All (method, offset, gate) cells for one bag, as a nested dict."""
    prepared, truth0, meta = prepare(path, occupancy, base, rate)
    cells = {}
    for method in METHODS:
        cells[method] = {}
        for gate in gates:
            # dead_reckoning ignores max_correction; compute it once and reuse.
            if method == 'dead_reckoning' and 'max_correction_%g' % gates[0] in cells[method]:
                cells[method]['max_correction_%g' % gate] = cells[method]['max_correction_%g' % gates[0]]
                continue
            by_offset = {}
            for label, wrong_start in offsets(truth0):
                pos, head, rejected = recover(prepared, occupancy, method, wrong_start, gate)
                by_offset[label] = score(label, wrong_start, pos, head, rejected)
            cells[method]['max_correction_%g' % gate] = by_offset
            print('    %s gate=%g done' % (method, gate), flush=True)
    return meta, cells


def aggregate(runs):
    """Per (method, gate, offset): recovered fraction and error means over runs."""
    gates = ['max_correction_%g' % g for g in GATES]
    labels = [label for label, _ in offsets(np.zeros(3))]
    agg = {}
    for method in METHODS:
        agg[method] = {}
        for gate in gates:
            agg[method][gate] = {}
            for label in labels:
                rows = [run['cells'][method][gate][label] for run in runs.values() if gate in run['cells'][method]]
                if not rows:
                    continue
                agg[method][gate][label] = {
                    'runs': len(rows), 'recovered_fraction': round(float(np.mean([r['recovered'] for r in rows])), 3),
                    'held_fraction': round(float(np.mean([r['held'] for r in rows])), 3),
                    'mean_final_pos_m': round(float(np.mean([r['final_pos_m'] for r in rows])), 3),
                    'mean_min_pos_m': round(float(np.mean([r['min_pos_m'] for r in rows])), 3),
                    'wrong_stable_fraction': round(float(np.mean([r['wrong_stable'] for r in rows])), 3)}
    return agg


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bags', nargs='+')
    parser.add_argument('--map', default='nav_maze')
    parser.add_argument('--base', default='base_footprint')
    parser.add_argument('--rate', type=float, default=2.0, help='scans per second replayed')
    parser.add_argument('--label', default='')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    occupancy = read_map(args.map)
    runs = {}
    for bag in args.bags:
        meta, cells = replay_bag(bag, occupancy, args.base, args.rate, GATES)
        runs[os.path.basename(os.path.normpath(bag))] = {'meta': meta, 'cells': cells}
        print(os.path.basename(os.path.normpath(bag)), '%d scans' % meta['scans_used'])
    agg = aggregate(runs)
    results = {
        'revision': subprocess.run(['git', '-C', HERE, 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip(),
        'host': '%s %s' % (platform.node(), platform.machine()), 'label': args.label, 'map': args.map,
        'rate_hz': args.rate, 'recover_pos_m': RECOVER_POS, 'recover_head_deg': round(math.degrees(RECOVER_HEAD), 1),
        'lost_m': LOST, 'gates': GATES, 'runs': runs, 'aggregate': agg}
    lines = ['# R7.3 recovery: scan matchers seeded at a wrong pose%s'
             % (' (%s)' % args.label if args.label else ''), '',
             'Revision `%s`; map %s; %d run(s); %.0f scans/s; each matcher seeded at the wrong pose and '
             'replayed. "recovered" = position < %.2f m and heading < %.0f deg at some scan.'
             % (results['revision'][:7], args.map, len(runs), args.rate, RECOVER_POS, math.degrees(RECOVER_HEAD)), '']
    for gate in ('max_correction_%g' % GATES[0], 'max_correction_%g' % GATES[1]):
        lines += ['## Gate %s m (recovered fraction, per offset)' % gate.split('_')[-1], '',
                  '| Method | ' + ' | '.join(l for l, _ in offsets(np.zeros(3))) + ' |',
                  '|---' * (1 + len(list(offsets(np.zeros(3))))) + '|']
        for method in METHODS:
            row = [method]
            for label in [l for l, _ in offsets(np.zeros(3))]:
                cell = agg[method][gate].get(label)
                row.append('%d/%d' % (round(cell['recovered_fraction'] * cell['runs']), cell['runs']) if cell else '-')
            lines.append('| ' + ' | '.join(row) + ' |')
        lines.append('')
    report = '\n'.join(lines) + '\n'
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'results.json'), 'w') as handle:
        json.dump(results, handle, indent=1)
    with open(os.path.join(args.out, 'report.md'), 'w') as handle:
        handle.write(report)
    print(report)


if __name__ == '__main__':
    main()
