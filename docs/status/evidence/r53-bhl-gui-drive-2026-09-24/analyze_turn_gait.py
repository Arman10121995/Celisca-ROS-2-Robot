#!/usr/bin/env python3
"""R5.3: windowed turn and gait-activity summary from a probe artifact.

Reads a JSON artifact written by ``probe_key_vel.py`` (run it with
``BHL_PROBE_EFFORT_VECTOR=1`` when the gait signal is needed, which also keeps
the records at full rate) and prints, for every one-second window of the run:
the commanded post-mux twist, the measured yaw rate and translation speed, the
peak body tilt, and - when the record carries the effort vector - the effort
activity that distinguishes a *stepping* policy from one parked at a fixed
target:

- ``mean|tau|`` / ``std|tau|``: mean and standard deviation of the summed
  absolute joint effort over the window,
- ``active``: fraction of consecutive sample pairs in which some joint effort
  moved by more than ``--threshold`` N.m, i.e. the effort signal is swinging
  rather than constant.

Usage::

    python3 analyze_turn_gait.py turn_gait.json [--threshold 0.25]
"""

from __future__ import annotations

import argparse
import json
import math
import sys


def load(path):
    with open(path, encoding='utf-8') as handle:
        report = json.load(handle)
    if 'records' not in report or not report['records']:
        raise ValueError(f'{path}: no records')
    return report


def window_rows(report, window_s):
    records = report['records']
    start, last = records[0][0], records[-1][0]
    window = start
    while window < last:
        rows = [r for r in records if window <= r[0] < window + window_s]
        if len(rows) >= 2:
            yield window, rows
        window += window_s


def effort_stats(rows, threshold):
    vectors = [r[8] for r in rows if len(r) > 8 and r[8]]
    if len(vectors) < 2:
        return None
    sums = [sum(abs(v) for v in vec) for vec in vectors]
    mean = sum(sums) / len(sums)
    var = sum((s - mean) ** 2 for s in sums) / len(sums)
    active = 0
    for previous, current in zip(vectors, vectors[1:]):
        if max(abs(a - b) for a, b in zip(previous, current)) > threshold:
            active += 1
    return mean, math.sqrt(var), active / (len(vectors) - 1)


def describe(window, rows, window_s, threshold):
    t0, t1 = rows[0][0], rows[-1][0]
    span = t1 - t0 if t1 > t0 else window_s
    yaw_rate = (rows[-1][5] - rows[0][5]) / span
    speed = math.hypot(rows[-1][1] - rows[0][1],
                       rows[-1][2] - rows[0][2]) / span
    tilt = max(r[3] for r in rows)
    command = (rows[-1][6], rows[-1][7])
    stats = effort_stats(rows, threshold)
    line = (f'{window:6.1f}-{window + window_s:<5.1f} s  '
            f'mux=({command[0]:+.2f},{command[1]:+.2f})  '
            f'yaw={yaw_rate:+.3f} rad/s  v={speed:.3f} m/s  '
            f'tilt<={tilt:.3f} rad')
    if stats is None:
        return line + '  tau=n/a'
    mean, spread, active = stats
    return (line + f'  mean|tau|={mean:6.2f}  std|tau|={spread:5.2f}  '
            f'active={active:4.0%}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('artifact')
    parser.add_argument('--window', type=float, default=1.0,
                        help='window length in simulated seconds (default 1)')
    parser.add_argument('--threshold', type=float, default=0.25,
                        help='per-joint effort change counted as activity '
                             '[N.m] (default 0.25)')
    args = parser.parse_args(argv)
    if args.window <= 0:
        parser.error('window must be positive')
    report = load(args.artifact)
    print(f'{args.artifact}: command window '
          f'{report.get("command_window")} '
          f'command={report.get("command")} '
          f'effort_vector={report.get("effort_vector")}')
    for window, rows in window_rows(report, args.window):
        print(describe(window, rows, args.window, args.threshold))
    return 0


if __name__ == '__main__':
    sys.exit(main())
