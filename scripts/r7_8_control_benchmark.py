#!/usr/bin/env python3
"""R7.8 benchmark: six path-tracking controllers on one differential-drive plant.

Every controller in robot_lab_algorithms.tracking_control drives the same
ActuatorPlant (unicycle, first-order actuator lag, acceleration limits,
saturation) at the ROS server's 20 Hz.  Tests:

  tracking      three paths (S-curve, rectilinear turns, tight arc)
  disturbance   a 0.25 m / 0.3 rad pose kick mid-run on the S-curve
  envelope      initial lateral/heading offsets on a straight path
  saturation    0.8 m lateral start with the turn rate capped at 0.3 rad/s
                (PID with and without anti-windup)
  deadline      compute time of every step against the 50 ms period

This is a model-plant benchmark: it measures the control laws, not a
simulator or a robot.  Gazebo runs of the same controllers are separate.

  python3 scripts/r7_8_control_benchmark.py --out <dir>
"""
import argparse
import json
import math
import os
import platform
import subprocess
import sys

# Small dense problems: single-threaded BLAS gives honest per-step timings.
for _variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_variable, '1')

import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'robot_lab_algorithms'))

from robot_lab_algorithms.tracking_control import (  # noqa: E402
    CONTROLLERS, ActuatorPlant, Limits, PidTracker, Reference, make_controller, wrap)

DT = 0.05
SETTLE_LATERAL = 0.03
SETTLE_HEADING = 0.05


def s_curve():
    xs = np.linspace(0.0, 12.0, 600)
    return [(x, math.sin(x / 2.0)) for x in xs]


def rectilinear():
    corners = [(0.0, 0.0), (3.0, 0.0), (3.0, 2.0), (6.0, 2.0), (6.0, -1.0), (9.0, -1.0)]
    points = []
    for (ax, ay), (bx, by) in zip(corners[:-1], corners[1:]):
        n = int(math.hypot(bx - ax, by - ay) / 0.05)
        points += [(ax + (bx - ax) * i / n, ay + (by - ay) * i / n) for i in range(n)]
    return points + [corners[-1]]


def tight_arc():
    angles = np.linspace(-math.pi / 2, math.pi, 300)
    return [(0.5 * math.cos(a), 0.5 + 0.5 * math.sin(a)) for a in angles]


def straight(length=10.0):
    return [(x, 0.0) for x in np.linspace(0.0, length, int(length / 0.05) + 1)]


PATHS = {'s_curve': s_curve, 'rectilinear_turns': rectilinear, 'tight_arc': tight_arc}


def run(controller, reference, pose, limits, duration, kick=None):
    """Simulate; returns per-step records and summary."""
    plant = ActuatorPlant(limits=limits, pose=pose)
    hint = reference.project(pose[0], pose[1], 0.0, back=0.0, ahead=2.0)
    steps, compute = [], []
    t = 0.0
    finished = False
    while t < duration:
        if kick is not None and abs(t - kick[0]) < DT / 2:
            _, _, theta, _ = reference.state(hint)
            plant.pose = plant.pose + np.array([-math.sin(theta) * kick[1], math.cos(theta) * kick[1], kick[2]])
        p = plant.pose
        hint = reference.project(p[0], p[1], hint)
        along, lateral, heading = reference.errors(p, hint)
        if hint >= reference.length - 0.05 and math.hypot(p[0] - reference.x[-1], p[1] - reference.y[-1]) < 0.1:
            finished = True
            break
        v, w = controller.command(tuple(p), tuple(plant.velocity), reference, hint, DT)
        compute.append(controller.last_compute_s)
        steps.append((t, lateral, heading, v, w, float(plant.velocity[0]), float(plant.velocity[1])))
        plant.step((v, w), DT)
        t += DT
    return steps, compute, finished, t


def effort(steps):
    w = np.array([s[6] for s in steps]) if steps else np.zeros(1)
    v = np.array([s[5] for s in steps]) if steps else np.zeros(1)
    cmd_w = np.array([s[4] for s in steps]) if steps else np.zeros(1)
    return {'integral_w_sq': round(float(np.sum(w * w) * DT), 4),
            'integral_v_sq': round(float(np.sum(v * v) * DT), 4),
            'command_w_total_variation': round(float(np.sum(np.abs(np.diff(cmd_w)))), 4)}


def settle_time(steps, after=0.0, hold=1.0):
    """First time after ``after`` from which errors stay settled for ``hold`` s."""
    candidate = None
    for t, lateral, heading, *_ in steps:
        if t < after:
            continue
        if abs(lateral) < SETTLE_LATERAL and abs(heading) < SETTLE_HEADING:
            if candidate is None:
                candidate = t
            if t - candidate >= hold:
                return round(candidate - after, 2)
        else:
            candidate = None
    return None


def tracking_tests(limits):
    rows, compute_all = [], {name: [] for name in CONTROLLERS}
    for path_name, factory in PATHS.items():
        reference = Reference(factory(), v_max=limits.v_max)
        x, y, theta, _ = reference.state(0.0)
        duration = 3.0 * reference.length / limits.v_max + 20.0
        for name in CONTROLLERS:
            controller = make_controller(name, limits=limits)
            steps, compute, finished, t = run(controller, reference, (x, y, theta), limits, duration)
            compute_all[name] += compute
            lateral = np.array([s[1] for s in steps])
            heading = np.array([s[2] for s in steps])
            rows.append({'path': path_name, 'controller': name, 'path_length_m': round(reference.length, 3),
                         'finished': finished, 'time_s': round(t, 2),
                         'rms_lateral_m': round(float(np.sqrt(np.mean(lateral ** 2))), 4),
                         'max_lateral_m': round(float(np.max(np.abs(lateral))), 4),
                         'rms_heading_deg': round(math.degrees(float(np.sqrt(np.mean(heading ** 2)))), 2),
                         **effort(steps)})
    return rows, compute_all


def disturbance_tests(limits, compute_all):
    reference = Reference(s_curve(), v_max=limits.v_max)
    x, y, theta, _ = reference.state(0.0)
    rows = []
    for name in CONTROLLERS:
        controller = make_controller(name, limits=limits)
        kick_t = 8.0
        steps, compute, finished, t = run(controller, reference, (x, y, theta), limits, 60.0,
                                          kick=(kick_t, 0.25, 0.3))
        compute_all[name] += compute
        after = [s for s in steps if s[0] >= kick_t]
        peak = max((abs(s[1]) for s in after), default=None)
        recovery = [s for s in after if s[0] <= kick_t + 10.0]
        rows.append({'controller': name, 'kick': {'time_s': kick_t, 'lateral_m': 0.25, 'heading_rad': 0.3},
                     'peak_lateral_after_kick_m': round(peak, 4) if peak is not None else None,
                     'recovery_s': settle_time(steps, after=kick_t),
                     'finished': finished, **effort(recovery)})
    return rows


def envelope_tests(limits, compute_all, laterals, headings_deg, horizon=20.0):
    reference = Reference(straight(12.0), v_max=limits.v_max)
    rows = []
    for name in CONTROLLERS:
        grid = []
        for lateral in laterals:
            for heading in headings_deg:
                controller = make_controller(name, limits=limits)
                steps, compute, _, _ = run(controller, reference, (0.5, lateral, math.radians(heading)),
                                           limits, horizon)
                compute_all[name] += compute
                settle = settle_time(steps)
                grid.append({'lateral_m': lateral, 'heading_deg': heading, 'settle_s': settle})
        converged = sum(1 for g in grid if g['settle_s'] is not None)
        rows.append({'controller': name, 'converged': converged, 'cells': len(grid),
                     'median_settle_s': round(float(np.median([g['settle_s'] for g in grid
                                                               if g['settle_s'] is not None])), 2)
                     if converged else None,
                     'grid': grid})
    return rows


def saturation_tests(compute_all):
    limits = Limits(w_max=0.3)
    reference = Reference(straight(12.0), v_max=limits.v_max)
    rows = []
    variants = [(name, lambda n=name: make_controller(n, limits=limits)) for name in CONTROLLERS]
    variants.append(('pid_without_anti_windup', lambda: PidTracker(limits=limits, anti_windup=False)))
    for label, factory in variants:
        controller = factory()
        steps, compute, _, _ = run(controller, reference, (0.5, 0.8, 0.0), limits, 35.0)
        if label in compute_all:
            compute_all[label] += compute
        lateral = np.array([s[1] for s in steps])
        saturated = sum(1 for s in steps if abs(s[4]) >= limits.w_max - 1e-9)
        rows.append({'controller': label, 'w_max': limits.w_max, 'initial_lateral_m': 0.8,
                     'overshoot_m': round(float(max(0.0, -lateral.min())), 4),
                     'settle_s': settle_time(steps), 'saturated_steps': saturated,
                     **effort(steps)})
    return rows


def deadline_summary(compute_all, deadline):
    rows = []
    for name, values in compute_all.items():
        if not values:
            continue
        ms = np.array(values) * 1e3
        rows.append({'controller': name, 'steps': int(len(ms)),
                     'p50_ms': round(float(np.percentile(ms, 50)), 3),
                     'p95_ms': round(float(np.percentile(ms, 95)), 3),
                     'p99_ms': round(float(np.percentile(ms, 99)), 3),
                     'max_ms': round(float(ms.max()), 2),
                     'misses': int(np.sum(ms > deadline * 1e3)),
                     'miss_rate': round(float(np.mean(ms > deadline * 1e3)), 5)})
    return rows


def markdown(results):
    lines = ['# R7.8 control benchmark (model plant)', '',
             'Revision `%s`, %s, Python %s, NumPy %s, SciPy %s.' % (
                 results['revision'][:7], results['host'], results['python'], results['numpy'],
                 results['scipy']), '',
             'Plant: unicycle, actuator lag %.2f s, |a| <= %.1f m/s^2, |alpha| <= %.1f rad/s^2, '
             'v in [%.1f, %.1f] m/s, |w| <= %.1f rad/s; control period %.2f s.' % (
                 results['plant']['tau_s'], results['plant']['a_max'], results['plant']['alpha_max'],
                 results['plant']['v_min'], results['plant']['v_max'], results['plant']['w_max'], DT), '',
             'Host load average (1/5/15 min) %s at start, %s at end, %d CPUs; other processes '
             'were running, so compute times are not from an idle host.' % (
                 results['host_load_average']['start'], results['host_load_average']['end'],
                 results['host_load_average']['cpus']), '',
             '## Tracking', '',
             '| Path | Controller | Finished | Time (s) | RMS lateral (m) | Max lateral (m) | '
             'RMS heading (deg) | int w^2 dt | Command TV |',
             '|---|---|---|---|---|---|---|---|---|']
    for r in results['tracking']:
        lines.append('| %s | %s | %s | %.1f | %.4f | %.3f | %.2f | %.3f | %.2f |' % (
            r['path'], r['controller'], 'yes' if r['finished'] else '**no**', r['time_s'],
            r['rms_lateral_m'], r['max_lateral_m'], r['rms_heading_deg'], r['integral_w_sq'],
            r['command_w_total_variation']))
    lines += ['', '## Disturbance (0.25 m, 0.3 rad kick at 8 s on the S-curve)', '',
              '| Controller | Peak lateral after kick (m) | Recovery (s) | int w^2 dt (10 s) |',
              '|---|---|---|---|']
    for r in results['disturbance']:
        lines.append('| %s | %s | %s | %.3f |' % (r['controller'], r['peak_lateral_after_kick_m'],
                                                 r['recovery_s'] if r['recovery_s'] is not None else '**none**',
                                                 r['integral_w_sq']))
    env = results['envelope']
    lines += ['', '## Stability envelope (straight path, settle within %.0f s)' % env['horizon_s'], '',
              'Initial lateral offsets %s m x heading offsets %s deg.' % (env['laterals_m'], env['headings_deg']),
              '', '| Controller | Converged | Median settle (s) | Not converged (lateral m, heading deg) |',
              '|---|---|---|---|']
    for r in env['rows']:
        failed = ['(%g, %g)' % (g['lateral_m'], g['heading_deg']) for g in r['grid'] if g['settle_s'] is None]
        lines.append('| %s | %d/%d | %s | %s |' % (r['controller'], r['converged'], r['cells'],
                                                   r['median_settle_s'], ', '.join(failed) or '-'))
    lines += ['', '## Saturation recovery (0.8 m lateral start, |w| <= 0.3 rad/s)', '',
              '| Controller | Overshoot (m) | Settle (s) | Saturated steps | int w^2 dt |', '|---|---|---|---|---|']
    for r in results['saturation']:
        lines.append('| %s | %.3f | %s | %d | %.3f |' % (r['controller'], r['overshoot_m'],
                                                         r['settle_s'] if r['settle_s'] is not None else '**none**',
                                                         r['saturated_steps'], r['integral_w_sq']))
    lines += ['', '## Compute time against the %.0f ms period' % (results['deadline_s'] * 1e3), '',
              '| Controller | Steps | p50 (ms) | p95 (ms) | p99 (ms) | Max (ms) | Misses |',
              '|---|---|---|---|---|---|---|']
    for r in results['deadline']:
        lines.append('| %s | %d | %.3f | %.3f | %.3f | %.2f | %d (%.3f%%) |' % (
            r['controller'], r['steps'], r['p50_ms'], r['p95_ms'], r['p99_ms'], r['max_ms'], r['misses'],
            100.0 * r['miss_rate']))
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', required=True)
    parser.add_argument('--quick', action='store_true', help='coarse envelope grid')
    args = parser.parse_args()
    import scipy
    limits = Limits()
    laterals = [-1.0, -0.5, 0.0, 0.5, 1.0] if args.quick else [-1.0, -0.5, -0.25, 0.0, 0.25, 0.5, 1.0]
    headings = [-120, -60, 0, 60, 120] if args.quick else [-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150]
    load_start = os.getloadavg()
    tracking, compute_all = tracking_tests(limits)
    results = {
        'revision': subprocess.run(['git', '-C', HERE, 'rev-parse', 'HEAD'], capture_output=True,
                                   text=True).stdout.strip(),
        'host': '%s %s' % (platform.node(), platform.machine()), 'python': platform.python_version(),
        'numpy': np.__version__, 'scipy': scipy.__version__, 'control_period_s': DT,
        'deadline_s': 0.05,
        'plant': {'model': 'unicycle', 'tau_s': 0.1, 'a_max': limits.a_max, 'alpha_max': limits.alpha_max,
                  'v_min': limits.v_min, 'v_max': limits.v_max, 'w_max': limits.w_max},
        'settle_criterion': {'lateral_m': SETTLE_LATERAL, 'heading_rad': SETTLE_HEADING, 'hold_s': 1.0},
        'tracking': tracking,
    }
    results['disturbance'] = disturbance_tests(limits, compute_all)
    results['envelope'] = {'horizon_s': 20.0, 'laterals_m': laterals, 'headings_deg': headings,
                           'rows': envelope_tests(limits, compute_all, laterals, headings)}
    results['saturation'] = saturation_tests(compute_all)
    results['deadline'] = deadline_summary(compute_all, 0.05)
    results['host_load_average'] = {'start': [round(x, 2) for x in load_start],
                                    'end': [round(x, 2) for x in os.getloadavg()],
                                    'cpus': os.cpu_count()}
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'results.json'), 'w') as handle:
        json.dump(results, handle, indent=1)
    with open(os.path.join(args.out, 'report.md'), 'w') as handle:
        handle.write(markdown(results))
    print(markdown(results))


if __name__ == '__main__':
    main()
