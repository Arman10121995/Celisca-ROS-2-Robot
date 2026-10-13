#!/usr/bin/env python3
"""R7.5 benchmark: sensor fusion filters on synthetic sensors with known truth.

Attitude (IMU at 100 Hz): a body that rolls (+-17 deg), pitches (+-11 deg)
and turns, with gyro bias (0.010, -0.008, 0.012) rad/s and white noise
0.003 rad/s, accelerometer noise 0.05 m/s^2, horizontal accelerations of
0.5 m/s^2 throughout and 2 m/s^2 from 60 to 80 s.  Every filter starts
20 deg (roll) and 17 deg (pitch) off.  Metrics: roll/pitch error, settling,
error during the 2 m/s^2 phase, yaw drift, compute per update.

Planar pose (50 Hz): a 0.5 m/s drive with stops (10 s in every 60 s),
wheel odometry with +2% speed and -3% yaw-rate scale errors, noise, and two
slip episodes (+0.1 rad/s for 10 s and 5 s); gyro bias 0.01 rad/s; GNSS
at 1 Hz with 0.5 m noise.  Wheel-only dead reckoning is the baseline.

Same seeds for every filter.  This measures the algorithms on modelled
sensors, not a simulator or a robot.

  python3 scripts/r7_5_fusion_benchmark.py --out <dir>
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

from robot_lab_algorithms.fusion_filters import (  # noqa: E402
    ATTITUDE_FILTERS, GRAVITY, WheelImuEkf, WheelImuGnssUkf, q_from_euler, q_matrix, q_multiply,
    q_to_euler, wrap)

GYRO_BIAS = np.array([0.010, -0.008, 0.012])


def attitude_truth(t):
    return q_from_euler(0.3 * math.sin(0.5 * t), 0.2 * math.sin(0.37 * t + 1.0), 0.2 * t + 0.3 * math.sin(0.1 * t))


def attitude_data(seed, duration=120.0, dt=0.01):
    rng = np.random.default_rng(seed)
    n = int(duration / dt)
    qs = [attitude_truth(k * dt) for k in range(n + 1)]
    samples = []
    for k in range(n):
        q0, q1 = qs[k], qs[k + 1]
        dq = q_multiply(np.array([q0[0], -q0[1], -q0[2], -q0[3]]), q1)
        if dq[0] < 0:
            dq = -dq
        angle = 2.0 * math.atan2(np.linalg.norm(dq[1:]), dq[0])
        axis = dq[1:] / max(np.linalg.norm(dq[1:]), 1e-12)
        omega = axis * angle / dt
        t = (k + 1) * dt
        amplitude = 2.0 if 60.0 <= t < 80.0 else 0.5
        accel_world = amplitude * np.array([math.sin(0.8 * t), math.cos(1.1 * t), 0.0])
        specific_force = q_matrix(q1).T @ (accel_world + np.array([0.0, 0.0, GRAVITY]))
        gyro = omega + GYRO_BIAS + rng.normal(0, 0.003, 3)
        accel = specific_force + rng.normal(0, 0.05, 3)
        samples.append((t, gyro, accel, q1))
    return samples, dt


# One gain per filter, chosen on the tuning seed (lowest tilt RMS over the
# whole run after 10 s, disturbance included) before the evaluation seeds.
GAIN_GRID = {
    'complementary': ('tau', [0.25, 0.5, 1.0, 2.0, 4.0, 8.0]),
    'mahony': ('kp', [0.05, 0.1, 0.25, 0.5, 1.0, 2.0]),
    'madgwick': ('beta', [0.005, 0.01, 0.02, 0.05, 0.1, 0.2]),
    'eskf': ('linear_accel_std', [0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0]),
}


def run_attitude(name, samples, dt, gain=None):
    q0 = q_from_euler(0.3 * math.sin(0.0) + math.radians(20), 0.2 * math.sin(1.0) - math.radians(17), 0.0)
    parameters = {GAIN_GRID[name][0]: gain} if gain is not None else {}
    flt = ATTITUDE_FILTERS[name](q0=q0, **parameters)
    rows, compute = [], []
    yaw0 = None
    for t, gyro, accel, q in samples:
        start = time.perf_counter()
        flt.update(gyro, accel, dt)
        compute.append(time.perf_counter() - start)
        r, p, y = q_to_euler(flt.q)
        rt, pt, yt = q_to_euler(q)
        if yaw0 is None:
            yaw0 = wrap(y - yt)
        rows.append((t, math.degrees(abs(wrap(r - rt))), math.degrees(abs(wrap(p - pt))),
                     math.degrees(wrap(y - yt - yaw0))))
    rows = np.array(rows)
    tilt = np.maximum(rows[:, 1], rows[:, 2])
    settle = None
    for i in range(len(rows)):
        if np.all(tilt[i:i + 100] < 2.0) and len(tilt[i:i + 100]) == 100:
            settle = round(float(rows[i, 0]), 2)
            break
    steady = (rows[:, 0] >= 10) & ((rows[:, 0] < 60) | (rows[:, 0] >= 85))
    disturbed = (rows[:, 0] >= 60) & (rows[:, 0] < 80)
    ms = np.array(compute) * 1e3
    after = rows[:, 0] >= 10
    return {
        'filter': name,
        'gain': {GAIN_GRID[name][0]: gain} if gain is not None else {},
        'overall_rms_deg': round(float(np.sqrt(np.mean(np.concatenate([rows[after, 1], rows[after, 2]]) ** 2))), 3),
        'settle_to_2deg_s': settle,
        'steady_rms_deg': {'roll': round(float(np.sqrt(np.mean(rows[steady, 1] ** 2))), 3),
                           'pitch': round(float(np.sqrt(np.mean(rows[steady, 2] ** 2))), 3)},
        'steady_max_deg': round(float(tilt[steady].max()), 3),
        'disturbed_rms_deg': round(float(np.sqrt(np.mean(np.concatenate([rows[disturbed, 1], rows[disturbed, 2]]) ** 2))), 3),
        'disturbed_max_deg': round(float(tilt[disturbed].max()), 3),
        'yaw_drift_deg_per_min': round(float((rows[-1, 3] - rows[int(10 / dt), 3]) / ((rows[-1, 0] - 10) / 60.0)), 3),
        'compute_ms': {'p50': round(float(np.percentile(ms, 50)), 4), 'p99': round(float(np.percentile(ms, 99)), 4)},
    }


def pose_data(seed, duration=300.0, dt=0.02):
    rng = np.random.default_rng(seed)
    x = y = th = 0.0
    out = []
    for k in range(int(duration / dt)):
        t = k * dt
        v = 0.5 if (t % 60.0) < 50.0 else 0.0
        w = 0.3 * math.sin(0.2 * t) if v > 0 else 0.0
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th = wrap(th + w * dt)
        slip = 0.1 if (100 < t < 110 or 200 < t < 205) and v > 0 else 0.0
        wheel_v = v * 1.02 + (rng.normal(0, 0.01) if v > 0 else 0.0)
        wheel_w = w * 0.97 + slip + (rng.normal(0, 0.02) if v > 0 else 0.0)
        gyro_z = w + 0.01 + rng.normal(0, 0.003)
        gnss = (x + rng.normal(0, 0.5), y + rng.normal(0, 0.5)) if k % int(1.0 / dt) == 0 else None
        out.append((t, x, y, th, wheel_v, wheel_w, gyro_z, gnss))
    return out, dt


def run_pose(name, data, dt):
    if name == 'dead_reckoning':
        state = [0.0, 0.0, 0.0]
    else:
        flt = {'wheel_imu_ekf': WheelImuEkf, 'ekf_gnss': WheelImuEkf, 'ukf_gnss': WheelImuGnssUkf}[name]()
    errors, heading, compute = [], [], []
    for t, x, y, th, wv, ww, gz, gnss in data:
        start = time.perf_counter()
        if name == 'dead_reckoning':
            state[0] += wv * math.cos(state[2]) * dt
            state[1] += wv * math.sin(state[2]) * dt
            state[2] = wrap(state[2] + ww * dt)
            ex, ey, eth = state
        else:
            flt.predict(gz, dt)
            flt.update_wheels(wv, ww)
            if gnss is not None and name != 'wheel_imu_ekf':
                flt.update_gnss(*gnss)
            ex, ey, eth = flt.state[:3]
        compute.append(time.perf_counter() - start)
        errors.append(math.hypot(ex - x, ey - y))
        heading.append(abs(wrap(eth - th)))
    e = np.array(errors)
    ms = np.array(compute) * 1e3
    result = {'filter': name, 'position_rmse_m': round(float(np.sqrt(np.mean(e ** 2))), 3),
              'final_error_m': round(float(e[-1]), 3), 'max_error_m': round(float(e.max()), 3),
              'heading_rmse_deg': round(math.degrees(float(np.sqrt(np.mean(np.array(heading) ** 2)))), 3),
              'compute_ms': {'p50': round(float(np.percentile(ms, 50)), 4), 'p99': round(float(np.percentile(ms, 99)), 4)}}
    if name != 'dead_reckoning':
        result['gyro_bias_estimate'] = round(float(flt.state[4]), 5)
    return result


def markdown(results):
    lines = ['# R7.5 sensor fusion benchmark (modelled sensors)', '',
             'Revision `%s`, %s, Python %s, NumPy %s; seeds %s.' % (
                 results['revision'][:7], results['host'], results['python'], results['numpy'], results['seeds']), '',
             '## Attitude (mean over evaluation seeds)', '',
             'Gains chosen on seed %d: %s.' % (results['tune_seed'], ', '.join(
                 '%s %s=%g' % (k, results['attitude_tuning'][k]['parameter'], v)
                 for k, v in results['attitude_gains'].items())), '',
             '| Filter | Overall RMS after 10 s (deg) | Settle to 2 deg (s) | Steady RMS roll / pitch (deg) | Steady max (deg) | '
             'RMS / max under 2 m/s^2 (deg) | Yaw drift (deg/min) | Compute p50 / p99 (ms) |',
             '|---|---|---|---|---|---|---|---|']
    for r in results['attitude']:
        lines.append('| %s | %.2f | %s | %.2f / %.2f | %.2f | %.2f / %.2f | %.2f | %.3f / %.3f |' % (
            r['filter'], r['overall_rms_deg'], r['settle_to_2deg_s'], r['steady_rms_deg']['roll'], r['steady_rms_deg']['pitch'],
            r['steady_max_deg'], r['disturbed_rms_deg'], r['disturbed_max_deg'], r['yaw_drift_deg_per_min'],
            r['compute_ms']['p50'], r['compute_ms']['p99']))
    lines += ['', '## Planar pose over 300 s (mean over seeds)', '',
              '| Method | Position RMSE (m) | Final error (m) | Max error (m) | Heading RMSE (deg) | '
              'Gyro bias estimate (true 0.010) | Compute p50 / p99 (ms) |', '|---|---|---|---|---|---|---|']
    for r in results['pose']:
        lines.append('| %s | %.2f | %.2f | %.2f | %.2f | %s | %.3f / %.3f |' % (
            r['filter'], r['position_rmse_m'], r['final_error_m'], r['max_error_m'], r['heading_rmse_deg'],
            r.get('gyro_bias_estimate', '-'), r['compute_ms']['p50'], r['compute_ms']['p99']))
    return '\n'.join(lines) + '\n'


def mean_rows(rows):
    """Average numeric leaves over seeds (settle: None if any seed did not settle)."""
    out = {}
    for key, value in rows[0].items():
        if isinstance(value, dict):
            out[key] = mean_rows([r[key] for r in rows])
        elif isinstance(value, (int, float)) and value is not None:
            values = [r[key] for r in rows]
            out[key] = None if any(v is None for v in values) else round(float(np.mean(values)), 4)
        else:
            out[key] = value if all(r[key] == value for r in rows) else [r[key] for r in rows]
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', required=True)
    parser.add_argument('--seeds', default='1,2,3')
    parser.add_argument('--tune-seed', type=int, default=0)
    args = parser.parse_args()
    seeds = [int(s) for s in args.seeds.split(',')]
    samples, dt = attitude_data(args.tune_seed)
    tuning, gains = {}, {}
    for name, (parameter, values) in GAIN_GRID.items():
        scores = {value: run_attitude(name, samples, dt, value)['overall_rms_deg'] for value in values}
        tuning[name] = {'parameter': parameter, 'overall_rms_deg': scores}
        gains[name] = min(scores, key=scores.get)
    attitude = {name: [] for name in ATTITUDE_FILTERS}
    pose = {name: [] for name in ('dead_reckoning', 'wheel_imu_ekf', 'ekf_gnss', 'ukf_gnss')}
    for seed in seeds:
        samples, dt = attitude_data(seed)
        for name in attitude:
            attitude[name].append(run_attitude(name, samples, dt, gains[name]))
        data, dt = pose_data(seed)
        for name in pose:
            pose[name].append(run_pose(name, data, dt))
    results = {
        'revision': subprocess.run(['git', '-C', HERE, 'rev-parse', 'HEAD'], capture_output=True,
                                   text=True).stdout.strip(),
        'host': '%s %s' % (platform.node(), platform.machine()), 'python': platform.python_version(),
        'numpy': np.__version__, 'seeds': seeds, 'tune_seed': args.tune_seed,
        'attitude_tuning': tuning, 'attitude_gains': gains,
        'attitude': [mean_rows(rows) for rows in attitude.values()],
        'pose': [mean_rows(rows) for rows in pose.values()],
        'per_seed': {'attitude': attitude, 'pose': pose},
    }
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'results.json'), 'w') as handle:
        json.dump(results, handle, indent=1)
    with open(os.path.join(args.out, 'report.md'), 'w') as handle:
        handle.write(markdown(results))
    print(markdown(results))


if __name__ == '__main__':
    main()
