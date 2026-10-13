#!/usr/bin/env python3
"""R7.5 on simulator data: replay recorded IMU and wheel odometry through the fusion filters.

Each bag (``EXTRA_BAG_TOPICS="/imu/out /robot_lab_controller/odom"`` with
``scripts/slam_mapping_check.sh`` in Gazebo) holds a robot driving a loop:
the simulator IMU (/imu/out: gyro and accelerometer with the description's
noise), the drive controller's wheel odometry (/robot_lab_controller/odom)
and simulator truth (/odom/ground_truth).

* Attitude: complementary, Mahony, Madgwick and the attitude ESKF
  (robot_lab_algorithms.fusion_filters, at their node defaults) integrate the
  IMU from the true start attitude; roll and pitch are scored against truth.
  On a wheeled robot on flat ground this is a check for drift and bias; a
  walking trunk rolls and pitches, which makes it a real test.
* Pose (bags with wheel odometry): the wheel+gyro EKF and UKF and wheel dead
  reckoning start at the true pose; the EKF/UKF predict with the gyro and
  correct with the wheel speed and yaw rate, as the nodes do.

  python3 scripts/r7_5_live_replay.py --out <dir> <run.bag> ...
"""
import argparse
import bisect
import glob
import json
import math
import os
import platform
import sqlite3
import subprocess
import sys

for _variable in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ.setdefault(_variable, '1')

import numpy as np  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'robot_lab_algorithms'))

from robot_lab_algorithms.fusion_filters import (  # noqa: E402
    AttitudeEskf, ComplementaryAttitude, MadgwickAhrs, MahonyAhrs, WheelImuEkf, WheelImuGnssUkf,
    q_to_euler, wrap)

# The sensor_fusion nodes' defaults (robot_lab_algorithms.sensor_fusion.AttitudeNode).
ATTITUDE = {
    'complementary': lambda q0: ComplementaryAttitude(tau=4.0, q0=q0),
    'mahony': lambda q0: MahonyAhrs(kp=0.25, q0=q0),
    'madgwick': lambda q0: MadgwickAhrs(beta=0.02, q0=q0),
    'eskf': lambda q0: AttitudeEskf(linear_accel_std=2.0, q0=q0),
}
WHEEL_ODOM = '/robot_lab_controller/odom'


def stamp(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read(path):
    files = glob.glob(os.path.join(path, '*.db3')) if os.path.isdir(path) else [path]
    db = sqlite3.connect(files[0])
    topics = {tid: (name, get_message(kind)) for tid, name, kind in db.execute('select id, name, type from topics')}
    wanted = {'/imu/out', WHEEL_ODOM, '/odom/ground_truth'}
    data = {name: [] for name in wanted}
    for tid, raw in db.execute('select topic_id, data from messages order by timestamp'):
        name, cls = topics[tid]
        if name in wanted:
            data[name].append(deserialize_message(raw, cls))
    return data


def truth_track(messages):
    t = np.array([stamp(m.header) for m in messages])
    order = np.argsort(t)
    rows = []
    for i in order:
        p, q = messages[i].pose.pose.position, messages[i].pose.pose.orientation
        roll, pitch, yaw = q_to_euler((q.w, q.x, q.y, q.z))
        rows.append((t[i], p.x, p.y, roll, pitch, yaw, (q.w, q.x, q.y, q.z)))
    return rows


def at(track, t):
    times = [r[0] for r in track]
    i = bisect.bisect_left(times, t)
    if i == 0 or i >= len(track) or track[i][0] - track[i - 1][0] > 0.2:
        return None
    return track[i] if track[i][0] - t < t - track[i - 1][0] else track[i - 1]


def attitude(data, track, skip=5.0):
    imu = sorted(data['/imu/out'], key=lambda m: stamp(m.header))
    first = at(track, stamp(imu[0].header))
    q0 = first[6] if first else None
    results = {}
    for name, make in ATTITUDE.items():
        flt = make(q0)
        last, errors, roll_err, pitch_err = None, [], [], []
        for m in imu:
            t = stamp(m.header)
            g, a = m.angular_velocity, m.linear_acceleration
            flt.update((g.x, g.y, g.z), (a.x, a.y, a.z), 0.0 if last is None else t - last)
            last = t
            if t - stamp(imu[0].header) < skip:
                continue
            truth = at(track, t)
            if truth is None:
                continue
            roll, pitch, _ = flt.euler()
            roll_err.append(wrap(roll - truth[3]))
            pitch_err.append(wrap(pitch - truth[4]))
        tilt = np.hypot(roll_err, pitch_err)
        results[name] = {'samples': len(tilt),
                         'tilt_rms_deg': round(math.degrees(float(np.sqrt(np.mean(tilt ** 2)))), 3),
                         'tilt_max_deg': round(math.degrees(float(tilt.max())), 3),
                         'roll_bias_deg': round(math.degrees(float(np.mean(roll_err))), 3),
                         'pitch_bias_deg': round(math.degrees(float(np.mean(pitch_err))), 3)}
    truth_tilt = np.array([math.hypot(r[3], r[4]) for r in track])
    return results, {'truth_tilt_rms_deg': round(math.degrees(float(np.sqrt(np.mean(truth_tilt ** 2)))), 3),
                     'truth_tilt_max_deg': round(math.degrees(float(truth_tilt.max())), 3),
                     'imu_rate_hz': round(len(imu) / max(1e-6, stamp(imu[-1].header) - stamp(imu[0].header)), 1)}


def pose(data, track):
    wheels = sorted(data[WHEEL_ODOM], key=lambda m: stamp(m.header))
    if not wheels:
        return None
    imu = sorted(data['/imu/out'], key=lambda m: stamp(m.header))
    events = sorted([(stamp(m.header), 'w', m) for m in wheels] + [(stamp(m.header), 'i', m) for m in imu],
                    key=lambda e: e[0])
    start = at(track, events[0][0]) or track[0]
    x0 = (start[1], start[2], start[5])
    filters = {'wheel_imu_ekf': WheelImuEkf(x0=x0), 'wheel_imu_ukf': WheelImuGnssUkf(x0=x0)}
    dead = list(x0)
    last_imu = last_wheel = None
    speed = rate = 0.0
    errors = {name: [] for name in list(filters) + ['wheel_dead_reckoning']}
    for t, kind, m in events:
        if kind == 'i':
            if last_imu is not None:
                for flt in filters.values():
                    flt.predict(m.angular_velocity.z, t - last_imu)
            last_imu = t
        else:
            speed, rate = m.twist.twist.linear.x, m.twist.twist.angular.z
            if last_wheel is not None:
                dt = t - last_wheel
                dead[0] += speed * math.cos(dead[2]) * dt
                dead[1] += speed * math.sin(dead[2]) * dt
                dead[2] = wrap(dead[2] + rate * dt)
            last_wheel = t
            for flt in filters.values():
                flt.update_wheels(speed, rate)
            truth = at(track, t)
            if truth is None:
                continue
            for name, flt in filters.items():
                errors[name].append((math.hypot(flt.state[0] - truth[1], flt.state[1] - truth[2]),
                                     abs(wrap(flt.state[2] - truth[5]))))
            errors['wheel_dead_reckoning'].append((math.hypot(dead[0] - truth[1], dead[1] - truth[2]),
                                                   abs(wrap(dead[2] - truth[5]))))
    travelled = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(track, track[1:]))
    out = {'travelled_m': round(travelled, 2)}
    for name, rows in errors.items():
        e = np.array(rows)
        out[name] = {'position_rmse_m': round(float(np.sqrt(np.mean(e[:, 0] ** 2))), 3),
                     'final_error_m': round(float(e[-1, 0]), 3),
                     'heading_rmse_deg': round(math.degrees(float(np.sqrt(np.mean(e[:, 1] ** 2)))), 3)}
        if name in filters:
            out[name]['gyro_bias_estimate'] = round(float(filters[name].state[4]), 5)
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bags', nargs='+')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    runs = {}
    for bag in args.bags:
        data = read(bag)
        track = truth_track(data['/odom/ground_truth'])
        att, meta = attitude(data, track)
        runs[os.path.basename(os.path.normpath(bag))] = {'meta': meta, 'attitude': att, 'pose': pose(data, track)}
    results = {
        'revision': subprocess.run(['git', '-C', HERE, 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip(),
        'host': '%s %s' % (platform.node(), platform.machine()), 'runs': runs}
    lines = ['# R7.5 replay of simulator IMU and wheel odometry', '',
             'Revision `%s`. Attitude filters at their node defaults, started at the true attitude; '
             'tilt error = roll and pitch against truth after 5 s.' % results['revision'][:7], '']
    for run, r in runs.items():
        lines += ['## %s' % run, '',
                  'IMU %.0f Hz; true trunk tilt RMS %.2f deg, max %.2f deg.' % (
                      r['meta']['imu_rate_hz'], r['meta']['truth_tilt_rms_deg'], r['meta']['truth_tilt_max_deg']), '',
                  '| Attitude filter | Tilt RMS (deg) | Tilt max (deg) | Roll / pitch bias (deg) |', '|---|---|---|---|']
        for name, a in r['attitude'].items():
            lines.append('| %s | %.2f | %.2f | %.2f / %.2f |' % (
                name, a['tilt_rms_deg'], a['tilt_max_deg'], a['roll_bias_deg'], a['pitch_bias_deg']))
        if r['pose']:
            lines += ['', 'Pose over %.1f m:' % r['pose']['travelled_m'], '',
                      '| Pose filter | Position RMSE (m) | Final (m) | Heading RMSE (deg) |', '|---|---|---|---|']
            for name, p in r['pose'].items():
                if isinstance(p, dict):
                    lines.append('| %s | %.3f | %.3f | %.2f |' % (
                        name, p['position_rmse_m'], p['final_error_m'], p['heading_rmse_deg']))
        lines.append('')
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, 'results.json'), 'w') as handle:
        json.dump(results, handle, indent=1)
    with open(os.path.join(args.out, 'report.md'), 'w') as handle:
        handle.write('\n'.join(lines))
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
