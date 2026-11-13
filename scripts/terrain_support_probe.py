#!/usr/bin/env python3
"""Support height of a generated terrain on each simulator, through the real launch.

For every simulator it launches a small robot (default ``bumperbot``,
Display mode) once on flat ``nav_empty`` and then on the terrain at chosen
near-flat probe points, and reads the settled body height from
``/odom/ground_truth``. Measured support = terrain height - flat height, so
the robot's own root offset cancels. Expected support is the generated
(quantised) terrain at that point. One simulation at a time; source the
workspace first. Prints and writes a JSON report; exit 0 when every point
on every simulator is within ``--tolerance``.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

WORKSPACE = Path(__file__).resolve().parents[1]


def probe_points(manifest, heights, count):
    """The spawn pad plus near-flat points far from it and from each other."""
    inputs = manifest['inputs']
    n, size = inputs['samples'], inputs['size_m']
    cell = size / n
    grid_y, grid_x = np.gradient(heights, cell)
    slope = np.degrees(np.arctan(np.hypot(grid_x, grid_y)))
    # Flat within ~0.5 m around the point, so the body settles without rolling.
    from scipy.ndimage import maximum_filter
    local = maximum_filter(slope, size=max(3, int(math.ceil(1. / cell)) | 1))
    xs = (np.arange(n) + .5) * cell - size / 2.
    ys = size / 2. - (np.arange(n) + .5) * cell
    points = [(inputs['spawn_x'], inputs['spawn_y'])]
    order = np.argsort(-heights, axis=None)          # high ground first: a real elevation change
    for index in order:
        if len(points) >= count:
            break
        row, column = np.unravel_index(index, heights.shape)
        x, y = float(xs[column]), float(ys[row])
        if (local[row, column] < 3. and max(abs(x), abs(y)) < .35 * size
                and all(math.hypot(x - px, y - py) > .2 * size for px, py in points)):
            points.append((x, y))
    for index in order[::-1]:                         # then low ground
        if len(points) >= count + 1:
            break
        row, column = np.unravel_index(index, heights.shape)
        x, y = float(xs[column]), float(ys[row])
        if (local[row, column] < 3. and max(abs(x), abs(y)) < .35 * size
                and all(math.hypot(x - px, y - py) > .2 * size for px, py in points)):
            points.append((x, y))
    return points


def expected_height(manifest, heights, x, y):
    """Bilinear height of the cell-centred quantised grid (row 0 = +y)."""
    inputs = manifest['inputs']
    n, size = inputs['samples'], inputs['size_m']
    cell = size / n
    column = (x + size / 2.) / cell - .5
    row = (size / 2. - y) / cell - .5
    c0, r0 = int(np.clip(math.floor(column), 0, n - 2)), int(np.clip(math.floor(row), 0, n - 2))
    fc, fr = column - c0, row - r0
    return float((1 - fr) * ((1 - fc) * heights[r0, c0] + fc * heights[r0, c0 + 1])
                 + fr * ((1 - fc) * heights[r0 + 1, c0] + fc * heights[r0 + 1, c0 + 1]))


def real_world_points(manifest, folder, roofs=2):
    """Imported area: the spawn, two open flat ground points clear of buildings and trees, and
    the largest roofs (a robot resting at roof height proves the building collisions)."""
    from robot_lab_utils.real_world_import import build as build_real, choose_spawn
    request = dict(manifest['real_world']['request'], id=manifest['id'])
    _, _, heights, _ = build_real(request, folder=folder, progress=lambda _message: None)
    inputs = manifest['inputs']
    text = (folder / 'maps' / 'map.pgm').read_bytes()
    header = text.split(b'\n', 3)
    columns, rows = (int(v) for v in header[1].split())
    grid = np.frombuffer(header[3], dtype=np.uint8).reshape(rows, columns)
    size, resolution = inputs['size_m'], inputs['occupancy_resolution_m']
    ground = [(inputs['spawn_x'], inputs['spawn_y'])]
    for prefer in ((size / 4, -size / 4), (-size / 4, size / 4)):
        ground.append(choose_spawn(grid, size, resolution, clearance=2.5, prefer=prefer))
    points = [dict(x=round(x, 3), y=round(y, 3), kind='ground', expected_m=round(expected_height(manifest, heights, x, y), 4))
              for x, y in ground]
    for roof in sorted(manifest['real_world']['roofs'], key=lambda r: -r['area_m2'])[:roofs]:
        points.append(dict(x=roof['x'], y=roof['y'], kind='roof', expected_m=round(roof['top_m'], 4)))
    return points


def settled_height(log_path, args, extra, timeout):
    """Launch, wait for a still body, return (z, details); always stops the launch."""
    command = ['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py',
               'robot_model:=' + args.robot, 'mode:=display', 'gui:=false', 'start_rviz:=false', *extra]
    with open(log_path, 'w') as log:
        launch = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        samples = []
        node = rclpy.create_node('terrain_support_probe_%d' % os.getpid())
        node.create_subscription(Odometry, '/odom/ground_truth', lambda m: samples.append(
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, m.pose.pose.position.x,
             m.pose.pose.position.y, m.pose.pose.position.z)), qos_profile_sensor_data)
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if launch.poll() is not None:
                    raise RuntimeError('launch exited with %s' % launch.returncode)
                if len(samples) < 20 or samples[-1][0] - samples[0][0] < args.settle_sim_s:
                    continue
                window = [s for s in samples if s[0] >= samples[-1][0] - 1.]
                z = [s[3] for s in window]
                if len(window) >= 5 and max(z) - min(z) < .002:
                    last = samples[-1]
                    return float(np.mean(z)), dict(x=last[1], y=last[2], sim_s=last[0] - samples[0][0],
                                                    z_spread_m=max(z) - min(z), samples=len(samples))
            raise TimeoutError('no settled ground truth within %.0f s (%d samples)' % (timeout, len(samples)))
        finally:
            node.destroy_node()
    finally:
        try:
            os.killpg(launch.pid, signal.SIGINT)
            launch.wait(timeout=40)
        except Exception:
            try:
                os.killpg(launch.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(3.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--terrain', required=True, help='terrain id under $ROBOT_LAB_RUNTIME_ROOT/terrain')
    parser.add_argument('--robot', default='bumperbot')
    parser.add_argument('--simulators', default='gazebo,mujoco,pybullet,isaac')
    parser.add_argument('--points', type=int, default=2, help='probe points besides the spawn pad')
    parser.add_argument('--tolerance', type=float, default=.03)
    parser.add_argument('--settle-sim-s', type=float, default=4.)
    parser.add_argument('--timeout', type=float, default=420.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(WORKSPACE / 'src' / 'robot_lab_utils'))
    from robot_lab_utils.terrain_generator import build, terrain_root
    folder = terrain_root() / args.terrain
    manifest = json.loads((folder / 'manifest.json').read_text())
    if 'real_world' in manifest:
        probe = real_world_points(manifest, folder)
    else:
        _, _, heights = build(manifest['inputs'])
        probe = [dict(x=round(x, 3), y=round(y, 3), kind='ground',
                      expected_m=round(expected_height(manifest, heights, x, y), 4))
                 for x, y in probe_points(manifest, heights, args.points)]
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(terrain=args.terrain, manifest_files=manifest['files'], robot=args.robot,
                  tolerance_m=args.tolerance, method='settled /odom/ground_truth z on terrain minus on nav_empty',
                  points=probe, simulators={}, passed=False)
    import rclpy
    rclpy.init()
    all_ok = True
    for simulator in args.simulators.split(','):
        result = dict(points=[])
        try:
            flat, details = settled_height(args.output / ('%s-flat.log' % simulator), args,
                                           ['simulator:=' + simulator, 'map_name:=nav_empty'], args.timeout)
            result['flat_z'] = round(flat, 4)
            result['flat'] = details
            for index, point in enumerate(report['points']):
                extra = ['simulator:=' + simulator, 'map_name:=' + args.terrain,
                         'spawn_x:=%.4f' % point['x'], 'spawn_y:=%.4f' % point['y'],
                         'spawn_z:=%.4f' % (point['expected_m'] + .1)]
                try:
                    z, details = settled_height(args.output / ('%s-%d.log' % (simulator, index)), args, extra,
                                                args.timeout)
                    measured = z - flat
                    error = measured - point['expected_m']
                    ok = abs(error) <= args.tolerance and math.hypot(details['x'] - point['x'],
                                                                      details['y'] - point['y']) < .3
                    result['points'].append(dict(point, measured_m=round(measured, 4), error_m=round(error, 4),
                                                 passed=ok, **{k: round(v, 4) if isinstance(v, float) else v
                                                               for k, v in details.items()}))
                except Exception as exc:
                    ok = False
                    result['points'].append(dict(point, passed=False, error=str(exc)))
                all_ok &= ok
                print(simulator, result['points'][-1], flush=True)
        except Exception as exc:
            result['error'] = str(exc)
            all_ok = False
            print(simulator, 'flat reference failed:', exc, flush=True)
        result['passed'] = bool(result['points']) and all(p['passed'] for p in result['points'])
        report['simulators'][simulator] = result
        (args.output / 'report.json').write_text(json.dumps(report, indent=1) + '\n')
    rclpy.shutdown()
    report['passed'] = bool(all_ok)
    (args.output / 'report.json').write_text(json.dumps(report, indent=1) + '\n')
    print(json.dumps(dict(passed=report['passed'], simulators={k: v['passed'] for k, v in report['simulators'].items()})))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
