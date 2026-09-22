#!/usr/bin/env python3
"""Bounded push probe of the real URDF MuJoCo backend and ROS standing node.

Source the ROS overlay; run with the workspace venv Python and an isolated
ROS_DOMAIN_ID. Injection is test-only: the controller sees ROS sensors only.
This instantiates the backend directly, not the common launch/GUI workflow.
"""
import argparse
import csv
import json
import hashlib
import inspect
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory as share
from robot_lab_mujoco.mujoco_spawner import MuJoCoSpawner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--axis', choices=['x', 'y'], default='y')
    parser.add_argument('--force', type=float, default=5.0)
    args = parser.parse_args()
    if not math.isfinite(args.force) or abs(args.force) > 10:
        parser.error('force must be finite and bounded to +/-10 N')
    args.output.mkdir(parents=True, exist_ok=True)
    robot = Path(share('robot_lab_robots')) / 'berkeley_humanoid_lite'
    params = {
        'model': str(robot / 'xacro/bhl_sim.xacro'),
        'effort_controller_config': str(robot / 'config/bhl_controllers.yaml'),
        'world_xml': str(Path(share('robot_lab_maps')) / 'mjcf/nav_empty.xml'),
        'robot_name': 'bhl', 'spawn_z': '-0.038', 'gui': 'false',
        'physics_rate': '250.0', 'publish_rate': '250.0',
        'use_sim_time': 'true', 'camera_rate': '0.0',
    }
    ros_args = ['--ros-args']
    for key, value in params.items():
        ros_args += ['-p', f'{key}:={value}']
    from robot_lab_adapter import bhl_balance, humanoid_standing_controller
    assets = [Path(__file__), Path(inspect.getfile(MuJoCoSpawner)),
              Path(bhl_balance.__file__), Path(humanoid_standing_controller.__file__),
              Path(params['model']), Path(params['effort_controller_config']),
              Path(params['world_xml'])]
    manifest = {
        'command': sys.argv, 'backend_parameters': params,
        'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'git_status': subprocess.check_output(['git', 'status', '--short'], text=True),
        'sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in assets},
    }
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    records = []
    axis = 0 if args.axis == 'x' else 1

    class PushSpawner(MuJoCoSpawner):
        def _step_physics(self):
            # Called under the backend physics lock. Force acts in WORLD axes
            # at the root body's COM, for exactly 0.2 simulated seconds.
            t = float(self._data.time)
            force = args.force if 3.0 - 1e-9 <= t < 3.2 - 1e-9 else 0.0
            self._data.xfrc_applied[self._body_id, :] = 0.0
            self._data.xfrc_applied[self._body_id, axis] = force
            super()._step_physics()
            q = self._data.xquat[self._body_id]
            tilt = math.acos(np.clip(1 - 2 * (q[1]**2 + q[2]**2), -1, 1))
            records.append({
                't': float(self._data.time), 'dt': float(self._data.time) - t,
                'force_n': force, 'tilt_rad': tilt,
                'x': self._bpos[0], 'y': self._bpos[1],
                'speed_m_s': float(np.linalg.norm(self._blin[:2])),
                'effort_nm': float(np.max(np.abs(self._data.qfrc_actuator))),
            })

    result = {'passed': False, 'axis': args.axis, 'force_n': args.force,
              'criteria': {'duration_s': 8, 'max_tilt_rad': .2,
                           'final_second_max_tilt_rad': .02,
                           'final_second_max_speed_m_s': .05,
                           'max_effort_nm': 20.0001}}
    node = None
    child = None
    rclpy.init(args=ros_args)
    try:
        with (args.output / 'controller.log').open('w') as log:
            child = subprocess.Popen([
                sys.executable, '-m', 'robot_lab_adapter.humanoid_standing_controller',
                '--ros-args', '-p', 'use_sim_time:=true'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(.5)
        node = PushSpawner()
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError('standing controller exited')
            rclpy.spin_once(node, timeout_sec=.01)
            if records and records[-1]['t'] >= 8:
                break
        if node._model_source != 'urdf' or len(node._effort_actuators) != 22:
            raise RuntimeError('expected the imported 22-actuator BHL plant')
        result['plant'] = {
            'source': node._model_source, 'root_body_id': int(node._body_id),
            'mass_kg': float(np.sum(node._model.body_mass)),
            'physics_timestep_s': float(node._model.opt.timestep),
            'actuator_count': len(node._effort_actuators),
        }
        # Stop the physics thread before taking the final evidence snapshot.
        node.destroy_node()
        node = None
        if not records or records[-1]['t'] < 8:
            raise RuntimeError('simulation did not finish its budget')
        final = [r for r in records if 7 <= r['t'] <= 8]
        result.update(
            samples=len(records), duration_s=records[-1]['t'],
            impulse_ns=sum(r['force_n'] * r['dt'] for r in records),
            max_tilt_rad=max(r['tilt_rad'] for r in records),
            final_max_tilt_rad=max(r['tilt_rad'] for r in final),
            final_max_speed_m_s=max(r['speed_m_s'] for r in final),
            max_effort_nm=max(r['effort_nm'] for r in records),
            displacement_m=math.hypot(records[-1]['x']-records[0]['x'],
                                      records[-1]['y']-records[0]['y']),
        )
        result['passed'] = bool(
            all(math.isfinite(v) for r in records for v in r.values())
            and abs(result['impulse_ns'] - args.force * .2) < 1e-6
            and result['max_tilt_rad'] < .2
            and result['final_max_tilt_rad'] < .02
            and result['final_max_speed_m_s'] < .05
            and result['max_effort_nm'] <= 20.0001)
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        if node is not None:
            node.destroy_node()
        if child is not None:
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=5)
            result['controller_exit_code'] = child.returncode
            result['passed'] = result['passed'] and child.returncode == 0
        rclpy.shutdown()
        (args.output / 'report.json').write_text(json.dumps(result, indent=2) + '\n')
        with (args.output / 'trace.csv').open('w') as stream:
            if records:
                writer = csv.DictWriter(stream, fieldnames=list(records[0]))
                writer.writeheader()
                writer.writerows(records)
        print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
