#!/usr/bin/env python3
"""Bounded push probe of the real URDF MuJoCo backend and ROS standing node.

Two protocols:

* default (stance): an 8 simulated-second run with a bounded lateral push
  (3.0-3.2 s) while the standing node drives the 22 joints.
* ``--policy``: the walking policy is commanded 0.25 m/s from 1 s, commanded
  to stop at 6 s, and recorded for 5 more seconds (11 s total) - the same
  stop budget the reference qualification gives its stop phase - so the
  final-second settling checks apply after the stop transient has decayed.

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
    parser.add_argument('--policy', action='store_true', help='probe policy walk/stop instead of standing')
    parser.add_argument('--command-start', type=float, default=1.0,
                        help='simulated second to start the policy walk (default: 1)')
    parser.add_argument('--spawn-x', type=float, default=0.0)
    parser.add_argument('--spawn-y', type=float, default=0.0)
    args = parser.parse_args()
    if not math.isfinite(args.force) or abs(args.force) > 10:
        parser.error('force must be finite and bounded to +/-10 N')
    if args.policy and args.force != 0:
        parser.error('policy walk probe requires --force 0')
    if not math.isfinite(args.command_start) or args.command_start < 0:
        parser.error('--command-start must be finite and nonnegative')
    if not all(math.isfinite(v) for v in (args.spawn_x, args.spawn_y)):
        parser.error('spawn coordinates must be finite')
    args.output.mkdir(parents=True, exist_ok=True)
    robot = Path(share('robot_lab_robots')) / 'berkeley_humanoid_lite'
    params = {
        'model': str(robot / 'xacro/bhl_sim.xacro'),
        'effort_controller_config': str(robot / 'config/bhl_controllers.yaml'),
        'world_xml': str(Path(share('robot_lab_maps')) / 'mjcf/nav_empty.xml'),
        'robot_name': 'bhl', 'spawn_z': '-0.038', 'gui': 'false',
        'spawn_x': str(args.spawn_x), 'spawn_y': str(args.spawn_y),
        'physics_rate': '250.0', 'publish_rate': '250.0',
        'use_sim_time': 'true', 'camera_rate': '0.0',
    }
    ros_args = ['--ros-args']
    for key, value in params.items():
        ros_args += ['-p', f'{key}:={value}']
    from robot_lab_adapter import bhl_balance, humanoid_standing_controller, humanoid_policy_controller, bhl_policy
    assets = [Path(__file__), Path(inspect.getfile(MuJoCoSpawner)),
              Path(bhl_balance.__file__), Path(humanoid_standing_controller.__file__),
              Path(humanoid_policy_controller.__file__), Path(bhl_policy.__file__),
              Path(params['model']), Path(params['effort_controller_config']),
              Path(params['world_xml'])]
    manifest = {
        'command': sys.argv, 'backend_parameters': params,
        'revision': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'git_status': subprocess.check_output(['git', 'status', '--short'], text=True),
        'sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in assets},
    }
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    controller_module = ('humanoid_policy_controller' if args.policy
                         else 'humanoid_standing_controller')
    records = []
    axis = 0 if args.axis == 'x' else 1
    # Protocol: the standing push probe runs 8 simulated seconds (3.0-3.2 s
    # force). The policy probe drives 0.25 m/s from 1 s and commands stop at
    # 6 s, then records the stopping behaviour for a further 5 s - the same
    # stop budget the reference qualification gives its stop phase
    # (qualify_policy.PHASES), so the final-second settling check is applied
    # after the vehicle has actually had time to stop.
    walk_until = args.command_start + 5.0
    budget = args.command_start + 10.0 if args.policy else 8.0

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
              'criteria': {'duration_s': budget, 'max_tilt_rad': .2,
                           'final_second_max_tilt_rad': .02,
                           'final_second_max_speed_m_s': .05,
                           'max_effort_nm': 20.0001}}
    if args.policy:
        # Policy bars reuse the project's reference qualification values
        # (qualify_policy.assess_phase): walk >= 60% of the commanded distance
        # and <= 0.5 m lateral drift over its own phase, and a stopped body
        # within 0.05 m/s and 0.25 rad in the final second, drifting no more
        # than 0.05 m. This probe's walking window is shorter (5 s of 0.25 m/s
        # that *includes* the controller's settle ramp, i.e. 1.25 m commanded),
        # so the walk floor is set at 30% of the commanded distance just to
        # detect gait, while the reference's 60% tracking rule is applied where
        # it is meaningful: the settled 2 s before the stop command
        # (min_settled_walk_tracking), measured from travelled distance.
        result['criteria'].update(
            max_tilt_rad=.7, final_second_max_tilt_rad=.25,
            final_second_tilt_growth_rad=.02, final_second_pose_drift_m=.05,
            min_forward_progress_m=.375, min_settled_walk_tracking=.6,
            max_lateral_displacement_m=.25)
    result['controller'] = controller_module
    node = None
    child = None
    rclpy.init(args=ros_args)
    try:
        with (args.output / 'controller.log').open('w') as log:
            child = subprocess.Popen([
                sys.executable, '-m', 'robot_lab_adapter.' + controller_module,
                '--ros-args', '-p', 'use_sim_time:=true'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(.5)
        node = PushSpawner()
        from geometry_msgs.msg import Twist
        command_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        last_command_t = -1.0
        deadline = time.monotonic() + 3 * budget + 90
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError('standing controller exited')
            rclpy.spin_once(node, timeout_sec=.01)
            if args.policy and records:
                t = records[-1]['t']
                if t >= args.command_start and t - last_command_t >= .04:
                    cmd = Twist()
                    cmd.linear.x = .25 if t < walk_until else 0.0
                    command_pub.publish(cmd)
                    last_command_t = t
            if records and records[-1]['t'] >= budget:
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
        if not records or records[-1]['t'] < budget:
            raise RuntimeError('simulation did not finish its budget')
        final = [r for r in records if budget - 1 <= r['t'] <= budget]
        result.update(
            samples=len(records), duration_s=records[-1]['t'],
            impulse_ns=sum(r['force_n'] * r['dt'] for r in records),
            max_tilt_rad=max(r['tilt_rad'] for r in records),
            final_max_tilt_rad=max(r['tilt_rad'] for r in final),
            final_max_speed_m_s=max(r['speed_m_s'] for r in final),
            max_effort_nm=max(r['effort_nm'] for r in records),
            forward_progress_m=records[-1]['x']-records[0]['x'],
            lateral_displacement_m=abs(records[-1]['y']-records[0]['y']),
            displacement_m=math.hypot(records[-1]['x']-records[0]['x'],
                                      records[-1]['y']-records[0]['y']),
        )
        # A plant that is slowly toppling can still sit under a generous tilt
        # bar for the whole final second, so also measure whether the body is
        # *changing*: tilt growth and pose drift across the last second.
        result['final_second_tilt_growth_rad'] = abs(
            final[-1]['tilt_rad'] - final[0]['tilt_rad'])
        result['final_second_pose_drift_m'] = math.hypot(
            final[-1]['x'] - final[0]['x'], final[-1]['y'] - final[0]['y'])
        result['final_second_mean_speed_m_s'] = float(
            np.mean([r['speed_m_s'] for r in final]))
        if args.policy:
            # Informative (non-gating) walk/stop measurements: the speed the
            # plant settled into while driving (position-derived, so stepping
            # oscillation does not inflate it), and how long the stopping
            # transient took after the stop command.
            walking = [r for r in records
                       if walk_until - 2 <= r['t'] <= walk_until]
            span = walking[-1]['t'] - walking[0]['t']
            result['walk_settled_window_s'] = [round(walking[0]['t'], 4),
                                               round(walking[-1]['t'], 4)]
            result['walk_settled_travel_speed_m_s'] = (
                walking[-1]['x'] - walking[0]['x']) / span
            result['walk_settled_speed_m_s'] = float(
                np.mean([r['speed_m_s'] for r in walking]))
            result['walk_commanded_speed_m_s'] = .25
            result['walk_settled_tracking_fraction'] = (
                result['walk_settled_travel_speed_m_s'] / .25)
            result['walk_until_s'] = walk_until
            after = [r for r in records if r['t'] >= walk_until]
            result['stop_settle_s'] = next(
                (after[i]['t'] - walk_until for i in range(len(after))
                 if all(r['speed_m_s'] < .05 for r in after[i:])), None)
        result['passed'] = bool(
            all(math.isfinite(v) for r in records for v in r.values())
            and abs(result['impulse_ns'] - args.force * .2) < 1e-6
            and result['max_tilt_rad'] < result['criteria']['max_tilt_rad']
            and result['final_max_tilt_rad'] < result['criteria']['final_second_max_tilt_rad']
            # Both the peak and the reference tool's mean statistic must hold
            # (qualify_policy.assess_phase uses the last-second mean).
            and result['final_max_speed_m_s'] < result['criteria']['final_second_max_speed_m_s']
            and result['final_second_mean_speed_m_s'] < result['criteria']['final_second_max_speed_m_s']
            and result['max_effort_nm'] <= 20.0001
            and (not args.policy or (
                result['forward_progress_m'] >= result['criteria']['min_forward_progress_m']
                and result['lateral_displacement_m'] <= result['criteria']['max_lateral_displacement_m']
                and result['walk_settled_tracking_fraction']
                >= result['criteria']['min_settled_walk_tracking']
                and result['final_second_tilt_growth_rad']
                <= result['criteria']['final_second_tilt_growth_rad']
                and result['final_second_pose_drift_m']
                <= result['criteria']['final_second_pose_drift_m'])))
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
