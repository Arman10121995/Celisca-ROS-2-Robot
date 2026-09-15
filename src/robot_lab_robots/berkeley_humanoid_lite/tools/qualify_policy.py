#!/usr/bin/env python3
"""Headless BHL policy qualification on its native, dynamically simulated model.

This development tool uses existing upstream ONNX checkpoints. It does not
connect to hardware or claim integration with the Robot Lab ROS adapter. Run it
with the workspace simulator Python environment; no gamepad/viewer is required.
Observation conventions follow Berkeley's low-level controller at commit
652777cc7c49884e7cd7ddfada758dc1979bf627 (command, gyro, gravity, q, dq, previous action).
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml

ROBOT_DIR = Path(__file__).resolve().parents[1]
UPSTREAM_DIR = ROBOT_DIR.parent / '_upstream/Berkeley-Humanoid-Lite'
PHASES = [('stand', 5.0), ('perturb', 5.0), ('walk', 10.0), ('turn', 5.0), ('stop', 5.0)]


def rotation_state(quaternion):
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    tilt = math.acos(np.clip(1 - 2 * (x * x + y * y), -1, 1))
    # R(q).T @ [0, 0, -1], quaternion convention (w,x,y,z).
    gravity = np.array([2 * (w * y - x * z), -2 * (w * x + y * z),
                        2 * (x * x + y * y) - 1])
    return yaw, tilt, gravity


def assess_phase(name, rows, duration, command, start_xy, start_yaw, safety_failure):
    """Outcome depends on observed completion/tracking, not elapsed wall time."""
    if not rows:
        return {'phase': name, 'passed': False, 'errors': ['no observations']}
    elapsed = rows[-1]['phase_time']
    dx, dy = rows[-1]['x'] - start_xy[0], rows[-1]['y'] - start_xy[1]
    progress = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
    cross_track = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
    yaw_change = float(np.unwrap([start_yaw] + [row['yaw'] for row in rows])[-1] - start_yaw)
    tail = [row for row in rows if row['phase_time'] >= duration - 1.0]
    final_speed = float(np.mean([row['speed'] for row in tail])) if tail else None
    errors = []
    if safety_failure:
        errors.append(safety_failure)
    if elapsed < duration - 1e-6:
        errors.append('phase did not finish its simulated-time budget')
    if name == 'walk':
        if progress < .6 * command[0] * duration:
            errors.append('forward progress below 60% of commanded distance')
        if abs(cross_track) > .5:
            errors.append('lateral drift exceeds 0.5 m')
    if name == 'turn' and yaw_change < .5 * command[2] * duration:
        errors.append('yaw progress below 50% of commanded rotation')
    if name in ('stand', 'perturb', 'stop'):
        if final_speed is None or final_speed > .05:
            errors.append('last-second mean planar speed exceeds 0.05 m/s or is unavailable')
        if rows[-1]['tilt'] > .25:
            errors.append('final body tilt exceeds 0.25 rad')
    if name == 'stop' and tail:
        drift = math.hypot(tail[-1]['x'] - tail[0]['x'], tail[-1]['y'] - tail[0]['y'])
        if drift > .05:
            errors.append('last-second stopping drift exceeds 0.05 m')
    return {'phase': name, 'passed': not errors, 'errors': errors,
            'simulated_seconds': elapsed, 'forward_progress_m': progress,
            'lateral_drift_m': cross_track, 'yaw_change_rad': yaw_change,
            'maximum_tilt_rad': max(row['tilt'] for row in rows),
            'last_second_mean_speed_m_s': final_speed,
            'maximum_joint_limit_excess_rad': max(row['joint_limit_excess'] for row in rows),
            'maximum_effort_fraction': max(row['effort_fraction'] for row in rows)}


def qualify(robot_dir, upstream_dir, policy, speed, output):
    import mujoco
    import onnxruntime as ort

    cfg_path = upstream_dir / 'configs' / f'{policy}.yaml'
    cfg = yaml.safe_load(cfg_path.read_text())
    checkpoint = upstream_dir / cfg['policy_checkpoint_path']
    model_path = robot_dir / 'mjcf/berkeley_humanoid_lite.xml'
    xml = ET.parse(model_path).getroot()
    xml.find('compiler').set('meshdir', str(robot_dir / 'meshes'))
    # The vendored meshes were flattened; upstream MJCF retains assets/merged/.
    mesh_paths = []
    for mesh in xml.findall('./asset/mesh'):
        mesh_path = robot_dir / 'meshes' / Path(mesh.get('file')).name
        if not mesh_path.is_file():
            raise ValueError(f'missing native-model mesh: {mesh_path}')
        mesh.set('file', str(mesh_path))
        mesh_paths.append(mesh_path)
    ET.SubElement(xml.find('worldbody'), 'geom', name='qualification_floor',
                  type='plane', size='20 20 .1', friction='1 .005 .0001')
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'))
    data = mujoco.MjData(model)
    model.opt.timestep = cfg['physics_dt']
    substeps = round(cfg['policy_dt'] / cfg['physics_dt'])
    if not math.isclose(substeps * cfg['physics_dt'], cfg['policy_dt']):
        raise ValueError('policy period must be a whole number of physics steps')
    joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in cfg['joints']]
    if any(joint < 0 for joint in joints) or len(set(joints)) != cfg['num_joints']:
        raise ValueError('configured joints do not match the native model')
    qidx, vidx = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    actuators = []
    for joint in joints:
        matches = [i for i in range(model.nu) if model.actuator_trnid[i, 0] == joint]
        if len(matches) != 1:
            raise ValueError('each configured joint must have exactly one actuator')
        actuators.append(matches[0])
    indices = np.array(cfg['action_indices'])
    nominal = np.array(cfg['default_joint_positions'])
    kp, kd, limits = (np.array(cfg[key]) for key in ('joint_kp', 'joint_kd', 'effort_limits'))
    if len(indices) != cfg['num_actions'] or cfg['history_length'] != 0:
        raise ValueError('this qualifier requires a configured feed-forward policy')
    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(checkpoint), options, providers=['CPUExecutionProvider'])
    input_info = session.get_inputs()[0]
    if input_info.shape != [1, cfg['num_observations']]:
        raise ValueError('policy observation shape does not match configuration')
    data.qpos[qidx] = nominal
    data.qpos[:3] = cfg['default_base_position']
    mujoco.mj_forward(model, data)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base')
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'qualification_floor')
    foot_bodies = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                   for name in ('leg_left_ankle_roll', 'leg_right_ankle_roll')}
    previous = np.zeros(cfg['num_actions'])
    commands = {'stand': [0, 0, 0], 'perturb': [0, 0, 0],
                'walk': [speed, 0, 0], 'turn': [0, 0, .6], 'stop': [0, 0, 0]}
    wall_start = time.monotonic()
    trace, reports, safety_failure = [], [], None
    for phase, duration in PHASES:
        start_xy, start_time = data.qpos[:2].copy(), data.time
        start_yaw = rotation_state(data.qpos[3:7])[0]
        rows = []
        for _ in range(round(duration / cfg['policy_dt'])):
            gravity = rotation_state(data.sensor('imu_quat').data)[2]
            obs = np.concatenate([commands[phase], data.sensor('imu_gyro').data, gravity,
                                  (data.qpos[qidx] - nominal)[indices], data.qvel[vidx][indices],
                                  previous]).astype(np.float32)[None, :]
            raw = np.asarray(session.run(None, {input_info.name: obs})[0])
            if raw.shape != (1, cfg['num_actions']) or not np.isfinite(raw).all():
                safety_failure = 'invalid policy output'
                break
            previous = np.clip(raw[0], cfg['action_limit_lower'], cfg['action_limit_upper'])
            target = nominal.copy()
            target[indices] += previous * cfg['action_scale']
            max_effort, max_excess, nonfoot_contacts = 0., 0., 0
            for _ in range(substeps):
                relative_time = data.time - start_time
                data.xfrc_applied[:] = 0
                if phase == 'perturb' and 1.0 <= relative_time < 1.2:
                    data.xfrc_applied[base_id, 1] = 5.0  # 5 N lateral, 0.2 s; 1 N.s impulse.
                data.ctrl[actuators] = np.clip(kp * (target - data.qpos[qidx])
                                              - kd * data.qvel[vidx], -limits, limits)
                mujoco.mj_step(model, data)
                max_effort = max(max_effort, float(np.max(np.abs(data.actuator_force[actuators]) / limits)))
                excess = np.maximum(model.jnt_range[joints, 0] - data.qpos[qidx],
                                    data.qpos[qidx] - model.jnt_range[joints, 1])
                max_excess = max(max_excess, float(np.max(excess)))
                for contact in data.contact:
                    if floor_id in (contact.geom1, contact.geom2):
                        other = contact.geom2 if contact.geom1 == floor_id else contact.geom1
                        nonfoot_contacts += int(model.geom_bodyid[other] not in foot_bodies)
                if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                    safety_failure = 'non-finite physics state'
                    break
                if rotation_state(data.qpos[3:7])[1] >= .7:
                    safety_failure = 'fall: body tilt reached 0.70 rad'
                    break
                if nonfoot_contacts:
                    safety_failure = 'non-foot body contact with floor'
                    break
                if max_excess > .03 or max_effort > 1.00001:
                    safety_failure = 'joint position or effort limit exceeded'
                    break
            yaw, tilt, _ = rotation_state(data.qpos[3:7])
            row = dict(phase=phase, sim_time=data.time, phase_time=data.time - start_time,
                       x=data.qpos[0], y=data.qpos[1], z=data.qpos[2], yaw=yaw, tilt=tilt,
                       speed=float(np.linalg.norm(data.qvel[:2])),
                       effort_fraction=max_effort, joint_limit_excess=max_excess,
                       nonfoot_floor_contacts=nonfoot_contacts)
            rows.append(row)
            if safety_failure:
                break
        trace.extend(rows)
        report = assess_phase(phase, rows, duration, commands[phase], start_xy, start_yaw, safety_failure)
        reports.append(report)
        print(json.dumps(report), flush=True)
        if safety_failure:
            break
    data.ctrl[:] = 0
    data.xfrc_applied[:] = 0
    output.mkdir(parents=True, exist_ok=True)
    if trace:
        with (output / 'trace.csv').open('w') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(trace[0]))
            writer.writeheader()
            writer.writerows(trace)
    files = [cfg_path, checkpoint, model_path, *mesh_paths]
    result = {'qualification': 'native MuJoCo policy probe; ROS integration not established',
              'policy': policy, 'passed': len(reports) == len(PHASES) and all(r['passed'] for r in reports),
              'mujoco_version': mujoco.__version__, 'onnxruntime_version': ort.__version__,
              'physics_dt': cfg['physics_dt'], 'policy_dt': cfg['policy_dt'],
              'commands': commands, 'perturbation': {'force_y_N': 5.0, 'duration_s': .2},
              'trial_protocol': 'deterministic, no injected random seed',
              'wall_seconds': time.monotonic() - wall_start,
              'artifacts_sha256': {str(p.relative_to(robot_dir.parent)): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in files}, 'phases': reports,
              'not_run': [name for name, _ in PHASES if name not in {r['phase'] for r in reports}]}
    (output / 'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-dir', type=Path, default=ROBOT_DIR)
    parser.add_argument('--upstream-dir', type=Path, default=UPSTREAM_DIR)
    parser.add_argument('--policy', choices=['policy_humanoid', 'policy_humanoid_legs'], default='policy_humanoid_legs')
    parser.add_argument('--speed', type=float, default=.5)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or not 0 < args.speed <= .5:
        parser.error('--speed must be finite, positive and at most 0.5 m/s')
    result = qualify(args.robot_dir.resolve(), args.upstream_dir.resolve(), args.policy, args.speed, args.out)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
