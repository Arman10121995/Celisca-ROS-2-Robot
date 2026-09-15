#!/usr/bin/env python3
"""Validate the ROS adapter policy path on the native MuJoCo model.

Unlike qualify_policy.py, which inlines its own observation/action code, this
tool runs the *same* BhlPolicyController the humanoid_policy_controller ROS
node calls, fed with onboard state only: the simulated IMU quaternion and
gyro and the measured joint states mapped by name. No simulator ground-truth
tracking feedback is used, so the result is what the deployed adapter would
see. Equivalence with the open-loop probe (r53-bhl-policy-probe) validates
the adapter's observation assembly, action conversion and safety path.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np

ROBOT_DIR = Path(__file__).resolve().parents[1]
TOOLS_DIR = Path(__file__).resolve().parent
ADAPTER_SRC = ROBOT_DIR.parent.parent / 'robot_lab_adapter'
for path in (str(TOOLS_DIR), str(ADAPTER_SRC)):
    if path not in sys.path:
        sys.path.insert(0, path)

from qualify_policy import PHASES, assess_phase, rotation_state  # noqa: E402
from robot_lab_adapter.bhl_policy import (  # noqa: E402
    BhlPolicyController, load_policy_config, upstream_dir)


def load_model(robot_dir: Path, policy: str):
    """The native MJCF with flattened meshes and a qualification floor."""
    import mujoco

    model_path = robot_dir / 'mjcf/berkeley_humanoid_lite.xml'
    xml = ET.parse(model_path).getroot()
    xml.find('compiler').set('meshdir', str(robot_dir / 'meshes'))
    for mesh in xml.findall('./asset/mesh'):
        mesh_path = robot_dir / 'meshes' / Path(mesh.get('file')).name
        if not mesh_path.is_file():
            raise ValueError(f'missing native-model mesh: {mesh_path}')
        mesh.set('file', str(mesh_path))
    ET.SubElement(xml.find('worldbody'), 'geom', name='qualification_floor',
                  type='plane', size='20 20 .1', friction='1 .005 .0001')
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding='unicode'))
    data = mujoco.MjData(model)
    return model_path, model, data


def run(robot_dir: Path, policy: str, speed: float, output: Path) -> dict:
    import mujoco
    import onnxruntime as ort

    config = load_policy_config(name=policy)  # adapter's own config loader
    model_path, model, data = load_model(robot_dir, policy)
    model.opt.timestep = config.physics_dt
    substeps = round(config.policy_dt / config.physics_dt)
    joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
              for name in config.joints]
    if any(joint < 0 for joint in joints):
        raise ValueError('configured joints do not match the native model')
    qidx, vidx = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    actuators = []
    for joint in joints:
        matches = [i for i in range(model.nu) if model.actuator_trnid[i, 0] == joint]
        if len(matches) != 1:
            raise ValueError('each configured joint must have exactly one actuator')
        actuators.append(matches[0])
    kp, kd, limits = config.kp, config.kd, config.effort_limits

    options = ort.SessionOptions()
    options.intra_op_num_threads = options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(config.checkpoint), options,
                                   providers=['CPUExecutionProvider'])
    # The adapter runs with the real inference session, dependency-injected.
    controller = BhlPolicyController(config, session=session)

    data.qpos[qidx] = config.nominal
    data.qpos[:3] = config.default_base_position
    mujoco.mj_forward(model, data)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base')
    floor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'qualification_floor')
    foot_bodies = {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
                   for name in ('leg_left_ankle_roll', 'leg_right_ankle_roll')}
    commands = {'stand': [0, 0, 0], 'perturb': [0, 0, 0],
                'walk': [speed, 0, 0], 'turn': [0, 0, .6], 'stop': [0, 0, 0]}

    wall_start = time.monotonic()
    trace, reports, safety_failure = [], [], None
    adapter_safe_stop = False
    for phase, duration in PHASES:
        start_xy, start_time = data.qpos[:2].copy(), data.time
        start_yaw = rotation_state(data.qpos[3:7])[0]
        rows = []
        for _ in range(round(duration / config.policy_dt)):
            # --- onboard state only: simulated IMU + joint states ----------
            imu_quat = data.sensor('imu_quat').data  # MuJoCo order (w, x, y, z)
            orientation = tuple(float(v) for v in imu_quat[[1, 2, 3, 0]])
            gyro = tuple(float(v) for v in data.sensor('imu_gyro').data)
            measured_positions = {name: float(data.qpos[qi])
                                  for name, qi in zip(config.joints, qidx)}
            measured_velocities = {name: float(data.qvel[vi])
                                   for name, vi in zip(config.joints, vidx)}
            cycle = controller.update(commands[phase], gyro, orientation,
                                      measured_positions, measured_velocities)
            if cycle.safety_state == 'SAFE_STOP':
                adapter_safe_stop = True
                safety_failure = f'adapter latched SAFE_STOP: {controller.reason}'
            target = np.array([cycle.position_targets[name] for name in config.joints])
            # --- backend PD tracking, identical to qualify_policy ----------
            max_effort, max_excess, nonfoot_contacts = 0., 0, 0
            for _ in range(substeps):
                relative_time = data.time - start_time
                data.xfrc_applied[:] = 0
                if phase == 'perturb' and 1.0 <= relative_time < 1.2:
                    data.xfrc_applied[base_id, 1] = 5.0  # 5 N lateral, 0.2 s
                data.ctrl[actuators] = np.clip(
                    kp * (target - data.qpos[qidx]) - kd * data.qvel[vidx],
                    -limits, limits)
                mujoco.mj_step(model, data)
                max_effort = max(max_effort, float(np.max(
                    np.abs(data.actuator_force[actuators]) / limits)))
                excess = np.maximum(model.jnt_range[joints, 0] - data.qpos[qidx],
                                    data.qpos[qidx] - model.jnt_range[joints, 1])
                max_excess = max(max_excess, float(np.max(excess)))
                for contact in data.contact:
                    if floor_id in (contact.geom1, contact.geom2):
                        other = (contact.geom2 if contact.geom1 == floor_id
                                 else contact.geom1)
                        nonfoot_contacts += int(
                            model.geom_bodyid[other] not in foot_bodies)
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
            rows.append(dict(
                phase=phase, sim_time=data.time, phase_time=data.time - start_time,
                x=data.qpos[0], y=data.qpos[1], z=data.qpos[2], yaw=yaw, tilt=tilt,
                speed=float(np.linalg.norm(data.qvel[:2])),
                effort_fraction=max_effort, joint_limit_excess=max_excess,
                policy_vx=commands[phase][0], policy_vy=commands[phase][1],
                policy_wz=commands[phase][2],
                adapter_safety_state=cycle.safety_state,
                nonfoot_floor_contacts=nonfoot_contacts))
            if safety_failure:
                break
        trace.extend(rows)
        report = assess_phase(phase, rows, duration, commands[phase],
                              start_xy, start_yaw, safety_failure)
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
    cfg_path = upstream_dir() / 'configs' / f'{policy}.yaml'
    files = [cfg_path, config.checkpoint, model_path]
    result = {
        'qualification': ('adapter-path validation: BhlPolicyController (the code the '
                          'humanoid_policy_controller ROS node calls) on the native '
                          'MuJoCo model, onboard state only'),
        'onboard_state_only': True,
        'policy': policy,
        'passed': (len(reports) == len(PHASES) and all(r['passed'] for r in reports)
                   and not adapter_safe_stop),
        'adapter_safe_stop': adapter_safe_stop,
        'mujoco_version': mujoco.__version__, 'onnxruntime_version': ort.__version__,
        'physics_dt': config.physics_dt, 'policy_dt': config.policy_dt,
        'commands': commands,
        'perturbation': {'force_y_N': 5.0, 'duration_s': .2},
        'trial_protocol': 'deterministic, no injected random seed',
        'wall_seconds': time.monotonic() - wall_start,
        'artifacts_sha256': {
            ('robot/' + str(p.relative_to(robot_dir)) if p.is_relative_to(robot_dir)
             else 'upstream/' + str(p.relative_to(upstream_dir()))):
            hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        'phases': reports,
        'not_run': [name for name, _ in PHASES
                    if name not in {r['phase'] for r in reports}]}
    (output / 'summary.json').write_text(
        json.dumps(result, indent=2, allow_nan=False) + '\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-dir', type=Path, default=ROBOT_DIR)
    parser.add_argument('--policy', choices=['policy_humanoid', 'policy_humanoid_legs'],
                        default='policy_humanoid_legs')
    parser.add_argument('--speed', type=float, default=.25)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or not 0 < args.speed <= .5:
        parser.error('--speed must be finite, positive and at most 0.5 m/s')
    result = run(args.robot_dir.resolve(), args.policy, args.speed, args.out.resolve())
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

