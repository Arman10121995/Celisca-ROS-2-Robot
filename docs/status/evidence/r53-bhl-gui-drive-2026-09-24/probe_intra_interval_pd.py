#!/usr/bin/env python3
"""Matched BHL A/B for 250 Hz held effort versus fresh 2 kHz PD.

This diagnostic is deliberately not a runtime qualification. It uses the
vendored native MJCF (the common MuJoCo backend mirrors its joint losses),
BhlPolicyController, StartupSettle, checkpoint gains/limits, spawn, and delayed
command schedule from the live ROS probe. The only difference is whether the
checkpoint PD is recomputed from fresh joint state every 2 kHz physics step or
held at the common path's 250 Hz effort-command rate.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[4]
ROBOT = REPO / "src/robot_lab_robots/berkeley_humanoid_lite"
sys.path[:0] = [str(ROBOT / "tools"), str(REPO / "src/robot_lab_adapter")]

from qualify_policy_adapter import load_model  # noqa: E402
from robot_lab_adapter.bhl_balance import (  # noqa: E402
    BodyState, balance_targets, pd_effort_command, TILT_FALL_RAD)
from robot_lab_adapter.bhl_policy import (  # noqa: E402
    BhlPolicyController, StartupSettle, load_policy_config)


def window(rows, lo, hi):
    values = [row for row in rows if lo <= row["t"] < hi]
    if not values:
        return {}
    spread = [row["effort_spread"] for row in values]
    return {
        "mean_yaw_rate": float(np.mean([row["yaw_rate"] for row in values])),
        "max_tilt": float(np.max([row["tilt"] for row in values])),
        "mean_effort_spread": float(np.mean(spread)),
        "std_effort_spread": float(np.std(spread)),
        "dyaw": float(values[-1]["yaw"] - values[0]["yaw"]),
    }


def run(policy_name, fresh_pd):
    import mujoco

    config = load_policy_config(name=policy_name)
    _, model, data = load_model(ROBOT, policy_name)
    model.opt.timestep = config.physics_dt
    policy_dt = config.policy_dt
    physics_steps_per_policy = round(policy_dt / config.physics_dt)
    if not math.isclose(physics_steps_per_policy * config.physics_dt, policy_dt):
        raise ValueError("policy period must contain whole physics steps")
    joints = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
              for name in config.joints]
    qadr = model.jnt_qposadr[joints]
    vadr = model.jnt_dofadr[joints]
    actuators = []
    for name in config.joints:
        matches = [i for i in range(model.nu)
                   if model.actuator_trnid[i, 0] == mujoco.mj_name2id(
                       model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        if len(matches) != 1:
            raise ValueError(f"expected one actuator for {name}")
        actuators.append(matches[0])
    mujoco.mj_resetData(model, data)
    data.qpos[2] = -0.038
    mujoco.mj_forward(model, data)
    controller = BhlPolicyController(config)
    settle = StartupSettle(config.hold_pose(), duration_s=2.0)
    targets = dict(config.hold_pose())
    driving = False
    rows = []
    start_yaw = None
    last_effort = np.zeros(len(config.joints))

    def measured():
        q = {name: float(data.qpos[adr]) for name, adr in zip(config.joints, qadr)}
        dq = {name: float(data.qvel[adr]) for name, adr in zip(config.joints, vadr)}
        quat_wxyz = data.sensor("imu_quat").data
        orientation = tuple(float(quat_wxyz[i]) for i in (1, 2, 3, 0))
        gyro = tuple(float(v) for v in data.sensor("imu_gyro").data)
        return q, dq, orientation, gyro

    def efforts(q, dq, orientation):
        if controller.safety_state == controller.SAFE_STOP:
            return np.zeros(len(config.joints))
        if not driving or not settle.settled:
            stance = balance_targets(targets, BodyState.from_quaternion(*orientation))
            values = pd_effort_command(stance, q, dq)
            return np.asarray([values[name] for name in config.joints])
        target = np.asarray([targets[name] for name in config.joints])
        position = np.asarray([q[name] for name in config.joints])
        velocity = np.asarray([dq[name] for name in config.joints])
        return np.clip(config.kp * (target - position) - config.kd * velocity,
                       -config.effort_limits, config.effort_limits)

    effort_period_steps = round(0.004 / config.physics_dt)
    for step in range(round(18.0 / config.physics_dt)):
        now = step * config.physics_dt
        q, dq, orientation, gyro = measured()
        if step % physics_steps_per_policy == 0:
            if now < 1.0:
                targets = settle.hold_targets(q)
            elif not settle.settled:
                if not settle.active:
                    settle.start(q)
                targets, _ = settle.targets(q, policy_dt)
            else:
                driving = True
                command = [0.0, 0.0, 0.3] if 3.0 <= now < 13.0 else [0.0, 0.0, 0.0]
                cycle = controller.update(command, gyro, orientation, q, dq)
                targets = cycle.position_targets
            q, dq, orientation, _ = measured()
            current = efforts(q, dq, orientation)


            if fresh_pd or step % effort_period_steps == 0:
                last_effort = current
            quat_wxyz = data.sensor("imu_quat").data
            x, y, z, w = (float(quat_wxyz[i]) for i in (1, 2, 3, 0))
            yaw = math.atan2(2.0 * (w * z + x * y),
                            1.0 - 2.0 * (y * y + z * z))
            tilt = math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))
            if start_yaw is None:
                start_yaw = yaw
            rows.append({"t": now, "yaw": yaw, "yaw_rate": float(gyro[2]),
                         "tilt": tilt, "effort_spread": float(np.sum(np.abs(last_effort))),
                         "effort": last_effort.round(5).tolist()})
            if tilt >= TILT_FALL_RAD or not np.isfinite(data.qpos).all():
                break
        if fresh_pd or step % effort_period_steps == 0:
            q, dq, orientation, _ = measured()
            last_effort = efforts(q, dq, orientation)
        data.ctrl[actuators] = last_effort
        mujoco.mj_step(model, data)

    return {"fresh_pd": fresh_pd, "physics_dt_s": config.physics_dt,
            "policy_dt_s": policy_dt, "rows": rows,
            "windows": {f"{lo}-{hi}": window(rows, lo, hi)
                        for lo, hi in ((3, 4), (4, 5), (5, 6), (8, 13), (13, 18))}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    result = {"diagnostic": "BHL fresh-PD timing only", "modes": [
        run("policy_humanoid", fresh_pd=False),
        run("policy_humanoid", fresh_pd=True)]}
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    for mode in result["modes"]:
        print(mode["fresh_pd"], json.dumps(mode["windows"], indent=2))


if __name__ == "__main__":
    main()

