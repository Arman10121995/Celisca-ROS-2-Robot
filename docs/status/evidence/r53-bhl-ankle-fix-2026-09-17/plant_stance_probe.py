#!/usr/bin/env python3
"""Closed-loop stance plant test of the committed R5.3 balance path.

Runs the exact humanoid_standing_controller control path (BhlBalanceController:
balance_targets + pd_effort_command, 50 Hz, tau clamped to 20 N.m) against the
vendored BHL URDF in pybullet, at the corrected spawn z=-0.038 (soles 2 mm
above ground; see sole_height_probe.txt).

Two modes:
  A  pure PD stance hold (targets = nominal rest pose, no balance reaction)
  B  full committed balance law (balance_targets with IMU roll/pitch feedback)

Lessons baked in (from the earlier failed attempts, /tmp lost + this file):
- The spawn must be NEGATIVE: the sole sits at +0.0400 m in the base frame.
- The URDF velocity limit (15 rad/s) must be enforced via maxJointVelocity,
  else saturated low-inertia arm joints spin to pybullet's internal 100 rad/s
  clamp, lose ground contact and explode the sim (artifacts, not physics).
- Sim at 1 kHz, control at 50 Hz (matching the standing controller).

Recorded output: plant_stance_probe.txt (committed next to this script).
Verdict per mode: HOLDS (no SAFE_STOP, max tilt < 0.20 rad) / WOBBLE / FALLS.
"""
import math
import sys

import pybullet as p
import pybullet_data

sys.path.insert(0, "/home/molar1/bumperbot_ws/src/robot_lab_adapter")
from robot_lab_adapter.bhl_balance import (  # noqa: E402
    BHL_JOINT_NAMES, BhlBalanceController, BodyState, clamp_effort)

URDF = ("/home/molar1/bumperbot_ws/src/robot_lab_robots/"
        "berkeley_humanoid_lite/urdf/berkeley_humanoid_lite.urdf")
SPAWN_Z = -0.038
SIM_HZ = 1000
CTRL_HZ = 50
DURATION_S = 15.0


def roll_pitch(q):
    x, y, z, w = q
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))


def run(mode):
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0 / SIM_HZ)
    p.loadURDF("plane.urdf")
    rid = p.loadURDF(URDF, basePosition=[0, 0, SPAWN_Z])
    n2i = {p.getJointInfo(rid, j)[1].decode(): j
           for j in range(p.getNumJoints(rid))}
    idx = [n2i[n] for n in BHL_JOINT_NAMES]
    for j in range(p.getNumJoints(rid)):
        p.changeDynamics(rid, j, maxJointVelocity=15.0)
        p.setJointMotorControl2(rid, j, p.VELOCITY_CONTROL, force=0)
    ctrl = BhlBalanceController()
    safe = False
    max_tilt = 0.0
    print(f"mode {mode}: spawn z={SPAWN_Z}, {SIM_HZ} Hz sim, {CTRL_HZ} Hz control, "
          f"{DURATION_S} s")
    for k in range(int(SIM_HZ * DURATION_S)):
        if k % (SIM_HZ // CTRL_HZ) == 0:
            st = p.getJointStates(rid, idx)
            meas = {n_: s[0] for n_, s in zip(BHL_JOINT_NAMES, st)}
            vel = {n_: s[1] for n_, s in zip(BHL_JOINT_NAMES, st)}
            roll, pitch = roll_pitch(p.getBasePositionAndOrientation(rid)[1])
            body = BodyState(roll_rad=roll, pitch_rad=pitch)
            cycle = ctrl.update(dt=1.0 / CTRL_HZ, measured_positions=meas,
                                measured_velocities=vel, body=body)
            if any("safe_stop" in i for i in cycle.issues):
                safe = True
            if mode == "A":  # pure stance hold, no balance reaction
                tau = [clamp_effort(n_, 120.0 * (0.0 - meas[n_]) + 4.0 * (0.0 - vel[n_]))
                       for n_ in BHL_JOINT_NAMES]
            else:            # committed balance law
                tau = [cycle.efforts[n_] for n_ in BHL_JOINT_NAMES]
            p.setJointMotorControlArray(rid, idx, p.TORQUE_CONTROL, forces=tau)
        p.stepSimulation()
        if k % (SIM_HZ // 2) == 0:
            roll, pitch = roll_pitch(p.getBasePositionAndOrientation(rid)[1])
            max_tilt = max(max_tilt, abs(roll), abs(pitch))
            if k % (2 * SIM_HZ) == 0:
                z = p.getBasePositionAndOrientation(rid)[0][2]
                print(f"  t={k / SIM_HZ:4.1f} roll={roll:+.4f} pitch={pitch:+.4f} "
                      f"z={z:+.4f}" + ("  [SAFE_STOP]" if safe else ""))
    verdict = ("HOLDS" if not safe and max_tilt < 0.20 else
               "WOBBLE" if not safe and max_tilt < 0.70 else "FALLS")
    print(f"MODE {mode}: safe_stop={safe} max_tilt={max_tilt:.4f} -> {verdict}")
    p.disconnect()


if __name__ == "__main__":
    for m in sys.argv[1:] or ["A", "B"]:
        run(m)
