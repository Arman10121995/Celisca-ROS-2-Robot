#!/usr/bin/env python3
"""Static stance feasibility: gravity-holding torque per joint at q=0.

The R5.3 live runs (and the earlier 0.78/0.042 spawn drafts) all end in a
topple or a passive leg collapse. Before blaming the balance law, this probe
answers the ground-level question with inverse dynamics: with the base
pinned at the rest pose (all joints 0) and the robot standing on its soles,
what CONSTANT torque does each joint need to hold gravity?

Method: pybullet calculateInverseDynamics with useFixedBase=True, zero
positions/velocities/accelerations, server gravity -9.81. The returned
tau is the torque each joint motor must supply for the configuration to be
static. Compare against the URDF 20 N.m effort limit.

Recorded output: static_torque_probe.txt (committed next to this script).
"""
import sys

import pybullet as p
import pybullet_data

sys.path.insert(0, "/home/molar1/bumperbot_ws/src/robot_lab_adapter")
from robot_lab_adapter.bhl_balance import BHL_JOINT_NAMES

URDF = ("/home/molar1/bumperbot_ws/src/robot_lab_robots/"
        "berkeley_humanoid_lite/urdf/berkeley_humanoid_lite.urdf")


def main():
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    rid = p.loadURDF(URDF, basePosition=[0, 0, -0.038],
                     baseOrientation=[0, 0, 0, 1], useFixedBase=True)
    n = p.getNumJoints(rid)
    qs, qds, qddts = [0.0] * n, [0.0] * n, [0.0] * n
    tau = p.calculateInverseDynamics(rid, qs, qds, qddts, flags=1)
    names = [p.getJointInfo(rid, j)[1].decode() for j in range(n)]
    print("static gravity-holding torque per joint (N.m), q=0, base pinned:")
    worst = 0.0
    for nm, t in zip(names, tau):
        flag = "  <-- EXCEEDS 20 N.m" if abs(t) > 20 else ""
        worst = max(worst, abs(t))
        print(f"  {nm:36s} {t:+8.3f}{flag}")
    print(f"max |tau| = {worst:.2f} N.m (limit 20 N.m)")
    print("joint order matches BHL_JOINT_NAMES for the leg/arm entries:",
          [nm for nm in names if nm in BHL_JOINT_NAMES] == BHL_JOINT_NAMES)


if __name__ == "__main__":
    main()
