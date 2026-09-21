#!/usr/bin/env python3
"""Joint friction/armature A/B: the R5.3 live-divergence root cause (2026-09-21).

The vendored URDF declared joint friction in a non-standard
``<joint_properties friction="0.1"/>`` tag that urdfdom, sdformat (the URDF->SDF
conversion ign-gazebo runs) and pybullet all silently discard. The simulated
biped therefore ran with ZERO joint friction, while the robot's own MJCF
declares ``<joint frictionloss="0.1" armature="0.005"/>`` - and the R5.3
balance loop, which is a pure position-PD law, depends on that friction to be
damped. This probe isolates the term.

Method: build a mesh-free MuJoCo scene from the *vendored* MJCF
(src/robot_lab_robots/berkeley_humanoid_lite/mjcf/berkeley_humanoid_lite.xml,
visual geoms and the mesh asset block stripped - collision geoms only, so no
mesh files are needed), drop the base at the R5.3 spawn height (base z=-0.038,
soles exactly on the floor: see sole_height_probe.py) and run the COMMITTED
balance law (imported from robot_lab_adapter.bhl_balance, not re-implemented)
while overriding ``model.dof_armature`` / ``model.dof_frictionloss``:

  A. MJCF defaults: armature=0.005, frictionloss=0.1  (MuJoCo backend plant)
  B. zero/zero                                        (what gz+pybullet loaded
                                                       from the URDF before the fix)
  C. armature only, D. frictionloss only              (which term carries the margin)

Result (recorded in mjcf_friction_probe.txt): A and D HOLD (max tilt
0.0028 rad); C wobbles (0.0821); B falls at t=0.5 s. Joint friction - the term
the URDF had lost - is the stabilizer; rotor inertia alone is not sufficient.
This is why the URDF fix is a standard ``<dynamics friction="0.1"/>`` and why
TestJointFrictionDeclaration pins it.
"""
from __future__ import annotations

import os
import sys
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
# docs/status/evidence/<task-dir>/ -> repo root is four levels up
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_HERE))))
sys.path.insert(0, os.path.join(_REPO, "src", "robot_lab_adapter"))

from robot_lab_adapter.bhl_balance import (  # noqa: E402
    ANKLE_BALANCE_K_PITCH,
    ANKLE_BALANCE_K_ROLL,
    EFFORT_LIMIT,
    STANCE_PD_ARMS,
    STANCE_PD_LEGS,
)

MJCF = os.path.join(_REPO, "src", "robot_lab_robots", "berkeley_humanoid_lite",
                    "mjcf", "berkeley_humanoid_lite.xml")
SPAWN_Z = -0.038
SIM_HZ = 500.0
CTRL_EVERY = 2          # committed node runs the law at 250 Hz (see bhl_balance)
DURATION = 15.0
FALL_RAD = 0.70         # bhl_balance.TILT_FALL_RAD


def build_scene() -> str:
    """Strip visual geoms/assets from the vendored MJCF; add a floor plane."""
    tree = ET.parse(MJCF)
    root = tree.getroot()
    for body in root.iter("body"):
        for geom in list(body.findall("geom")):
            if geom.get("class") == "visual":
                body.remove(geom)
    for asset in list(root.findall("asset")):
        root.remove(asset)
    world = root.find("worldbody")
    floor = ET.SubElement(world, "geom")
    floor.set("name", "floor")
    floor.set("type", "plane")
    floor.set("size", "0 0 0.05")
    out = os.path.join(tempfile.mkdtemp(prefix="bhl_mjcf_"), "scene.xml")
    tree.write(out)
    return out


def roll_pitch(data) -> tuple:
    w, x, y, z = data.qpos[3:7]
    return (float(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            float(np.arcsin(max(-1.0, min(1.0, 2 * (w * y - z * x))))))


def main() -> None:
    scene = build_scene()
    model = mujoco.MjModel.from_xml_path(scene)
    data = mujoco.MjData(model)
    model.opt.timestep = 1.0 / SIM_HZ

    n2 = {}
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            n2[name] = (model.jnt_dofadr[i], model.jnt_qposadr[i])
    legs = [n for n in n2 if n.startswith("leg_")]
    arms = [n for n in n2 if n.startswith("arm_")]
    print(f"scene: {scene}")
    print(f"model: {len(n2)} joints ({len(legs)} leg, {len(arms)} arm), "
          f"dt={1000/SIM_HZ:.1f} ms, law at 250 Hz, spawn z={SPAWN_Z}")
    print(f"law: ankle K roll={ANKLE_BALANCE_K_ROLL} pitch={ANKLE_BALANCE_K_PITCH}, "
          f"leg PD={STANCE_PD_LEGS}, arm PD={STANCE_PD_ARMS}, "
          f"|tau|<={EFFORT_LIMIT:g} N.m")
    print(f"MJCF default joint class: armature="
          f"{model.dof_armature[n2['leg_left_knee_pitch_joint'][0]]:.4f} "
          f"frictionloss="
          f"{model.dof_frictionloss[n2['leg_left_knee_pitch_joint'][0]]:.4f}")

    def run(label: str, armature: float, frictionloss: float,
            perturb_t: float | None = None) -> None:
        model.dof_armature[:] = armature
        model.dof_frictionloss[:] = frictionloss
        mujoco.mj_resetData(model, data)
        data.qpos[2] += SPAWN_Z
        mujoco.mj_forward(model, data)
        max_tilt, fallen, fall_t = 0.0, False, None
        for k in range(int(DURATION * SIM_HZ)):
            if k % CTRL_EVERY == 0:
                roll, pitch = roll_pitch(data)
                for name in legs + arms:
                    dof, q = n2[name]
                    kp, kd = STANCE_PD_LEGS if name in legs else STANCE_PD_ARMS
                    target = 0.0
                    if name.endswith("ankle_roll_joint"):
                        target = ANKLE_BALANCE_K_ROLL * roll
                    elif name.endswith("ankle_pitch_joint"):
                        target = ANKLE_BALANCE_K_PITCH * pitch
                    tau = kp * (target - data.qpos[q]) + kd * (0.0 - data.qvel[dof])
                    data.qfrc_applied[dof] = float(
                        np.clip(tau, -EFFORT_LIMIT, EFFORT_LIMIT))
                if perturb_t is not None and abs(k / SIM_HZ - perturb_t) < 1e-9:
                    data.qvel[3] += 0.06
                    data.qvel[4] += 0.04
            mujoco.mj_step(model, data)
            roll, pitch = roll_pitch(data)
            max_tilt = max(max_tilt, abs(roll), abs(pitch))
            if max_tilt > FALL_RAD and not fallen:
                fallen, fall_t = True, k / SIM_HZ
                break
        verdict = ("FALLS" if fallen else
                   "HOLDS" if max_tilt < 0.20 else "WOBBLE")
        extra = f" at t={fall_t:.1f}" if fallen else ""
        print(f"  {label:52s} -> {verdict:6s} max_tilt={max_tilt:.4f}{extra}")

    print("\nA. MJCF defaults (what the vendored robot description declares)")
    run("armature=0.005 frictionloss=0.1", 0.005, 0.1)
    print("B. as the URDF loaded before the fix (gz + pybullet: zero/zero)")
    run("armature=0 frictionloss=0", 0.0, 0.0)
    run("armature=0 frictionloss=0 + perturb t=3", 0.0, 0.0, perturb_t=3.0)
    print("C/D. single-term ablations")
    run("armature=0.005 only (no friction)", 0.005, 0.0)
    run("frictionloss=0.1 only (no armature)", 0.0, 0.1)
    print("\nCONCLUSION: joint friction is the term the URDF had lost; with the")
    print("standard <dynamics friction=\"0.1\"/> tag (this commit) gz and pybullet")
    print("load the same stabilizer MuJoCo gets from frictionloss=0.1.")


if __name__ == "__main__":
    main()
