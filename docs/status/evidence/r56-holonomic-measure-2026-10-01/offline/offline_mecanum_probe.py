#!/usr/bin/env python3
"""Offline MuJoCo check of the mecanum roller's physical lateral support.

Builds the imported mecanum model (60 roller bodies/joints/colliders) and
drives it with the mecanum drive model: pure lateral (vy), forward and pure
yaw.  Prints body velocity, driven-wheel rates and the fastest roller spins,
so a blocked lateral motion is attributed to the contact model rather than
guessed at.
"""
import os
import re
import sys

import numpy as np
import yaml

SRC = "/home/molar1/bumperbot_ws/src"
sys.path[:0] = [os.path.join(SRC, "robot_lab_mujoco", "python"),
                os.path.join(SRC, "robot_lab_utils")]

import mujoco  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from robot_lab_mujoco import mujoco_spawner as sp  # noqa: E402
from robot_lab_utils.drive_kinematics import drive_from_config  # noqa: E402

ROBOTS = os.path.join(SRC, "robot_lab_robots", "config", "robots.yaml")
ROBOT = "mecanum_car"
config = yaml.safe_load(open(ROBOTS))["robots"][ROBOT]
drive = drive_from_config(config.get("drive") or {})

path = os.path.join(get_package_share_directory("robot_lab_robots"),
                    config["xacro"])
urdf = sp._xacro_to_urdf(path)
packages = {}
for package in set(re.findall(r"package://([^/]+)/", urdf)):
    try:
        packages[package] = get_package_share_directory(package)
    except Exception:
        pass
mjcf = sp._build_mjcf_from_urdf(urdf, packages, logger=None, robot_name=ROBOT,
                                base_dir=os.path.dirname(os.path.abspath(path)))
mjcf = sp._add_wheel_velocity_actuators(mjcf, drive.wheel_joints)
world = os.path.join(get_package_share_directory("robot_lab_maps"), "mjcf",
                     "nav_empty.xml")
model = mujoco.MjModel.from_xml_string(
    sp.MuJoCoSpawner._merge_mjcf(world, mjcf))
data = mujoco.MjData(model)
free = [j for j in range(model.njnt)
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE][0]
qadr, vadr = model.jnt_qposadr[free], model.jnt_dofadr[free]
data.qpos[qadr + 2] = 0.0
for _ in range(int(1.0 / model.opt.timestep)):   # settle on the floor
    mujoco.mj_step(model, data)

act = {j: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, j + "_velocity")
       for j in drive.wheel_joints}
roller_dofs = [model.jnt_dofadr[j] for j in range(model.njnt)
               if "roller" in (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or "")]

print("wheels:", list(drive.wheel_joints), "roller dofs:", len(roller_dofs))


def phase(label, vx, vy, wz, duration):
    drive.reset()
    n = int(duration / model.opt.timestep)
    for _ in range(n):
        targets = drive.targets(vx, wz, dt=model.opt.timestep, vy=vy)
        for joint, rate in targets.velocity.items():
            data.ctrl[act[joint]] = max(-50.0, min(50.0, rate))
        mujoco.mj_step(model, data)
    w, _ = data.qpos[qadr + 3:qadr + 7][3], None
    quat = data.qpos[qadr + 3:qadr + 7]
    yaw = 2.0 * np.arctan2(quat[3], quat[0])
    body = data.qvel[vadr:vadr + 3]
    yaw_rate = data.qvel[vadr + 5]
    wheel_rates = " ".join(
        "%s=%+.2f" % (j.split("_wheel_joint")[0][-2:],
                      data.qvel[model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)]])
        for j in drive.wheel_joints)
    spins = np.abs(data.qvel[roller_dofs]) if roller_dofs else np.zeros(1)
    print("\n=== %s  cmd vx=%.2f vy=%.2f wz=%.2f ===" % (label, vx, vy, wz))
    print("  body vx=%.3f vy=%.3f wz=%.3f  (world frame vx=%.3f vy=%.3f)"
          % (body[0], body[1], yaw_rate, body[0], body[1]))
    print("  wheel qvel:", wheel_rates)
    print("  roller spins: max=%.2f mean=%.2f rad/s" % (spins.max(), spins.mean()))


phase("settle", 0.0, 0.0, 0.0, 0.5)
phase("lateral +y", 0.0, 0.3, 0.0, 5.0)
phase("lateral -y", 0.0, -0.3, 0.0, 5.0)
phase("forward", 0.4, 0.0, 0.0, 5.0)
phase("yaw", 0.0, 0.0, 1.0, 5.0)
