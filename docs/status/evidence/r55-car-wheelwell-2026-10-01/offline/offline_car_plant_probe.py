#!/usr/bin/env python3
"""Offline MuJoCo diagnosis of the ackermann_car drive plant.

Builds the exact MJCF the ROS spawner builds (wheel velocity + steering
position actuators), then drives straight and through the `left_arc` command,
printing body speed, wheel rates, actuator forces, steering angles and wheel
contact forces.  No ROS.
"""
import os
import re
import sys

import yaml

SRC = "/home/molar1/bumperbot_ws/src"
sys.path[:0] = [os.path.join(SRC, "robot_lab_mujoco", "python"),
                os.path.join(SRC, "robot_lab_utils")]

import mujoco  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from robot_lab_mujoco import mujoco_spawner as sp  # noqa: E402
from robot_lab_utils.drive_kinematics import drive_from_config  # noqa: E402

ROBOTS = os.path.join(SRC, "robot_lab_robots", "config", "robots.yaml")
ROBOT = sys.argv[1] if len(sys.argv) > 1 else "ackermann_car"


def robot_mjcf(robot_id):
    config = yaml.safe_load(open(ROBOTS))["robots"][robot_id]
    path = os.path.join(get_package_share_directory("robot_lab_robots"),
                        config["xacro"])
    urdf = sp._xacro_to_urdf(path)
    packages = {}
    for package in set(re.findall(r"package://([^/]+)/", urdf)):
        try:
            packages[package] = get_package_share_directory(package)
        except Exception:
            pass
    return sp._build_mjcf_from_urdf(urdf, packages, logger=None,
                                    robot_name=robot_id,
                                    base_dir=os.path.dirname(os.path.abspath(path)))


config = yaml.safe_load(open(ROBOTS))["robots"][ROBOT]
drive = drive_from_config(config.get("drive") or {})

mjcf = robot_mjcf(ROBOT)
mjcf = sp._add_wheel_velocity_actuators(mjcf, drive.wheel_joints)
mjcf = sp._add_steer_position_actuators(mjcf, drive.steer_joints)
world = os.path.join(get_package_share_directory("robot_lab_maps"), "mjcf",
                     "nav_empty.xml")
model = mujoco.MjModel.from_xml_string(
    sp.MuJoCoSpawner._merge_mjcf(world, mjcf))
data = mujoco.MjData(model)

free = [j for j in range(model.njnt)
        if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE][0]
qadr, vadr = model.jnt_qposadr[free], model.jnt_dofadr[free]
data.qpos[qadr + 2] = 0.05          # wheel-centre height, wheels on the floor

def act(name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

actuators = {j: {"velocity": act(j + "_velocity"), "position": act(j + "_position")}
             for j in list(drive.wheel_joints) + list(drive.steer_joints)}

def geom_id(name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)

# Wheel collision geoms after fixed-link fusion.
wheel_geoms = {}
for gid in range(model.ngeom):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid)
    if name and ("wheel" in name or "roller" in name):
        wheel_geoms[gid] = name
print("wheel geoms:", wheel_geoms)
for gid in wheel_geoms:
    print("  friction", wheel_geoms[gid], model.geom_friction[gid])

import numpy as np

def wheel_contact_forces():
    total = 0.0
    force = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        if c.geom1 in wheel_geoms or c.geom2 in wheel_geoms:
            mujoco.mj_contactForce(model, data, i, force)
            total += float(force[0])
    return total

def drive_phase(vx, wz, duration, label):
    drive.reset()
    n = int(duration / model.opt.timestep)
    for _ in range(n):
        targets = drive.targets(vx, wz, dt=model.opt.timestep)
        for joint, rate in targets.velocity.items():
            data.ctrl[actuators[joint]["velocity"]] = max(-50.0, min(50.0, rate))
        for joint, angle in targets.position.items():
            data.ctrl[actuators[joint]["position"]] = angle
        mujoco.mj_step(model, data)
    body_vx = data.qvel[vadr + 0]
    body_wz = data.qvel[vadr + 5]
    print("\n=== %s cmd=(%.2f, %.2f) ===" % (label, vx, wz))
    print("body vx=%.3f wz=%.3f  contacts=%d wheel_normal=%.1f N"
          % (body_vx, body_wz, data.ncon, wheel_contact_forces()))
    for joint in list(drive.steer_joints) + list(drive.wheel_joints):
        if joint in drive.steer_joints:
            adr = drive.steer_joints.index(joint)
        else:
            adr = 0
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        q = data.qpos[model.jnt_qposadr[jid]]
        v = data.qvel[model.jnt_dofadr[jid]]
        f = data.qfrc_actuator[model.jnt_dofadr[jid]]
        print("  %-28s qpos=%7.4f qvel=%7.3f qfrc_actuator=%7.3f"
              % (joint, q, v, f))


drive_phase(0.4, 0.0, 5.0, "straight")

# Stationary steering sweep: does the servo reach the target at standstill?
def static_steer(fl, fr, duration, label):
    n = int(duration / model.opt.timestep)
    for _ in range(n):
        data.ctrl[actuators[drive.steer_joints[0]]["position"]] = fl
        data.ctrl[actuators[drive.steer_joints[1]]["position"]] = fr
        for joint in drive.wheel_joints:
            data.ctrl[actuators[joint]["velocity"]] = 0.0
        mujoco.mj_step(model, data)
    print("\n=== %s (targets %.3f / %.3f) ===" % (label, fl, fr))
    for joint in drive.steer_joints:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        q = data.qpos[model.jnt_qposadr[jid]]
        f = data.qfrc_actuator[model.jnt_dofadr[jid]]
        print("  %-28s qpos=%7.4f qfrc_actuator=%7.3f" % (joint, q, f))
    print("  contacts=%d wheel_normal=%.1f N" % (data.ncon, wheel_contact_forces()))

static_steer(0.4516, 0.3281, 4.0, "stationary steer")

# Where does the steering resisting torque come from?
print("\n--- constraint/passive audit ---")
for joint in drive.steer_joints:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
    adr = model.jnt_dofadr[jid]
    print("  %s: qfrc_actuator=%.3f qfrc_constraint=%.3f qfrc_passive=%.3f "
          "qfrc_applied=%.3f dof_damping=%.4f dof_frictionloss=%.4f"
          % (joint, data.qfrc_actuator[adr], data.qfrc_constraint[adr],
             data.qfrc_passive[adr], data.qfrc_applied[adr],
             model.dof_damping[adr], model.dof_frictionloss[adr]))
force = np.zeros(6)
print("\n--- all contacts after static steer (geom idx, names, pos, dist, force) ---")
for i in range(data.ncon):
    c = data.contact[i]
    n1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
    n2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
    mujoco.mj_contactForce(model, data, i, force)
    print("  [%d] g%d(%s) | g%d(%s) pos=%s dist=%+.5f f=%s"
          % (i, c.geom1, n1, c.geom2, n2, np.round(c.pos, 4), c.dist,
             np.round(force[:3], 3)))
print("geom.bodyid for the two box-pad candidates:",
      model.geom_bodyid[0], model.geom_bodyid[1])
for i in range(model.ngeom):
    print("  g%d name=%s type=%d group=%d contype=%d conaff=%d"
          % (i, mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i),
             model.geom_type[i], model.geom_group[i], model.geom_contype[i],
             model.geom_conaffinity[i]))

drive_phase(0.4, 0.5, 6.0, "left_arc")



# Report the (lateral, spin) force state on one front wheel at the arc.
print("\nmodel timestep", model.opt.timestep, "solver", model.opt.solver)
