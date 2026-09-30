#!/usr/bin/env python3
"""Which robot geoms carry a settled Go2 rest, and do the topics cover them?

/go2/foot_contact_forces publishes the four ``*_foot_contact_0`` geoms and
/go2/trunk_contact_forces publishes ``trunk_contact_0``.  Consumers read the sum
of the two as "how the ground is holding the robot", so that sum is only
meaningful if feet and trunk are the geoms that carry the load.  A real trial
disagreed: after its landing, the settled flank read 23-85 N -- 45.8 N on one
sample, 25.4 N on the last -- of a 126.5 N robot, with the trunk at 0.0 N
throughout.

Replaying a recorded pose cannot answer this -- pinning the root removes the
very reaction being measured, and releasing it finds a different equilibrium.
So this probe does not replay.  It drops the plant from a range of roll angles
and lets it come to rest under gravity with the same joint hold the spawner
applies, then partitions the settled normal force over *every* robot geom.  The
result is a per-geom support table and a coverage fraction for the two topics,
both measured at genuine static equilibrium.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(REPO / "src/robot_lab_mujoco/python")]

import mujoco  # noqa: E402

from robot_lab_mujoco.mujoco_spawner import (  # noqa: E402
    MuJoCoSpawner, _add_joint_hold_springs, _build_mjcf_from_urdf,
    _native_joint_dynamics, _world_normal_contact_force, _xacro_to_urdf)

FOOT_LEGS = ("FL", "FR", "RL", "RR")
PUBLISHED = tuple(f"{leg}_foot_contact_0" for leg in FOOT_LEGS) + ("trunk_contact_0",)
SETTLE_SECONDS = 4.0
DROP_HEIGHT_M = 0.4
# The roll angles the fall ladder actually drove the robot to: both settled
# flanks (-0.5 is the -0.520 rad rest the brace trials reached), the 0.5/0.9
# side rests the entry analysis uses, 1.4, the 2.4 rad inverted-tilt threshold
# and fully inverted.  Upright is the control.
ROLES = (-0.5, 0.0, 0.5, 0.9, 1.4, 2.4, 3.14159265)


def build_model():
    """The plant exactly as mujoco_spawner builds it for a Go2 run."""
    from ament_index_python.packages import get_package_share_directory
    from robot_lab_mujoco.joint_effort import JointEffortCommand

    robots = Path(get_package_share_directory("robot_lab_robots"))
    xacro = robots / "unitree/go2_description/xacro/go2_sim.xacro"
    config = robots / "unitree/go2_description/config/go2_controllers.yaml"
    urdf = _xacro_to_urdf(str(xacro))
    robot_mjcf = _build_mjcf_from_urdf(urdf, {"robot_lab_robots": str(robots)},
                                       logger=None, robot_name="go2",
                                       base_dir=str(xacro.parent))
    command = JointEffortCommand(str(config), urdf)
    _, native = _native_joint_dynamics(str(xacro), command.names)
    world = Path(get_package_share_directory("robot_lab_maps")) / "mjcf/nav_empty.xml"
    merged = MuJoCoSpawner._merge_mjcf(str(world), command.add_actuators(robot_mjcf, native),
                                       logger=None)
    return mujoco.MjModel.from_xml_string(merged), command.names


def geom_inventory(model):
    """Every colliding robot geom, grouped by owning body."""
    inventory = defaultdict(list)
    for index in range(model.ngeom):
        if model.geom_bodyid[index] == 0:
            continue
        if not (model.geom_contype[index] or model.geom_conaffinity[index]):
            continue  # a visual-only geom cannot carry support
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[index]))
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, index) or f"<{index}>"
        inventory[body].append(name)
    return dict(inventory)


def partition(model, data):
    """Normal force [N] against the world, per geom, grouped by owning body."""
    per_body = defaultdict(float)
    per_geom = defaultdict(float)
    contact_force = np.zeros(6)
    for index in range(data.ncon):
        contact = data.contact[index]
        for own, other in ((int(contact.geom1), int(contact.geom2)),
                           (int(contact.geom2), int(contact.geom1))):
            if model.geom_bodyid[other] != 0:
                continue  # contact with another robot part, not with the world
            mujoco.mj_contactForce(model, data, index, contact_force)
            geom = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, own) or f"<{own}>"
            body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY,
                                     int(model.geom_bodyid[own]))
            normal = max(0.0, float(contact_force[0]))
            per_geom[geom] += normal
            per_body[body] += normal
            break
    return per_body, per_geom


def settle(model, joint_names, roll_rad):
    """Drop the plant at ``roll_rad`` and step until it stops moving.

    The joint hold is the spawner's own ``_add_joint_hold_springs``, so the legs
    keep a stance instead of collapsing and the rest is a real equilibrium the
    ground produced, not an artefact of released actuators.
    """
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[2] = DROP_HEIGHT_M  # clear of the floor, then fall
    quat = np.zeros(4)
    mujoco.mju_euler2Quat(quat, np.array([roll_rad, 0.0, 0.0]), "xyz")
    data.qpos[3:7] = quat
    mujoco.mj_forward(model, data)
    _add_joint_hold_springs(model, data, joint_names)
    mujoco.mj_forward(model, data)
    steps = int(SETTLE_SECONDS / model.opt.timestep)
    for _ in range(steps):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)
    return data


def report(model, joint_names):
    """Settle at every role in turn; print the table and return the record."""
    foot_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{leg}_foot_contact_0")
                for leg in FOOT_LEGS]
    trunk_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "trunk_contact_0")]
    weight = sum(model.body_mass[i] for i in range(model.nbody)) * abs(model.opt.gravity[2])

    print(f"robot weight: {weight:.1f} N\n")
    header = (f"{'roll':>6} {'contact':>9} {'published':>10} {'hidden':>8} "
              f"{'cover':>7} {'qvel':>6}  resting on")
    print(header)
    print("-" * len(header))
    rests = []
    for roll in ROLES:
        data = settle(model, joint_names, roll)
        per_body, per_geom = partition(model, data)
        total = sum(per_geom.values())
        published = (_world_normal_contact_force(model, data, foot_ids)
                     + _world_normal_contact_force(model, data, trunk_ids))
        hidden = {g: round(f, 2) for g, f in per_geom.items()
                  if g not in PUBLISHED and f > 0.05}
        cover = 100.0 * published / total if total > 0.05 else 100.0
        qvel = float(np.max(np.abs(data.qvel)))
        carriers = ", ".join(
            f"{body} {force:.0f}N" for body, force in
            sorted(per_body.items(), key=lambda kv: -kv[1])[:3])
        shown = carriers + ("  [hidden: " + ", ".join(
            f"{g} {f:.0f}N" for g, f in hidden.items()) + "]" if hidden else "")
        print(f"{roll:6.2f} {total:8.1f}N {published:9.1f}N {total - published:7.1f}N "
              f"{cover:6.1f}% {qvel:6.3f}  {shown}")
        rests.append({
            "roll_rad": roll,
            "contact_n": round(total, 2),
            "published_n": round(published, 2),
            "hidden_n": round(total - published, 2),
            "coverage_percent": round(cover, 1),
            "max_abs_qvel": round(qvel, 4),
            "per_body_n": {body: round(force, 2)
                           for body, force in sorted(per_body.items(),
                                                     key=lambda kv: -kv[1])
                           if force > 0.05},
            "per_geom_n": {geom: round(force, 2)
                           for geom, force in sorted(per_geom.items(),
                                                     key=lambda kv: -kv[1])
                           if force > 0.05},
            "hidden_geoms_n": hidden,
        })
    worst = min(rests, key=lambda rest: rest["coverage_percent"])
    print(f"\nworst topic coverage: {worst['coverage_percent']:.1f}% of settled "
          f"support at roll {worst['roll_rad']:.2f} rad")
    return {"robot_weight_n": round(weight, 2), "rests": rests,
            "worst_roll_rad": worst["roll_rad"],
            "worst_coverage_percent": worst["coverage_percent"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    model, joint_names = build_model()
    inventory = geom_inventory(model)
    print("colliding robot geoms by body (published ones in brackets):")
    for body, geoms in sorted(inventory.items()):
        marked = ", ".join(f"[{g}]" if g in PUBLISHED else g for g in geoms)
        print(f"  {str(body):<12} {marked}")
    hidden = sum(1 for geoms in inventory.values() for g in geoms if g not in PUBLISHED)
    print(f"\n{hidden} colliding geoms publish no force; the two topics cover "
          f"{len(PUBLISHED)} of {hidden + len(PUBLISHED)}\n")
    measured = report(model, joint_names)
    summary = {
        "task": "R5.2 Go2 settled-rest support attribution",
        "settle_seconds": SETTLE_SECONDS,
        "drop_height_m": DROP_HEIGHT_M,
        "published_geoms": list(PUBLISHED),
        "colliding_geoms_by_body": {
            str(body): sorted(geoms) for body, geoms in sorted(inventory.items())},
        "measured": measured,
        "interpretation": {
            "grain": (
                "Each rest is a drop from %.1f m at the named roll, settled for "
                "%g s under the spawner's own joint-hold springs, then partitioned "
                "by normal force against static world geoms over every colliding "
                "robot geom. published is the sum the two contact topics carry; "
                "hidden is what they miss." % (DROP_HEIGHT_M, SETTLE_SECONDS)),
            "limitations": [
                "These are simulated equilibria from a dropped free base, not "
                "replays of the recorded trials: pinning the root would remove "
                "the very reaction being measured, and releasing it finds a "
                "different equilibrium.",
                "The joint-hold springs are the spawner's stance assist, so the "
                "legs hold a stance instead of collapsing; a different assist "
                "would settle differently.",
                "Support is the normal component against the world only, as in "
                "the published topics; tangential friction and robot "
                "self-contact are excluded, so the sum is a support partition, "
                "not a full force balance.",
                "max_abs_qvel is the settle check: a large value would mean the "
                "reported rest is still moving and its partition is not an "
                "equilibrium.",
            ],
        },
    }
    out_path = args.out or (
        Path(__file__).resolve().parent / "support_attribution.json")
    out_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(f"support attribution written to {out_path}")


if __name__ == "__main__":
    main()
