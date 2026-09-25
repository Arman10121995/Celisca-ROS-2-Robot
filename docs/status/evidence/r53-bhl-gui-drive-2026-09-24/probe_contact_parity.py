#!/usr/bin/env python3
"""Compare the compiled common BHL MuJoCo plant with the native qualifier plant."""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[4]
ROBOT = REPO / "src/robot_lab_robots/berkeley_humanoid_lite"
sys.path[:0] = [str(ROBOT / "tools"), str(REPO / "src/robot_lab_adapter")]

from qualify_policy_adapter import load_model  # noqa: E402
from robot_lab_mujoco.joint_effort import JointEffortCommand  # noqa: E402
from robot_lab_mujoco.mujoco_spawner import (  # noqa: E402
    MuJoCoSpawner, _build_mjcf_from_urdf, _native_joint_dynamics,
    _xacro_to_urdf)


def unique_rows(values):
    array = np.asarray(values)
    if array.ndim == 1:
        array = array.reshape(1, -1)
    rows = sorted({tuple(np.round(row, 7)) for row in array})
    return [list(map(float, row)) for row in rows]


def snapshot(model):
    import mujoco
    names = {mujoco_name(model, i, mujoco.mjtObj.mjOBJ_BODY): i
             for i in range(model.nbody)
             if mujoco_name(model, i, mujoco.mjtObj.mjOBJ_BODY)}
    static = [i for i in range(model.ngeom) if model.geom_bodyid[i] == 0]
    robot = [i for i in range(model.ngeom) if model.geom_bodyid[i] != 0]
    return {
        "counts": {"nbody": model.nbody, "njnt": model.njnt, "ngeom": model.ngeom,
                    "nu": model.nu, "nq": model.nq, "nv": model.nv},
        "options": {"timestep": float(model.opt.timestep),
                    "gravity": unique_rows(model.opt.gravity),
                    "integrator": int(model.opt.integrator),
                    "solver": int(model.opt.solver), "cone": int(model.opt.cone),
                    "impratio": float(model.opt.impratio)},
        "static_geom": [{"type": int(model.geom_type[i]),
                          "contype": int(model.geom_contype[i]),
                          "conaffinity": int(model.geom_conaffinity[i]),
                          "friction": model.geom_friction[i].tolist(),
                          "solref": model.geom_solref[i].tolist(),
                          "solimp": model.geom_solimp[i].tolist()} for i in static],
        "robot_geom_unique": {
            "type": unique_rows(model.geom_type[robot]),
            "friction": unique_rows(model.geom_friction[robot]),
            "solref": unique_rows(model.geom_solref[robot]),
            "solimp": unique_rows(model.geom_solimp[robot]),
        },
        "bodies": {name: {"mass": float(model.body_mass[i]),
                           "inertia": model.body_inertia[i].tolist(),
                           "ipos": model.body_ipos[i].tolist()}
                    for name, i in names.items() if name},
    }


def mujoco_name(model, index, kind):
    import mujoco
    return mujoco.mj_id2name(model, kind, index)


def build_common():
    import mujoco
    from ament_index_python.packages import get_package_share_directory
    from robot_lab_mujoco.mujoco_spawner import _native_joint_dynamics

    robot_share = Path(get_package_share_directory("robot_lab_robots"))
    model_path = robot_share / "berkeley_humanoid_lite/xacro/bhl_sim.xacro"
    config_path = robot_share / "berkeley_humanoid_lite/config/bhl_controllers.yaml"
    urdf = _xacro_to_urdf(str(model_path))
    pkg_map = {"robot_lab_robots": str(robot_share)}
    robot_mjcf = _build_mjcf_from_urdf(
        urdf, pkg_map, logger=None, robot_name="bhl",
        base_dir=str(model_path.parent))
    command = JointEffortCommand(str(config_path), urdf)
    _, native = _native_joint_dynamics(str(model_path), command.names)
    robot_mjcf = command.add_actuators(robot_mjcf, native)
    world_path = Path(get_package_share_directory("robot_lab_maps")) / "mjcf/nav_empty.xml"
    merged = MuJoCoSpawner._merge_mjcf(str(world_path), robot_mjcf, logger=None)
    return mujoco.MjModel.from_xml_string(merged)


def passive_trace(model):
    import mujoco
    data = mujoco.MjData(model)
    data.qpos[2] = -0.038
    mujoco.mj_forward(model, data)
    rows = []
    for step in range(round(2.0 / model.opt.timestep)):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        if step % max(1, round(0.02 / model.opt.timestep)) == 0:
            contacts = []
            for contact in data.contact:
                contacts.append(float(contact.dist))
            rows.append({"t": float(data.time), "z": float(data.qpos[2]),
                         "contacts": len(contacts),
                         "max_penetration": max((abs(d) for d in contacts), default=0.0)})
    return rows


def main():
    native_path, native, _ = load_model(ROBOT, "policy_humanoid")
    common = build_common()
    native.opt.timestep = 0.0005
    common.opt.timestep = 0.0005
    result = {"native_model": str(native_path),
              "common_model": "installed bhl_sim.xacro + nav_empty.xml",
              "native": snapshot(native), "common": snapshot(common),
              "native_passive": passive_trace(native),
              "common_passive": passive_trace(common)}
    result["colliding_static_equal"] = (
        [geom for geom in result["native"]["static_geom"]
         if geom["contype"] or geom["conaffinity"]]
        == [geom for geom in result["common"]["static_geom"]
            if geom["contype"] or geom["conaffinity"]])
    result["material_equal"] = (
        result["colliding_static_equal"]
        and result["native"]["robot_geom_unique"] == result["common"]["robot_geom_unique"])
    result["body_differences"] = sorted(set(result["native"]["bodies"]) ^ set(result["common"]["bodies"]))
    result["body_value_differences"] = [
        name for name in set(result["native"]["bodies"]) & set(result["common"]["bodies"])
        if result["native"]["bodies"][name] != result["common"]["bodies"][name]]
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("contact_parity.json")
    out.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"material_equal": result["material_equal"],
                      "body_differences": result["body_differences"],
                      "body_value_differences": result["body_value_differences"],
                      "native_options": result["native"]["options"],
                      "common_options": result["common"]["options"]}, indent=2))


if __name__ == "__main__":
    main()
