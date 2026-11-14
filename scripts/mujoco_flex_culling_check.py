#!/usr/bin/env python3
"""Static flex culling leaves the physics unchanged (P4).

Stages a map's world like the MuJoCo bridge, adds a 0.3 m free box at the
spawn with gravity off and launches it at 3 m/s in four directions, so it
flies into the walls. Each flight runs twice, once with every world flex
colliding and once with world_collision.StaticFlexCulling, and compares
the final state bit for bit and the contact count of every step.

  python3 scripts/mujoco_flex_culling_check.py celisca_floor_1 --output result.json
"""
import argparse
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from ament_index_python.packages import get_package_share_directory

from mujoco_world_step_benchmark import spawn_of
from robot_lab_mujoco.mujoco_spawner import _stage_world_meshes
from robot_lab_mujoco.world_collision import StaticFlexCulling, configure_world_midphase


def model_for(map_name):
    text = _stage_world_meshes(Path(get_package_share_directory('robot_lab_maps'), 'mjcf', map_name + '.xml').read_text())
    root = ET.fromstring(text)
    size = root.find('size') if root.find('size') is not None else ET.SubElement(root, 'size')
    size.set('memory', '512M')       # without the midphase a wall contact tests every element of its flex
    x, y = spawn_of(map_name)
    body = ET.SubElement(root.find('worldbody'), 'body', name='probe', pos='%g %g 0.6' % (x, y))
    ET.SubElement(body, 'freejoint')
    ET.SubElement(body, 'geom', type='box', size='0.15 0.15 0.1', mass='4')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    configure_world_midphase(model, 'auto')
    model.opt.gravity[:] = 0
    return model


def flight(model, angle, culled, steps):
    data = mujoco.MjData(model)
    culling = StaticFlexCulling(model, data) if culled else None
    data.qvel[:2] = [3 * np.cos(angle), 3 * np.sin(angle)]
    contacts, enabled, flex_contacts = [], [], 0
    started = time.perf_counter()
    for _ in range(steps):
        if culling:
            enabled.append(culling.update(model, data))
        mujoco.mj_step(model, data)
        contacts.append(int(data.ncon))
        flex_contacts += int(np.sum(data.contact.flex[:data.ncon] >= 0)) if data.ncon else 0
    elapsed = time.perf_counter() - started
    if culling:
        culling.restore(model)
    return dict(qpos=data.qpos.copy(), contacts=contacts, flex_contacts=flex_contacts,
                ms=elapsed / steps * 1e3, enabled=float(np.mean(enabled)) if enabled else float(model.nflex))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('maps', nargs='+')
    parser.add_argument('--steps', type=int, default=6000)
    parser.add_argument('--output')
    options = parser.parse_args()
    result = {}
    for map_name in options.maps:
        model = model_for(map_name)
        runs = {}
        for direction in range(4):
            angle = direction * np.pi / 2 + 0.3
            full, culled = (flight(model, angle, c, options.steps) for c in (False, True))
            runs[direction] = dict(identical_qpos=bool(np.array_equal(full['qpos'], culled['qpos'])),
                                   identical_ncon=full['contacts'] == culled['contacts'],
                                   flex_contacts=full['flex_contacts'], ms_all=round(full['ms'], 3),
                                   ms_culled=round(culled['ms'], 3), mean_enabled_flexes=round(culled['enabled'], 1),
                                   final_xy=[round(float(v), 3) for v in full['qpos'][:2]])
        result[map_name] = dict(nflex=int(model.nflex), nflexelem=int(model.nflexelem), runs=runs)
        print(map_name, json.dumps(result[map_name]))
    if options.output:
        Path(options.output).write_text(json.dumps(result, indent=1) + '\n')


if __name__ == '__main__':
    main()
