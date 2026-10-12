#!/usr/bin/env python3
"""Native MuJoCo step cost of a map's static world, as the bridge builds it.

Stages the map's generated MJCF exactly like the MuJoCo spawner
(`_stage_world_meshes`: mesh conversion plus rigid-flex/static collision),
drops a 0.3 m free test box at the map spawn, and reports native timers per
step.  No ROS, robot controllers or sensors run; the result isolates the
world's per-step physics cost (P4).

  python3 scripts/mujoco_world_step_benchmark.py celisca_floor_1 nav_obstacle ...
"""
import argparse
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import yaml
from ament_index_python.packages import get_package_share_directory

from robot_lab_mujoco.mujoco_spawner import _stage_world_meshes
from robot_lab_mujoco.world_collision import configure_world_midphase


def spawn_of(map_name):
    maps = yaml.safe_load(Path(get_package_share_directory('robot_lab_bringup'), 'config',
                               'sim_maps.yaml').read_text())
    entry = (maps.get('maps') or maps).get(map_name, {})
    spawn = entry.get('spawn', {})
    return [float(spawn.get(axis, 0.0)) for axis in ('x', 'y')]


def measure(map_name, steps, midphase='auto'):
    path = Path(get_package_share_directory('robot_lab_maps'), 'mjcf', map_name+'.xml')
    started = time.perf_counter()
    text = _stage_world_meshes(path.read_text())
    staged_s = time.perf_counter()-started
    root = ET.fromstring(text)
    x, y = spawn_of(map_name)
    body = ET.SubElement(root.find('worldbody'), 'body', name='probe', pos='%g %g 0.2' % (x, y))
    ET.SubElement(body, 'freejoint')
    ET.SubElement(body, 'geom', type='box', size='0.15 0.15 0.1', mass='4')
    started = time.perf_counter()
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    compile_s = time.perf_counter()-started
    midphase_disabled = configure_world_midphase(model, midphase)
    data = mujoco.MjData(model)
    for _ in range(50):
        mujoco.mj_step(model, data)
    timers = {name: int(index) for name, index in mujoco.mjtTimer.__members__.items() if name != 'mjNTIMER'}
    before = {name: float(data.timer[index].duration) for name, index in timers.items()}
    previous = mujoco.get_mjcb_time()
    mujoco.set_mjcb_time(time.perf_counter)
    try:
        wall = time.perf_counter()
        for _ in range(steps):
            mujoco.mj_step(model, data)
        wall = time.perf_counter()-wall
    finally:
        mujoco.set_mjcb_time(previous)
    per = {name.replace('mjTIMER_', '').lower(): (float(data.timer[index].duration)-before[name])*1000/steps
           for name, index in timers.items()}
    return dict(map=map_name, engine=mujoco.__version__, stage_s=round(staged_s, 2),
                compile_s=round(compile_s, 2), timestep=model.opt.timestep,
                ngeom=model.ngeom, nmesh=model.nmesh, nflex=model.nflex, nflexelem=model.nflexelem,
                midphase_disabled=midphase_disabled,
                step_ms=round(wall*1000/steps, 3),
                physics_rtf_bound=round(model.opt.timestep/(wall/steps), 2),
                kinematics_ms=round(per['pos_kinematics'], 3), broadphase_ms=round(per['col_broad'], 3),
                narrowphase_ms=round(per['col_narrow'], 3), probe_z=round(float(data.qpos[2]), 3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('maps', nargs='+')
    parser.add_argument('--steps', type=int, default=300)
    parser.add_argument('--midphase', default='auto', choices=('auto', 'on', 'off'))
    parser.add_argument('--output')
    args = parser.parse_args()
    results = []
    for name in args.maps:
        try:
            result = measure(name, args.steps, args.midphase)
        except Exception as exc:  # report and continue with the next map
            result = dict(map=name, error=str(exc))
        print(json.dumps(result), flush=True)
        results.append(result)
    if args.output:
        Path(args.output).write_text(json.dumps(results, indent=1)+'\n')


if __name__ == '__main__':
    main()
