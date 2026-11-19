#!/usr/bin/env python3
"""Saved-map Nav2 goals for the multi-leg gait models (R10.6).

For each model, two isolated runs through the measured Experiments runner
(``ConcurrentRunner`` with the same extension workloads as **Gait map
acquisition** and **Gait navigation**):

1. ``gait_simulator task:=slam``: physical travel and an archived acquired
   occupancy map (``/gait/mapping/save``);
2. ``gait_simulator task:=nav map_yaml:=<that map>``: a Nav2 goal ``--ahead``
   metres along the measured start heading, judged by the workload's
   independent settled physical endpoint.

  python3 scripts/qualify_gait_saved_map.py --output /workspace/molar/robot_lab_runtime/gait-saved-map \\
      --robots hexapod_ros2 lab_octopod lab_centipede
"""
import argparse
import json
import math
from pathlib import Path

from robot_lab_benchmark.concurrent_runner import ConcurrentRunner, RunSpec


def mission(result):
    path = Path(result['output'])/'run.json'
    run = json.loads(path.read_text()) if path.exists() else result
    return run.get('mission_result') or {}


def launch(robot, world, task, extra=()):
    return ['ros2', 'launch', 'robot_lab_bringup', 'gait_simulator.launch.py', 'robot_model:='+robot,
            'task:='+task, 'map_name:='+world, 'gui:=false', 'rviz:=false', *extra]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--robots', nargs='+', default=['hexapod_ros2', 'lab_octopod', 'lab_centipede'])
    parser.add_argument('--map', default='nav_empty')
    parser.add_argument('--ahead', type=float, default=.5, help='goal distance along the start heading, m')
    parser.add_argument('--timeout', type=float, default=240.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    runner = ConcurrentRunner(args.output/'runs', 1, lambda event: None)
    report = []
    for robot in args.robots:
        entry = dict(robot=robot, map=args.map)
        acquired = runner.run([RunSpec(command=launch(robot, args.map, 'slam'), duration_s=120.,
            workload=dict(schema='robot_lab.extension_workload.v1', workflow='gait', task='mapping',
                          truth_coordinates='selected_world', distance_m=.2))])[0]
        result = mission(acquired)
        entry['acquisition'] = dict(run=acquired['output'], success=result.get('success'), reason=result.get('reason'))
        measured = (result.get('measurements') or [{}])[0]
        artifact = measured.get('map_artifact') or {}
        if not result.get('success') or not artifact.get('map_yaml'):
            report.append(entry)
            continue
        x, y, yaw = measured['physical_pose']
        goal = [x+args.ahead*math.cos(yaw), y+args.ahead*math.sin(yaw), yaw]
        entry['acquisition'].update(map_yaml=artifact['map_yaml'], known_cells=artifact.get('known_cells'),
                                    start=measured['physical_pose'])
        navigated = runner.run([RunSpec(command=launch(robot, args.map, 'nav', ['map_yaml:='+artifact['map_yaml']]),
            duration_s=args.timeout, workload=dict(schema='robot_lab.extension_workload.v1', workflow='gait',
            task='navigation', truth_coordinates='selected_world', goals=[goal], timeout_s=args.timeout-20.))])[0]
        result = mission(navigated)
        entry['navigation'] = dict(run=navigated['output'], goal=goal, success=result.get('success'),
                                   reason=result.get('reason'), measurements=result.get('measurements'))
        report.append(entry)
        (args.output/'report.json').write_text(json.dumps(report, indent=1, default=str)+'\n')
    (args.output/'report.json').write_text(json.dumps(report, indent=1, default=str)+'\n')
    for entry in report:
        print(entry['robot'], entry['acquisition'].get('success'), (entry.get('navigation') or {}).get('success'),
              (entry.get('navigation') or {}).get('reason'))


if __name__ == '__main__':
    main()
