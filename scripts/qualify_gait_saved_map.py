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
    if run.get('mission_result'):
        return run['mission_result']
    # The runner can leave run.json without it; the workload writes its own (2026-11-20).
    own = Path(result['output'])/'extension-mission-result.json'
    return json.loads(own.read_text()) if own.exists() else {}


def launch(robot, world, task, extra=(), spawn=None, simulator='mujoco'):
    pose = [] if spawn is None else ['spawn_%s:=%g' % (axis, value) for axis, value in zip(('x', 'y', 'yaw'), spawn)]
    return ['ros2', 'launch', 'robot_lab_bringup', 'gait_simulator.launch.py', 'robot_model:='+robot,
            'simulator:='+simulator, 'task:='+task, 'map_name:='+world, 'gui:=false', 'rviz:=false', *pose, *extra]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--robots', nargs='+', default=['hexapod_ros2', 'lab_octopod', 'lab_centipede'])
    parser.add_argument('--map', default='nav_empty')
    parser.add_argument('--simulator', default='mujoco', choices=['mujoco', 'pybullet', 'isaac', 'gazebo'])
    parser.add_argument('--ahead', type=float, default=.5, help='goal distance along the start heading, m')
    parser.add_argument('--timeout', type=float, default=240.)
    parser.add_argument('--spawn', type=lambda s: [float(v) for v in s.split(',')],
                        help='x,y,yaw for both launches (default: the world spawn)')
    parser.add_argument('--max-rss-mb', type=float,
                        help='resource budget per run (default 4096; 12288 for Isaac, whose runtime alone exceeds 4 GB)')
    parser.add_argument('--max-cpu-percent', type=float,
                        help='CPU budget per run (default 800; 1200 for Isaac and Gazebo, whose start-up used 9-10 cores)')
    parser.add_argument('--max-log-mb', type=float,
                        help='artifact budget per run (default 64; 512 for Isaac, whose slow runs wrote 65 MB of telemetry in 7 min)')
    parser.add_argument('--map-timeout', type=float,
                        help='mapping run budget, wall s (default 120; 600 for Isaac, which reached 6 s of simulation in 120 s)')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rss = args.max_rss_mb or (12288. if args.simulator == 'isaac' else 4096.)
    cpu = args.max_cpu_percent or (1200. if args.simulator in ('isaac', 'gazebo') else 800.)
    log_mb = args.max_log_mb or (512. if args.simulator == 'isaac' else 64.)
    map_timeout = args.map_timeout or (600. if args.simulator == 'isaac' else 120.)
    args.output.mkdir(parents=True, exist_ok=False)
    runner = ConcurrentRunner(args.output/'runs', 1, lambda event: None)
    report = []
    for robot in args.robots:
        entry = dict(robot=robot, map=args.map, spawn=args.spawn, simulator=args.simulator)
        acquired = runner.run([RunSpec(command=launch(robot, args.map, 'slam', spawn=args.spawn, simulator=args.simulator), duration_s=map_timeout, max_rss_mb=rss, max_cpu_percent=cpu, max_log_mb=log_mb,
            workload=dict(schema='robot_lab.extension_workload.v1', workflow='gait', task='mapping',
                          truth_coordinates='selected_world', distance_m=.2,
                          **({'timeout_s': map_timeout-20.} if map_timeout > 320. else {})))])[0]
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
        navigated = runner.run([RunSpec(command=launch(robot, args.map, 'nav', ['map_yaml:='+artifact['map_yaml']], args.spawn, args.simulator),
            duration_s=args.timeout, max_rss_mb=rss, max_cpu_percent=cpu, max_log_mb=log_mb, workload=dict(schema='robot_lab.extension_workload.v1', workflow='gait',
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
