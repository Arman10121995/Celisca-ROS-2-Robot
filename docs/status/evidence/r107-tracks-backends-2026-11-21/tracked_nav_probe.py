"""One Nav2 goal for lab_tracked through the normal launch, measured by truth.

Usage: LAB_EXTENSION_RUN=<dir> tracked_nav_probe.py <simulator> <map> <goal_x> <goal_y> [duration_s]
(the October 27 tracked_nav_probe.py with the simulator, map and goal as arguments).
"""
import json, os, sys, time
from pathlib import Path
from robot_lab_benchmark.concurrent_runner import ConcurrentRunner, RunSpec
simulator, world, gx, gy = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
duration = float(sys.argv[5]) if len(sys.argv) > 5 else 240.
root = Path(os.environ['LAB_EXTENSION_RUN']) / ('tracked-navigation-%s-%s-%d' % (simulator, world, time.time_ns()))
spec = RunSpec(command=['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py', 'robot_model:=lab_tracked',
                        'mode:=nav', 'map_name:=' + world, 'simulator:=' + simulator, 'gui:=false', 'start_rviz:=false',
                        'enable_joystick:=false'],
               duration_s=duration, max_rss_mb=8000, max_cpu_percent=1600, max_log_mb=256,
               workload=dict(schema='robot_lab.nav2_workload.v1', truth_coordinates='selected_world', goals=[[gx, gy, 0.]]),
               selection=dict(robot='lab_tracked', map=world, backend=simulator, mode='nav'))
r = ConcurrentRunner(root, 1, lambda r: print(json.dumps({k: r.get(k) for k in ('id', 'state', 'mission_result_error')}, default=str), flush=True)).run([spec])
print(json.dumps(r, default=str), flush=True)
