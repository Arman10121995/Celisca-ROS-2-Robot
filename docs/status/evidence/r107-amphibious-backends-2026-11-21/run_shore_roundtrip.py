"""The 10-30 sprung shore round trip (bank -> water -> bank) on any backend.

Usage: run_shore_roundtrip.py <simulator> <output_root>
(docs/status/evidence/amphibious-facility-2026-10-30/run_shore_roundtrip.py with the simulator and output as arguments).
"""
import json, sys
from pathlib import Path
from robot_lab_benchmark.concurrent_runner import ConcurrentRunner, RunSpec
simulator, root = sys.argv[1], Path(sys.argv[2])
runner = ConcurrentRunner(root, 1, lambda e: print(json.dumps({k: e.get(k) for k in ('id', 'state', 'mission_result_error')}, default=str), flush=True))
command = ['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py', 'robot_model:=lab_amphibious', 'simulator:=' + simulator,
           'mode:=marine', 'map_name:=amphibious_shore', 'gui:=false', 'start_rviz:=false', 'marine_slam:=false']
print(json.dumps(runner.run([RunSpec(command=command, duration_s=300, max_rss_mb=8000, max_cpu_percent=1800, max_log_mb=256,
    workload=dict(schema='robot_lab.extension_workload.v1', workflow='marine', task='navigation', truth_coordinates='selected_world',
                  timeout_s=260., goals=[[6.3, 0., .134], [-5.8, 0., .74]]))]), default=str), flush=True)
