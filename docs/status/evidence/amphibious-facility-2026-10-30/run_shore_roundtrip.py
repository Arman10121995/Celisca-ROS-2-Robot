import json
from pathlib import Path
from robot_lab_benchmark.concurrent_runner import ConcurrentRunner,RunSpec
root=Path('/workspace/molar/robot_lab_runtime/amphibious-infrastructure-20261030-mcpiqquv');runner=ConcurrentRunner(root/'shore_roundtrip',1,lambda e:print(json.dumps(e,default=str),flush=True))
command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=lab_amphibious','simulator:=gazebo','mode:=marine','map_name:=amphibious_shore','gui:=false','start_rviz:=false','marine_slam:=false']
print(json.dumps(runner.run([RunSpec(command=command,duration_s=300,max_rss_mb=8000,max_cpu_percent=1800,max_log_mb=256,workload=dict(schema='robot_lab.extension_workload.v1',workflow='marine',task='navigation',truth_coordinates='selected_world',timeout_s=260.,goals=[[6.3,0.,.134],[-5.8,0.,.74]]))]),indent=2),flush=True)
