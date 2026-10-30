from pathlib import Path
import json
from robot_lab_benchmark.concurrent_runner import ConcurrentRunner,RunSpec
root=Path('/workspace/molar/robot_lab_runtime/amphibious-infrastructure-20261030-mcpiqquv')
command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=lab_amphibious','simulator:=gazebo','mode:=marine','map_name:=amphibious_shore','gui:=false','start_rviz:=false','marine_slam:=true']
runner=ConcurrentRunner(root/'amphibious_mapping',1,lambda e:print(json.dumps(e,default=str),flush=True))
print(json.dumps(runner.run([RunSpec(command=command,duration_s=130,max_rss_mb=8000,max_cpu_percent=1800,max_log_mb=256,workload=dict(schema='robot_lab.extension_workload.v1',workflow='marine',task='mapping',truth_coordinates='selected_world',timeout_s=95.,distance_m=.4))]),indent=2),flush=True)
