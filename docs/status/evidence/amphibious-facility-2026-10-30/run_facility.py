import sys,json
from pathlib import Path
from robot_lab_benchmark.concurrent_runner import ConcurrentRunner,RunSpec
root=Path('/workspace/molar/robot_lab_runtime/amphibious-infrastructure-20261030-mcpiqquv');runner=ConcurrentRunner(root/'facility_mission',1,lambda e:print(json.dumps(e,default=str),flush=True))
command=['ros2','launch','robot_lab_bringup','shared_world.launch.py','session:='+str(root/'facility-session.yaml'),'navigation:=true','gui:=false','rviz:=false']
print(json.dumps(runner.run([RunSpec(command=command,task_command=[sys.executable,str(root/'facility_trial.py')],duration_s=265,max_rss_mb=10000,max_cpu_percent=1800,max_log_mb=256)]),indent=2),flush=True)
