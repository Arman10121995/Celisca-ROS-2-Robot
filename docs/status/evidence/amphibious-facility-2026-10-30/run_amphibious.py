import sys,json
from pathlib import Path
from robot_lab_benchmark.concurrent_runner import ConcurrentRunner,RunSpec
root=Path('/workspace/molar/robot_lab_runtime/amphibious-infrastructure-20261030-mcpiqquv')
command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=lab_amphibious','simulator:=gazebo','mode:=marine','map_name:=amphibious_shore','gui:=false','start_rviz:=false','enable_joystick:=false','marine_slam:=false']
runner=ConcurrentRunner(root/'physical_transition',1,lambda e:print(json.dumps(e,default=str),flush=True));print(json.dumps(runner.run([RunSpec(command=command,task_command=[sys.executable,str(root/'amphibious_trial.py')],duration_s=240,max_rss_mb=8000,max_cpu_percent=1800,max_log_mb=256)]),indent=2),flush=True)
