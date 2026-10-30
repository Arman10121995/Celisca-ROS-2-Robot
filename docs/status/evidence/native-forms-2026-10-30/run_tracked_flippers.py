from pathlib import Path
import json,sys
from robot_lab_benchmark.concurrent_runner import ConcurrentRunner,RunSpec
root=Path('/workspace/molar/robot_lab_runtime/extension-completion-20261029-vsebubus')
runner=ConcurrentRunner(root/'tracked_flippers',1,lambda e:print(json.dumps(e,default=str),flush=True))
command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=lab_tracked_articulated',
 'simulator:=gazebo','mode:=display','map_name:=nav_empty','gui:=false','start_rviz:=false','enable_joystick:=false']
print(json.dumps(runner.run([RunSpec(command=command,task_command=[sys.executable,str(root/'tracked_flipper_workload.py')],
 duration_s=80,max_rss_mb=8000,max_cpu_percent=1800,max_log_mb=256)]),indent=2),flush=True)
