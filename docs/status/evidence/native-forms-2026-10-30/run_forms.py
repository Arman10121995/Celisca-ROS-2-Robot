import json,os,sys
from pathlib import Path
from robot_lab_benchmark.concurrent_runner import ConcurrentRunner,RunSpec
root=Path('/workspace/molar/robot_lab_runtime/extension-completion-20261029-vsebubus')
kind=sys.argv[1]
if kind.startswith('snake_'):
 import time
 os.environ['ROBOT_LAB_MUJOCO_DUMP_XML']=str(root/('compiled-snake-'+str(time.time_ns())+'.xml'))
 mapping=kind.endswith('mapping')
 command=['ros2','launch','robot_lab_bringup','gait_simulator.launch.py','robot_model:=lab_snake',
  'task:='+('3d_slam' if kind=='snake_3d_mapping' else 'slam' if mapping else 'nav'),'map_name:=nav_empty','gui:=false','rviz:=false']
 if os.environ.get('LAB_SAVED_MAP'):command.append('map_yaml:='+os.environ['LAB_SAVED_MAP'])
 workload=dict(schema='robot_lab.extension_workload.v1',workflow='gait',task='mapping' if mapping else 'navigation',truth_coordinates='selected_world',timeout_s=180.)
 if mapping:workload['distance_m']=.15
 else:workload['goals']=[[0.,-.5,0.]]
else:
 command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=lab_usv',
  'mode:=marine','simulator:=gazebo','map_name:=surface_harbor','gui:=false','start_rviz:=false',
  'enable_joystick:=false','marine_slam:='+str(kind=='surface_mapping').lower()]
 workload=dict(schema='robot_lab.extension_workload.v1',workflow='marine',task='mapping' if kind=='surface_mapping' else 'navigation',
  truth_coordinates='selected_world',timeout_s=180.)
 if kind=='surface_mapping':workload['distance_m']=.4
 else:workload['goals']=[[-2.,-2.,0.],[-2.,-1.,0.]]
runner=ConcurrentRunner(root/kind,1,lambda e:print(json.dumps(e,default=str),flush=True))
results=runner.run([RunSpec(command=command,duration_s=260,max_rss_mb=8000,max_cpu_percent=1800,max_log_mb=384,workload=workload)])
print(json.dumps(results,indent=2,default=str),flush=True)
