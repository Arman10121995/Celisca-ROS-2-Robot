import json,os
from pathlib import Path
from robot_lab_benchmark.concurrent_runner import ConcurrentRunner,RunSpec
root=Path(os.environ['LAB_EXTENSION_RUN'])
mode=os.environ.get('LAB_WORKLOAD','marine')
if mode=='marine':
 command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=lab_rov','mode:=marine','simulator:=gazebo','map_name:=marine_tank','gui:=false','start_rviz:=false','enable_joystick:=false','marine_slam:=true']
 workload=dict(schema='robot_lab.extension_workload.v1',workflow='marine',task='mapping',truth_coordinates='selected_world',distance_m=.2,timeout_s=150.)
elif mode=='aerial':
 command=['ros2','launch','robot_lab_bringup','aerial_swarm.launch.py','map_name:=nav_empty','gui:=false']
 import yaml
 reference=os.environ.get('LAB_AERIAL_REFERENCE','gnss')
 preset=root/('aerial-'+reference+'.yaml');preset.write_text(yaml.safe_dump(dict(schema_version=1,map='nav_empty',spacing=3.,members=[dict(id='air1',spawn={}),dict(id='air2',spawn={})],estimator=dict(height_reference=reference))))
 command.append('session:='+str(preset))
 workload=dict(schema='robot_lab.extension_workload.v1',workflow='aerial_swarm',task=os.environ.get('LAB_AERIAL_TASK','formation'),truth_coordinates='selected_world',timeout_s=220.)
elif mode=='gait':
 command=['ros2','launch','robot_lab_bringup','gait_simulator.launch.py','robot_model:=hexapod_ros2','task:=slam','map_name:=nav_empty','gui:=false','rviz:=false']
 workload=dict(schema='robot_lab.extension_workload.v1',workflow='gait',task='mapping',truth_coordinates='selected_world',distance_m=.15,timeout_s=280.)
else:raise ValueError(mode)
runner=ConcurrentRunner(root/('automatic-'+mode+('-'+os.environ.get('LAB_AERIAL_REFERENCE','gnss')+'-'+os.environ.get('LAB_AERIAL_TASK','formation') if mode=='aerial' else '')),1,lambda e:print(json.dumps(e,default=str),flush=True))
results=runner.run([RunSpec(command=command,duration_s=340,max_rss_mb=9000,max_cpu_percent=2000,max_log_mb=512,workload=workload,readiness_topic='/aerial_swarm/state' if mode=='aerial' else '/clock')])
print(json.dumps(results,indent=2,default=str),flush=True)
