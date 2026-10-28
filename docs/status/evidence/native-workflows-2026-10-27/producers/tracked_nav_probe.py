import json,os,time
from pathlib import Path
from robot_lab_benchmark.concurrent_runner import ConcurrentRunner,RunSpec
root=Path(os.environ['LAB_EXTENSION_RUN'])/('tracked-navigation-'+str(time.time_ns()))
spec=RunSpec(command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=lab_tracked','mode:=nav','map_name:=nav_empty','simulator:=gazebo','gui:=false','start_rviz:=false','enable_joystick:=false'],duration_s=150,max_rss_mb=8000,max_cpu_percent=1600,max_log_mb=256,workload=dict(schema='robot_lab.nav2_workload.v1',truth_coordinates='selected_world',goals=[[.5,-.5,0.]]),selection=dict(robot='lab_tracked',map='nav_empty',backend='gazebo',mode='nav'))
r=ConcurrentRunner(root,1,lambda r:print(json.dumps({k:r.get(k) for k in ('id','state','mission_result_error')},default=str),flush=True)).run([spec]);print(json.dumps(r,default=str),flush=True)
