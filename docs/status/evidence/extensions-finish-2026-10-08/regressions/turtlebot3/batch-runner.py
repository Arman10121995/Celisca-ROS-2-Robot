import json,os,subprocess,time
from pathlib import Path
root=Path('/workspace/molar/ros_ws/bumperbot_ws');out=Path('/workspace/molar/robot_lab_runtime/extensions-finish-2026-10-07/turtlebot3-current-drive');out.mkdir(exist_ok=False)
producer=root/'docs/status/evidence/turtlebot3-sensors-2026-10-07/asset_turtlebot3_burger-mujoco/producer.py';rows=[]
for robot in ('asset_turtlebot3_burger','asset_turtlebot3_waffle','asset_turtlebot3_waffle_pi'):
 for backend in ('gazebo','mujoco','pybullet','isaac'):
  env=dict(os.environ,PROBE_ROBOT=robot,PROBE_BACKEND=backend,PROBE_ROOT=str(out),PROBE_BUDGET='240',ROS_DOMAIN_ID='220',IGN_PARTITION='drive_regression_'+robot+'_'+backend,GZ_PARTITION='drive_regression_'+robot+'_'+backend)
  print('RUN',robot,backend,flush=True);begin=time.monotonic()
  with (out/(robot+'-'+backend+'.log')).open('w') as log:
   result=subprocess.run(['xvfb-run','-a','python3',str(producer)],env=env,cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=1100)
  path=out/(robot+'-'+backend)/'report.json';report=json.loads(path.read_text()) if path.is_file() else {}
  
  for task in Path('/proc').iterdir():
   if not task.name.isdigit():continue
   try:
    cmd=(task/'cmdline').read_bytes();environ=(task/'environ').read_bytes()
    if any(name in cmd for name in (b'isaac_runtime.py',b'mujoco_spawner',b'pybullet_spawner',b'ign gazebo')) and ('PROBE_ROOT='+str(out)).encode()+b'\0' in environ:raise RuntimeError('Owned plant remains '+task.name)
   except (FileNotFoundError,PermissionError,ProcessLookupError):pass
  rows.append(dict(robot=robot,backend=backend,producer_returncode=result.returncode,launch_returncode=report.get('launch_returncode'),passed=report.get('passed') is True,wall_s=time.monotonic()-begin,report=str(path)))
  (out/'batch-report.json').write_text(json.dumps(rows,indent=2)+'\n');print(rows[-1],flush=True)
  if result.returncode!=0 or not rows[-1]['passed'] or report.get('launch_returncode')!=0:raise SystemExit(1)
