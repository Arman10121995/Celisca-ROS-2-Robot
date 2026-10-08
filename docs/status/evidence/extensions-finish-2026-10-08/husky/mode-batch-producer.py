import json,os,subprocess,time
from pathlib import Path
root=Path.cwd();base=Path('/workspace/molar/robot_lab_runtime/extensions-finish-2026-10-07');out=base/'husky/modes-current';out.mkdir(exist_ok=False)
rows=[]
for backend in ('gazebo','mujoco','pybullet','isaac'):
 for mode,world in [('loc','nav_empty'),('slam','nav_empty'),('3d_slam','nav_empty'),('nav','nav_empty'),('nav','nav_obstacle')]:
  label=backend+'-'+mode+'-'+world;env=dict(os.environ,ROS_DOMAIN_ID='223',IGN_PARTITION='husky_modes_'+label,GZ_PARTITION='husky_modes_'+label,HUSKY_MODE_PROBE=str(out))
  command=['xvfb-run','-a','python3','scripts/qualify_robot_modes_gui.py','--robot','asset_husky','--backend',backend,'--mode',mode,'--map',world,'--experimental-provider',env['ROBOT_LAB_RUNTIME_ROOT'],'--output',str(out/label)]
  print('RUN',label,flush=True);begin=time.monotonic()
  with (out/(label+'.log')).open('w') as log:result=subprocess.run(command,env=env,cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=1100)
  path=out/label/'report.json';report=json.loads(path.read_text()) if path.is_file() else {}
  for task in Path('/proc').iterdir():
   if not task.name.isdigit():continue
   try:
    cmd=(task/'cmdline').read_bytes();environ=(task/'environ').read_bytes()
    if any(name in cmd for name in (b'isaac_runtime.py',b'mujoco_spawner',b'pybullet_spawner',b'ign gazebo')) and ('HUSKY_MODE_PROBE='+str(out)).encode()+b'\0' in environ:raise RuntimeError('Owned plant remains '+task.name)
   except (FileNotFoundError,PermissionError,ProcessLookupError):pass
  row=dict(robot='asset_husky',backend=backend,mode=mode,map=world,producer_returncode=result.returncode,launch_returncode=report.get('launch_returncode'),passed=report.get('passed') is True,wall_s=time.monotonic()-begin,report=str(path));rows.append(row)
  (out/'batch-report.json').write_text(json.dumps(rows,indent=2)+'\n');print(row,flush=True)
  if result.returncode!=0 or not row['passed'] or report.get('launch_returncode')!=0:raise SystemExit(1)
