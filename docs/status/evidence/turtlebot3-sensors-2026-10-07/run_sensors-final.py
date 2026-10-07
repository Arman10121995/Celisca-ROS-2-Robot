import json,os,subprocess,time
from pathlib import Path
r=Path(__file__).parent;env=os.environ.copy();env.update(PROBE_ROOT=str(r/'sensors-final'),ROS_DOMAIN_ID='219',PROBE_BUDGET='250')
results=[]
for variant in ['burger','waffle','waffle_pi']:
 for backend in ['gazebo','mujoco','pybullet','isaac']:
  key='asset_turtlebot3_'+variant+'-'+backend;out=r/'sensors-final'/key
  if out.exists():raise RuntimeError('Preserve previous run before repeating '+key)
  env.update(PROBE_ROBOT='asset_turtlebot3_'+variant,PROBE_BACKEND=backend)
  print(time.strftime('%H:%M:%S'),'START',key,flush=True)
  with (r/(key+'-sensors-final.log')).open('w') as log:
   rc=subprocess.run(['xvfb-run','-a','-s','-screen 0 1600x1000x24','python3',str(r/'probe_sensors_gui.py')],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=750).returncode
  report=json.loads((out/'report.json').read_text());pid=report['owned_pid'];p=Path('/proc')/str(pid)
  if p.exists() and (p/'stat').read_text().split()[2]!='Z':raise RuntimeError('Owned launch remains '+str(pid))
  for task in Path('/proc').iterdir():
   if not task.name.isdigit():continue
   try:
    cmd=(task/'cmdline').read_bytes();e=(task/'environ').read_bytes()
    if (b'isaac_runtime.py' in cmd or b'mujoco_spawner' in cmd or b'pybullet_spawner' in cmd or b'ign gazebo' in cmd) and ('PROBE_ROOT='+env['PROBE_ROOT']).encode()+b'\0' in e:raise RuntimeError('Owned plant remains '+task.name)
   except (FileNotFoundError,PermissionError,ProcessLookupError):pass
  results.append(dict(case=key,returncode=rc,passed=report['passed'],rtf=report.get('real_time_factor')))
  (r/'sensors-final-batch.json').write_text(json.dumps(results,indent=2)+'\n')
  print(time.strftime('%H:%M:%S'),'END',key,rc,report['passed'],flush=True)
  if rc!=0 or not report['passed']:raise RuntimeError('Sensor qualification failed: '+key)
print('Measured sensors/GUI Drive complete',len(results),flush=True)
