import json,os,subprocess,time,sys
from pathlib import Path
r=Path(__file__).parent;env=os.environ.copy();env.update(PROBE_ROOT=str(r/'modes-final'),ROBOT_LAB_RUNTIME_ROOT=str(r/'qualification'),ROS_DOMAIN_ID='220',PROBE_BUDGET='900')
results=[];filters=sys.argv[1:]
for variant in ['burger','waffle','waffle_pi']:
 for backend in ['pybullet','mujoco','gazebo','isaac']:
  for mode,world in [('loc','nav_empty'),('slam','nav_empty'),('nav','nav_empty'),('nav','nav_obstacle')]:
   key=f'asset_turtlebot3_{variant}-{backend}-{mode}-{world}'
   if filters and not any(f in key for f in filters):continue
   out=r/'modes-final/workflows'/key
   if out.exists():
    report=json.loads((out/'report.json').read_text()) if (out/'report.json').is_file() else {}
    if report.get('passed') is True and report.get('launch_returncode')==0:
     print('RETAIN previous real proof',key,flush=True);continue
    raise RuntimeError('Preserve failed trial before repeating '+key)
   env.update(PROBE_ROBOT='asset_turtlebot3_'+variant,PROBE_BACKEND=backend,PROBE_MODE=mode,PROBE_MAP=world)
   print(time.strftime('%H:%M:%S'),'START',key,flush=True)
   with (r/(key+'-modes-final.log')).open('w') as log:
    rc=subprocess.run(['xvfb-run','-a','-s','-screen 0 1600x1000x24','python3',str(r/'probe_workflow_gui.py')],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1000).returncode
   report=json.loads((out/'report.json').read_text());pid=report.get('owned_pid');task=Path('/proc')/str(pid)
   if task.exists() and (task/'stat').read_text().split()[2]!='Z':raise RuntimeError('Owned launch remains '+str(pid))
   for task in Path('/proc').iterdir():
    if not task.name.isdigit():continue
    try:
     cmd=(task/'cmdline').read_bytes();e=(task/'environ').read_bytes()
     if (b'isaac_runtime.py' in cmd or b'mujoco_spawner' in cmd or b'pybullet_spawner' in cmd or b'ign gazebo' in cmd) and ('PROBE_ROOT='+env['PROBE_ROOT']).encode()+b'\0' in e:raise RuntimeError('Owned plant remains '+task.name)
    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
   results.append(dict(case=key,returncode=rc,passed=report['passed']))
   (r/'modes-final-batch.json').write_text(json.dumps(results,indent=2)+'\n')
   print(time.strftime('%H:%M:%S'),'END',key,rc,report['passed'],flush=True)
   if rc!=0 or not report['passed']:raise RuntimeError('Mode qualification failed: '+key)
print('Completed actual mode checks in this invocation',len(results),flush=True)
