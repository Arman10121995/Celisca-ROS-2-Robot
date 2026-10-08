import json,os,subprocess,time
from pathlib import Path
root=Path.cwd(); base=Path('/workspace/molar/robot_lab_runtime/extensions-finish-2026-10-07'); out=base/'shared-regressions';out.mkdir(exist_ok=False)
jobs=[]
for robot in ('asset_turtlebot4_standard','asset_turtlebot4_lite'):
 jobs.append((robot,['xvfb-run','-a','python3',str(root/'docs/status/evidence/turtlebot4-sensors-2026-10-06/asset_turtlebot4_standard-isaac/producer.py')],dict(PROBE_ROBOT=robot,PROBE_BACKEND='isaac',PROBE_ROOT=str(out),PROBE_BUDGET='240'),out/(robot+'-isaac')/'report.json'))
jobs.append(('panda-hand',['xvfb-run','-a','python3',str(base/'panda-hand-producer.py')],{},base/'panda-gripper/report.json'))
jobs.append(('panda-planning',['xvfb-run','-a','python3',str(root/'docs/status/evidence/panda-cartesian-controls-column-2026-10-07/final-responsive/producer.py')],dict(PANDA_PROBE_OUT=str(out/'panda-planning'),PANDA_PROBE_NORMAL='1'),out/'panda-planning/report.json'))
rows=[]
for label,command,extra,path in jobs:
 env=dict(os.environ,ROS_DOMAIN_ID='221',IGN_PARTITION='shared_regression_'+label,GZ_PARTITION='shared_regression_'+label,**extra)
 print('RUN',label,flush=True);begin=time.monotonic()
 with (out/(label+'.log')).open('w') as log:result=subprocess.run(command,env=env,cwd=root,stdout=log,stderr=subprocess.STDOUT,timeout=1500)
 report=json.loads(path.read_text()) if path.is_file() else {}
 row=dict(job=label,producer_returncode=result.returncode,launch_returncode=report.get('launch_returncode'),passed=report.get('passed') is True,wall_s=time.monotonic()-begin,report=str(path));rows.append(row)
 (out/'batch-report.json').write_text(json.dumps(rows,indent=2)+'\n');print(row,flush=True)
 if result.returncode!=0 or not row['passed'] or report.get('launch_returncode')!=0:raise SystemExit(1)
