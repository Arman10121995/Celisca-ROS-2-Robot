from pathlib import Path
import json,os,subprocess,time
root=Path('/workspace/molar/ros_ws/bumperbot_ws')
out=Path(__file__).resolve().parent
cases=[('bumperbot','gazebo'),('bumperbot','pybullet'),('bumperbot','isaac'),('labbot','gazebo'),('labbot','pybullet'),('labbot','mujoco'),('labbot','isaac')]
summary={'scope':'Normal GUI short ground-floor hospital navigation on occupancy v4, serial independent physical endpoints; no obstacle/all-map qualification.','records':[]}
for i,(robot,backend) in enumerate(cases):
    folder=out/(robot+'-dataset_hospital-'+backend)
    environment=dict(os.environ,ROS_DOMAIN_ID=str(211+i),IGN_PARTITION='hospital_v4_'+robot+'_'+backend,GZ_PARTITION='hospital_v4_'+robot+'_'+backend)
    command=['xvfb-run','-a','python3',str(root/'scripts/qualify_world_navigation_gui.py'),'--robot',robot,'--backend',backend,'--map','dataset_hospital','--output',str(folder),'--timeout','600']
    start=time.monotonic()
    with (out/(robot+'-'+backend+'.log')).open('w') as log:
        code=subprocess.run(command,cwd=root,env=environment,stdout=log,stderr=subprocess.STDOUT).returncode
    record=dict(robot=robot,backend=backend,returncode=code,wall_s=round(time.monotonic()-start,3),report=str(folder/'report.json'),command=command,domain=environment['ROS_DOMAIN_ID'])
    if (folder/'report.json').is_file():record['navigation']=json.loads((folder/'report.json').read_text()).get('navigation')
    summary['records'].append(record)
    (out/'batch-report.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(robot,backend,code,record.get('navigation'),flush=True)
    if code:raise SystemExit(code)
