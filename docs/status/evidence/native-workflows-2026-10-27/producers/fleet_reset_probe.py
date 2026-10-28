import json,os,time,threading
from pathlib import Path
from robot_lab_benchmark.concurrent_runner import ConcurrentRunner,RunSpec
from robot_lab_benchmark.workload_contract import FLEET_WORKLOAD_SCHEMA
base=Path(os.environ['LAB_EXTENSION_RUN'])/('fleet-reset-'+time.strftime('%H%M%S'))
runner=ConcurrentRunner(base,max_concurrent=1,notify=lambda r:print(json.dumps({k:r.get(k) for k in ['id','state','reason','directory','measured_workload']}),flush=True))
spec=RunSpec(command=['ros2','launch','robot_lab_bringup','shared_world.launch.py','navigation:=true','rmf:=false','map_name:=nav_empty','gui:=false','rviz:=false'],duration_s=180.,startup_timeout_s=60.,max_cpu_percent=1000.,max_rss_mb=6144.,max_log_mb=128.,workload=dict(schema=FLEET_WORKLOAD_SCHEMA,truth_coordinates='selected_world',timeout_s=150.,fleet_goals=[dict(id='alpha',x=.3,y=-.5,yaw=0.),dict(id='beta',x=.3,y=-1.5,yaw=0.)]),selection=dict(robot='fleet',map='nav_empty',backend='gazebo',mode='fleet'))
def reset_when_running():
 deadline=time.monotonic()+90.
 while time.monotonic()<deadline:
  with runner.lock: jobs=list(runner.jobs.items())
  for identifier,job in jobs:
   trace=job['directory']/'fleet-trace.jsonl'
   if trace.exists() and '"kind": "command"' in trace.read_text() and '"op": "route"' in trace.read_text():
    time.sleep(2.)
    try:receipt=runner.reset(identifier)
    except Exception as exc:receipt=dict(error=str(exc))
    (base/'reset-observation.json').write_text(json.dumps(receipt,indent=2));return
  time.sleep(.2)
thread=threading.Thread(target=reset_when_running);thread.start()
result=runner.run([spec]);thread.join(timeout=45.);(base/'diagnostic.json').write_text(json.dumps(result,indent=2));print(json.dumps(dict(directory=str(base),result=result)),flush=True)
