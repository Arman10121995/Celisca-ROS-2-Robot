import json,math,os,signal,subprocess,time
from pathlib import Path
import psutil,rclpy
from std_msgs.msg import String

root=Path(os.environ.get('FLEET_CASE_DIRECTORY',str(Path(__file__).parent/'fleet-concurrent-1')));root.mkdir(parents=True,exist_ok=True)
import hashlib
workspace=Path('/workspace/molar/ros_ws/bumperbot_ws')
files=list((workspace/'src/robot_lab_utils/robot_lab_utils').glob('fleet*.py'))+list((workspace/'src/robot_lab_maps/maps/fleet_facility').rglob('*'))
(root/'source-sha256.json').write_text(json.dumps({str(f.relative_to(workspace)):hashlib.sha256(f.read_bytes()).hexdigest() for f in files if f.is_file()},indent=2)+'\n')
log=(root/'plant.log').open('w')
plant=subprocess.Popen(['ros2','launch','robot_lab_bringup','shared_world.launch.py','session:='+str(Path(__file__).parent/'fleet_three.yaml'),
    'navigation:=true','gui:=false','rviz:=false'],stdout=log,stderr=subprocess.STDOUT,cwd=root,start_new_session=True)
rclpy.init();node=rclpy.create_node('concurrent_floor_fleet_probe');fleet={};facility={}
node.create_subscription(String,'/fleet/state',lambda msg:fleet.update(json.loads(msg.data)),10)
node.create_subscription(String,'/fleet/infrastructure/state',lambda msg:facility.update(json.loads(msg.data)),10)
heartbeat=node.create_publisher(String,'/fleet/operator_heartbeat',10);command=node.create_publisher(String,'/fleet/command',10)
fac=node.create_publisher(String,'/fleet/infrastructure/command',10)
trace=(root/'trace.jsonl').open('w',buffering=1);start=time.monotonic();last=0.;phase='waiting';passed=False;reason='deadline';landings=[];overlap=[]
def send(op,facility_op=False,**kwargs):
    (fac if facility_op else command).publish(String(data=json.dumps(dict(session_id=fleet['session_id'],operator_id='concurrent-probe',op=op,**kwargs))))
try:
    while time.monotonic()-start<1000.:
        rclpy.spin_once(node,timeout_sec=.025);now=time.monotonic()
        if plant.poll() is not None:raise RuntimeError('Owned plant exited')
        if log.tell()>128*1024*1024:raise RuntimeError('Log budget exceeded')
        if not fleet or not facility:continue
        heartbeat.publish(String(data=json.dumps(dict(session_id=fleet['session_id'],operator_id='concurrent-probe'))))
        if now-last>.25:
            trace.write(json.dumps(dict(wall=now,phase=phase,fleet=fleet,facility=facility),allow_nan=False)+'\n');last=now
        members={row['id']:row for row in fleet['members']}
        if phase=='waiting' and all(row.get('navigation_ready') and row['mode']=='idle' for row in members.values()) and facility.get('operator_id')=='concurrent-probe' and facility.get('operator_fresh'):
            send('traffic',serial=False,targets=[]);phase='traffic'
        elif phase=='traffic' and fleet.get('traffic',{}).get('serial') is False:
            send('transport_queue',True,requests=[dict(robot=name,floor=floor,parking='auto') for floor in ('L1','L0') for name in ('alpha','beta')])
            send('route',targets=['gamma'],request_id='independent-gamma',goals=[dict(x=-2.,y=1.,yaw=0.,timeout=120.)]);phase='running'
        queue=facility.get('transport_queue',{});history=queue.get('history',[])
        if members['gamma']['mode'] in ('navigation','dispatching','settling') and any(row.get('phase') not in ('succeeded','failed','canceled') for row in facility.get('transports',{}).values()):overlap.append(now)
        if any(row['phase'] in ('failed','canceled') for row in history):raise RuntimeError('Transport failed: '+str(history))
        if len(history)>len(landings):
            row=history[len(landings)];member=members[row['robot']];b=member.get('truth');g=row.get('parking_goal')
            if not b or not g:raise RuntimeError('Missing independent parking observation')
            height=3. if row['floor']=='L1' else 0.
            if math.dist([b['x'],b['y'],b['z']],[g['x'],g['y'],height])>=.15 or abs(math.atan2(math.sin(b['yaw']-g['yaw']),math.cos(b['yaw']-g['yaw'])))>=.15 or abs(b['linear'])>=.03 or abs(b['angular'])>=.05:
                raise RuntimeError('Independent parking endpoint disagreed')
            landings.append(dict(robot=row['robot'],floor=row['floor'],parking_goal=g,body=b,map=facility['active_robot_maps'][row['robot']]))
        gamma=[row for row in fleet.get('task_history',[]) if row.get('source')=='route:independent-gamma']
        if any(row.get('state')!='succeeded' for row in gamma):raise RuntimeError('Independent concurrent ground mission failed')
        if len(landings)==4 and gamma and not queue.get('pending') and queue.get('active') is None:
            passed=bool(overlap and overlap[-1]-overlap[0]>1.)
            reason='Actual three-base concurrent ground mission and two-member parked floor round-trips' if passed else 'No measured simultaneous task interval'
            break
        if facility.get('last_rejection'):raise RuntimeError(facility['last_rejection'])
except (Exception,KeyboardInterrupt) as exc:reason=type(exc).__name__+': '+str(exc)
finally:
    if fleet and rclpy.ok():send('all_stop');rclpy.spin_once(node,timeout_sec=.1)
    result=dict(passed=passed,reason=reason,wall_seconds=time.monotonic()-start,phase=phase,landings=landings,overlap_s=(overlap[-1]-overlap[0]) if overlap else 0.,fleet=fleet,facility=facility)
    (root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False));print(json.dumps({k:v for k,v in result.items() if k not in ('fleet','facility')},indent=2),flush=True)
    children=psutil.Process(plant.pid).children(recursive=True) if plant.poll() is None else []
    if plant.poll() is None:
        os.killpg(plant.pid,signal.SIGINT)
        try:plant.wait(timeout=8.)
        except subprocess.TimeoutExpired:os.killpg(plant.pid,signal.SIGKILL)
    for child in children:
        try:
            if child.is_running():child.kill()
        except psutil.NoSuchProcess:pass
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    trace.close();log.close()
