import json,math,os,signal,subprocess,time
from pathlib import Path
import psutil,rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

root=Path(os.environ.get('AUV_CASE_DIRECTORY',str(Path(__file__).parent/'auv-curve-1')));root.mkdir(parents=True,exist_ok=True)
log=(root/'plant.log').open('w')
plant=subprocess.Popen(['ros2','launch','robot_lab_bringup','auv_simulator.launch.py','map_name:=marine_tank',
    'spawn_x:=-3.5','spawn_y:=-3.0','spawn_z:=-2.0','spawn_yaw:=0.0','gui:=false','rviz:=false'],
    stdout=log,stderr=subprocess.STDOUT,cwd=root,start_new_session=True)
rclpy.init();node=rclpy.create_node('curved_auv_mission_probe');native={};mission={};body={}
node.create_subscription(String,'/marine/state',lambda msg:native.update(json.loads(msg.data)),10)
node.create_subscription(String,'/marine/mission/state',lambda msg:mission.update(json.loads(msg.data)),10)
def observe(msg):
    p=msg.pose.pose.position;q=msg.pose.pose.orientation
    body.update(x=p.x,y=p.y,z=p.z,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
node.create_subscription(Odometry,'/marine/odometry',observe,10)
heartbeat=node.create_publisher(String,'/marine/operator_heartbeat',10)
control=node.create_publisher(String,'/marine/command',10);commands=node.create_publisher(String,'/marine/mission/command',10)
trace=(root/'trace.jsonl').open('w',buffering=1);start=time.monotonic();last=0.;phase='waiting';passed=False;reason='deadline';goal=None;initial=None
def send(op,mission_op=False,**kwargs):
    (commands if mission_op else control).publish(String(data=json.dumps(dict(session_id=native['session_id'],operator_id='auv-curve-probe',op=op,**kwargs))))
try:
    while time.monotonic()-start<210.:
        rclpy.spin_once(node,timeout_sec=.025);now=time.monotonic()
        if plant.poll() is not None:raise RuntimeError('Owned plant exited')
        if log.tell()>64*1024*1024:raise RuntimeError('Log budget exceeded')
        if not native:continue
        heartbeat.publish(String(data=json.dumps(dict(session_id=native['session_id'],operator_id='auv-curve-probe'))))
        if now-last>.15:
            trace.write(json.dumps(dict(wall=now,phase=phase,native=native,mission=mission,body=body),allow_nan=False)+'\n');last=now
        if phase=='waiting' and mission.get('ready') and native.get('operator_fresh') and native.get('operator_id')=='auv-curve-probe':
            initial=dict(body)
            p=mission['position'];yaw=mission['estimated_yaw']+math.pi;turn=math.radians(20.);rho=12.
            goal=[p[0]+math.cos(yaw)+rho*(math.sin(yaw+turn)-math.sin(yaw)),p[1]+math.sin(yaw)-rho*(math.cos(yaw+turn)-math.cos(yaw)),p[2]]
            send('plan',True,x=goal[0],y=goal[1],z=goal[2],goal_heading=yaw+turn,turn_radius=rho);phase='planning'
        elif phase=='planning' and mission.get('status')=='planned':send('arm');phase='arming'
        elif phase=='arming' and native.get('enabled'):send('execute',True);phase='executing'
        elif phase=='executing' and mission.get('status')=='completed':
            distance=math.dist([body['x'],body['y'],body['z']],goal)
            yaw_change=math.atan2(math.sin(body['yaw']-initial['yaw']),math.cos(body['yaw']-initial['yaw']))
            speed=math.sqrt(sum(value*value for value in native.get('velocity_world',[100.,100.,100.])))
            passed=distance<.6 and .35<yaw_change<1.2 and speed<.06
            reason='Observed native curved travel and settled braking' if passed else 'Estimated completion disagreed with independent body/yaw/speed screen'
            break
        if phase not in ('waiting',) and mission.get('status') in ('canceled','idle') and mission.get('reason','').startswith(('AUV','Planning failed','Marine mission','Native marine')):
            raise RuntimeError(mission['reason'])
except (Exception,KeyboardInterrupt) as exc:reason=type(exc).__name__+': '+str(exc)
finally:
    if native and rclpy.ok():send('stop');rclpy.spin_once(node,timeout_sec=.1)
    result=dict(passed=passed,reason=reason,phase=phase,wall_seconds=time.monotonic()-start,goal=goal,initial_body=initial,final_body=body,native=native,mission=mission)
    (root/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False));print(json.dumps({k:v for k,v in result.items() if k not in ('native','mission')},indent=2),flush=True)
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
