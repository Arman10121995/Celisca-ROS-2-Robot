import json, time, os, math, hashlib
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
rclpy.init();n=rclpy.create_node('source_bound_energy_probe')
root=Path(os.environ['CASE_OUTPUT']);root.mkdir(exist_ok=True);trace=(root/'trace.jsonl').open('w', buffering=1)
state={};fixture={};rows={name:[] for name in ('alpha','beta')};poses={name:[] for name in rows};tf={};phase='wait'
def record(kind,**v):trace.write(json.dumps(dict(kind=kind,phase=phase,wall=time.monotonic(),**v),allow_nan=False)+'\n')
def joint(name,m):
 row=dict(t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,names=list(m.name),v=list(m.velocity),e=list(m.effort));rows[name].append(row)
 if len(rows[name])%10==0:record('motor',robot=name,**row)
def pose(name,m):
 p=m.pose.pose.position;poses[name].append([p.x,p.y,p.z])
def transforms(msg):
 for t in msg.transforms:
  tf.setdefault(t.child_frame_id,set()).add(t.header.frame_id)
for name in rows:
 n.create_subscription(JointState,'/'+name+'/joint_states',lambda m,k=name:joint(k,m),10)
 n.create_subscription(Odometry,'/'+name+'/odom/ground_truth',lambda m,k=name:pose(k,m),10)
n.create_subscription(String,'/fleet/state',lambda m:state.update(json.loads(m.data)),10)
n.create_subscription(String,'/fleet/infrastructure/state',lambda m:fixture.update(json.loads(m.data)),10)
n.create_subscription(TFMessage,'/tf',transforms,100)
h=n.create_publisher(String,'/fleet/operator_heartbeat',10);c=n.create_publisher(String,'/fleet/command',10)
d={k:n.create_publisher(Twist,'/'+k+'/key_vel',10) for k in rows}
start=time.monotonic();phase_at=start;done=False
while time.monotonic()-start<100:
 rclpy.spin_once(n,timeout_sec=.03)
 now=time.monotonic()
 if not state:continue
 base=dict(session_id=state['session_id'],operator_id='energy-source-probe')
 h.publish(String(data=json.dumps(base)))
 ready=state.get('operator_id')==base['operator_id'] and all(m.get('navigation_ready') for m in state.get('members',[])) and len(state.get('members',[]))==2
 if phase=='wait' and ready:
  c.publish(String(data=json.dumps(dict(base,op='manual',targets=list(rows)))));phase='forward';phase_at=now;record('begin',state=state)
 elif phase in ('forward','reverse','stop'):
  speed=.1 if phase=='forward' else -.1 if phase=='reverse' else 0.
  for pub in d.values():
   t=Twist();t.linear.x=speed;pub.publish(t)
  if now-phase_at>3:
   record('phase_end',state=state,fixture=fixture)
   if phase=='stop':done=True;break
   phase='reverse' if phase=='forward' else 'stop';phase_at=now
if state:c.publish(String(data=json.dumps(dict(base,op='all_stop'))))
metrics={}
for name,values in rows.items():
 complete=[r for r in values if len(r['names'])==len(r['v'])==len(r['e']) and len(r['e'])>0 and all(math.isfinite(x) for x in r['v']+r['e'])]
 effort=max((abs(e) for r in complete for e in r['e']),default=0.)
 drive=max((sum(max(0.,v*e) for v,e in zip(r['v'],r['e'])) for r in complete),default=0.)
 distance=max((math.dist(p,poses[name][0]) for p in poses[name]),default=0.)
 metrics[name]=dict(samples=len(values),complete=len(complete),max_effort_Nm=effort,max_positive_mechanical_W=drive,body_displacement_m=distance)
result=dict(passed=done and all(m['max_effort_Nm']>1e-4 and m['max_positive_mechanical_W']>1e-4 and m['body_displacement_m']>.1 and m['samples']==m['complete'] for m in metrics.values()),metrics=metrics,tf_parents={k:sorted(v) for k,v in tf.items()},final=state,fixture=fixture)
(root/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k not in ('final','fixture')},indent=2))
trace.close();n.destroy_node();rclpy.shutdown()
