"""Actual GUI inputs and independent backend body/wheel feedback, not a mission."""
import json, math, os, time
from pathlib import Path
from types import SimpleNamespace
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from robot_lab_gui.launcher import SimulationLauncherGui

robot=os.environ['PROBE_ROBOT']; backend=os.environ['PROBE_BACKEND']
out=Path(__file__).parent/(robot+'-'+backend); out.mkdir(exist_ok=True)
app=SimulationLauncherGui(); app.update(); node=None; samples=[]; joints=[]; commands=[]
report=dict(robot=robot,backend=backend,map='nav_empty',mode='display',passed=False,
            scope='Actual GUI neutral, forward/reverse/turn/Stop/watchdog; physics body truth, not mapping/navigation')
def pump(wall=.05):
    end=time.monotonic()+wall
    while time.monotonic()<end:
        app.update(); rclpy.spin_once(node,timeout_sec=.005)
def state(m):
    p=m.pose.pose; q=p.orientation
    samples.append(dict(wall=time.monotonic(),sim=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
        x=p.position.x,y=p.position.y,z=p.position.z,
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
        vx=m.twist.twist.linear.x,wz=m.twist.twist.angular.z,tilt=math.acos(max(-1,min(1,1-2*(q.x*q.x+q.y*q.y))))))
def interval(seconds):
    start=samples[-1]['sim']; deadline=time.monotonic()+max(45,seconds*40)
    while samples[-1]['sim']-start<seconds and time.monotonic()<deadline:pump()
    assert samples[-1]['sim']-start>=seconds,'clock stalled'
def measure(points):
    assert len(points)>10
    last=points[-1]; tail=[s for s in points if s['sim']>last['sim']-.5]; first=tail[0]
    dt=last['sim']-first['sim']; dyaw=sum(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])) for a,b in zip(tail,tail[1:]))
    h=first['yaw']+dyaw/2
    return dict(dx=last['x']-points[0]['x'],dy=last['y']-points[0]['y'],yaw=sum(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])) for a,b in zip(points,points[1:])),
        tail_vx=((last['x']-first['x'])*math.cos(h)+(last['y']-first['y'])*math.sin(h))/dt,
        tail_wz=dyaw/dt,z=last['z'],max_tilt=max(p['tilt'] for p in points),sim_duration=last['sim']-points[0]['sim'])
def key(name,down=True):
    event=SimpleNamespace(keysym=name,widget=app)
    (app._drive_key_press if down else app._drive_key_release)(event)
def stopped():
    begin=len(samples); key('space'); interval(2.5); result=measure(samples[begin:])
    assert abs(result['tail_vx'])<.025 and abs(result['tail_wz'])<.05,result
    return result
try:
    app.robot_var.set(robot); app.simulator_var.set(backend); app.map_var.set('nav_empty')
    app.mode_var.set('display'); app.gui_var.set('false'); app._update_from_selection(); app.update()
    report['command']=app.command_var.get(); assert app.start_button.instate(['!disabled']),app.validation_var.get()
    assert app.drive_input_checkbox.instate(['!disabled'])
    rclpy.init(); node=rclpy.create_node('turtlebot4_gui_drive_observer')
    node.create_subscription(Odometry,'/odom/ground_truth',state,qos_profile_sensor_data)
    node.create_subscription(JointState,'/joint_states',lambda m:joints.append(dict(names=m.name,positions=list(m.position),velocities=list(m.velocity))),10)
    node.create_subscription(Twist,'/key_vel',lambda m:commands.append(dict(wall=time.monotonic(),vx=m.linear.x,wz=m.angular.z)),10)
    app._start_launch(); report['owned_pid']=app.process.pid
    deadline=time.monotonic()+float(os.environ.get('PROBE_BUDGET','150'))
    while time.monotonic()<deadline and (len(samples)<50 or len(joints)<30):
        pump()
        assert app.process.poll() is None,'launch exited'
    assert len(samples)>=50 and len(joints)>=30,(len(samples),len(joints))
    interval(1.5)
    report['phases']={}
    begin=len(samples); count=len(commands); app.drive_input_enabled.set(True); app._toggle_drive_input(); interval(1)
    report['phases']['armed_neutral']=measure(samples[begin:]); report['neutral_commands']=len(commands)-count
    assert report['neutral_commands']==0,report['neutral_commands']
    for label,k in [('forward','w'),('reverse','s'),('left_turn','a'),('right_turn','d')]:
        begin=len(samples); key(k); interval(2.5); key(k,False)
        result=measure(samples[begin:]); report['phases'][label]=result
        if k=='w':assert .15<result['tail_vx']<.36,result
        elif k=='s':assert -.36<result['tail_vx']<-.15,result
        elif k=='a':assert .5<result['tail_wz']<1.45,result
        else:assert -1.45<result['tail_wz']<-.5,result
        report['phases'][label+'_stop']=stopped()
    begin=len(samples); key('w'); interval(2); report['phases']['pre_loss']=measure(samples[begin:])
    if app.drive_repeat_job is not None:app.after_cancel(app.drive_repeat_job);app.drive_repeat_job=None
    begin=len(samples); count=len(commands); interval(2.5)
    report['phases']['publisher_loss']=measure(samples[begin:]); report['silence_commands']=len(commands)-count
    assert report['silence_commands']<=1,report['silence_commands']
    result=report['phases']['publisher_loss']; assert abs(result['tail_vx'])<.025 and abs(result['tail_wz'])<.05,result
    report['joint_names']=joints[-1]['names']; assert {'left_wheel_joint','right_wheel_joint','wheel_drop_left_joint','wheel_drop_right_joint'}<=set(report['joint_names'])
    assert max(p['tilt'] for p in samples)<.3, 'unstable body tilt'
    assert all(math.isfinite(value) for j in joints for value in j['positions']+j['velocities'])
    report['body_samples']=len(samples);report['joint_samples']=len(joints)
    app._stop_drive();app._stop_launch();deadline=time.monotonic()+25
    while app._launch_running and time.monotonic()<deadline:pump()
    report['launch_returncode']=app.process.poll();assert report['launch_returncode']==0
    report['passed']=True
finally:
    if app.process and app.process.poll() is None:
        app._stop_drive();app._stop_launch();deadline=time.monotonic()+25
        while app._launch_running and time.monotonic()<deadline:
            app.update();time.sleep(.02)
    report['launch_returncode']=app.process.poll() if app.process else None
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'trace.json').write_text(json.dumps(dict(body=samples,joints=joints,commands=commands)))
    (out/'gui-output.log').write_text(app.output.get('1.0','end'))
    if node:node.destroy_node()
    app.destroy()
    if rclpy.ok():rclpy.shutdown()
