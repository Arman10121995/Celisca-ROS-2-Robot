import json, math, os, sys, time, hashlib, subprocess
from types import SimpleNamespace
from pathlib import Path
import rclpy
sys.path.insert(0,str(Path.cwd()/'scripts'))
from sim_nav_check import Check
from robot_lab_gui.launcher import SimulationLauncherGui
root=Path(os.environ['ROBOT_LAB_RUNTIME_ROOT'])/'car-parking-gui-20261114'
report=dict(robot='ackermann_car',map='nav_empty',backend='pybullet',scope='One installed normal-GUI parking screen; independent simulator odometry; not the car course or a backend matrix',passed=False)
report['revision']=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
files=['install/robot_lab_utils/local/lib/python3.10/dist-packages/robot_lab_utils/car_parking.py',
       'install/robot_lab_utils/lib/python3.10/site-packages/robot_lab_utils/car_parking.py',
       'install/robot_lab_utils/lib/robot_lab_utils/car_parking.py',
       'install/robot_lab_utils/lib/python3.10/site-packages/robot_lab_utils/car_parking_clearance.py',
       'install/robot_lab_gui/lib/python3.10/site-packages/robot_lab_gui/car_parking_controls.py',
       'install/robot_lab_gui/lib/python3.10/site-packages/robot_lab_gui/launcher.py',
       'install/robot_lab_navigation/share/robot_lab_navigation/launch/navigation.launch.py',
       'install/robot_lab_navigation/share/robot_lab_navigation/config/robots/ackermann_car.yaml']
report['installed_sha256']={f:hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in files if Path(f).is_file()}
app=SimulationLauncherGui();app.update();check=None
states=[]
def pump():
    app.update()
    if check is not None:
        for _ in range(4):rclpy.spin_once(check,timeout_sec=0.)
    time.sleep(.01)
def until(predicate,budget,label):
    end=time.monotonic()+budget
    while time.monotonic()<end:
        pump()
        if predicate():return
        if app.process and app.process.poll() is not None:raise RuntimeError('Launch exited: '+str(app.process.returncode))
    raise TimeoutError(label)
try:
    app.robot_var.set('ackermann_car');app.simulator_var.set('pybullet');app.map_var.set('nav_empty')
    app.mode_var.set('nav');app.gui_var.set('false')
    app._update_from_selection();app.update()
    assert app.mode_var.get()=='nav',app.validation_var.get()
    assert app.start_button.instate(['!disabled']),app.validation_var.get()
    report['command']=app.command_var.get();app.start_button.invoke()
    if not rclpy.ok():rclpy.init()
    check=Check();check.monitor_motion=True
    until(lambda:app.car_parking_controls.owned(),10,'GUI owned launch')
    page=app.car_parking_controls;app.show_tab('Drive');page.connect_button.invoke()
    until(lambda:page.ready() and page.state.get('pose') and page.start_button.instate(['!disabled']) and check.truth is not None and check.action.server_is_ready(),90,'Parking state, localization, Nav2 and body truth')
    page.measured_button.invoke();start=[v.get() for v in page.target]
    report['initial_estimate']=start
    page.target[0].set(start[0]+.6*math.cos(math.radians(start[2])))
    page.target[1].set(start[1]+.6*math.sin(math.radians(start[2])))
    page.target[2].set(start[2]+10.);page.timeout.set(75.)
    report['target']=[v.get() for v in page.target]
    page.start_button.invoke()
    end=time.monotonic()+85;began=False
    while time.monotonic()<end:
        pump();state=dict(page.state)
        if not states or state!=states[-1]:states.append(state)
        phase=state.get('phase')
        if phase in ('navigating','aligning','trimming'):began=True
        if began and phase in ('parked','failed','canceled'):break
    else:raise TimeoutError('Parking did not finish')
    report['parking']=dict(page.state)
    for _ in range(100):pump()
    assert check.truth is not None and time.monotonic()-check.truth_received<1.,'No fresh simulator body truth'
    pose=check.truth.pose.pose;q=pose.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    target=report['target']
    report['truth']=dict(x=pose.position.x,y=pose.position.y,z=pose.position.z,yaw_deg=math.degrees(yaw),
        position_error_m=math.hypot(pose.position.x-target[0],pose.position.y-target[1]),
        yaw_error_deg=math.degrees(math.atan2(math.sin(yaw-math.radians(target[2])),math.cos(yaw-math.radians(target[2])))))
    report['body_samples']=len(check.trace)
    assert report['parking']['phase']=='parked',report['parking']
    assert report['truth']['position_error_m']<=.15 and abs(report['truth']['yaw_error_deg'])<=5.,report['truth']
    assert len(check.trace)>10
    # A second real navigation request loses only the GUI parking heartbeat.
    # No robot/pose/ROS state is mocked; the controller must cancel and stop.
    page.measured_button.invoke();page.target[0].set(page.target[0].get()+1.)
    page.start_button.invoke();until(lambda:page.state.get('phase')=='navigating',5,'Second parking request')
    held_heartbeat=page.heartbeat;page.heartbeat=SimpleNamespace(publish=lambda message:None)
    lost=time.monotonic()
    try:
        until(lambda:page.state.get('phase')=='failed' and 'heartbeat' in page.state.get('reason',''),5,'Operator-loss interruption')
        failure=dict(page.state)
        def body_speed():
            v=check.truth.twist.twist.linear
            return math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)
        until(lambda:check.truth is not None and body_speed()<.05,4,'Full body speed after operator loss')
        settled=time.monotonic()
        while time.monotonic()-settled<.5:
            pump()
            assert body_speed()<.05,'Body speed rose during stop settling'

        report['operator_loss']=dict(state=failure,elapsed_wall_s=time.monotonic()-lost,
            observed_body_speed_m_s=body_speed(),velocity_vector_m_s=[check.truth.twist.twist.linear.x,check.truth.twist.twist.linear.y,check.truth.twist.twist.linear.z],settled_wall_s=.5)
    finally:page.heartbeat=held_heartbeat
    report['passed']=True
except BaseException as exc:
    report['exception']=repr(exc)
finally:
    if check:
        (root/'body-trace.json').write_text(json.dumps(dict(samples=check.trace,commands=check.command_trace)))
        check.destroy_node()
    app.car_parking_controls.cancel()
    if app.process and app.process.poll() is None:
        app._stop_drive();app._stop_launch();end=time.monotonic()+45
        while app._launch_running and time.monotonic()<end:app.update();time.sleep(.03)
    report['launch_returncode']=app.process.poll() if app.process else None
    (root/'live-report.json').write_text(json.dumps(report,indent=2)+'\n')
    (root/'parking-states.json').write_text(json.dumps(states,indent=2)+'\n')
    (root/'gui-output.log').write_text(app.output.get('1.0','end'))
    app.car_parking_controls.close()
    if app.arm_tab:app.arm_tab.close()
    if app.hand_tab:app.hand_tab.close()
    app.destroy()
print(json.dumps(report,indent=2))
raise SystemExit(0 if report['passed'] else 1)
