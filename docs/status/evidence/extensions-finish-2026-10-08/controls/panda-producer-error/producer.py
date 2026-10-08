"""Actual GUI MoveIt planning/native-actuator execution with independent body TF.

Experimental qualification adds the planning flag only to this GUI instance.
It does not promote the normal installed robot catalog before measured proof.
"""
import hashlib, json, math, os, subprocess, time, traceback
from pathlib import Path
import mujoco as mj
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from robot_lab_gui.launcher import SimulationLauncherGui

root = Path.cwd()
out = Path(os.environ['PANDA_PROBE_OUT'])
out.mkdir(parents=True, exist_ok=False)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
(out/'producer.py').write_bytes(Path(__file__).read_bytes())
app = SimulationLauncherGui(); app.update()
arm = app.arm_tab; cart = arm.cartesian
node = None; samples = []; states = []; scene = []; observations = []
report = dict(robot='menagerie_franka_emika_panda', backend='mujoco', map='nav_empty',
    mode='display', passed=False, checks={},
    limits=dict(tcp_position_error_m=.02, orientation_error_deg=5.,
                hold_velocity_rad_s=.03, hold_drift_rad=.02, heartbeat_abort_wall_s=1.3),
    scope='Actual Tk Plan/Execute, native Panda PD actuators and independent physical hand-body TF; static nav_empty only')
last_output = 0.

def pump():
    global last_output
    app.update()
    if node:
        for _ in range(16): observer_executor.spin_once(timeout_sec=0.)
    if time.monotonic()-last_output > 3:
        (out/'live-gui-output.log').write_text(app.output.get('1.0','end'))
        last_output = time.monotonic()
    time.sleep(.005)

def until(predicate, budget=30):
    end = time.monotonic()+budget
    while not predicate() and time.monotonic()<end:
        pump()
        if app.process and app.process.poll() is not None:
            raise AssertionError('Owned launch exited: '+str(app.process.returncode))
    assert predicate(), 'Timed out: '+cart.status.get()+' / '+arm.status_var.get()

def wait_sim(duration):
    start = samples[-1]['sim']
    until(lambda: samples[-1]['sim'] >= start+duration, max(20, duration*10))

def rotation(q):
    result = np.zeros(9); mj.mju_quat2Mat(result, np.array(q))
    return result.reshape(3,3)

def tool():
    until(lambda: buffer.lookup_transform('native_world','native_body_9',rclpy.time.Time()).header.stamp.sec+
        buffer.lookup_transform('native_world','native_body_9',rclpy.time.Time()).header.stamp.nanosec*1e-9 >= samples[-1]['sim']-.08, 5)
    transform = buffer.lookup_transform('native_world','native_body_9',rclpy.time.Time())
    t = transform.transform
    quaternion = [t.rotation.w,t.rotation.x,t.rotation.y,t.rotation.z]
    position = np.array([t.translation.x,t.translation.y,t.translation.z]) + rotation(quaternion)@tcp_offset
    result = dict(sim=transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9,
                  position=position.tolist(), quaternion_wxyz=quaternion,
                  age_sim_s=samples[-1]['sim']-(transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9))
    observations.append(result)
    return result

def plan(offset=None):
    until(lambda: cart.plan_button.instate(['!disabled']))
    if offset is not None:
        for variable, value in zip(cart.offset, offset): variable.set(value)
    cart.plan_button.invoke()
    until(lambda: not cart.planning, 12)
    return cart.points

def plan_required(offset):
    points = plan(offset)
    assert points, cart.status.get()
    until(lambda: cart.execute_button.instate(['!disabled']))
    target = cart.target
    return dict(offset_m=list(offset), duration_sim_s=points[-1][0],
                waypoints=[dict(time=t, positions=q) for t,q in points],
                target_position=[target.position.x,target.position.y,target.position.z],
                target_quaternion_wxyz=[target.orientation.w,target.orientation.x,target.orientation.y,target.orientation.z])

def execute_complete(offset, minimum_displacement=.02):
    measured_plan = plan_required(offset)
    first = tool(); first_sample = len(samples)
    cart.execute_button.invoke()
    until(lambda: arm.handle is not None or arm.state['busy'])
    until(lambda: arm.handle is None and not arm.state['busy'] and not arm.sending, 50)
    assert 'target reached' in arm.state['status'], arm.state
    wait_sim(1.)
    actual = tool()
    error = math.dist(actual['position'], measured_plan['target_position'])
    a, b = rotation(actual['quaternion_wxyz']), rotation(measured_plan['target_quaternion_wxyz'])
    angle = math.degrees(math.acos(float(np.clip((np.trace(a.T@b)-1)/2,-1,1))))
    displacement = math.dist(first['position'], actual['position'])
    measured_plan.update(actual=actual, position_error_m=error, orientation_error_deg=angle,
                         physical_displacement_m=displacement, joint_samples=len(samples)-first_sample,
                         terminal_status=dict(arm.state))
    assert error < .02 and angle < 5. and displacement > minimum_displacement, measured_plan
    assert not any(s['contact_blocked'] for s in states[-20:])
    return measured_plan

def hold():
    wait_sim(1.5)
    before = dict(samples[-1]['positions'])
    wait_sim(.8)
    drift = max(abs(samples[-1]['positions'][n]-q) for n,q in before.items() if n.startswith('joint'))
    speed = max(abs(v) for n,v in samples[-1]['velocities'].items() if n.startswith('joint'))
    assert drift < .02 and speed < .03, (drift,speed)
    return dict(drift_rad=drift, max_velocity_rad_s=speed, status=dict(arm.state), tcp=tool())

try:
    app.robot_var.set(report['robot']); app.simulator_var.set('mujoco')
    app.map_var.set('nav_empty'); app.mode_var.set('display'); app.gui_var.set('false')
    if os.environ.get('PANDA_PROBE_NORMAL') != '1':
        app.robot_profiles[report['robot']]['arm_planning']='moveit'
    else:
        assert app.robot_profiles[report['robot']].get('arm_planning')=='moveit', 'Normal profile planning not enabled'
    app._update_from_selection(); app.update()
    report['command']=app.command_var.get()
    assert 'arm_control:=panda' in report['command'] and 'arm_planning:=moveit' in report['command']
    assert app.start_button.instate(['!disabled'])
    profile = app.robot_profiles[report['robot']]
    m = mj.MjModel.from_xml_path(profile['native_mjcf']); d = mj.MjData(m)
    mj.mj_resetDataKeyframe(m,d,0); mj.mj_forward(m,d)
    pads=[g for g in range(m.ngeom) if int(m.geom_bodyid[g]) in (10,11)
          and int(m.geom_type[g])==int(mj.mjtGeom.mjGEOM_BOX)
          and np.allclose(m.geom_size[g],[.0085,.004,.0085])]
    assert len(pads)==2
    tcp_offset=d.xmat[9].reshape(3,3).T@(np.mean(d.geom_xpos[pads],axis=0)-d.xpos[9])
    report['tcp_native_hand_offset_m']=tcp_offset.tolist()
    source={str(p.relative_to(root)):sha(p) for package in ('robot_lab_utils','robot_lab_gui','robot_lab_bringup','robot_lab_mujoco')
            for p in (root/'src'/package).rglob('*') if p.is_file() and p.suffix in ('.py','.xml','.yaml')}
    installed={str(p.relative_to(root)):sha(p) for package in ('robot_lab_utils','robot_lab_gui','robot_lab_bringup','robot_lab_mujoco')
               for p in (root/'install'/package).rglob('*.py') if p.is_file()}
    (out/'source-manifest.json').write_text(json.dumps(dict(git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        source_sha256=source, installed_sha256=installed, profile=profile,
        native_xml_sha256=sha(Path(profile['native_mjcf'])),
        native_asset_sha256={str(p.relative_to(Path(profile['native_mjcf']).parent)):sha(p) for p in Path(profile['native_mjcf']).parent.rglob('*') if p.is_file()}, producer_sha256=sha(Path(__file__)),
        experimental_override='none; normal installed profile' if os.environ.get('PANDA_PROBE_NORMAL')=='1' else 'arm_planning=moveit in this GUI instance only; production support requires this physical measurement'),indent=2)+'\n')
    snapshots=out/'source-snapshot'
    for relative in source:
        path=snapshots/relative;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes((root/relative).read_bytes())
    from ament_index_python.packages import get_package_prefix
    plugin_paths=[]
    for pkg in ('moveit_ros_move_group','moveit_planners_ompl','moveit_kinematics','moveit_ros_planning','moveit_simple_controller_manager'):
        plugin_paths.extend((Path(get_package_prefix(pkg))/'lib').glob('libmoveit*.so'))
    (out/'sdk-manifest.json').write_text(json.dumps(dict(mujoco_version=mj.__version__,moveit_packages=subprocess.check_output(['dpkg-query','-W','ros-humble-moveit-*'],text=True),plugins={str(p):sha(p) for p in plugin_paths}),indent=2)+'\n')
    app._start_launch(); report['owned_pid']=app.process.pid
    (out/'ownership.json').write_text(json.dumps(dict(pid=app.process.pid,command=report['command']))+'\n')
    app._ensure_ros_publisher(); node=Node('panda_cartesian_physical_observer')
    from rclpy.executors import SingleThreadedExecutor
    observer_executor=SingleThreadedExecutor();observer_executor.add_node(node)
    buffer=Buffer(); listener=TransformListener(buffer,node)
    def joints(msg):
        samples.append(dict(wall=time.monotonic(),sim=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            positions=dict(zip(msg.name,msg.position)),velocities=dict(zip(msg.name,msg.velocity))))
    node.create_subscription(JointState,'/joint_states',joints,20)
    node.create_subscription(String,'/arm/status',lambda msg:states.append(json.loads(msg.data)),20)
    node.create_subscription(String,'/arm/planning_scene_status',lambda msg:scene.append(json.loads(msg.data)),10)
    until(lambda: cart.ready() and len(samples)>10 and buffer.can_transform('native_world','native_body_9',rclpy.time.Time()),60)
    app.live_monitor_tab._connect()
    until(lambda:len(app.live_monitor_tab._clock_buf)>5)
    app.show_tab('Arm'); app.update(); wait_sim(1)
    report['scene']=dict(cart.scene); report['initial_tcp']=tool()
    report['checks']['selected_world_scene_acknowledged']=True
    # Actual planning-service negatives are checked inside this same owned
    # physics launch; they must not produce an action or move the plant.
    from moveit_msgs.srv import GetPositionFK, GetStateValidity, GetMotionPlan
    from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
    from geometry_msgs.msg import Pose
    from shape_msgs.msg import SolidPrimitive
    def rpc(client, request):
        assert client.service_is_ready(), client.srv_name
        future=client.call_async(request);until(future.done,12);return future.result()
    fk=node.create_client(GetPositionFK,'/compute_fk')
    validity=node.create_client(GetStateValidity,'/check_state_validity')
    planner=node.create_client(GetMotionPlan,'/plan_kinematic_path')
    until(lambda:fk.service_is_ready() and validity.service_is_ready() and planner.service_is_ready())
    names=['joint'+str(i) for i in range(1,8)]+['finger_joint1','finger_joint2']
    current=[samples[-1]['positions'][n] for n in names]
    def fk_request(positions):
        request=GetPositionFK.Request();request.header.frame_id='native_world';request.fk_link_names=['panda_tcp']
        request.robot_state.joint_state.name=names;request.robot_state.joint_state.position=positions
        return request
    home_fk=rpc(fk,fk_request(current)).pose_stamped[0].pose
    physical=tool();report['measured_fk_error_m']=math.dist(physical['position'],
        [home_fk.position.x,home_fk.position.y,home_fk.position.z])
    assert report['measured_fk_error_m']<.003
    floor_state=list(m.key_qpos[0]);floor_state[1]+=1.6
    floor_request=fk_request(floor_state)
    floor_pose=rpc(fk,floor_request).pose_stamped[0].pose
    request=GetStateValidity.Request();request.group_name='panda_arm';request.robot_state=floor_request.robot_state
    checked=rpc(validity,request)
    report['floor_collision']=dict(valid=checked.valid,contacts=[dict(first=c.contact_body_1,second=c.contact_body_2) for c in checked.contacts])
    assert not checked.valid and checked.contacts
    self_state=[0.,0.,0.,-2.9,0.,1.4,0.,.04,.04]
    d.qpos[:]=self_state; mj.mj_forward(m,d)
    assert d.ncon>0
    request.robot_state=fk_request(self_state).robot_state;self_checked=rpc(validity,request)
    report['self_collision']=dict(valid=self_checked.valid,
        native_contacts=[dict(first=int(m.geom_bodyid[c.geom[0]]),second=int(m.geom_bodyid[c.geom[1]])) for c in d.contact],
        planner_contacts=[dict(first=c.contact_body_1,second=c.contact_body_2) for c in self_checked.contacts])
    assert not self_checked.valid and any(c.contact_body_1.startswith('native_body_') and c.contact_body_2.startswith('native_body_') for c in self_checked.contacts)
    report['planning_negatives']=[]
    for label, pose in [('floor_blocked',floor_pose),('unreachable',Pose())]:
        if label=='unreachable':pose.position.x=10.;pose.position.y=home_fk.position.y;pose.position.z=home_fk.position.z;pose.orientation=home_fk.orientation
        pc=PositionConstraint(link_name='panda_tcp',weight=1.);pc.header.frame_id='native_world'
        pc.constraint_region.primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[.003])]
        region=Pose(position=pose.position);region.orientation.w=1.;pc.constraint_region.primitive_poses=[region]
        oc=OrientationConstraint(link_name='panda_tcp',weight=1.,orientation=pose.orientation,
            absolute_x_axis_tolerance=.03,absolute_y_axis_tolerance=.03,absolute_z_axis_tolerance=.03)
        oc.header.frame_id='native_world'
        request=GetMotionPlan.Request();motion=request.motion_plan_request
        motion.group_name,motion.pipeline_id,motion.planner_id='panda_arm','ompl','RRTConnect'
        motion.start_state=fk_request([samples[-1]['positions'][n] for n in names]).robot_state
        motion.goal_constraints=[Constraints(position_constraints=[pc],orientation_constraints=[oc])]
        motion.allowed_planning_time,motion.num_planning_attempts=3.,1
        before=dict(samples[-1]['positions']);answer=rpc(planner,request).motion_plan_response
        drift=max(abs(samples[-1]['positions'][n]-q) for n,q in before.items())
        report['planning_negatives'].append(dict(case=label,error_code=answer.error_code.val,
            waypoints=len(answer.trajectory.joint_trajectory.points),physical_drift_rad=drift))
        assert answer.error_code.val!=1 and not answer.trajectory.joint_trajectory.points and drift<.003
    report['checks']['collision_and_unreachable_rejection']=True
    # New operator workflow: real measured FK read, three small axis selections,
    # Plan/Execute and independent physics TCP; target edits alone must not move.
    report['intuitive_targets']=[]
    for index, sign in ((0,-1),(1,1),(2,1)):
        until(lambda:cart.use_pose_button.instate(['!disabled']))
        before=dict(samples[-1]['positions'])
        cart.use_pose_button.invoke()
        until(lambda:not cart.reading_pose and cart.reference is not None,8)
        measured=tool()
        fk_error=math.dist(cart.reference,measured['position'])
        assert fk_error<.003 and all(value.get()==0. for value in cart.offset)
        cart.step.set(.01)
        cart.target_buttons[index*2+(1 if sign>0 else 0)].invoke()
        target_selected=cart.target_display.get()
        assert cart.offset[index].get()==sign*.01 and cart.points is None
        wait_sim(.25)
        drift=max(abs(samples[-1]['positions'][n]-q) for n,q in before.items())
        assert drift<.003, ('Target selection moved the arm',drift)
        motion=execute_complete(None,.005)
        report['intuitive_targets'].append(dict(axis='xyz'[index],sign=sign,step_m=.01,
            pose_fk_error_m=fk_error,selection_joint_drift_rad=drift,displayed_target=target_selected,motion=motion))
    report['checks']['measured_pose_and_three_axis_target_buttons']=True
    report['motions']=[execute_complete((0.,0.,.05)),execute_complete((0.,.04,0.))]
    report['checks']['two_cartesian_targets_measured']=True
    # A planned target must become unusable after a real GUI joint jog.
    report['stale_plan']=plan_required((0.,0.,.04))
    before_joints=dict(samples[-1]['positions'])
    arm.step_var.set(.02); arm.jog_buttons[1].invoke()
    until(lambda:arm.handle is not None or arm.state['busy'])
    until(lambda:arm.handle is None and not arm.state['busy'] and not arm.sending)
    assert cart.points is None and cart.execute_button.instate(['disabled'])
    report['joint_invalidation']=dict(change_rad=max(abs(samples[-1]['positions'][n]-q) for n,q in before_joints.items() if n.startswith('joint')),execute_disabled=cart.execute_button.instate(['disabled']),plan_removed=cart.points is None)
    report['checks']['real_joint_change_invalidates_plan']=True
    # Real finger motion also invalidates a plan whose collision scene used
    # the previous hand opening, even though the arm remained stationary.
    report['hand_stale_plan']=plan_required((0.,0.,.04))
    before_fingers=dict(samples[-1]['positions'])
    app.hand_tab.close_button.invoke()
    until(lambda:app.hand_tab.handle is not None and app.hand_tab.state['busy'])
    until(lambda:cart.points is None)
    app.hand_tab.cancel_button.invoke()
    until(lambda:app.hand_tab.handle is None and not app.hand_tab.state['busy'])
    assert cart.execute_button.instate(['disabled'])
    report['hand_invalidation']=dict(change_m=max(abs(samples[-1]['positions'][n]-q) for n,q in before_fingers.items() if n.startswith('finger')),execute_disabled=cart.execute_button.instate(['disabled']),plan_removed=cart.points is None)
    report['checks']['real_hand_change_invalidates_plan']=True
    until(lambda:app.hand_tab.open_button.instate(['!disabled']))
    app.hand_tab.open_button.invoke()
    until(lambda:app.hand_tab.handle is not None)
    until(lambda:app.hand_tab.handle is None and not app.hand_tab.state['busy'])
    # GUI's input guard rejects offsets outside its bounded operator contract.
    before=dict(samples[-1]['positions']); cart.offset[0].set(1.)
    until(lambda:cart.plan_button.instate(['!disabled'])); cart.plan_button.invoke(); wait_sim(.4)
    assert not cart.points and 'within' in cart.status.get()
    assert max(abs(samples[-1]['positions'][n]-q) for n,q in before.items())<.003
    report['invalid_offset_drift_rad']=max(abs(samples[-1]['positions'][n]-q) for n,q in before.items()); report['invalid_offset_status']=cart.status.get(); report['checks']['invalid_offset_no_motion']=True
    for interruption in ('cancel','stop','heartbeat_loss'):
        measured_plan=plan_required((0.,0.,.06)); cart.execute_button.invoke()
        until(lambda:arm.handle is not None and arm.state['busy'])
        wait_sim(.1)
        begin=time.monotonic()
        if interruption=='cancel': arm.cancel_button.invoke()
        elif interruption=='stop': arm.stop_button.invoke()
        else:
            original=arm.heartbeat_pub.publish
            arm.heartbeat_pub.publish=lambda _:None
        try:
            until(lambda:arm.handle is None and not arm.state['busy'] and not arm.sending,4)
            elapsed=time.monotonic()-begin
            expected={'cancel':'canceled','stop':'stopped','heartbeat_loss':'heartbeat lost'}[interruption]
            assert expected in arm.state['status'],arm.state
            if interruption=='heartbeat_loss':assert .65<=elapsed<1.3,elapsed
            report[interruption]=dict(abort_wall_s=elapsed,planned_duration_sim_s=measured_plan['duration_sim_s'],**hold())
            report['checks'][interruption]=True
        finally:
            if interruption=='heartbeat_loss':arm.heartbeat_pub.publish=original
    report['reset_cached_plan']=plan_required((0.,0.,.04))
    before=samples[-1]['sim'];app._update_reset_button();assert app.reset_robot_button.instate(['!disabled'])
    app.reset_robot_button.invoke()
    until(lambda:cart.points is None and max(abs(samples[-1]['positions'][n]-q) for n,q in zip(names[:7],arm.state['home']))<.015)
    wait_sim(1.)
    report['reset']=dict(plan_removed=cart.points is None,execute_disabled=cart.execute_button.instate(['disabled']),before_sim=before,after_sim=samples[-1]['sim'],home_error_rad=max(abs(samples[-1]['positions'][n]-q) for n,q in zip(names[:7],arm.state['home'])),tcp=tool())
    assert report['reset']['after_sim']>before and math.dist(report['initial_tcp']['position'],report['reset']['tcp']['position'])<.004
    report['checks']['reset_invalidates_plan_and_restores_home']=True
    report['live_monitor']=dict(clock_samples=len(app.live_monitor_tab._clock_buf),connected=app.live_monitor_tab._running)
    assert all(math.isfinite(v) for s in samples for group in ('positions','velocities') for v in s[group].values())
    report['joint_samples']=len(samples); report['real_time_factor']=(samples[-1]['sim']-samples[0]['sim'])/(samples[-1]['wall']-samples[0]['wall'])
    app._stop_launch(); end=time.monotonic()+30
    while app._launch_running and time.monotonic()<end:pump()
    assert not app._launch_running and app.process.poll()==0
    import re
    until(lambda:bool(re.search(r'\[INFO\] \[move_group-\d+\]: process has finished cleanly',app.output.get('1.0','end'))),3)
    assert 'Segmentation fault' not in app.output.get('1.0','end')
    report['planner_clean_exit']=True
    report['checks']['owned_cleanup']=True; report['passed']=True
except BaseException as exc:
    report['error']=repr(exc); (out/'error.log').write_text(traceback.format_exc());raise
finally:
    if app.process and app.process.poll() is None:
        app._stop_launch();end=time.monotonic()+30
        while app._launch_running and time.monotonic()<end:pump()
    report['launch_returncode']=app.process.poll() if app.process else None
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'trace.json').write_text(json.dumps(dict(joints=samples,arm_status=states,scenes=scene,measured_tcp=observations))+'\n')
    (out/'gui-output.log').write_text(app.output.get('1.0','end'))
    if node:
        observer_executor.shutdown();node.destroy_node()
    app._on_close()
    if rclpy.ok():rclpy.shutdown()
