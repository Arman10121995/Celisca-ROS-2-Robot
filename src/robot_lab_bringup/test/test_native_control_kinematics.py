"""Real native FK/Jacobian checks for source actuator and mimic/tendon contracts."""
import math
import numpy as np
import pytest

mj=pytest.importorskip('mujoco')
from robot_lab_utils.native_actuation import position_channels
from robot_lab_utils.native_control_kinematics import ControlKinematics


XML='''<mujoco><compiler autolimits="true"/><worldbody>
<body name="base"><freejoint/><geom type="box" size=".1 .1 .1" mass="1"/>
<body name="lift"><joint name="lift_j" type="slide" axis="0 0 1" range="0 1"/>
<geom type="sphere" size=".01" mass=".1"/>
<body pos=".02 0 0"><joint name="arm0" type="slide" axis="1 0 0" range="0 .2"/>
<geom type="sphere" size=".01" mass=".1"/>
<body pos=".02 0 0"><joint name="arm1" type="slide" axis="1 0 0" range="0 .2"/>
<geom type="sphere" size=".01" mass=".1"/>
<body name="left" pos="0 .02 0"><joint name="grip_j" type="slide" axis="0 1 0" range="0 .04"/>
<geom type="box" size=".01 .005 .01" mass=".01"/></body>
<body name="right" pos="0 -.02 0"><geom type="box" size=".01 .005 .01" mass=".01"/></body>
</body></body></body></body>
<body name="object" pos=".25 0 .4"><freejoint/><geom type="sphere" size=".01" mass=".01"/></body>
</worldbody><tendon><fixed name="extension"><joint joint="arm0" coef="1"/><joint joint="arm1" coef="1"/></fixed></tendon>
<equality><joint joint1="arm0" joint2="arm1"/></equality>
<actuator><position name="lift" joint="lift_j" kp="100" ctrlrange="0 1"/>
<position name="arm" tendon="extension" kp="100" ctrlrange="0 .4"/>
<position name="grip" joint="grip_j" kp="100" ctrlrange="0 .04"/></actuator></mujoco>'''


@pytest.fixture
def plant():
    model=mj.MjModel.from_xml_string(XML);data=mj.MjData(model);mj.mj_forward(model,data)
    channels=position_channels(model,model.njnt-1,allow_mobile_base=True)
    return model,data,channels


def test_fixed_tendon_and_mimic_fk_preserve_original_plant(plant):
    model,data,channels=plant;before=data.qpos.copy();gain=model.actuator_gainprm.copy()
    ik=ControlKinematics(model,data,channels,model.njnt-1)
    probe=ik.forward([.3,.2,.01])
    assert probe.qpos[model.jnt_qposadr[2]]==pytest.approx(.1)
    assert probe.qpos[model.jnt_qposadr[3]]==pytest.approx(.1)
    assert list(probe.actuator_length)==pytest.approx([.3,.2,.01])
    np.testing.assert_array_equal(data.qpos,before)
    np.testing.assert_array_equal(model.actuator_gainprm,gain)


def test_analytic_control_jacobian_matches_actual_coupled_fk(plant):
    model,data,channels=plant
    ik=ControlKinematics(model,data,channels,model.njnt-1)
    controls=np.array([.3,.2,.01]);ik.forward(controls)
    bodies=[mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,n) for n in ('left','right')]
    jac=ik.midpoint_jacobian(bodies);origin=ik.midpoint(bodies).copy()
    for i in range(3):
        moved=controls.copy();moved[i]+=1e-5;ik.forward(moved)
        np.testing.assert_allclose((ik.midpoint(bodies)-origin)/1e-5,jac[:,i],atol=1e-7)


def test_local_reach_uses_native_geometry_and_refuses_unreachable_target(plant):
    model,data,channels=plant
    ik=ControlKinematics(model,data,channels,model.njnt-1)
    bodies=[mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,n) for n in ('left','right')]
    solved=ik.reach([.1,.1,.01],bodies,[.25,.005,.4],[0,1])
    ik.forward(solved)
    np.testing.assert_allclose(ik.midpoint(bodies),[.25,.005,.4],atol=.008)
    with pytest.raises(ValueError,match='outside'):
        ik.reach([.1,.1,.01],bodies,[4.,.005,.4],[0,1])


def test_source_bounds_and_nonlinear_coupling_are_never_silently_replaced(plant):
    model,data,channels=plant
    ik=ControlKinematics(model,data,channels,model.njnt-1)
    with pytest.raises(ValueError,match='limits'):
        ik.forward([.1,.5,.01])
    model.eq_data[0,2]=1.
    with pytest.raises(ValueError,match='Nonlinear'):
        ControlKinematics(model,data,channels,model.njnt-1)


def test_rgbd_cylinder_uses_camera_position_and_checks_compiled_collision(monkeypatch):
    from types import SimpleNamespace
    from robot_lab_mujoco import native_mobile_reach
    from robot_lab_mujoco.native_mobile_tasks import NativeMobileTasks
    from robot_lab_utils.object_estimates import ObjectEstimates, RGBD_PRIMITIVE_SOURCE

    model = mj.MjModel.from_xml_string(XML.replace('type="sphere" size=".01" mass=".01"',
                                                  'type="cylinder" size=".0175 .03" mass=".035"'))
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    object_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'object')
    task = NativeMobileTasks.__new__(NativeMobileTasks)
    task.node = SimpleNamespace(model=model, data=data, mujoco=mj, robot_bodies=object_id, robot_joints=model.njnt-1)
    task.arm = SimpleNamespace(plant_id='plant', source_sha256='source', channels=[],
                               measured_controls=lambda: np.array([]))
    task.scene_id = 'scene'
    task.mobile = SimpleNamespace(joints=[], moving=lambda: False)
    task.pose = lambda: [0., 0., 0.]
    task.camera_approach, task.index = None, 1
    task.gripped_body = task.payload_body = task.reach_worker = None
    task.generation = 1
    task.record = lambda **kwargs: None
    task.enter = lambda *args: None
    camera = ObjectEstimates(RGBD_PRIMITIVE_SOURCE)
    task.perception_estimates = {RGBD_PRIMITIVE_SOURCE: camera}
    target = dict(object='object', position=[.27, .02, .39], orientation_wxyz=[1., 0., 0., 0.],
                  observation_id='image', position_std_bound_m=.004, type='cylinder',
                  pose_reference='collision_cylinder_center', size=[.035, .035, .06])
    identity = task.estimate_identity()
    record = dict(schema='robot_lab.object_estimates.v1', **identity, frame='native_world',
                  stamp_s=data.time, pose_source=RGBD_PRIMITIVE_SOURCE, targets={'object': target})
    step = dict(object='object', left_body='left', right_body='right', pose_source=RGBD_PRIMITIVE_SOURCE,
                offset=[0., 0., .08])
    planned = []
    monkeypatch.setattr(native_mobile_reach, 'plan_reach',
                        lambda node, snapshot, channels, initial, step, *args: planned.append(step) or {})

    target['size'] = [.04, .04, .06]
    camera.receive(record, identity, data.time)
    with pytest.raises(ValueError, match='dimensions'):
        task.begin_reach(step)
    target['size'] = [.035, .035, .06]
    camera.receive(record, identity, data.time)
    task.gripped_body = object_id
    with pytest.raises(ValueError, match='unheld object'):
        task.begin_reach(step)
    task.gripped_body = None
    task.begin_reach(step)
    task.reach_worker.join(timeout=2.)
    assert len(planned) == 1
    np.testing.assert_allclose(planned[0]['target_xyz'], [.27, .02, .47])
    task.index += 1
    data.time = 11.
    camera.receive(dict(record, stamp_s=data.time, targets={}), identity, data.time)
    descent = dict(step, offset=[0., 0., 0.], reuse_camera_on_occlusion=True)
    anchor = task.camera_approach
    task.begin_reach(descent)
    task.reach_worker.join(timeout=2.)
    np.testing.assert_allclose(planned[-1]['target_xyz'], [.27, .02, .39])
    task.camera_approach = anchor
    data.xpos[object_id, 0] += .02
    with pytest.raises(ValueError, match='object moved'):
        task.begin_reach(descent)


def test_native_placement_uses_pad_offset_and_requires_contact_on_the_named_support():
    from types import SimpleNamespace
    from robot_lab_mujoco.native_mobile_tasks import NativeMobileTasks

    scene = XML.replace('<body name="object"', '''
      <body name="spare_pad"><geom type="sphere" size=".01"/></body>
      <body name="object"''').replace('</worldbody>', '''
      <geom name="delivery" type="box" size=".2 .1 .25" pos=".55 0 .25"/>
      <geom name="other" type="box" size=".2 .1 .25" pos="1.2 0 .25"/>
    </worldbody>''')
    model = mj.MjModel.from_xml_string(scene)
    data = mj.MjData(model)
    mj.mj_forward(model, data)
    obj = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'object')
    support = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, 'delivery')
    other = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, 'other')
    step = dict(object='object', left_body='left', right_body='right', support_geom='delivery',
                offset=[0., 0., 0.])
    task = NativeMobileTasks.__new__(NativeMobileTasks)
    task.node = SimpleNamespace(model=model, data=data, mujoco=mj, robot_bodies=obj)
    task.support_candidates = {'delivery': support, 'other': other}
    task.payload_confirmed = True
    task.gripped_body = task.payload_body = obj
    task.gripped_step = dict(step, condition='grasped')
    task.gripped_step.pop('support_geom')
    measure = task.measure_object
    task.measure_object = lambda step: dict(supported=False, left_force=.2, right_force=.3)
    np.testing.assert_allclose(task._support_target(step), [.34, 0., .12], atol=1e-7)
    with pytest.raises(ValueError, match='pads that grasped'):
        task._support_target(dict(step, left_body='spare_pad'))
    with pytest.raises(ValueError, match='does not fit'):
        task._support_target(dict(step, offset=[0., .22, 0.]))
    task.measure_object = measure
    start = model.jnt_qposadr[model.body_jntadr[obj]]
    data.qpos[start:start+3] = [.55, 0., .508]
    mj.mj_forward(model, data)
    assert task.measure_object(dict(step, condition='released'))['supported']
    assert not task.measure_object(dict(step, condition='released', support_geom='other'))['supported']


def test_planner_honours_deadline(plant):
    model,data,channels=plant
    ik=ControlKinematics(model,data,channels,model.njnt-1)
    bodies=[mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,n) for n in ('left','right')]
    with pytest.raises(ValueError,match='deadline'):
        ik.reach([.1,.1,.01],bodies,[.25,.005,.4],[0,1],deadline=0.)


def test_pose_ik_uses_actual_first_pad_orientation_and_refuses_unavailable_rotation(plant):
    model,data,channels=plant
    ik=ControlKinematics(model,data,channels,model.njnt-1)
    bodies=[mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,n) for n in ('left','right')]
    solved=ik.reach([.1,.1,.01],bodies,[.25,.005,.4],[0,1],orientation=[1.,0.,0.,0.])
    ik.forward(solved)
    np.testing.assert_allclose(ik.midpoint(bodies),[.25,.005,.4],atol=.008)
    with pytest.raises(ValueError,match='orientation is unreachable'):
        ik.reach([.1,.1,.01],bodies,[.25,.005,.4],[0,1],orientation=[.9238795,0.,0.,.3826834])


def test_pose_ik_turns_a_real_authored_wrist_to_the_requested_orientation():
    from scipy.spatial.transform import Rotation
    source=XML.replace('<compiler autolimits="true"/>','<compiler autolimits="true" angle="radian"/>')
    source=source.replace('<body name="lift">',
        '<body name="wrist"><joint name="wrist_j" type="hinge" axis="0 0 1" range="-1 1"/>'
        '<geom type="sphere" size=".01" mass=".1"/><body name="lift">')
    source=source.replace('</body></body></body></body>','</body></body></body></body></body>')
    source=source.replace('</actuator>','<position name="wrist_yaw" joint="wrist_j" kp="100" ctrlrange="-1 1"/></actuator>')
    model=mj.MjModel.from_xml_string(source);data=mj.MjData(model);mj.mj_forward(model,data)
    before=data.qpos.copy();gains=model.actuator_gainprm.copy()
    channels=position_channels(model,model.njnt-1,allow_mobile_base=True)
    ik=ControlKinematics(model,data,channels,model.njnt-1)
    bodies=[mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,n) for n in ('left','right')]
    controls=np.array([.4,.18,.01,.5]);ik.forward(controls)
    target=ik.midpoint(bodies).copy();orientation=ik.probe.xquat[bodies[0]].copy()
    result=ik.reach([.2,.1,.01,0.],bodies,target,[0,1,3],orientation=orientation)
    ik.forward(result)
    np.testing.assert_allclose(ik.midpoint(bodies),target,atol=.008)
    error=Rotation.from_matrix(ik.probe.xmat[bodies[0]].reshape(3,3)@
        Rotation.from_quat(orientation[[1,2,3,0]]).as_matrix().T).magnitude()
    assert error<.05 and result[2]==pytest.approx(.01)
    np.testing.assert_array_equal(data.qpos,before)
    np.testing.assert_array_equal(model.actuator_gainprm,gains)


def test_reach_worker_returns_real_source_motor_waypoints_and_refuses_collisions(plant):
    from types import SimpleNamespace
    from robot_lab_mujoco.native_mobile_reach import plan_reach
    model,data,channels=plant
    ik=ControlKinematics(model,data,channels,model.njnt-1)
    initial=[.1,.1,.01];snapshot=ik.forward(initial)
    step=dict(kind='reach_object',object='object',left_body='left',right_body='right',offset=[0.,.005,.08])
    node=SimpleNamespace(model=model,mujoco=mj)
    plan=plan_reach(node,snapshot,channels,initial,step,7,model.njnt-1,[])
    assert 1<=len(plan['waypoints'])<=64
    last=[plan['waypoints'][-1]['targets'][c['name']] for c in channels]
    ik.forward(last)
    np.testing.assert_allclose(ik.midpoint([5,6]),[.25,.005,.48],atol=.008)
    np.testing.assert_array_equal(data.qpos,model.qpos0)
    blocked=mj.MjModel.from_xml_string(XML.replace('</worldbody>',
        '<geom name="blocker" type="box" pos=".25 .005 .48" size=".03 .03 .03"/></worldbody>'))
    blocked_data=mj.MjData(blocked);mj.mj_forward(blocked,blocked_data)
    blocked_channels=position_channels(blocked,blocked.njnt-1,allow_mobile_base=True)
    blocked_ik=ControlKinematics(blocked,blocked_data,blocked_channels,blocked.njnt-1)
    blocked_snapshot=blocked_ik.forward(initial)
    with pytest.raises(ValueError,match='collision'):
        plan_reach(SimpleNamespace(model=blocked,mujoco=mj),blocked_snapshot,blocked_channels,
                   initial,step,7,blocked.njnt-1,[])


def test_stow_uses_source_telescope_minimum_without_releasing_grip(plant):
    from types import SimpleNamespace
    from robot_lab_mujoco.native_mobile_reach import plan_reach
    model,data,channels=plant;initial=[.2,.2,.03]
    ik=ControlKinematics(model,data,channels,model.njnt-1);snapshot=ik.forward(initial)
    step=dict(kind='stow_arm',object='object',left_body='left',right_body='right')
    plan=plan_reach(SimpleNamespace(model=model,mujoco=mj),snapshot,channels,initial,step,7,model.njnt-1,[])
    target=plan['waypoints'][-1]['targets']
    assert target['arm']==pytest.approx(0.)
    assert target['grip']==pytest.approx(.03)
    assert target['lift']==pytest.approx(.2)


def test_keep_measured_orientation_is_carried_through_the_actual_reach_result(plant):
    from types import SimpleNamespace
    from robot_lab_mujoco.native_mobile_reach import plan_reach
    model,data,channels=plant;initial=[.2,.1,.01]
    ik=ControlKinematics(model,data,channels,model.njnt-1);snapshot=ik.forward(initial)
    step=dict(kind='reach_object',object='object',left_body='left',right_body='right',
              target_xyz=[.25,.005,.48],keep_orientation=True)
    plan=plan_reach(SimpleNamespace(model=model,mujoco=mj),snapshot,channels,initial,step,7,model.njnt-1,[])
    assert plan['orientation_body']=='left'
    np.testing.assert_allclose(plan['target_orientation_wxyz'],[1.,0.,0.,0.])
    for point in plan['waypoints']:
        ik.forward([point['targets'][channel['name']] for channel in channels])
        np.testing.assert_allclose(ik.probe.xquat[5],[1.,0.,0.,0.])
    explicit=dict(step,keep_orientation=False,orientation_wxyz=[.99,0.,0.,0.])
    result=plan_reach(SimpleNamespace(model=model,mujoco=mj),snapshot,channels,initial,explicit,7,model.njnt-1,[])
    np.testing.assert_allclose(result['target_orientation_wxyz'],[1.,0.,0.,0.])


@pytest.mark.parametrize('keep,finished,should_stop',[(True,False,True),(False,False,False),(False,True,True)])
def test_live_task_orientation_guard_checks_held_paths_and_explicit_pose_endpoints(keep,finished,should_stop):
    pytest.importorskip('nav2_msgs.action')
    import time
    from types import SimpleNamespace
    from robot_lab_mujoco.native_mobile_tasks import NativeMobileTasks
    task=NativeMobileTasks.__new__(NativeMobileTasks);events=[]
    task.steps=[dict(kind='reach_object',keep_orientation=keep)];task.index=0
    task.active=lambda:task.steps is not None
    task.last_heartbeat=task.started=task.phase_started=time.monotonic()
    task.state={'state':'articulating'};task.sequence_id='owned'
    task.reach_orientation=(0,np.array([1.,0.,0.,0.]))
    task.node=SimpleNamespace(data=SimpleNamespace(time=0., qpos=np.zeros(1),
                                                  xquat=np.array([[math.cos(.1),0.,0.,math.sin(.1)]])))
    task.node.get_parameter=lambda key:SimpleNamespace(value='display')
    task.node.sensors=SimpleNamespace(footprint_receipts=SimpleNamespace(received={}))
    task.mobile=SimpleNamespace(fault='',moving=lambda:False,payload_footprint_ready=lambda:True)
    task.arm=SimpleNamespace(fault='',plant_id='plant',source_sha256='source',sequence_state=dict(sequence_id='owned',
        state='measured_waypoints_reached' if finished else 'executing'))
    task.gripped_step=None;task.custody=None
    task.publisher=SimpleNamespace(publish=lambda message:None)
    task.scene_id='scene';task.pose=lambda:[0.,0.,0.];task.pending=False;task.handle=None
    from robot_lab_utils.object_estimates import ObjectEstimates
    task.object_estimates=ObjectEstimates();task.perception_error=''
    task.perception_estimates={'aruco_pnp':task.object_estimates,'rgbd_box_top':ObjectEstimates('rgbd_box_top')}
    task.object_candidates=[];task.finger_candidates=[];task.support_candidates={}
    task.payload_body=None;task.payload_confirmed=False
    task.event_path=None;task.object_outcome='not assessed';task.object_measurement={}
    def cancel(reason,failed=False):events.append(reason);task.steps=None
    task.cancel=cancel;task.tick()
    assert bool(events)==should_stop


def test_configuration_search_uses_actual_geometry_to_route_around_a_blocker():
    from types import SimpleNamespace
    from robot_lab_mujoco.native_mobile_reach import plan_reach
    model=mj.MjModel.from_xml_string(XML.replace('</worldbody>',
        '<geom name="barrier" type="box" pos=".20 0 .32" size=".022 .12 .09"/></worldbody>'))
    data=mj.MjData(model);mj.mj_forward(model,data)
    original=data.qpos.copy();gains=model.actuator_gainprm.copy()
    channels=position_channels(model,model.njnt-1,allow_mobile_base=True)
    initial=np.array([.2,.1,.01])
    ik=ControlKinematics(model,data,channels,model.njnt-1)
    snapshot=ik.forward(initial)
    step=dict(kind='reach_object',object='object',left_body='left',right_body='right',
              target_xyz=[.28,.005,.48])
    route=plan_reach(SimpleNamespace(model=model,mujoco=mj),snapshot,channels,initial,
                     step,7,model.njnt-1,[])
    assert route['planner']['algorithm']=='rrt_connect'
    barrier_id=mj.mj_name2id(model,mj.mjtObj.mjOBJ_GEOM,'barrier')
    # Independently resample every returned segment more densely, following
    # the real rate-clipped motor schedule, and inspect native contacts.
    origin=initial.copy();rates=np.array([c['max_rate'] for c in channels])
    for waypoint in route['waypoints']:
        end=np.array([waypoint['targets'][c['name']] for c in channels])
        assert end[2]==pytest.approx(initial[2])
        delta=end-origin;duration=max(abs(delta)/rates)
        for elapsed in np.linspace(0,duration,101):
            probe=ik.forward(origin+np.sign(delta)*np.minimum(abs(delta),rates*elapsed))
            assert not any(barrier_id in contact.geom and contact.dist<-.001
                           for contact in probe.contact[:probe.ncon])
        origin=end
    np.testing.assert_allclose(ik.midpoint([5,6]),step['target_xyz'],atol=.008)
    np.testing.assert_array_equal(data.qpos,original)
    np.testing.assert_array_equal(model.actuator_gainprm,gains)


def test_alternate_real_wrist_ik_avoids_the_first_colliding_goal_without_moving_grip():
    from types import SimpleNamespace
    from robot_lab_mujoco.native_mobile_reach import plan_reach
    source=XML.replace('<compiler autolimits="true"/>','<compiler autolimits="true" angle="radian"/>')
    # Keep the free object out of the route: this case isolates the wrist bar
    # and goal blocker rather than also requiring a detour around the object.
    source=source.replace('<body name="object" pos=".25 0 .4">','<body name="object" pos=".6 0 .15">')
    source=source.replace('<body name="left"',
        '<body name="wrist"><joint name="wrist_j" type="hinge" axis="0 0 1" range="-3 3"/>'
        '<geom name="tool_bar" type="box" pos=".04 0 0" size=".04 .01 .01" mass=".02"/>'
        '<body name="left"')
    source=source.replace('</body></body></body></body>','</body></body></body></body></body>')
    source=source.replace('</actuator>','<position name="wrist_yaw" joint="wrist_j" kp="100" ctrlrange="-3 3"/></actuator>')
    source=source.replace('</worldbody>',
        '<geom name="wrist_blocker" type="box" pos=".30 0 .48" size=".02 .025 .025"/></worldbody>')
    model=mj.MjModel.from_xml_string(source);data=mj.MjData(model);mj.mj_forward(model,data)
    channels=position_channels(model,model.njnt-1,allow_mobile_base=True)
    original=data.qpos.copy();gains=model.actuator_gainprm.copy()
    initial=np.array([.2,.1,0.,0.]);ik=ControlKinematics(model,data,channels,model.njnt-1)
    snapshot=ik.forward(initial)
    step=dict(kind='reach_object',object='object',left_body='left',right_body='right',target_xyz=[.25,0.,.48])
    route=plan_reach(SimpleNamespace(model=model,mujoco=mj),snapshot,channels,initial,step,8,model.njnt-1,[])
    assert route['planner']['ik_attempts'][0]['result']=='goal_collision'
    assert route['planner']['selected_ik_attempt']>0
    blocker=mj.mj_name2id(model,mj.mjtObj.mjOBJ_GEOM,'wrist_blocker')
    rates=np.array([c['max_rate'] for c in channels]);origin=initial.copy()
    for waypoint in route['waypoints']:
        end=np.array([waypoint['targets'][c['name']] for c in channels])
        assert end[2]==pytest.approx(0.)
        delta=end-origin;duration=max(abs(delta)/rates)
        for elapsed in np.linspace(0.,duration,101):
            probe=ik.forward(origin+np.sign(delta)*np.minimum(abs(delta),rates*elapsed))
            assert not any(blocker in contact.geom and contact.dist<-.001 for contact in probe.contact[:probe.ncon])
        origin=end
    pads=[mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,n) for n in ('left','right')]
    np.testing.assert_allclose(ik.midpoint(pads),step['target_xyz'],atol=.008)
    np.testing.assert_array_equal(data.qpos,original)
    np.testing.assert_array_equal(model.actuator_gainprm,gains)


def test_contact_blocked_settling_exemption_is_only_for_closing_gripper():
    pytest.importorskip('nav2_msgs.action')
    from types import SimpleNamespace
    from robot_lab_mujoco.native_mobile_tasks import NativeMobileTasks
    task=NativeMobileTasks.__new__(NativeMobileTasks)
    task.gripped_step,task.gripped_body={},7
    task.active=lambda:True
    task.arm=SimpleNamespace(channels=[dict(name='grip'),dict(name='lift')],
        measured_controls=lambda:np.array([.01,.2]),target=np.array([0.,.4]))
    task.measure_object=lambda step:dict(left_force=1.,right_force=1.)
    np.testing.assert_array_equal(task.grasp_settling_mask(),[True,False])
    task.arm.target[0]=.04
    assert not np.any(task.grasp_settling_mask())
    task.arm.target[0]=0.
    task.measure_object=lambda step:dict(left_force=1.,right_force=0.)
    assert not np.any(task.grasp_settling_mask())


def test_articulation_keeps_other_joints_velocity_and_command_settling_checks():
    pytest.importorskip('std_msgs.msg')
    import time
    from types import SimpleNamespace
    from robot_lab_mujoco.native_articulation_control import NativeArticulationControl
    arm=NativeArticulationControl.__new__(NativeArticulationControl)
    arm.channels=[dict(name='grip',limits=[0.,.04],max_rate=.05,units='m'),
                  dict(name='lift',limits=[0.,1.],max_rate=.05,units='m')]
    arm.target=np.array([0.,.2]);arm.commanded=arm.target.copy()
    arm.measured_controls=lambda:np.array([.01,.2]);arm.measured_rates=lambda:np.zeros(2)
    arm.node=SimpleNamespace(mobile_tasks=SimpleNamespace(grasp_settling_mask=lambda:np.array([True,False])))
    arm.sequence=[(arm.target,.2)];arm.sequence_index=0;arm.sequence_state={}
    arm.sequence_started_wall=arm.waypoint_started_wall=time.monotonic()
    arm.data=SimpleNamespace(time=0.);arm.settled_since=None
    arm.advance_sequence();arm.data.time=.3;arm.advance_sequence()
    assert arm.sequence is None
    arm.sequence=[(arm.target,.2)];arm.sequence_index=0;arm.settled_since=None
    arm.measured_controls=lambda:np.array([.01,.1])
    arm.advance_sequence();assert arm.settled_since is None  # lift error still blocks
    arm.measured_controls=lambda:np.array([.01,.2]);arm.measured_rates=lambda:np.array([.1,0.])
    arm.advance_sequence();assert arm.settled_since is None  # moving grip still blocks
