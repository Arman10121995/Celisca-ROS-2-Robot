"""Arm-only planning seeds must retain monitored gripper and payload state."""
import pytest
from concurrent.futures import Future
from types import SimpleNamespace

pytest.importorskip('moveit_msgs.srv')
from robot_lab_gui.cartesian_controls import CartesianControls


def test_arm_only_fk_ik_and_plan_seed_is_a_state_diff():
    joints={'joint'+str(i):i*.01 for i in range(1,8)}
    request=CartesianControls.fk_request(joints)
    assert request.robot_state.is_diff
    assert request.robot_state.joint_state.name==list(joints)
    assert list(request.robot_state.joint_state.position)==list(joints.values())
    assert request.fk_link_names==['panda_tcp']


def test_bounded_alternate_seeds_preserve_fingers_and_original_start():
    from geometry_msgs.msg import Pose
    panel=CartesianControls.__new__(CartesianControls)
    names=['joint'+str(i) for i in range(1,8)]
    process=object();panel.arm=SimpleNamespace(app=SimpleNamespace(process=process),
        state=dict(joint_names=names,limits=[[-2.,2.]]*7))
    panel.ik_owner=id(process);panel.ik_cache=[dict.fromkeys(names,.3),dict.fromkeys(names,99.)]
    panel.target=Pose();sent=[]
    panel.ik=SimpleNamespace(call_async=lambda request:(sent.append(request),Future())[1])
    original=panel.fk_request(dict(dict.fromkeys(names,.1),finger_joint1=.015,finger_joint2=.015)).robot_state
    panel.solve_ik(1,original)
    panel.next_ik(1,original)
    assert len(panel.ik_attempts)<=2
    assert list(sent[0].ik_request.robot_state.joint_state.position)==[.1]*7+[.015]*2
    assert list(sent[1].ik_request.robot_state.joint_state.position)==[.3]*7+[.015]*2
    assert list(original.joint_state.position)==[.1]*7+[.015]*2
    assert all(request.ik_request.avoid_collisions for request in sent)


def test_alternate_ik_solution_still_plans_from_measured_start():
    panel=CartesianControls.__new__(CartesianControls)
    names=['joint'+str(i) for i in range(1,8)]
    panel.generation=1;panel.ready=panel.unchanged=lambda:True
    panel.arm=SimpleNamespace(state=dict(joint_names=names))
    panel.ik_cache=[];sent=[]
    panel.planner=SimpleNamespace(call_async=lambda request:(sent.append(request),Future())[1])
    measured=panel.fk_request(dict.fromkeys(names,.1)).robot_state
    solution=panel.fk_request(dict.fromkeys(names,.3)).robot_state
    future=Future();future.set_result(SimpleNamespace(error_code=SimpleNamespace(val=1),solution=solution))
    panel.got_ik(future,1,measured)
    assert list(sent[0].motion_plan_request.start_state.joint_state.position)==[.1]*7
    assert [c.position for c in sent[0].motion_plan_request.goal_constraints[0].joint_constraints]==[.3]*7


def test_placement_keeps_one_orientation_despite_compliant_object_rotation():
    import copy
    from robot_lab_gui.panda_object_task import PandaObjectTask
    task=PandaObjectTask.__new__(PandaObjectTask)
    obj=dict(id='cube',frame='native_world',type='box',size=[.03]*3,position=[0.,-.62,.515],
             orientation_wxyz=[1.,0.,0.,0.],plant_id='plant',source_sha256='source')
    attachment=dict(object_id='cube',position=[0.,0.,.006],orientation_wxyz=[1.,0.,0.,0.],
                    plant_id='plant',source_sha256='source')
    support=dict(id='bench',frame='native_world',type='box',size=[.3,.3,.5],
                 position=[0.,-.62,.25],orientation_wxyz=[1.,0.,0.,0.])
    task.grasp=SimpleNamespace(object=obj,catalog=dict(supports={'bench':support}))
    task.hand=SimpleNamespace(sending=False,state=dict(busy=False))
    task.support='bench';task.release_gap=.003;task.lift_height=.05;task.placement=None
    planned=[]
    def plan(pose,label):
        planned.append(copy.deepcopy(pose));task.cart.target=pose;task.cart.planning=True
    task.cart=SimpleNamespace(ready=lambda:True,scene=dict(object_in_tool=attachment,link_padding_m=.01),
                             plan_pose=plan,planning=False,generation=1)
    task.dispatch=lambda fn,*args:fn(*args);task.record=lambda **_:None
    assert task.begin('plan_above')==('plan',1)
    obj['orientation_wxyz']=[.9992,0.,.04,0.]  # Observed compliant rotation.
    task.begin('plan_lower')
    assert planned[0].orientation==planned[1].orientation
    assert abs(planned[0].position.z-planned[1].position.z-.05)<1e-12
    assert task.retreat['position']==[getattr(planned[0].position,a) for a in 'xyz']
