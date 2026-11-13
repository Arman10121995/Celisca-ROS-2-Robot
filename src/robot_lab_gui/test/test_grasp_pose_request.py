"""Explicit object targets retain measured-state seeding and collision-aware IK."""
import pytest
pytest.importorskip('moveit_msgs.srv')
from types import SimpleNamespace
from geometry_msgs.msg import Pose
from robot_lab_gui.cartesian_controls import CartesianControls


def test_object_pose_uses_existing_seeded_moveit_ik_without_executing():
    calls=[];text=[]
    control=CartesianControls.__new__(CartesianControls)
    control.ready=lambda:True;control.planning=False;control.reading_pose=False
    control.generation=0;control.points=None;control.start=None
    control.execute_button=SimpleNamespace(state=lambda *a:None)
    control.status=control.target_display=SimpleNamespace(set=text.append)
    control.ik=SimpleNamespace(service_is_ready=lambda:True,
        call_async=lambda request:(calls.append(request) or SimpleNamespace(add_done_callback=lambda fn:None)))
    control.planner=SimpleNamespace(service_is_ready=lambda:True)
    control.joints={**{'joint'+str(i):i*.1 for i in range(1,8)},'finger_joint1':.04,'finger_joint2':.04}
    pose=Pose();pose.position.x=.4;pose.position.z=.3;pose.orientation.w=1.
    control.plan_pose(pose,'test candidate')
    assert len(calls)==1 and control.planning
    request=calls[0].ik_request
    assert request.avoid_collisions and request.group_name=='panda_arm' and request.ik_link_name=='panda_tcp'
    assert request.pose_stamped.header.frame_id=='native_world'
    assert dict(zip(request.robot_state.joint_state.name,request.robot_state.joint_state.position))==control.joints
    control.planning=False;pose.position.x=float('nan');control.plan_pose(pose)
    assert len(calls)==1
