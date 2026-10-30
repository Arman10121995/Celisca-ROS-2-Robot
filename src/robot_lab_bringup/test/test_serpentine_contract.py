"""Actual source chain/encoder constraints; these tests are not missions."""
from pathlib import Path
import math
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/robot_lab_utils'),str(ROOT/'src/robot_lab_mujoco/python')]
from robot_lab_utils.serpentine_model import SerpentineModel,encoder_velocity
from robot_lab_mujoco.joint_pd import JointPDCommand


def model(description=None):
    folder=ROOT/'src/robot_lab_robots/lab_snake'
    return SerpentineModel(description or (folder/'urdf/lab_snake.urdf').read_text(),folder/'config/gait.yaml')


def test_source_wheel_radius_and_chain_changes_are_refused():
    source=ROOT/'src/robot_lab_robots/lab_snake/urdf/lab_snake.urdf'
    root=ET.parse(source).getroot()
    root.find("joint[@name='spine_1']/origin").set('xyz','-.09 0 0')
    with pytest.raises(ValueError,match='geometry'):model(ET.tostring(root,encoding='unicode'))
    root=ET.parse(source).getroot()
    root.find("link[@name='wheel_0_left']/collision/geometry/cylinder").set('radius','.04')
    with pytest.raises(ValueError,match='geometry'):model(ET.tostring(root,encoding='unicode'))


def test_curved_rolling_body_is_reconstructed_without_truth_and_slip_is_refused():
    body=model();angle=-.12;omega=.1
    points=body.wheels({name:angle for name in body.names})
    centre=-body.config['segment_length']/2/math.tan(angle/2)
    actual=np.array([omega*centre,0.])
    rates={name:float(np.array([math.cos(yaw),math.sin(yaw)])@(actual+omega*np.array([-p[1],p[0]])))/body.config['wheel_radius']
           for name,(p,yaw) in points.items()}
    measured,scatter=encoder_velocity(points,points,rates,set(points),.02,omega,body.config['wheel_radius'])
    np.testing.assert_allclose(measured,actual,atol=1e-12);assert scatter<1e-12
    with pytest.raises(ValueError,match='contacts'):
        encoder_velocity(points,points,rates,set(list(points)[:2]),.02,omega,body.config['wheel_radius'])
    rates['wheel_3_left']+=50.
    with pytest.raises(ValueError,match='slip'):
        encoder_velocity(points,points,rates,set(points),.02,omega,body.config['wheel_radius'])


def test_native_timeout_holds_last_bounded_spine_target_without_actuating_wheels():
    body=model();config=body.pd_config();pd=JointPDCommand(config)
    assert len(pd.names)==9 and not any('wheel' in name for name in pd.names)
    target,ik,stance=body.targets(.3,[.05,0,.02],body.initial,'serpenoid')
    assert ik is None and stance=={}
    assert np.all(target>=body.lower) and np.all(target<=body.upper)
    assert pd.receive(target,10.)
    np.testing.assert_array_equal(pd.command(100.),target)
    assert pd.stale(100.)
    legacy=JointPDCommand(dict(config,stale_mode='default'))
    assert legacy.receive(target,10.)
    np.testing.assert_array_equal(legacy.command(100.),body.initial)


def test_spine_motion_does_not_invent_zero_lateral_speed_at_every_segment():
    body=model();dt=.02;omega=.1;actual=np.array([.06,.025])
    old=body.wheels({name:-.11 for name in body.names})
    current=body.wheels({name:-.12 for name in body.names})
    rates={}
    for name,(point,heading) in current.items():
        relative=(point-old[name][0])/dt+omega*np.array([-point[1],point[0]])
        rates[name]=float(np.array([math.cos(heading),math.sin(heading)])@(actual+relative))/body.config['wheel_radius']
    measured,scatter=encoder_velocity(old,current,rates,set(current),dt,omega,body.config['wheel_radius'])
    np.testing.assert_allclose(measured,actual,atol=1e-12);assert scatter<1e-12


def test_simulation_age_and_new_missing_motion_bound_gate_localization_recovery():
    import time
    from types import SimpleNamespace
    from robot_lab_utils.gait_controller import GaitController
    node=SimpleNamespace(rolling_valid=True,rolling_at=time.monotonic(),rolling_pose_uncertain=True,
        sim_time=20.,localization_sim_stamp=18.5,localization_at=time.monotonic()-3.,
        rolling_missing_bound=.12,localization_missing_bound=.1)
    assert GaitController.rolling_navigation_ready(node)
    node.rolling_at=time.monotonic()-1.
    assert not GaitController.rolling_navigation_ready(node)
    node.rolling_at=time.monotonic();node.rolling_missing_bound=.2
    assert not GaitController.rolling_navigation_ready(node)
    node.rolling_missing_bound=.12;node.sim_time=22.
    assert not GaitController.rolling_navigation_ready(node)


def test_instantaneous_spine_jacobian_matches_source_chain_motion():
    body=model();angles={name:.1*math.sin(i) for i,name in enumerate(body.names)}
    rates={name:.2*math.cos(i) for i,name in enumerate(body.names)};step=1e-7
    current,velocity=body.wheel_kinematics(angles,rates)
    advanced=body.wheels({name:angles[name]+step*rates[name] for name in body.names})
    for name in current:
        np.testing.assert_allclose((advanced[name][0]-current[name][0])/step,velocity[name],atol=1e-7)


def test_goal_terminal_holds_shape_and_refuses_queued_old_velocity():
    from types import SimpleNamespace
    from action_msgs.msg import GoalStatusArray,GoalStatus
    from geometry_msgs.msg import Twist
    from robot_lab_utils.gait_controller import GaitController
    target=np.linspace(-.2,.2,9);calls=[]
    node=SimpleNamespace(navigation_goal=None,navigation_terminal=None,navigation_motion_active=True,
        command=np.array([.02,0.,.01]),filtered=np.array([.02,0.,.01]),cmd_at=1.,enabled=True,
        input_owner='navigation',reason='',targets=target.copy(),phase=.37,drive=calls.append)
    status=GoalStatus();status.goal_info.goal_id.uuid=list(range(16));status.status=4
    GaitController.observe_navigation_status(node,GoalStatusArray(status_list=[status]))
    np.testing.assert_array_equal(node.targets,target);assert node.phase==.37
    assert np.linalg.norm(node.filtered)==0 and not node.navigation_motion_active
    stale=Twist();stale.linear.x=.08;GaitController.nav_drive(node,stale);assert calls==[]
