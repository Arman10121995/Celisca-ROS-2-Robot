"""Curvature geometry and bounded estimated guidance; physical receipts are separate."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src/robot_lab_utils'))
from robot_lab_utils.auv_routes import plan_auv,AuvPathFollower


def curve(scene=None,**kwargs):
    kwargs.setdefault('turn_radius',12.)
    return plan_auv(scene or SimpleNamespace(segment=lambda *args:True),[-3.5,-3.,-2.],math.pi,
        [-2.5+12./math.sqrt(2),-3.+12.*(1.-1/math.sqrt(2)),-2.],1.3,-6.,0.,
        lambda:False,goal_heading=math.pi/4,**kwargs)


def test_pinned_curve_preserves_heading_radius_and_terminal_corridor():
    calls=[]
    path,info=curve(SimpleNamespace(segment=lambda *args:calls.append(args) or True))
    assert info['length_m']==pytest.approx(1.+12.*math.pi/4,abs=1e-5)
    assert info['straight_runup_m']==1.
    assert abs(info['headings_rad'][0])<1e-6
    assert info['headings_rad'][-1]==pytest.approx(math.pi/4)
    assert max(np.linalg.norm(np.diff(path,axis=0),axis=1))<.16
    assert np.linalg.norm(calls[-1][1]-calls[-1][0])==pytest.approx(3.)
    points=np.array(path)
    assert np.allclose(points[:,2],-2.)
    assert np.max(np.abs(np.linalg.norm(points[7:,:2]-[-2.5,9.],axis=1)-12.))<1e-4


def test_no_collision_route_is_not_published_as_a_plan():
    with pytest.raises(ValueError,match='No source-clear'):
        curve(SimpleNamespace(segment=lambda *args:False))


def test_heading_and_nonnegative_turn_radius_are_real_constraints():
    for radius in (0.,-2.,9.,31.,float('nan')):
        with pytest.raises(ValueError):curve(turn_radius=radius)


def test_auv_does_not_pivot_or_sneak_through_a_shallow_surface():
    scene=SimpleNamespace(segment=lambda *args:True)
    with pytest.raises(ValueError,match='2 m'):
        plan_auv(scene,[0.,0.,-2.],0.,[.1,0.,-2.],1.3,-6.,0.,lambda:False)
    with pytest.raises(ValueError,match='depth clearance'):
        plan_auv(scene,[0.,0.,-.5],0.,[3.,0.,-.5],1.3,-6.,0.,lambda:False)
    with pytest.raises(TimeoutError):
        plan_auv(scene,[0.,0.,-2.],0.,[3.,0.,-2.],1.3,-6.,0.,lambda:True)


def test_follower_uses_source_negative_x_and_curvature_feedforward():
    path,info=curve();follower=AuvPathFollower(path,info['headings_rad'])
    command,target,index,brake=follower.command(path[0],math.pi,[0.,0.,0.])
    assert command[0]==.3 and command[1]==command[2]==0.
    assert command[3]==pytest.approx(0.,abs=1e-9)  # Straight flow run-up precedes the curve.
    assert not brake and index==0
    final=np.array(path[-1]);yaw=info['headings_rad'][-1]-math.pi
    for position in path:
        # Advance through measured estimates; a loop cannot be skipped by one distant sample.
        i=int(np.argmin(np.linalg.norm(np.asarray(path)-position,axis=1)))
        heading=info['headings_rad'][i]
        follower.command(position,heading-math.pi,[.3*math.cos(heading),.3*math.sin(heading),0.])
    heading=info['headings_rad'][-1]
    assert follower.command(final,yaw,[.3*math.cos(heading),.3*math.sin(heading),0.])[-1]


def test_unobserved_tracking_error_stops_instead_of_spinning_or_strafing():
    path,info=curve();follower=AuvPathFollower(path,info['headings_rad'])
    with pytest.raises(ValueError,match='tracking envelope'):
        follower.command(np.array(path[0])+[0.,2.,0.],math.pi,[0.,0.,0.])
