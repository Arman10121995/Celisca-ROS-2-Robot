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


class Reef:
    """A block on the seabed across the straight line: x 8–10, top at z=-3."""
    def segment(self, a, b, radius, deadline=None, canceled=None):
        for t in np.linspace(0., 1., 12):
            p = np.asarray(a) + t * (np.asarray(b) - np.asarray(a))
            if 8. - radius < p[0] < 10. + radius and -3. + radius > p[2] and abs(p[1]) < 3. + radius:
                return False
        return True


def test_volumetric_profile_climbs_over_an_obstacle_within_the_slope():
    path, info = plan_auv(Reef(), [0., 0., -2.9], math.pi, [22., 0., -2.9], .5, -6., 0., lambda: False,
                          goal_heading=0.)
    points = np.array(path)
    assert info['volumetric'] and info['max_depth_departure_m'] > .35
    above = points[(points[:, 0] > 7.4) & (points[:, 0] < 10.6)]
    assert (above[:, 2] > -2.5).all()                       # clears the reef top plus radius
    slope = np.abs(np.diff(points[:, 2])) / np.maximum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1), 1e-9)
    assert slope.max() <= .1 + 1e-6
    assert points[-1, 2] == pytest.approx(-2.9) and points[0, 2] == pytest.approx(-2.9)


def test_clear_water_keeps_the_straight_depth_ramp():
    path, info = plan_auv(SimpleNamespace(segment=lambda *args: True), [0., 0., -2.], math.pi, [16., 0., -3.],
                          .5, -6., 0., lambda: False, goal_heading=0.)
    assert not info['volumetric'] and info['max_depth_departure_m'] == 0.


def test_current_ranks_by_time_and_refuses_excessive_crab():
    from robot_lab_utils.auv_routes import route_time
    straight = np.array([[0., 0., -2.], [10., 0., -2.]])
    assert route_time(straight, [.1, 0., 0.])[0] == pytest.approx(10. / .4)
    assert route_time(straight, [-.1, 0., 0.])[0] == pytest.approx(10. / .2)
    seconds, crab = route_time(straight, [0., .2, 0.])
    assert seconds is None and crab > .5                     # 0.2 m/s across needs 0.73 rad of crab
    path, info = plan_auv(SimpleNamespace(segment=lambda *args: True), [0., 0., -2.], math.pi, [16., 0., -2.],
                          .5, -6., 0., lambda: False, goal_heading=0., current=[.1, 0., 0.])
    assert info['expected_duration_s'] < info['length_m'] / .3 and info['current']['vector'] == [.1, 0., 0.]
    with pytest.raises(ValueError, match='authored current'):
        plan_auv(SimpleNamespace(segment=lambda *args: True), [0., 0., -2.], math.pi, [16., 0., -2.],
                 .5, -6., 0., lambda: False, goal_heading=0., current=[0., .25, 0.])
