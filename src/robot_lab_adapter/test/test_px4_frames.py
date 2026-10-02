"""Check frame conversions against physical basis directions."""
import math
import pytest
from robot_lab_adapter.px4_frames import attitude_enu_flu, enu_to_ned, ned_to_enu


def rotate(q, vector):
    # Independent quaternion rotation by cross products.
    x, y, z, w = q
    vx, vy, vz = vector
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx)


@pytest.mark.parametrize('ned,enu', [((1,0,0),(0,1,0)), ((0,1,0),(1,0,0)), ((0,0,1),(0,0,-1)), ((3,4,-5),(4,3,5))])
def test_positions(ned, enu):
    assert ned_to_enu(*ned) == enu
    assert enu_to_ned(*enu) == ned


@pytest.mark.parametrize('yaw,forward,left', [(0,(0,1,0),(-1,0,0)), (math.pi/2,(1,0,0),(0,1,0)), (math.pi, (0,-1,0),(1,0,0))])
def test_level_attitude_axes(yaw, forward, left):
    q = attitude_enu_flu(0,0,yaw)
    assert rotate(q, (1,0,0)) == pytest.approx(forward, abs=1e-12)
    assert rotate(q, (0,1,0)) == pytest.approx(left, abs=1e-12)
    assert rotate(q, (0,0,1)) == pytest.approx((0,0,1), abs=1e-12)


def test_positive_ned_pitch_tilts_body_forward_upward():
    q = attitude_enu_flu(0, math.pi/6, 0)
    assert rotate(q, (1,0,0)) == pytest.approx((0, math.sqrt(3)/2, 0.5), abs=1e-12)


def test_position_mask_and_distinct_waypoints():
    pytest.importorskip('pymavlink')
    from unittest.mock import Mock
    from robot_lab_adapter.px4_mavlink import Flight
    master = Mock(target_system=1, target_component=1)
    flight = Flight(master)
    flight.send_setpoint(3,4,-5,0.7)
    args = master.mav.set_position_target_local_ned_send.call_args.args
    mask = args[4]
    assert mask & 0x7 == 0  # XYZ must be active
    assert mask & (1 << 10) == 0  # yaw active
    assert mask & 0x1f8 == 0x1f8  # no velocity / acceleration controller
    assert args[5:8] == (3.0,4.0,-5.0)
    assert args[-2:] == (0.7,0.0)
