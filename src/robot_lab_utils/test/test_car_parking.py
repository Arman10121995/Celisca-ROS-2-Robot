"""Car parking runtime: Nav2 goal, then heading shuffle and depth trim on the estimate (R5.5)."""
import json
import math
import threading
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from robot_lab_utils import car_parking  # noqa: E402
from robot_lab_utils.car_alignment import integrate, Arc  # noqa: E402


class FakeCar:
    """Unicycle at the commanded (vx, wz), integrated by wall time; starts off-heading."""
    def __init__(self, x, y, yaw):
        self.pose = [x, y, yaw]
        self.command = (0., 0.)
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        last = time.monotonic()
        while self.running:
            time.sleep(.005)
            now = time.monotonic()
            dt, last = now - last, now
            with self.lock:
                vx, wz = self.command
                x, y, yaw = self.pose
                self.pose = [x + vx * math.cos(yaw) * dt, y + vx * math.sin(yaw) * dt, yaw + wz * dt]


def park(car, goal, nav_status=4):
    node = SimpleNamespace(cancel=threading.Event(), state={}, goal_handle=None,context=SimpleNamespace(ok=lambda:True))
    node.pose = lambda: tuple(car.pose)
    def publish(vx, wz):
        with car.lock:
            car.command = (0., 0.) if node.cancel.is_set() else (vx, wz)
    node.publish = publish
    node.spin = lambda seconds: time.sleep(seconds)
    params = dict(turning_radius=.5, tolerance_deg=3., trim=True)
    node.get_parameter = lambda key: SimpleNamespace(value=params[key])
    node.check_budget=lambda:None  # Numerical alignment fixture, not a watchdog certificate.
    result = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(status=nav_status))
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result, cancel_goal_async=lambda: None)
    sent = SimpleNamespace(done=lambda: True, result=lambda: handle,add_done_callback=lambda f:None)
    node.nav = SimpleNamespace(wait_for_server=lambda timeout_sec: True, send_goal_async=lambda request: sent)
    car_parking.CarParking.park(node, goal, timeout=60.)
    car.running = False
    return node.state


@pytest.mark.parametrize('initial_x,phase',[(1.95,'parked'),(1.9,'failed')])
def test_correction_only_reports_parked_when_both_heading_and_position_fit(initial_x,phase):
    # The historical 1.9 case leaves >10 cm lateral error. Correcting heading
    # and depth alone must not turn that into a successful parking result.
    car = FakeCar(initial_x, 0.02, math.radians(75.))
    state = park(car, dict(x=2.0, y=0.05, yaw=math.radians(90.)))
    assert state['phase'] == phase, state
    assert abs(state['nav2_heading_error_deg'] - 15.) < .5
    assert abs(state['heading_error_deg']) <= 3.5
    assert abs(state['along_m']) <= .03 and state['position_error_m'] < .2
    assert (state['position_error_m']<=.1)==(phase=='parked')


def test_failed_nav2_goal_reports_failure_and_stops():
    car = FakeCar(0., 0., 0.)
    state = park(car, dict(x=1., y=0., yaw=0.), nav_status=6)
    assert state['phase'] == 'failed' and 'status 6' in state['reason'] and car.command == (0., 0.)


def test_heartbeat_and_whole_operation_deadline_are_independent_of_nav2():
    node=SimpleNamespace(cancel=threading.Event(),last_heartbeat=time.monotonic()-1.,deadline=math.inf,
        get_parameter=lambda key:SimpleNamespace(value=.8),navigator_active=True,lifecycle_received=time.monotonic(),
        state={},active_nav_goals=set(),goal_handle=None)
    with pytest.raises(RuntimeError,match='heartbeat'):car_parking.CarParking.check_budget(node)
    node.last_heartbeat=time.monotonic();node.deadline=time.monotonic()-1.
    with pytest.raises(TimeoutError,match='whole-operation'):car_parking.CarParking.check_budget(node)
    node.cancel.set()
    with pytest.raises(InterruptedError):car_parking.CarParking.check_budget(node)
    node.cancel.clear();node.deadline=math.inf;node.state={'phase':'aligning'};node.active_nav_goals={b'other'}
    with pytest.raises(RuntimeError,match='ownership'):car_parking.CarParking.check_budget(node)


def test_no_timeout_or_cancel_can_leave_a_late_accepted_nav2_goal_running():
    from concurrent.futures import Future
    sent=Future();canceled=[]
    node=SimpleNamespace(cancel=threading.Event(),state={},goal_handle=None,publish=lambda *args:None,
        context=SimpleNamespace(ok=lambda:True))
    calls=[]
    def budget():
        calls.append(1)
        if len(calls)>2:raise TimeoutError('deadline')
    node.check_budget=budget
    node.nav=SimpleNamespace(wait_for_server=lambda **kwargs:True,send_goal_async=lambda _:sent)
    car_parking.CarParking.park(node,dict(x=1.,y=0.,yaw=0.),10.)
    assert node.state['phase']=='failed' and node.cancel.is_set()
    sent.set_result(SimpleNamespace(accepted=True,cancel_goal_async=lambda:canceled.append(True)))
    assert canceled==[True]


def test_arrival_with_wrong_lateral_position_cannot_report_parked():
    car=FakeCar(0.,1.,0.)
    state=park(car,dict(x=0.,y=0.,yaw=0.))
    assert state['phase']=='failed' and state['position_error_m']>.9


def test_server_discovery_cannot_send_a_goal_after_the_lease_expires():
    calls=[];sent=[]
    node=SimpleNamespace(cancel=threading.Event(),state={},goal_handle=None,publish=lambda *args:None)
    def check():
        calls.append(1)
        if len(calls)>1:raise RuntimeError('heartbeat lost during discovery')
    node.check_budget=check
    node.nav=SimpleNamespace(wait_for_server=lambda **kwargs:True,send_goal_async=lambda goal:sent.append(goal))
    car_parking.CarParking.park(node,dict(x=1.,y=0.,yaw=0.),10.)
    assert not sent and node.state['phase']=='failed' and node.cancel.is_set()


@pytest.mark.parametrize('stamp,valid',[(99.,False),(100.5,False),(99.9,True)])
def test_localization_transform_age_is_checked_in_the_ros_clock(stamp,valid):
    from rclpy.time import Time
    from rclpy.clock import ClockType
    t=SimpleNamespace(header=SimpleNamespace(stamp=Time(seconds=stamp).to_msg()),transform=SimpleNamespace(
        translation=SimpleNamespace(x=1.,y=2.),rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
    node=SimpleNamespace(tf=SimpleNamespace(lookup_transform=lambda *args:t),
        get_parameter=lambda key:SimpleNamespace(value='base_footprint'),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=100,clock_type=ClockType.ROS_TIME)))
    if valid:assert car_parking.CarParking.fresh_transform(node,'map')==(1.,2.,0.)
    else:
        with pytest.raises(RuntimeError,match='stale or future'):car_parking.CarParking.fresh_transform(node,'map')
