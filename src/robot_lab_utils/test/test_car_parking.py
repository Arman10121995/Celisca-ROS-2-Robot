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
    node = SimpleNamespace(cancel=threading.Event(), state={}, goal_handle=None)
    node.pose = lambda: tuple(car.pose)
    def publish(vx, wz):
        with car.lock:
            car.command = (0., 0.) if node.cancel.is_set() else (vx, wz)
    node.publish = publish
    node.spin = lambda seconds: time.sleep(seconds)
    params = dict(turning_radius=.5, tolerance_deg=3., trim=True)
    node.get_parameter = lambda key: SimpleNamespace(value=params[key])
    result = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(status=nav_status))
    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result, cancel_goal_async=lambda: None)
    sent = SimpleNamespace(done=lambda: True, result=lambda: handle)
    node.nav = SimpleNamespace(wait_for_server=lambda timeout_sec: True, send_goal_async=lambda request: sent)
    car_parking.CarParking.park(node, goal, timeout=60.)
    car.running = False
    return node.state


def test_nav2_arrival_off_heading_is_shuffled_and_trimmed_to_the_bay():
    car = FakeCar(1.9, 0.02, math.radians(75.))            # Nav2 left it 15 deg off, a little short
    state = park(car, dict(x=2.0, y=0.05, yaw=math.radians(90.)))
    assert state['phase'] == 'parked', state
    assert abs(state['nav2_heading_error_deg'] - 15.) < .5
    assert abs(state['heading_error_deg']) <= 3.5
    assert abs(state['along_m']) <= .03 and state['position_error_m'] < .2


def test_failed_nav2_goal_reports_failure_and_stops():
    car = FakeCar(0., 0., 0.)
    state = park(car, dict(x=1., y=0., yaw=0.), nav_status=6)
    assert state['phase'] == 'failed' and 'status 6' in state['reason'] and car.command == (0., 0.)
