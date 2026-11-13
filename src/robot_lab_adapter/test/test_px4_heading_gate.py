"""Takeoff waits for a steady FCU heading at rest; a drifting PX4 estimator is restarted (R5.11)."""
import math
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from robot_lab_adapter import px4_ros_controller as pc
from robot_lab_adapter.px4_ros_controller import PX4Controller


def controller(drift_deg_s, climb=0.):
    restarts = []
    c = SimpleNamespace(has_armed=False, phase='idle', heading_samples=[], estimator_restarts=0,
                        estimator_restart=lambda: restarts.append(1), first_ready_fcu_ms=0,
                        preflight_fcu_seconds=8., get_logger=lambda: SimpleNamespace(warning=lambda *_: None))
    for name in ('heading_drift', 'height_unsettled', 'heading_settled', 'watch_heading'):
        setattr(c, name, getattr(PX4Controller, name).__get__(c))
    state = dict(ms=8000)
    c.flight = SimpleNamespace(armed=lambda: False,
                               local_position=SimpleNamespace(time_boot_ms=8000, z=0., vz=0.),
                               attitude=SimpleNamespace(yaw=1.5))

    def advance(seconds):
        for _ in range(int(seconds*20)):
            state['ms'] += 50
            c.flight.local_position.time_boot_ms = state['ms']
            c.flight.attitude.yaw = 1.5 + math.radians(drift_deg_s)*(state['ms']-8000)*.001
            c.flight.local_position.z, c.flight.local_position.vz = -climb*(state['ms']-8000)*.001, -climb
            if c.first_ready_fcu_ms is None:          # as PX4Controller.ready() does
                c.first_ready_fcu_ms = state['ms']
            c.watch_heading()
    return c, advance, restarts


def test_steady_heading_settles_without_restart():
    c, advance, restarts = controller(.05)
    assert not c.heading_settled()                  # window not yet covered
    advance(pc.HEADING_WINDOW_S + 1)
    assert c.heading_settled() and not restarts


def test_drifting_heading_blocks_takeoff_and_restarts_estimator():
    c, advance, restarts = controller(5.)
    advance(pc.HEADING_WINDOW_S + 1)
    assert restarts and not c.heading_settled()
    assert c.first_ready_fcu_ms > 8000               # preflight settling started again after the restart
    advance(5)
    assert len(restarts) == 1                        # no restart before the new preflight window ends...
    advance(120)
    assert len(restarts) == pc.ESTIMATOR_RESTARTS    # ...then bounded


def test_heading_wraps_and_flight_is_not_judged():
    c, advance, _ = controller(0.)
    c.flight.attitude.yaw = math.pi - .001
    c.heading_samples = [(0, math.pi - .001, 0., 0.), (4000, -math.pi + .001, 0., 0.)]
    assert c.heading_drift() < .05                   # 0.002 rad across ±pi, not a full turn
    c.has_armed = True
    c.heading_samples = []
    assert c.heading_settled()


def test_a_running_away_height_blocks_takeoff_and_restarts_the_estimator():
    c, advance, restarts = controller(.05, climb=1.)          # steady heading, height estimate running away at rest
    advance(pc.HEADING_WINDOW_S - .5)
    assert c.height_unsettled() and not c.heading_settled()   # 3.5 m in 3.5 s at rest
    advance(1.5)
    assert restarts and not c.heading_settled()               # restarted; settling begins again
