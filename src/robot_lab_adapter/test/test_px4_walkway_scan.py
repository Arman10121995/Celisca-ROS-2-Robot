"""Looking both ways before crossing a walkway that moving tracks have swept (R5.11)."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
from robot_lab_adapter import px4_planning
from robot_lab_adapter.px4_planning import PX4Planning
from robot_lab_utils import aerial_tracking as at


class Clock:
    def __init__(self):
        self.now = 100.

    def monotonic(self):
        return self.now


def walker(y, vy, high=1.8):
    track = at.Track(np.array([5., y, 1.]), .3, 0., 0., high)
    track.x[3:] = (0., vy, 0.)
    track.frames = at.MIN_FRAMES
    return track


def planner(monkeypatch, tracks=()):
    clock = Clock()
    monkeypatch.setattr(px4_planning, 'time', clock)
    memory = at.SweptMemory()
    memory.record([walker(y, 1.) for y in np.arange(-4., 4.5, .5)], now=0.)     # a walkway along x = 5
    tracker = SimpleNamespace(moving=lambda: list(tracks))
    camera = SimpleNamespace(height=480, k=[433., 0., 320., 0., 433., 240., 0., 0., 1.])   # 29 deg half vertical FOV
    obstacles = SimpleNamespace(swept=memory, tracker=tracker, route_guard=lambda *_: '', intrinsic=(camera, 0.))
    flight = SimpleNamespace(attitude=SimpleNamespace(yaw=math.pi / 2))
    controller = SimpleNamespace(phase='flying', ready=lambda: True, manual=(0., 0., 0., 0.), route=[[10., 0., 1.2]],
                                 target_yaw=0., flight=flight, target=None, route_status='', ground_z=0.)
    controller.position = lambda: [2.5, 0., 1.2]
    controller.clear_path = lambda reason: (setattr(controller, 'route', []), setattr(controller, 'route_status', reason))
    controller.set_path = lambda route: setattr(controller, 'route', [list(p) for p in route[1:]])   # as PX4Controller
    p = SimpleNamespace(controller=controller, obstacles=obstacles, state=dict(state='executing'), sensor_loss_since=None,
                        scanning=None, crossing_until=-math.inf, last_scan=-math.inf, scans=0,
                        last_evasion=-math.inf, canceled=SimpleNamespace(set=lambda: None))
    for name in ('walkway', 'overfly', 'scan'):
        setattr(p, name, getattr(PX4Planning, name).__get__(p))

    def run(seconds):
        for _ in range(int(seconds * 20)):
            clock.now += .05
            flight.attitude.yaw = math.pi / 2 - controller.target_yaw    # the FCU turns at once (NED yaw)
            p.scan()
    return p, controller, run


def test_holds_looks_both_ways_and_crosses_a_clear_walkway(monkeypatch):
    p, controller, run = planner(monkeypatch)
    found = p.walkway(controller.route)
    assert found['column'][0] == pytest.approx(5., abs=.01) and 1. < found['length'] < 4.
    assert not found['inside'] and 0. < found['entry'] < 2.5
    run(.05)
    assert p.state['state'] == 'scanning' and controller.route == []
    assert controller.target_yaw == pytest.approx(math.radians(px4_planning.SCAN_YAW_DEG))
    headings = set()
    for _ in range(100):
        run(.1)
        headings.add(round(math.degrees(controller.target_yaw)))
        if p.state['state'] == 'executing':
            break
    assert headings >= {75, -75, 0}
    assert p.state['state'] == 'executing'
    # Crossed above the walkway's swept columns (top 1.8 + 0.3 pad), then on to the goal.
    climb, across, goal = controller.route[0], controller.route[1], controller.route[-1]
    assert climb[:2] == [2.5, 0.] and climb[2] == pytest.approx(2.1 + .7 + px4_planning.OVERFLY_ABOVE_M)
    assert climb[2] <= .8 * 8. * 240 / 433. - .3                # low enough for the camera to see the ground
    assert across[2] == pytest.approx(climb[2]) and across[0] > 6.
    assert goal == [10., 0., 1.2] and p.state['walkway']['crossing_z'] == pytest.approx(across[2])
    assert controller.target_yaw == 0. and p.state['walkway']['crossed_after_s'] > 2.
    run(1.)                                                     # crossing: no second scan of the same walkway
    assert p.state['state'] == 'executing'


def test_waits_while_a_walker_is_predicted_on_the_crossing(monkeypatch):
    # Taller than the overflight (a vehicle, not a walker), near y = 0 when the drone, after its climb, crosses.
    approaching = walker(-3., .25, high=5.)
    p, controller, run = planner(monkeypatch, [approaching])
    run(6.)
    assert p.state['state'] == 'scanning' and controller.route == []
    assert 'moving obstacle' in p.state['walkway']['waiting_for']


def test_walkway_found_inside_and_not_when_far(monkeypatch):
    p, controller, _ = planner(monkeypatch)
    controller.position = lambda: [5., 0., 1.2]
    assert p.walkway(controller.route)['inside']
    controller.position = lambda: [-5., 0., 1.2]
    assert p.walkway(controller.route) is None


def test_rechecks_timing_without_turning_soon_after_a_look(monkeypatch):
    p, controller, run = planner(monkeypatch)
    p.last_scan = 100.                                          # looked both ways a moment ago
    run(.1)                                                     # hold, then the timing check clears it
    assert p.state['state'] == 'executing' and p.state['walkway']['looked'] is False
    assert controller.target_yaw == 0. and controller.route[-1] == [10., 0., 1.2]
    run(1.)                                                     # entering: not re-held every tick
    assert p.state['state'] == 'executing'


def test_climbs_when_a_walker_is_predicted_in_the_walkway_it_is_crossing(monkeypatch):
    p, controller, run = planner(monkeypatch, [walker(-3., .8)])
    controller.position = lambda: [4.6, 0., 1.2]
    run(.05)
    assert p.state['state'] == 'evading' and p.state['evasion']['walkway']
    assert controller.route == [] and controller.target[2] >= 1.8 + px4_planning.EVADE_ABOVE_M - 1e-6
    assert controller.route_status.startswith('Observed route held:')


def test_no_overflight_when_the_camera_could_not_see_the_ground(monkeypatch):
    p, controller, _ = planner(monkeypatch)
    p.obstacles.intrinsic[0].k[4] = 1500.                      # narrow vertical view: ground out of range
    route, _ = p.overfly(controller.route, p.walkway(controller.route))
    assert route == controller.route
