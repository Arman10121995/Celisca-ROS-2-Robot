"""Car final-heading shuffle: arc plans and closed-loop convergence on a kinematic car."""
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_lab_utils.car_alignment import HeadingShuffle, integrate, shuffle_plan, wrap  # noqa: E402


@pytest.mark.parametrize('error_deg', [-23.0, -12.8, 3.5, 11.6, 14.3, 40.0])
def test_plan_rotates_by_the_error_and_stays_in_place(error_deg):
    error = math.radians(error_deg)
    plan = shuffle_plan(error, 0.575, max_step=0.12)
    assert plan and len(plan) % 2 == 0
    assert all(arc.length <= 0.12 + 1e-9 for arc in plan)
    assert [arc.direction for arc in plan[:2]] == [-1, +1], 'reverse first, out of a nose-in bay'
    pose = (1.0, 2.0, 0.3)
    for arc in plan:
        pose = integrate(pose, arc, 0.575)
    assert wrap(pose[2] - 0.3) == pytest.approx(error, abs=1e-3)
    # Displacements cancel to first order: well under one arc length.
    assert math.hypot(pose[0] - 1.0, pose[1] - 2.0) < 0.4 * plan[0].length * len(plan) / 2 + 0.01


def test_no_plan_without_an_error_and_bad_radius_is_rejected():
    assert shuffle_plan(0.0, 0.5) == []
    with pytest.raises(ValueError):
        shuffle_plan(0.1, 0.0)


class KinematicCar:
    """Unicycle with a minimum turning radius, commanded by (vx, wz)."""

    def __init__(self, yaw, min_radius=0.5):
        self.x, self.y, self.yaw = 0.0, 0.0, yaw
        self.vx = self.wz = 0.0
        self.min_radius = min_radius
        self.commands = []

    def publish(self, vx, wz):
        if abs(vx) > 1e-9 and abs(wz / vx) > 1.0 / self.min_radius + 1e-9:
            raise AssertionError('command tighter than the turning radius')
        self.vx, self.wz = vx, wz
        self.commands.append((vx, wz))

    def spin(self, seconds, dt=0.01):
        for _ in range(int(round(seconds / dt))):
            self.x += self.vx * math.cos(self.yaw) * dt
            self.y += self.vx * math.sin(self.yaw) * dt
            self.yaw = wrap(self.yaw + self.wz * dt)

    def pose(self):
        return self.x, self.y, self.yaw


@pytest.mark.parametrize('start_deg', [-14.3, 11.6, -23.0])
def test_closed_loop_shuffle_reaches_the_heading_within_tolerance(start_deg):
    goal = math.pi / 2
    car = KinematicCar(goal + math.radians(start_deg))
    shuffle = HeadingShuffle(car.pose, car.publish, car.spin, turning_radius=0.5)
    report = shuffle.run(goal)
    assert report['attempted'] and report['pairs'] >= 1
    assert abs(report['heading_error_after_deg']) <= 3.0
    assert report['displacement_m'] < 0.1
    assert car.commands[-1] == (0.0, 0.0), 'the car is left stopped'
    assert any(vx < 0 for vx, _ in car.commands) and any(vx > 0 for vx, _ in car.commands)


def test_already_aligned_car_does_not_move():
    car = KinematicCar(math.radians(1.0))
    report = HeadingShuffle(car.pose, car.publish, car.spin, turning_radius=0.5).run(0.0)
    assert report['pairs'] == 0 and not car.commands
