"""Final-heading alignment for car-like robots: a forward/reverse shuffle.

A car cannot turn on the spot, and Nav2's car stack (Hybrid-A* plus
Regulated Pure Pursuit) ends a goal at its position: pure pursuit aims at
the goal point once it is inside the lookahead, so a small lateral offset
becomes a heading error on arrival (8-23 degrees in the car course, rounds
51-57).  A shuffle corrects it in place, as a driver straightens up in a
bay: reverse with the wheels turned one way, then drive forward with them
turned the other way.  Both arcs rotate the car the same way, and their
displacements cancel to first order, so the car stays where Nav2 left it.

``shuffle_plan`` splits a heading error into such arc pairs, each no
longer than ``max_step``.  ``HeadingShuffle`` runs them closed-loop on a
measured pose (x, y, yaw) and a Twist publisher: (vx, wz = vx / radius).
It is used after the parking goal of scripts/sim_car_course.py.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class Arc:
    direction: int     # +1 forward, -1 reverse
    turn: int          # +1 curvature left, -1 right (in the body frame)
    length: float      # metres along the arc


def shuffle_plan(heading_error: float, turning_radius: float, max_step: float = 0.12) -> List[Arc]:
    """Arc pairs (reverse, then forward) that rotate the car by ``heading_error``.

    Reversing with the wheels turned right and driving forward with them
    turned left both rotate the car counter-clockwise (for a positive error),
    by length / radius each.
    """
    if turning_radius <= 0 or max_step <= 0:
        raise ValueError('turning radius and step must be positive')
    total = abs(heading_error) * turning_radius     # arc length of all segments
    if total < 1e-6:
        return []
    pairs = max(1, math.ceil(total / (2.0 * max_step)))
    length = total / (2.0 * pairs)
    sign = 1 if heading_error > 0 else -1
    plan = []
    for _ in range(pairs):
        plan.append(Arc(direction=-1, turn=-sign, length=length))
        plan.append(Arc(direction=+1, turn=+sign, length=length))
    return plan


def integrate(pose: Tuple[float, float, float], arc: Arc, radius: float, steps: int = 200):
    """Pose after driving one arc (unicycle model of the base frame)."""
    x, y, yaw = pose
    ds = arc.direction * arc.length / steps
    curvature = arc.turn / radius
    for _ in range(steps):
        x += ds * math.cos(yaw)
        y += ds * math.sin(yaw)
        yaw += ds * curvature          # yaw rate = v * curvature
    return x, y, wrap(yaw)


class HeadingShuffle:
    """Closed-loop shuffle on a measured pose.

    ``pose()`` returns (x, y, yaw) or None; ``publish(vx, wz)`` commands the
    car; ``spin(seconds)`` lets time pass while callbacks run.  Each arc ends
    when its length has been driven, when the heading error has been removed
    or after its time limit; the plan is recomputed after every pair.
    """

    def __init__(self, pose: Callable[[], Optional[Tuple[float, float, float]]],
                 publish: Callable[[float, float], None], spin: Callable[[float], None],
                 turning_radius: float, speed: float = 0.15, tolerance: float = math.radians(3.0),
                 max_step: float = 0.12, max_pairs: int = 8, radius_margin: float = 1.15):
        self.pose, self.publish, self.spin = pose, publish, spin
        # A little above the minimum radius keeps the steering off its stops.
        self.radius = float(turning_radius) * radius_margin
        self.speed, self.tolerance = float(speed), float(tolerance)
        self.max_step, self.max_pairs = float(max_step), int(max_pairs)
        self.log = []

    def _drive(self, arc: Arc, goal_yaw: float, period: float = 0.05):
        start = self.pose()
        if start is None:
            return False
        sign = 1 if wrap(goal_yaw - start[2]) > 0 else -1
        limit = 3.0 * arc.length / self.speed + 2.0
        elapsed = 0.0
        vx = arc.direction * self.speed
        wz = vx * arc.turn / self.radius       # path curvature = wz / vx
        while elapsed < limit:
            now = self.pose()
            if now is None:
                break
            driven = math.hypot(now[0] - start[0], now[1] - start[1])
            remaining = wrap(goal_yaw - now[2])
            if driven >= arc.length or remaining * sign <= self.tolerance * 0.5:
                break
            self.publish(vx, wz)
            self.spin(period)
            elapsed += period
        self.publish(0.0, 0.0)
        self.spin(0.5)
        return True

    def run(self, goal_yaw: float) -> dict:
        """Shuffle until the heading is within tolerance; returns a report."""
        before = self.pose()
        if before is None:
            return dict(attempted=False, note='no pose')
        pairs = 0
        while pairs < self.max_pairs:
            now = self.pose()
            error = wrap(goal_yaw - now[2])
            if abs(error) <= self.tolerance:
                break
            plan = shuffle_plan(error, self.radius, self.max_step)[:2]
            for arc in plan:
                self._drive(arc, goal_yaw)
            pairs += 1
            after = self.pose()
            self.log.append(dict(pair=pairs, heading_error_deg=round(math.degrees(wrap(goal_yaw - after[2])), 2),
                                 x=round(after[0], 3), y=round(after[1], 3)))
        after = self.pose()
        return dict(attempted=True, pairs=pairs,
                    heading_error_before_deg=round(math.degrees(wrap(goal_yaw - before[2])), 2),
                    heading_error_after_deg=round(math.degrees(wrap(goal_yaw - after[2])), 2),
                    displacement_m=round(math.hypot(after[0] - before[0], after[1] - before[1]), 3),
                    log=self.log)
