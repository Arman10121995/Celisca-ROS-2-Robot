"""Bounded differential-drive group kernels and conservative sensor guards.

These are explicit educational velocity controllers, not Open-RMF scheduling
or a claim of ROS2swarm's full algorithms. No function changes a body pose.
"""
from dataclasses import dataclass
import math


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Neighbor:
    name: str
    x: float
    y: float
    yaw: float
    speed: float
    radius: float


def point_velocity(robot, target, max_linear, max_angular, tolerance=.08):
    dx, dy = target[0] - robot.x, target[1] - robot.y
    distance = math.hypot(dx, dy)
    if distance < tolerance:
        return 0., 0.
    heading = angle(math.atan2(dy, dx) - robot.yaw)
    # Rotate to a forward feasible heading; do not ask a diff base to strafe.
    linear = min(max_linear, .6 * distance) * max(0., math.cos(heading))
    if abs(heading) > .9:
        linear = 0.
    return linear, max(-max_angular, min(max_angular, 1.4 * heading))


SWARM_MODES = ('formation', 'rendezvous', 'flocking', 'clearance', 'dispersion')

def swarm_velocities(mode, neighbors, max_linear, max_angular, spacing=1., anchor=None, heading=0., clearance_radius=None):
    if mode not in SWARM_MODES:
        raise ValueError(f'Unknown ground swarm controller. Mode must be one of: {SWARM_MODES}')
    if not .6 <= spacing <= 3 or not all(math.isfinite(v) for v in (spacing, heading)):
        raise ValueError('Swarm spacing must be finite and between 0.6 and 3 m')
    robots = sorted(neighbors, key=lambda r: r.name)
    if not robots:
        return {}
    centre = anchor or (sum(r.x for r in robots) / len(robots), sum(r.y for r in robots) / len(robots))
    if not all(math.isfinite(v) for v in centre):
        raise ValueError('Swarm anchor must be finite')
    # Build a radius map for clearance-aware coordination
    robot_radii = {r.name: r.radius for r in robots}
    if clearance_radius is None:
        clearance_radius = max(robot_radii.values()) * 1.5 if robot_radii else 0.1
    commands = {}
    for index, robot in enumerate(robots):
        if mode == 'formation':
            offset = (index - (len(robots) - 1) / 2) * spacing
            target = (centre[0] - math.sin(heading) * offset, centre[1] + math.cos(heading) * offset)
        elif mode == 'rendezvous':
            # Rendezvous at a ring, preserving body separation at the centre.
            phase = 2 * math.pi * index / len(robots)
            radius = 0. if len(robots) == 1 else max(spacing / 2, spacing / (2 * math.sin(math.pi / len(robots))))
            target = (centre[0] + radius * math.cos(phase), centre[1] + radius * math.sin(phase))
        elif mode == 'flocking':
            # Cohesion, circular heading alignment and separation with an
            # explicit operator heading bias. No hidden leader pose writes.
            hx = sum(math.cos(r.yaw) for r in robots) / len(robots)
            hy = sum(math.sin(r.yaw) for r in robots) / len(robots)
            rx = .15 * (centre[0] - robot.x) + .35 * hx + .5 * math.cos(heading)
            ry = .15 * (centre[1] - robot.y) + .35 * hy + .5 * math.sin(heading)
            length = math.hypot(rx, ry)
            scale = 1. / max(1., length)
            target = (robot.x + rx * scale, robot.y + ry * scale)
        elif mode in ('clearance', 'dispersion'):
            # Clearance-aware cooperative sweep allocation.
            # Each robot sweeps while maintaining minimum separation from neighbors.
            rx = ry = 0.
            for other in robots:
                if robot.name == other.name:
                    continue
                dx, dy = robot.x - other.x, robot.y - other.y
                distance = math.hypot(dx, dy)
                min_distance = robot.radius + other.radius + clearance_radius
                if distance < min_distance:
                    # Push apart to maintain clearance
                    weight = (min_distance - distance) / max(distance, 0.05)
                    rx += weight * dx / distance
                    ry += weight * dy / distance
            # Add cohesion toward centre with heading bias
            rx += .2 * (centre[0] - robot.x)
            ry += .2 * (centre[1] - robot.y)
            # Add heading alignment bias
            hx = sum(math.cos(r.yaw) for r in robots) / len(robots)
            hy = sum(math.sin(r.yaw) for r in robots) / len(robots)
            rx += .3 * hx + .5 * math.cos(heading)
            ry += .3 * hy + .5 * math.sin(heading)
            length = math.hypot(rx, ry)
            scale = 1. / max(1., length)
            target = (robot.x + rx * scale, robot.y + ry * scale)
        commands[robot.name] = point_velocity(robot, target, max_linear, max_angular)
    return commands


def scan_guard(linear, angular, ranges, angle_min, angle_increment, radius, acceleration=.7):
    if not all(math.isfinite(v) for v in (linear, angular, angle_min, angle_increment, radius, acceleration)):
        return 0., 0., 'invalid command/scan'
    if not ranges or angle_increment <= 0 or acceleration <= 0:
        return 0., 0., 'invalid scan'
    nearest = math.inf
    ahead = math.inf
    for index, value in enumerate(ranges):
        if math.isnan(value) or value <= 0:
            # Invalid rays cannot provide free-space evidence.
            value = 0.
        if not math.isfinite(value) and value != math.inf:
            value = 0.
        nearest = min(nearest, value)
        bearing = angle(angle_min + index * angle_increment - (math.pi if linear < 0 else 0.))
        if abs(bearing) < .65:
            ahead = min(ahead, value)
    if nearest < radius + .03:
        return 0., 0., 'scan footprint proximity'
    stopping = radius + .12 + abs(linear) * .25 + linear * linear / (2 * acceleration)
    if linear and ahead < stopping:
        return 0., 0., 'scan stopping corridor'
    return linear, angular, ''


def separation_guard(robot, others, linear, angular, horizon=.7):
    """Check swept centre distances against every measured fresh member."""
    for other in others:
        if robot.name == other.name:
            continue
        threshold = robot.radius + other.radius + .12
        if math.hypot(robot.x - other.x, robot.y - other.y) < threshold:
            return 0., 0., 'member footprint proximity: ' + other.name
        for tick in range(1, 8):
            t = horizon * tick / 7
            yaw = robot.yaw + angular * t / 2
            x, y = robot.x + linear * t * math.cos(yaw), robot.y + linear * t * math.sin(yaw)
            ox = other.x + other.speed * t * math.cos(other.yaw)
            oy = other.y + other.speed * t * math.sin(other.yaw)
            if math.hypot(x - ox, y - oy) < threshold:
                return 0., 0., 'predicted member separation: ' + other.name
    return linear, angular, ''
