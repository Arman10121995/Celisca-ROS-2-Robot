"""Distributed (per-member) ground swarm behaviours from each robot's own scan (R10.3).

Ported from ROS2swarm (https://github.com/ROS2swarm/ROS2swarm, Apache-2.0,
master 10499321): ``ScanCalculationFunctions.potential_field`` (dispersion),
``repulsion_field`` (attraction), the aggregation state machine and the
minimalist flocking zones, with the TurtleBot3 burger parameters. A behaviour
sees only its own member's LaserScan: no peer poses, no central planner.
Deviations, all for Robot Lab's scans and controllers:

- NaN and inf readings are dropped (ROS2swarm's lidar layer passes clean
  1° scans); readings below the pattern's min range are ignored, as there;
- robots are recognised by physical arc width (0.05–0.6 m) instead of a
  ray count, so any scan resolution and robot size works (flocking's
  cohesion zones too), and must stand free: the rays beside the object must
  be at least 0.3 m farther (or missing). Wall pieces split by grazing
  angles otherwise pass as robots and stop aggregation at walls; flocking's obstacle ray thresholds scale with the
  angular resolution (they were defined for 1° rays);
- aggregation stay counters run on seconds (ROS2swarm counts 5 Hz scans),
  and leaving a group lasts at most twice the base stay time: ROS2swarm
  leaves until no robot is in range, which never ends beside corners that
  still read as robots (measured 2026-11-14);
- speeds are clipped to the member's limits; the fleet controller's
  separation and scan guards still apply after a behaviour.

As in ROS2swarm, a hardware protection layer runs after every behaviour:
when more than a few rays are inside the member radius + 0.18 m it replaces
the command with a potential field away from the obstacle (ROS2swarm's
burger layer uses 0.3 m, front attraction 2 and 5 rays).
"""
from dataclasses import dataclass, field
import math
import random

import numpy as np

BEHAVIOURS = ('local_dispersion', 'local_attraction', 'local_aggregation', 'local_flocking')

# ROS2swarm config/burger/movement_pattern/basic/*.yaml
BURGER = dict(
    dispersion=dict(max_range=3.5, min_range=.12, front_attraction=1., threshold=1, linear_if_alone=.2, angular_if_alone=0.),
    attraction=dict(max_range=3., min_range=.15, front_attraction=-1., threshold=1, linear_if_alone=0., angular_if_alone=1.),
    aggregation=dict(max_range=3.5, min_range=.08, front_attraction=.5, time_scale_s=24 / 5., base_stay_s=16 / 5.,
                     group_distance=.85, object_threshold=.2, min_width_m=.05, max_width_m=.6, stay_in_growing_groups=True),
    protection=dict(margin=.18, min_range=.0, front_attraction=2., threshold=5),
    flocking=dict(linear=.2, left=.2, right=-.2, zone1=.5, zone2=.5, zone3=1.5, zone4=2.5, zone2_robot_rays=18,
                  robot_rays=4, borders=(math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4),
                  max_range=3.5, min_range=.12))


def scan_rays(ranges, angle_min, angle_increment, range_min=0., range_max=math.inf):
    """Valid (range, angle) pairs in the robot frame; invalid rays are dropped."""
    rays = []
    for index, value in enumerate(ranges):
        if value is None or not math.isfinite(value) or value < max(range_min, 1e-3) or value > range_max:
            continue
        rays.append((float(value), angle_min + index * angle_increment))
    return rays


def _twist(vector, max_linear, max_angular):
    """ROS2swarm create_normed_twist_message: unit vector -> (linear, angular)."""
    norm = math.hypot(vector[0], vector[1])
    if norm < 1e-9 or not math.isfinite(norm):
        return 0., 0.
    x, y = vector[0] / norm, vector[1] / norm
    return max_linear * x, max_angular * math.asin(max(-1., min(1., y)))


def _field(rays, params, max_linear, max_angular, near_weighted):
    """potential_field (near_weighted) or repulsion_field (far-weighted)."""
    max_range = params['max_range']
    inside = [(r, a) for r, a in rays if params['min_range'] <= r < max_range]
    alone = len(inside) <= params['threshold']
    if alone:
        return params['linear_if_alone'], params['angular_if_alone'], True
    vx = vy = 0.
    for r, a in inside:
        weight = 1 - r / max_range if near_weighted else r / max_range
        if weight < 1.:
            vx += weight * math.cos(a)
            vy += weight * math.sin(a)
    if near_weighted:
        vx, vy = -vx + params['front_attraction'], -vy          # flip, then add front attraction
    else:
        # repulsion_field: add the negated attraction to the flipped sum, then flip again.
        vx, vy = vx + params['front_attraction'], vy
    linear, angular = _twist((vx, vy), max_linear, max_angular)
    return linear, angular, False


def robots_in_scan(rays, params):
    """ROS2swarm identify_robots with a physical width window; returns [(range, angle)] centres."""
    objects, current = [], []
    for r, a in rays:
        if not params['min_range'] < r < params['max_range']:
            if current:
                objects.append(current)
                current = []
            continue
        if current and abs(r - current[-1][0]) >= params['object_threshold']:
            objects.append(current)
            current = []
        current.append((r, a))
    if current:
        objects.append(current)
    if len(objects) >= 2 and rays and objects[0][0] == rays[0] and objects[-1][-1] == rays[-1] \
            and abs(objects[0][0][0] - objects[-1][-1][0]) < params['object_threshold']:
        objects[0] = objects.pop() + objects[0]                  # wrap-around object
    centres = []
    index = {ray: i for i, ray in enumerate(rays)}
    def background(ray, step):
        # The neighbour beyond an object's edge must be farther (free-standing
        # body with background behind it); grazing wall fragments fail this.
        i = index.get(ray)
        if i is None:
            return False
        r, a = rays[(i + step) % len(rays)]
        gap = abs(math.atan2(math.sin(a - ray[1]), math.cos(a - ray[1])))
        return gap > 3 * (abs(rays[1][1] - rays[0][1]) if len(rays) > 1 else 1.) or r > ray[0] + params.get('background_m', .3)
    for obj in objects:
        if not (background(obj[0], -1) and background(obj[-1], 1)):
            continue
        span = abs(obj[-1][1] - obj[0][1]) if len(obj) > 1 else 0.
        span = min(span, 2 * math.pi - span)
        width = 2 * np.mean([r for r, _ in obj]) * math.sin(span / 2) if len(obj) > 1 else 0.
        if params['min_width_m'] <= width <= params['max_width_m']:
            x = sum(math.cos(a) for _, a in obj)
            y = sum(math.sin(a) for _, a in obj)
            centres.append((float(np.mean([r for r, _ in obj])), math.atan2(y, x)))
    return centres


def _towards(centres, params, max_linear, max_angular, away):
    vx = vy = 0.
    for r, a in centres:
        weight = 1 - r / params['max_range']
        vx += weight * math.cos(a)
        vy += weight * math.sin(a)
    if away:
        vx, vy = -vx, -vy
    return _twist((vx + params['front_attraction'], vy), max_linear, max_angular)


@dataclass
class LocalBehaviour:
    """One member's behaviour; ``step`` sees only that member's scan."""
    kind: str
    max_linear: float
    max_angular: float
    params: dict = None
    radius: float = .12
    state: str = 'init'
    stay_until: float = 0.
    leave_until: float = 0.
    group_size: int = 0
    last: tuple = (0., 0.)
    held: int = 0
    rng: random.Random = field(default_factory=lambda: random.Random(0))

    def __post_init__(self):
        if self.kind not in BEHAVIOURS:
            raise ValueError('Unknown distributed swarm behaviour: ' + str(self.kind))
        if self.params is None:
            self.params = dict(BURGER[self.kind.split('_', 1)[1]])

    def step(self, ranges, angle_min, angle_increment, now, range_min=0., range_max=math.inf):
        rays = scan_rays(ranges, angle_min, angle_increment, range_min, range_max)
        if self.kind == 'local_dispersion':
            linear, angular, alone = _field(rays, self.params, self.max_linear, self.max_angular, True)
            info = dict(state='alone' if alone else 'repelling')
        elif self.kind == 'local_attraction':
            linear, angular, alone = _field(rays, self.params, self.max_linear, self.max_angular, False)
            info = dict(state='alone' if alone else 'attracting')
        elif self.kind == 'local_aggregation':
            linear, angular = self._aggregation(rays, now)
            info = dict(state=self.state, group_size=self.group_size)
        else:
            linear, angular = self._flocking(rays, angle_increment)
            info = dict(state=self.state)
        protected = self._protect(rays, angle_increment)
        if protected:
            linear, angular = protected
            info['protected'] = True
        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        self.last = (linear, angular)
        return linear, angular, info

    def _protect(self, rays, angle_increment):
        """ROS2swarm hardware_protection_layer: potential field away from close obstacles."""
        p = BURGER['protection']
        scale = (math.pi / 180.) / max(abs(angle_increment), 1e-6)
        params = dict(max_range=self.radius + p['margin'], min_range=p['min_range'], front_attraction=p['front_attraction'],
                      threshold=max(1, round(p['threshold'] * scale)), linear_if_alone=0., angular_if_alone=0.)
        linear, angular, free = _field(rays, params, self.max_linear, self.max_angular, True)
        return None if free else (linear, angular)

    def _aggregation(self, rays, now):
        p = self.params
        robots = robots_in_scan(rays, p)
        if self.state in ('init', 'explore'):
            self.state = 'join' if robots else 'explore'
            if self.state == 'explore':
                return self.max_linear, 0.
        if self.state == 'join':
            if not robots:
                self.state = 'explore'
                return self.max_linear, 0.
            if any(r < p['group_distance'] for r, _ in robots):
                self.state, self.group_size = 'stay', len(robots)
                self.stay_until = now + 2 ** len(robots) * p['time_scale_s'] + p['base_stay_s']
                return 0., 0.
            return _towards(robots, p, self.max_linear, self.max_angular, away=False)
        if self.state == 'stay':
            if now < self.stay_until:
                return 0., 0.
            if p['stay_in_growing_groups'] and len(robots) > self.group_size:
                self.group_size = len(robots)
                self.stay_until = now + 2 ** len(robots) * p['time_scale_s'] + p['base_stay_s']
                return 0., 0.
            self.state, self.leave_until = 'leave', now + 2 * p['base_stay_s']
        if self.state == 'leave':
            if robots and now < self.leave_until:
                return _towards(robots, p, self.max_linear, self.max_angular, away=True)
            self.state = 'explore'
            return self.max_linear, 0.
        return 0., 0.

    def _flocking(self, rays, angle_increment):
        p = self.params
        scale = (math.pi / 180.) / max(abs(angle_increment), 1e-6)
        robot_rays, zone2_rays = max(1, round(p['robot_rays'] * scale)), max(2, round(p['zone2_robot_rays'] * scale))
        b = p['borders']
        zones = dict(front=[], left=[], behind=[], right=[])
        for r, a in sorted(((r, a % (2 * math.pi)) for r, a in rays), key=lambda item: item[1]):
            r = r if p['min_range'] <= r <= p['max_range'] else p['max_range']
            if b[2] < a <= b[3]:
                zones['right'].append(r)
            elif a > b[3] or a <= b[0]:
                zones['front'].append(r)
            elif b[0] < a <= b[1]:
                zones['left'].append(r)
            else:
                zones['behind'].append(r)
        def count(limit, values, low=0.):
            return sum(1 for v in values if low < v < limit)
        peers = robots_in_scan(rays, dict(min_range=p['min_range'], max_range=p['max_range'], object_threshold=.2,
                                          min_width_m=.05, max_width_m=.6))
        def zone_of(a):
            a %= 2 * math.pi
            return ('right' if b[2] < a <= b[3] else 'front' if a > b[3] or a <= b[0]
                    else 'left' if b[0] < a <= b[1] else 'behind')
        def robot(name, low, limit):
            return sum(1 for r, a in peers if zone_of(a) == name and low < r < limit)
        turn_left, turn_right, move = (0., p['left']), (0., p['right']), (p['linear'], 0.)
        close = [1 < count(p['zone2'], zones[k]) < zone2_rays for k in ('left', 'front', 'right')]
        if count(p['zone1'], zones['front']) > robot_rays:
            self.state = 'avoid'
            front = zones['front']
            first_half = front[:len(front) // 2]
            nearest = min(front) if front else p['max_range']
            return turn_right if self.last[1] == 0. and nearest in first_half else (self.last if self.last[1] else turn_left)
        if any(close):
            self.state = 'separation'
            if close[0]:
                return turn_right
            if close[2]:
                return turn_left
            return self.last if self.last[1] else (turn_left if self.rng.random() < .5 else turn_right)
        left, right = robot('left', p['zone3'], p['max_range']), robot('right', p['zone3'], p['max_range'])
        if left or right:
            self.state = 'cohesion'
            return move if left == right else (turn_left if left > right else turn_right)
        if robot('behind', p['zone4'], p['max_range']):
            self.state = 'cohesion_behind'
            behind = zones['behind']
            nearest = min(behind)
            return turn_left if self.last[1] == 0. and nearest in behind[:len(behind) // 2] else turn_right
        self.state = 'forward'
        return move
