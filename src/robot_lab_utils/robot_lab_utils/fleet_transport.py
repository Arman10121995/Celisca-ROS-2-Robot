"""Automatic physical lift transport; actions are intents, never arrival proof."""
import math
import uuid


TERMINAL = {'succeeded', 'failed', 'canceled'}


def lift_door_admission(position, rate, floor, height, admitted, precision, tracking):
    """Require precise initial leveling; retain original load-tracking bounds."""
    if not all(math.isfinite(v) for v in (precision,tracking)) or not 0<precision<=tracking:
        raise ValueError('Door admission tolerance must be positive and within floor tracking tolerance')
    if position is None or not math.isfinite(position) or not math.isfinite(rate) or abs(rate)>=.01:
        return None,False
    error=abs(position-height)
    if error>=tracking:return None,False
    if admitted==floor or error<precision:return floor,True
    return None,False


def door_position_mode(position, closed, opened, tolerance=.04):
    # A panel beyond the required opening still clears the passage. It must
    # not become "moving" solely because a bounded servo overshot the target.
    direction=1. if opened>closed else -1.
    if direction*(position-opened)>=-tolerance:return 2
    return 0 if abs(position-closed)<tolerance else 1


class LiftTransport:
    def __init__(self, robot, operator, origin, destination, config, now, parking=None):
        if origin == destination or origin not in config['landings'] or destination not in config['landings']:
            raise ValueError('Choose a different configured landing floor')
        self.robot, self.operator = robot, operator
        self.origin, self.destination = origin, destination
        self.config = config
        self.budgets={key:float(config.get(key,default)) for key,default in
            [('total_timeout_s',300.),('phase_timeout_s',100.),('route_timeout_s',90.)]}
        for key,ceiling in [('total_timeout_s',900.),('phase_timeout_s',300.),('route_timeout_s',180.)]:
            if not math.isfinite(self.budgets[key]) or not 0<self.budgets[key]<=ceiling:raise ValueError('Invalid bounded transport '+key)
        self.parking = parking
        self.prefix = 'lift-' + uuid.uuid4().hex
        self.phase = 'starting'
        self.started = now
        self.phase_at = now
        self.route_id = None
        self.acknowledged = False
        self.settled_at = None
        self.reason = 'Waiting for physical approach'

    @classmethod
    def exit_only(cls, robot, operator, floor, config, now, parking=None):
        """Drive a recovered passenger out of a level, open car on its floor."""
        if floor not in config['landings']:
            raise ValueError('Choose a configured landing floor')
        other = next(name for name in config['landings'] if name != floor)
        task = cls(robot, operator, other, floor, config, now, parking)
        task.origin = floor
        task._phase('exit_ready', now, 'Waiting for the passenger floor map before exit')
        return task

    @property
    def active(self):
        return self.phase not in TERMINAL

    def state(self):
        return dict(robot=self.robot, operator_id=self.operator, origin=self.origin,
                    destination=self.destination, phase=self.phase, reason=self.reason,
                    request_prefix=self.prefix, route_id=self.route_id, parking_goal=self.parking)

    def cancel(self, reason, failed=False):
        if not self.active:
            return []
        self.phase = 'failed' if failed else 'canceled'
        self.reason = reason
        return [dict(kind='stop'), dict(kind='release', hold=True)]

    def _phase(self, phase, now, reason):
        self.phase, self.phase_at, self.reason = phase, now, reason
        self.settled_at = None

    def _route(self, phase, goal, now):
        self._phase(phase, now, 'Waiting for actual Nav2 and independently settled ' + phase)
        self.route_id = self.prefix + '-' + phase
        self.acknowledged = False
        return dict(kind='route', request_id=self.route_id, goal=dict(goal, timeout=self.budgets['route_timeout_s']))

    def in_cabin(self, xyz):
        x0, x1, y0, y1 = self.config['cabin']
        return x0 <= xyz[0] <= x1 and y0 <= xyz[1] <= y1

    def tick(self, context, now):
        if not self.active:
            return []
        if not context['fresh'] or not context['operator_fresh']:
            return self.cancel('Feedback or operator ownership lost; explicit new request required', True)
        if now - self.started > self.budgets['total_timeout_s'] or now - self.phase_at > self.budgets['phase_timeout_s']:
            return self.cancel('Bounded lift transport deadline exceeded', True)
        member, body = context['member'], context['body']
        if member.get('mode') in ('manual', 'swarm', 'resetting', 'paused'):
            return self.cancel('Robot control was replaced or paused', True)
        if self.phase == 'exit_ready':
            if not self.in_cabin(body['xyz']):
                return self.cancel('Recovered passenger is no longer in the car', True)
            if context['floor'] == self.destination and context['map_ready'] and context['door_open']:
                return [self._route('exiting', self.config['landings'][self.destination], now)]
            return []
        if self.phase == 'starting':
            if self.in_cabin(body['xyz']):
                self._phase('opening', now, 'Waiting for the origin car door')
                return [dict(kind='lift', floor=self.origin)]
            return [self._route('approaching', self.config['landings'][self.origin], now)]
        if self.phase in ('approaching', 'boarding', 'exiting', 'parking'):
            source = 'route:' + self.route_id
            outcomes = [item for item in context['history'] if item.get('source') == source]
            queued = [item for item in member.get('queued_tasks', []) if item.get('source') == source]
            self.acknowledged |= bool(outcomes or queued or member.get('active_source') == source)
            if any(item['state'] != 'succeeded' for item in outcomes):
                return self.cancel('Owned transport Nav2 task failed/canceled', True)
            if not outcomes:
                if not self.acknowledged and now - self.phase_at > 5:
                    return self.cancel('No actual acknowledgement for the transport route', True)
                return []
            # Nav2 success alone is insufficient; independently observe the endpoint.
            goal = (self.parking if self.phase == 'parking' else self.config['boarding'] if self.phase == 'boarding'
                    else self.config['landings'][self.destination if self.phase == 'exiting' else self.origin])
            good = (body['speed'] is not None and body['speed'] < .03
                    and body['angular'] is not None and abs(body['angular']) < .05
                    and math.hypot(body['xyz'][0] - goal['x'], body['xyz'][1] - goal['y']) < .15
                    and abs(math.atan2(math.sin(body['yaw'] - goal['yaw']), math.cos(body['yaw'] - goal['yaw']))) < .15)
            if self.phase == 'boarding':
                good &= self.in_cabin(body['xyz'])
            good &= context['floor'] == (self.destination if self.phase in ('exiting','parking') else self.origin)
            self.settled_at = context['clock'] if good and self.settled_at is None else self.settled_at if good else None
            if self.settled_at is None or context['clock'] - self.settled_at < 1.:
                return []
            if self.phase == 'approaching':
                self._phase('opening', now, 'Waiting for observed origin door open')
                return [dict(kind='lift', floor=self.origin)]
            if self.phase == 'boarding':
                self._phase('riding', now, 'Waiting for physical ride, observed floor/map and car-door opening')
                return [dict(kind='lift', floor=self.destination)]
            if self.in_cabin(body['xyz']):
                return self.cancel('Exit goal remained in the cabin', True)
            if self.phase == 'exiting' and self.parking:
                return [self._route('parking',self.parking,now)]
            self._phase('succeeded', now, 'Actual approach, boarding, floor/map change and settled exit observed')
            return [dict(kind='release', hold=False)]
        if self.phase == 'opening' and context['car_floor'] == self.origin and context['door_open']:
            return [self._route('boarding', self.config['boarding'], now)]
        if self.phase == 'riding' and (context['car_floor'] == self.destination
                and context['floor'] == self.destination and context['map_ready'] and context['door_open']):
            return [self._route('exiting', self.config['landings'][self.destination], now)]
        return []
