"""Observed footprint interlocks for explicit lift recovery, never automatic rearm."""
import math


def recovery_passengers(config, members, bodies, positions, rates, now):
    lift = config['lift']
    if not math.isfinite(rates.get(lift['joint'], math.inf)) or abs(rates[lift['joint']]) >= .01:
        raise ValueError('Wait for the held car to settle before recovery')
    if set(bodies) != {member['id'] for member in members} or any(now - body['at'] >= .6 for body in bodies.values()):
        raise ValueError('Recovery requires fresh independent observations of every member')
    height = positions.get(lift['joint'])
    if height is None or not math.isfinite(height):
        raise ValueError('Recovery requires a measured car height')
    x0, x1, y0, y1 = config['transport']['cabin']
    passengers = []
    for member in members:
        name = member['id']
        body = bodies[name]
        x, y, z = body['xyz']
        radius = float(member.get('radius', .35))
        if not math.isfinite(radius) or radius <= 0:
            raise ValueError('Recovery requires a positive footprint radius')
        if not all(math.isfinite(value) for value in (x, y, z)):
            raise ValueError('Recovery requires finite body observations')
        overlaps = x + radius > x0 and x - radius < x1 and y + radius > y0 and y - radius < y1
        if not overlaps:
            continue
        # An upper-floor peer outside the car is not a passenger; the landing
        # gate still has its own fresh-body closing sweep interlock.
        if abs(z - height) > .2:
            continue
        if not (x - radius >= x0 and x + radius <= x1 and y - radius >= y0 and y + radius <= y1):
            raise ValueError('A member straddles the car boundary; clear it before recovery')
        if (body.get('speed') is None or body['speed'] >= .03 or
                body.get('angular') is None or abs(body['angular']) >= .05):
            raise ValueError('Every passenger must be independently observed stationary')
        passengers.append(name)
    if len(passengers) > 1:
        raise ValueError('This single-member car cannot recover multiple passengers')
    return sorted(passengers)
