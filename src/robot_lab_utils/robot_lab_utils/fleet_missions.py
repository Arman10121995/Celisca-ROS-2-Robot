"""Bounded fleet missions and clearance-aware cooperative sweep targets.

Targets are plans, never a claim that an area has been covered. Completion
requires a terminal Nav2 result followed by measured physical settling.
"""
from collections import deque
import math
import time
import uuid

import numpy as np
from scipy.ndimage import label

from .fleet_session import finite


MAX_QUEUE = 64


def mission(goal, source='operator', now=None):
    return dict(task=uuid.uuid4().hex, goal=dict(goal), source=source,
                admitted=time.monotonic() if now is None else now)


def enqueue(queue, item):
    if len(queue) >= MAX_QUEUE:
        raise ValueError('Member queue is full (64 goals); cancel or finish tasks first')
    queue.append(item)


def settled(goal, pose, linear, angular, position_tolerance=.25, yaw_tolerance=.20):
    if pose is None or linear is None or angular is None:
        return False
    values = (*pose, linear, angular)
    if not all(math.isfinite(v) for v in values):
        return False
    yaw_error = math.atan2(math.sin(pose[2] - goal['yaw']), math.cos(pose[2] - goal['yaw']))
    return (math.hypot(pose[0] - goal['x'], pose[1] - goal['y']) <= position_tolerance
            and abs(yaw_error) <= yaw_tolerance and abs(linear) <= .03 and abs(angular) <= .05)


def coverage_plan(grid, members, center, width, height, spacing, max_goals=256):
    """Partition boustrophedon lanes among robots in one eroded free component.

    Clearance/unknown/boundary checks come from the selected occupancy grid.
    Nav2 still plans actual obstacle-aware connecting paths. This is a finite
    sweep-waypoint mission, not a completeness proof or a terrain planner.
    """
    width, height, spacing = [finite(v, 'coverage extent/spacing') for v in (width, height, spacing)]
    cx, cy = [finite(v, 'coverage center') for v in center]
    if not .5 <= width <= 100 or not .5 <= height <= 100 or not .3 <= spacing <= 10:
        raise ValueError('Coverage width/height must be 0.5–100 m; spacing 0.3–10 m')
    if not members or len(members) > 8:
        raise ValueError('Coverage requires 1–8 fresh idle members')
    radius = max(finite(m['radius'], 'body radius') for m in members) + .10
    safe = grid.clearance > radius + math.sqrt(2) * grid.resolution
    components, _ = label(safe)  # Four-connected free cells; no diagonal corner cutting.
    seeds = [grid.cell(m['x'], m['y']) for m in members]
    ids = []
    for col, row in seeds:
        if not 0 <= row < safe.shape[0] or not 0 <= col < safe.shape[1] or not safe[row, col]:
            raise ValueError('Every coverage member must start in known free body clearance')
        ids.append(int(components[row, col]))
    if len(set(ids)) != 1:
        raise ValueError('Coverage members must share one connected clearance region')
    # Bound generation before allocating large arrays or queues.
    nx, ny = max(2, math.ceil(width / spacing) + 1), max(2, math.ceil(height / spacing) + 1)
    if nx * ny > max_goals * 8:
        raise ValueError('Coverage area is too dense; increase spacing or reduce extents')
    lanes = []
    for index, y in enumerate(np.linspace(cy - height / 2, cy + height / 2, ny)):
        xs = np.linspace(cx - width / 2, cx + width / 2, nx)
        if index % 2:
            xs = xs[::-1]
        lane = []
        for x in xs:
            col, row = grid.cell(float(x), float(y))
            if (0 <= row < safe.shape[0] and 0 <= col < safe.shape[1]
                    and int(components[row, col]) == ids[0]):
                lane.append(dict(x=float(x), y=float(y), yaw=0. if index % 2 == 0 else math.pi,
                                 timeout=180.))
        if lane:
            lanes.append(lane)
    if not lanes or sum(map(len, lanes)) > max_goals:
        raise ValueError('No safe sweep targets, or more than 256 targets; adjust the region/spacing')
    result = {m['id']: [] for m in members}
    positions = {m['id']: (m['x'], m['y']) for m in members}
    for lane in lanes:
        candidates = [m for m in members if len(result[m['id']]) + len(lane) <= MAX_QUEUE]
        if not candidates:
            raise ValueError('Coverage exceeds per-member queue capacity; reduce the area')
        owner = min(candidates, key=lambda m: (len(result[m['id']]) * spacing
                    + math.dist(positions[m['id']], (lane[0]['x'], lane[0]['y'])), m['id']))['id']
        result[owner].extend(lane)
        positions[owner] = (lane[-1]['x'], lane[-1]['y'])
    return result


class MissionHistory:
    """Bounded actual task outcomes, including canceled queued work."""
    def __init__(self):
        self.rows = deque(maxlen=128)

    def finish(self, item, state, reason, observation=None):
        row = dict(task=item['task'], source=item.get('source', 'operator'),
                   goal=item['goal'], state=state, reason=reason,
                   finished=time.monotonic(), observation=observation)
        self.rows.append(row)
        return row


def rmf_navigation_graph(grid, members, extent=12., spacing=1.2):
    """Local annotated graph derived from actual eroded occupancy, not a demo map."""
    radius = max(m['radius'] for m in members) + .15
    ax, ay = members[0]['spawn']['x'], members[0]['spawn']['y']
    vertices, cells, lanes = [], {}, []
    count = min(10, math.ceil(extent / spacing / 2))
    for y in range(-count, count + 1):
        for x in range(-count, count + 1):
            px, py = ax + x * spacing, ay + y * spacing
            if grid.clear(px, py, radius):
                cells[x, y] = len(vertices)
                vertices.append(dict(name='p' + str(len(vertices)), x=px, y=py))
    for (x, y), first in cells.items():
        for neighbor in ((x + 1, y), (x, y + 1)):
            if neighbor not in cells:
                continue
            second = cells[neighbor]
            a, b = vertices[first], vertices[second]
            samples = max(2, math.ceil(math.hypot(b['x'] - a['x'], b['y'] - a['y']) / (grid.resolution / 2)))
            if all(grid.clear(a['x'] + t * (b['x'] - a['x']), a['y'] + t * (b['y'] - a['y']), radius)
                   for t in np.linspace(0., 1., samples)):
                lanes.append([first, second])
    # Every distinct spawn is a real holding vertex. Snapping a second member
    # up to a metre onto its peer's graph node corrupts RMF's traffic picture.
    for member in members:
        px, py = member['spawn']['x'], member['spawn']['y']
        if any(math.hypot(v['x']-px, v['y']-py) < .10 for v in vertices):
            continue
        first = len(vertices)
        connected = []
        for second in sorted(range(len(vertices)), key=lambda i: math.hypot(vertices[i]['x']-px, vertices[i]['y']-py)):
            b = vertices[second]
            distance = math.hypot(b['x']-px, b['y']-py)
            if distance > spacing*1.6:
                break
            steps = max(2, math.ceil(distance / (grid.resolution/2)))
            if all(grid.clear(px+t*(b['x']-px), py+t*(b['y']-py), radius)
                   for t in np.linspace(0., 1., steps)):
                connected.append([first, second])
                if len(connected) == 3:
                    break
        if not connected:
            raise ValueError('No clearance-valid RMF graph connection for '+member['id'])
        vertices.append(dict(name='home_'+member['id'], x=px, y=py))
        lanes.extend(connected)
    if not vertices or not lanes:
        raise ValueError('No connected RMF lanes fit the selected occupancy/body clearance')
    return dict(vertices=vertices, lanes=lanes, scope='Local occupancy-derived graph; no door/lift/charging infrastructure')


def traffic_yield_goal(grid, robot, peers):
    """Find a nearby static-clear holding pose with a peer-clear retreat path."""
    if not peers:
        raise ValueError('A measured blocking peer is required')
    nearest = min(peers, key=lambda p: math.hypot(robot.x-p.x, robot.y-p.y))
    away = math.atan2(robot.y-nearest.y, robot.x-nearest.x)
    for distance in (.65, 1., 1.5):
        for offset in (0., .65, -.65, 1.2, -1.2):
            heading = away+offset
            x, y = robot.x+distance*math.cos(heading), robot.y+distance*math.sin(heading)
            if not grid.clear(x, y, robot.radius+.12):
                continue
            safe = True
            for t in np.linspace(0., 1., max(3, math.ceil(distance/(grid.resolution/2)))):
                tx, ty = robot.x+t*(x-robot.x), robot.y+t*(y-robot.y)
                if not grid.clear(tx, ty, robot.radius+.03) or any(
                        math.hypot(tx-p.x, ty-p.y) < robot.radius+p.radius+.04 for p in peers):
                    safe = False
                    break
            if safe:
                return dict(x=x, y=y, yaw=heading, timeout=45., not_before=0.)
    raise ValueError('No observed peer-clear retreat to a free holding pose; operator intervention required')
