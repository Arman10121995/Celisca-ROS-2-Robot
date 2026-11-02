"""Source-declared door/dock events on clearance-tested RMF lanes.

Lift travel is a separate multi-level mission contract: lift_session_graph()
adds per-floor vertex copies connected by lift-shaft lanes with declared
door-open/close events at each landing.  It is not inferred from a moving
door or from the availability of floor occupancy files.
"""
import copy
import math


def _touches(a, b, box):
    lower, upper = 0., 1.
    for axis, lo, hi in (('x', box[0], box[1]), ('y', box[2], box[3])):
        delta = b[axis] - a[axis]
        if abs(delta) < 1e-12:
            if not lo <= a[axis] <= hi:
                return False
        else:
            first, last = sorted(((lo - a[axis]) / delta, (hi - a[axis]) / delta))
            lower, upper = max(lower, first), min(upper, last)
            if lower > upper:
                return False
    return True


def facility_graph(data, grid, members, config):
    result = copy.deepcopy(data)
    vertices, lanes = result['vertices'], result['lanes']
    radius = max(member['radius'] for member in members) + .15
    # A coarse lattice can miss a narrow portal entirely. Add explicit source
    # approaches and check every connecting segment with the same body erosion.
    for name, door in config.get('doors', {}).items():
        if door.get('z_joint') or not door.get('approach'):
            continue
        approaches = door['approach']
        if len(approaches) != 2:
            raise ValueError('Door requires two explicit approach points')
        indices = []
        for side, (x, y) in enumerate(approaches):
            if not all(map(math.isfinite, (x, y))) or not grid.clear(x, y, radius):
                raise ValueError('Door approach violates fleet-body clearance: ' + name)
            same = next((i for i, vertex in enumerate(vertices)
                         if math.hypot(vertex['x'] - x, vertex['y'] - y) < .001), None)
            if same is not None:
                vertices[same]['name'] = 'door_' + name + '_' + str(side)
                indices.append(same)
                continue
            index = len(vertices)
            connections = []
            for other in sorted(range(index), key=lambda i: math.hypot(vertices[i]['x'] - x, vertices[i]['y'] - y)):
                vertex = vertices[other]
                distance = math.hypot(vertex['x'] - x, vertex['y'] - y)
                if distance > 2.4:
                    break
                steps = max(2, math.ceil(distance / (grid.resolution / 2)))
                if all(grid.clear(x + t / steps * (vertex['x'] - x),
                                  y + t / steps * (vertex['y'] - y), radius)
                       for t in range(steps + 1)):
                    connections.append([other, index])
                    if len(connections) == 3:
                        break
            if not connections:
                raise ValueError('Door approach has no clearance-valid graph connection: ' + name)
            vertices.append(dict(name='door_' + name + '_' + str(side), x=x, y=y))
            lanes.extend(connections)
            indices.append(index)
        a, b = (vertices[i] for i in indices)
        steps = max(2, math.ceil(math.hypot(b['x'] - a['x'], b['y'] - a['y']) / (grid.resolution / 2)))
        if not all(grid.clear(a['x'] + t / steps * (b['x'] - a['x']),
                              a['y'] + t / steps * (b['y'] - a['y']), radius)
                   for t in range(steps + 1)):
            raise ValueError('Door portal is not traversable at fleet-body clearance: ' + name)
        lanes.append(indices)
    docks = {}
    for name, dock in config.get('docks', {}).items():
        if not all(math.isfinite(float(dock[k])) for k in ('x', 'y', 'yaw')):
            raise ValueError('Finite source dock geometry required')
        if not grid.clear(dock['x'], dock['y'], radius):
            raise ValueError('Dock is outside fleet-body occupancy clearance: ' + name)
        same = next((i for i, vertex in enumerate(vertices)
                     if math.hypot(vertex['x'] - dock['x'], vertex['y'] - dock['y']) < .001), None)
        if same is not None:
            index = same
            vertices[index]['name'] = name
        else:
            index = len(vertices)
            connections = []
            for other in sorted(range(index), key=lambda i: math.hypot(vertices[i]['x'] - dock['x'], vertices[i]['y'] - dock['y'])):
                vertex = vertices[other]
                distance = math.hypot(vertex['x'] - dock['x'], vertex['y'] - dock['y'])
                if distance > 2.4:
                    break
                steps = max(2, math.ceil(distance / (grid.resolution / 2)))
                if all(grid.clear(dock['x'] + step / steps * (vertex['x'] - dock['x']),
                                  dock['y'] + step / steps * (vertex['y'] - dock['y']), radius)
                       for step in range(steps + 1)):
                    connections.append([other, index])
                    if len(connections) == 3:
                        break
            if not connections:
                raise ValueError('No clearance-valid dock approach: ' + name)
            vertices.append(dict(name=name, x=dock['x'], y=dock['y']))
            lanes.extend(connections)
        docks[index] = name
        vertices[index]['dock'] = name
    doors = {name: value for name, value in config.get('doors', {}).items() if not value.get('z_joint')}
    directed = []
    for first, second in lanes:
        for entry, exit_ in ((first, second), (second, first)):
            a, b = vertices[entry], vertices[exit_]
            row = dict(entry=entry, exit=exit_, entry_event=None, exit_event=None)
            for name, door in doors.items():
                x0, x1, y0, y1 = door['sweep']
                box = [x0 - radius, x1 + radius, y0 - radius, y1 + radius]
                if not _touches(a, b, box):
                    continue
                if row['entry_event']:
                    raise ValueError('Overlapping door events require an explicitly split RMF lane')
                row['entry_event'] = dict(kind='door_open', name=name)
                if not box[0] <= b['x'] <= box[1] or not box[2] <= b['y'] <= box[3]:
                    row['exit_event'] = dict(kind='door_close', name=name)
            if exit_ in docks:
                if row['exit_event']:
                    raise ValueError('Dock approach overlaps a door close; split the source lane')
                row['exit_event'] = dict(kind='dock', name=docks[exit_])
            directed.append(row)
    for vertex in vertices:
        vertex['holding'] = not any(
            door['sweep'][0] - radius <= vertex['x'] <= door['sweep'][1] + radius and
            door['sweep'][2] - radius <= vertex['y'] <= door['sweep'][3] + radius
            for door in doors.values())
    result.update(directed_lanes=directed,
                  scope='Clearance-derived single-level RMF graph with declared native door/dock events; lift lane sessions pending')
    return result


def install_lanes(nav_graph, data, lane_api):
    """Use official Humble factories; preserve integer indices across appends."""
    from datetime import timedelta

    def event(value):
        if value is None:
            return None
        name = value['name']
        if value['kind'] == 'dock':
            return lane_api.Event.dock_make(lane_api.Dock(name, timedelta(seconds=120)))
        if value['kind'] == 'door_open':
            return lane_api.Event.door_open_make(lane_api.DoorOpen(name, timedelta(seconds=20)))
        if value['kind'] == 'door_close':
            return lane_api.Event.door_close_make(lane_api.DoorClose(name, timedelta(seconds=20)))
        raise ValueError('Unknown facility lane event')

    if 'directed_lanes' not in data:
        for first, second in data['lanes']:
            nav_graph.add_bidir_lane(first, second)
        return
    for row in data['directed_lanes']:
        start, end = event(row['entry_event']), event(row['exit_event'])
        nav_graph.add_lane(lane_api.Node(row['entry'], start) if start else lane_api.Node(row['entry']),
                           lane_api.Node(row['exit'], end) if end else lane_api.Node(row['exit']))


def lift_session_graph(data, grid, members, config):
    """Add per-floor vertex copies connected by lift-shaft lanes.

    Each floor gets its own copy of the single-level vertices.  Lift-shaft
    lanes connect the landing position on each floor to the same position on
    adjacent floors, with declared lift-door open/close events at each end.
    The RMF task planner then routes multi-floor missions through the lift
    automatically.

    Parameters
    ----------
    data : dict
        Output of facility_graph() with vertices, lanes and directed_lanes.
    grid : SpawnGrid
        Occupancy grid for clearance checks.
    members : list[dict]
        Session members, used for body radius.
    config : dict
        Facility infrastructure config with 'lift' section containing
        'floors' (floor name -> z height), 'cabin' ([x0,x1,y0,y1]),
        'landings' (floor name -> [x,y]) and 'joint'.

    Returns
    -------
    dict
        Extended data with 'lift_vertices' (per-floor vertex copies),
        'lift_lanes' (shaft connections), 'lift_directed' (lanes with
        lift-door events) and updated 'directed_lanes' including lift lanes.
    """
    lift = config.get('lift')
    if not lift:
        return data
    floors = lift.get('floors')
    if not floors or len(floors) < 2:
        return data
    cabin = lift.get('cabin')
    if not cabin or len(cabin) != 4:
        raise ValueError('Lift cabin requires [x0, x1, y0, y1]')
    landings = lift.get('landings')
    if not landings:
        raise ValueError('Lift landings required per floor')

    radius = max(member['radius'] for member in members) + .15
    vertices = data['vertices']
    floor_order = sorted(floors, key=lambda f: float(floors[f]))

    lift_vertices = {}
    for floor in floor_order:
        floor_vertices = []
        for v in vertices:
            floor_vertices.append(dict(
                name=v['name'] + '@' + floor,
                x=v['x'], y=v['y'],
                floor=floor,
                holding=v.get('holding', True),
                dock=v.get('dock'),
            ))
        lift_vertices[floor] = floor_vertices

    lift_lanes = []
    for i in range(len(floor_order) - 1):
        f0, f1 = floor_order[i], floor_order[i + 1]
        if f0 not in landings or f1 not in landings:
            continue
        p0 = landings[f0]
        p1 = landings[f1]
        if not all(map(math.isfinite, (*p0, *p1))):
            continue
        def nearest(floor_vertices, px, py):
            best = min(range(len(floor_vertices)),
                       key=lambda j: math.hypot(floor_vertices[j]['x'] - px, floor_vertices[j]['y'] - py))
            return best
        i0 = nearest(lift_vertices[f0], *p0)
        i1 = nearest(lift_vertices[f1], *p1)
        steps = max(2, math.ceil(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / (grid.resolution / 2)))
        if not all(grid.clear(p0[0] + t / steps * (p1[0] - p0[0]),
                              p0[1] + t / steps * (p1[1] - p0[1]), radius)
                   for t in range(steps + 1)):
            raise ValueError('Lift shaft segment is not clearance-valid: ' + f0 + ' -> ' + f1)
        lift_lanes.append(dict(
            entry=lift_vertices[f0][i0]['name'],
            exit=lift_vertices[f1][i1]['name'],
            entry_event=dict(kind='lift_door_open', name='lift_' + f0),
            exit_event=dict(kind='lift_door_close', name='lift_' + f1),
            floors=(f0, f1),
        ))
        lift_lanes.append(dict(
            entry=lift_vertices[f1][i1]['name'],
            exit=lift_vertices[f0][i0]['name'],
            entry_event=dict(kind='lift_door_open', name='lift_' + f1),
            exit_event=dict(kind='lift_door_close', name='lift_' + f0),
            floors=(f1, f0),
        ))

    lift_directed = []
    for lane in lift_lanes:
        lift_directed.append(dict(
            entry=lane['entry'], exit=lane['exit'],
            entry_event=lane['entry_event'], exit_event=lane['exit_event'],
        ))
    directed = list(data.get('directed_lanes', [])) + lift_directed

    result = copy.deepcopy(data)
    result['lift_vertices'] = lift_vertices
    result['lift_lanes'] = lift_lanes
    result['lift_directed'] = lift_directed
    result['directed_lanes'] = directed
    result['scope'] = ('Multi-level RMF graph: per-floor vertex copies connected by '
                       'lift-shaft lanes with declared lift-door events; '
                       'level-aware dispatch routes tasks across floors')
    return result
