"""Clearance-derived official RMF building YAML for real lift-session events.

The native Humble parser supplies LiftSessionBegin/Move/DoorOpen/SessionEnd;
no mock-floor event, synthetic vehicle motion or private binding is used.
"""
import math

from robot_lab_utils.fleet_facility_graph import _touches
from robot_lab_utils.fleet_missions import rmf_navigation_graph
from robot_lab_utils.fleet_session import SpawnGrid


def building_graph(session, config):
    radius = max(member['radius'] for member in session['members']) + .15
    lift = config['lift']
    transport = config['transport']
    levels, metadata = {}, []
    fixed_doors = {name: door for name, door in config['doors'].items()
                   if not door.get('z_joint') and not door.get('landing_floor')}
    for floor in sorted(session['floor_maps'], key=lambda name: lift['floors'][name]):
        grid = SpawnGrid(session['floor_maps'][floor])
        reference = dict(id='reference_' + floor, radius=radius - .15,
                         spawn=transport['landings'][floor])
        members = session['members'] if floor == 'L0' else [reference]
        raw = rmf_navigation_graph(grid, members, extent=24.)
        zones = [[2.8, 5.2, -1.2, 1.2]]  # The source car footprint/portal.
        doors = fixed_doors if floor == 'L0' else {}
        for door in doors.values():
            x0, x1, y0, y1 = door['sweep']
            zones.append([x0 - radius, x1 + radius, y0 - radius, y1 + radius])

        def outside(vertex):
            return not any(box[0] <= vertex['x'] <= box[1] and box[2] <= vertex['y'] <= box[3]
                           for box in zones)

        def clear_segment(a, b, avoid_zones=True):
            if avoid_zones and any(_touches(a, b, box) for box in zones):
                return False
            steps = max(2, math.ceil(math.hypot(b['x'] - a['x'], b['y'] - a['y']) / (grid.resolution / 2)))
            return all(grid.clear(a['x'] + i / steps * (b['x'] - a['x']),
                                  a['y'] + i / steps * (b['y'] - a['y']), radius)
                       for i in range(steps + 1))

        retained = [index for index, vertex in enumerate(raw['vertices']) if outside(vertex)]
        indices = {old: new for new, old in enumerate(retained)}
        vertices = [dict(raw['vertices'][i], floor=floor) for i in retained]
        for vertex in vertices:
            if floor != 'L0':
                vertex['name'] = floor + '_' + vertex['name']
        lanes = []

        def pair(first, second, properties=None):
            lanes.extend([[first, second, dict(properties or {})],
                          [second, first, dict(properties or {})]])

        for first, second in raw['lanes']:
            if first in indices and second in indices and clear_segment(raw['vertices'][first], raw['vertices'][second]):
                pair(indices[first], indices[second])

        def anchor(name, pose):
            if not outside(pose) or not grid.clear(pose['x'], pose['y'], radius):
                raise ValueError('Source approach violates floor/body clearance: ' + name)
            same = next((i for i, v in enumerate(vertices)
                         if math.hypot(v['x'] - pose['x'], v['y'] - pose['y']) < .001), None)
            if same is not None:
                vertices[same]['name'] = name
                return same
            connected = []
            for other in sorted(range(len(vertices)), key=lambda i: math.hypot(vertices[i]['x'] - pose['x'], vertices[i]['y'] - pose['y'])):
                if math.hypot(vertices[other]['x'] - pose['x'], vertices[other]['y'] - pose['y']) > 2.4:
                    break
                if clear_segment(pose, vertices[other]):
                    connected.append(other)
                    if len(connected) == 3:
                        break
            if not connected:
                raise ValueError('No source-clear graph connection: ' + name)
            index = len(vertices)
            vertices.append(dict(name=name, x=pose['x'], y=pose['y'], floor=floor))
            for other in connected:
                pair(other, index)
            return index

        for name, door in doors.items():
            approaches = [dict(x=x, y=y) for x, y in door['approach']]
            first, second = [anchor('door_' + name + '_' + str(i), p) for i, p in enumerate(approaches)]
            if not clear_segment(approaches[0], approaches[1], False):
                raise ValueError('Source door portal is obstructed')
            pair(first, second, dict(door_name=name))

        # Only these two gated lanes enter/exit each car. Ordinary lattice
        # edges cannot bypass the native lift-session handshake.
        landing = anchor(floor + '_landing', transport['landings'][floor])
        boarding = transport['boarding']
        if not grid.clear(boarding['x'], boarding['y'], radius) or not clear_segment(transport['landings'][floor], boarding, False):
            raise ValueError('Lift boarding violates declared body clearance')
        inside = len(vertices)
        vertices.append(dict(name='lift_' + floor + '_inside', floor=floor,
                             x=boarding['x'], y=boarding['y'], lift=lift['name'], selectable=False))
        pair(landing, inside, dict(orientation_constraint='forward'))

        for name, dock in config.get('docks', {}).items():
            if abs(dock.get('z', 0.) - lift['floors'][floor]) > dock.get('height_tolerance', .12):
                continue
            index = anchor(name, dock)
            vertices[index]['dock'] = name
            for lane in lanes:
                if lane[1] == index:
                    if lane[2].get('door_name'):
                        raise ValueError('Split overlapping native dock/door approaches')
                    lane[2]['dock_name'] = name

        for vertex in vertices:
            options = dict(name=vertex['name'], is_holding_point=not bool(vertex.get('lift')))
            if vertex.get('lift'):
                options['lift'] = vertex['lift']
            if vertex.get('dock'):
                options['is_charger'] = True
            vertex['options'] = options
        levels[floor] = dict(vertices=[[v['x'], v['y'], v['options']] for v in vertices], lanes=lanes)
        metadata.extend([{k: v for k, v in vertex.items() if k != 'options'} for vertex in vertices])
    return dict(building_name=session['map'], levels=levels), metadata
