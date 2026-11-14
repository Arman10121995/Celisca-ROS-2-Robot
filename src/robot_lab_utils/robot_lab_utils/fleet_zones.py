"""Shared-zone reservations: one robot at a time through single-lane passages,
doors and chargers (ROADMAP R10.2).

Zones come from the facility declaration (``shared_zones``, door sweeps,
docks) and from the occupancy map: corridors too narrow for two members to
pass become single-lane zones. Each zone is granted to one member. A member
whose fresh Nav2 plan enters a zone granted to a peer is held before the
entry; it is not moved by this module. Grants follow observed bodies and
plans, never declared intent: a parked member inside a zone holds it.
"""
from dataclasses import dataclass, field
import math

import numpy as np

LOOKAHEAD_M = 6.
STOP_DISTANCE_M = 1.
GRANT_DISTANCE_M = 2.
PLAN_FRESH_S = 3.


@dataclass
class Zone:
    name: str
    kind: str                       # passage | door | dock | declared
    rects: list = field(default_factory=list)   # [(x0, x1, y0, y1)]
    grid: object = None             # SpawnGrid for map-derived masks
    mask: object = None             # bool array aligned with grid.free
    floor: str = ''
    holding: list = field(default_factory=list)  # optional waiting poses
    bounds: tuple = None

    def contains(self, x, y, radius=0.):
        if self.bounds:
            x0, x1, y0, y1 = self.bounds
            if x + radius < x0 or x - radius > x1 or y + radius < y0 or y - radius > y1:
                return False
        if self.mask is None:
            return any(x + radius > x0 and x - radius < x1 and y + radius > y0 and y - radius < y1
                       for x0, x1, y0, y1 in self.rects)
        points = [(x, y)] + [(x + radius * math.cos(a), y + radius * math.sin(a))
                             for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)] if radius else [(x, y)]
        for px, py in points:
            col, row = self.grid.cell(px, py)
            if 0 <= row < self.mask.shape[0] and 0 <= col < self.mask.shape[1] and self.mask[row, col]:
                return True
        return False

    def describe(self):
        return dict(name=self.name, kind=self.kind, floor=self.floor or None,
                    bounds=[round(v, 3) for v in self.bounds] if self.bounds else None,
                    holding=self.holding)


def _rect(values):
    x0, x1, y0, y1 = (float(v) for v in values)
    if not all(map(math.isfinite, (x0, x1, y0, y1))) or x0 >= x1 or y0 >= y1:
        raise ValueError('A zone rectangle needs finite x0 < x1, y0 < y1')
    return (x0, x1, y0, y1)


def declared_zones(config, dock_half_m=.6):
    """Zones from a facility declaration: explicit zones, door sweeps, docks."""
    zones = []
    for name, data in (config.get('shared_zones') or {}).items():
        rects = [_rect(r) for r in data['rects']]
        holding = [dict(x=float(p['x']), y=float(p['y']), yaw=float(p.get('yaw', 0.))) for p in data.get('holding', [])]
        zones.append(Zone(name, data.get('kind', 'declared'), rects, floor=str(data.get('floor', '')), holding=holding,
                          bounds=(min(r[0] for r in rects), max(r[1] for r in rects),
                                  min(r[2] for r in rects), max(r[3] for r in rects))))
    for name, door in (config.get('doors') or {}).items():
        # The lift car has its own reservation; landing gates belong to it.
        if door.get('z_joint') or door.get('landing_floor') or 'sweep' not in door:
            continue
        rect = _rect(door['sweep'])
        zones.append(Zone('door:' + name, 'door', [rect], floor=str(door.get('floor', '')), bounds=rect))
    for name, dock in (config.get('docks') or {}).items():
        rect = (dock['x'] - dock_half_m, dock['x'] + dock_half_m, dock['y'] - dock_half_m, dock['y'] + dock_half_m)
        zones.append(Zone('dock:' + name, 'dock', [rect], floor=str(dock.get('floor', '')), bounds=rect))
    names = [z.name for z in zones]
    if len(names) != len(set(names)):
        raise ValueError('Duplicate shared-zone names')
    return zones


def narrow_passages(grid, radius, passing_clearance=None, min_length=.3, floor=''):
    """Single-lane corridors from an occupancy grid.

    A ridge (medial) cell with clearance d is passable for one member when
    d > radius and too narrow for two side by side when d < passing_clearance
    (default 2 radius + 0.25 m, i.e. two footprints plus Nav2 inflation gaps).
    Each connected ridge piece plus the free cells it spans becomes a zone.
    """
    from scipy.ndimage import distance_transform_edt, label, maximum_filter
    passing_clearance = 2 * radius + .25 if passing_clearance is None else passing_clearance
    d = np.where(grid.free, grid.clearance, 0.)
    ridge = grid.free & (d >= maximum_filter(d, size=3) - 1e-9)
    single = ridge & (d > radius + grid.resolution) & (d < passing_clearance)
    labels, count = label(single, structure=np.ones((3, 3)))
    if not count:
        return []
    lengths = np.bincount(labels.ravel())
    distance, (rows, cols) = distance_transform_edt(labels == 0, return_indices=True)
    nearest = labels[rows, cols]
    reach = d[rows, cols] + grid.resolution
    region = grid.free & (distance * grid.resolution <= reach)
    zones = []
    for index in range(1, count + 1):
        if lengths[index] * grid.resolution < min_length:
            continue
        mask = region & (nearest == index)
        rr, cc = np.nonzero(mask)
        corners = [_world(grid, c, r) for r, c in ((rr.min(), cc.min()), (rr.max() + 1, cc.max() + 1),
                                                    (rr.min(), cc.max() + 1), (rr.max() + 1, cc.min()))]
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        zones.append(Zone('passage:%s%d' % (floor + ':' if floor else '', len(zones)), 'passage', grid=grid,
                          mask=mask, floor=floor, bounds=(min(xs), max(xs), min(ys), max(ys))))
    return zones


def _world(grid, col, row):
    c, s = math.cos(grid.origin[2]), math.sin(grid.origin[2])
    u, v = col * grid.resolution, row * grid.resolution
    return grid.origin[0] + c * u - s * v, grid.origin[1] + s * u + c * v


def plan_entry(zone, plan, position, radius, lookahead=LOOKAHEAD_M):
    """Distance along the plan to its first footprint inside the zone, or None."""
    if not plan:
        return None
    # Nav2 plans start at the robot; skip points the robot has already passed.
    start = min(range(len(plan)), key=lambda i: math.hypot(plan[i][0] - position[0], plan[i][1] - position[1]))
    travelled, previous = math.hypot(plan[start][0] - position[0], plan[start][1] - position[1]), plan[start]
    checked = -math.inf
    for index, point in enumerate(plan[start:], start):
        travelled += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
        if travelled > lookahead:
            return None
        # Nav2 plans are spaced at the costmap resolution; 10 cm steps suffice.
        if travelled - checked >= .1 or index == len(plan) - 1:
            checked = travelled
            if zone.contains(point[0], point[1], radius):
                return travelled
    return None


class ZoneArbiter:
    """Grants each zone to one member and lists members that must wait.

    ``update`` takes observed rows: {name: dict(x, y, radius, floor,
    plan=[(x, y), ...] or None, navigating, waiting_for=zone or None,
    waiting_since)}. Priority: a member inside the zone, then the current
    grant while its holder still claims it, then members already waiting
    (longest first), then the nearest plan entry within the grant distance.
    A distant claim does not take a grant, so it cannot hold up a member
    that reaches the entry first.
    """
    def __init__(self, zones, lookahead=LOOKAHEAD_M, stop_distance=STOP_DISTANCE_M, grant_distance=GRANT_DISTANCE_M):
        self.zones = {zone.name: zone for zone in zones}
        self.lookahead, self.stop_distance, self.grant_distance = lookahead, stop_distance, grant_distance
        self.grants = {}

    def update(self, rows):
        occupants, claims, waits = {}, {}, {}
        for zone in self.zones.values():
            for name, row in rows.items():
                if zone.floor and row.get('floor') and zone.floor != row['floor']:
                    continue
                if zone.contains(row['x'], row['y'], row['radius']):
                    occupants.setdefault(zone.name, []).append(name)
                elif row.get('waiting_for') == zone.name:
                    claims.setdefault(zone.name, []).append((0, row.get('waiting_since', 0.), name, 0.))
                elif row.get('navigating') and row.get('plan'):
                    entry = plan_entry(zone, row['plan'], (row['x'], row['y']), row['radius'], self.lookahead)
                    if entry is not None:
                        claims.setdefault(zone.name, []).append((1, entry, name, entry))
        for name in list(self.grants):
            holder = self.grants[name]
            if holder not in occupants.get(name, []) and holder not in [c[2] for c in claims.get(name, [])]:
                del self.grants[name]
        for zone in self.zones:
            inside = sorted(occupants.get(zone, []))
            if inside:
                if self.grants.get(zone) not in inside:
                    self.grants[zone] = inside[0]
            elif zone not in self.grants:
                near = [c for c in claims.get(zone, []) if c[0] == 0 or c[3] <= self.grant_distance]
                if near:
                    self.grants[zone] = min(near)[2]
            holder = self.grants.get(zone)
            for _rank, _key, name, entry in claims.get(zone, []):
                if name == holder or name in inside:
                    continue
                if rows[name].get('waiting_for') == zone or entry <= self.stop_distance:
                    # Keep the nearest blocking zone per member.
                    if name not in waits or entry < waits[name]['entry_m']:
                        waits[name] = dict(zone=zone, holder=holder, entry_m=round(entry, 3),
                                           holding=self.zones[zone].holding)
        return dict(grants=dict(self.grants), occupants={k: sorted(v) for k, v in occupants.items()}, waits=waits,
                    shared=sorted(z for z, v in occupants.items() if len(v) > 1))
