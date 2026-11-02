"""Source-checked formation itineraries and estimated settling, separate from flight proof."""
import math
import time

import numpy as np


def slots(anchor, names, spacing):
    return {name: np.asarray(anchor, dtype=float) + [0., (i-(len(names)-1)/2)*spacing, 0.]
            for i, name in enumerate(sorted(names))}


def plan_route(scene, positions, origins, anchors, spacing, canceled, timeout=8.):
    names = sorted(positions)
    if not 2 <= len(names) <= 4 or set(origins) != set(names):
        raise ValueError('Formation routes need 2–4 distinct airborne members')
    if not math.isfinite(spacing) or not 2. <= spacing <= 8.:
        raise ValueError('Formation spacing must be 2–8 m')
    points = np.asarray(anchors, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not 1 <= len(points) <= 16 or not np.isfinite(points).all():
        raise ValueError('Provide 1–16 finite world XYZ formation anchors')
    starts = {name: np.asarray(positions[name], dtype=float) for name in names}
    if any(value.shape != (3,) or not np.isfinite(value).all() for value in starts.values()):
        raise ValueError('Fresh finite member positions required')
    previous, total = starts, 0.
    deadline = time.monotonic() + timeout
    for anchor in points:
        targets = slots(anchor, names, spacing)
        for name in names:
            origin = np.asarray(origins[name], dtype=float)
            if origin.shape != (3,) or not np.isfinite(origin).all() or not origin[2]+.5 <= targets[name][2] <= origin[2]+10.:
                raise ValueError('Every formation slot must stay within its member’s 0.5–10 m height envelope')
            if canceled() or time.monotonic() >= deadline:
                raise TimeoutError('Formation planning canceled or deadline exceeded')
            if not scene.segment(previous[name], targets[name], .8, deadline, canceled):
                raise ValueError('Formation corridor intersects selected-world source geometry: ' + name)
            if np.linalg.norm(targets[name]-previous[name]) > 20.:
                raise ValueError('A formation leg must be at most 20 m within its execution deadline')
        # Check swept pair trajectories, not only endpoint separation. This
        # planner admits synchronized straight transfers; runtime guards still
        # observe actual peers and never assume they follow the plan exactly.
        for i, first in enumerate(names):
            for second in names[i+1:]:
                relative = previous[first] - previous[second]
                delta = targets[first] - previous[first] - targets[second] + previous[second]
                progress = float(np.clip(-np.dot(relative, delta)/max(float(np.dot(delta, delta)), 1e-12), 0., 1.))
                if np.linalg.norm(relative + progress*delta) < 2.:
                    raise ValueError('Formation transfer crosses another selected member’s 2 m separation envelope')
        total += max(float(np.linalg.norm(targets[name]-previous[name])) for name in names)
        if total > 100.:
            raise ValueError('Formation route exceeds the bounded 100 m anchor travel budget')
        previous = targets
    return dict(targets=names, anchors=points.tolist(), spacing=spacing,
                starts={name: value.tolist() for name, value in starts.items()},
                distance_m=total, observer='FCU estimated positions; source-static corridors')


class FormationRoute:
    def __init__(self, plan, now):
        self.plan = plan
        self.index = 0
        self.phase = 'executing'
        self.reason = 'Following source-checked formation anchors'
        self.started = self.leg_started = now
        self.settled_stamps = None

    @property
    def anchor(self):
        return self.plan['anchors'][self.index]

    def cancel(self, reason):
        if self.phase == 'executing':
            self.phase, self.reason = 'canceled', reason

    def tick(self, positions, speeds, stamps, now):
        if self.phase != 'executing':
            return
        if now-self.started > 600. or now-self.leg_started > 120.:
            self.cancel('Formation route deadline exceeded; explicit Plan/Execute required')
            return
        targets = slots(self.anchor, self.plan['targets'], self.plan['spacing'])
        settled = all(name in positions and name in speeds and speeds[name] is not None
                      and stamps.get(name) is not None and math.isfinite(stamps[name])
                      and math.isfinite(speeds[name]) and 0. <= speeds[name] < .08
                      and np.linalg.norm(np.asarray(positions[name])-target) < .2 for name, target in targets.items())
        self.settled_stamps = ({name: stamps[name] for name in targets} if settled and self.settled_stamps is None
                               else self.settled_stamps if settled else None)
        if self.settled_stamps and all(stamps[name]-start >= 1. for name, start in self.settled_stamps.items()):
            if self.index+1 == len(self.plan['anchors']):
                self.phase, self.reason = 'completed', 'All formation anchors settled in FCU estimates; independent flight qualification remains separate'
            else:
                self.index += 1
                self.leg_started, self.settled_stamps = now, None

    def state(self):
        return dict(phase=self.phase, reason=self.reason, waypoint=self.index+1,
                    total=len(self.plan['anchors']), anchor=self.anchor,
                    observer='FCU estimated pose and timestamp-derived speed; no simulator truth control')
