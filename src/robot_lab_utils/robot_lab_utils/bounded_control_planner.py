"""Bounded bidirectional RRT-Connect over finite source control coordinates.

The caller supplies directed collision validation including its actual command
interpolation. This is a local implementation of the published algorithm, not
an OMPL binding or a guarantee of completeness within the declared budgets.
"""
import time

import numpy as np


def connect_controls(start, goal, limits, edge_valid, *, deadline, seed=0,
                     max_nodes=384, max_edges=2048, extension=.18):
    start, goal, limits = (np.asarray(value, dtype=float) for value in (start, goal, limits))
    if (start.ndim != 1 or not 1 <= len(start) <= 12 or goal.shape != start.shape
            or limits.shape != (len(start), 2) or not all(np.all(np.isfinite(v)) for v in (start, goal, limits))
            or np.any(limits[:, 1] <= limits[:, 0]) or not np.isfinite(deadline)
            or not np.isfinite(extension) or extension <= 0 or max_nodes < 2 or max_edges < 1):
        raise ValueError('Finite bounded control-space planning inputs required')
    lower, span = limits[:, 0], limits[:, 1]-limits[:, 0]
    if any(np.any(v < lower) or np.any(v > lower+span) for v in (start, goal)):
        raise ValueError('Planning endpoint exceeds source control limits')
    rng = np.random.default_rng(seed)
    checks, nodes = 0, 2

    def budget():
        if time.monotonic() >= deadline:
            raise ValueError('Source configuration planning deadline exceeded')
        if checks >= max_edges:
            raise ValueError('Source configuration collision-edge budget exhausted')

    def valid(a, b):
        nonlocal checks
        budget()
        checks += 1
        return bool(edge_valid(a, b))

    if not valid(start, start) or not valid(goal, goal):
        raise ValueError('Source collision at planning start or goal; choose another approach')
    if valid(start, goal):
        return dict(path=[start.copy(), goal.copy()], planner='direct', seed=seed,
                    nodes=nodes, collision_edges=checks)

    # Each tree records whether it grows from the start. Goal-tree edges must
    # be checked child -> parent because actuator slew need not be reversible.
    trees = [dict(points=[start.copy()], parents=[-1], start=True),
             dict(points=[goal.copy()], parents=[-1], start=False)]

    def grow(tree, target):
        nonlocal nodes
        budget()
        points = np.asarray(tree['points'])
        nearest = int(np.argmin(np.linalg.norm((points-target)/span, axis=1)))
        origin = points[nearest]
        distance = float(np.linalg.norm((target-origin)/span))
        if distance < 1e-12:
            return nearest, True
        if nodes >= max_nodes:
            raise ValueError('Source configuration node budget exhausted; no collision-free route found')
        candidate = target.copy() if distance <= extension else origin+(target-origin)*(extension/distance)
        free = valid(origin, candidate) if tree['start'] else valid(candidate, origin)
        if not free:
            return None, False
        tree['points'].append(candidate)
        tree['parents'].append(nearest)
        nodes += 1
        return len(tree['points'])-1, distance <= extension

    def ancestry(tree, index):
        result = []
        while index >= 0:
            result.append(tree['points'][index].copy())
            index = tree['parents'][index]
        return result[::-1]

    while nodes < max_nodes:
        budget()
        active, other = trees
        sample = other['points'][0] if rng.random() < .15 else lower+rng.random(len(start))*span
        first, _ = grow(active, sample)
        if first is not None:
            target = active['points'][first]
            while True:
                second, reached = grow(other, target)
                if second is None:
                    break
                if reached:
                    a, b = ancestry(active, first), ancestry(other, second)
                    path = a+b[-2::-1] if active['start'] else b+a[-2::-1]
                    # Greedy directed shortcuts are optional; never throw away
                    # a found route merely because simplification ran out of time.
                    index, attempts = 0, 0
                    while index < len(path)-2 and attempts < 32:
                        for end in range(len(path)-1, index+1, -1):
                            if time.monotonic() >= deadline or checks >= max_edges or attempts >= 32:
                                break
                            attempts += 1
                            if valid(path[index], path[end]):
                                path[index+1:end] = []
                                break
                        index += 1
                    return dict(path=path, planner='rrt_connect', seed=seed,
                                nodes=nodes, collision_edges=checks)
                if nodes >= max_nodes:
                    break
        trees.reverse()
    raise ValueError('Source configuration node budget exhausted; no collision-free route found')
