"""Bounded bidirectional RRT-Connect over finite source control coordinates.

The caller supplies directed collision validation including its actual command
interpolation. This is a local implementation of the published algorithm, not
an OMPL binding or a guarantee of completeness within the declared budgets.
"""
import time

import numpy as np


class PlanningBudgetExceeded(ValueError):
    """A finite planner or caller-owned collision inspection budget expired."""


def connect_controls(start, goal, limits, edge_valid, *, deadline, seed=0,
                     max_nodes=384, max_edges=2048, extension=.18):
    """Connect to one goal vector or any of at most eight explicit goal rows."""
    start, goal, limits = (np.asarray(value, dtype=float) for value in (start, goal, limits))
    goals=goal[None,:] if goal.ndim==1 else goal
    if (start.ndim != 1 or not 1 <= len(start) <= 12
            or goals.ndim!=2 or goals.shape[1:]!=start.shape or not 1<=len(goals)<=8
            or limits.shape != (len(start), 2) or not all(np.all(np.isfinite(v)) for v in (start, goal, limits))
            or np.any(limits[:, 1] <= limits[:, 0]) or not np.isfinite(deadline)
            or not np.isfinite(extension) or extension <= 0 or max_nodes < 2 or max_edges < 1):
        raise ValueError('Finite bounded control-space planning inputs required')
    lower, span = limits[:, 0], limits[:, 1]-limits[:, 0]
    if any(np.any(v < lower) or np.any(v > lower+span) for v in (start, goals)):
        raise ValueError('Planning endpoint exceeds source control limits')
    rng = np.random.default_rng(seed)
    checks, nodes = 0, 1

    def budget():
        if time.monotonic() >= deadline:
            raise PlanningBudgetExceeded('Source configuration planning deadline exceeded')
        if checks >= max_edges:
            raise PlanningBudgetExceeded('Source configuration collision-edge budget exhausted')

    def valid(a, b):
        nonlocal checks
        budget()
        checks += 1
        return bool(edge_valid(a, b))

    if not valid(start,start):
        raise ValueError('Source collision at planning start; choose another approach')
    accepted=[index for index,target in enumerate(goals) if valid(target,target)]
    if not accepted:
        raise ValueError('Source collision at planning goals; choose another approach')
    nodes+=len(accepted)
    if nodes>max_nodes:
        raise ValueError('Source configuration goal roots exceed the node budget')
    for index in accepted:
        if valid(start,goals[index]):
            return dict(path=[start.copy(),goals[index].copy()],planner='direct',seed=seed,
                        nodes=nodes,collision_edges=checks,goal_index=index,goal_count=len(accepted))

    # Each tree records whether it grows from the start. Goal-tree edges must
    # be checked child -> parent because actuator slew need not be reversible.
    trees = [dict(points=[start.copy()], parents=[-1], start=True,roots=1),
             dict(points=[goals[index].copy() for index in accepted],parents=[-1]*len(accepted),
                  start=False,roots=len(accepted))]

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
        if rng.random()<.15:
            root=int(rng.integers(other['roots'])) if other['roots']>1 else 0
            sample=other['points'][root]
        else:
            sample=lower+rng.random(len(start))*span
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
                            try:
                                shortcut=valid(path[index],path[end])
                            except PlanningBudgetExceeded:
                                attempts=32
                                break
                            if shortcut:
                                path[index+1:end] = []
                                break
                        index += 1
                    return dict(path=path, planner='rrt_connect', seed=seed,
                                nodes=nodes, collision_edges=checks,
                                goal_index=next(i for i in accepted if np.array_equal(path[-1],goals[i])),
                                goal_count=len(accepted))
                if nodes >= max_nodes:
                    break
        trees.reverse()
    raise ValueError('Source configuration node budget exhausted; no collision-free route found')
