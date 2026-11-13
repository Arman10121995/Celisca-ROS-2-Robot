"""Independent route, directed-edge, reproducibility and budget contracts."""
import time

import numpy as np
import pytest

from robot_lab_utils.bounded_control_planner import connect_controls


def plan(edge, **kwargs):
    return connect_controls([.1,.2],[.9,.2],[[0.,1.],[0.,1.]],edge,
                            deadline=time.monotonic()+2.,**kwargs)


def barrier(a,b):
    points=np.linspace(a,b,101)
    return not np.any((points[:,0]>.42)&(points[:,0]<.58)&(points[:,1]<.65))


def test_detour_crosses_only_through_real_open_region_and_repeats_from_seed():
    first=plan(barrier,seed=9)
    second=plan(barrier,seed=9)
    assert first['planner']=='rrt_connect'
    np.testing.assert_array_equal(first['path'],second['path'])
    np.testing.assert_array_equal(first['path'][0],[.1,.2])
    np.testing.assert_array_equal(first['path'][-1],[.9,.2])
    assert max(p[1] for p in first['path'])>.65
    assert all(barrier(a,b) for a,b in zip(first['path'],first['path'][1:]))


def test_goal_tree_checks_actual_forward_edge_direction():
    # A collision model with irreversible control motion exercises goal-tree
    # child->parent validation; a reversed-edge assumption loses this route.
    def forward_only(a,b):
        return b[0]>=a[0]-1e-12 and barrier(a,b)
    route=plan(forward_only,seed=3,max_nodes=1000)
    assert route['planner']=='rrt_connect'
    assert all(forward_only(a,b) for a,b in zip(route['path'],route['path'][1:]))


def test_direct_path_and_blocked_goal_are_distinct():
    assert plan(lambda a,b:True)['planner']=='direct'
    with pytest.raises(ValueError,match='collision at planning'):
        plan(lambda a,b:b[0]<.8)


def test_disconnected_space_cannot_return_a_fabricated_route():
    def wall(a,b):
        return not (min(a[0],b[0])<=.5<=max(a[0],b[0]))
    with pytest.raises(ValueError,match='budget'):
        plan(wall,max_nodes=12)


def test_expired_deadline_and_invalid_control_bounds_are_refused():
    with pytest.raises(ValueError,match='deadline'):
        connect_controls([0.],[1.],[[0.,1.]],lambda a,b:True,deadline=0.)
    with pytest.raises(ValueError,match='limits'):
        connect_controls([0.],[2.],[[0.,1.]],lambda a,b:True,deadline=time.monotonic()+1.)
