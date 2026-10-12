"""Grid planners and Nav2 action adapters (R7.6/R7.7).

The robot_lab node planners now drive through Nav2's actions; these checks
pin the algorithms (collision-free, start/goal exact, failure when there is
no path) and the control laws without a ROS graph.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_lab_algorithms import grid_planners as gp  # noqa: E402
from robot_lab_algorithms import nav2_adapters as na  # noqa: E402


def two_rooms(door=True):
    """10 x 6 m, 0.05 m cells: a wall at x = 5 m with a 1 m door."""
    data = np.zeros((120, 200), dtype=np.int16)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
    data[:, 99:101] = 100
    if door:
        data[50:70, 99:101] = 0
    return gp.GridView(data, (0.0, 0.0), 0.05)


START, GOAL = (1.0, 1.0), (9.0, 5.0)


def collision_free(view, path):
    return all(view.segment_free(a, b) for a, b in zip(path[:-1], path[1:]))


@pytest.mark.parametrize('name', sorted(gp.PLANNERS))
def test_each_planner_finds_a_collision_free_path_through_the_door(name):
    view = two_rooms()
    planner = gp.PLANNERS[name]
    path = planner(view, START, GOAL) if name == 'voronoi_planner' else \
        planner(view, START, GOAL, np.random.default_rng(1))
    assert path, name
    assert path[0] == pytest.approx(START) and path[-1] == pytest.approx(GOAL)
    assert collision_free(view, path)
    # Through the door: some waypoint lies in the doorway band.
    dense = gp.densify(path, 0.05)
    crossing = [p for p in dense if 4.95 <= p[0] <= 5.05]
    assert crossing and all(2.5 <= p[1] <= 3.5 for p in crossing)
    straight = math.hypot(GOAL[0] - START[0], GOAL[1] - START[1])
    assert straight <= gp.path_length(path) <= 2.5 * straight


@pytest.mark.parametrize('name', sorted(gp.PLANNERS))
def test_planners_report_no_path_when_the_rooms_are_closed(name):
    view = two_rooms(door=False)
    planner = gp.PLANNERS[name]
    kwargs = {} if name == 'voronoi_planner' else {'rng': np.random.default_rng(1)}
    if name in ('rrt_planner', 'rrt_star_planner'):
        kwargs['time_budget'] = 0.3
    assert planner(view, START, GOAL, **kwargs) == []


def test_voronoi_path_keeps_more_clearance_than_the_shortest_sampled_path():
    view = two_rooms()
    _, clearance = gp.voronoi_diagram(view)

    def mean_clearance(path):
        dense = np.asarray(gp.densify(path, 0.05))
        rows, cols = view.cells(dense)
        return float(np.mean(clearance[rows, cols]))
    voronoi = gp.voronoi(view, START, GOAL)
    rrt_star = gp.rrt_star(view, START, GOAL, np.random.default_rng(1))
    assert mean_clearance(voronoi) > mean_clearance(rrt_star)


def test_blocked_cells_follow_the_nav2_costmap_translation():
    view = gp.GridView(np.array([[0, 98, 99, 100, -1]], dtype=np.int16), (0, 0), 1.0)
    assert view.free.tolist() == [[True, True, False, False, True]]
    strict = gp.GridView(np.array([[-1]], dtype=np.int16), (0, 0), 1.0, allow_unknown=False)
    assert not strict.free.any()


def test_path_poses_carry_segment_headings_and_the_goal_heading():
    poses = na.path_poses([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], goal_yaw=0.3, spacing=0.5)
    assert poses[0][2] == pytest.approx(0.0)
    assert poses[-2][2] == pytest.approx(math.pi / 2)
    assert poses[-1][:2] == pytest.approx((1.0, 1.0)) and poses[-1][2] == pytest.approx(0.3)


def test_nearest_free_moves_a_start_out_of_the_inflated_zone():
    view = two_rooms()
    inside_wall = (5.0, 1.0)
    moved = na.nearest_free(view, inside_wall)
    assert moved is not None and view.is_free(*moved)
    assert math.hypot(moved[0] - 5.0, moved[1] - 1.0) <= 0.2


LIMITS = na.TrackingLimits(max_linear=0.3, max_angular=1.0, lookahead=0.6, robot_radius=0.12)


def test_pure_pursuit_drives_straight_turns_and_rotates_in_place():
    v, w = na.pure_pursuit((0, 0, 0), (0.6, 0.0), 3.0, LIMITS)
    assert v == pytest.approx(0.3) and w == pytest.approx(0.0)
    v, w = na.pure_pursuit((0, 0, 0), (0.5, 0.3), 3.0, LIMITS)
    assert 0 < v < 0.3 and w > 0
    v, w = na.pure_pursuit((0, 0, 0), (-0.6, 0.1), 3.0, LIMITS)
    assert v == 0.0 and w > 0
    # Slows down on the approach.
    assert na.pure_pursuit((0, 0, 0), (0.6, 0.0), 0.2, LIMITS)[0] < 0.3


def test_follow_the_gap_steers_around_a_wall_toward_the_open_side():
    angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    ranges = np.full(360, 5.0)
    ranges[(angles > -0.2) & (angles < 0.6)] = 0.5      # wall ahead and to the left
    scan = (float(angles[0]), float(angles[1] - angles[0]), ranges.tolist())
    v, w = na.follow_the_gap((0, 0, 0), (3.0, 0.0), 3.0, LIMITS, scan)
    assert w < 0                                          # around the right side


def test_blocked_ahead_sees_an_obstacle_in_the_swept_path_only():
    angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    ranges = np.full(360, 5.0)
    ranges[np.abs(angles) < 0.05] = 0.2
    scan = (float(angles[0]), float(angles[1] - angles[0]), ranges.tolist())
    assert na.blocked_ahead(scan, 0.3, 0.0, LIMITS)
    assert not na.blocked_ahead(scan, 0.0, 0.5, LIMITS)     # rotating in place
    clear = (scan[0], scan[1], np.full(360, 5.0).tolist())
    assert not na.blocked_ahead(clear, 0.3, 0.0, LIMITS)


def test_lookahead_point_advances_along_the_path():
    path = [(x * 0.1, 0.0, 0.0) for x in range(30)]
    index, target = na.lookahead_point(path, (1.0, 0.05, 0.0), 0, 0.6)
    assert index == 10 and target[0] == pytest.approx(1.6, abs=0.06)
