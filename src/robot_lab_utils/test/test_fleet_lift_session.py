#!/usr/bin/env python3
"""Tests for lift_session_graph: multi-level RMF graph with lift-shaft lanes.

These are unit tests of the graph construction only. They do not start Gazebo,
ROS, or Open-RMF, and do not claim any physical lift mission qualification.
"""
import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_lab_utils'))

from robot_lab_utils.fleet_facility_graph import lift_session_graph


class FakeGrid:
    """Minimal occupancy grid that reports everything clear within bounds."""
    def __init__(self, resolution=0.1):
        self.resolution = resolution

    def clear(self, x, y, radius):
        return math.isfinite(x) and math.isfinite(y)


class TestLiftSessionGraph(unittest.TestCase):
    def setUp(self):
        self.config = {
            'lift': {
                'floors': {'L0': 0.0, 'L1': 3.0},
                'cabin': [3.0, 4.0, -0.5, 0.5],
                'landings': {'L0': [3.5, 0.0], 'L1': [3.5, 0.0]},
                'joint': 'lift_joint',
            },
            'doors': {},
            'docks': {},
        }
        self.members = [
            {'id': 'alpha', 'model': 'bumperbot', 'radius': 0.35},
            {'id': 'beta', 'model': 'labbot', 'radius': 0.45},
        ]
        self.grid = FakeGrid()
        self.base_data = {
            'vertices': [
                {'name': 'p0', 'x': 0.0, 'y': 0.0, 'holding': True},
                {'name': 'p1', 'x': 2.0, 'y': 0.0, 'holding': True},
                {'name': 'p2', 'x': 5.0, 'y': 0.0, 'holding': False},
            ],
            'lanes': [[0, 1], [1, 2]],
            'directed_lanes': [
                {'entry': 0, 'exit': 1, 'entry_event': None, 'exit_event': None},
                {'entry': 1, 'exit': 0, 'entry_event': None, 'exit_event': None},
                {'entry': 1, 'exit': 2, 'entry_event': None, 'exit_event': None},
                {'entry': 2, 'exit': 1, 'entry_event': None, 'exit_event': None},
            ],
        }

    def test_returns_unchanged_without_lift_config(self):
        config_no_lift = {'doors': {}, 'docks': {}}
        result = lift_session_graph(self.base_data, self.grid, self.members, config_no_lift)
        self.assertIs(result, self.base_data)
        self.assertNotIn('lift_vertices', result)

    def test_returns_unchanged_with_single_floor(self):
        config_single = dict(self.config)
        config_single['lift'] = dict(self.config['lift'], floors={'L0': 0.0})
        result = lift_session_graph(self.base_data, self.grid, self.members, config_single)
        self.assertIs(result, self.base_data)

    def test_creates_per_floor_vertex_copies(self):
        result = lift_session_graph(self.base_data, self.grid, self.members, self.config)
        self.assertIn('lift_vertices', result)
        self.assertIn('L0', result['lift_vertices'])
        self.assertIn('L1', result['lift_vertices'])
        self.assertEqual(len(result['lift_vertices']['L0']), 3)
        self.assertEqual(len(result['lift_vertices']['L1']), 3)
        l0_names = {v['name'] for v in result['lift_vertices']['L0']}
        l1_names = {v['name'] for v in result['lift_vertices']['L1']}
        self.assertEqual(l0_names, {'p0@L0', 'p1@L0', 'p2@L0'})
        self.assertEqual(l1_names, {'p0@L1', 'p1@L1', 'p2@L1'})
        self.assertEqual(result['lift_vertices']['L0'][0]['floor'], 'L0')
        self.assertEqual(result['lift_vertices']['L1'][0]['floor'], 'L1')
        self.assertEqual(result['lift_vertices']['L0'][0]['x'], result['lift_vertices']['L1'][0]['x'])
        self.assertEqual(result['lift_vertices']['L0'][0]['y'], result['lift_vertices']['L1'][0]['y'])

    def test_creates_lift_shaft_lanes_between_adjacent_floors(self):
        result = lift_session_graph(self.base_data, self.grid, self.members, self.config)
        self.assertIn('lift_lanes', result)
        self.assertEqual(len(result['lift_lanes']), 2)
        fwd = result['lift_lanes'][0]
        rev = result['lift_lanes'][1]
        self.assertEqual(fwd['floors'], ('L0', 'L1'))
        self.assertEqual(rev['floors'], ('L1', 'L0'))
        self.assertTrue(fwd['entry'].endswith('@L0'))
        self.assertTrue(fwd['exit'].endswith('@L1'))
        self.assertTrue(rev['entry'].endswith('@L1'))
        self.assertTrue(rev['exit'].endswith('@L0'))

    def test_lift_lanes_have_door_events(self):
        result = lift_session_graph(self.base_data, self.grid, self.members, self.config)
        for lane in result['lift_lanes']:
            self.assertEqual(lane['entry_event']['kind'], 'lift_door_open')
            self.assertEqual(lane['exit_event']['kind'], 'lift_door_close')
            self.assertTrue(lane['entry_event']['name'].startswith('lift_'))
            self.assertTrue(lane['exit_event']['name'].startswith('lift_'))

    def test_lift_directed_lanes_appended(self):
        result = lift_session_graph(self.base_data, self.grid, self.members, self.config)
        self.assertIn('lift_directed', result)
        self.assertEqual(len(result['lift_directed']), 2)
        self.assertEqual(len(result['directed_lanes']), 4 + 2)

    def test_scope_updated(self):
        result = lift_session_graph(self.base_data, self.grid, self.members, self.config)
        self.assertIn('scope', result)
        self.assertIn('Multi-level RMF graph', result['scope'])
        self.assertIn('level-aware dispatch', result['scope'])

    def test_landing_missing_floor_skips_shaft(self):
        config_partial = dict(self.config)
        config_partial['lift'] = dict(self.config['lift'])
        config_partial['lift']['landings'] = {'L0': [3.5, 0.0]}
        result = lift_session_graph(self.base_data, self.grid, self.members, config_partial)
        self.assertEqual(result.get('lift_lanes', []), [])

    def test_invalid_cabin_raises(self):
        config_bad = dict(self.config)
        config_bad['lift'] = dict(self.config['lift'], cabin=[3.0, 4.0, -0.5])
        with self.assertRaises(ValueError):
            lift_session_graph(self.base_data, self.grid, self.members, config_bad)

    def test_invalid_landings_raises(self):
        config_bad = dict(self.config)
        config_bad['lift'] = dict(self.config['lift'], landings={})
        with self.assertRaises(ValueError):
            lift_session_graph(self.base_data, self.grid, self.members, config_bad)

    def test_shaft_clearance_check(self):
        class BlockedGrid(FakeGrid):
            def clear(self, x, y, radius):
                if 3.0 <= x <= 4.5 and -1.0 <= y <= 1.0:
                    return False
                return True
        with self.assertRaises(ValueError):
            lift_session_graph(self.base_data, BlockedGrid(), self.members, self.config)


if __name__ == '__main__':
    unittest.main()