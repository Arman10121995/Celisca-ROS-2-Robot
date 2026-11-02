"""Source door/dock geometry and fail-closed lane event composition."""
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.fleet_facility_graph import facility_graph
from robot_lab_utils.fleet_missions import rmf_navigation_graph
from robot_lab_utils.fleet_session import SpawnGrid


def test_real_facility_graph_has_reachable_dock_and_both_door_directions():
    directory = ROOT / 'src/robot_lab_maps/maps/fleet_facility'
    grid = SpawnGrid(directory / 'maps/map.yaml')
    config = yaml.safe_load((directory / 'infrastructure.yaml').read_text())
    members = [dict(radius=.18, id='alpha', spawn=dict(x=-4.5, y=-3.))]
    data = facility_graph(rmf_navigation_graph(grid, members), grid, members, config)
    index = next(i for i, v in enumerate(data['vertices']) if v['name'] == 'west_charger')
    arrivals = [lane for lane in data['directed_lanes'] if lane['exit'] == index]
    assert arrivals and all(lane['exit_event'] == dict(kind='dock', name='west_charger') for lane in arrivals)
    openings = [lane for lane in data['directed_lanes'] if lane['entry_event']]
    assert openings
    for lane in openings:
        assert lane['entry_event']['name'] == 'main'  # Never infer lift sessions.
    assert any(lane['exit_event'] == dict(kind='door_close', name='main') for lane in openings)
    assert all(vertex['holding'] for vertex in data['vertices'] if vertex['name'].startswith('door_main_'))
    center = dict(vertices=[dict(name='inside', x=0., y=0.), dict(name='outside', x=1.2, y=0.)], lanes=[[0, 1]])
    guarded = facility_graph(center, grid, members, dict(doors={'main': config['doors']['main']}, docks={}))
    assert guarded['vertices'][0]['holding'] is False


def test_invalid_dock_does_not_create_an_unreachable_success_anchor():
    class Grid:
        def clear(self, *args):
            return False
    config = dict(docks={'bad': dict(x=0., y=0., yaw=0.)})
    with pytest.raises(ValueError, match='clearance'):
        facility_graph(dict(vertices=[], lanes=[]), Grid(), [dict(radius=.1)], config)
