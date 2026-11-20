"""Source door/dock geometry and fail-closed lane event composition."""
from pathlib import Path
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.fleet_facility_graph import facility_graph
from robot_lab_utils.fleet_building_graph import building_graph
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


def facility_session(members):
    directory = ROOT / 'src/robot_lab_maps/maps/fleet_facility'
    config = yaml.safe_load((directory / 'infrastructure.yaml').read_text())
    session = dict(map='fleet_facility', members=members,
                   floor_maps={floor: str(directory / 'maps' / filename)
                               for floor, filename in [('L0', 'map.yaml'), ('L1', 'map_L1.yaml')]})
    return session, config


def test_building_graph_restricts_each_car_to_gated_landing_lanes():
    session, config = facility_session([dict(radius=.18, id='alpha', spawn=dict(x=2.5, y=0.))])
    building, metadata = building_graph(session, config)
    assert set(building['levels']) == {'L0', 'L1'}
    assert len({v['name'] for v in metadata}) == len(metadata)
    for floor, level in building['levels'].items():
        vertices, lanes = level['vertices'], level['lanes']
        inside = next(i for i, v in enumerate(vertices) if v[2].get('lift'))
        landing = next(i for i, v in enumerate(vertices) if v[2]['name'] == floor + '_landing')
        assert {(a, b) for a, b, props in lanes if inside in (a, b)} == {(landing, inside), (inside, landing)}
        assert all(props.get('orientation_constraint') == 'forward' for a, b, props in lanes if inside in (a, b))
        assert not next(v for v in metadata if v['name'] == vertices[inside][2]['name'])['selectable']
        assert vertices[inside][2]['lift'] == config['lift']['name']
        for a, b, props in lanes:
            if props.get('door_name'):
                assert floor == 'L0' and props['door_name'] == 'main'
                assert abs(vertices[a][0]) >= 1.19 and abs(vertices[b][0]) >= 1.19


def test_molar_keeps_gate_approach_separate_from_the_exit_in_the_shared_graph():
    members = [dict(radius=.18, id='alpha', model='bumperbot', spawn=dict(x=-3., y=0.)),
               dict(radius=.49, id='molar', model='molar_hri', spawn=dict(x=1.5, y=0.))]
    session, config = facility_session(members)
    building, metadata = building_graph(session, config)
    for floor, level in building['levels'].items():
        vertices, lanes = level['vertices'], level['lanes']
        index = {options['name']: i for i, (_, _, options) in enumerate(vertices)}
        approach, car, landing = (index[name] for name in (floor+'_approach', 'lift_'+floor+'_inside', floor+'_landing'))
        assert [vertices[i][:2] for i in (approach, car, landing)] == [[1.75, 0.], [3.95, 0.], [2.1, 0.]]
        assert {(a, b) for a, b, _ in lanes if car in (a, b)} == {(approach, car), (car, landing)}
        if floor == 'L1':
            assert [props for a, b, props in lanes if (a, b) == (landing, approach)] == [
                {'orientation_constraint': 'forward'}]
    assert next(v for v in metadata if v['name'] == 'L0_landing').get('selectable', True)
    assert next(v for v in metadata if v['name'] == 'lift_L0_inside')['exit_heading'] == pytest.approx(3.141593)
    assert next(v for v in metadata if v['name'] == 'L1_approach')['boarding_heading'] == pytest.approx(0.)
    assert config['transport']['boarding']['x'] == 3.65
    assert config['transport']['models']['molar_hri']['landings']['L1']['x'] == 2.1

    config['transport']['models']['bumperbot'] = dict(boarding=dict(x=3.65, y=0., yaw=0.))
    with pytest.raises(ValueError, match='incompatible transport overrides'):
        building_graph(session, config)
