"""Real-world import (R6.8) offline: synthetic elevation and OSM, real pipeline."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

pytest.importorskip('pyproj')
pytest.importorskip('shapely')
from robot_lab_utils import real_world_import as rw
from robot_lab_utils import terrain_generator as tg

SOUTH, WEST, NORTH, EAST = 54.0880, 12.1390, 54.0898, 12.1420     # about 200 x 200 m


def osm_fixture(projection):
    """A 20 x 12 m, 4-level building, a road across the area, one tree and a pond (in metres -> lat/lon)."""
    def node(i, x, y):
        lat, lon = projection.inverse_many(np.array([x]), np.array([y]))
        return dict(type='node', id=i, lat=float(lat[0]), lon=float(lon[0]))
    nodes = [node(1, 20, 10), node(2, 40, 10), node(3, 40, 22), node(4, 20, 22),
             node(5, -90, -40), node(6, 90, -40), node(8, -60, 60), node(9, -40, 60), node(10, -40, 75)]
    tree = dict(node(7, -20, -10), tags={'natural': 'tree', 'height': '10'})
    return dict(elements=nodes + [tree,
        dict(type='way', id=100, nodes=[1, 2, 3, 4, 1], tags={'building': 'yes', 'building:levels': '4'}),
        dict(type='way', id=101, nodes=[5, 6], tags={'highway': 'residential'}),
        dict(type='way', id=102, nodes=[8, 9, 10, 8], tags={'natural': 'water'})])


@pytest.fixture
def offline(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_LAB_RUNTIME_ROOT', str(tmp_path))
    _, projection, _, _ = rw.validate_area(SOUTH, WEST, NORTH, EAST)

    def fake_dem(bounds, projection, size_m, samples, cache=None, progress=print):
        grid = (np.arange(samples) + .5) * size_m / samples - size_m / 2
        gx, gy = np.meshgrid(grid, grid[::-1])
        return 30. + .02 * gx + .01 * gy, dict(zoom=15, tiles=1, source='synthetic')
    monkeypatch.setattr(rw, 'fetch_dem', fake_dem)
    monkeypatch.setattr(rw, 'fetch_osm', lambda bounds, cache=None, progress=print: osm_fixture(projection))
    return tmp_path / 'terrain'


def request(**extra):
    return dict(dict(id='test_place', south=SOUTH, west=WEST, north=NORTH, east=EAST, samples=129), **extra)


def test_area_limits_and_projection_round_trip():
    with pytest.raises(ValueError, match='40'):
        rw.validate_area(54.0, 12.0, 54.0001, 12.0001)
    with pytest.raises(ValueError, match='2000'):
        rw.validate_area(54.0, 12.0, 54.1, 12.1)
    _, projection, width, height = rw.validate_area(SOUTH, WEST, NORTH, EAST)
    assert 150 < width < 250 and 150 < height < 250
    x, y = projection.forward(54.0890, 12.1400)
    lat, lon = projection.inverse_many(np.array([x]), np.array([y]))
    assert lat[0] == pytest.approx(54.0890, abs=1e-9) and lon[0] == pytest.approx(12.1400, abs=1e-9)


def test_import_builds_buildings_roads_trees_water_and_a_free_spawn(offline):
    manifest = rw.generate(request(), progress=lambda _m: None)
    folder = offline / 'test_place'
    counts = manifest['real_world']['counts']
    assert counts == dict(buildings=1, roads=1, trees=1, water=1, truncated=dict(buildings=0, trees=0))
    world = ET.parse(folder / 'worlds' / 'test_place.world').getroot()
    names = [model.get('name') for model in world.iter('model')]
    assert {'terrain', 'building_0001', 'tree_0000', 'roads', 'water'} <= set(names)
    obj = (folder / 'worlds' / 'buildings' / 'b0001.obj').read_text()
    assert any(line.startswith('vn ') for line in obj.splitlines()) and '//' in obj   # normals (Fortress ODE collider)
    roof = manifest['real_world']['roofs'][0]
    assert roof['top_m'] == pytest.approx(max(float(v.split()[3]) for v in obj.splitlines() if v.startswith('v ')))
    assert 11. < roof['top_m'] - manifest['elevation']['min_m'] < 16.                 # 4 levels x 3 m above ground
    pgm = (folder / 'maps' / 'map.pgm').read_bytes().split(b'\n', 3)
    columns, rows = (int(v) for v in pgm[1].split())
    grid = np.frombuffer(pgm[3], dtype=np.uint8).reshape(rows, columns)
    size, resolution = manifest['inputs']['size_m'], manifest['inputs']['occupancy_resolution_m']
    cell = lambda x, y: grid[int((size / 2 - y) / resolution), int((x + size / 2) / resolution)]
    assert cell(30, 16) == tg.OCCUPIED and cell(-50, 67) == tg.OCCUPIED and cell(0, -40) == tg.FREE
    sx, sy = manifest['inputs']['spawn_x'], manifest['inputs']['spawn_y']
    assert cell(sx, sy) == tg.FREE
    sources = manifest['real_world']['sources']
    assert any('OpenStreetMap' in s['attribution'] and 'ODbL' in s['license'] for s in sources)
    assert any('Terrain Tiles' in s['attribution'] for s in sources)


def test_offline_regeneration_is_byte_identical(offline):
    rw.generate(request(), progress=lambda _m: None)
    result = tg.regenerate(offline / 'test_place')                  # delegates to the real-world rebuild
    assert result and all(result.values()), [k for k, v in result.items() if not v]


def test_relief_above_the_limit_is_refused(offline):
    with pytest.raises(ValueError, match='rises'):
        rw.generate(request(max_relief_m=1.), progress=lambda _m: None)


def test_traffic_tracks_become_gazebo_actors(offline, monkeypatch):
    from robot_lab_utils import real_world_traffic as traffic
    _, projection, _, _ = rw.validate_area(SOUTH, WEST, NORTH, EAST)
    lat, lon = projection.inverse_many(np.array([-50., 0., 50.]), np.array([-40., -40., -40.]))
    steps = ''.join('<timestep time="%.1f"><person id="p0" x="%.7f" y="%.7f" angle="90"/>'
                    '<vehicle id="v0" x="%.7f" y="%.7f" angle="90"/></timestep>'
                    % (t, lon[i % 3], lat[i % 3], lon[(i + 1) % 3], lat[(i + 1) % 3]) for i, t in enumerate(range(8)))
    fcd = '<fcd-export>%s</fcd-export>' % steps
    monkeypatch.setattr(traffic, 'run_sumo', lambda osm, spec, work, progress=print: fcd)
    manifest = rw.generate(request(id='test_traffic', people=True, traffic=True), progress=lambda _m: None)
    assert manifest['real_world']['traffic']['people'] == 1 and manifest['real_world']['traffic']['vehicles'] == 1
    world = ET.parse(offline / 'test_traffic' / 'worlds' / 'test_traffic.world').getroot()
    actors = {actor.get('name'): actor for actor in world.iter('actor')}
    assert set(actors) == {'person_01', 'vehicle_01'}
    assert actors['person_01'].find('skin/filename').text == traffic.PERSON_SKIN
    assert len(actors['vehicle_01'].findall('.//waypoint')) >= 4
    assert (offline / 'test_traffic' / 'mjcf' / 'test_traffic.xml').is_file()          # MuJoCo skips actors
    assert all(tg.regenerate(offline / 'test_traffic').values())                        # replays saved tracks
