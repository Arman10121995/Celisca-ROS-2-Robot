"""R6.1: independently exercise geometry, map corruption and complete routes."""
import copy
import math
from pathlib import Path
import shutil
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from arena_clearance import Box2D, reference_route, segment_clearance
from validate_nav_arenas import (
    ARENAS, MAPS_DIR, main, map_alignment_errors, map_route_clearance,
    parse_world_boxes, read_pgm, validate_arena,
)


@pytest.mark.parametrize('start,end,distance', [
    ((-2, 0), (2, 0), 0),  # Free endpoints, collision between them.
    ((-2, .6), (2, .6), .35),
    ((-2, .25), (2, .25), 0),
    ((0, 0), (0, 0), 0),
    ((2, 0), (2, 0), 1.5),
    ((1.5, 1.25), (2, 2), math.sqrt(2)),
])
def test_exact_rectangle_segment_distance(start, end, distance):
    assert Box2D('box', 0, 0, .5, .25).segment_distance(start, end) == pytest.approx(distance)


def test_footprint_collides_even_when_center_line_is_clear():
    box = Box2D('box', 0, 0, .5, .25)
    assert box.segment_distance((-2, .35), (2, .35)) > 0
    assert segment_clearance([box], (-2, .35), (2, .35), .14) == pytest.approx(-.04)
    assert segment_clearance([box], (-2, .39), (2, .39), .14) == pytest.approx(0)


def test_rotated_rectangle():
    box = Box2D('rotated', 1, 2, .5, .25, math.pi / 2)
    assert box.segment_distance((1, 0), (1, 4)) == 0
    assert box.segment_distance((1.6, 0), (1.6, 4)) == pytest.approx(.35)
    assert box.point_distance((1, 2)) == pytest.approx(-.25)


def test_reference_route_avoids_wall_and_is_deterministic():
    boxes = [Box2D('wall', 0, 0, .2, 1)]
    args = boxes, (-2, 0), (2, 0), ((-3, -3), (3, 3))
    route = reference_route(*args)
    assert route == reference_route(*args)
    assert len(route) > 2
    assert route[0] == (-2, 0) and route[-1] == (2, 0)
    assert all(segment_clearance(boxes, a, b, .14) >= .2
               for a, b in zip(route, route[1:]))
    assert sum(math.dist(a, b) for a, b in zip(route, route[1:])) > 4


def test_reference_route_rejects_invalid_start_and_disconnected_goal():
    boxes = [Box2D('wall', 0, 0, .2, 4)]
    with pytest.raises(ValueError, match='spawn or goal'):
        reference_route(boxes, (0, 0), (2, 0), ((-3, -3), (3, 3)))
    with pytest.raises(ValueError, match='no reference route'):
        reference_route(boxes, (-2, 0), (2, 0), ((-3, -3), (3, 3)))


def test_pgm_preserves_whitespace_pixels_and_comments(tmp_path):
    path = tmp_path / 'map.pgm'
    body = bytes([10, 13, 32, 9])
    path.write_bytes(b'P5\n# comment\n2 2\n255\n' + body)
    assert read_pgm(path) == (2, 2, 255, bytearray(body))


@pytest.mark.parametrize('payload', [b'P5\n', b'P5\n2 2\n255\n\x00', b'P5\n2 2\n65535\n0000'])
def test_pgm_rejects_invalid_or_truncated_data(tmp_path, payload):
    path = tmp_path / 'bad.pgm'
    path.write_bytes(payload)
    with pytest.raises(ValueError):
        read_pgm(path)


@pytest.fixture
def copied_map(tmp_path):
    shutil.copytree(MAPS_DIR / 'nav_obstacle' / 'maps', tmp_path / 'maps')
    boxes = parse_world_boxes(MAPS_DIR / 'nav_obstacle/worlds/nav_obstacle.world')
    return boxes, tmp_path / 'maps/map.yaml'


def test_map_checks_obstacle_interior_away_from_center(copied_map):
    boxes, metadata = copied_map
    assert map_alignment_errors(boxes, metadata) == []
    config = yaml.safe_load(metadata.read_text())
    image = metadata.parent / config['image']
    cols, rows, _, body = read_pgm(image)
    # Cut a hole in the central box at x=0.8, y=0.2, preserving its center.
    column = int((.8 - config['origin'][0]) / config['resolution'])
    row = rows - 1 - int((.2 - config['origin'][1]) / config['resolution'])
    assert body[row * cols + column] == 0
    body[row * cols + column] = 254
    image.write_bytes(f'P5\n{cols} {rows}\n255\n'.encode() + body)
    assert any('interior pixels' in error for error in map_alignment_errors(boxes, metadata))


@pytest.mark.parametrize('field,value', [('origin', [-9.0, -9.5, 0]), ('resolution', .06)])
def test_map_uses_actual_origin_and_resolution(copied_map, field, value):
    boxes, metadata = copied_map
    config = yaml.safe_load(metadata.read_text())
    config[field] = value
    metadata.write_text(yaml.safe_dump(config))
    assert map_alignment_errors(boxes, metadata)


def test_map_clearance_checks_whole_route_and_map_boundary(copied_map):
    _, metadata = copied_map
    assert map_route_clearance(metadata, [(-1.5, 0), (1.5, 0)], .14) < 0
    assert map_route_clearance(metadata, [(-20, 0), (-19, 0)], .14) < 0


@pytest.mark.parametrize('name', list(ARENAS))
def test_checked_in_arena_has_clear_spawn_goal_and_swept_route(name):
    report = validate_arena(name)
    assert report['valid'], report['errors']
    assert all(path['minimum_clearance_m'] >= .2 for path in report['paths'])
    assert all(path['map_clearance_lower_bound_m'] > 0 for path in report['paths'])


def test_valid_waypoints_do_not_mask_colliding_segment():
    metadata = yaml.safe_load((MAPS_DIR.parent / 'config/arena_navigation.yaml').read_text())['arenas']
    metadata = copy.deepcopy(metadata)
    metadata['nav_obstacle']['reference_paths'][0]['waypoints'] = [
        {'x': -7, 'y': -7, 'yaw': 0}, {'x': 7, 'y': 7, 'yaw': 0}]
    report = validate_arena('nav_obstacle', navigation=metadata)
    assert not report['valid']
    assert any('segment 0' in error for error in report['errors'])


def test_larger_robot_does_not_inherit_bumperbot_qualification():
    assert not validate_arena('nav_narrow_passage', radius=.8)['valid']


def test_missing_paths_and_invalid_radius_fail_closed():
    metadata = {'nav_empty': {'goals': [], 'reference_paths': []}}
    assert not validate_arena('nav_empty', navigation=metadata)['valid']
    with pytest.raises(ValueError, match='radius'):
        validate_arena('nav_empty', radius=math.nan)


def test_cli_records_radius_specific_failure(tmp_path):
    output = tmp_path / 'report.json'
    assert main(['--arena', 'nav_narrow_passage', '--radius', '.8', '--json', str(output)]) == 1
    assert output.is_file()
