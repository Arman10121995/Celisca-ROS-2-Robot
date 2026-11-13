"""Generated terrain (R6.7): determinism, conventions, occupancy, registry."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from robot_lab_utils import terrain_generator as t


@pytest.fixture
def root(tmp_path, monkeypatch):
    monkeypatch.setenv('ROBOT_LAB_RUNTIME_ROOT', str(tmp_path))
    return tmp_path / 'terrain'


def spec(**extra):
    return dict(dict(id='test_hills', seed=3, size_m=20., samples=65, relief_m=1.5), **extra)


def test_same_inputs_give_identical_bytes_and_regenerate_confirms_it(root):
    first, manifest, _ = t.build(spec())
    second, _, _ = t.build(spec())
    assert first == second
    assert t.build(spec(seed=4))[0]['worlds/test_hills.png'] != first['worlds/test_hills.png']
    manifest = t.generate(spec())
    folder = root / 'test_hills'
    assert set(manifest['files']) == {'worlds/test_hills.world', 'worlds/test_hills.png',
                                      'worlds/test_hills_visual.obj', 'maps/map.pgm', 'maps/map.yaml',
                                      'preview.png', 'mjcf/test_hills.xml'}
    assert all(t.regenerate(folder).values())
    (folder / 'maps' / 'map.yaml').write_text('edited\n')
    with pytest.raises(ValueError):          # an existing folder is never overwritten silently
        t.generate(spec())
    manifest_copy = json.loads((folder / 'manifest.json').read_text())
    assert manifest_copy['files']['maps/map.yaml'] == manifest['files']['maps/map.yaml']


def test_raster_follows_the_measured_gazebo_convention():
    outputs, manifest, heights = t.build(spec(spawn_pad_m=0.))
    from PIL import Image
    import io
    raster = np.asarray(Image.open(io.BytesIO(outputs['worlds/test_hills.png'])), dtype=float)
    assert raster.max() == 255 and raster.min() == 0           # normalised by the raster maximum
    assert np.allclose(heights, raster * manifest['elevation']['max_m'] / 255.)
    world = ET.fromstring(outputs['worlds/test_hills.world'])
    size = [float(v) for v in world.find('.//collision//heightmap/size').text.split()]
    assert size == pytest.approx([20., 20., manifest['elevation']['max_m']])
    assert world.find('.//visual//mesh/uri').text == 'test_hills_visual.obj'
    # The visual mesh is the shared converter's grid: same heights, row 0 at +y.
    vertices = np.array([[float(v) for v in line.split()[1:]] for line in
                         outputs['worlds/test_hills_visual.obj'].decode().splitlines() if line.startswith('v ')])
    assert len(vertices) == 65 * 65
    assert vertices[0, 1] > 0 and vertices[0, 0] < 0
    assert vertices[0, 2] == pytest.approx(heights[0, 0], abs=1e-4)


def test_spawn_pad_is_flat_and_spawn_height_is_on_the_surface():
    outputs, manifest, heights = t.build(spec(spawn_x=2., spawn_y=-3., spawn_pad_m=1.5))
    grid_x, grid_y = t.sample_coordinates(t.validate(spec(spawn_x=2., spawn_y=-3.)))
    pad = np.hypot(grid_x - 2., grid_y + 3.) <= 1.5
    assert np.ptp(heights[pad]) <= manifest['raster']['quantisation_m'] + 1e-9
    assert manifest['spawn']['z'] == pytest.approx(manifest['elevation']['spawn_m'] + .1)


def test_occupancy_blocks_slopes_steps_and_the_edge():
    validated = t.validate(spec())
    flat = np.zeros((65, 65))
    flat[:, 40:] = .5                                              # a 0.5 m cliff
    pgm, map_yaml, stats, blocked = t.occupancy(validated, flat)
    assert blocked[:, 39:41].all() and not blocked[:, :30].any()
    grid = np.frombuffer(pgm.split(b'\n', 3)[3], dtype=np.uint8).reshape(200, 200)
    assert (grid[:, 0] == t.OCCUPIED).all() and (grid[0, :] == t.OCCUPIED).all()
    cliff_column = int((40 * 20. / 65) / .1)
    assert (grid[5:-5, cliff_column - 2:cliff_column + 2] == t.OCCUPIED).any(axis=1).all()
    assert (grid[50:150, 20:80] == t.FREE).all()
    assert 'origin: [-10.000, -10.000' in map_yaml


@pytest.mark.parametrize('bad', [dict(id='Bad Name'), dict(samples=100), dict(relief_m=0.), dict(relief_m=float('nan')),
                                 dict(spawn_x=9.5), dict(seed=-1), dict(seed=True), dict(provider='web'),
                                 dict(min_wavelength_m=.1), dict(smoothness=9.)])
def test_invalid_specs_are_refused(bad):
    with pytest.raises(ValueError):
        t.validate(spec(**bad))


def test_imported_raster_needs_terms_and_keeps_its_provenance(tmp_path, root):
    x, y = np.meshgrid(np.linspace(0, 1, 300), np.linspace(0, 1, 300))
    dem = 120. + 30. * np.sin(6 * x) * np.cos(4 * y)               # metres above some datum
    path = tmp_path / 'dem.npy'
    np.save(path, dem)
    raster = dict(id='test_dem', provider='raster', raster_path=str(path), source_metres_per_pixel=.5,
                  size_m=100., samples=65, relief_m=50.)
    with pytest.raises(ValueError, match='attribution'):
        t.validate(raster)
    raster.update(attribution='Test DEM', license='CC-BY-4.0')
    manifest = t.generate(raster)
    assert manifest['provider']['attribution'] == 'Test DEM' and manifest['provider']['license'] == 'CC-BY-4.0'
    assert manifest['provider']['source_sha256'] and manifest['raster']['metres_per_pixel'] == pytest.approx(100. / 65)
    assert 30. < manifest['elevation']['max_m'] <= 60.             # the raster's own relief, not rescaled
    assert all(t.regenerate(root / 'test_dem').values())
    np.save(path, dem + 1.)
    with pytest.raises(ValueError, match='changed'):
        t.regenerate(root / 'test_dem')
    with pytest.raises(ValueError, match='covers'):               # 300 px x 0.5 m cannot cover 200 m
        t.build(dict(raster, id='too_big', size_m=200.))


def test_registered_terrain_is_an_installed_map_and_cannot_shadow_one(root):
    from robot_lab_utils.installed_assets import installed_profiles
    t.generate(spec())
    profile = installed_profiles('maps')['test_hills']
    assert Path(profile['gazebo']['world_path']).is_file() and profile['map']['has_2d_map']
    assert float(profile['spawn']['z']) > 0
    with pytest.raises(ValueError, match='already a map id'):
        t.generate(spec(id='nav_empty'))
    assert t.unregister('test_hills') and 'test_hills' not in installed_profiles('maps')
    assert (root / 'test_hills' / 'manifest.json').is_file()       # files stay


def test_mujoco_scene_sits_beside_the_world_where_the_launch_looks(root):
    t.generate(spec())
    folder = root / 'test_hills'
    world_path = folder / 'worlds' / 'test_hills.world'
    # mujoco_simulator.launch.py: dirname(dirname(world_path))/mjcf/<world_name>.xml
    mjcf = world_path.parent.parent / 'mjcf' / 'test_hills.xml'
    scene = ET.parse(mjcf).getroot()
    hfield = scene.find('asset/hfield')
    assert Path(hfield.get('file')).is_file()
    radius_x, radius_y, elevation, _ = (float(v) for v in hfield.get('size').split())
    assert (radius_x, radius_y) == (10., 10.) and elevation == pytest.approx(
        json.loads((folder / 'manifest.json').read_text())['elevation']['max_m'])


def test_registered_terrain_has_a_valid_registry_environment(root):
    from robot_lab_utils.installed_assets import installed_registry_entities
    t.generate(spec())
    entity = next(e for e in installed_registry_entities('maps') if e['id'] == 'test_hills')
    assert Path(entity['world_file']).is_file() and entity['dimension'] == '3D'
    from robot_lab_registry.schemas import validate_entity
    valid, errors = validate_entity('environment', entity)
    assert valid, errors


@pytest.mark.parametrize('style', t.STYLES)
def test_every_style_is_deterministic_and_distinct(style):
    first = t.build(spec(style=style))[0]['worlds/test_hills.png']
    assert first == t.build(spec(style=style))[0]['worlds/test_hills.png']
    others = [t.build(spec(style=other))[0]['worlds/test_hills.png'] for other in t.STYLES if other != style]
    assert first not in others


def test_crater_has_a_rim_above_its_bowl():
    _, manifest, heights = t.build(spec(style='crater', roughness=0., spawn_x=8., spawn_y=8., spawn_pad_m=.5))
    centre = heights[32, 32]
    rim = heights[32, int(32 + .55 * 32)]
    assert rim > centre + .5 * manifest['elevation']['max_m']


def test_command_line_generates_registers_and_checks(root, capsys):
    assert t.main(['cli_ridges', '--style', 'ridges', '--samples', '33', '--size-m', '10', '--relief-m', '1']) == 0
    assert json.loads(capsys.readouterr().out)['id'] == 'cli_ridges'
    assert 'cli_ridges' in t.registered_profiles()
    assert t.main(['cli_ridges', '--check']) == 0
    with pytest.raises(ValueError):
        t.main(['cli_ridges', '--samples', '33'])                  # exists; needs --replace


def test_spec_file_mode_reports_json_for_the_gui(root, tmp_path, capsys):
    path = tmp_path / 'spec.json'
    path.write_text(json.dumps(spec(id='spec_hills', samples=33)))
    assert t.main(['--spec', str(path)]) == 0
    assert json.loads(capsys.readouterr().out)['manifest']['id'] == 'spec_hills'
    path.write_text(json.dumps(spec(id='Bad Name')))
    assert t.main(['--spec', str(path)]) == 2
    assert 'id:' in json.loads(capsys.readouterr().out)['error']
