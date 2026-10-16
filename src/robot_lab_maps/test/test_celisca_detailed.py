"""Detailed floor geometry/registration guards; no runtime qualification."""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[3]
MAPS = ROOT / 'src/robot_lab_maps/maps'
spec = importlib.util.spec_from_file_location('provision_celisca_detailed', ROOT / 'scripts/provision_celisca_detailed.py')
provision = importlib.util.module_from_spec(spec)
spec.loader.exec_module(provision)


def test_explicit_nested_gltf_frames_and_reflection_winding():
    positions = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], '<f4')
    indices = np.array([0, 1, 2], '<u2')
    blob = positions.tobytes() + indices.tobytes()
    document = {'scene': 0, 'scenes': [{'nodes': [0]}],
                'nodes': [{'translation': [5, 2, 3], 'children': [1]}, {'scale': [-2, 1, 1], 'mesh': 0}],
                'meshes': [{'primitives': [{'attributes': {'POSITION': 0}, 'indices': 1}]}],
                'bufferViews': [{'byteLength': 36}, {'byteOffset': 36, 'byteLength': 6}],
                'accessors': [{'bufferView': 0, 'componentType': 5126, 'count': 3, 'type': 'VEC3'},
                              {'bufferView': 1, 'componentType': 5123, 'count': 3, 'type': 'SCALAR'}]}
    _, actual, faces, _, _, _ = next(provision.placed_primitives(document, blob))
    np.testing.assert_allclose(actual, [[5, -3, 2], [3, -3, 2], [5, -3, 3]])
    np.testing.assert_array_equal(faces, [[0, 2, 1]])
    assert np.linalg.det(provision.ROTATION) == 1


def test_source_copy_refuses_changed_inputs_and_destination(tmp_path):
    source, destination = tmp_path / 'source', tmp_path / 'target'
    source.write_bytes(b'original')
    pin = provision.digest(source)
    provision.copy_verified(source, destination, pin)
    source.write_bytes(b'changed')
    with pytest.raises(ValueError, match='Source SHA-256'):
        provision.copy_verified(source, destination, pin)
    source.write_bytes(b'original'); destination.write_bytes(b'user edit')
    with pytest.raises(ValueError, match='Destination differs'):
        provision.copy_verified(source, destination, pin)


@pytest.mark.parametrize('floor', [1, 2])
def test_detailed_worlds_keep_unit_source_collision_and_no_added_slab(floor):
    map_id = f'celisca_floor_{floor}_detailed'
    target = MAPS / map_id
    record = json.loads((target / 'PROVENANCE.json').read_text())
    assert record['building_floor'] == floor + 1
    assert record['full_glb']['sha256'] == provision.PINS[floor]['detailed']
    root = ET.parse(target / 'worlds' / f'{map_id}.world').getroot()
    assert not root.findall('.//box') and not root.findall('.//plane')
    assert root.find('.//model/static').text == 'true'
    assert root.find('.//collision/geometry/mesh/scale').text == '1 1 1'
    assert root.find('.//collision/geometry/mesh/uri').text.endswith('/adapters/meshes/collision.obj')
    assert root.find('.//visual/geometry/mesh/uri').text.endswith('/z_up_visual_v1/visual_full.dae')
    for relative, digest in record['copied_adapter_sha256'].items():
        assert provision.digest(target / relative) == digest
    assert record['source_manifest']['detailParts'] == 96
    assert record['source_manifest']['sanitaryFixtures'] == 11
    assert record['visual']['triangles'] > 2_000_000


@pytest.mark.parametrize('floor', [1, 2])
def test_spawn_and_registered_grid_agree_and_keep_unsupported_cells_unknown(floor):
    map_id = f'celisca_floor_{floor}_detailed'; target = MAPS / map_id
    projection = json.loads((target / 'maps/projection.json').read_text())
    metadata = yaml.safe_load((target / 'maps/map.yaml').read_text())
    profiles = yaml.safe_load((ROOT / 'src/robot_lab_bringup/config/sim_maps.yaml').read_text())['maps']
    assert profiles[map_id]['spawn'] == projection['spawn']
    assert profiles[map_id]['map']['has_2d_map'] is True
    assert projection['spawn_mask_clearance_m'] >= .9
    assert projection['spawn_independent_horizontal_ray_clearance_m'] >= .8
    assert abs(projection['spawn_floor_z_m']) < .025
    assert projection['engine'].startswith('offline')
    image = np.array(Image.open(target / 'maps/map.pgm'))
    x = int((projection['spawn']['x'] - metadata['origin'][0]) / metadata['resolution'])
    y = image.shape[0] - 1 - int((projection['spawn']['y'] - metadata['origin'][1]) / metadata['resolution'])
    assert image[y, x] == 254
    assert projection['counts']['unknown'] > 0 and projection['counts']['occupied'] > 0
    assert provision.digest(target / 'maps/map.pgm') == projection['map_sha256']
