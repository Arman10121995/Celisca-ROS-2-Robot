"""Reviewed occupancy-grid registration (R6.5)."""
import hashlib
import json

import numpy as np
import pytest
import yaml
from PIL import Image

from robot_lab_utils.map_registration import apply, check_grid, register, unregister


def grid(tmp_path, free_box=(10, 30, 10, 30), size=40):
    image = np.full((size, size), 205, dtype=np.uint8)
    r0, r1, c0, c1 = free_box
    image[r0:r1, c0:c1] = 254
    run = tmp_path / 'run'
    run.mkdir()
    Image.fromarray(image).save(run / 'w.pgm')
    (run / 'w.yaml').write_text(yaml.safe_dump(dict(image='w.pgm', resolution=.1, origin=[-2., -2., 0.],
                                                    negate=0, occupied_thresh=.65, free_thresh=.196)))
    world = tmp_path / 'w.world'
    world.write_text('<sdf/>')
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    report = run / 'w.generation.json'
    report.write_text(json.dumps(dict(output_yaml=str(run / 'w.yaml'), source_world=str(world), source_sha256=sha(world),
                                      artifact_sha256={'.yaml': sha(run / 'w.yaml'), '.pgm': sha(run / 'w.pgm')},
                                      regions_mode='seed')))
    profile = dict(gazebo=dict(world_path=str(world)), spawn=dict(x=0., y=0.), map=dict(has_2d_map=False))
    return report, profile, run


def test_register_and_apply_then_unregister(tmp_path):
    report, profile, _ = grid(tmp_path)
    entry = register('w', report, profile, 'walls match the world', root=tmp_path / 'reg')
    assert entry['map']['has_2d_map'] and entry['free_cells'] == 400
    applied = apply(dict(w=profile, other={}), root=tmp_path / 'reg')
    assert applied['w']['map']['path'].endswith('map.yaml') and applied['w']['map_registration']['note']
    assert applied['other'] == {}
    unregister('w', root=tmp_path / 'reg')
    assert apply(dict(w=profile), root=tmp_path / 'reg')['w'] == profile


def test_changed_grid_spawn_outside_and_missing_note_are_refused(tmp_path):
    report, profile, run = grid(tmp_path)
    with pytest.raises(ValueError, match='note'):
        register('w', report, profile, ' ', root=tmp_path / 'reg')
    with pytest.raises(ValueError, match='spawn'):
        check_grid(report, dict(profile, spawn=dict(x=1.8, y=1.8)))
    (run / 'w.pgm').write_bytes((run / 'w.pgm').read_bytes() + b'x')
    with pytest.raises(ValueError, match='changed'):
        check_grid(report, profile)
