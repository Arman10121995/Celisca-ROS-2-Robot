"""Furnished occupancy maps come from the furnished world's collision geometry."""
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
import yaml
from PIL import Image

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
MAPS = Path(__file__).resolve().parents[1] / 'maps'


def _world(path, boxes):
    models = ''.join(
        f'<model name="b{i}"><static>true</static><pose>{x} {y} {h/2} 0 0 0</pose><link name="l">'
        f'<collision name="c"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>'
        f'</link></model>' for i, (x, y, sx, sy, h) in enumerate(boxes))
    path.write_text(f'<?xml version="1.0"?><sdf version="1.6"><world name="w">{models}</world></sdf>')


def _base_map(directory):
    image = np.full((100, 100), 205, dtype=np.uint8)
    image[10:90, 10:90] = 254                  # free room
    image[10, 10:90] = image[89, 10:90] = 0    # drawn walls
    Image.fromarray(image).save(directory / 'map.pgm')
    (directory / 'map.yaml').write_text('image: map.pgm\nmode: trinary\nresolution: 0.05\n'
                                        'origin: [-2.5, -2.5, 0]\nnegate: 0\noccupied_thresh: 0.65\n'
                                        'free_thresh: 0.25\n')


def _run(*args):
    return subprocess.run([sys.executable, str(TOOLS / 'furnish_occupancy_map.py'), *map(str, args)],
                          capture_output=True, text=True)


def test_furniture_is_rasterized_on_the_base_grid(tmp_path):
    base = tmp_path / 'base'; base.mkdir(); _base_map(base)
    world = tmp_path / 'w.world'
    _world(world, [(1.0, 0.0, 0.4, 0.6, 0.7),     # table-height box at x=1
                   (-1.0, -1.0, 0.2, 0.2, 0.05)])  # too low to cross any slice
    result = _run('--base', base / 'map.yaml', '--world', world, '--output', tmp_path / 'out',
                  '--spawn', -1.5, 1.5)
    assert result.returncode == 0, result.stderr
    out = np.array(Image.open(tmp_path / 'out' / 'map.pgm'))
    meta = yaml.safe_load((tmp_path / 'out' / 'map.yaml').read_text())
    assert meta['origin'] == [-2.5, -2.5, 0] and meta['resolution'] == 0.05

    def cell(x, y):
        return out[99 - int((y + 2.5) / 0.05), int((x + 2.5) / 0.05)]
    assert cell(1.2, 0.0) == 0 and cell(0.8, 0.0) == 0      # box outline occupied
    assert cell(-1.0, -1.0) == 254                           # below the slices: free
    assert cell(-1.5, 1.5) == 254 and cell(0.0, 2.4) == 205  # base pixels kept
    # Drawn walls the geometry does not confirm are no longer phantom walls.
    assert out[10, 50] == 205
    report = json.loads((tmp_path / 'out' / 'map_source.json').read_text())
    assert report['occupied_cells'] > 0 and report['base_free_kept'] > 0.9


def test_a_spawn_inside_furniture_is_refused(tmp_path):
    base = tmp_path / 'base'; base.mkdir(); _base_map(base)
    world = tmp_path / 'w.world'
    _world(world, [(0.0, 0.0, 0.4, 0.4, 0.7)])
    result = _run('--base', base / 'map.yaml', '--world', world, '--output', tmp_path / 'out',
                  '--spawn', 0.2, 0.0)
    assert result.returncode != 0 and 'spawn' in result.stderr


@pytest.mark.parametrize('floor', [1, 2])
def test_installed_furnished_maps_record_their_source(floor):
    directory = MAPS / f'celisca_floor_{floor}_furniture' / 'maps'
    report = json.loads((directory / 'map_source.json').read_text())
    assert report['generator'] == 'src/robot_lab_maps/tools/furnish_occupancy_map.py'
    assert report['world'].endswith(f'celisca_floor_{floor}_furniture.world')
    plain = (MAPS / f'celisca_floor_{floor}' / 'maps' / 'map.pgm').read_bytes()
    assert (directory / 'map.pgm').read_bytes() != plain
