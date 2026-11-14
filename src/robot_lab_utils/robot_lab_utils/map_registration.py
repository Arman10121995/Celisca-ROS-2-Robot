"""Register a reviewed occupancy grid for a world (ROADMAP R6.5).

Generation never changes which grid Nav2 uses. After reviewing a generated
grid, the operator registers it explicitly. Registration copies the grid into
the SSD registry and records the review; it refuses when:

- the files changed after the generation report hashed them;
- the world source changed after generation;
- the world's spawn point is not a free cell of the grid;
- the grid has too little free space.

Map profiles then use the registered grid. The original grid stays recorded
and can be restored with ``unregister``.
"""
import hashlib
import json
import math
import shutil
import time
from pathlib import Path

import numpy as np
import yaml


def registry_root():
    import os
    return Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'registered_maps'


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_registry(root=None):
    path = (root or registry_root()) / 'registry.yaml'
    return (yaml.safe_load(path.read_text()) or {}) if path.is_file() else {}


def _save(registry, root):
    root.mkdir(parents=True, exist_ok=True)
    temporary = root / 'registry.yaml.new'
    temporary.write_text(yaml.safe_dump(registry, sort_keys=True))
    temporary.replace(root / 'registry.yaml')


def check_grid(report_path, profile):
    """Validate a generation report against its files, world and spawn; returns a summary."""
    report_path = Path(report_path)
    report = json.loads(report_path.read_text())
    grid_yaml = Path(report['output_yaml'])
    for suffix, expected in report['artifact_sha256'].items():
        if _sha(grid_yaml.with_suffix(suffix)) != expected:
            raise ValueError('The grid %s changed after generation; generate and review it again' % suffix)
    world = profile.get('gazebo', {}).get('world_path')
    if world and Path(world).is_file() and _sha(world) != report['source_sha256'] and Path(report['source_world']) == Path(world):
        raise ValueError('The world source changed after this grid was generated')
    meta = yaml.safe_load(grid_yaml.read_text())
    from PIL import Image
    image = np.asarray(Image.open(grid_yaml.parent / meta['image']))
    spawn = profile.get('spawn') or {}
    x, y = float(spawn.get('x', 0.)), float(spawn.get('y', 0.))
    resolution, (ox, oy) = float(meta['resolution']), meta['origin'][:2]
    col, row = int(math.floor((x - ox) / resolution)), image.shape[0] - 1 - int(math.floor((y - oy) / resolution))
    if not (0 <= row < image.shape[0] and 0 <= col < image.shape[1]) or image[row, col] != 254:
        raise ValueError('The world spawn (%.2f, %.2f) is not a free cell of this grid; pick the right seed/region' % (x, y))
    free = int(np.sum(image == 254))
    if free * resolution ** 2 < 1.:
        raise ValueError('The grid has less than 1 m² of free space')
    regions = report_path.with_name(report_path.name.replace('.generation.json', '.regions.json'))
    return dict(grid_yaml=str(grid_yaml), free_cells=free, occupied_cells=int(np.sum(image == 0)),
                unknown_cells=int(np.sum(image == 205)), resolution_m=resolution,
                regions_mode=report.get('regions_mode', 'seed'),
                regions=json.loads(regions.read_text())['regions'] if regions.is_file() else None,
                spawn=[x, y], report=str(report_path))


def register(world, report_path, profile, note='', root=None):
    """Copy a checked grid into the registry and make it the world's map."""
    if not note.strip():
        raise ValueError('Registration needs a review note')
    summary = check_grid(report_path, profile)
    root = root or registry_root()
    target = root / world / time.strftime('%Y%m%d-%H%M%S')
    target.mkdir(parents=True)
    source = Path(summary['grid_yaml'])
    meta = yaml.safe_load(source.read_text())
    shutil.copy2(source.parent / meta['image'], target / meta['image'])
    shutil.copy2(source, target / 'map.yaml')
    meta_path = target / 'map.yaml'
    copied = yaml.safe_load(meta_path.read_text())
    copied['image'] = meta['image']
    meta_path.write_text(yaml.safe_dump(copied))
    shutil.copy2(report_path, target / 'generation.json')
    registry = load_registry(root)
    registry[world] = dict(map=dict(has_2d_map=True, path=str(meta_path)), registered=time.strftime('%Y-%m-%dT%H:%M:%S'),
                           note=note.strip(), previous_map=profile.get('map'), grid_sha256=_sha(meta_path.parent / meta['image']),
                           **{k: summary[k] for k in ('free_cells', 'occupied_cells', 'unknown_cells', 'regions_mode', 'report')})
    _save(registry, root)
    return registry[world]


def unregister(world, root=None):
    root = root or registry_root()
    registry = load_registry(root)
    removed = registry.pop(world, None)
    _save(registry, root)
    return removed


def apply(profiles, root=None):
    """Map profiles with registered grids applied (absolute paths)."""
    registry = load_registry(root)
    result = dict(profiles)
    for world, entry in registry.items():
        if world in result and Path(entry['map']['path']).is_file():
            result[world] = dict(result[world], map=dict(entry['map']), map_registration=dict(
                registered=entry['registered'], note=entry['note'], regions_mode=entry.get('regions_mode')))
    return result
