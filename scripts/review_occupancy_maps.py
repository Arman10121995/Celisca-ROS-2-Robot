#!/usr/bin/env python3
"""Review a generated occupancy map against the world geometry it came from.

The October 5 generator exports previews of the *seed-connected* free region;
before a generated grid may be registered for navigation its origin, scale,
height, seed, free/occupied/unknown cells and obstacle alignment must be
reviewed against the source world, and disconnected free regions must be
reported rather than silently ignored.

Checks (see docs/ASSET_EXTENSION_GUIDE.md): ``metadata``,
``image_integrity``, ``cell_alignment``, ``freshness`` (sidecar hashes of the
PGM/YAML and the source world), ``extent_agreement`` (the map extent must
recompute from the source world with the generator's own conservative recipe
within two cells and cover the world's XY bounds), ``seed_free``,
``free_regions`` (disconnected regions are reported, not failed - the export
is a seed-connected preview), ``geometry_agreement`` (every cell centre
exactly inside a primitive, every mesh slice sample and every solid terrain
sample at the slice must be non-free; anything shown *free* is real geometry
presented as drivable and fails), ``rotated_geometry`` (the non-identity
orientation subset is counted and checked separately) and
``decorative_occupied`` (occupied cells beyond one cell from any slice
geometry fail).

World-dependent checks are ``skipped`` when ``--world`` is not given.  This
tool reviews artifacts; it does not run a robot and does not qualify
navigation.
"""
import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

try:
    from robot_lab_utils.sdf_world import extract_static_shapes, uri_resolver
    from robot_lab_utils.occupancy_slice import primitive_slice_cells
except ImportError:  # source-tree execution without an installed workspace
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent
                            / 'src' / 'robot_lab_utils'))
    from robot_lab_utils.sdf_world import extract_static_shapes, uri_resolver
    from robot_lab_utils.occupancy_slice import primitive_slice_cells

try:
    from scipy import ndimage as _ndimage
    from scipy.spatial import cKDTree as _cKDTree
except ImportError:  # pragma: no cover - host always ships scipy
    _ndimage = _cKDTree = None

_HEIGHT_EPS = 1e-6
# Declared conservative cell limits (cells, at the map's own resolution).
EXTENT_TOLERANCE_CELLS = 2
DECORATIVE_TOLERANCE_CELLS = 1.5
MAX_DECORATIVE_SAMPLES = 5000


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_map(yaml_path):
    """Return (metadata, image ndarray, image_path)."""
    yaml_path = Path(yaml_path)
    metadata = yaml.safe_load(yaml_path.read_text())
    image_path = yaml_path.parent / metadata['image']
    image = np.asarray(Image.open(image_path))
    if image.ndim != 2:
        raise ValueError('Occupancy image must be a 2-D grayscale grid')
    return metadata, image, image_path


def classify(metadata, image):
    """ROS map_server classification of each pixel value."""
    negate = int(metadata.get('negate', 0))
    occupied_thresh = float(metadata['occupied_thresh'])
    free_thresh = float(metadata['free_thresh'])
    darkness = (255.0 - image.astype(float)) / 255.0
    if negate:
        darkness = 1.0 - darkness
    free = darkness < free_thresh
    occupied = darkness > occupied_thresh
    unknown = ~free & ~occupied
    return free, occupied, unknown


def world_to_cell(metadata, image, x, y):
    """(row, col) of the PGM (row 0 = top = max y) for a world point."""
    res = float(metadata['resolution'])
    height, width = image.shape
    col = int(math.floor((x - metadata['origin'][0]) / res))
    row = height - 1 - int(math.floor((y - metadata['origin'][1]) / res))
    return row, col


def cell_to_world(metadata, image, row, col):
    res = float(metadata['resolution'])
    height, _ = image.shape
    x = metadata['origin'][0] + (col + 0.5) * res
    y = metadata['origin'][1] + (height - 1 - row + 0.5) * res
    return x, y


def _status(ok, warn=False):
    if ok:
        return 'pass'
    return 'warn' if warn else 'fail'
def check_metadata(metadata):
    problems = []
    for key in ('image', 'resolution', 'origin', 'occupied_thresh', 'free_thresh'):
        if key not in metadata:
            problems.append('missing key %s' % key)
    if 'resolution' in metadata:
        res = float(metadata['resolution'])
        if not math.isfinite(res) or res <= 0:
            problems.append('resolution must be finite and positive')
    if 'origin' in metadata:
        origin = metadata['origin']
        if not (isinstance(origin, list) and len(origin) == 3
                and all(math.isfinite(float(v)) for v in origin)):
            problems.append('origin must be three finite numbers')
    if 'occupied_thresh' in metadata and 'free_thresh' in metadata:
        occupied = float(metadata['occupied_thresh'])
        free = float(metadata['free_thresh'])
        if not (0.0 <= free < occupied <= 1.0):
            problems.append('require 0 <= free_thresh < occupied_thresh <= 1')
    negate = int(metadata.get('negate', 0))
    if negate not in (0, 1):
        problems.append('negate must be 0 or 1')
    return {'status': _status(not problems), 'problems': problems}


def check_image(metadata, image):
    values, counts = np.unique(image, return_counts=True)
    histogram = {int(v): int(c) for v, c in zip(values, counts)}
    free, occupied, unknown = classify(metadata, image)
    problems = []
    if int(free.sum()) < 10:
        problems.append('fewer than ten free cells')
    unexpected = sorted(set(histogram) - {0, 254, 205})
    if unexpected:
        problems.append('unexpected pixel values %s' % unexpected[:8])
    return {'status': _status(not problems), 'problems': problems,
            'free_cells': int(free.sum()),
            'occupied_cells': int(occupied.sum()),
            'unknown_cells': int(unknown.sum()),
            'histogram': histogram}


def check_cell_alignment(metadata, image):
    res = float(metadata['resolution'])
    offsets = [float(v) / res for v in metadata['origin'][:2]]
    misaligned = [o for o in offsets if abs(o - round(o)) > 1e-6]
    return {'status': _status(not misaligned),
            'origin_in_cells': offsets,
            'problems': (['origin is not on a cell boundary'] if misaligned else [])}


def _generation_sidecar(map_yaml):
    sidecar = Path(map_yaml).with_suffix('.generation.json')
    return json.loads(sidecar.read_text()) if sidecar.is_file() else None


def check_freshness(map_yaml, metadata, image, world, generation):
    problems, warnings = [], []
    if generation is None:
        return {'status': 'warn', 'problems': [],
                'warnings': ['no generation sidecar; artifact freshness cannot be proven']}
    recorded = generation.get('artifact_sha256', {})
    actual_yaml = sha256(map_yaml)
    image_path = Path(map_yaml).parent / metadata['image']
    if recorded.get('.yaml') and recorded['.yaml'] != actual_yaml:
        problems.append('map YAML changed since generation')
    if recorded.get('.pgm') and recorded['.pgm'] != sha256(image_path):
        problems.append('map PGM changed since generation')
    if generation.get('origin') and list(generation['origin']) != list(metadata['origin']):
        problems.append('recorded origin differs from map YAML')
    if world is not None:
        recorded_source = generation.get('source_sha256')
        if recorded_source and recorded_source != sha256(world):
            problems.append('source world changed since the map was generated')
    elif generation.get('source_sha256'):
        warnings.append('source world not supplied; source freshness not checked')
    return {'status': _status(not problems), 'problems': problems,
            'warnings': warnings,
            'recorded_source_sha256': generation.get('source_sha256')}


def _world_shapes(world):
    """Resolve and extract static collision geometry like the generator does."""
    world = Path(world)
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('robot_lab_maps'))
        dirs = [str(p) for p in share.glob('maps/*/models')]
        package_share = get_package_share_directory
    except Exception:
        dirs, package_share = [], None
    dirs += [str(world.parent), str(world.parent.parent / 'models')]
    shapes, skipped = extract_static_shapes(str(world), uri_resolver(dirs, package_share),
                                            collision_only=True)
    return shapes, skipped
def _pose_matrix(shape):
    quat = np.asarray(shape['orientation'], dtype=float)
    quat = quat / np.linalg.norm(quat)
    w, x, y, z = quat
    rotation = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(shape['position'], dtype=float)
    return transform


def _shape_bounds_xy(shape):
    """Generator-recipe XY corners (metres) that decide the map reach."""
    import itertools
    kind = shape['type']
    if kind == 'plane':
        return []
    if kind == 'mesh':
        from robot_lab_utils.mesh_assets import _load_indexed_mesh
        vertices, _ = _load_indexed_mesh(shape['mesh'])
        vertices = np.asarray(vertices, dtype=float) * np.asarray(shape['scale'], dtype=float)
        transform = _pose_matrix(shape)
        corners = vertices @ transform[:3, :3].T + transform[:3, 3]
        return corners[:, :2]
    if kind == 'heightmap':
        from robot_lab_utils.heightfield import heightfield_grid
        vertices, _ = heightfield_grid(shape)
        transform = _pose_matrix(shape)
        vertices = np.asarray(vertices, dtype=float)
        corners = vertices @ transform[:3, :3].T + transform[:3, 3]
        return corners[:, :2]
    extent = (np.asarray(shape['size'], dtype=float) / 2 if kind == 'box'
              else np.full(3, shape['size'][0]))
    if kind == 'cylinder':
        extent[2] = shape['size'][1] / 2
    quat = np.asarray(shape['orientation'], dtype=float)
    quat = quat / np.linalg.norm(quat)
    w, x, y, z = quat
    rotation = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return [(np.asarray(shape['position']) + rotation @ (extent * np.array(signs)))[:2]
            for signs in itertools.product((-1, 1), repeat=3)]


def check_extent(metadata, image, shapes, generation):
    problems, warnings = [], []
    res = float(metadata['resolution'])
    height, width = image.shape
    bounds = []
    for shape in shapes:
        bounds.extend(_shape_bounds_xy(shape))
    if not bounds:
        return {'status': 'warn', 'problems': [],
                'warnings': ['world contributes no non-plane bounds']}
    bounds = np.asarray(bounds, dtype=float)
    reach = np.max(np.abs(bounds), axis=0) + 2.0
    expected_sizes = np.ceil(2 * reach / res) * res
    expected_cells = np.round(expected_sizes / res).astype(int)
    actual_cells = np.array([width, height])
    delta = np.abs(actual_cells - expected_cells)
    if np.any(delta > EXTENT_TOLERANCE_CELLS):
        problems.append('map extent %s cells differs from world recipe %s cells by more than %d cells'
                        % (actual_cells.tolist(), expected_cells.tolist(), EXTENT_TOLERANCE_CELLS))
    origin = np.asarray(metadata['origin'][:2], dtype=float)
    map_min = origin
    map_max = origin + actual_cells * res
    world_min = bounds.min(axis=0)
    world_max = bounds.max(axis=0)
    if np.any(map_min > world_min + res) or np.any(map_max < world_max - res):
        problems.append('map extent does not cover the world XY bounds')
    if generation and generation.get('origin'):
        if not np.allclose(np.asarray(generation['origin'][:2], dtype=float), origin, atol=1e-9):
            problems.append('generation sidecar origin differs from map YAML')
    margins = {'left': float(world_min[0] - map_min[0]),
               'right': float(map_max[0] - world_max[0]),
               'bottom': float(world_min[1] - map_min[1]),
               'top': float(map_max[1] - world_max[1])}
    return {'status': _status(not problems), 'problems': problems, 'warnings': warnings,
            'expected_cells': expected_cells.tolist(),
            'actual_cells': [int(width), int(height)],
            'cell_delta': delta.tolist(),
            'world_bounds_xy': {'min': world_min.tolist(), 'max': world_max.tolist()},
            'coverage_margins': margins}


def check_seed(metadata, image, seed):
    problems = []
    if seed is None:
        return {'status': 'warn', 'problems': [],
                'warnings': ['no seed supplied (--seed or generation sidecar)']}
    row, col = world_to_cell(metadata, image, float(seed[0]), float(seed[1]))
    height, width = image.shape
    inside = 0 <= row < height and 0 <= col < width
    value = int(image[row, col]) if inside else None
    free, _, _ = classify(metadata, image)
    is_free = bool(inside and free[row, col])
    if not is_free:
        problems.append('seed cell is not free (value %s)' % value)
    x, y = cell_to_world(metadata, image, row, col) if inside else (None, None)
    return {'status': _status(not problems), 'problems': problems,
            'seed_xy': [float(seed[0]), float(seed[1])],
            'cell': [int(row), int(col)] if inside else None,
            'cell_center_xy': [x, y], 'pixel_value': value}
def check_free_regions(metadata, image, free, seed):
    height, width = image.shape
    if _ndimage is None:
        return {'status': 'warn', 'problems': [],
                'warnings': ['scipy unavailable; free regions not labelled']}
    labels, count = _ndimage.label(free, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]]))
    if count == 0:
        return {'status': 'fail', 'problems': ['no free region at all']}
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0
    res = float(metadata['resolution'])
    seed_label = 0
    if seed is not None:
        row, col = world_to_cell(metadata, image, float(seed[0]), float(seed[1]))
        if 0 <= row < height and 0 <= col < width:
            seed_label = int(labels[row, col])
    regions = []
    for label in np.argsort(sizes)[::-1][:20]:
        if label == 0 or sizes[label] == 0:
            continue
        rows, cols = np.where(labels == label)
        cx, cy = cell_to_world(metadata, image, int(round(rows.mean())), int(round(cols.mean())))
        regions.append({'label': int(label), 'cells': int(sizes[label]),
                        'area_m2': float(sizes[label] * res * res),
                        'centroid_xy': [cx, cy],
                        'seed_connected': bool(label == seed_label)})
    disconnected = [r for r in regions if not r['seed_connected']]
    problems, warnings = [], []
    if seed is not None and seed_label == 0:
        problems.append('seed does not belong to any free region')
    if seed_label and seed_label != int(np.argmax(sizes)):
        warnings.append('seed-connected region is not the largest free region')
    return {'status': _status(not problems, warn=bool(warnings)), 'problems': problems,
            'warnings': warnings,
            'region_count': int(count),
            'seed_region_cells': int(sizes[seed_label]) if seed_label else 0,
            'largest_region_cells': int(sizes.max()),
            'disconnected_regions': disconnected,
            'regions_reported': len(regions)}


def _inside_shape(shape, x, y, height_m):
    quat = np.asarray(shape['orientation'], dtype=float)
    quat = quat / np.linalg.norm(quat)
    w, qx, qy, qz = quat
    rotation = np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * w), 2 * (qx * qz + qy * w)],
                         [2 * (qx * qy + qz * w), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * w)],
                         [2 * (qx * qz - qy * w), 2 * (qy * qz + qx * w), 1 - 2 * (qx * qx + qy * qy)]])
    local = rotation.T @ (np.array([x, y, height_m]) - np.asarray(shape['position'], dtype=float))
    kind, size = shape['type'], shape['size']
    if kind == 'box':
        return bool(np.all(np.abs(local) <= np.asarray(size, dtype=float) / 2 + 1e-9))
    if kind == 'sphere':
        return float(np.linalg.norm(local)) <= size[0] + 1e-9
    if kind == 'cylinder':
        return (float(np.hypot(local[0], local[1])) <= size[0] + 1e-9
                and abs(local[2]) <= size[1] / 2 + 1e-9)
    return False


def _is_rotated(shape, tolerance=1e-6):
    quat = np.asarray(shape['orientation'], dtype=float)
    return bool(abs(abs(quat[0] / np.linalg.norm(quat)) - 1.0) > tolerance)


def _mesh_slice_points(shape, height_m, spacing):
    import trimesh
    from robot_lab_utils.mesh_assets import _load_indexed_mesh
    vertices, faces = _load_indexed_mesh(shape['mesh'])
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mesh.apply_scale(shape['scale'])
    mesh.apply_transform(_pose_matrix(shape))
    section = mesh.section(plane_origin=[0, 0, height_m], plane_normal=[0, 0, 1])
    points = []
    if section is None:
        return points
    for polyline in section.discrete:
        polyline = np.asarray(polyline, dtype=float)[:, :2]
        for start, end in zip(polyline[:-1], polyline[1:]):
            length = float(np.linalg.norm(end - start))
            steps = max(1, int(math.ceil(length / spacing)))
            for step in range(steps + 1):
                point = start + (end - start) * (step / steps)
                points.append(point)
    return points


def _heightmap_solid_points(shape, height_m):
    from robot_lab_utils.heightfield import heightfield_grid
    vertices, _ = heightfield_grid(shape)
    vertices = np.asarray(vertices, dtype=float)
    transform = _pose_matrix(shape)
    world = vertices @ transform[:3, :3].T + transform[:3, 3]
    solid = world[world[:, 2] >= height_m - _HEIGHT_EPS]
    unique_x = np.unique(np.round(solid[:, 0], 6))
    spacing = float(np.min(np.diff(unique_x))) if len(unique_x) > 1 else 0.0
    return solid[:, :2], spacing
def check_geometry(metadata, image, free, occupied, shapes, height_m):
    """Real geometry must never be classified free; report rotated subset."""
    res = float(metadata['resolution'])
    height, width = image.shape
    problems = []
    rotated_problems = []
    stats = {'primitive_shapes': 0, 'mesh_shapes': 0, 'heightmap_shapes': 0,
             'shapes_at_slice': 0, 'rotated_shapes_at_slice': 0,
             'primitive_cells_checked': 0, 'primitive_cells_shown_free': 0,
             'mesh_samples_checked': 0, 'mesh_samples_shown_free': 0,
             'heightmap_samples_checked': 0, 'heightmap_samples_shown_free': 0,
             'examples_shown_free': []}
    footprint_points = []

    def note_violation(kind, x, y, rotated):
        if rotated:
            rotated_problems.append('%s at (%.2f, %.2f) is free' % (kind, x, y))
        if len(stats['examples_shown_free']) < 8:
            stats['examples_shown_free'].append(
                {'kind': kind.split()[0], 'xy': [round(float(x), 3), round(float(y), 3)]})

    for shape in shapes:
        kind = shape['type']
        if kind == 'plane':
            continue
        rotated = _is_rotated(shape)
        if kind in ('box', 'sphere', 'cylinder'):
            stats['primitive_shapes'] += 1
            cells = primitive_slice_cells(shape, [width * res, height * res], res, height_m)
            if cells is None:
                continue
            stats['shapes_at_slice'] += 1
            if rotated:
                stats['rotated_shapes_at_slice'] += 1
            x0, y0, x1, y1 = cells
            x0, y0 = max(x0, 0), max(y0, 0)
            x1, y1 = min(x1, width - 1), min(y1, height - 1)
            # The projected rectangle (the generator's conservative mask) is
            # the reference footprint for the decorative-occupied check.
            for row in range(y0, y1 + 1):
                footprint_points.extend((height - 1 - row, col)
                                        for col in range(x0, x1 + 1))
            for row in range(y0, y1 + 1):
                for col in range(x0, x1 + 1):
                    x = metadata['origin'][0] + (col + 0.5) * res
                    y = metadata['origin'][1] + (row + 0.5) * res
                    if not _inside_shape(shape, x, y, height_m):
                        continue
                    img_row = height - 1 - row
                    stats['primitive_cells_checked'] += 1
                    footprint_points.append((img_row, col))
                    if free[img_row, col]:
                        stats['primitive_cells_shown_free'] += 1
                        note_violation('rotated primitive cell' if rotated
                                       else 'primitive cell', x, y, rotated)
        elif kind == 'mesh':
            stats['mesh_shapes'] += 1
            points = _mesh_slice_points(shape, height_m, res / 2.0)
            if not points:
                continue
            stats['shapes_at_slice'] += 1
            if rotated:
                stats['rotated_shapes_at_slice'] += 1
            for point in points:
                row, col = world_to_cell(metadata, image, float(point[0]), float(point[1]))
                if not (0 <= row < height and 0 <= col < width):
                    continue
                stats['mesh_samples_checked'] += 1
                footprint_points.append((row, col))
                if free[row, col]:
                    stats['mesh_samples_shown_free'] += 1
                    note_violation('rotated mesh slice sample' if rotated
                                   else 'mesh slice sample', point[0], point[1], rotated)
        elif kind == 'heightmap':
            stats['heightmap_shapes'] += 1
            solid, spacing = _heightmap_solid_points(shape, height_m)
            if not len(solid):
                continue
            stats['shapes_at_slice'] += 1
            if rotated:
                stats['rotated_shapes_at_slice'] += 1
            radius = spacing / 2.0
            span = int(math.ceil(radius / res))
            for point in solid:
                row, col = world_to_cell(metadata, image, float(point[0]), float(point[1]))
                for drow in range(-span, span + 1):
                    for dcol in range(-span, span + 1):
                        r, c = row + drow, col + dcol
                        if not (0 <= r < height and 0 <= c < width):
                            continue
                        x, y = cell_to_world(metadata, image, r, c)
                        if math.hypot(x - point[0], y - point[1]) > radius + res / 2:
                            continue
                        stats['heightmap_samples_checked'] += 1
                        footprint_points.append((r, c))
                        if free[r, c]:
                            stats['heightmap_samples_shown_free'] += 1
                            note_violation('heightmap solid cell', x, y, rotated)
    shown = (stats['primitive_cells_shown_free'] + stats['mesh_samples_shown_free']
             + stats['heightmap_samples_shown_free'])
    if shown:
        problems.append('%d cells of real geometry are classified free at the slice' % shown)
    stats['rotated_examples_shown_free'] = rotated_problems[:8]
    geometry = {'status': _status(not problems), 'problems': problems}
    rotated_result = {'status': _status(not rotated_problems),
                      'problems': rotated_problems[:8],
                      'rotated_shapes_at_slice': stats['rotated_shapes_at_slice']}
    if stats['rotated_shapes_at_slice'] == 0:
        rotated_result['note'] = 'no rotated shape crosses the slice height'
    return geometry, rotated_result, stats, footprint_points
def check_decorative(metadata, image, occupied, footprint_points):
    """Occupied cells with no source geometry within one cell are decorative."""
    if not footprint_points:
        return {'status': 'warn', 'problems': [],
                'warnings': ['no slice footprint to compare occupied cells against']}
    points = np.asarray(footprint_points, dtype=float)
    rows, cols = np.where(occupied)
    stride = max(1, len(rows) // MAX_DECORATIVE_SAMPLES)
    sample_rows, sample_cols = rows[::stride], cols[::stride]
    if _cKDTree is not None:
        tree = _cKDTree(points)
        distances, _ = tree.query(np.column_stack([sample_rows, sample_cols]), k=1)
        cell_distances = distances
    else:  # pragma: no cover - scipy ships with this host
        cell_distances = np.array([np.abs(points - np.array([r, c])).max(axis=1).min()
                                   for r, c in zip(sample_rows, sample_cols)])
    unmatched_mask = cell_distances > DECORATIVE_TOLERANCE_CELLS
    unmatched = [(int(r), int(c)) for r, c in
                 zip(sample_rows[unmatched_mask], sample_cols[unmatched_mask])][:8]
    fraction = float(unmatched_mask.mean()) if len(cell_distances) else 0.0
    problems = []
    if fraction > 0.01:
        problems.append('%.1f%% of sampled occupied cells have no geometry within %.1f cells'
                        % (100 * fraction, DECORATIVE_TOLERANCE_CELLS))
    return {'status': _status(not problems), 'problems': problems,
            'sampled_occupied_cells': int(len(sample_rows)),
            'unmatched_fraction': round(fraction, 6),
            'unmatched_examples_cells': unmatched}


def review(map_yaml, world=None, seed=None, height=None, title=None):
    """Run every check and return the JSON-serialisable report."""
    map_yaml = Path(map_yaml).resolve()
    metadata, image, image_path = load_map(map_yaml)
    generation = _generation_sidecar(map_yaml)
    if height is None:
        height = float((generation or {}).get('slice_height_m', 0.3))
    if seed is None and generation and generation.get('seed_xy'):
        seed = generation['seed_xy']
    free, occupied, unknown = classify(metadata, image)

    report = {
        'tool': 'review_occupancy_maps.py',
        'version': 1,
        'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'map_yaml': str(map_yaml),
        'map_image': str(image_path),
        'world': str(Path(world).resolve()) if world else None,
        'slice_height_m': float(height),
        'declared_limits': {
            'extent_tolerance_cells': EXTENT_TOLERANCE_CELLS,
            'decorative_tolerance_cells': DECORATIVE_TOLERANCE_CELLS,
            'disconnected_free_regions': 'reported, not failed; the export is a '
                                         'seed-connected preview until navigation '
                                         'registration proves otherwise',
            'scope': 'artifact review only; no robot, simulator or mission is run',
        },
        'checks': {},
    }
    if title:
        report['title'] = title

    report['checks']['metadata'] = check_metadata(metadata)
    report['checks']['image_integrity'] = check_image(metadata, image)
    report['checks']['cell_alignment'] = check_cell_alignment(metadata, image)
    report['checks']['freshness'] = check_freshness(map_yaml, metadata, image, world, generation)
    report['checks']['seed_free'] = check_seed(metadata, image, seed)
    report['checks']['free_regions'] = check_free_regions(metadata, image, free, seed)
    if world is None:
        for name in ('extent_agreement', 'geometry_agreement', 'rotated_geometry',
                     'decorative_occupied'):
            report['checks'][name] = {'status': 'skipped', 'problems': [],
                                      'warnings': ['no --world supplied']}
    else:
        shapes, skipped = _world_shapes(world)
        rejected = [note for note in skipped if not note.startswith('actor ')]
        report['world_static_shapes'] = len(shapes)
        report['world_skipped_notes'] = skipped
        if rejected:
            failed = {'status': 'fail', 'problems': ['world geometry unresolved: '
                                                     + '; '.join(rejected)]}
            report['checks']['extent_agreement'] = failed
            for name in ('geometry_agreement', 'rotated_geometry', 'decorative_occupied'):
                report['checks'][name] = {'status': 'skipped', 'problems': [],
                                          'warnings': ['world geometry unresolved']}
        else:
            report['checks']['extent_agreement'] = check_extent(metadata, image, shapes,
                                                                generation)
            geometry, rotated, stats, footprint = check_geometry(
                metadata, image, free, occupied, shapes, float(height))
            report['checks']['geometry_agreement'] = dict(geometry, **stats)
            report['checks']['rotated_geometry'] = rotated
            report['checks']['decorative_occupied'] = check_decorative(
                metadata, image, occupied, footprint)
        report['dynamic_actor_notes_skipped'] = [n for n in skipped if n.startswith('actor ')]

    failing = [name for name, result in report['checks'].items()
               if result.get('status') == 'fail']
    warning = [name for name, result in report['checks'].items()
               if result.get('status') == 'warn']
    report['failed_checks'] = failing
    report['warned_checks'] = warning
    report['passed'] = not failing
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, required=True, help='map YAML to review')
    parser.add_argument('--world', type=Path, default=None,
                        help='source SDF world for geometry agreement checks')
    parser.add_argument('--seed', type=float, nargs=2, default=None, metavar=('X', 'Y'),
                        help='free-space seed (defaults to the generation sidecar)')
    parser.add_argument('--height', type=float, default=None,
                        help='slice height in metres (defaults to the sidecar, else 0.3)')
    parser.add_argument('--title', default=None, help='label recorded in the report')
    parser.add_argument('--out', type=Path, default=None, help='write the JSON report here')
    args = parser.parse_args()
    report = review(args.map, world=args.world, seed=args.seed,
                    height=args.height, title=args.title)
    text = json.dumps(report, indent=2) + '\n'
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)
    print(text, end='')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())






