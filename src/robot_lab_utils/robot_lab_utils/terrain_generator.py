"""Procedural terrain worlds for every backend (ROADMAP R6.7).

``generate()`` writes one terrain world under ``<out_root>/<name>/``:

* ``worlds/<name>.world`` - an SDF world whose ground is a ``<heightmap>``,
  and ``worlds/terrain.png``, its 8-bit raster;
* ``maps/map.yaml`` + ``map.pgm`` - occupancy from the same elevation field:
  a cell is an obstacle where the slope is steeper than ``max_slope_deg``
  (a ground robot's limit), not where the ground is high;
* ``preview.png`` - shaded relief with the occupancy and the spawn;
* ``manifest.json`` - every input, the generator version, the conventions,
  the sha256 of each output and the attribution (procedural: no provider data).

The raster follows the conventions measured against the installed Gazebo
(``robot_lab_utils.heightfield``): intensity is normalised by the raster's own
maximum, row 0 is maximum +y, samples are cell-centred over ``(n-1)/2 *
size/n``.  MuJoCo, PyBullet and Isaac read the same ``<heightmap>`` through
``heightfield.py``, so one world serves all four backends.

Generation is deterministic: the same inputs give byte-identical files.
Provider elevation data (DEM tiles) is not used here; a provider workflow
would write the same files plus its attribution and must keep tokens out of
the repository and logs.
"""

import hashlib
import io
import json
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np

VERSION = 1
STYLES = ('rolling_hills', 'ridges', 'rough', 'crater')
RASTER_NAME = 'terrain.png'


def _value_noise(samples: int, cells: int, rng: np.random.RandomState) -> np.ndarray:
    """Smoothly interpolated random lattice (cells x cells) sampled on a
    samples x samples grid; values in [-1, 1]."""
    lattice = rng.uniform(-1.0, 1.0, (cells + 2, cells + 2))
    t = np.linspace(0.0, cells, samples)
    i = np.minimum(t.astype(int), cells - 1)
    f = t - i
    f = f * f * (3.0 - 2.0 * f)                      # smoothstep
    a = lattice[np.ix_(i, i)]
    b = lattice[np.ix_(i, i + 1)]
    c = lattice[np.ix_(i + 1, i)]
    d = lattice[np.ix_(i + 1, i + 1)]
    fx, fy = f[None, :], f[:, None]
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


def _fractal(samples: int, rng: np.random.RandomState, base_cells: int, octaves: int,
             persistence: float) -> np.ndarray:
    total, amplitude, norm = np.zeros((samples, samples)), 1.0, 0.0
    for octave in range(octaves):
        total += amplitude * _value_noise(samples, base_cells * 2 ** octave, rng)
        norm += amplitude
        amplitude *= persistence
    return total / norm


def elevation_field(style: str, samples: int, seed: int, roughness: float) -> np.ndarray:
    """Unit-range elevation (0..1) of ``style`` on a samples x samples grid,
    row 0 = maximum +y."""
    if style not in STYLES:
        raise ValueError('style must be one of %s' % ', '.join(STYLES))
    rng = np.random.RandomState(seed)
    roughness = float(np.clip(roughness, 0.0, 1.0))
    persistence = 0.35 + 0.35 * roughness
    u = np.linspace(-1.0, 1.0, samples)
    xx, yy = np.meshgrid(u, -u)
    if style == 'rolling_hills':
        z = _fractal(samples, rng, 2, 4, persistence)
    elif style == 'ridges':
        z = 1.0 - np.abs(_fractal(samples, rng, 2, 5, persistence))
        z = z ** 2
    elif style == 'rough':
        z = _fractal(samples, rng, 4, 6, 0.5 + 0.3 * roughness)
    else:   # crater: a rim around a bowl, with some fractal texture
        r = np.hypot(xx, yy)
        z = np.exp(-((r - 0.55) / 0.18) ** 2) - 0.6 * np.exp(-(r / 0.35) ** 2)
        z = z + 0.25 * roughness * _fractal(samples, rng, 4, 4, 0.5)
    z = z - z.min()
    return z / z.max() if z.max() > 0 else z


def _flatten_pad(z: np.ndarray, size: float, spawn: Tuple[float, float], radius: float) -> np.ndarray:
    """Blend the field to the spawn point's height within ``radius`` metres
    (so a robot spawns level), smoothly over another radius."""
    n = z.shape[0]
    cell = size / n
    coords = (np.arange(n) + 0.5) * cell - size / 2.0
    xx, yy = np.meshgrid(coords, coords[::-1])
    d = np.hypot(xx - spawn[0], yy - spawn[1])
    col = int(np.clip(round((spawn[0] + size / 2.0) / cell - 0.5), 0, n - 1))
    row = int(np.clip(round((size / 2.0 - spawn[1]) / cell - 0.5), 0, n - 1))
    level = z[row, col]
    w = np.clip((d - radius) / max(radius, 1e-6), 0.0, 1.0)
    w = w * w * (3.0 - 2.0 * w)
    return level + (z - level) * w


def slope_occupancy(metres: np.ndarray, size: float, max_slope_deg: float, resolution: float,
                    margin: float = 1.0):
    """Occupancy PGM bytes, map YAML text, occupied count and shape (columns,
    rows): cells steeper than ``max_slope_deg`` are obstacles; the border
    ``margin`` outside the terrain is free (flat)."""
    n = metres.shape[0]
    cell = size / n
    gy, gx = np.gradient(metres, cell)                 # row 0 is +y: sign does not matter for |grad|
    steep = np.degrees(np.arctan(np.hypot(gx, gy))) > max_slope_deg
    lo, hi = -size / 2.0 - margin, size / 2.0 + margin
    columns = rows = int(round((hi - lo) / resolution))
    xs = lo + (np.arange(columns) + 0.5) * resolution
    ys = hi - (np.arange(rows) + 0.5) * resolution
    ci = np.floor((xs + size / 2.0) / cell).astype(int)
    ri = np.floor((size / 2.0 - ys) / cell).astype(int)
    inside_c = (ci >= 0) & (ci < n)
    inside_r = (ri >= 0) & (ri < n)
    grid = np.full((rows, columns), 254, dtype=np.uint8)
    sub = steep[np.ix_(np.clip(ri, 0, n - 1), np.clip(ci, 0, n - 1))]
    mask = sub & inside_r[:, None] & inside_c[None, :]
    grid[mask] = 0
    pgm = b'P5\n%d %d\n255\n' % (columns, rows) + grid.tobytes()
    yaml_text = ('image: map.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0.000000]\n'
                 'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n' % (resolution, lo, lo))
    return pgm, yaml_text, int(mask.sum()), (columns, rows)


def world_sdf(name: str, size: float, height: float) -> str:
    geometry = '<heightmap><uri>%s</uri><size>%g %g %g</size></heightmap>' % (RASTER_NAME, size, size, height)
    return f'''<?xml version="1.0"?>
<!-- Generated by robot_lab_utils.terrain_generator (procedural; see manifest.json). -->
<sdf version="1.7">
  <world name="{name}">
    <physics type="ode">
      <max_step_size>0.01</max_step_size>
    </physics>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.3 -1.0</direction>
    </light>
    <model name="terrain">
      <static>true</static>
      <!-- Elevation is measured from this pose, not centred on it. -->
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>{geometry}</geometry>
          <surface>
            <friction><ode><mu>100</mu><mu2>50</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>{geometry}</geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
'''


def _png(array: np.ndarray, mode: str = 'L') -> bytes:
    from PIL import Image
    buffer = io.BytesIO()
    Image.fromarray(array, mode).save(buffer, format='PNG')
    return buffer.getvalue()


def _preview(metres: np.ndarray, pgm_grid: np.ndarray, size: float, spawn) -> bytes:
    """Hill-shaded relief (light from the north-west), steep cells in red,
    the spawn as a green square."""
    gy, gx = np.gradient(metres)
    shade = np.clip(0.5 + 0.5 * (-gx + gy) / (np.abs(gx).max() + np.abs(gy).max() + 1e-9) * 2, 0, 1)
    height = (metres - metres.min()) / max(float(np.ptp(metres)), 1e-9)
    rgb = np.stack([0.35 + 0.4 * height, 0.45 + 0.35 * height, 0.3 + 0.2 * height], -1) * (0.6 + 0.4 * shade[..., None])
    image = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
    from PIL import Image
    steep = Image.fromarray(pgm_grid).resize(image.shape[1::-1], Image.NEAREST)
    image[np.asarray(steep) == 0] = (200, 60, 60)
    n = image.shape[0]
    c = int((spawn[0] + size / 2.0) / size * n)
    r = int((size / 2.0 - spawn[1]) / size * n)
    image[max(r - 3, 0):r + 4, max(c - 3, 0):c + 4] = (40, 220, 60)
    return _png(image, 'RGB')


def generate(name: str, out_root, size: float = 20.0, height: float = 2.0, samples: int = 129,
             seed: int = 1, style: str = 'rolling_hills', roughness: float = 0.5,
             spawn: Tuple[float, float] = (0.0, 0.0), flat_radius: float = 1.5,
             max_slope_deg: float = 20.0, resolution: float = 0.05) -> Dict:
    """Write the terrain world; returns the manifest (also saved as JSON)."""
    if not name.replace('_', '').isalnum():
        raise ValueError('name: letters, digits and underscores only')
    if samples < 17 or (samples - 1) & (samples - 2):
        raise ValueError('samples must be 2^n + 1 (17, 33, 65, 129, 257 ...)')
    if not (2.0 <= size <= 500.0 and 0.0 < height <= 50.0):
        raise ValueError('size must be 2-500 m and height 0-50 m')
    if abs(spawn[0]) > size / 2.0 - flat_radius or abs(spawn[1]) > size / 2.0 - flat_radius:
        raise ValueError('the spawn pad must lie inside the terrain')
    unit = elevation_field(style, samples, seed, roughness)
    unit = _flatten_pad(unit, size, spawn, flat_radius)
    unit = unit - unit.min()
    raster = np.rint(unit / unit.max() * 255.0).astype(np.uint8) if unit.max() > 0 else \
        np.zeros_like(unit, dtype=np.uint8)
    metres = raster.astype(float) / max(int(raster.max()), 1) * height     # what Gazebo will build
    pgm, map_yaml, occupied, shape = slope_occupancy(metres, size, max_slope_deg, resolution)
    grid = np.frombuffer(pgm[pgm.index(b'255\n') + 4:], dtype=np.uint8).reshape(shape[1], shape[0])
    root = Path(out_root) / name
    files = {
        root / 'worlds' / ('%s.world' % name): world_sdf(name, size, height).encode('utf-8'),
        root / 'worlds' / RASTER_NAME: _png(raster),
        root / 'maps' / 'map.yaml': map_yaml.encode('utf-8'),
        root / 'maps' / 'map.pgm': pgm,
        root / 'preview.png': _preview(metres, grid, size, spawn),
    }
    for path, data in files.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    i, j = int(round((size / 2.0 - spawn[1]) / (size / samples) - 0.5)), int(round((spawn[0] + size / 2.0) / (size / samples) - 0.5))
    spawn_z = float(metres[min(max(i, 0), samples - 1), min(max(j, 0), samples - 1)])
    manifest = {
        'name': name, 'generator': 'robot_lab_utils.terrain_generator', 'version': VERSION,
        'provider': 'procedural', 'attribution': 'Procedurally generated by Robot Lab; no external elevation data',
        'inputs': {'size_m': size, 'height_m': height, 'samples': samples, 'seed': seed, 'style': style,
                   'roughness': roughness, 'spawn': list(spawn), 'flat_radius_m': flat_radius,
                   'max_slope_deg': max_slope_deg, 'resolution_m': resolution},
        'conventions': 'Gazebo heightmap: intensity / raster max * size_z; row 0 = +y; cell-centred samples',
        'vertical_datum': 'terrain minimum = 0 m at the collision pose',
        'metres_per_sample': size / samples,
        'elevation_range_m': [0.0, height],
        'spawn_z_m': round(spawn_z, 4),
        'occupancy': {'rule': 'slope > max_slope_deg', 'occupied_cells': occupied, 'shape': list(shape)},
        'files': {str(path.relative_to(root)): hashlib.sha256(data).hexdigest() for path, data in files.items()},
    }
    (root / 'manifest.json').write_text(json.dumps(manifest, indent=1, sort_keys=True) + '\n')
    return manifest


def map_profile(manifest: Dict, root) -> Dict:
    """A map profile (as in sim_maps.yaml) for a generated terrain."""
    root = Path(root) / manifest['name']
    sx, sy = manifest['inputs']['spawn']
    z = '%.3f' % (manifest['spawn_z_m'] + 0.1)
    return {
        'source_id': 'robot_lab_terrain', 'source_entry': 'procedural:%s seed %d' % (
            manifest['inputs']['style'], manifest['inputs']['seed']),
        'gazebo': {'world_package': 'robot_lab_maps', 'world_name': manifest['name'],
                   'world_path': str(root / 'worlds' / ('%s.world' % manifest['name']))},
        'map': {'has_2d_map': True, 'package': 'robot_lab_maps', 'path': str(root / 'maps' / 'map.yaml')},
        'spawn': {'x': str(sx), 'y': str(sy), 'z': z, 'yaw': '0.0'},
        'initial_pose': {'x': str(sx), 'y': str(sy), 'yaw': '0.0'},
        'notes': 'Generated terrain (%s, seed %d, %g m, %g m high). Occupancy marks slopes above %g deg. '
                 'Robot missions on it are not yet qualified.' % (
                     manifest['inputs']['style'], manifest['inputs']['seed'], manifest['inputs']['size_m'],
                     manifest['inputs']['height_m'], manifest['inputs']['max_slope_deg']),
    }


def register(manifest: Dict, root) -> Path:
    """Add the terrain to ``<root>/maps.yaml`` (read by
    robot_lab_utils.installed_assets with the installed worlds)."""
    import yaml
    path = Path(root) / 'maps.yaml'
    data = (yaml.safe_load(path.read_text()) or {}) if path.is_file() else {}
    data.setdefault('maps', {})[manifest['name']] = map_profile(manifest, root)
    path.write_text(yaml.safe_dump(data, sort_keys=True))
    return path


def default_root() -> Path:
    import os
    return Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'terrain'


def load_registered(root: Optional[Path] = None) -> Dict:
    import yaml
    path = Path(root or default_root()) / 'maps.yaml'
    if not path.is_file():
        return {}
    maps = (yaml.safe_load(path.read_text()) or {}).get('maps') or {}
    return {k: v for k, v in maps.items() if Path(v.get('gazebo', {}).get('world_path', '')).is_file()}


def main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('name')
    ap.add_argument('--style', choices=STYLES, default='rolling_hills')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--size', type=float, default=20.0)
    ap.add_argument('--height', type=float, default=2.0)
    ap.add_argument('--samples', type=int, default=129)
    ap.add_argument('--roughness', type=float, default=0.5)
    ap.add_argument('--spawn', type=float, nargs=2, default=(0.0, 0.0))
    ap.add_argument('--max-slope', type=float, default=20.0)
    ap.add_argument('--out-root', type=Path, default=None)
    ap.add_argument('--register', action='store_true', help='add it to the Launch world list')
    args = ap.parse_args(argv)
    root = args.out_root or default_root()
    manifest = generate(args.name, root, size=args.size, height=args.height, samples=args.samples, seed=args.seed,
                        style=args.style, roughness=args.roughness, spawn=tuple(args.spawn),
                        max_slope_deg=args.max_slope)
    if args.register:
        register(manifest, root)
    print(json.dumps({'world': str(Path(root) / args.name), 'occupied_cells': manifest['occupancy']['occupied_cells'],
                      'registered': bool(args.register)}))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
