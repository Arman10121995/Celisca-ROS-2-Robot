"""Generated terrain worlds for all four simulators (ROADMAP R6.7).

One elevation field, from a seed or an imported elevation raster, becomes:

* ``worlds/<id>.world`` + ``worlds/<id>.png``: an SDF ``<heightmap>`` that
  Gazebo reads natively and PyBullet/Isaac convert through
  ``robot_lab_utils.heightfield`` at spawn;
* ``mjcf/<id>.xml``: MuJoCo's hfield scene, from ``gen_mjcf_worlds.convert_world``
  (the MuJoCo launch finds it beside ``worlds/``);
* ``maps/map.yaml`` + ``map.pgm``: a 2D occupancy grid that marks slopes and
  steps a ground robot cannot take, and the terrain edge;
* ``manifest.json``: every input, the provider and attribution, the datum,
  metres per pixel, elevation range and the sha256 of each output, so the
  terrain regenerates identically (``regenerate``).

Everything is written to ``$ROBOT_LAB_RUNTIME_ROOT/terrain/<id>/`` and
registered in ``$ROBOT_LAB_RUNTIME_ROOT/terrain/maps.yaml``, which
``installed_assets.installed_profiles('maps')`` merges into the map list.

Raster conventions are the ones measured against Gazebo (``heightfield.py``):
intensity is normalised by the raster maximum, row 0 is +y, samples are
cell-centred. The raster is 8-bit, so heights are quantised to relief/255;
occupancy and support heights are computed from the quantised field the
simulators actually load.
"""
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import tempfile

import numpy as np

SCHEMA = 'robot_lab.terrain.v1'
GENERATOR_VERSION = 1
SAMPLES = (33, 65, 129, 257, 513)      # 2^n + 1, the heightmap convention
ID_PATTERN = re.compile(r'^[a-z][a-z0-9_]{2,47}$')
RASTER_SUFFIXES = ('.tif', '.tiff', '.npy', '.png')
FREE, OCCUPIED = 254, 0

STYLES = ('spectral', 'rolling_hills', 'ridges', 'rough', 'crater')

DEFAULTS = dict(provider='procedural', style='spectral', seed=7, size_m=40.0, samples=129, relief_m=2.0,
                smoothness=3.0, min_wavelength_m=4.0, roughness=0.5, spawn_x=0.0, spawn_y=0.0,
                spawn_pad_m=1.5, max_slope_deg=15.0, max_step_m=0.08,
                occupancy_resolution_m=0.1, friction=1.0)


def terrain_root():
    return Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'terrain'


def _finite(spec, key, low, high):
    value = spec[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('%s must be a finite number' % key)
    if not low <= value <= high:
        raise ValueError('%s must be within %g..%g' % (key, low, high))
    return float(value)


def validate(spec):
    """A complete, bounded spec (defaults filled in) or ValueError."""
    spec = dict(DEFAULTS, **(spec or {}))
    if not isinstance(spec.get('id'), str) or not ID_PATTERN.match(spec['id']):
        raise ValueError('id: 3-48 characters, lower-case letters, digits and _ , starting with a letter')
    if spec['provider'] not in ('procedural', 'raster'):
        raise ValueError('provider must be procedural or raster')
    if spec['samples'] not in SAMPLES:
        raise ValueError('samples must be one of %s' % (SAMPLES,))
    spec['size_m'] = _finite(spec, 'size_m', 4., 500.)
    spec['relief_m'] = _finite(spec, 'relief_m', .05, 200.)
    spec['spawn_pad_m'] = _finite(spec, 'spawn_pad_m', 0., 10.)
    spec['max_slope_deg'] = _finite(spec, 'max_slope_deg', 2., 45.)
    spec['max_step_m'] = _finite(spec, 'max_step_m', .01, 1.)
    spec['occupancy_resolution_m'] = _finite(spec, 'occupancy_resolution_m', .05, .5)
    spec['friction'] = _finite(spec, 'friction', .1, 2.)
    half = spec['size_m'] / 2 - spec['spawn_pad_m'] - spec['size_m'] / spec['samples']
    for key in ('spawn_x', 'spawn_y'):
        if abs(_finite(spec, key, -250., 250.)) > max(half, 0.):
            raise ValueError('%s must leave the spawn pad inside the terrain' % key)
        spec[key] = float(spec[key])
    if spec['provider'] == 'procedural':
        if isinstance(spec['seed'], bool) or not isinstance(spec['seed'], int) or not 0 <= spec['seed'] < 2**31:
            raise ValueError('seed must be an integer 0..2^31-1')
        if spec['style'] not in STYLES:
            raise ValueError('style must be one of %s' % ', '.join(STYLES))
        spec['roughness'] = _finite(spec, 'roughness', 0., 1.)
        spec['smoothness'] = _finite(spec, 'smoothness', 1.5, 4.5)
        cell = spec['size_m'] / spec['samples']
        spec['min_wavelength_m'] = _finite(spec, 'min_wavelength_m', 2 * cell, spec['size_m'])
    else:
        source = spec.get('raster_path')
        if not isinstance(source, str) or not Path(source).is_file() or Path(source).suffix.lower() not in RASTER_SUFFIXES:
            raise ValueError('raster_path must be an existing %s file' % '/'.join(RASTER_SUFFIXES))
        spec['source_metres_per_pixel'] = _finite(spec, 'source_metres_per_pixel', .01, 1000.)
        for key in ('attribution', 'license'):
            if not isinstance(spec.get(key), str) or not spec[key].strip():
                raise ValueError('an imported raster needs its %s (provider terms travel with the terrain)' % key)
        spec['source_sha256'] = hashlib.sha256(Path(source).read_bytes()).hexdigest()
    return spec


# ── Elevation fields ─────────────────────────────────────────────────────────

def _value_noise(samples, cells, rng):
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


def _fractal(samples, rng, base_cells, octaves, persistence):
    total, amplitude, norm = np.zeros((samples, samples)), 1.0, 0.0
    for octave in range(octaves):
        total += amplitude * _value_noise(samples, base_cells * 2 ** octave, rng)
        norm += amplitude
        amplitude *= persistence
    return total / norm


def style_field(style, samples, seed, roughness):
    """Unit-range elevation of a named value-noise style (row 0 = +y):
    rolling hills, sharp ridges, rough ground, or a crater (a rim around a bowl)."""
    rng = np.random.RandomState(seed)
    roughness = float(np.clip(roughness, 0.0, 1.0))
    persistence = 0.35 + 0.35 * roughness
    u = np.linspace(-1.0, 1.0, samples)
    xx, yy = np.meshgrid(u, -u)
    if style == 'rolling_hills':
        z = _fractal(samples, rng, 2, 4, persistence)
    elif style == 'ridges':
        z = (1.0 - np.abs(_fractal(samples, rng, 2, 5, persistence))) ** 2
    elif style == 'rough':
        z = _fractal(samples, rng, 4, 6, 0.5 + 0.3 * roughness)
    else:   # crater
        r = np.hypot(xx, yy)
        z = np.exp(-((r - 0.55) / 0.18) ** 2) - 0.6 * np.exp(-(r / 0.35) ** 2)
        z = z + 0.25 * roughness * _fractal(samples, rng, 4, 4, 0.5)
    z = z - z.min()
    return z / z.max() if z.max() > 0 else z


def procedural_field(spec):
    """The ``spectral`` style, or a named value-noise style (``style_field``).

    Spectral fractal terrain: power-law spectrum, random phases from the seed.

    ``smoothness`` is the spectral exponent (larger = smoother, 2 is rough
    hills, 4 rolling ones); wavelengths shorter than ``min_wavelength_m`` are
    removed so the surface has no bumps smaller than a wheel can ride.
    """
    if spec.get('style', 'spectral') != 'spectral':
        return style_field(spec['style'], spec['samples'], spec['seed'], spec['roughness'])
    n, size = spec['samples'], spec['size_m']
    rng = np.random.RandomState(spec['seed'])
    frequency_x = np.fft.fftfreq(n, d=size / n)[None, :]
    frequency_y = np.fft.fftfreq(n, d=size / n)[:, None]
    radial = np.hypot(frequency_x, frequency_y)
    radial[0, 0] = np.inf
    amplitude = radial ** (-spec['smoothness'] / 2.)
    amplitude[radial > 1. / spec['min_wavelength_m']] = 0.
    spectrum = amplitude * (rng.normal(size=(n, n)) + 1j * rng.normal(size=(n, n)))
    field = np.real(np.fft.ifft2(spectrum))
    if not np.ptp(field) > 0:
        raise ValueError('min_wavelength_m removes every feature; lower it')
    return field


def raster_field(spec):
    """The imported elevation raster (metres), centre-cropped and resampled."""
    from scipy.ndimage import zoom
    path = Path(spec['raster_path'])
    suffix = path.suffix.lower()
    if suffix in ('.tif', '.tiff'):
        import tifffile
        data = tifffile.imread(str(path))
    elif suffix == '.npy':
        data = np.load(str(path), allow_pickle=False)
    else:
        from PIL import Image
        with Image.open(path) as image:
            data = np.asarray(image)
    data = np.asarray(data, dtype=float)
    if data.ndim == 3:
        data = data[..., 0]
    if data.ndim != 2 or min(data.shape) < 8 or data.size > 64e6:
        raise ValueError('expected one 2D elevation band of at least 8 x 8 and at most 64 M samples')
    valid = np.isfinite(data) & (data > -1e4) & (data < 1e4)
    if valid.mean() < .99:
        raise ValueError('more than 1 % of the raster is no-data; crop it to a complete area first')
    data = np.where(valid, data, np.nanmedian(np.where(valid, data, np.nan)))
    needed = int(math.ceil(spec['size_m'] / spec['source_metres_per_pixel']))
    if needed > min(data.shape):
        raise ValueError('raster covers %.0f m; size_m %.0f m needs %d pixels per side'
                         % (min(data.shape) * spec['source_metres_per_pixel'], spec['size_m'], needed))
    top, left = (data.shape[0] - needed) // 2, (data.shape[1] - needed) // 2
    crop = data[top:top + needed, left:left + needed]
    return zoom(crop, spec['samples'] / needed, order=1, mode='nearest')[:spec['samples'], :spec['samples']]


def sample_coordinates(spec):
    """Cell-centred sample positions; row 0 is +y (measured Gazebo layout)."""
    n, size = spec['samples'], spec['size_m']
    xs = (np.arange(n) + .5) * size / n - size / 2.
    ys = size / 2. - (np.arange(n) + .5) * size / n
    return np.meshgrid(xs, ys)


def shape_heights(spec, field):
    """Scale to [0, relief], flatten the spawn pad, quantise as the simulators will."""
    grid_x, grid_y = sample_coordinates(spec)
    field = np.asarray(field, dtype=float)
    field = field - field.min()
    if spec['provider'] == 'procedural':
        field = field * (spec['relief_m'] / field.max())
    elif field.max() > spec['relief_m'] + 1e-9:
        # Real terrain is never compressed: that would change every slope.
        raise ValueError('the imported area rises %.1f m, more than the %.1f m height range; raise the height '
                         'range (8-bit steps of range/255) or choose a smaller area' % (field.max(), spec['relief_m']))
    if spec['spawn_pad_m'] > 0:
        distance = np.hypot(grid_x - spec['spawn_x'], grid_y - spec['spawn_y'])
        centre = float(field[np.unravel_index(np.argmin(distance), distance.shape)])
        blend = max(spec['spawn_pad_m'], 3 * spec['size_m'] / spec['samples'])
        weight = np.clip(1 - (distance - spec['spawn_pad_m']) / blend, 0, 1)
        weight = weight * weight * (3 - 2 * weight)            # smoothstep: no new step at the pad edge
        field = field * (1 - weight) + centre * weight
        field = field - field.min()
    peak = float(field.max())
    if peak <= 0:
        raise ValueError('the terrain is flat; raise relief_m or import another area')
    intensity = np.rint(field / peak * 255.).astype(np.uint8)
    heights = intensity.astype(float) * (peak / 255.)
    return intensity, heights, peak


# ── Outputs ──────────────────────────────────────────────────────────────────

def world_sdf(spec, raster_name, peak, mesh_name):
    geometry = '<heightmap><uri>%s</uri><size>%g %g %.6g</size></heightmap>' % (
        raster_name, spec['size_m'], spec['size_m'], peak)
    # Fortress' heightmap renderer (Ogre2 Terra) aborts compiling its shader on
    # this Jetson when a camera renders it (2026-11-13), so the visual is the
    # same grid as a mesh; collision stays the measured <heightmap>.
    visual = '<mesh><uri>%s</uri></mesh>' % mesh_name
    return """<?xml version="1.0"?>
<!-- Generated by robot_lab_utils.terrain_generator (%s) - regenerate, do not edit. -->
<sdf version="1.7">
  <world name="%s">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
    </physics>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <direction>-0.5 0.3 -1.0</direction>
    </light>
    <model name="terrain">
      <static>true</static>
      <!-- Elevation is measured from this pose; the lowest sample is at z = 0. -->
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>%s</geometry>
          <surface>
            <friction><ode><mu>%g</mu><mu2>%g</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>%s</geometry>
          <material>
            <ambient>0.42 0.5 0.32 1</ambient>
            <diffuse>0.42 0.5 0.32 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
""" % (SCHEMA, spec['id'], geometry, spec['friction'], spec['friction'], visual)


def terrain_mesh(raster_bytes, size, title):
    """OBJ bytes of an SDF heightmap raster through the shared converter (the
    layout the other backends collide with). ``size`` is the SDF <size>
    (x, y, z); rasters over 257 samples per side are halved for the visual."""
    from .heightfield import heightfield_grid
    with tempfile.NamedTemporaryFile('wb', suffix='.png', delete=False) as handle:
        handle.write(raster_bytes)
        path = handle.name
    try:
        from PIL import Image
        with Image.open(path) as image:
            large = max(image.size) > 257
        vertices, faces = heightfield_grid(dict(heightmap=path, size=[float(v) for v in size]),
                                           samples=2 if large else None)
    finally:
        os.unlink(path)
    lines = ['# Visual mesh of %s (heightfield.py layout; collision stays the SDF heightmap)' % title]
    lines += ['v %.5f %.5f %.5f' % tuple(vertex) for vertex in vertices]
    lines += ['f %d %d %d' % (a + 1, b + 1, c + 1) for a, b, c in faces]
    return ('\n'.join(lines) + '\n').encode()


def occupancy(spec, heights):
    """Cells a ground robot cannot drive: steeper than max_slope_deg, a step
    above max_step_m between neighbouring samples, or off the terrain."""
    cell = spec['size_m'] / spec['samples']
    gradient_y, gradient_x = np.gradient(heights, cell)
    slope = np.degrees(np.arctan(np.hypot(gradient_x, gradient_y)))
    step = np.zeros_like(heights)
    step[:, 1:] = np.maximum(step[:, 1:], np.abs(np.diff(heights, axis=1)))
    step[:, :-1] = np.maximum(step[:, :-1], np.abs(np.diff(heights, axis=1)))
    step[1:, :] = np.maximum(step[1:, :], np.abs(np.diff(heights, axis=0)))
    step[:-1, :] = np.maximum(step[:-1, :], np.abs(np.diff(heights, axis=0)))
    # A step is a jump a drivable slope cannot explain across one cell: on a coarse grid a
    # gentle slope alone changes the height by more than max_step_m between samples.
    step_limit = max(spec['max_step_m'], cell * math.tan(math.radians(spec['max_slope_deg'])))
    blocked = (slope > spec['max_slope_deg']) | (step > step_limit)
    resolution = spec['occupancy_resolution_m']
    half = spec['size_m'] / 2.
    columns = rows = int(round(spec['size_m'] / resolution))
    centres = (np.arange(columns) + .5) * resolution - half
    # Nearest terrain sample for each map cell (map row 0 is the top, +y).
    sample_columns = np.clip(((centres + half) / cell).astype(int), 0, spec['samples'] - 1)
    sample_rows = np.clip(((half - centres) / cell).astype(int), 0, spec['samples'] - 1)
    grid = np.where(blocked[np.ix_(sample_rows, sample_columns)], OCCUPIED, FREE).astype(np.uint8)
    edge = max(1, int(math.ceil(cell / resolution)))
    grid[:edge, :] = grid[-edge:, :] = OCCUPIED
    grid[:, :edge] = grid[:, -edge:] = OCCUPIED
    pgm = b'P5\n%d %d\n255\n' % (columns, rows) + grid.tobytes()
    yaml_text = ('image: map.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0.000000]\n'
                 'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n' % (resolution, -half, -half))
    stats = dict(max_slope_deg=round(float(slope.max()), 2), blocked_fraction=round(float(blocked.mean()), 4),
                 occupied_cells=int((grid == OCCUPIED).sum()), cells=int(grid.size))
    return pgm, yaml_text, stats, blocked


def preview_png(heights, peak, blocked=None, spawn_cell=None):
    """Hill-shaded elevation for the GUI (north up), blocked samples in red,
    the spawn as a green square."""
    from PIL import Image
    gradient_y, gradient_x = np.gradient(heights)
    shade = np.clip(.6 + .4 * (gradient_x - gradient_y) / (np.abs(np.gradient(heights)).max() + 1e-9), 0, 1)
    tint = heights / peak
    rgb = np.stack([.35 + .45 * tint, .55 + .25 * tint, .30 + .20 * tint], axis=-1) * shade[..., None]
    pixels = np.rint(np.clip(rgb, 0, 1) * 255).astype(np.uint8)
    if blocked is not None:
        pixels[blocked] = (200, 60, 60)
    if spawn_cell is not None:
        row, column = spawn_cell
        pixels[max(row - 2, 0):row + 3, max(column - 2, 0):column + 3] = (40, 220, 60)
    image = Image.fromarray(pixels, 'RGB')
    buffer = io.BytesIO()
    image.save(buffer, format='PNG')
    return buffer.getvalue()


def _png(intensity):
    from PIL import Image
    buffer = io.BytesIO()
    Image.fromarray(intensity, 'L').save(buffer, format='PNG')
    return buffer.getvalue()


def _mjcf(world_path, name, asset_dir):
    """MuJoCo scene through the same converter every installed world uses."""
    import importlib.util
    candidates = []
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(Path(get_package_share_directory('robot_lab_maps')) / 'tools' / 'gen_mjcf_worlds.py')
    except Exception:
        pass
    candidates.append(Path(__file__).resolve().parents[2] / 'robot_lab_maps' / 'tools' / 'gen_mjcf_worlds.py')
    tool = next((path for path in candidates if path.is_file()), None)
    if tool is None:
        raise ValueError('robot_lab_maps/tools/gen_mjcf_worlds.py not found; build robot_lab_maps')
    module_spec = importlib.util.spec_from_file_location('robot_lab_gen_mjcf_worlds', tool)
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    text, skipped = module.convert_world(str(world_path), name, asset_dir=str(asset_dir))
    # Actors (people/traffic, Gazebo-only scripted motion) are not static geometry; anything else is an error.
    skipped = [note for note in skipped if not (note.startswith("actor '") and 'scripted motion' in note)]
    if skipped:
        raise ValueError('MuJoCo conversion skipped terrain geometry: ' + '; '.join(skipped))
    return text


def build(spec):
    """Every output as bytes (no files written): {relative path: bytes}, manifest."""
    spec = validate(spec)
    field = procedural_field(spec) if spec['provider'] == 'procedural' else raster_field(spec)
    intensity, heights, peak = shape_heights(spec, field)
    name = spec['id']
    raster_name, mesh_name = name + '.png', name + '_visual.obj'
    raster = _png(intensity)
    outputs = {
        'worlds/%s.world' % name: world_sdf(spec, raster_name, peak, mesh_name).encode(),
        'worlds/' + raster_name: raster,
        'worlds/' + mesh_name: terrain_mesh(raster, (spec['size_m'], spec['size_m'], peak), name),
    }
    pgm, map_yaml, stats, blocked = occupancy(spec, heights)
    outputs['maps/map.pgm'] = pgm
    outputs['maps/map.yaml'] = map_yaml.encode()
    grid_x, grid_y = sample_coordinates(spec)
    spawn_index = np.unravel_index(np.argmin(np.hypot(grid_x - spec['spawn_x'], grid_y - spec['spawn_y'])),
                                   grid_x.shape)
    outputs['preview.png'] = preview_png(heights, peak, blocked, spawn_index)
    spawn_z = float(heights[spawn_index])
    cell = spec['size_m'] / spec['samples']
    manifest = dict(
        schema=SCHEMA, generator_version=GENERATOR_VERSION, id=name, inputs=spec,
        provider=dict(name=spec['provider'],
                      attribution=spec.get('attribution', 'Procedural terrain generated by Robot Lab (no external data)'),
                      license=spec.get('license', 'Generated; no third-party terms'),
                      source_sha256=spec.get('source_sha256'),
                      source_metres_per_pixel=spec.get('source_metres_per_pixel')),
        datum=dict(frame='map (ENU, metres)', origin='terrain centre at x=0, y=0; lowest sample at z=0',
                   polygon_m=[[-spec['size_m'] / 2, -spec['size_m'] / 2], [spec['size_m'] / 2, -spec['size_m'] / 2],
                              [spec['size_m'] / 2, spec['size_m'] / 2], [-spec['size_m'] / 2, spec['size_m'] / 2]]),
        raster=dict(samples=spec['samples'], metres_per_pixel=cell,
                    span_m=(spec['samples'] - 1) * cell, quantisation_m=peak / 255., bits=8),
        elevation=dict(min_m=0., max_m=peak, spawn_m=spawn_z),
        spawn=dict(x=spec['spawn_x'], y=spec['spawn_y'], z=spawn_z + .1, yaw=0.),
        occupancy=stats,
        conventions='intensity / raster max * size_z; row 0 = +y; cell-centred samples (heightfield.py, measured)',
        scope='Generated terrain geometry. Driving, walking and flight on it are separate measured acceptance.',
    )
    return outputs, manifest, heights


def _write_atomic(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('wb', dir=str(path.parent), delete=False) as handle:
        handle.write(data)
        temporary = handle.name
    os.replace(temporary, str(path))


def existing_map_ids():
    """Every map id the launch knows except generated terrain."""
    import yaml
    ids = set()
    for candidate in _sim_maps_candidates():
        if candidate.is_file():
            ids |= set((yaml.safe_load(candidate.read_text()) or {}).get('maps', {}))
            break
    installed = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / \
        'external_assets' / 'installed' / 'maps.yaml'
    if installed.is_file():
        ids |= set((yaml.safe_load(installed.read_text()) or {}).get('maps', {}))
    return ids


def _sim_maps_candidates():
    paths = []
    try:
        from ament_index_python.packages import get_package_share_directory
        paths.append(Path(get_package_share_directory('robot_lab_bringup')) / 'config' / 'sim_maps.yaml')
    except Exception:
        pass
    paths.append(Path(__file__).resolve().parents[2] / 'robot_lab_bringup' / 'config' / 'sim_maps.yaml')
    return paths


def generate(spec, root=None, replace=False):
    """Write the terrain under ``root/<id>/``, register it, return the manifest."""
    root = Path(root) if root else terrain_root()
    outputs, manifest, _ = build(spec)
    name = manifest['id']
    if name in existing_map_ids():
        raise ValueError('%s is already a map id; choose another name' % name)
    folder = root / name
    if folder.exists() and not replace:
        raise ValueError('%s exists; choose another name or regenerate it from its manifest' % folder)
    for relative, data in outputs.items():
        _write_atomic(folder / relative, data)
    world = folder / 'worlds' / (name + '.world')
    mjcf_dir = folder / 'mjcf'
    mjcf_dir.mkdir(parents=True, exist_ok=True)
    mjcf = _mjcf(world, name, mjcf_dir).encode()
    _write_atomic(mjcf_dir / (name + '.xml'), mjcf)
    files = {relative: hashlib.sha256(data).hexdigest() for relative, data in outputs.items()}
    files['mjcf/%s.xml' % name] = hashlib.sha256(mjcf).hexdigest()
    manifest['files'] = files
    _write_atomic(folder / 'manifest.json', (json.dumps(manifest, indent=1, sort_keys=True) + '\n').encode())
    register(manifest, folder, root)
    return manifest


def regenerate(folder):
    """Rebuild from a saved manifest and compare every output byte for byte.

    Returns {relative path: True if identical}. Nothing is written.
    """
    folder = Path(folder)
    manifest = json.loads((folder / 'manifest.json').read_text())
    if manifest.get('schema') != SCHEMA:
        raise ValueError('not a terrain manifest')
    if 'real_world' in manifest:               # an imported place rebuilds from its saved downloads
        from .real_world_import import regenerate as regenerate_real_world
        return regenerate_real_world(folder)
    inputs = dict(manifest['inputs'])
    if inputs.get('provider') == 'raster':
        if hashlib.sha256(Path(inputs['raster_path']).read_bytes()).hexdigest() != inputs['source_sha256']:
            raise ValueError('the imported raster changed since generation')
    outputs, _, _ = build(inputs)
    result = {relative: manifest['files'].get(relative) == hashlib.sha256(data).hexdigest()
              for relative, data in outputs.items()}
    # The MJCF names its hfield PNG after the world's absolute path, so it is
    # rebuilt from the registered world, and only once that world is identical.
    name = manifest['id']
    world = folder / 'worlds' / (name + '.world')
    same_world = all(result['worlds/%s%s' % (name, suffix)] for suffix in ('.world', '.png', '_visual.obj')) and \
        world.is_file() and world.read_bytes() == outputs['worlds/%s.world' % name]
    mjcf = _mjcf(world, name, folder / 'mjcf') if same_world else ''
    result['mjcf/%s.xml' % name] = bool(mjcf) and manifest['files'].get('mjcf/%s.xml' % name) == \
        hashlib.sha256(mjcf.encode()).hexdigest()
    return result


def map_profile(manifest, folder):
    folder = Path(folder)
    name, spawn = manifest['id'], manifest['spawn']
    return dict(
        gazebo=dict(world_name=name, world_package='robot_lab_maps',
                    world_path=str(folder / 'worlds' / (name + '.world'))),
        map=dict(has_2d_map=True, package='robot_lab_maps', path=str(folder / 'maps' / 'map.yaml')),
        spawn={key: '%.4f' % spawn[key] for key in ('x', 'y', 'z', 'yaw')},
        initial_pose={key: '%.4f' % spawn[key] for key in ('x', 'y', 'yaw')},
        terrain=dict(manifest=str(folder / 'manifest.json'), provider=manifest['provider']['name'],
                     relief_m=round(manifest['elevation']['max_m'], 4),
                     metres_per_pixel=round(manifest['raster']['metres_per_pixel'], 4)),
        notes='Generated terrain (%s, %s). Occupancy marks slopes over %g deg and steps over %g m. '
              'Driving, walking and flight on it are not yet measured.' % (
                  manifest['provider']['name'], manifest['provider']['attribution'],
                  manifest['inputs']['max_slope_deg'], manifest['inputs']['max_step_m']),
    )


def registry_entity(manifest, folder):
    """Registry environment entry (the launcher's composition check needs one)."""
    folder = Path(folder)
    name, inputs, spawn = manifest['id'], manifest['inputs'], manifest['spawn']
    provider = manifest['provider']
    return dict(
        id=name, version='1.0.0', name=name.replace('_', ' ').title(), status='available',
        dimension='3D', simulator='gazebo', simulators=['gazebo', 'mujoco', 'pybullet', 'isaac'],
        world_file=str(folder / 'worlds' / (name + '.world')),
        occupancy_map=str(folder / 'maps' / 'map.pgm'), occupancy_yaml=str(folder / 'maps' / 'map.yaml'),
        supported_robot_classes=['mobile', 'legged', 'humanoid', 'aerial', 'manipulator', 'hybrid'],
        spawn_zones=[dict(id='default', pose=dict(x=spawn['x'], y=spawn['y'], z=spawn['z']))],
        size=dict(x=inputs['size_m'], y=inputs['size_m'], z=manifest['elevation']['max_m']),
        dynamics=dict(static_obstacles=True, dynamic_obstacles=False),
        source=dict(repository='generated' if provider['name'] == 'procedural' else 'imported elevation raster',
                    license=provider['license']),
        description='Generated terrain (%s; %s). Support height of generated terrain is measured on four '
                    'simulators; robot missions on this terrain are not.' % (provider['name'], provider['attribution']),
        tags=['generated', 'terrain', provider['name']])


def registry_entities(root=None):
    """Registry entries for every registered terrain."""
    root = Path(root) if root else terrain_root()
    entities = []
    for name, profile in registered_profiles(root).items():
        manifest_path = Path(profile.get('terrain', {}).get('manifest', ''))
        if manifest_path.is_file():
            entities.append(registry_entity(json.loads(manifest_path.read_text()), manifest_path.parent))
    return entities


def register(manifest, folder, root=None):
    import yaml
    root = Path(root) if root else terrain_root()
    path = root / 'maps.yaml'
    data = (yaml.safe_load(path.read_text()) or {}) if path.is_file() else {}
    maps = data.setdefault('maps', {})
    maps[manifest['id']] = map_profile(manifest, folder)
    _write_atomic(path, yaml.safe_dump(data, sort_keys=True).encode())


def unregister(name, root=None):
    """Remove a terrain from the map list (its files stay on the SSD)."""
    import yaml
    path = (Path(root) if root else terrain_root()) / 'maps.yaml'
    if not path.is_file():
        return False
    data = yaml.safe_load(path.read_text()) or {}
    removed = data.get('maps', {}).pop(name, None) is not None
    _write_atomic(path, yaml.safe_dump(data, sort_keys=True).encode())
    return removed


def registered_profiles(root=None):
    """Registered terrain map profiles whose files still exist."""
    import yaml
    path = (Path(root) if root else terrain_root()) / 'maps.yaml'
    if not path.is_file():
        return {}
    maps = (yaml.safe_load(path.read_text()) or {}).get('maps', {}) or {}
    return {name: profile for name, profile in maps.items()
            if Path(profile.get('gazebo', {}).get('world_path', '')).is_file()
            and Path(profile.get('map', {}).get('path', '')).is_file()}


def main(argv=None):
    """Command line: generate and register one terrain (the GUI page does the same)."""
    import argparse
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('id', nargs='?')
    parser.add_argument('--spec', type=Path, help='a complete JSON spec (the GUI uses this); prints the '
                        'manifest, or {"error": ...} with exit 2')
    parser.add_argument('--style', choices=STYLES, default=DEFAULTS['style'])
    parser.add_argument('--raster', help='elevation raster (switches to the raster provider)')
    parser.add_argument('--metres-per-pixel', type=float, help='raster pixel size')
    parser.add_argument('--attribution')
    parser.add_argument('--license')
    for key in ('seed', 'samples'):
        parser.add_argument('--' + key.replace('_', '-'), type=int, default=DEFAULTS[key])
    for key in ('size_m', 'relief_m', 'smoothness', 'min_wavelength_m', 'roughness', 'spawn_x', 'spawn_y',
                'spawn_pad_m', 'max_slope_deg', 'max_step_m', 'occupancy_resolution_m', 'friction'):
        parser.add_argument('--' + key.replace('_', '-'), type=float, default=DEFAULTS[key])
    parser.add_argument('--replace', action='store_true', help='overwrite an existing terrain folder')
    parser.add_argument('--check', action='store_true', help='only compare an existing terrain with its manifest')
    args = parser.parse_args(argv)
    if args.spec:
        try:
            manifest = generate(json.loads(args.spec.read_text()), replace=args.replace)
        except (ValueError, OSError) as exc:
            print(json.dumps(dict(error=str(exc))))
            return 2
        print(json.dumps(dict(manifest=manifest)))
        return 0
    if not args.id:
        parser.error('a terrain id (or --spec) is required')
    if args.check:
        result = regenerate(terrain_root() / args.id)
        print(json.dumps(result, indent=1))
        return 0 if all(result.values()) else 1
    spec = {key: value for key, value in vars(args).items()
            if key in DEFAULTS and key != 'provider'}
    spec['id'] = args.id
    if args.raster:
        spec.update(provider='raster', raster_path=args.raster, source_metres_per_pixel=args.metres_per_pixel,
                    attribution=args.attribution, license=args.license)
    manifest = generate(spec, replace=args.replace)
    print(json.dumps(dict(id=manifest['id'], folder=str(terrain_root() / manifest['id']),
                          elevation=manifest['elevation'], occupancy=manifest['occupancy']), indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
