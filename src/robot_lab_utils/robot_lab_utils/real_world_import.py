"""Real-world areas as worlds for all four simulators (ROADMAP R6.8).

A latitude/longitude box becomes a registered terrain world:

* elevation from the open **Terrain Tiles** (AWS Open Data, Terrarium
  encoding: no account or token), resampled onto a local metric grid and
  written through ``terrain_generator`` (SDF heightmap collision, mesh visual,
  MuJoCo hfield, occupancy, manifest);
* **OpenStreetMap** features from the Overpass API: buildings (extruded from
  their footprints, height from ``height`` / ``building:levels``), roads
  (visual ribbons draped on the ground), single trees and forests (trunk
  collision, crown visual), and water (visual, blocked in the 2D grid);
* optional **people and traffic** from Eclipse SUMO, run on the same OSM road
  network (``real_world_traffic``), as Gazebo actors.

Downloads are cached and saved with the world (``inputs/``), so it
regenerates from its own files. Every data source keeps its attribution in
the manifest. Coordinates: a local azimuthal-equidistant projection centred
on the area (pyproj), x east, y north, metres; the terrain's lowest sample is
z = 0.

Known limits: Fortress heightmaps are 8-bit (height steps of relief/255,
recorded in the manifest); MuJoCo collides with each building's convex hull
(L- and U-shaped buildings fill their courtyards there); roofs are flat.
"""
import hashlib
import io
import json
import math
import os
from pathlib import Path
import time
import urllib.error
import urllib.parse
import urllib.request

import numpy as np

from . import terrain_generator as tg

SCHEMA = 'robot_lab.real_world.v1'
VERSION = 1
TERRARIUM = 'https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png'
OVERPASS = ('https://overpass-api.de/api/interpreter', 'https://overpass.kumi.systems/api/interpreter')
USER_AGENT = 'RobotLab-terrain-import/1.0'       # no personal identifiers are ever sent
DEM_ATTRIBUTION = ('Terrain Tiles, AWS Open Data (Mapzen/Tilezen Joerd); sources include SRTM, NED, '
                   'ETOPO1, GMTED and others: https://github.com/tilezen/joerd/blob/master/docs/attribution.md')
DEM_LICENSE = 'Mixed open sources; attribution required (see the Joerd attribution document)'
OSM_ATTRIBUTION = '© OpenStreetMap contributors'
OSM_LICENSE = 'ODbL 1.0 (https://www.openstreetmap.org/copyright)'
MAX_SIDE_M, MIN_SIDE_M = 2000., 40.
LEVEL_HEIGHT_M, DEFAULT_BUILDING_M = 3.0, 6.0
ROAD_WIDTHS = {'motorway': 12., 'trunk': 10., 'primary': 9., 'secondary': 8., 'tertiary': 7., 'residential': 6.,
               'unclassified': 5., 'service': 4., 'living_street': 5., 'pedestrian': 4., 'footway': 2.,
               'path': 1.5, 'cycleway': 2., 'track': 3., 'steps': 2.}
MAX_BUILDINGS, MAX_TREES = 1500, 600


# ── area and projection ─────────────────────────────────────────────────────

def validate_area(south, west, north, east):
    values = [float(v) for v in (south, west, north, east)]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('finite coordinates required')
    south, west, north, east = values
    if not (-85 < south < north < 85 and -180 <= west < east <= 180):
        raise ValueError('need south < north (within ±85°) and west < east')
    projection = Projection((south + north) / 2, (west + east) / 2)
    corners = [projection.forward(lat, lon) for lat, lon in ((south, west), (south, east), (north, west), (north, east))]
    width = max(x for x, _ in corners) - min(x for x, _ in corners)
    height = max(y for _, y in corners) - min(y for _, y in corners)
    if max(width, height) > MAX_SIDE_M or min(width, height) < MIN_SIDE_M:
        raise ValueError('area must be %g–%g m per side (this one is %.0f × %.0f m)'
                         % (MIN_SIDE_M, MAX_SIDE_M, width, height))
    return (south, west, north, east), projection, width, height


class Projection:
    """Local azimuthal-equidistant metres around the area centre (x east, y north)."""

    def __init__(self, lat0, lon0):
        from pyproj import Transformer
        self.lat0, self.lon0 = float(lat0), float(lon0)
        self.definition = '+proj=aeqd +lat_0=%.9f +lon_0=%.9f +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs' % (
            self.lat0, self.lon0)
        self._forward = Transformer.from_crs('EPSG:4326', self.definition, always_xy=True)
        self._inverse = Transformer.from_crs(self.definition, 'EPSG:4326', always_xy=True)

    def forward(self, lat, lon):
        x, y = self._forward.transform(lon, lat)
        return float(x), float(y)

    def forward_many(self, lat, lon):
        x, y = self._forward.transform(np.asarray(lon), np.asarray(lat))
        return np.asarray(x), np.asarray(y)

    def inverse_many(self, x, y):
        lon, lat = self._inverse.transform(np.asarray(x), np.asarray(y))
        return np.asarray(lat), np.asarray(lon)


# ── downloads (cached) ──────────────────────────────────────────────────────

def _get(url, data=None, attempts=4):
    request = urllib.request.Request(url, data=data, headers={'User-Agent': USER_AGENT})
    for attempt in range(attempts):
        try:
            with urllib.request.urlopen(request, timeout=90) as response:
                return response.read()
        except urllib.error.HTTPError as exc:
            if exc.code not in (429, 502, 503, 504) or attempt == attempts - 1:
                raise
        except urllib.error.URLError:
            if attempt == attempts - 1:
                raise
        time.sleep(2 ** attempt * 3)                     # back off: shared public services


def cache_root():
    return Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'real_world_cache'


def tile_xy(lat, lon, zoom):
    n = 2 ** zoom
    lat, lon = np.asarray(lat, dtype=float), np.asarray(lon, dtype=float)
    x = (lon + 180.) / 360. * n
    y = (1. - np.arcsinh(np.tan(np.radians(lat))) / np.pi) / 2. * n
    return x, y


def fetch_dem(bounds, projection, size_m, samples, cache=None, progress=print):
    """Elevation (metres above sea level) on the terrain grid: (samples, samples), row 0 = +y."""
    from PIL import Image
    cache = Path(cache) if cache else cache_root() / 'terrarium'
    south, west, north, east = bounds
    cell = size_m / samples
    # Terrarium tiles are 256 px; choose the zoom whose pixels are no larger than the grid cells (max 15).
    lat_c = (south + north) / 2
    zoom = 15
    for candidate in range(10, 16):
        metres_per_pixel = 156543.03 * math.cos(math.radians(lat_c)) / 2 ** candidate
        if metres_per_pixel <= cell:
            zoom = candidate
            break
    grid = (np.arange(samples) + .5) * cell - size_m / 2
    gx, gy = np.meshgrid(grid, grid[::-1])                 # row 0 = +y
    lat, lon = projection.inverse_many(gx, gy)
    px, py = tile_xy(lat, lon, zoom)
    px, py = np.asarray(px) * 256 - .5, np.asarray(py) * 256 - .5
    tiles = {}
    needed = {(int(tx), int(ty)) for tx in range(int(px.min() // 256), int(px.max() // 256) + 2)
              for ty in range(int(py.min() // 256), int(py.max() // 256) + 2)}
    progress('Elevation: %d Terrain Tiles at zoom %d' % (len(needed), zoom))
    for tx, ty in sorted(needed):
        path = cache / str(zoom) / str(tx) / ('%d.png' % ty)
        if not path.is_file():
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(_get(TERRARIUM.format(z=zoom, x=tx, y=ty)))
        rgb = np.asarray(Image.open(path).convert('RGB'), dtype=float)
        tiles[tx, ty] = rgb[..., 0] * 256 + rgb[..., 1] + rgb[..., 2] / 256 - 32768.
    x0, y0 = min(t[0] for t in needed), min(t[1] for t in needed)
    x1, y1 = max(t[0] for t in needed), max(t[1] for t in needed)
    mosaic = np.zeros(((y1 - y0 + 1) * 256, (x1 - x0 + 1) * 256))
    for (tx, ty), data in tiles.items():
        mosaic[(ty - y0) * 256:(ty - y0 + 1) * 256, (tx - x0) * 256:(tx - x0 + 1) * 256] = data
    from scipy.ndimage import map_coordinates
    heights = map_coordinates(mosaic, [py - y0 * 256, px - x0 * 256], order=1, mode='nearest')
    return heights, dict(zoom=zoom, tiles=len(needed), source=TERRARIUM)


OVERPASS_QUERY = """[out:json][timeout:120];
(
  way["building"]({b});
  relation["building"]["type"="multipolygon"]({b});
  way["highway"]({b});
  node["natural"="tree"]({b});
  way["natural"~"^(wood|water)$"]({b});
  way["landuse"~"^(forest|reservoir|basin)$"]({b});
  way["waterway"="riverbank"]({b});
);
out body;
>;
out skel qt;"""


def fetch_osm(bounds, cache=None, progress=print):
    """OSM features in the box (Overpass JSON), cached by box."""
    south, west, north, east = bounds
    box = '%.6f,%.6f,%.6f,%.6f' % (south, west, north, east)
    cache = Path(cache) if cache else cache_root() / 'osm'
    path = cache / (hashlib.sha256(box.encode()).hexdigest()[:20] + '.json')
    if not path.is_file():
        progress('OpenStreetMap: querying Overpass for %s' % box)
        query = OVERPASS_QUERY.replace('{b}', box)
        data, error = None, None
        for endpoint in OVERPASS:
            try:
                data = _get(endpoint, data=urllib.parse.urlencode({'data': query}).encode())
                break
            except Exception as exc:                     # try the next public mirror
                error = exc
        if data is None:
            raise ValueError('Overpass unavailable: %s' % error)
        json.loads(data)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
    return json.loads(path.read_text())



# ── OSM → features in local metres ──────────────────────────────────────────

def _number(text):
    try:
        return float(str(text).split()[0].replace(',', '.').rstrip('m'))
    except (ValueError, IndexError):
        return None


def building_height(tags):
    height = _number(tags.get('height')) or _number(tags.get('building:height'))
    if height and 2 <= height <= 300:
        return height
    levels = _number(tags.get('building:levels'))
    if levels and 1 <= levels <= 100:
        return levels * LEVEL_HEIGHT_M + (1. if tags.get('roof:shape') not in (None, 'flat') else 0.)
    return DEFAULT_BUILDING_M


def extract_features(osm, projection, half):
    """Buildings, roads, trees and water inside the square (|x|,|y| < half), in metres."""
    from shapely.geometry import Polygon, LineString, box
    from shapely.ops import unary_union
    nodes = {element['id']: (element['lat'], element['lon']) for element in osm['elements'] if element['type'] == 'node'}
    ways = {element['id']: element for element in osm['elements'] if element['type'] == 'way'}
    area = box(-half, -half, half, half)

    def coordinates(node_ids):
        points = [nodes[n] for n in node_ids if n in nodes]
        if not points:
            return []
        x, y = projection.forward_many([p[0] for p in points], [p[1] for p in points])
        return list(zip(x.tolist(), y.tolist()))

    def polygon(node_ids):
        points = coordinates(node_ids)
        if len(points) < 4:
            return None
        shape = Polygon(points).buffer(0)
        shape = shape.intersection(area)
        return shape if not shape.is_empty and shape.area > 1. else None

    buildings, roads, trees, forests, water = [], [], [], [], []
    for element in osm['elements']:
        tags = element.get('tags') or {}
        if element['type'] == 'way' and 'building' in tags:
            shape = polygon(element.get('nodes', []))
            if shape is not None:
                buildings.append(dict(shape=shape, height=building_height(tags), id='w%d' % element['id'],
                                      name=tags.get('name', '')))
        elif element['type'] == 'relation' and 'building' in tags:
            outers = [polygon(ways[m['ref']]['nodes']) for m in element.get('members', [])
                      if m.get('type') == 'way' and m.get('role') == 'outer' and m['ref'] in ways]
            inners = [polygon(ways[m['ref']]['nodes']) for m in element.get('members', [])
                      if m.get('type') == 'way' and m.get('role') == 'inner' and m['ref'] in ways]
            outer = unary_union([o for o in outers if o is not None])
            if not outer.is_empty:
                shape = outer.difference(unary_union([i for i in inners if i is not None])) if inners else outer
                buildings.append(dict(shape=shape, height=building_height(tags), id='r%d' % element['id'],
                                      name=tags.get('name', '')))
        elif element['type'] == 'way' and 'highway' in tags and tags['highway'] in ROAD_WIDTHS:
            points = coordinates(element.get('nodes', []))
            if len(points) >= 2:
                line = LineString(points).intersection(area)
                if not line.is_empty and line.length > .5:
                    width = _number(tags.get('width')) or ROAD_WIDTHS[tags['highway']]
                    roads.append(dict(line=line, width=min(max(width, 1.), 30.), kind=tags['highway']))
        elif element['type'] == 'node' and tags.get('natural') == 'tree':
            x, y = projection.forward(element['lat'], element['lon'])
            if abs(x) < half - 1 and abs(y) < half - 1:
                trees.append(dict(x=x, y=y, height=min(max(_number(tags.get('height')) or 8., 3.), 40.)))
        elif element['type'] == 'way' and (tags.get('natural') == 'wood' or tags.get('landuse') == 'forest'):
            shape = polygon(element.get('nodes', []))
            if shape is not None:
                forests.append(shape)
        elif element['type'] == 'way' and (tags.get('natural') == 'water' or tags.get('waterway') == 'riverbank'
                                           or tags.get('landuse') in ('reservoir', 'basin')):
            shape = polygon(element.get('nodes', []))
            if shape is not None:
                water.append(shape)
    # Forest interiors get trees on a jittered 6 m lattice (seeded by the area), never inside buildings.
    footprints = unary_union([b['shape'] for b in buildings]) if buildings else None
    rng = np.random.RandomState(1729)
    for forest in forests:
        minx, miny, maxx, maxy = forest.bounds
        for x in np.arange(minx + 3, maxx, 6.):
            for y in np.arange(miny + 3, maxy, 6.):
                px, py = x + rng.uniform(-2, 2), y + rng.uniform(-2, 2)
                from shapely.geometry import Point
                point = Point(px, py)
                if forest.contains(point) and (footprints is None or not footprints.contains(point)) \
                        and abs(px) < half - 1 and abs(py) < half - 1:
                    trees.append(dict(x=float(px), y=float(py), height=float(rng.uniform(10, 20))))
    buildings.sort(key=lambda b: -b['shape'].area)
    return dict(buildings=buildings[:MAX_BUILDINGS], roads=roads, trees=trees[:MAX_TREES], water=water,
                forests=forests, truncated=dict(buildings=max(0, len(buildings) - MAX_BUILDINGS),
                                                trees=max(0, len(trees) - MAX_TREES)))


# ── geometry ────────────────────────────────────────────────────────────────

class Ground:
    """Bilinear height of the generated (quantised) terrain at metric (x, y)."""

    def __init__(self, heights, size_m):
        self.heights, self.size, self.n = heights, size_m, heights.shape[0]

    def __call__(self, x, y):
        cell = self.size / self.n
        column = np.clip((np.asarray(x) + self.size / 2) / cell - .5, 0, self.n - 1.001)
        row = np.clip((self.size / 2 - np.asarray(y)) / cell - .5, 0, self.n - 1.001)
        c0, r0 = np.floor(column).astype(int), np.floor(row).astype(int)
        fc, fr = column - c0, row - r0
        h = self.heights
        return ((1 - fr) * ((1 - fc) * h[r0, c0] + fc * h[r0, c0 + 1])
                + fr * ((1 - fc) * h[r0 + 1, c0] + fc * h[r0 + 1, c0 + 1]))


def _obj(vertices, faces, title):
    """OBJ with vertex normals: Fortress' DART/ODE collider reads mesh normals unconditionally
    and segfaults on a mesh without them (OdeMesh::fillArrays, 2026-11-13)."""
    import trimesh
    mesh = trimesh.Trimesh(vertices=np.asarray(vertices, dtype=float), faces=np.asarray(faces, dtype=int), process=False)
    normals = mesh.vertex_normals
    lines = ['# %s' % title] + ['v %.3f %.3f %.3f' % tuple(v) for v in mesh.vertices] + \
            ['vn %.4f %.4f %.4f' % tuple(n) for n in normals] + \
            ['f %d//%d %d//%d %d//%d' % (a + 1, a + 1, b + 1, b + 1, c + 1, c + 1) for a, b, c in mesh.faces]
    return ('\n'.join(lines) + '\n').encode()


def building_mesh(shape, base, height):
    """Footprint extruded from just below the lowest ground under it to its height above that ground."""
    import trimesh
    parts = [shape] if shape.geom_type == 'Polygon' else list(shape.geoms)
    meshes = []
    for part in parts:
        if part.area < 1.:
            continue
        mesh = trimesh.creation.extrude_polygon(part.simplify(.05), height + .5)
        mesh.apply_translation([0, 0, base - .5])         # sunk 0.5 m so sloped ground never shows a gap
        meshes.append(mesh)
    if not meshes:
        return None
    return trimesh.util.concatenate(meshes)


def ribbon(line, width, ground, lift=.04, step=2.):
    """A road strip draped on the ground (visual only)."""
    vertices, faces = [], []
    lines = [line] if line.geom_type == 'LineString' else [g for g in getattr(line, 'geoms', []) if g.geom_type == 'LineString']
    for part in lines:
        length = part.length
        if length < .5:
            continue
        distances = np.append(np.arange(0, length, step), length)
        points = np.array([part.interpolate(d).coords[0] for d in distances])
        tangents = np.gradient(points, axis=0)
        norms = np.linalg.norm(tangents, axis=1, keepdims=True)
        tangents = tangents / np.maximum(norms, 1e-9)
        normals = np.column_stack([-tangents[:, 1], tangents[:, 0]]) * width / 2
        left, right = points + normals, points - normals
        start = len(vertices)
        for a, b in zip(left, right):
            vertices.append((a[0], a[1], float(ground(a[0], a[1])) + lift))
            vertices.append((b[0], b[1], float(ground(b[0], b[1])) + lift))
        for i in range(len(points) - 1):
            k = start + 2 * i
            faces += [(k, k + 1, k + 3), (k, k + 3, k + 2)]
    return vertices, faces


def flat_patch(shape, ground, lift=.03):
    import trimesh
    parts = [shape] if shape.geom_type == 'Polygon' else list(shape.geoms)
    vertices, faces = [], []
    for part in parts:
        try:
            v2, f = trimesh.creation.triangulate_polygon(part.simplify(.2))
        except Exception:
            continue
        start = len(vertices)
        vertices += [(x, y, float(ground(x, y)) + lift) for x, y in v2]
        faces += [(a + start, b + start, c + start) for a, b, c in f]
    return vertices, faces


# ── world assembly ──────────────────────────────────────────────────────────

def _model(name, pose, link):
    return ('    <model name="%s">\n      <static>true</static>\n      <pose>%.3f %.3f %.3f 0 0 %.4f</pose>\n'
            '      <link name="link">\n%s      </link>\n    </model>\n' % ((name,) + tuple(pose) + (link,)))


def _mesh_geometry(uri):
    return '<geometry><mesh><uri>%s</uri></mesh></geometry>' % uri


# Roads and water are visual only. A model with no <collision> has its visuals used as collision
# by sdf_world (MuJoCo, PyBullet, Isaac), which lifted robots onto the 4 cm road ribbon there
# (2026-11-13); this far-underground placeholder keeps them visual-only on every backend.
NO_COLLISION = ('        <collision name="visual_only_placeholder"><pose>0 0 -100 0 0 0</pose>'
                '<geometry><box><size>0.01 0.01 0.01</size></box></geometry></collision>\n')


def _material(rgba):
    return '<material><ambient>%s</ambient><diffuse>%s</diffuse></material>' % (rgba, rgba)


def assemble(world_text, name, features, ground, extra_models=''):
    """World SDF with the OSM models inserted, and the extra files {relative path: bytes}."""
    files, models = {}, []
    count = 0
    for building in features['buildings']:
        shape = building['shape']
        xs = np.linspace(shape.bounds[0], shape.bounds[2], 6)
        ys = np.linspace(shape.bounds[1], shape.bounds[3], 6)
        from shapely.geometry import Point
        samples = [float(ground(x, y)) for x in xs for y in ys if shape.buffer(.5).contains(Point(x, y))]
        samples += [float(ground(*shape.exterior.coords[0]))] if shape.geom_type == 'Polygon' else []
        base = min(samples) if samples else float(ground(*shape.centroid.coords[0]))
        mesh = building_mesh(shape, base, building['height'])
        if mesh is None:
            continue
        count += 1
        uri = 'buildings/b%04d.obj' % count
        files['worlds/' + uri] = _obj(mesh.vertices, mesh.faces, 'OSM building %s %s' % (building['id'], building['name']))
        building['base'], building['top'] = base, base + building['height']
        geometry = _mesh_geometry(uri)
        models.append(_model('building_%04d' % count, (0, 0, 0, 0),
                             '        <collision name="collision">%s</collision>\n'
                             '        <visual name="visual">%s%s</visual>\n'
                             % (geometry, geometry, _material('0.78 0.74 0.68 1'))))
    for index, tree in enumerate(features['trees']):
        z = float(ground(tree['x'], tree['y']))
        trunk = min(3., tree['height'] * .35)
        crown = max(1.2, tree['height'] * .22)
        models.append(_model('tree_%04d' % index, (tree['x'], tree['y'], z, 0),
                             '        <collision name="trunk"><pose>0 0 %.2f 0 0 0</pose><geometry><cylinder>'
                             '<radius>0.18</radius><length>%.2f</length></cylinder></geometry></collision>\n'
                             '        <visual name="trunk"><pose>0 0 %.2f 0 0 0</pose><geometry><cylinder>'
                             '<radius>0.18</radius><length>%.2f</length></cylinder></geometry>%s</visual>\n'
                             '        <visual name="crown"><pose>0 0 %.2f 0 0 0</pose><geometry><sphere>'
                             '<radius>%.2f</radius></sphere></geometry>%s</visual>\n'
                             % (trunk / 2, trunk, trunk / 2, trunk, _material('0.36 0.25 0.15 1'),
                                trunk + crown * .8, crown, _material('0.2 0.45 0.18 1'))))
    if features['roads']:
        vertices, faces = [], []
        for road in features['roads']:
            v, f = ribbon(road['line'], road['width'], ground)
            faces += [(a + len(vertices), b + len(vertices), c + len(vertices)) for a, b, c in f]
            vertices += v
        if faces:
            files['worlds/roads.obj'] = _obj(vertices, faces, 'OSM roads (visual only)')
            models.append(_model('roads', (0, 0, 0, 0), NO_COLLISION + '        <visual name="visual">%s%s</visual>\n'
                                 % (_mesh_geometry('roads.obj'), _material('0.28 0.28 0.3 1'))))
    if features['water']:
        vertices, faces = [], []
        for shape in features['water']:
            v, f = flat_patch(shape, ground)
            faces += [(a + len(vertices), b + len(vertices), c + len(vertices)) for a, b, c in f]
            vertices += v
        if faces:
            files['worlds/water.obj'] = _obj(vertices, faces, 'OSM water (visual only; blocked in the 2D grid)')
            models.append(_model('water', (0, 0, 0, 0), NO_COLLISION + '        <visual name="visual">%s%s</visual>\n'
                                 % (_mesh_geometry('water.obj'), _material('0.2 0.4 0.75 1'))))
    world = world_text.replace('  </world>', ''.join(models) + extra_models + '  </world>')
    return world, files, count


def rasterise(grid_shape, size_m, resolution, polygons=(), circles=()):
    """Boolean mask (map row 0 = +y) of polygons and (x, y, radius) circles."""
    from PIL import Image, ImageDraw
    rows, columns = grid_shape
    image = Image.new('1', (columns, rows), 0)
    draw = ImageDraw.Draw(image)
    half = size_m / 2

    def pixel(x, y):
        return ((x + half) / resolution, (half - y) / resolution)
    for shape in polygons:
        for part in ([shape] if shape.geom_type == 'Polygon' else list(getattr(shape, 'geoms', []))):
            if part.geom_type != 'Polygon':
                continue
            draw.polygon([pixel(x, y) for x, y in part.exterior.coords], fill=1)
            for interior in part.interiors:
                draw.polygon([pixel(x, y) for x, y in interior.coords], fill=0)
    for x, y, radius in circles:
        cx, cy = pixel(x, y)
        r = max(radius / resolution, .7)
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=1)
    return np.asarray(image, dtype=bool)


def occupancy_with_features(spec, heights, features):
    pgm, map_yaml, stats, blocked = tg.occupancy(spec, heights)
    header, body = pgm.split(b'\n', 3)[:3], pgm.split(b'\n', 3)[3]
    columns, rows = (int(v) for v in header[1].split())
    grid = np.frombuffer(body, dtype=np.uint8).reshape(rows, columns).copy()
    resolution = spec['occupancy_resolution_m']
    mask = rasterise((rows, columns), spec['size_m'], resolution,
                     polygons=[b['shape'] for b in features['buildings']] + list(features['water']),
                     circles=[(t['x'], t['y'], .25) for t in features['trees']])
    grid[mask] = tg.OCCUPIED
    stats = dict(stats, feature_cells=int(mask.sum()), occupied_cells=int((grid == tg.OCCUPIED).sum()))
    return b'\n'.join(header) + b'\n' + grid.tobytes(), map_yaml, stats, grid


def choose_spawn(grid, size_m, resolution, clearance=1.5, prefer=(0., 0.)):
    """The free cell with ``clearance`` metres of free space nearest ``prefer``."""
    from scipy.ndimage import distance_transform_edt
    free = grid == tg.FREE
    distance = distance_transform_edt(free) * resolution
    rows, columns = np.nonzero(distance >= clearance)
    if not len(rows):
        raise ValueError('no free spot with %.1f m clearance in this area' % clearance)
    half = size_m / 2
    xs = (columns + .5) * resolution - half
    ys = half - (rows + .5) * resolution
    index = int(np.argmin(np.hypot(xs - prefer[0], ys - prefer[1])))
    return float(xs[index]), float(ys[index])


# ── public entry points ─────────────────────────────────────────────────────

def plan(spec):
    """Validated import spec with defaults; the area decides the terrain size."""
    spec = dict(dict(samples=257, buildings=True, roads=True, trees=True, water=True, traffic=False,
                     people=False, max_relief_m=200., occupancy_resolution_m=.1, max_slope_deg=15.,
                     max_step_m=.08, spawn_clearance_m=1.5, traffic_seconds=300, vehicles_per_minute=6,
                     people_per_minute=12), **spec)
    (south, west, north, east), projection, width, height = validate_area(
        spec['south'], spec['west'], spec['north'], spec['east'])
    if not isinstance(spec.get('id'), str) or not tg.ID_PATTERN.match(spec['id']):
        raise ValueError('id: 3-48 characters, lower-case letters, digits and _ , starting with a letter')
    if spec['samples'] not in tg.SAMPLES:
        raise ValueError('samples must be one of %s' % (tg.SAMPLES,))
    spec.update(south=south, west=west, north=north, east=east, size_m=round(max(width, height), 2))
    return spec, projection


def build(spec, folder=None, progress=print):
    """All outputs (no files written): {relative path: bytes}, manifest, terrain heights, features."""
    spec, projection = plan(spec)
    inputs = (Path(folder) / 'inputs') if folder else None
    bounds = (spec['south'], spec['west'], spec['north'], spec['east'])
    # Inputs: the saved copies when regenerating, otherwise fresh (cached) downloads.
    if inputs and (inputs / 'dem.npy').is_file():
        dem = np.load(inputs / 'dem.npy').astype(np.float32)
        osm = json.loads((inputs / 'osm.json').read_text())
        dem_meta = json.loads((inputs / 'dem.json').read_text())
    else:
        dem, dem_meta = fetch_dem(bounds, projection, spec['size_m'], spec['samples'], progress=progress)
        dem = np.asarray(dem, dtype=np.float32)        # the precision saved in inputs/, so rebuilds match
        osm = fetch_osm(bounds, progress=progress) if any(spec[k] for k in ('buildings', 'roads', 'trees', 'water')) \
            else dict(elements=[])
    relief = float(dem.max() - dem.min())
    if relief > spec['max_relief_m']:
        raise ValueError('this area rises %.0f m; the limit is %.0f m (8-bit heightmaps)' % (relief, spec['max_relief_m']))
    base_spec = dict(id=spec['id'], provider='raster', samples=spec['samples'], size_m=spec['size_m'],
                     source_metres_per_pixel=spec['size_m'] / spec['samples'], relief_m=max(relief, .05) + 1e-6,
                     attribution='%s; %s' % (DEM_ATTRIBUTION, OSM_ATTRIBUTION), license='%s; %s' % (DEM_LICENSE, OSM_LICENSE),
                     spawn_pad_m=0., occupancy_resolution_m=spec['occupancy_resolution_m'],
                     max_slope_deg=spec['max_slope_deg'], max_step_m=spec['max_step_m'], friction=1.)
    scratch = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'tmp'
    scratch.mkdir(parents=True, exist_ok=True)
    dem_file = scratch / ('real_world_%s_%d.npy' % (spec['id'], os.getpid()))
    np.save(dem_file, dem.astype(float) - float(dem.min()))
    try:
        base_spec['raster_path'] = str(dem_file)
        _, _, heights = tg.build(base_spec)              # quantised ground, used to seat the features
        ground = Ground(heights, spec['size_m'])
        progress('Features from OpenStreetMap')
        features = extract_features(osm, projection, spec['size_m'] / 2)
        for key, flag in (('buildings', 'buildings'), ('roads', 'roads'), ('trees', 'trees'), ('water', 'water')):
            if not spec[flag]:
                features[key] = []
        _, _, _, grid = occupancy_with_features(tg.validate(base_spec), heights, features)
        spawn = choose_spawn(grid, spec['size_m'], spec['occupancy_resolution_m'], spec['spawn_clearance_m'])
        base_spec.update(spawn_x=round(spawn[0], 2), spawn_y=round(spawn[1], 2))
        outputs, manifest, heights = tg.build(base_spec)
        validated = tg.validate(base_spec)
    finally:
        dem_file.unlink(missing_ok=True)
    pgm, map_yaml, stats, grid = occupancy_with_features(validated, heights, features)
    outputs['maps/map.pgm'], outputs['maps/map.yaml'] = pgm, map_yaml.encode()
    extra_models, traffic = '', None
    if spec['traffic'] or spec['people']:
        from .real_world_traffic import traffic_actors
        extra_models, traffic, traffic_files = traffic_actors(spec, osm, projection, ground, folder, progress=progress)
        outputs.update(traffic_files)
    world_path = 'worlds/%s.world' % spec['id']
    world, files, building_count = assemble(outputs[world_path].decode(), spec['id'], features, ground, extra_models)
    outputs[world_path] = world.encode()
    outputs.update(files)
    outputs['inputs/dem.npy'] = _npy(dem)
    outputs['inputs/dem.json'] = (json.dumps(dem_meta, sort_keys=True) + '\n').encode()
    outputs['inputs/osm.json'] = (json.dumps(osm, sort_keys=True) + '\n').encode()
    manifest['occupancy'] = stats
    manifest['real_world'] = dict(
        schema=SCHEMA, version=VERSION, request={k: v for k, v in spec.items() if k != 'id'},
        bounds_wgs84=dict(south=spec['south'], west=spec['west'], north=spec['north'], east=spec['east']),
        origin_wgs84=dict(lat=projection.lat0, lon=projection.lon0), projection=projection.definition,
        ground_datum_m=float(dem.min()), relief_m=relief, dem=dem_meta,
        sources=[dict(data='elevation', attribution=DEM_ATTRIBUTION, license=DEM_LICENSE),
                 dict(data='buildings, roads, trees, water', attribution=OSM_ATTRIBUTION, license=OSM_LICENSE)],
        counts=dict(buildings=building_count, roads=len(features['roads']), trees=len(features['trees']),
                    water=len(features['water']), truncated=features['truncated']),
        roofs=[dict(x=round(float(b['shape'].representative_point().x), 2),
                    y=round(float(b['shape'].representative_point().y), 2),
                    top_m=round(b['top'], 3), area_m2=round(b['shape'].area, 1))
               for b in features['buildings'] if 'top' in b][:20],
        traffic=traffic,
        limits=['8-bit heightmap: height steps of %.3f m' % manifest['raster']['quantisation_m'],
                'MuJoCo collides with each building\'s convex hull', 'flat roofs; no interiors'])
    return outputs, manifest, heights, features


def _npy(array):
    buffer = io.BytesIO()
    np.save(buffer, np.asarray(array, dtype=np.float32), allow_pickle=False)
    return buffer.getvalue()


def generate(spec, root=None, replace=False, progress=print):
    """Download (cached), build, write under ``root/<id>/`` and register; returns the manifest."""
    root = Path(root) if root else tg.terrain_root()
    spec, _ = plan(spec)
    if spec['id'] in tg.existing_map_ids():
        raise ValueError('%s is already a map id; choose another name' % spec['id'])
    folder = root / spec['id']
    if folder.exists() and not replace:
        raise ValueError('%s exists; choose another name or regenerate it from its manifest' % folder)
    outputs, manifest, _, _ = build(spec, progress=progress)
    for relative, data in outputs.items():
        tg._write_atomic(folder / relative, data)
    world = folder / 'worlds' / (spec['id'] + '.world')
    (folder / 'mjcf').mkdir(parents=True, exist_ok=True)
    progress('MuJoCo scene')
    mjcf = tg._mjcf(world, spec['id'], folder / 'mjcf').encode()
    tg._write_atomic(folder / 'mjcf' / (spec['id'] + '.xml'), mjcf)
    files = {relative: hashlib.sha256(data).hexdigest() for relative, data in outputs.items()}
    files['mjcf/%s.xml' % spec['id']] = hashlib.sha256(mjcf).hexdigest()
    manifest['files'] = files
    manifest['inputs'] = dict(manifest['inputs'], raster_path='inputs/dem.npy (see real_world)')
    tg._write_atomic(folder / 'manifest.json', (json.dumps(manifest, indent=1, sort_keys=True) + '\n').encode())
    tg.register(manifest, folder, root)
    progress('Registered %s' % spec['id'])
    return manifest


def regenerate(folder):
    """Rebuild from the saved inputs and compare every output (no network, nothing written)."""
    folder = Path(folder)
    manifest = json.loads((folder / 'manifest.json').read_text())
    request = dict(manifest['real_world']['request'], id=manifest['id'])
    outputs, _, _, _ = build(request, folder=folder, progress=lambda _message: None)
    result = {relative: manifest['files'].get(relative) == hashlib.sha256(data).hexdigest()
              for relative, data in outputs.items()}
    world = folder / 'worlds' / (manifest['id'] + '.world')
    if result.get('worlds/%s.world' % manifest['id']):
        mjcf = tg._mjcf(world, manifest['id'], folder / 'mjcf')
        result['mjcf/%s.xml' % manifest['id']] = manifest['files'].get('mjcf/%s.xml' % manifest['id']) == \
            hashlib.sha256(mjcf.encode()).hexdigest()
    return result


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('id', nargs='?')
    parser.add_argument('--bbox', type=float, nargs=4, metavar=('SOUTH', 'WEST', 'NORTH', 'EAST'))
    parser.add_argument('--samples', type=int, default=257)
    for flag in ('no-buildings', 'no-roads', 'no-trees', 'no-water', 'traffic', 'people', 'replace'):
        parser.add_argument('--' + flag, action='store_true')
    parser.add_argument('--spec', type=Path, help='JSON request (the globe page uses this); prints JSON')
    parser.add_argument('--check', action='store_true', help='regenerate an existing import and compare')
    args = parser.parse_args(argv)
    if args.spec:
        try:
            request = json.loads(args.spec.read_text())
            manifest = generate(request, replace=bool(request.get('replace')),
                                progress=lambda message: print(json.dumps(dict(progress=message)), flush=True))
            print(json.dumps(dict(manifest=dict(id=manifest['id'], elevation=manifest['elevation'],
                                                counts=manifest['real_world']['counts'], spawn=manifest['spawn']))),
                  flush=True)
            return 0
        except Exception as exc:                          # reported to the page as JSON
            print(json.dumps(dict(error='%s: %s' % (type(exc).__name__, exc))), flush=True)
            return 2
    if not args.id:
        parser.error('an id (or --spec) is required')
    if args.check:
        result = regenerate(tg.terrain_root() / args.id)
        print(json.dumps(result, indent=1))
        return 0 if all(result.values()) else 1
    if not args.bbox:
        parser.error('--bbox SOUTH WEST NORTH EAST is required')
    south, west, north, east = args.bbox
    manifest = generate(dict(id=args.id, south=south, west=west, north=north, east=east, samples=args.samples,
                             buildings=not args.no_buildings, roads=not args.no_roads, trees=not args.no_trees,
                             water=not args.no_water, traffic=args.traffic, people=args.people), replace=args.replace)
    print(json.dumps(dict(id=manifest['id'], counts=manifest['real_world']['counts'], spawn=manifest['spawn'],
                          elevation=manifest['elevation']), indent=1))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
