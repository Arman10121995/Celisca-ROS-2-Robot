"""SDF ``<polyline>`` geometry as convex triangle prisms in binary STL.

A polyline is a 2D outline extruded from z = 0 to ``<height>`` in its
geometry frame (Gazebo's ``CreateExtrudedPolyline``). Every backend here
already loads STL meshes, and MuJoCo collides with a mesh's convex hull, so
each outline is triangulated (ear clipping, simple polygons without holes)
and every triangle becomes its own prism: a convex piece, exact for any
outline. Files are cached by content hash, so repeated parses reuse them.
"""
import hashlib
import os
import struct
from pathlib import Path


def cache_dir():
    root = os.environ.get('ROBOT_LAB_RUNTIME_ROOT') or os.path.join(os.path.expanduser('~'), '.cache', 'robot_lab')
    path = Path(root) / 'polyline_meshes'
    path.mkdir(parents=True, exist_ok=True)
    return path


def _area(points):
    return .5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(points, points[1:] + points[:1]))


def _inside(p, a, b, c):
    def side(u, v, w):
        return (v[0] - u[0]) * (w[1] - u[1]) - (v[1] - u[1]) * (w[0] - u[0])
    return side(a, b, p) >= 0 and side(b, c, p) >= 0 and side(c, a, p) >= 0


def triangulate(points):
    """Ear-clip a simple polygon; returns index triples, counter-clockwise."""
    points = [tuple(map(float, p)) for p in points]
    if len(points) > 3 and points[0] == points[-1]:
        points = points[:-1]                   # SDF outlines may repeat the first point
    if len(points) < 3 or abs(_area(points)) < 1e-12:
        raise ValueError('A polyline needs at least three non-collinear points')
    order = list(range(len(points)))
    if _area(points) < 0:
        order.reverse()
    triangles = []
    guard = 0
    while len(order) > 3:
        guard += 1
        if guard > 10 * len(points) ** 2:
            raise ValueError('Polyline outline is not a simple polygon')
        for k in range(len(order)):
            i, j, l = order[k - 1], order[k], order[(k + 1) % len(order)]
            a, b, c = points[i], points[j], points[l]
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 1e-12:
                continue                       # reflex or degenerate corner
            if any(_inside(points[m], a, b, c) for m in order if m not in (i, j, l)):
                continue
            triangles.append((i, j, l))
            del order[k]
            break
    triangles.append(tuple(order))
    return points, triangles


def _prism_stl(a, b, c, height):
    bottom = [(a[0], a[1], 0.), (b[0], b[1], 0.), (c[0], c[1], 0.)]
    top = [(x, y, height) for x, y, _ in bottom]
    facets = [(bottom[0], bottom[2], bottom[1]), (top[0], top[1], top[2])]
    for i in range(3):
        j = (i + 1) % 3
        facets += [(bottom[i], bottom[j], top[j]), (bottom[i], top[j], top[i])]
    data = bytearray(b'robot_lab polyline prism'.ljust(80, b' ')) + struct.pack('<I', len(facets))
    for facet in facets:
        data += struct.pack('<3f', 0., 0., 0.)
        for vertex in facet:
            data += struct.pack('<3f', *vertex)
        data += struct.pack('<H', 0)
    return bytes(data)


def polyline_prisms(outlines, height, directory=None):
    """STL paths of the convex prisms for one <polyline> geometry (one or more outlines)."""
    if not height > 0:
        raise ValueError('A polyline needs a positive <height>')
    directory = Path(directory) if directory else cache_dir()
    paths = []
    for outline in outlines:
        points, triangles = triangulate(outline)
        for triangle in triangles:
            data = _prism_stl(*(points[i] for i in triangle), height)
            path = directory / ('%s.stl' % hashlib.sha256(data).hexdigest()[:24])
            if not path.is_file():
                path.write_bytes(data)
            paths.append(str(path))
    return paths
