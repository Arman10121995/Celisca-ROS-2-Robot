"""Static world collision as convex pieces (P4: large-world speed), an alternative to rigid flexes.

MuJoCo collides only convex meshes; non-convex meshes are convex-hulled.
``world_collision.add_static_mesh_collisions`` therefore turns world meshes
into rigid flexes, which keep every triangle but which MuJoCo updates vertex
by vertex every step: 3.7 of 4 ms per tick on celisca_floor_1. This module
instead decomposes each world mesh with CoACD (approximate convex
decomposition; Wei et al., SIGGRAPH 2022; MIT) into convex hulls. Static
convex geoms cost nothing per step until a robot is near them. Each mesh is
first split into the same compact spatial pieces as the flexes, so CoACD's
voxel preprocessing resolves walls and door gaps at piece scale rather than
building scale.

Contacts change (hulls approximate the triangles within ``threshold``
concavity), so ``check_against_grid`` measures the result against the
world's reviewed occupancy map before a world is switched to it.

Results are cached on SSD by mesh hash and parameters
(``$ROBOT_LAB_RUNTIME_ROOT/mujoco_convex``).
"""
import hashlib
import json
import os
from pathlib import Path

import numpy as np

PIECE_FACES = 3000


def cache_root():
    return Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'mujoco_convex'


def decompose(vertices, faces, threshold=0.05, piece_faces=PIECE_FACES, key=None, log=None):
    """[(hull vertices, hull faces)] for a world-frame mesh, cached by content and parameters."""
    import coacd
    from robot_lab_mujoco.world_collision import _spatial_chunks
    digest = hashlib.sha256()
    digest.update(np.ascontiguousarray(vertices, dtype=np.float64).tobytes())
    digest.update(np.ascontiguousarray(faces, dtype=np.int64).tobytes())
    digest.update(json.dumps(dict(threshold=threshold, piece_faces=piece_faces, coacd=getattr(coacd, '__version__', '1.0.14'),
                                  key=key)).encode())
    path = cache_root() / (digest.hexdigest() + '.npz')
    if path.is_file():
        data = np.load(path)
        return [(data['v%d' % i], data['f%d' % i]) for i in range(int(data['count']))]
    coacd.set_log_level('error')
    hulls = []
    pieces = _spatial_chunks(np.asarray(vertices, dtype=float), np.asarray(faces), piece_faces)
    for index, (piece_vertices, piece_faces_) in enumerate(pieces):
        for hull_vertices, hull_faces in coacd.run_coacd(coacd.Mesh(piece_vertices, piece_faces_), threshold=threshold,
                                                         preprocess_mode='auto', max_convex_hull=-1):
            hulls.append((np.asarray(hull_vertices, dtype=float), np.asarray(hull_faces, dtype=np.int64)))
        if log:
            log('convex decomposition: piece %d/%d, %d hulls so far' % (index + 1, len(pieces), len(hulls)))
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays = {'count': len(hulls)}
    for i, (v, f) in enumerate(hulls):
        arrays['v%d' % i], arrays['f%d' % i] = v, f
    temporary = path.with_suffix('.tmp.npz')
    np.savez_compressed(temporary, **arrays)
    temporary.replace(path)
    return hulls


def halfspaces(hull_vertices):
    """Outward face planes (normal, offset) of a convex hull."""
    from scipy.spatial import ConvexHull
    hull = ConvexHull(hull_vertices)
    return hull.equations                      # rows [nx, ny, nz, d]: n.x + d <= 0 inside


def inside_any(points, hulls, tolerance=0.0):
    """Boolean per point: inside at least one convex hull (planes from ``halfspaces``)."""
    points = np.asarray(points, dtype=float)
    result = np.zeros(len(points), dtype=bool)
    for equations, lo, hi in hulls:
        candidates = np.all((points >= lo - tolerance) & (points <= hi + tolerance), axis=1) & ~result
        if not candidates.any():
            continue
        subset = points[candidates]
        result[np.flatnonzero(candidates)[np.all(subset @ equations[:, :3].T + equations[:, 3] <= tolerance, axis=1)]] = True
    return result


def check_against_grid(hulls, grid, origin, resolution, heights=(0.1, 0.3), occupied=100, free=0):
    """Compare hulls with a reviewed occupancy grid at robot heights.

    Returns the fraction of free cells whose centres fall inside a hull at
    any of ``heights`` (false obstacles: closed doors, thickened walls), and
    the fraction of occupied cells inside none (false openings).
    """
    planes = []
    for v, _ in hulls:
        try:
            planes.append((halfspaces(v), v.min(axis=0), v.max(axis=0)))
        except Exception:
            continue
    grid = np.asarray(grid)
    rows, cols = np.indices(grid.shape)
    xy = np.stack([origin[0] + (cols + .5) * resolution, origin[1] + (rows + .5) * resolution], axis=-1)
    report = {}
    for name, value in (('free', free), ('occupied', occupied)):
        mask = grid == value
        hit = np.zeros(int(mask.sum()), dtype=bool)
        for z in heights:
            points = np.c_[xy[mask], np.full(int(mask.sum()), z)]
            hit |= inside_any(points, planes)
        report[name] = dict(cells=int(mask.sum()), inside_hulls=int(hit.sum()), fraction=round(float(hit.mean()) if hit.size else 0., 4))
    return report
