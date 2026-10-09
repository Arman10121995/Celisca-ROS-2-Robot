"""Reduce redundant coplanar triangulation without filling mesh openings.

Only edge-connected, numerically coplanar patches with equivalent planar
surface unions are accepted. Curved, nonmanifold or unsupported patches retain
their original triangles. Visual and raycast assets are never edited.
"""
import hashlib
import os
from pathlib import Path
import uuid

import numpy as np

VERSION = b'robot_lab_coplanar_collision_v1'


def retriangulate(vertices, faces):
    from shapely.geometry import Polygon
    from shapely.errors import ShapelyError
    from shapely.ops import triangulate, unary_union
    vertices, faces = np.asarray(vertices, dtype=float), np.asarray(faces, dtype=np.int64)
    if len(faces) < 8:
        return vertices, faces
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    offsets = np.einsum('ij,ij->i', normals, triangles[:, 0])
    edges = np.sort(np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]])), axis=1)
    face_ids = np.tile(np.arange(len(faces)), 3)
    order = np.lexsort((edges[:, 1], edges[:, 0]))
    edges, face_ids = edges[order], face_ids[order]
    starts = np.r_[0, np.flatnonzero(np.any(edges[1:] != edges[:-1], axis=1))+1, len(edges)]
    parents = np.arange(len(faces))

    def root(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    plane_tolerance = 1e-10*max(1., float(np.max(np.abs(vertices))))
    for start, end in zip(starts[:-1], starts[1:]):
        if end-start != 2:
            continue
        a, b = face_ids[start:end]
        # Same winding; mixed/double-sided faces stay distinct.
        if np.linalg.norm(normals[a]-normals[b]) <= 1e-10 and abs(offsets[a]-offsets[b]) <= plane_tolerance:
            parents[root(b)] = root(a)
    groups = {}
    for index in range(len(faces)):
        groups.setdefault(root(index), []).append(index)
    reduced_vertices, reduced_faces = [], []
    count = 0
    for indices in groups.values():
        original = faces[indices]
        used = np.unique(original)
        points = vertices[used]
        candidate_points, candidate_faces = None, None
        if len(indices) >= 8:
            try:
                normal = normals[indices[0]]
                origin = triangles[indices[0], 0]
                direction = triangles[indices[0], 1]-origin
                direction /= np.linalg.norm(direction)
                basis = np.stack((direction, np.cross(normal, direction)), axis=1)
                if np.max(abs((points-origin) @ normal)) > plane_tolerance:
                    raise ValueError('Patch accumulated nonplanar deviation')
                projected = (points-origin) @ basis
                remapped = np.searchsorted(used, original)
                surface = unary_union([Polygon(projected[face]) for face in remapped])
                if not surface.is_valid:
                    raise ValueError('Invalid planar union')
                # Remove exactly collinear redundant boundary points only;
                # preserve topology and retain all real corners and openings.
                surface = surface.simplify(0., preserve_topology=True)
                patches = [surface] if surface.geom_type == 'Polygon' else list(surface.geoms)
                generated = [triangle for patch in patches for triangle in triangulate(patch)
                             if patch.covers(triangle) and triangle.area > 0]
                if not generated or len(generated) >= len(original):
                    raise ValueError('No useful exact planar reduction')
                union = unary_union(generated)
                if surface.symmetric_difference(union).area > 1e-12*max(1., surface.area):
                    raise ValueError('Retriangulation changed the planar surface or an opening')
                xy = np.asarray([list(triangle.exterior.coords)[:3] for triangle in generated])
                candidate_points, inverse = np.unique(xy.reshape(-1, 2), axis=0, return_inverse=True)
                candidate_faces = inverse.reshape(-1, 3)
                candidate_points = origin+candidate_points @ basis.T
                orientation = np.cross(candidate_points[candidate_faces[:, 1]]-candidate_points[candidate_faces[:, 0]],
                                       candidate_points[candidate_faces[:, 2]]-candidate_points[candidate_faces[:, 0]]) @ normal
                candidate_faces[orientation < 0] = candidate_faces[orientation < 0][:, [0, 2, 1]]
            except (ValueError, TypeError, ShapelyError):
                candidate_points, candidate_faces = None, None
        if candidate_points is None:
            candidate_points, inverse = np.unique(vertices[original].reshape(-1, 3), axis=0, return_inverse=True)
            candidate_faces = inverse.reshape(-1, 3)
        reduced_vertices.append(candidate_points)
        reduced_faces.append(candidate_faces+count)
        count += len(candidate_points)
    points, inverse = np.unique(np.concatenate(reduced_vertices), axis=0, return_inverse=True)
    output = inverse[np.concatenate(reduced_faces)]
    if len(points) >= len(vertices) and len(output) >= len(faces):
        return vertices, faces
    return points, output


def cached_retriangulation(vertices, faces):
    # Optional at installation/runtime: unavailable Shapely retains exact
    # original collision triangles, rather than reducing geometry fidelity.
    try:
        import shapely
    except ImportError:
        return vertices, faces
    runtime = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')).resolve()
    if not str(runtime).startswith('/workspace/') or runtime.stat().st_dev == Path('/').stat().st_dev:
        return vertices, faces
    digest = hashlib.sha256(VERSION+shapely.__version__.encode()+np.asarray(vertices).tobytes()+np.asarray(faces).tobytes()).hexdigest()
    directory = runtime/'cache/planar_collisions'
    directory.mkdir(parents=True, exist_ok=True)
    path = directory/(digest+'.npz')
    if path.is_file():
        try:
            with np.load(path, allow_pickle=False) as arrays:
                points, triangles = arrays['vertices'], arrays['faces']
                if (points.ndim == 2 and points.shape[1] == 3 and np.all(np.isfinite(points))
                        and triangles.ndim == 2 and triangles.shape[1] == 3
                        and np.issubdtype(triangles.dtype, np.integer) and triangles.size
                        and np.min(triangles) >= 0 and np.max(triangles) < len(points)):
                    return points, triangles
        except (OSError, ValueError, KeyError):
            pass  # Recompute a damaged optional cache from original assets.
    output = retriangulate(vertices, faces)
    temporary = directory/(digest+'.'+uuid.uuid4().hex+'.npz')
    np.savez_compressed(temporary, vertices=output[0], faces=output[1])
    temporary.replace(path)
    return output
