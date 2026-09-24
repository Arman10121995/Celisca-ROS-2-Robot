"""Keep open rooms open: static triangle surfaces instead of convex hulls.

MuJoCo's rigid flexes support non-convex triangle collision (MuJoCo >= 3).
Only direct world geoms from our flattened SDF converter are rewritten.
The original mesh remains visible and available to the range sensor rays.
"""
import xml.etree.ElementTree as ET
import numpy as np


# Triangles per flex.  MuJoCo's flex compile time grows much faster than
# linearly with its size: one 92k-triangle Celisca floor took 134 s to
# compile as a single triangle soup; welded and split into ~30 spatial
# pieces it takes 0.5 s.
_FLEX_FACES = 3000


def _spatial_chunks(vertices, faces, size=_FLEX_FACES):
    """Split a mesh into compact pieces of about *size* triangles each.

    Faces are ordered along a grid of their centroids so every piece covers
    one region of the building, which keeps each flex's bounding volume
    small for collision.
    """
    if len(faces) <= size:
        return [(vertices, faces)]
    centroids = vertices[faces].mean(axis=1)
    lo, hi = centroids.min(axis=0), centroids.max(axis=0)
    cells = max(1, int(np.ceil(np.sqrt(len(faces) / size))))
    grid = np.floor((centroids[:, :2] - lo[:2]) / np.maximum(hi[:2] - lo[:2], 1e-9)
                    * (cells - 1e-9)).astype(np.int64)
    order = np.lexsort((centroids[:, 2], grid[:, 1], grid[:, 0]))
    pieces = []
    for chunk in np.array_split(faces[order], int(np.ceil(len(faces) / size))):
        used, inverse = np.unique(chunk.ravel(), return_inverse=True)
        pieces.append((vertices[used], inverse.reshape(-1, 3)))
    return pieces


def add_static_mesh_collisions(root):
    import mujoco
    from robot_lab_utils.mesh_assets import load_indexed_mesh

    assets = {mesh.get('name'): mesh for mesh in root.findall('./asset/mesh')}
    # Rigid flex/static-geom contacts have no movable DOFs. Exclude
    # world-versus-world constraints while retaining robot/world contact.
    for static in root.findall('./worldbody/geom'):
        static.set('conaffinity', '0')
    count = 0
    for geom in root.findall('./worldbody/geom'):
        if geom.get('type') != 'mesh' or (geom.get('contype') == '0' and geom.get('conaffinity') == '0'):
            continue
        asset = assets[geom.get('mesh')]
        # Shared vertices (STL is a triangle soup): a welded surface is a
        # sixth of the flex vertices.  No optional packages needed.
        vertices, faces = load_indexed_mesh(asset.get('file'))
        tri = vertices[faces]
        valid = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1) > 1e-12
        faces = faces[valid]
        if not len(faces):
            raise ValueError('world collision mesh has no nondegenerate triangles: ' + asset.get('file'))
        vertices = vertices * np.fromstring(asset.get('scale', '1 1 1'), sep=' ')
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, np.fromstring(geom.get('quat', '1 0 0 0'), sep=' '))
        vertices = vertices @ rotation.reshape(3, 3).T + np.fromstring(geom.get('pos', '0 0 0'), sep=' ')
        deformable = root.find('deformable')
        if deformable is None:
            deformable = ET.SubElement(root, 'deformable')
        contact = {'contype': geom.get('contype', '1'),
                   'conaffinity': geom.get('conaffinity', '1'),
                   'selfcollide': 'none', 'internal': 'false'}
        if geom.get('friction'):
            contact['friction'] = geom.get('friction')
        for piece, (piece_vertices, piece_faces) in enumerate(_spatial_chunks(vertices, faces)):
            flex = ET.SubElement(deformable, 'flex', name=f'world_collision_{count}_{piece}',
                                 dim='2', body='world', radius='0.001', group='3',
                                 vertex=' '.join(format(v, '.9g') for v in piece_vertices.ravel()),
                                 element=' '.join(str(i) for i in piece_faces.ravel()))
            ET.SubElement(flex, 'contact', **contact)
        geom.set('contype', '0')
        geom.set('conaffinity', '0')
        count += 1
    return count
