"""Keep open rooms open: static triangle surfaces instead of convex hulls.

MuJoCo's rigid flexes support non-convex triangle collision (MuJoCo >= 3).
Only direct world geoms from our flattened SDF converter are rewritten.
The original mesh remains visible (moved to geom group 2); range sensors
cast against exact copies of its triangles split into compact pieces
(group 3, see add_ray_proxies), because mj_ray tests every triangle of a
mesh it reaches and one large world mesh cost ~0.18 ms per ray.
"""
import xml.etree.ElementTree as ET
import numpy as np


# Triangles per flex.  MuJoCo's flex compile time grows much faster than
# linearly with its size: one 92k-triangle Celisca floor took 134 s to
# compile as a single triangle soup; welded and split into ~30 spatial
# pieces it takes 0.5 s.  Without the midphase (configure_world_midphase)
# a contact tests every element of the flex it touches, so pieces stay
# small: on celisca_floor_1, 1000-triangle pieces step in 1.90 ms against
# 2.04 ms for 3000 (narrowphase 0.05 vs 0.24 ms).
_FLEX_FACES = 1000


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
        # Fixed SDF surfaces need no solid-body mass inference. Hospital
        # portals/floors can be open planar meshes; MuJoCo's default volume
        # inference rejects them even after their contacts become rigid flexes.
        # Shell inference preserves the source vertices and robot inertias.
        asset.set('inertia', 'shell')
        # Shared vertices (STL is a triangle soup): a welded surface is a
        # sixth of the flex vertices.  No optional packages needed.
        vertices, faces = load_indexed_mesh(asset.get('file'))
        tri = vertices[faces]
        valid = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1) > 1e-12
        faces = faces[valid]
        if not len(faces):
            raise ValueError('world collision mesh has no nondegenerate triangles: ' + asset.get('file'))
        from robot_lab_utils.planar_mesh import cached_retriangulation
        vertices, faces = cached_retriangulation(vertices, faces)
        vertices = vertices * np.fromstring(asset.get('scale', '1 1 1'), sep=' ')
        rotation = np.empty(9)
        mujoco.mju_quat2Mat(rotation, np.fromstring(geom.get('quat', '1 0 0 0'), sep=' '))
        vertices = vertices @ rotation.reshape(3, 3).T + np.fromstring(geom.get('pos', '0 0 0'), sep=' ')
        deformable = root.find('deformable')
        if deformable is None:
            deformable = ET.SubElement(root, 'deformable')
        contact = {'contype': geom.get('contype', '1'),
                   'conaffinity': geom.get('conaffinity', '1'),
                   'selfcollide': 'none'}
        # MuJoCo 3.3.1 made internal flex contacts disabled by default;
        # 3.15 removed the XML attribute. Older engines still need the
        # explicit opt-out. Preserve rigid surface contact semantics on both.
        version = tuple(int(part) for part in mujoco.__version__.split('.')[:3])
        if version < (3, 3, 1):
            contact['internal'] = 'false'
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


# Geom groups of the staged world.  Rendering shows 0-2; group 3 holds the
# rigid collision flexes and the ray proxies, which range sensors include.
WORLD_VISUAL_GROUP = 2
WORLD_RAY_GROUP = 3
# Collision meshes of a model drawn from its textured visuals
# (gen_mjcf_worlds.py): not rendered, and not ranged either, because their
# triangles are in the group-3 proxies.
WORLD_HIDDEN_GROUP = 4
VISUAL_PREFIX = 'visual_'
# Ray mask for range sensors: everything except the world's visual meshes
# and hidden collision meshes (their exact triangles are in the group-3
# proxies).
RANGE_SENSOR_GEOMGROUP = (1, 1, 0, 1, 0, 1)
_RAY_PROXY_FACES = 200


def add_ray_proxies(root, size=_RAY_PROXY_FACES):
    """Exact, spatially split copies of every world mesh for range sensors.

    mj_ray bounds each geom before testing its triangles one by one, so a
    building split into ~200-triangle pieces is ray-cast several times
    faster with identical hits (measured on celisca_floor_1: 360 rays in
    18 ms instead of 65 ms, distances equal to 1e-6 m).  The proxies do not
    collide and are not drawn; the original meshes stay as the visuals.
    Returns the number of proxy geoms.
    """
    from robot_lab_utils.mesh_assets import load_indexed_mesh

    assets = {mesh.get('name'): mesh for mesh in root.findall('./asset/mesh')}
    asset_root = root.find('asset')
    worldbody = root.find('worldbody')
    count = 0
    for geom in list(worldbody.findall('geom')):
        if geom.get('type') != 'mesh' or geom.get('mesh') not in assets:
            continue
        if (geom.get('name') or '').startswith(VISUAL_PREFIX):
            continue   # textured visual parts: drawn only, never ranged
        mesh = assets[geom.get('mesh')]
        if not mesh.get('file'):
            continue
        vertices, faces = load_indexed_mesh(mesh.get('file'))
        tri = vertices[faces]
        valid = np.linalg.norm(np.cross(tri[:, 1]-tri[:, 0], tri[:, 2]-tri[:, 0]), axis=1) > 1e-12
        faces = faces[valid]
        if not len(faces):
            continue
        vertices = vertices * np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
        if geom.get('group') != str(WORLD_HIDDEN_GROUP):
            geom.set('group', str(WORLD_VISUAL_GROUP))
        for piece_vertices, piece_faces in _spatial_chunks(vertices, faces, size):
            name = 'world_ray_%d' % count
            ET.SubElement(asset_root, 'mesh', name=name, inertia='shell', maxhullvert='4',
                          vertex=' '.join(format(v, '.9g') for v in piece_vertices.ravel()),
                          face=' '.join(str(i) for i in piece_faces.ravel()))
            # mj_ray skips geoms with zero alpha; group 3 is not rendered.
            ET.SubElement(worldbody, 'geom', name=name, type='mesh', mesh=name,
                          pos=geom.get('pos', '0 0 0'), quat=geom.get('quat', '1 0 0 0'),
                          group=str(WORLD_RAY_GROUP), contype='0', conaffinity='0',
                          rgba='0.5 0.5 0.5 1')
            count += 1
    return count


# Above this many world flex elements the collision midphase costs more
# than it saves: its per-step BVH refit of static triangles dominated the
# step (hospital 194k elements: 10.6 ms -> 4.7 ms per step without it,
# Celisca floor 1 70k: 4.2 -> 2.0 ms).  Disabling it only removes an
# acceleration structure: contacts and trajectories are identical.
MIDPHASE_FLEX_ELEMENTS = 5000


def configure_world_midphase(model, mode='auto'):
    """Disable MuJoCo's midphase for large static-flex worlds ('auto').

    Returns True when the midphase was disabled.
    """
    import mujoco
    mode = str(mode or 'auto').lower()
    disable = mode == 'off' or (mode == 'auto' and int(model.nflexelem) > MIDPHASE_FLEX_ELEMENTS)
    if disable:
        model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_MIDPHASE)
    return disable
