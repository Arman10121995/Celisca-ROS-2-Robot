"""Keep open rooms open: static triangle surfaces instead of convex hulls.

MuJoCo's rigid flexes support non-convex triangle collision (MuJoCo >= 3).
Only direct world geoms from our flattened SDF converter are rewritten.
The original mesh remains visible and available to the range sensor rays.
"""
import xml.etree.ElementTree as ET
import numpy as np


def add_static_mesh_collisions(root):
    import mujoco
    import trimesh

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
        mesh = trimesh.load(asset.get('file'), force='mesh', process=True)
        vertices = np.asarray(mesh.vertices, dtype=float)
        faces = np.asarray(mesh.faces, dtype=int)
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
        flex = ET.SubElement(deformable, 'flex', name=f'world_collision_{count}',
                             dim='2', body='world', radius='0.001', group='3',
                             vertex=' '.join(format(v, '.9g') for v in vertices.ravel()),
                             element=' '.join(str(i) for i in faces.ravel()))
        contact = {'contype': geom.get('contype', '1'),
                   'conaffinity': geom.get('conaffinity', '1'),
                   'selfcollide': 'none', 'internal': 'false'}
        if geom.get('friction'):
            contact['friction'] = geom.get('friction')
        ET.SubElement(flex, 'contact', **contact)
        geom.set('contype', '0')
        geom.set('conaffinity', '0')
        count += 1
    return count
