"""Export actual compiled MuJoCo geometry for ROS model visualization.

The native simulator publishes each body's measured transform. This URDF is
its geometry description, not a replacement plant or a trajectory controller.
Compiled mesh vertices include the source scale and mesh alignment.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def body_frame(index):
    return 'native_world' if index == 0 else 'native_body_' + str(index)


def quaternion_rpy(quat):
    w, x, y, z = quat
    return (np.arctan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
            np.arcsin(np.clip(2 * (w*y - z*x), -1, 1)),
            np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)))


def numbers(values):
    return ' '.join('%.12g' % value for value in values)


def export_display_urdf(model, output):
    import mujoco
    import trimesh

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    meshes = output.parent / 'meshes'
    meshes.mkdir(exist_ok=True)
    robot = ET.Element('robot', name='native_mjcf_asset')
    links = []
    for index in range(model.nbody):
        link = ET.SubElement(robot, 'link', name=body_frame(index))
        links.append(link)
        if index:
            joint = ET.SubElement(robot, 'joint', name='native_frame_' + str(index), type='fixed')
            ET.SubElement(joint, 'parent', link=body_frame(int(model.body_parentid[index])))
            ET.SubElement(joint, 'child', link=body_frame(index))
            ET.SubElement(joint, 'origin', xyz=numbers(model.body_pos[index]),
                          rpy=numbers(quaternion_rpy(model.body_quat[index])))
    mesh_files = {}
    unsupported = []
    for index in range(model.ngeom):
        kind = int(model.geom_type[index])
        # Collision-only shapes duplicate authored visual meshes in most models.
        body = int(model.geom_bodyid[index])
        if model.geom_group[index] == 3 and any(
                int(model.geom_bodyid[g]) == body and model.geom_group[g] == 2
                for g in range(model.ngeom)):
            continue
        shape = None
        size = model.geom_size[index]
        if kind == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = int(model.geom_dataid[index])
            if mesh not in mesh_files:
                vertex = int(model.mesh_vertadr[mesh]); count = int(model.mesh_vertnum[mesh])
                face = int(model.mesh_faceadr[mesh]); nface = int(model.mesh_facenum[mesh])
                shape = trimesh.Trimesh(model.mesh_vert[vertex:vertex+count].copy(),
                                        model.mesh_face[face:face+nface].copy(), process=False)
                target = meshes / ('mesh_%d.stl' % mesh)
                shape.export(target)
                mesh_files[mesh] = target
            geometry_attributes = ('mesh', {'filename': mesh_files[mesh].resolve().as_uri()})
        elif kind == mujoco.mjtGeom.mjGEOM_BOX:
            geometry_attributes = ('box', {'size': numbers(2 * size)})
        elif kind == mujoco.mjtGeom.mjGEOM_SPHERE:
            geometry_attributes = ('sphere', {'radius': str(size[0])})
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            geometry_attributes = ('cylinder', {'radius': str(size[0]), 'length': str(2 * size[1])})
        elif kind in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_ELLIPSOID):
            if kind == mujoco.mjtGeom.mjGEOM_CAPSULE:
                shape = trimesh.creation.capsule(height=2 * size[1], radius=size[0], count=[16, 16])
                # trimesh capsule spans z=0..height, MuJoCo is centred.
                shape.apply_translation([0, 0, -size[1]])
            else:
                shape = trimesh.creation.icosphere(subdivisions=2)
                shape.apply_scale(size)
            target = meshes / ('geom_%d.stl' % index)
            shape.export(target)
            geometry_attributes = ('mesh', {'filename': target.resolve().as_uri()})
        elif kind == mujoco.mjtGeom.mjGEOM_PLANE:
            continue  # the selected environment supplies the ground
        else:
            unsupported.append({'geom': index, 'type': kind})
            continue
        visual = ET.SubElement(links[body], 'visual', name='geom_' + str(index))
        ET.SubElement(visual, 'origin', xyz=numbers(model.geom_pos[index]),
                      rpy=numbers(quaternion_rpy(model.geom_quat[index])))
        geometry = ET.SubElement(visual, 'geometry')
        ET.SubElement(geometry, *geometry_attributes)
        material = ET.SubElement(visual, 'material', name='color_' + str(index))
        rgba = model.geom_rgba[index]
        if model.geom_matid[index] >= 0:
            rgba = model.mat_rgba[int(model.geom_matid[index])]
        ET.SubElement(material, 'color', rgba=numbers(rgba))
    ET.ElementTree(robot).write(output, encoding='unicode')
    return {'urdf': str(output), 'bodies': model.nbody, 'meshes': len(mesh_files),
            'unsupported_geometry': unsupported,
            'scope': 'Native measured-body visualization; no exported controller or physics equivalence'}
