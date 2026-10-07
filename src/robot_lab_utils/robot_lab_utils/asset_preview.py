"""Static registry geometry in authored metres, without starting a plant.

URDF joint frames, native MuJoCo keyframes and SDF visual/include poses are
resolved for the embedded OpenGL viewport. No movement command is produced.
"""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .sdf_world import compose, quaternion_from_rpy, uri_resolver, extract_static_shapes


IDENTITY = ((0., 0., 0.), (1., 0., 0., 0.))


def _values(text, default):
    values = [float(value) for value in text.split()] if text else list(default)
    if len(values) != len(default) or not all(math.isfinite(value) for value in values):
        raise ValueError('Invalid geometry vector: ' + str(text))
    return values


def _origin(element):
    origin = element.find('origin')
    if origin is None:
        return IDENTITY
    return (_values(origin.get('xyz'), (0., 0., 0.)),
            quaternion_from_rpy(*_values(origin.get('rpy'), (0., 0., 0.))))


def urdf_shapes(description, resolve, base_dir='', joint_positions=None):
    """Resolve visual FK, including prismatic/revolute/mimic joints."""
    root = ET.fromstring(description)
    links = {link.get('name'): link for link in root.findall('link')}
    if not links:
        raise ValueError('Robot description has no links')
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    children = [joint.find('child').get('link') for joint in joints.values()]
    if len(set(children)) != len(children):
        raise ValueError('Robot description gives a link multiple parents')
    roots = set(links)-set(children)
    if len(roots) != 1:
        raise ValueError('Robot preview needs one connected root link')
    frames = {next(iter(roots)): IDENTITY}
    positions, resolving = dict(joint_positions or {}), set()

    def position(name):
        if name in positions:
            value = float(positions[name])
            if not math.isfinite(value):
                raise ValueError('Non-finite preview joint position: ' + name)
            return value
        if name in resolving:
            raise ValueError('Cyclic mimic joint: ' + name)
        resolving.add(name)
        joint = joints[name]
        mimic = joint.find('mimic')
        if mimic is not None:
            value = (float(mimic.get('multiplier', '1')) * position(mimic.get('joint'))
                     + float(mimic.get('offset', '0')))
        else:
            value = 0.
            limit = joint.find('limit')
            if limit is not None and joint.get('type') != 'continuous':
                value = max(float(limit.get('lower', '-inf')),
                            min(float(limit.get('upper', 'inf')), value))
        if not math.isfinite(value):
            raise ValueError('Non-finite preview joint position: ' + name)
        positions[name] = value
        resolving.remove(name)
        return value

    pending = list(joints.values())
    while pending:
        ready = [joint for joint in pending if joint.find('parent').get('link') in frames]
        if not ready:
            raise ValueError('Disconnected or cyclic robot joint tree')
        for joint in ready:
            name, kind = joint.get('name'), joint.get('type')
            frame = compose(frames[joint.find('parent').get('link')], _origin(joint))
            if kind not in ('fixed', 'revolute', 'continuous', 'prismatic'):
                raise ValueError('Unsupported URDF preview joint: ' + str(kind))
            if kind != 'fixed':
                axis_element = joint.find('axis')
                axis = np.asarray(_values(axis_element.get('xyz') if axis_element is not None else None,
                                           (1., 0., 0.)))
                norm = np.linalg.norm(axis)
                if norm <= 1e-12:
                    raise ValueError('Zero preview joint axis: ' + name)
                axis /= norm
                value = position(name)
                motion = ((axis*value, IDENTITY[1]) if kind == 'prismatic' else
                          (IDENTITY[0], (math.cos(value/2), *(axis*math.sin(value/2)))))
                frame = compose(frame, motion)
            child = joint.find('child').get('link')
            if child not in links:
                raise ValueError('Joint references missing link: ' + child)
            frames[child] = frame
            pending.remove(joint)
    if set(frames) != set(links):
        raise ValueError('Disconnected robot preview links')
    materials = {item.get('name'): item for item in root.findall('material')}
    shapes, notes = [], []
    for name, link in links.items():
        visuals = link.findall('visual')
        if not visuals and link.findall('collision'):
            notes.append('Link ' + name + ' has no visual; showing its authored collision geometry')
            visuals = link.findall('collision')
        for visual in visuals:
            geometry = visual.find('geometry')
            if geometry is None:
                continue
            frame = compose(frames[name], _origin(visual))
            shape = dict(model=name, position=list(frame[0]), orientation=list(frame[1]),
                         rgba=[.68, .73, .8, 1.])
            material = visual.find('material')
            if material is not None:
                if material.find('color') is None:
                    material = materials.get(material.get('name'), material)
                color = material.find('color')
                if color is not None:
                    shape['rgba'] = _values(color.get('rgba'), shape['rgba'])
            child = next(iter(geometry), None)
            if child is None:
                continue
            kind = child.tag
            shape['type'] = kind
            if kind == 'box':
                shape['size'] = _values(child.get('size'), (1., 1., 1.))
            elif kind == 'sphere':
                shape['size'] = [float(child.get('radius'))]
            elif kind == 'cylinder':
                shape['size'] = [float(child.get('radius')), float(child.get('length'))]
            elif kind == 'mesh':
                source = resolve(child.get('filename'), base_dir)
                if not source:
                    raise ValueError('Unresolved preview mesh: ' + str(child.get('filename')))
                shape.update(mesh=str(source), scale=_values(child.get('scale'), (1., 1., 1.)))
            else:
                notes.append('Unsupported visual geometry ' + kind + ' on ' + name)
                continue
            shapes.append(shape)
    return shapes, notes


def _rotation(quaternion):
    w, x, y, z = quaternion
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _mesh_bounds(path):
    import trimesh
    mesh = trimesh.load(str(path), force='mesh', process=False)
    if not len(mesh.vertices):
        raise ValueError('Empty preview mesh: ' + str(path))
    return np.array(mesh.bounds)


def scene_bounds(shapes):
    """AABB in world metres; decorative ground planes do not dominate Fit."""
    bounds, planes, meshes = [], [], {}
    for shape in shapes:
        kind, size = shape['type'], shape.get('size', [])
        if kind == 'mesh':
            path = shape['mesh']
            if path not in meshes:
                meshes[path] = _mesh_bounds(path)
            lo, hi = meshes[path]*np.asarray(shape.get('scale', [1., 1., 1.]))
            lo, hi = np.minimum(lo, hi), np.maximum(lo, hi)
        else:
            half = {'box': np.asarray(size)/2, 'sphere': np.repeat(size[0], 3),
                    'cylinder': np.asarray([size[0], size[0], size[1]/2]) if len(size) > 1 else [],
                    'plane': np.asarray([*size[:2], .02])/2}.get(kind)
            if half is None:
                raise ValueError('Cannot fit unsupported geometry: ' + kind)
            lo, hi = -half, half
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        world = corners @ _rotation(shape['orientation']).T + np.asarray(shape['position'])
        (planes if kind == 'plane' else bounds).append(np.array([world.min(axis=0), world.max(axis=0)]))
    all_bounds = np.asarray(bounds or planes)
    if not len(all_bounds):
        raise ValueError('No geometry available for 3D preview')
    raw_lo, raw_hi = all_bounds[:, 0].min(axis=0), all_bounds[:, 1].max(axis=0)
    # Some upstream editor exports contain isolated fixtures hundreds of
    # millions of metres below the building. Preserve all authored geometry
    # and raw bounds, but keep the default camera useful for the main scene.
    centers = all_bounds.mean(axis=1)
    center = np.median(centers, axis=0)
    distances = np.linalg.norm(centers-center, axis=1)
    remote = distances > max(1000., 100*float(np.percentile(distances, 90)))
    fitted = all_bounds[~remote] if remote.any() else all_bounds
    lo, hi = fitted[:, 0].min(axis=0), fitted[:, 1].max(axis=0)
    if not np.isfinite([lo, hi]).all():
        raise ValueError('Non-finite preview bounds')
    return dict(min_xyz=lo.tolist(), max_xyz=hi.tolist(),
                center_xyz=((lo+hi)/2).tolist(), span_m=float(np.linalg.norm(hi-lo)),
                raw_min_xyz=raw_lo.tolist(), raw_max_xyz=raw_hi.tolist(),
                remote_geometry_count=int(remote.sum()))


def prepare_scene(kind, profile, output, package_share):
    """Build a static inspection scene; generated meshes stay on the SSD."""
    from .mesh_assets import stage_mesh_file
    from .robot_description import load_description
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    notes, source = [], None
    model_dirs = []
    try:
        model_dirs.append(str(Path(package_share('robot_lab_models'))/'models'))
    except (LookupError, FileNotFoundError):
        pass
    model_dirs.extend(profile.get('gazebo', {}).get('model_dirs', []))
    resolve = uri_resolver(model_dirs, package_share)
    if kind == 'maps':
        gazebo = profile.get('gazebo', {})
        source = Path(gazebo['world_path'])
        if not source.is_absolute():
            source = Path(package_share(gazebo['world_package']))/source
        shapes, notes = extract_static_shapes(str(source), resolve, prefer_visual=True)
    elif kind == 'robots':
        if profile.get('native_mjcf'):
            import mujoco
            from .native_mjcf_assets import export_display_urdf
            source = Path(profile['native_mjcf'])
            model = mujoco.MjModel.from_xml_path(str(source))
            data = mujoco.MjData(model)
            if model.nkey:
                key = next((i for i in range(model.nkey) if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, i)
                            in ('home', 'stand', 'standing')), 0)
                mujoco.mj_resetDataKeyframe(model, data, key)
            mujoco.mj_forward(model, data)
            description = output/'native.urdf'
            exported = export_display_urdf(model, description)
            notes.extend('Unsupported native geometry '+str(item) for item in exported['unsupported_geometry'])
            shapes, urdf_notes = urdf_shapes(description.read_text(), resolve, str(output))
            notes.extend(urdf_notes)
            visuals = list(ET.parse(description).getroot().iter('visual'))
            # Each exported visual is named by the exact compiled geom index.
            by_geom = {visual.get('name'): shape for visual, shape in zip(visuals, shapes)}
            shapes = []
            for name, shape in by_geom.items():
                index = int(name.removeprefix('geom_'))
                if int(model.geom_bodyid[index]) == 0:
                    continue
                quaternion = np.empty(4)
                mujoco.mju_mat2Quat(quaternion, data.geom_xmat[index])
                shape.update(position=data.geom_xpos[index].tolist(), orientation=quaternion.tolist())
                shapes.append(shape)
            notes.append('Native compiled nominal/keyframe pose; static inspection, no policy or plant')
        else:
            source = Path(profile.get('xacro', ''))
            if not profile.get('xacro'):
                if profile.get('name') != 'px4_x500':
                    raise ValueError('No installed description for this registry robot')
                import os
                px4 = Path(os.environ.get('PX4_ROOT', '/workspace/molar/px4/PX4-Autopilot'))
                upstream = px4/'Tools/simulation/gz'
                source = upstream/'models/x500_base/model.sdf'
                # Read source visuals directly. Static inspection must not
                # import the FCU/offboard module or require MAVLink packages.
                resolve = uri_resolver([str(upstream/'models'), *model_dirs], package_share)
                shapes, notes = extract_static_shapes(str(source), resolve, prefer_visual=True)
                description = None
            else:
                if not source.is_absolute():
                    source = Path(package_share(profile['package']))/source
                description = load_description(source, 'is_sim:=false')
            if description is not None:
                (output/'robot.urdf').write_text(description)
                shapes, notes = urdf_shapes(description, resolve, str(source.parent))
                notes.append('URDF nominal joint pose; static inspection, no controller or plant')
            else:
                notes.append('SDF nominal link pose; static inspection, no flight controller or plant')
    else:
        raise ValueError('Only robots and maps have a 3D preview')
    if source is None or not source.is_file():
        raise ValueError('Preview source is not installed: '+str(source))
    prepared, converted = [], []
    for index, raw in enumerate(shapes):
        shape = dict(raw)
        if shape['type'] == 'heightmap':
            import trimesh
            from .heightfield import heightfield_grid
            vertices, faces = heightfield_grid(shape)
            target = output/'meshes'/('heightmap_%d.stl' % index)
            target.parent.mkdir(parents=True, exist_ok=True)
            trimesh.Trimesh(vertices, faces, process=False).export(target)
            shape.update(type='mesh', mesh=str(target), scale=[1., 1., 1.])
        if shape['type'] == 'mesh':
            original = Path(shape['mesh'])
            # Content-derived stems avoid collisions between identical file
            # names in unrelated included models. Do not decimate visuals.
            digest = hashlib.sha256(original.read_bytes()).hexdigest()
            target = stage_mesh_file(str(original), str(output/'meshes'),
                                     stem=digest, max_faces=1000000000)
            if not target:
                raise ValueError('Could not prepare preview mesh: '+str(original))
            if Path(target).suffix.lower() == '.msh':
                import trimesh
                converted_path = output/'meshes'/(digest+'.stl')
                trimesh.load(target, force='mesh').export(converted_path)
                target = str(converted_path)
            shape['mesh'] = str(Path(target).resolve())
            converted.append(dict(source=str(original), sha256=digest, preview_mesh=shape['mesh']))
        prepared.append(shape)
    bounds = scene_bounds(prepared)
    if bounds['remote_geometry_count']:
        notes.append(str(bounds['remote_geometry_count'])+' remote source geometries are outside the default camera Fit. '
                     'Whole scene shows their raw extent; authored poses are unchanged and require source review.')
    return dict(kind=kind, source=str(source.resolve()), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                shapes=prepared, bounds=bounds, notes=notes, meshes=converted,
                scope='Static source geometry in metres; scripted actors, physics, policies and missions are not executed')


def prepare_render_geometry(scene, output):
    """Reuse Trimesh primitives/normals; keep GPU-ready buffers off the UI thread."""
    import trimesh
    meshes, arrays = {}, {}
    triangles = 0
    for shape in scene['shapes']:
        kind = shape['type']
        key = (kind, shape['mesh']) if kind == 'mesh' else (kind, tuple(shape['size']))
        if key not in meshes:
            if kind == 'mesh':
                mesh = trimesh.load(shape['mesh'], force='mesh', process=False)
            elif kind == 'box':
                mesh = trimesh.creation.box(extents=shape['size'])
            elif kind == 'plane':
                mesh = trimesh.creation.box(extents=[*shape['size'][:2], .01])
            elif kind == 'sphere':
                mesh = trimesh.creation.icosphere(subdivisions=2, radius=shape['size'][0])
            elif kind == 'cylinder':
                mesh = trimesh.creation.cylinder(radius=shape['size'][0], height=shape['size'][1], sections=32)
            else:
                raise ValueError('Unsupported preview render geometry: ' + kind)
            if not len(mesh.faces) or not np.isfinite(mesh.vertices).all():
                raise ValueError('Invalid preview render mesh')
            index = len(meshes)
            meshes[key] = index
            arrays['vertices_'+str(index)] = np.asarray(mesh.vertices, dtype=np.float32)
            arrays['normals_'+str(index)] = np.asarray(mesh.vertex_normals, dtype=np.float32)
            arrays['faces_'+str(index)] = np.asarray(mesh.faces, dtype=np.uint32).ravel()
        shape['render_mesh'] = meshes[key]
        triangles += len(arrays['faces_'+str(shape['render_mesh'])])//3
    np.savez(output, **arrays)
    return dict(path=str(output), meshes=len(meshes), triangles=triangles,
                scope='Full source geometry; colors retained where declared; embedded textures are not reconstructed')
