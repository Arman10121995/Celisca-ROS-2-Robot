"""Explicit, source-hashed corrections for defective imported worlds.

Original downloads remain untouched. Never clamp arbitrary world poses: only
the reviewed hospital fixtures from the exact dataset snapshot are repaired.
The AWS ros2 snapshot has the same escaped vertical poses, so it cannot serve
as a corrected replacement. Its neighbouring fixtures establish 0/3 m floors.
"""
import hashlib
import re
import xml.etree.ElementTree as ET
from pathlib import Path


def cached_occupancy_matches(report, source_sha256, seed):
    """Invalidate a real export when any referenced geometry changes in place."""
    try:
        if (report.get('source_sha256') != source_sha256 or report.get('seed_xy') != seed
                or report.get('projection_recipe') != 'complete-static-height-slice-v3'):
            return False
        artifacts = report.get('artifact_sha256', {})
        if not {'.pgm','.yaml'} <= set(artifacts):
            return False
        dependencies = dict(report['mesh_sha256'])
        dependencies.update({str(Path(report['output_yaml']).with_suffix(suffix)):sha
                             for suffix,sha in artifacts.items()})
        return all(Path(name).is_file() and hashlib.sha256(Path(name).read_bytes()).hexdigest() == sha
                   for name,sha in dependencies.items())
    except (KeyError,ValueError,TypeError,OSError):
        return False


def floor_mesh_polygons(mesh):
    """Actual near-zero support triangles of a large thin floor, including holes.

    A bounding rectangle or convex hull would invent support over floor gaps.
    Upper decks, furniture, and walls must not become a ground-floor spawn.
    """
    bounds = mesh.bounds
    extent = bounds[1]-bounds[0]
    if (min(extent[:2]) < 2. or extent[2] > .3
            or bounds[0,2] < -.3 or bounds[0,2] > .05):
        return []
    triangles = mesh.vertices[mesh.faces]
    supported = ((mesh.face_normals[:,2] > .95)
                 & (triangles[:,:,2].min(axis=1) >= -.1)
                 & (triangles[:,:,2].max(axis=1) <= .05))
    return list(triangles[supported,:,:2])


def gazebo_collada_copy(source, directory, resolve):
    """Make shared-reader geometry match Gazebo's actual node-frame convention.

    Fortress's ColladaLoader applies scene nodes and metres, and ignores the
    up_axis tag. Our standards-aware shared reader also rotates that tag.
    A derived Z_UP declaration avoids a second axis conversion on import;
    original scene nodes, vertices, scale, textures and source remain intact.
    """
    original_text = source.read_text()
    try:
        root = ET.fromstring(original_text)
        comments_repaired = False
        attributes_repaired = False
    except ET.ParseError:
        # Some OSRF meshes contain '--' in comments, accepted by Gazebo but
        # invalid XML. Repair only comments; bad geometry still raises.
        cleaned = re.sub(r'<!--.*?-->', '',original_text,flags=re.S)
        comments_repaired = cleaned != original_text
        # Assimp-exported meshes also use literal '<STL_BINARY>' in node
        # id/name attributes. Escape XML syntax without renaming the node
        # or changing a vertex/transform; TinyXML accepts the original.
        escaped = re.sub(r'''(["'])(.*?)\1''',
            lambda m:m[1]+m[2].replace('<','&lt;').replace('>','&gt;')+m[1],cleaned,flags=re.S)
        attributes_repaired = escaped != cleaned
        try:
            root = ET.fromstring(escaped)
        except ET.ParseError as exc:
            raise ValueError('Invalid Collada geometry '+str(source)+': '+str(exc)) from exc
    namespace = root.tag.partition('}')[0]+'}' if root.tag.startswith('{') else ''
    if namespace:
        # Gazebo's Collada loader uses literal XML element names. ET's
        # generated ns0: prefixes are legal XML but make it skip geometry.
        # Retain the original default COLLADA namespace for both readers.
        ET.register_namespace('', namespace[1:-1])
    asset = root.find(namespace+'asset')
    if asset is None:
        asset = ET.SubElement(root,namespace+'asset')
    axis = asset.find(namespace+'up_axis')
    original_axis = axis.text.strip() if axis is not None and axis.text else 'Y_UP'
    if original_axis == 'Z_UP' and not (comments_repaired or attributes_repaired):
        return source, None
    if axis is None:
        axis = ET.SubElement(asset,namespace+'up_axis')
    axis.text = 'Z_UP'
    textures, unresolved = {}, []
    for node in root.findall('.//'+namespace+'library_images/'+namespace+'image/'+namespace+'init_from'):
        if not node.text:
            continue
        original = node.text.strip()
        try:
            path = resolve(original,source.parent)
            node.text = path.as_uri()
            textures[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        except (ValueError,OSError) as exc:
            unresolved.append(dict(uri=original, reason=str(exc)))
    sha = hashlib.sha256(source.read_bytes()).hexdigest()
    target = directory/sha/source.name
    target.parent.mkdir(parents=True,exist_ok=True)
    ET.ElementTree(root).write(target,encoding='utf-8',xml_declaration=True)
    return target, dict(source_mesh=str(source), source_sha256=sha,
        derived_mesh=str(target), derived_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        original_up_axis=original_axis, derived_up_axis='Z_UP',
        invalid_xml_comments_removed=comments_repaired,
        invalid_xml_attribute_brackets_escaped=attributes_repaired,
        recipe='gazebo-node-frame-v2', texture_sha256=textures, unresolved_textures=unresolved)


HOSPITAL_SOURCES = {
    'aa0d4cb3810553f06bddca64819d04967fc09972e48d0dbc84a10f00e51fe65b': 0,
    '1491be78fb21862ca3e1aa825de2c67d6a7e2828b4f73b73e386c35d7375488b': 1,
}
FIXTURES = ('StorageRack_1', 'WhiteChipChair_1', 'TrolleyBed_1')


def preserve_static_containers(root):
    """Keep an empty pose wrapper around static models out of the physics solver.

    Classic exports use linkless model wrappers around included static fixtures.
    Fortress treats the missing wrapper static flag as dynamic, creating a
    moving assembly for an otherwise static building. Only unmarked, linkless,
    jointless containers consisting entirely of static children are changed.
    An explicit dynamic flag or any moving/link-bearing child stays intact.
    """
    records = []
    for model in reversed(list(root.iter('model'))):
        children = model.findall('model')
        if (model.find('static') is not None or not children
                or any(model.find(tag) is not None for tag in ('link','joint','include'))
                or any(child.findtext('static','').strip().lower() not in ('true','1')
                       for child in children)):
            continue
        ET.SubElement(model,'static').text = 'true'
        records.append(dict(model=model.get('name'),
            reason='Linkless pose wrapper contains only explicitly static fixtures'))
    return records


def static_fixture_snapshot(world):
    """Explicit static environment derivative, matching the other three engines.

    Shared world import represents model geometry as fixed collision bodies;
    it does not implement furniture dynamics or stripped Classic plugins.
    Keep that contract in Gazebo too, retaining all geometry and nested poses.
    Original downloads and separately declared actors are unchanged. Record
    each changed top-level flag so this never masquerades as dynamic parity.
    """
    records = []
    for model in world.findall('model'):
        static = model.find('static')
        previous = static.text if static is not None else None
        if (previous or '').strip().lower() in ('true','1'):
            continue
        if static is None:
            static = ET.SubElement(model,'static')
        static.text = 'true'
        records.append(dict(model=model.get('name'),source_static=previous,
            derived_static=True,reason='Static environment snapshot; dynamic fixtures are not implemented'))
    return records


def repair_hospital_poses(source, root):
    floors = HOSPITAL_SOURCES.get(hashlib.sha256(source.read_bytes()).hexdigest())
    if floors is None:
        return []
    repairs = []
    for model in root.iter('model'):
        name = model.get('name', '')
        upper = name.startswith('floor_1_')
        base_name = name.removeprefix('floor_1_')
        if base_name not in FIXTURES or (upper and not floors):
            continue
        pose = model.find('pose')
        if pose is None:
            pose = model.find('include/pose')
        if pose is None:
            raise ValueError('Reviewed hospital fixture has no pose: ' + name)
        values = pose.text.split()
        if len(values) != 6 or float(values[2]) > -1e8:
            raise ValueError('Reviewed hospital defect changed: ' + name)
        original = pose.text
        values[2] = '3' if upper else '0'
        pose.text = ' '.join(values)
        repairs.append(dict(model=name, original_pose=original,
                            derived_pose=pose.text,
                            reason='Restore reviewed fixture to its authored 0/3 m floor; preserve XY and orientation'))
    if len(repairs) != 3 * (floors + 1):
        raise ValueError('Reviewed hospital fixture count changed')
    return repairs
