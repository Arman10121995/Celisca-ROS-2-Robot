"""Resolve downloaded Classic worlds into installed ROS 2 world profiles.

Original archives/models are preserved. Derived SDFs resolve their real model
resources and record removed Classic plugins. Static geometry is shared by all
four backends; scripted dynamics need separate qualification.
"""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from PIL import Image, ImageDraw
import yaml

from catalog_external_assets import ROOT, digest, load_source
from provision_extension_assets import source_checkout, write_json


def extract_archive(archive, destination):
    """Extract actual dependencies without archive path/symlink escapes."""
    destination.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(archive) as stream:
        if sum(item.file_size for item in stream.infolist()) > 8*1024**3:
            raise ValueError('Archive exceeds the bounded extraction size')
        for item in stream.infolist():
            target = (destination/item.filename).resolve()
            if not target.is_relative_to(destination.resolve()) or (item.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError('Unsafe archive member: '+item.filename)
            if item.is_dir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            content = stream.read(item)
            if target.exists() and target.read_bytes() != content:
                raise ValueError('Conflicting archive dependency: '+str(target))
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists():
                target.write_bytes(content)


def model_resolver(resources, preferred):
    model_dirs = {}
    for path in resources:
        if path.name == 'model.sdf':
            model_dirs.setdefault(path.parent.name, []).append(path.parent)
            if path.parent.name.endswith('_ariac'):
                model_dirs.setdefault(path.parent.name.removesuffix('_ariac'), []).append(path.parent)
    def resolve(uri, base):
        if uri.strip() == 'file://media/materials/scripts/gazebo.material':
            return ROOT/'src/robot_lab_maps/vendor/gazebo_classic/gazebo.material'
        value = uri.strip().removeprefix('file://')
        if uri.startswith('model://'):
            name, _, suffix = uri.removeprefix('model://').partition('/')
            candidates = [directory/suffix for directory in model_dirs.get(name, [])]
            candidates = [path for path in candidates if path.exists()]
            if candidates:
                candidates.sort(key=lambda p: (not p.is_relative_to(preferred), str(p)))
                return candidates[0]
        else:
            direct = (base/value).resolve()
            if direct.exists():
                return direct
        # Several AWS archive URIs have case errors on Linux. Require a
        # unique existing path rather than replacing missing geometry.
        lowered = value.removeprefix('models/').lower()
        matches = [p for p in resources if p.as_posix().lower().endswith('/'+lowered)]
        if len(matches) == 1:
            return matches[0]
        parts = Path(value).parts
        for length in range(min(7, len(parts)), 1, -1):
            suffix = '/'.join(parts[-length:])
            matches = [path for path in resources if path.as_posix().endswith('/'+suffix)]
            local = [path for path in matches if path.is_relative_to(preferred)]
            if len(local) == 1:
                return local[0]
            if len(matches) == 1:
                return matches[0]
        raise ValueError('Unresolved original world resource: '+uri)
    return resolve


def convert_world(source, target, name, resources, preferred):
    resolve = model_resolver(resources, preferred)
    # Upstream Gazebo examples contain decorative '--' inside comments,
    # tolerated by SDFormat but rejected by Python's strict XML parser.
    # Comments do not affect geometry; originals remain untouched.
    source_text = re.sub(r'<!--.*?-->', '', source.read_text(), flags=re.S)
    root = ET.fromstring(source_text.lstrip())
    world = root.find('world')
    if world is None:
        model = root.find('model') if root.tag == 'sdf' else root
        if model is None or model.tag != 'model':
            raise ValueError('No world/model in SDF')
        root = ET.Element('sdf', version='1.7')
        world = ET.SubElement(root, 'world', name=name)
        world.append(model)
    root.set('version', '1.7'); world.set('name', name)
    removed, dependencies, include_count = [], {}, 0
    standard = ET.parse(ROOT/'src/robot_lab_maps/maps/empty/worlds/empty.world').getroot().find('world')

    def walk(parent, base, depth=0):
        nonlocal include_count
        if depth > 16:
            raise ValueError('World includes exceed the bounded recursion depth')
        for node in list(parent):
            if node.tag == 'script' and parent.tag == 'material' and node.findtext('uri', '').strip() == 'file://media/materials/scripts/gazebo.material':
                material = node.findtext('name', '')
                material_file = resolve(node.findtext('uri'), base)
                text = material_file.read_text()
                block = re.search(r'material\s+'+re.escape(material)+r'\s*\{(.*?)(?=\nmaterial|\Z)', text, re.S)
                if block is None: raise ValueError('Unknown upstream Classic material: '+material)
                for color in ('ambient', 'diffuse', 'specular', 'emissive'):
                    match = re.search(r'\b'+color+r'\s+([\d.eE+\- ]+)', block.group(1))
                    if match:
                        values = match.group(1).split()[:4]
                        if len(values) == 3: values.append('1')
                        if parent.find(color) is None: ET.SubElement(parent, color).text = ' '.join(values)
                dependencies[str(material_file)] = digest(material_file)
                parent.remove(node)
                continue
            if node.tag == 'plugin':
                removed.append(node.attrib)
                parent.remove(node)
                continue
            if node.tag == 'include':
                uri = node.findtext('uri', '').strip()
                standard_name = uri.removeprefix('model://')
                if standard_name in ('sun', 'ground_plane'):
                    original = next(n for n in standard if n.get('name') == standard_name)
                    replacement = copy.deepcopy(original)
                else:
                    directory = resolve(uri, base)
                    included = directory/'model.sdf' if directory.is_dir() else directory
                    included_root = ET.fromstring(included.read_text().lstrip())
                    replacement = included_root.find('model')
                    if replacement is None:
                        raise ValueError('Included resource is not a model: '+str(included))
                    replacement = copy.deepcopy(replacement)
                    walk(replacement, included.parent, depth+1)
                    dependencies[str(included)] = digest(included)
                if node.find('name') is not None:
                    replacement.set('name', node.findtext('name'))
                pose = node.find('pose')
                if pose is not None:
                    # Nested model preserves both the include and model poses.
                    wrapper = ET.Element('model', name=replacement.get('name')+'_include')
                    wrapper.append(copy.deepcopy(pose))
                    ET.SubElement(wrapper, 'static').text = 'true'
                    wrapper.append(replacement); replacement = wrapper
                parent.remove(node); parent.append(replacement)
                include_count += 1
                continue
            if node.tag == 'uri' and node.text:
                resource = resolve(node.text.strip(), base)
                node.text = resource.as_uri()
                if resource.is_file(): dependencies[str(resource)] = digest(resource)
            walk(node, base, depth)
    walk(world, source.parent)
    for filename, system in [('physics', 'Physics'), ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster')]:
        ET.SubElement(world, 'plugin', filename='ignition-gazebo-'+filename+'-system',
                      name='ignition::gazebo::systems::'+system)
    if world.find('light') is None:
        world.append(copy.deepcopy(next(n for n in standard if n.tag == 'light')))
    physics = world.find('physics')
    if physics is None: physics = ET.SubElement(world, 'physics', type='ode')
    step = physics.find('max_step_size')
    if step is None: step = ET.SubElement(physics, 'max_step_size')
    step.text = '.002'
    target.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(target, encoding='unicode', xml_declaration=True)
    return {'source_world': str(source), 'source_sha256': digest(source), 'derived_world_sha256': digest(target),
            'resolved_include_count': include_count, 'dependency_sha256': dependencies,
            'removed_classic_plugins': removed,
            'dynamic_actors': [actor.get('name') for actor in world.findall('actor')]}


def load_tool(name):
    path = ROOT/'src/robot_lab_maps/tools'/(name+'.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def seed_for_world(world, output, resolution=.1, height=.3):
    """Choose a clear point in a bounded free region from actual geometry."""
    from scipy import ndimage
    import trimesh
    mapping = load_tool('generate_occupancy_map')
    mapping.prepare_mapping_world(world, output/'seed-projection.sdf', output/'seed', resolution, height, [0., 0.])
    mask_file = output/'seed-projection.collision-mask'
    with mask_file.open('rb') as stream:
        width, rows = map(int, stream.readline().split())
        mask = Image.frombytes('L', (width, rows), stream.read())
    draw = ImageDraw.Draw(mask)
    support = Image.new('L', (width, rows))
    support_draw = ImageDraw.Draw(support)
    shapes, skipped = mapping.extract_static_shapes(str(world), mapping.uri_resolver())
    for shape in shapes:
        kind = shape['type']
        if kind in ('mesh', 'plane', 'heightmap'): continue
        size = shape['size']
        if kind == 'box': mesh = trimesh.creation.box(size)
        elif kind == 'sphere': mesh = trimesh.creation.icosphere(radius=size[0], subdivisions=2)
        else: mesh = trimesh.creation.cylinder(radius=size[0], height=size[1], sections=32)
        mesh.apply_transform(mapping.world_transform(shape))
        # Prefer a real floor footprint over free space outside a building.
        # The factory contains multiple disconnected floor slabs.
        if kind == 'box' and min(size[:2]) >= 2. and size[2] <= .3 and -.1 <= mesh.bounds[1,2] <= .05:
            corners = mesh.vertices[mesh.vertices[:,2] >= mesh.bounds[1,2]-.001]
            from scipy.spatial import ConvexHull
            polygon = corners[ConvexHull(corners[:,:2]).vertices,:2]
            support_draw.polygon([(x/resolution+width/2,y/resolution+rows/2) for x,y in polygon],fill=255)
        section = mesh.section(plane_origin=[0, 0, height], plane_normal=[0, 0, 1])
        if section is not None:
            for path in section.discrete:
                pixels = [(x/resolution+width/2, y/resolution+rows/2) for x,y in path[:,:2]]
                draw.polygon(pixels, fill=255)
    occupied = np.asarray(mask) > 0
    free = ~occupied
    if np.asarray(support).any():
        free &= np.asarray(support) > 0
    labels, _ = ndimage.label(free)
    boundary = set(np.unique(np.concatenate([labels[0], labels[-1], labels[:,0], labels[:,-1]])))
    candidates = [(int(np.sum(labels == component)), component) for component in np.unique(labels)
                  if component and component not in boundary]
    if candidates:
        region = labels == max(candidates)[1]
    if not candidates or ndimage.distance_transform_edt(region).max()*resolution < .45:
        region = free.copy()
        occupied_rows, occupied_columns = np.where(occupied)
        if len(occupied_rows) and not any(shape['type']=='plane' for shape in shapes):
            region[:occupied_rows.min()+2] = False; region[occupied_rows.max()-1:] = False
            region[:,:occupied_columns.min()+2] = False; region[:,occupied_columns.max()-1:] = False
        elif not len(occupied_rows):
            region[:rows//4] = False; region[3*rows//4:] = False
            region[:,:width//4] = False; region[:,3*width//4:] = False
    clearance = ndimage.distance_transform_edt(region)
    row, column = np.unravel_index(np.argmax(clearance), clearance.shape)
    if clearance[row,column] * resolution < .45:
        raise ValueError('No spawn with 0.45 m projected clearance')
    return [float((column+.5-width/2)*resolution), float((row+.5-rows/2)*resolution)], float(clearance[row,column]*resolution)


def generate_map(world, directory, seed):
    directory.mkdir(parents=True, exist_ok=True)
    previous = list(directory.rglob('*.generation.json'))
    for path in sorted(previous, key=lambda p: p.stat().st_mtime, reverse=True):
        report = json.loads(path.read_text())
        if (report.get('source_sha256') == digest(world) and report.get('seed_xy') == seed
                and report.get('projection_recipe') == 'complete-static-height-slice-v2'
                and all(Path(report['output_yaml']).with_suffix(suffix).is_file()
                        and digest(Path(report['output_yaml']).with_suffix(suffix)) == sha
                        for suffix,sha in report.get('artifact_sha256',{}).items())
                and {'.pgm','.yaml'} <= set(report.get('artifact_sha256',{}))):
            return report, path
    log = directory/'generation.log'
    command = [sys.executable, str(ROOT/'src/robot_lab_maps/tools/generate_occupancy_map.py'),
        '--world', str(world), '--output-dir', str(directory), '--resolution', '.1', '--height', '.3',
        '--seed-x', str(seed[0]), '--seed-y', str(seed[1]), '--timeout', '90']
    with log.open('w') as stream:
        result = subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT, timeout=180)
    if result.returncode:
        raise ValueError('Occupancy generation failed: '+str(log))
    report_path = max(directory.rglob('*.generation.json'), key=lambda p: p.stat().st_mtime)
    return json.loads(report_path.read_text()), report_path


def install_worlds(store, download, source_id='gazebo_world_dataset'):
    source = load_source(source_id)
    checkout = source_checkout(source, store, download)
    extracted = store/'extracted'/source['id']/source['revision']
    entry_for = {}
    for entry in source['entries']:
        if entry.endswith('.zip'):
            folder = extracted/Path(entry).parent.name
            extract_archive(checkout/entry, folder)
            for path in folder.rglob('*'):
                entry_for.setdefault(path, entry)
    resources = [p for directory in (checkout, extracted) for p in directory.rglob('*')
                 if p.is_file() and '.git' not in p.parts]
    worlds = [p for p in resources if p.suffix in ('.world', '.model') and 'models' not in p.parts]
    if source_id == 'gazebo_examples':
        # Install the self-contained environment examples first. The other
        # upstream SDFs are retained and explicitly require fixture-specific
        # resource/plugin review; do not turn robot/plugin demos into maps.
        worlds = [checkout/'examples/worlds'/name for name in ('empty.sdf','default.sdf','shapes.sdf')]
    for dependency_id in ('gazebo_classic_models', 'servicesim_models'):
        dependency_source = load_source(dependency_id)
        dependency_root = source_checkout(dependency_source, store, download)
        resources.extend(p for p in dependency_root.rglob('*') if p.is_file() and '.git' not in p.parts)
    profiles, entities, records = {}, [], []
    grouped = {entry: {'source_id': source['id'], 'revision': source['revision'], 'entry': entry,
                      'status': 'downloaded; fixture integration pending' if source_id == 'gazebo_examples' else 'downloaded dependency',
                      'profiles': []} for entry in source['entries']}
    converter = load_tool('gen_mjcf_worlds')
    for original in sorted(worlds):
        family = original.parent.name
        if family == 'worlds': family = original.parent.parent.name
        stem = original.stem
        name = ('gazebo_example_'+stem if source_id == 'gazebo_examples' else
                'dataset_'+family+('_'+stem if stem not in (family, 'world') else ''))
        name = re.sub('[^a-zA-Z0-9_]', '_', name)
        entry = str(original.relative_to(checkout)) if original.is_relative_to(checkout) else entry_for[original]
        record = grouped.setdefault(entry, {'source_id': source['id'], 'revision': source['revision'], 'entry': entry, 'profiles': []})
        directory = store/'worlds'/name/source['revision']
        target = directory/'worlds'/(name+'.world')
        directory.mkdir(parents=True, exist_ok=True)
        try:
            preferred = original.parent
            check = convert_world(original, target, name, resources, preferred)
            from robot_lab_utils.sdf_world import extract_static_shapes, uri_resolver
            shapes, skipped = extract_static_shapes(str(target), uri_resolver())
            unresolved = [item for item in skipped if not item.startswith('actor ')]
            if unresolved or not shapes:
                raise ValueError('Incomplete world geometry: '+str(unresolved))
            mjcf, notes = converter.convert_world(str(target), name, str(directory/'mjcf'))
            mjcf_path = directory/'mjcf'/(name+'.xml'); mjcf_path.parent.mkdir(exist_ok=True)
            mjcf_path.write_text(mjcf)
            output = directory/'maps'; output.mkdir(exist_ok=True)
            seed, clearance = seed_for_world(target, output)
            check.update(shapes=len(shapes), skipped_dynamic_geometry=skipped, mjcf_notes=notes,
                         spawn_xy=seed, projected_spawn_clearance_m=clearance,
                         runtime_mission_qualified=False)
            occupancy = None
            try:
                occupancy, generation = generate_map(target, output, seed)
                check['occupancy_generation_report'] = str(generation)
            except Exception as exc:
                check['occupancy_error'] = str(exc)
            profile = {'source_id': source['id'], 'source_entry': entry,
                'gazebo': {'world_package': 'robot_lab_maps', 'world_name': name, 'world_path': str(target)},
                'map': {'has_2d_map': bool(occupancy)},
                'spawn': {'x': str(seed[0]), 'y': str(seed[1]), 'z': '.1', 'yaw': '0.0'},
                'initial_pose': {'x': str(seed[0]), 'y': str(seed[1]), 'yaw': '0.0'},
                'notes': 'Installed upstream world with resolved static geometry. Classic plugins removed; dynamic behaviors and robot missions remain unqualified.'}
            if occupancy:
                profile['map'].update(package='robot_lab_maps', path=occupancy['output_yaml'])
            evidence = directory/'integration-check.json'; write_json(evidence, check)
            profiles[name] = profile
            entity = {'id': name, 'version': '1.0.0', 'name': name.replace('_', ' ').title(),
                'status': 'available', 'dimension': '2D' if occupancy else '3D', 'simulator': 'gazebo',
                'simulators': ['gazebo', 'pybullet', 'mujoco', 'isaac'], 'world_file': str(target),
                'supported_robot_classes': ['mobile', 'legged', 'humanoid', 'aerial', 'manipulator', 'hybrid'],
                'spawn_zones': [{'id': 'default', 'pose': {'x': seed[0], 'y': seed[1], 'z': .1}}],
                'source': {'repository': source['repository'], 'revision': source['revision'], 'license': source['license_note']},
                'description': profile['notes'], 'tags': ['extension', source['id']],
                'evidence': [{'kind': 'integration_test', 'reference': str(evidence),
                             'description': 'World dependency/conversion and named actual occupancy generation; no backend mission qualification', 'date': '2026-10-05'}]}
            if occupancy:
                entity.update(occupancy_yaml=occupancy['output_yaml'], occupancy_map=str(Path(occupancy['output_yaml']).with_suffix('.pgm')))
            entities.append(entity)
            record.update(status='installed')
            record['profiles'].append({'id': name, 'kind': 'world', 'support': 'World + 2D map' if occupancy else 'World; occupancy pending', 'notes': profile['notes']})
            print(name, 'installed', len(shapes), 'shapes', 'map='+str(bool(occupancy)), flush=True)
        except Exception as exc:
            record.update(status='needs integration repair', reason=str(exc))
            print(name, 'needs integration repair:', exc, flush=True)
    records.extend(grouped.values())
    return profiles, entities, records
