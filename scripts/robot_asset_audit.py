#!/usr/bin/env python3
"""License, mesh, texture and skin check of every installed robot model (ROADMAP R3.6).

For each robot profile Launch offers:

- **License:** the license file nearest the model, walking up from the
  model file to the source checkout root (or the robot_lab_robots package),
  identified from its text (``external_source_licenses.identify``); the
  package.xml ``<license>`` is the fallback for workspace models.
- **Model files:** an MJCF must compile in MuJoCo (that proves every
  mesh, texture, height field and skin it names exists); for a URDF, every
  mesh reference is resolved (``package://``, ``file://``, relative) and
  must exist. Xacro files are expanded first.
- **Textures inside meshes:** OBJ ``mtllib``/``map_*`` and COLLADA
  ``<init_from>`` images must exist next to the mesh.

Nothing is inferred from a project's name; a model without a license file
stays ``unresolved``.

  python3 scripts/robot_asset_audit.py --json audit.json --markdown audit.md
"""
import argparse
import json
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from external_source_licenses import LICENSE_FILE, identify  # noqa: E402

# Only <image> holds a file; <surface><init_from> names an image id.
DAE_IMAGE = re.compile(r'<image\b[^>]*>\s*<init_from>\s*([^<\s][^<]*?)\s*</init_from>', re.S)


def nearest_license(start, stop):
    """(license id, file) of the first license file from *start* up to *stop* (inclusive)."""
    start, stop = Path(start).resolve(), Path(stop).resolve()
    directory = start if start.is_dir() else start.parent
    while True:
        for path in sorted(directory.iterdir()) if directory.is_dir() else []:
            if path.is_file() and LICENSE_FILE.match(path.name):
                try:
                    found = identify(path.read_text(errors='replace')[:20000])
                except OSError:
                    continue
                if found:
                    return found, str(path)
        if directory == stop or stop not in directory.parents:
            return None, None
        directory = directory.parent


def package_xml_license(directory, stop):
    directory = Path(directory).resolve()
    while True:
        package = directory / 'package.xml'
        if package.is_file():
            tags = [t.text.strip() for t in ET.parse(package).getroot().findall('license') if t.text]
            return ', '.join(tags) or None, str(package)
        if directory == Path(stop).resolve() or directory == directory.parent:
            return None, None
        directory = directory.parent


def mesh_textures(mesh):
    """Image files a mesh refers to, with whether each exists."""
    mesh = Path(mesh)
    refs = []
    try:
        if mesh.suffix.lower() == '.obj':
            for line in mesh.read_text(errors='replace').splitlines():
                if line.startswith('mtllib '):
                    mtl = mesh.parent / line.split(None, 1)[1].strip()
                    refs.append(mtl)
                    if mtl.is_file():
                        for row in mtl.read_text(errors='replace').splitlines():
                            parts = row.split()
                            if parts and parts[0].lower().startswith('map_') and len(parts) > 1:
                                refs.append(mtl.parent / parts[-1])
        elif mesh.suffix.lower() == '.dae':
            for name in DAE_IMAGE.findall(mesh.read_text(errors='replace')):
                if not name.startswith('#'):
                    refs.append(mesh.parent / name.replace('file://', ''))
    except OSError:
        pass
    return [(str(ref), ref.is_file()) for ref in refs]


def resolve_uri(uri, base, search_roots):
    if uri.startswith('file://'):
        return Path(uri[7:])
    if uri.startswith('package://'):
        package, _, rest = uri[10:].partition('/')
        try:
            from ament_index_python.packages import get_package_share_directory
            candidate = Path(get_package_share_directory(package)) / rest
            if candidate.exists():
                return candidate
        except Exception:  # noqa: BLE001 - not an installed package
            pass
        for root in search_roots:
            for directory in [root] + [p for p in Path(root).rglob(package) if p.is_dir()][:5]:
                candidate = (directory if directory.name == package else directory / package) / rest
                if candidate.exists():
                    return candidate
        return Path(root if search_roots else base) / package / rest
    return (Path(base) / uri).resolve()


def urdf_text(path):
    if str(path).endswith('.xacro'):
        result = subprocess.run(['xacro', str(path)], capture_output=True, text=True, timeout=120)
        if result.returncode:
            raise ValueError('xacro failed: ' + result.stderr.strip()[-300:])
        return result.stdout
    return Path(path).read_text(errors='replace')


def check_urdf(path, search_roots):
    root = ET.fromstring(urdf_text(path))
    meshes, missing, textures = set(), [], []
    for mesh in root.iter('mesh'):
        uri = mesh.get('filename')
        if not uri:
            continue
        target = resolve_uri(uri, Path(path).parent, search_roots)
        if target in meshes:
            continue
        meshes.add(target)
        if not target.is_file():
            missing.append(uri)
        else:
            textures.extend(mesh_textures(target))
    return dict(format='urdf', meshes=len(meshes), missing_meshes=missing,
                mesh_textures=len(textures), missing_mesh_textures=sorted({t for t, ok in textures if not ok}))


def check_mjcf(path):
    import mujoco
    try:
        model = mujoco.MjModel.from_xml_path(str(path))
    except Exception as exc:  # noqa: BLE001 - the compile error is the finding
        return dict(format='mjcf', compiles=False, error=str(exc)[:300])
    textures, mesh_files = [], set()
    xml = ET.parse(path).getroot()
    compiler = xml.find('compiler')
    base = Path(path).parent
    meshdir = base / ((compiler.get('meshdir') or compiler.get('assetdir') or '') if compiler is not None else '')
    for mesh in xml.iter('mesh'):
        if mesh.get('file'):
            target = (meshdir / mesh.get('file')).resolve()
            if target.is_file() and target not in mesh_files:
                mesh_files.add(target)
                textures.extend(mesh_textures(target))
    # MuJoCo draws MJCF <texture>/<material>; material files an OBJ names are
    # not used by it, so a missing one is recorded, not a problem.
    return dict(format='mjcf', compiles=True, meshes=int(model.nmesh), textures=int(model.ntex),
                skins=int(model.nskin), mesh_textures=len(textures),
                unused_missing_mesh_materials=sorted({t for t, ok in textures if not ok}))


def check_sdf(path, models):
    """Mesh URIs of a PX4 Gazebo vehicle SDF (model://, relative), following its included models."""
    seen, missing, pending = set(), [], [Path(path)]
    meshes = 0
    while pending:
        sdf = pending.pop()
        if sdf in seen or not sdf.is_file():
            continue
        seen.add(sdf)
        root = ET.parse(sdf).getroot()
        for uri in root.iter('uri'):
            text = (uri.text or '').strip()
            if text.startswith('model://'):
                name, _, rest = text[8:].partition('/')
                target = models / name / rest if rest else models / name / 'model.sdf'
            else:
                target = sdf.parent / text
            if rest_is_mesh(target):
                meshes += 1
                if not target.is_file():
                    missing.append(text)
            elif target.name == 'model.sdf' or target.is_dir():
                pending.append(target if target.name == 'model.sdf' else target / 'model.sdf')
    return dict(format='sdf', meshes=meshes, missing_meshes=missing)


def rest_is_mesh(path):
    return Path(path).suffix.lower() in ('.dae', '.stl', '.obj', '.glb', '.gltf')


def audit_px4(name, row):
    import os
    px4 = Path(os.environ.get('PX4_ROOT', '/workspace/molar/px4/PX4-Autopilot'))
    row['license'], row['license_file'] = nearest_license(px4, px4) if px4.is_dir() else (None, None)
    vehicle = px4 / 'Tools/simulation/gz/models' / name.replace('px4_', '') / 'model.sdf'
    if vehicle.is_file():
        row['model'] = check_sdf(vehicle, vehicle.parent.parent)
    else:
        row['model'] = dict(format='generated', note='PX4 SITL vehicle whose plant is generated in code '
                            '(robot_lab_adapter/px4_mujoco_plant.py); no model file to check')
    return row


def audit_profile(name, profile, store, package_root):
    source_id = profile.get('source_id')
    row = dict(id=name, source_id=source_id)
    if (profile.get('drive') or {}).get('type') == 'px4':
        row['source_id'] = 'px4'
        audit_px4(name, row)
        if row['license_file']:
            row['license_kind'] = 'license file'
        problems = [] if row['license'] else ['license unresolved']
        if row['model'].get('missing_meshes'):
            problems.append('%d missing meshes' % len(row['model']['missing_meshes']))
        row['status'], row['problems'] = ('ok' if not problems else 'attention'), problems
        return row
    if source_id:
        revision_dirs = sorted((store / 'sources' / source_id).glob('*')) if (store / 'sources' / source_id).is_dir() else []
        revision = next((d for d in revision_dirs if d.is_dir() and d.name in str(profile.get('native_mjcf') or '')
                         or d.is_dir() and (d / str(profile.get('source_entry') or '')).exists()), None)
        source_root = revision
        entry = (revision / profile['source_entry']) if revision and profile.get('source_entry') else None
        row['source_revision'] = revision.name if revision else None
        if entry is not None and entry.exists():
            row['license'], row['license_file'] = nearest_license(entry, source_root)
            if not row['license']:
                # A sparse checkout may hold only the package, not the repository's root license.
                row['license'], row['license_file'] = package_xml_license(entry if entry.is_dir() else entry.parent,
                                                                          source_root)
        else:
            row['license'], row['license_file'] = None, None
        search_roots = [p for p in (source_root,) if p]
    else:
        xacro = profile.get('xacro')
        source_root = package_root
        entry = package_root / xacro if xacro and not Path(xacro).is_absolute() else (Path(xacro) if xacro else None)
        row['license'], row['license_file'] = (nearest_license(entry, package_root) if entry and entry.exists()
                                               else (None, None))
        if not row['license'] and entry is not None:
            row['license'], row['license_file'] = package_xml_license(entry.parent if entry.exists() else package_root,
                                                                      package_root)
        search_roots = [package_root, package_root.parent]
    if row['license_file'] and row['license_file'].endswith('package.xml'):
        row['license_kind'] = 'package.xml tag'
    elif row['license_file']:
        row['license_kind'] = 'license file'
    try:
        if profile.get('native_mjcf'):
            row['model'] = check_mjcf(profile['native_mjcf'])
        elif entry is not None and entry.exists() and entry.suffix in ('.urdf', '.xacro'):
            row['model'] = check_urdf(entry if not profile.get('xacro', '').endswith('.urdf')
                                      or not Path(profile['xacro']).is_absolute() else profile['xacro'], search_roots)
        elif profile.get('xacro') and Path(profile['xacro']).is_file():
            row['model'] = check_urdf(profile['xacro'], search_roots)
        else:
            row['model'] = dict(format=None, error='no model file recorded for this profile')
    except (ET.ParseError, OSError, ValueError, subprocess.TimeoutExpired) as exc:
        row['model'] = dict(format=None, error=str(exc)[:300])
    model = row['model']
    problems = []
    if not row['license']:
        problems.append('license unresolved')
    if model.get('error'):
        problems.append('model: ' + model['error'][:120])
    if model.get('missing_meshes'):
        problems.append('%d missing meshes' % len(model['missing_meshes']))
    if model.get('missing_mesh_textures'):
        problems.append('%d missing mesh textures' % len(model['missing_mesh_textures']))
    row['status'] = 'ok' if not problems else 'attention'
    row['problems'] = problems
    return row


def markdown(rows):
    ok = sum(r['status'] == 'ok' for r in rows)
    lines = ['# Robot model license and asset audit', '',
             'Generated by `scripts/robot_asset_audit.py`. Licenses come from the license file nearest each model '
             '(or its package.xml); MJCF models must compile; URDF meshes and mesh textures must exist.', '',
             'Summary: %d profiles, %d ok, %d need attention.' % (len(rows), ok, len(rows) - ok), '',
             '| Profile | Source | License | From | Model | Problems |', '|---|---|---|---|---|---|']
    for r in rows:
        model = r['model']
        desc = (model.get('format') or '-') + (' %d meshes' % model['meshes'] if 'meshes' in model else '')
        lines.append('| `%s` | %s | %s | %s | %s | %s |' % (r['id'], r['source_id'] or 'workspace', r['license'] or '-',
                     r.get('license_kind', '-'), desc, '; '.join(r['problems']).replace('|', '/') or '-'))
    return '\n'.join(lines) + '\n'


def main():
    import os
    import yaml
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--json', default='')
    parser.add_argument('--markdown', default='')
    options = parser.parse_args()
    from ament_index_python.packages import get_package_share_directory
    from robot_lab_utils.installed_assets import merge_installed_profiles
    share = Path(get_package_share_directory('robot_lab_robots'))
    profiles = merge_installed_profiles(yaml.safe_load((share / 'config' / 'robots.yaml').read_text()), 'robots')['robots']
    store = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'external_assets'
    rows = [audit_profile(name, profile, store, ROOT / 'src/robot_lab_robots') for name, profile in sorted(profiles.items())]
    summary = dict(profiles=len(rows), ok=sum(r['status'] == 'ok' for r in rows),
                   attention=sum(r['status'] != 'ok' for r in rows),
                   license_unresolved=sum(not r['license'] for r in rows))
    if options.json:
        Path(options.json).write_text(json.dumps(dict(summary=summary, profiles=rows), indent=1) + '\n')
    if options.markdown:
        Path(options.markdown).write_text(markdown(rows))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
