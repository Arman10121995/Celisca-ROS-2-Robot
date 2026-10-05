#!/usr/bin/env python3
"""Stage pinned catalog assets on SSD and inspect real model dependencies.

Downloads never modify installed robot/map profiles. Native MJCF import checks
are geometry checks; they do not establish controller or mission support.
"""
import argparse
import ctypes
import fcntl
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / 'docs/status/asset-sources-2026-10-05.yaml'
# Also works from the GUI without an installed ROS Python overlay.
sys.path.insert(0,str(ROOT/'src/robot_lab_utils'))


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def relative_entry(value):
    path = PurePosixPath(value)
    if not value or path.is_absolute() or '..' in path.parts or value.startswith('-'):
        raise ValueError('Entry must be a relative path inside the pinned source')
    return path


def load_source(source_id, catalog=CATALOG):
    sources = yaml.safe_load(Path(catalog).read_text())['sources']
    return next(source for source in sources if source['id'] == source_id)


def manifest_path(destination, source, entry):
    identity = hashlib.sha256(entry.encode()).hexdigest()[:16]
    return Path(destination)/'manifests'/source['id']/source['revision']/(identity+'.json')


def run_git(args, cwd, timeout=900):
    environment = dict(os.environ, GIT_LFS_SKIP_SMUDGE='1', GIT_TERMINAL_PROMPT='0')
    process = subprocess.Popen(['git', '-c', 'core.hooksPath=/dev/null', *args],
        cwd=cwd, env=environment, start_new_session=True)
    try:
        code = process.wait(timeout=timeout)
        if code:
            raise RuntimeError('Git operation failed: '+args[0])
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def checkout(source, entry, destination):
    """One locked sparse checkout per source pin, expanded without resetting edits."""
    revision = source['revision']
    if not re.fullmatch(r'[0-9a-f]{40}', revision):
        raise ValueError('This directory entry needs a pinned upstream commit before download')
    if not source['repository'].startswith('https://github.com/'):
        raise ValueError('Only the inspected public GitHub source repositories can be staged')
    relative = relative_entry(entry)
    source_path = Path(destination)/'sources'/source['id']/revision
    lock = Path(destination)/'locks'/(source['id']+'-'+revision+'.lock')
    lock.parent.mkdir(parents=True, exist_ok=True)
    with lock.open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        source_path.mkdir(parents=True, exist_ok=True)
        if not (source_path/'.git').is_dir():
            run_git(['init', '-q'], source_path)
            run_git(['remote', 'add', 'origin', source['repository']], source_path)
            run_git(['config', 'remote.origin.promisor', 'true'], source_path)
            run_git(['config', 'remote.origin.partialclonefilter', 'blob:none'], source_path)
        remote = subprocess.check_output(['git', 'remote', 'get-url', 'origin'], cwd=source_path, text=True).strip()
        if remote != source['repository']:
            raise ValueError('Cached checkout belongs to a different repository')
        dirty = subprocess.check_output(['git', 'status', '--porcelain'], cwd=source_path, text=True)
        if dirty:
            raise ValueError('Cached source has local edits; preserve them in a separate checkout')
        head = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=source_path, capture_output=True, text=True)
        if head.returncode or head.stdout.strip() != revision:
            if head.returncode == 0:
                raise ValueError('Cached source revision differs from its pinned directory')
            run_git(['fetch', '--depth=1', '--filter=blob:none', 'origin', revision], source_path)
            run_git(['sparse-checkout', 'init', '--cone'], source_path)
            subtree = relative.parent if relative.suffix else relative
            run_git(['sparse-checkout', 'set', str(subtree)], source_path)
            run_git(['checkout', '--detach', 'FETCH_HEAD'], source_path)
        else:
            subtree = relative.parent if relative.suffix else relative
            run_git(['sparse-checkout', 'add', str(subtree)], source_path)
        verified = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=source_path, text=True).strip()
        if verified != revision:
            raise ValueError('Checkout did not resolve to the requested source pin')
    target = (source_path/str(relative)).resolve(strict=True)
    if not target.is_relative_to(source_path.resolve()):
        raise ValueError('Asset symlink escapes the source checkout')
    return source_path, target


def inspect_urdf(path, checkout_root):
    root = ET.parse(path).getroot()
    if root.tag != 'robot':
        raise ValueError('Selected file is not a URDF robot')
    meshes, missing, pointers = {}, [], []
    for node in [*root.findall('.//mesh'), *root.findall('.//texture')]:
        name = node.get('filename', '')
        if not name:
            continue
        if name.startswith('package://') or '$' in name:
            missing.append(name)
            continue
        asset = (path.parent/name.removeprefix('file://')).resolve()
        if not asset.is_relative_to(checkout_root.resolve()) or not asset.is_file():
            missing.append(name)
            continue
        with asset.open('rb') as stream:
            if stream.read(100).startswith(b'version https://git-lfs.github.com/spec/'):
                pointers.append(name)
                continue
        meshes[str(asset.relative_to(checkout_root))] = digest(asset)
    joints = [{'name': joint.get('name'), 'type': joint.get('type'),
               'limit': joint.find('limit').attrib if joint.find('limit') is not None else {}}
              for joint in root.findall('joint') if joint.get('type') != 'fixed']
    return {'format': 'urdf', 'links': len(root.findall('link')), 'joints': joints,
            'dependency_sha256': meshes, 'unresolved_dependencies': missing,
            'lfs_pointers': pointers, 'dependencies_complete': not missing and not pointers,
            'native_import': {'status': 'not_tested', 'reason': 'URDF staging does not actuate or import a simulator'}}


def inspect_mjcf(path):
    result = {'format': 'mjcf', 'native_import': {'status': 'not_tested'}}
    try:
        import mujoco
    except ImportError:
        result['native_import']['reason'] = 'MuJoCo is not installed in this Python environment'
        return result
    try:
        model = mujoco.MjModel.from_xml_path(str(path))
        result['native_import'] = {'status': 'import_checked', 'engine': 'mujoco',
            'version': mujoco.__version__, 'bodies': model.nbody, 'joints': model.njnt,
            'geometries': model.ngeom, 'actuators': model.nu}
    except Exception as exc:
        result['native_import'] = {'status': 'import_failed', 'reason': str(exc)}
    return result


def prepare_urdf_for_import(path, destination):
    """Resolve and convert actual meshes; never substitute a missing shape."""
    from robot_lab_utils.mesh_assets import stage_mesh_file
    root=ET.parse(path).getroot()
    destination.mkdir(parents=True,exist_ok=True)
    for node in root.findall('.//mesh'):
        uri=node.get('filename','')
        if '://' in uri and not uri.startswith('file://'):
            raise ValueError('Unresolved package resource: '+uri)
        original=(path.parent/uri.removeprefix('file://')).resolve(strict=True)
        staged=stage_mesh_file(str(original),str(destination),stem=digest(original)[:20])
        if not staged:
            raise ValueError('Cannot convert the actual mesh: '+uri)
        node.set('filename',staged)
    for node in root.findall('.//texture'):
        uri=node.get('filename','')
        node.set('filename',str((path.parent/uri.removeprefix('file://')).resolve(strict=True)))
    output=destination/'model.urdf'
    ET.ElementTree(root).write(output,encoding='unicode')
    return output


def inspect_pybullet(path, destination):
    try:
        import pybullet as bullet
    except ImportError:
        return {'status':'not_tested','reason':'PyBullet is not installed'}
    client=-1
    try:
        model_path=prepare_urdf_for_import(path,destination)
        client=bullet.connect(bullet.DIRECT)
        body=bullet.loadURDF(str(model_path),useFixedBase=True,physicsClientId=client)
        return {'status':'import_checked','engine':'pybullet','api_version':bullet.getAPIVersion(),
                'joints':bullet.getNumJoints(body,physicsClientId=client),
                'visual_shapes':len(bullet.getVisualShapeData(body,physicsClientId=client)),
                'derived_urdf':str(model_path),'derived_urdf_sha256':digest(model_path)}
    except Exception as exc:
        return {'status':'import_failed','reason':str(exc)}
    finally:
        if client>=0:bullet.disconnect(client)


def inspect(source, entry, source_path, target, derived_directory=None):
    models = []
    candidates = [target] if target.is_file() else sorted(target.glob('*.xml'))
    for path in candidates:
        if path.suffix == '.urdf':
            model={'path': str(path.relative_to(source_path)), 'sha256': digest(path), **inspect_urdf(path, source_path)}
            if derived_directory is not None and model['dependencies_complete']:
                model['native_import']=inspect_pybullet(path,derived_directory)
            models.append(model)
        elif path.suffix == '.xml':
            root = ET.parse(path).getroot()
            if root.tag == 'mujoco':
                models.append({'path': str(path.relative_to(source_path)), 'sha256': digest(path), **inspect_mjcf(path)})
    scope = target.parent if target.is_file() else target
    licenses = [p for directory in [source_path, scope] for p in directory.glob('*')
                if p.is_file() and p.name.lower().startswith(('license', 'copying', 'notice'))]
    return {'source_id': source['id'], 'repository': source['repository'], 'revision': source['revision'],
        'entry': entry, 'downloaded_path': str(target), 'stage': 'downloaded',
        'license_note': source['license_note'], 'license_files': {str(p.relative_to(source_path)): digest(p) for p in licenses},
        'entry_sha256': digest(target) if target.is_file() else None,
        'models': models, 'runnable_profile': False, 'mission_qualified': False,
        'scope': 'Downloaded source and named static/native model import checks; no controller or robot mission qualification'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog', type=Path, default=CATALOG)
    parser.add_argument('--source', required=True)
    parser.add_argument('--entry', required=True)
    parser.add_argument('--output-dir', type=Path, default=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))/'external_assets')
    args = parser.parse_args()
    parent = os.getppid()
    if ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0) != 0 or os.getppid() != parent:
        parser.error('Unable to bind asset staging to its parent process')
    def terminate(_sig, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    source = load_source(args.source, args.catalog)
    entries = [e for e in source['entries'] if isinstance(e, str)]
    if args.entry not in entries:
        parser.error('Choose an actual pinned catalog entry; directory links need individual upstream pins')
    workspace = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))
    destination = args.output_dir.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    if destination.stat().st_dev != workspace.stat().st_dev:
        parser.error('Source checkouts and downloaded assets must live on the workspace SSD')
    try:
        source_path, target = checkout(source, args.entry, destination)
        derived=destination/'derived'/source['id']/source['revision']/hashlib.sha256(args.entry.encode()).hexdigest()[:16]
        report = inspect(source, args.entry, source_path, target, derived)
        report['inspected_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        output = manifest_path(destination, source, args.entry)
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix('.new')
        temporary.write_text(json.dumps(report, indent=2)+'\n')
        temporary.replace(output)
        print(json.dumps({'manifest': str(output), **report}, indent=2), flush=True)
        return 0
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
