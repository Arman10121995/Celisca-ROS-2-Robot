#!/usr/bin/env python3
"""Install official Humble Open-RMF deb contents on SSD without sudo.

Versions and repository SHA256s are recorded. This is a provisioner, never a
GUI download operation; the resulting overlay leaves /opt and dpkg unchanged.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile


def run(*args, cwd=None):
    return subprocess.check_output(args, cwd=cwd, text=True, stderr=subprocess.STDOUT)


def main():
    base = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))
    if not str(base.resolve()).startswith('/workspace/') or base.stat().st_dev == Path('/').stat().st_dev:
        raise SystemExit('Source scripts/ssd_env.sh; Open-RMF must be provisioned on the mounted SSD')
    root = base / 'rmf'
    root.mkdir(exist_ok=True)
    plan = run('apt-get', '-s', '--no-install-recommends', 'install', 'ros-humble-rmf-fleet-adapter-python')
    packages = re.findall(r'^Inst (\S+) \((\S+)', plan, re.M)
    # Always include the target; apt may already have it installed on another host.
    if not any(name == 'ros-humble-rmf-fleet-adapter-python' for name, _ in packages):
        print('Open-RMF is already installed by the OS; no SSD overlay needed')
        return
    folder = Path(tempfile.mkdtemp(prefix='humble-', dir=root))
    downloads, prefix = folder / 'debs', folder / 'prefix'
    downloads.mkdir(); prefix.mkdir()
    manifest = []
    for name, version in packages:
        metadata = run('apt-cache', 'show', name + '=' + version)
        match = re.search(r'^SHA256: (\w+)$', metadata, re.M)
        if not match:
            raise RuntimeError('No repository SHA256 for ' + name)
        run('apt-get', 'download', name + '=' + version, cwd=downloads)
        archives = list(downloads.glob(name.replace(':arm64', '') + '_*.deb'))
        if len(archives) != 1:
            raise RuntimeError('Ambiguous downloaded archive for ' + name)
        archive = archives[0]
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        if digest != match[1]:
            raise RuntimeError('Repository SHA256 mismatch for ' + name)
        run('dpkg-deb', '-x', str(archive), str(prefix))
        manifest.append(dict(package=name, version=version, sha256=digest, archive=str(archive)))
        print(name + ' ' + version, flush=True)
    (folder / 'manifest.json').write_text(json.dumps(dict(packages=manifest,
        provenance='Official configured apt repositories; extracted binaries, not a source build',
        prefix=str(prefix), qualification='Provisioning only, not an RMF mission'), indent=2) + '\n')
    candidate = root / 'current.new'
    candidate.unlink(missing_ok=True)
    candidate.symlink_to(folder, target_is_directory=True)
    candidate.replace(root / 'current')
    print('Open-RMF overlay: ' + str(root / 'current/prefix/opt/ros/humble'))


if __name__ == '__main__':
    main()
