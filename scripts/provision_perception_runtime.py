#!/usr/bin/env python3
"""Extract official Humble scan/cloud converter packages onto SSD, without sudo.

Reuses the configured apt repository, records exact versions and SHA256s and
leaves /opt and dpkg untouched. Source scripts/ssd_env.sh again before the GUI.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile


TARGET='ros-humble-pointcloud-to-laserscan'


def output(*args, cwd=None):
    return subprocess.check_output(args,cwd=cwd,text=True,stderr=subprocess.STDOUT)


def main():
    base=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))
    if not base.is_dir() or not base.resolve().is_relative_to('/workspace') or base.stat().st_dev==Path('/').stat().st_dev:
        raise SystemExit('Source scripts/ssd_env.sh; perception runtime must use the mounted SSD')
    plan=output('apt-get','-s','--no-install-recommends','install',TARGET)
    packages=re.findall(r'^Inst (\S+) \((\S+)',plan,re.M)
    if not any(name==TARGET for name,_ in packages):
        print('Converter already installed by the OS; no SSD overlay needed')
        return
    # Non-ROS dependencies must be supplied by the host package manager. Do not
    # shadow libc, the dynamic linker or other system libraries in an overlay.
    if any(not name.startswith('ros-humble-') for name,_ in packages):
        raise SystemExit('Install missing non-ROS dependencies with the host package manager first:\n'+plan)
    root=base/'perception';root.mkdir(exist_ok=True)
    folder=Path(tempfile.mkdtemp(prefix='humble-',dir=root))
    downloads=folder/'debs';prefix=folder/'prefix'
    downloads.mkdir();prefix.mkdir();manifest=[]
    for name,version in packages:
        metadata=output('apt-cache','show',name+'='+version)
        match=re.search(r'^SHA256: ([a-fA-F0-9]{64})$',metadata,re.M)
        if not match:raise RuntimeError('No repository SHA256 for '+name)
        output('apt-get','download',name+'='+version,cwd=downloads)
        archives=list(downloads.glob(name.split(':')[0]+'_*.deb'))
        if len(archives)!=1:raise RuntimeError('Ambiguous downloaded archive for '+name)
        archive=archives[0];digest=hashlib.sha256(archive.read_bytes()).hexdigest()
        if digest!=match[1].lower():raise RuntimeError('Repository SHA256 mismatch for '+name)
        output('dpkg-deb','-x',str(archive),str(prefix))
        manifest.append(dict(package=name,version=version,sha256=digest,archive=str(archive)))
        print(name+' '+version,flush=True)
    (folder/'manifest.json').write_text(json.dumps(dict(packages=manifest,prefix=str(prefix),
        provenance='Configured apt repositories; hash-checked binary extraction without maintainer scripts',
        qualification='Provisioning only; no robot mission'),indent=2)+'\n')
    # Publish only a complete overlay; failed downloads leave the previous one.
    pending=root/('current.'+folder.name)
    pending.symlink_to(folder,target_is_directory=True)
    pending.replace(root/'current')
    print('Source scripts/ssd_env.sh again: '+str(root/'current/prefix/opt/ros/humble'))


if __name__=='__main__':main()
