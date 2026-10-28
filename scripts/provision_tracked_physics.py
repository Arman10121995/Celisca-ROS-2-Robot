#!/usr/bin/env python3
"""Build only Fortress's DART track/contact plugin in a private SSD prefix.

The system Fortress/DART ABI stays unchanged for existing robots. OSRF DART
6.13 headers/libraries are extracted, not installed over Ubuntu's packages.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sysconfig

PIN='a6c11b5be5267d3e1ac21f8a1eae6f9735cc2871'
VERSION='6.13.2+ds1-1~osrf2~jammy'
PACKAGES=('libdart6.13-dev','libdart6.13-collision-bullet-dev','libdart6.13-collision-ode-dev',
          'libdart6.13-utils-dev','libdart6.13-utils-urdf-dev','libdart-utils6.13','libdart-utils-urdf6.13',
          'libdart6.13-external-odelcpsolver-dev','libdart6.13-external-convhull-3d-dev',
          'libdart-core+collisions+odelcpsolver6.13')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--jobs',type=int,choices=range(1,5),default=2);args=parser.parse_args()
    base=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))/'native_tracks'
    base.mkdir(parents=True,exist_ok=True)
    if base.stat().st_dev==Path('/').stat().st_dev:raise ValueError('Private physics source/builds must stay on SSD')
    packages=base/'packages';packages.mkdir(exist_ok=True)
    def run(command,cwd=base,timeout=1200):
        with (base/'provision.log').open('a',buffering=1) as log:
            log.write(json.dumps(command)+'\n')
            subprocess.run(command,cwd=cwd,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=timeout)
    run(['apt-get','download',*[p+'='+VERSION for p in PACKAGES]],cwd=packages)
    dependencies=base/'dependencies'
    for path in packages.glob('*.deb'):run(['dpkg-deb','-x',str(path),str(dependencies)])
    source=base/'gz-physics'
    if not source.exists():run(['git','clone','--depth','1','--branch','ignition-physics5_5.4.0',
        'https://github.com/gazebosim/gz-physics.git',str(source)])
    revision=subprocess.check_output(['git','rev-parse','HEAD'],cwd=source,text=True).strip()
    if revision!=PIN:raise ValueError('Unexpected private physics source revision; preserve checkout and inspect')
    run(['cmake','-S',str(source),'-B',str(base/'build'),'-DCMAKE_BUILD_TYPE=Release','-DBUILD_TESTING=OFF',
        '-DCMAKE_INSTALL_PREFIX='+str(base/'prefix'),'-DCMAKE_PREFIX_PATH='+str(dependencies/'usr'),
        '-DDART_DIR='+str(dependencies/'usr/share/dart/cmake')])
    run(['cmake','--build',str(base/'build'),'--target','ignition-physics5-dartsim-plugin','-j'+str(args.jobs)])
    candidates=list((base/'build/lib').glob('libignition-physics5-dartsim-plugin.so.5*'))
    if not candidates:raise RuntimeError('Private native track engine was not built')
    engine=max(candidates,key=lambda p:len(p.name)).resolve()
    flags=base/'build/dartsim/CMakeFiles/ignition-physics5-dartsim-plugin.dir/flags.make'
    if 'DART_HAS_CONTACT_SURFACE' not in flags.read_text():raise RuntimeError('Built engine lacks native contact-surface support')
    record=dict(schema_version=1,source_revision=revision,dart_version=VERSION,engine=str(engine),
        engine_sha256=hashlib.sha256(engine.read_bytes()).hexdigest(),
        library_paths=[str(base/'build/lib'),str(dependencies/'usr/lib'/sysconfig.get_config_var('MULTIARCH'))],
        packages={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in packages.glob('*.deb')},
        scope='Private native contact-surface implementation; robot mission acceptance requires physical traces')
    (base/'receipt.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record,indent=2))


if __name__=='__main__':main()
