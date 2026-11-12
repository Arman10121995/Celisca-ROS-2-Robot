#!/usr/bin/env python3
"""Build the pinned VRX water-force plugins privately on the SSD for Fortress.

The derivative changes Gazebo library/header/namespace names only. It retains
the upstream Apache-2.0 source and force equations; no system Gazebo replacement.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

from catalog_external_assets import checkout, digest, load_source


def build(store, download=False):
    source = load_source('vrx_vendor')
    if download:
        checkout(source, 'vrx_gz/src', store)
    root = store / 'sources' / source['id'] / source['revision']
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
    if revision != source['revision']:
        raise ValueError('Unexpected VRX source revision')
    dirty = subprocess.check_output(['git', 'status', '--porcelain'], cwd=root, text=True)
    if dirty:
        raise ValueError('Preserve edited VRX source before provisioning')
    recipe = Path(__file__).read_bytes()
    identity = hashlib.sha256(recipe + source['revision'].encode()).hexdigest()
    target = store / 'adapters/vrx_fortress' / identity
    code = target / 'src'
    code.mkdir(parents=True, exist_ok=True)
    hashes = {}
    for stem in ('Surface', 'SimpleHydrodynamics', 'Wavefield'):
        for suffix in ('.hh', '.cc'):
            relative = 'vrx_gz/src/' + stem + suffix
            path = root / relative
            text = path.read_text().replace('gz/sim/', 'ignition/gazebo/')
            text = text.replace('gz/', 'ignition/').replace('gz::sim', 'ignition::gazebo')
            text = text.replace('gz::', 'ignition::').replace('namespace gz;', 'namespace ignition;')
            text = text.replace('sim::', 'gazebo::').replace('namespace sim;', 'namespace gazebo;')
            text = text.replace('GZ_ADD_PLUGIN', 'IGNITION_ADD_PLUGIN').replace('GZ_PROFILE', 'IGN_PROFILE').replace('GZ_UTILS_', 'IGN_UTILS_')
            # Console names changed with the same ABI rename.
            for before, after in [('gzerr', 'ignerr'), ('gzdbg', 'igndbg'), ('gzwarn', 'ignwarn'), ('gzmsg', 'ignmsg')]:
                text = text.replace(before, after)
            output=code / (stem + suffix)
            if not output.exists() or output.read_text()!=text:output.write_text(text)
            hashes[relative] = {'source_sha256': digest(path), 'derivative_sha256': digest(code / (stem + suffix))}
    shutil.copy2(root / 'LICENSE', code / 'LICENSE')
    cmake='''cmake_minimum_required(VERSION 3.16)
project(robot_lab_vrx_fortress)
set(CMAKE_CXX_STANDARD 17)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(Eigen3 REQUIRED)
foreach(plugin Surface SimpleHydrodynamics)
  add_library(${plugin} SHARED ${plugin}.cc)
  target_link_libraries(${plugin} ignition-gazebo6::ignition-gazebo6 ignition-plugin1::register Eigen3::Eigen)
endforeach()
target_sources(Surface PRIVATE Wavefield.cc)
'''
    output=code/'CMakeLists.txt'
    if not output.exists() or output.read_text()!=cmake:output.write_text(cmake)
    (target/'build-recipe.py').write_bytes(recipe)
    for command in (['cmake', '-S', str(code), '-B', str(target / 'build'), '-DCMAKE_BUILD_TYPE=Release'],
                    ['cmake', '--build', str(target / 'build'), '--parallel', '2']):
        subprocess.run(command, check=True, timeout=600)
    receipt = dict(source_id=source['id'], revision=revision, recipe_sha256=hashlib.sha256(recipe).hexdigest(),
                   source_files=hashes, license='Apache-2.0',
                   libraries={name: str(target / 'build' / ('lib' + name + '.so'))
                              for name in ('Surface', 'SimpleHydrodynamics')},
                   scope='Fortress ABI port of upstream water-force plugins; runtime qualification separate')
    receipt['library_sha256'] = {name: digest(Path(path)) for name, path in receipt['libraries'].items()}
    (target / 'build-receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--download', action='store_true')
    parser.add_argument('--store', type=Path, default=Path(os.environ.get(
        'ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'external_assets')
    args = parser.parse_args()
    if args.store.resolve().stat().st_dev == Path('/').stat().st_dev:
        parser.error('Builds must remain on the mounted SSD')
    print(json.dumps(build(args.store.resolve(), args.download), indent=2))
