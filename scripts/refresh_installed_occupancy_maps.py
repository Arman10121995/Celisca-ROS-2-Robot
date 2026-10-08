#!/usr/bin/env python3
"""Regenerate installed extension grids without moving their proven spawns.

Sources/builds/logs stay on SSD. Execute serially with no other simulator
running. Original grids and profile snapshots are preserved. Real exports
are separate from robot navigation qualification.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time

import yaml

from integrate_external_worlds import generate_map


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_yaml(path, value):
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        yaml.safe_dump(value, stream, sort_keys=False)
    try:
        temporary.chmod(path.stat().st_mode & 0o777)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--store', type=Path, default=Path(os.environ.get(
        'ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))/'external_assets')
    parser.add_argument('--map', action='append', dest='maps', help='Exact installed map ID; repeat, or omit for all extensions')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    store = args.store.resolve(strict=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if store.stat().st_dev == Path('/').stat().st_dev or args.output.parent.stat().st_dev != store.stat().st_dev:
        parser.error('Installed assets and trial output must be on the workspace SSD')
    args.output.mkdir(exist_ok=False)
    out = args.output
    (out/'producer.py').write_bytes(Path(__file__).read_bytes())
    maps_path, registry_path = store/'installed/maps.yaml', store/'installed/registry_maps.yaml'
    locks = store/'locks'
    locks.mkdir(exist_ok=True)
    report = dict(scope='Actual occupancy exports/cache reuse only; no motion/navigation qualification.',
        revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        source_sha256={str(p): sha(p) for p in [Path(__file__),
            Path(__file__).parent/'external_world_repairs.py',
            Path(__file__).parents[1]/'src/robot_lab_maps/tools/generate_occupancy_map.py']}, records=[])
    with (locks/'provision.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        profiles = yaml.safe_load(maps_path.read_text())
        registry = yaml.safe_load(registry_path.read_text())
        selected = args.maps or list(profiles['maps'])
        unknown = set(selected)-set(profiles['maps'])
        if unknown:
            parser.error('Unknown installed maps: '+', '.join(sorted(unknown)))
        for p in [maps_path, registry_path]:
            (out/('before-'+p.name)).write_bytes(p.read_bytes())
        for name in selected:
            profile = profiles['maps'][name]
            world = Path(profile['gazebo']['world_path'])
            record = dict(map=name, world=str(world), previous_map=profile.get('map'),
                spawn=dict(profile['spawn']), outcome='failed')
            start = time.monotonic()
            try:
                seed = [float(profile['spawn'][k]) for k in ('x', 'y')]
                maps_directory = world.parent.parent/'maps'
                previous_log = maps_directory/'generation.log'
                if previous_log.is_file():
                    (out/(name+'-previous-generation.log')).write_bytes(previous_log.read_bytes())
                existing = set(maps_directory.rglob('*.generation.json'))
                generated, receipt = generate_map(world, maps_directory, seed)
                grid = Path(generated['output_yaml'])
                record.update(outcome='exported' if receipt not in existing else 'verified_cache',
                    generation_report=str(receipt), generation_report_sha256=sha(receipt),
                    world_sha256=sha(world), output_yaml=str(grid),
                    free_cells=generated['free_cells'], occupied_cells=generated['occupied_cells'],
                    unknown_cells=generated['unknown_cells'], recipe=generated['projection_recipe'])
                profile['map'] = dict(has_2d_map=True, package='robot_lab_maps', path=str(grid))
                for entity in registry:
                    if entity.get('id') == name:
                        entity.update(occupancy_yaml=str(grid), occupancy_map=str(grid.with_suffix('.pgm')))
                atomic_yaml(maps_path, profiles)
                atomic_yaml(registry_path, registry)
            except Exception as exc:
                record['exception'] = repr(exc)
            finally:
                latest_log = world.parent.parent/'maps/generation.log'
                if latest_log.is_file():
                    copied = out/(name+'-generation.log')
                    copied.write_bytes(latest_log.read_bytes())
                    record['generation_log'] = dict(path=str(copied), sha256=sha(copied))
                record['wall_s'] = round(time.monotonic()-start, 3)
                report['records'].append(record)
                (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
                print(name, record['outcome'], record.get('exception', ''), flush=True)
        report['profile_sha256'] = {str(p): sha(p) for p in [maps_path, registry_path]}
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    return int(any(r['outcome'] == 'failed' for r in report['records']))


if __name__ == '__main__':
    raise SystemExit(main())
