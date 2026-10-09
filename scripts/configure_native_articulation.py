#!/usr/bin/env python3
"""Register implemented native actuator controls without reimporting assets.

Inspecting authored actuator topology is installation, not a physical mission.
Original models, Panda certificates, other robot profiles and maps are retained.
"""
import argparse
import fcntl
import os
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/robot_lab_utils'))
from robot_lab_utils.native_actuation import position_channels


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--store', type=Path, default=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT',
        '/workspace/molar/robot_lab_runtime'))/'external_assets')
    args = parser.parse_args()
    store = args.store.resolve()
    if store.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Installed assets must remain on the mounted workspace SSD')
    import mujoco
    with (store/'locks/provision.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        path = store/'installed/robots.yaml'
        original = path.read_text()
        data = yaml.safe_load(original)
        updated = []
        for name, profile in data['robots'].items():
            if not profile.get('native_mjcf') or profile.get('arm_control') or profile.get('locomotion_policy_config'):
                continue
            model = mujoco.MjModel.from_xml_path(profile['native_mjcf'])
            channels = position_channels(model)
            if not channels:
                continue
            profile['native_articulation'] = dict(channels=channels,
                implementation_state='implemented', validation_state='pending')
            features = profile.setdefault('features', [])
            if 'articulation_position_control' not in features:
                features.append('articulation_position_control')
            note = ' Native fixed-base position-actuator jogging is implemented in Arm; enable Native joint controls before Run. Physical validation pending.'
            if note not in profile.get('notes', ''):
                profile['notes'] = profile.get('notes', '')+note
            updated.append(name)
        backup = path.with_name('robots-before-native-articulation.yaml')
        if not backup.exists():
            backup.write_text(original)
        temporary = path.with_suffix('.new')
        temporary.write_text(yaml.safe_dump(data, sort_keys=False))
        temporary.replace(path)
        print('Implemented controls registered for '+str(len(updated))+' native fixed-base models; runtime validation pending.')
        print('\n'.join(updated))


if __name__ == '__main__':
    main()
