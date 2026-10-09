#!/usr/bin/env python3
"""Install exact-source native Stretch base/articulation candidates on SSD."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/robot_lab_utils'))
from robot_lab_utils.native_actuation import position_channels


def configure_profile(store, name, profile):
    import mujoco
    variant = {"menagerie_hello_robot_stretch": "stretch",
               "menagerie_hello_robot_stretch_3": "stretch_3"}[name]
    native = Path(profile['native_mjcf']).resolve()
    model = mujoco.MjModel.from_xml_path(str(native))
    channels = position_channels(model, allow_mobile_base=True)
    if not channels:
        raise ValueError('Source native mobile articulation missing: '+name)
    limits = dict(max_linear_m_s=.09, max_yaw_rad_s=.3, linear_acceleration_m_s2=.2,
        yaw_acceleration_rad_s2=.5, max_drive_extension_m=.15, wheel_velocity_gain=.3)
    manifest = dict(schema='robot_lab.native_stretch.v1', variant=variant,
        native_model=str(native), native_sha256=hashlib.sha256(native.read_bytes()).hexdigest(),
        source_id=profile['source_id'], source_entry=profile['source_entry'], limits=limits,
        implementation_state='implemented', validation_state='pending',
        contract='Original geometry, velocity servos/tendon motors, force limits and coupling. '
                 'Lab velocity gain/command bounds declared explicitly; no pose-write drive.')
    directory = store/'controllers'/name
    directory.mkdir(parents=True, exist_ok=True)
    configuration = directory/'mobile.json'
    configuration.write_text(json.dumps(manifest, indent=2)+'\n')
    profile['mobile_control_config'] = str(configuration)
    profile['native_articulation'] = dict(channels=channels, implementation_state='implemented', validation_state='pending')
    profile['locomotion_drive_limits'] = dict(max_speed=.09, max_angular_speed=.3,
        max_accel=.2, max_angular_accel=.5)
    profile['implementation_state'], profile['validation_state'] = 'implemented', 'pending'
    for feature in ('native_mobile_control', 'articulation_position_control'):
        if feature not in profile['features']:
            profile['features'].append(feature)
    note = ' Native combined base/arm/hand control is implemented for MuJoCo Display; enable Native mobile controls before Run. Stop the base before articulation; retract the arm before Drive. Runtime mission validation pending.'
    if note not in profile['notes']:
        profile['notes'] += note
    return profile


def main():
    store = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))/'external_assets'
    if store.stat().st_dev == Path('/').stat().st_dev:
        raise ValueError('Native mobile assets must stay on SSD')
    with (store/'locks/provision.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        path = store/'installed/robots.yaml'
        original = path.read_text()
        data = yaml.safe_load(original)
        for name, variant in (('menagerie_hello_robot_stretch', 'stretch'), ('menagerie_hello_robot_stretch_3', 'stretch_3')):
            profile = data['robots'][name]
            data['robots'][name] = configure_profile(store, name, profile)
            print(name+': installed native mobile/articulation candidate, validation pending')
        backup = path.with_name('robots-before-native-mobile.yaml')
        if not backup.exists():
            backup.write_text(original)
        temporary = path.with_suffix('.new')
        temporary.write_text(yaml.safe_dump(data, sort_keys=False))
        temporary.replace(path)


if __name__ == '__main__':
    main()
