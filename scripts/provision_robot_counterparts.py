#!/usr/bin/env python3
"""Install reviewed open-source counterparts into the normal robot catalog.

Existing source profiles/qualification are preserved. New variants explicitly
separate implemented experimental workflows from measured runtime support.
"""
import argparse
import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.vacuum_sensors import create3_hazard_derivative
from provision_extension_assets import checkpoint_robots, digest, load_source, robot_entity, write_json


def turtlebot4_cleaning(store):
    original_id, name = 'asset_turtlebot4_lite', 'asset_turtlebot4_lite_cleaning'
    installed = store / 'installed'
    profiles = yaml.safe_load((installed / 'robots.yaml').read_text())['robots']
    base = profiles[original_id]
    source = load_source('turtlebot4_vendor')
    check_path = installed / 'checks' / ('turtlebot4_vendor-' + hashlib.sha256(
        base['source_entry'].encode()).hexdigest()[:16] + '.json')
    check = json.loads(check_path.read_text())
    description = Path(base['xacro'])
    upstream = description.parent / 'input.urdf'
    if (check.get('revision') != source['revision'] or digest(description) != check['drive_urdf_sha256']
            or digest(upstream) != check['model_sha256']):
        raise ValueError('Original TurtleBot source/drive bytes differ from their installed import receipt')
    text, sensors = create3_hazard_derivative(description.read_text(), upstream.read_text(), name)
    identity = hashlib.sha256((text + '\ncreate3-hazards-v1').encode()).hexdigest()
    target = store / 'models' / name / identity
    target.mkdir(parents=True, exist_ok=True)
    output = target / 'cleaning.urdf'
    if output.exists() and output.read_text() != text:
        raise ValueError('A content-addressed cleaning derivative was changed')
    output.write_text(text)
    profile = copy.deepcopy(base)
    for key in ('runtime_screens', 'qualification_recheck', 'qualification_state', 'default_algorithms',
                'supported_modes_by_simulator', 'experimental_modes_by_simulator'):
        profile.pop(key, None)
    profile.update(name=name, xacro=str(output), supported_simulators=['gazebo'],
        supported_modes=['display'], default_mode='display', default_map='nav_empty',
        supported_modes_by_simulator={'gazebo': ['display']},
        experimental_modes_by_simulator={'gazebo': ['display', 'loc', 'slam', '3d_slam', 'nav']},
        qualification_state='experimental_unqualified', supports_room_vacuum=True,
        features=['velocity_base', 'lidar_2d', 'rgbd_camera', 'bumper', 'cliff'],
        gazebo_max_physics_step_s=.002,
        vacuum=dict(radius=.171, cleaning_width=.24, side_brush_reach=0., max_speed=.3,
            base_frame='base_link', bumper_topic='/bumper',
            cliff_topics=['/cliff/front_left', '/cliff/front_right', '/cliff/side_left', '/cliff/side_right'],
            cliff_absolute_threshold_m=.08, require_cliff_feedback=True),
        notes='Source TurtleBot 4 Lite / Create 3 cleaning variant: unchanged chassis, wheels, suspension, '
              'controller and sensor geometry; source bumper and four cliff sensors restored at 20 Hz. '
              'Cleaning coverage/battery are lab models; there is no vacuum hardware. Gazebo Display drive '
              'implemented, higher modes require explicit experimental opt-in; new runtime acceptance pending.')
    evidence = target / 'integration.json'
    receipt = dict(recipe='create3-hazards-v1', source_id=source['id'], revision=source['revision'],
        source_drive=str(description), source_drive_sha256=digest(description),
        source_expanded_description=str(upstream), source_expanded_sha256=digest(upstream),
        description=str(output), description_sha256=digest(output), restored_sensors=sensors,
        source_dependencies=check.get('source_dependencies', []), runtime_mission_qualified=False,
        scope='Source/native hazard integration; original profile and its certificates unchanged')
    write_json(evidence, receipt)
    entity = robot_entity(name, profile, source, base['source_entry'], evidence)
    record = dict(source_id=source['id'], revision=source['revision'],
        entry='robot-lab/Create3-cleaning-variant-v1', status='installed', check=str(evidence),
        profiles=[dict(id=name, kind='robot', support='Gazebo Display; explicit experimental cleaning modes',
                       notes=profile['notes'])])
    checkpoint_robots(store, {name: profile}, [entity], [record])
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', choices=['turtlebot4_cleaning'], default='turtlebot4_cleaning')
    parser.add_argument('--store', type=Path, default=Path(os.environ.get(
        'ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'external_assets')
    args = parser.parse_args()
    store = args.store.resolve()
    if store.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Counterpart assets must remain on the mounted SSD')
    with (store / 'locks/provision.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        print(json.dumps(turtlebot4_cleaning(store), indent=2))


if __name__ == '__main__':
    main()
