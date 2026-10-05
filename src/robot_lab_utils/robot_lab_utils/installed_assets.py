"""Installed extension profiles shared by the GUI, launch and registry.

Provisioning is an agent/install step, never a GUI download operation. Large
upstream models stay in the SSD store; unavailable models are not selectable.
"""
import os
from pathlib import Path

import yaml


def installation_root():
    return Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT',
                               '/workspace/molar/robot_lab_runtime')) / 'external_assets' / 'installed'


def installed_profiles(kind):
    if kind not in ('robots', 'maps'):
        raise ValueError('Unknown asset profile kind: ' + kind)
    path = installation_root() / (kind + '.yaml')
    if not path.is_file():
        return {}
    profiles = (yaml.safe_load(path.read_text()) or {}).get(kind, {})
    available = {}
    for name, profile in profiles.items():
        paths = [profile.get('xacro')] if kind == 'robots' else [profile.get('gazebo', {}).get('world_path')]
        if kind == 'robots' and profile.get('native_mjcf'):
            paths.append(profile['native_mjcf'])
        if paths and all(path and Path(path).is_file() for path in paths):
            available[name] = profile
    return available


def merge_installed_profiles(data, kind):
    result = dict(data)
    existing = dict(data.get(kind, {}))
    for name, profile in installed_profiles(kind).items():
        if name in existing:
            raise ValueError('Installed extension would replace existing ' + kind + ' profile: ' + name)
        existing[name] = profile
    result[kind] = existing
    return result


def installed_registry_entities(kind):
    profiles = installed_profiles(kind)
    path = installation_root() / ('registry_' + kind + '.yaml')
    if not path.is_file():
        return []
    return [entity for entity in yaml.safe_load(path.read_text()) or []
            if entity.get('id') in profiles]
