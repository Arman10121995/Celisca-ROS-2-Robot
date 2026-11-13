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
    profiles = (yaml.safe_load(path.read_text()) or {}).get(kind, {}) if path.is_file() else {}
    if kind == 'maps':
        # Terrain generated in the GUI (terrain_generator, R6.7) has its own
        # registry, so provisioning can rewrite maps.yaml without losing it.
        from .terrain_generator import registered_profiles
        for name, profile in registered_profiles().items():
            if name in profiles:
                raise ValueError('Generated terrain would replace installed map: ' + name)
            profiles[name] = profile
    available = {}
    for name, profile in profiles.items():
        paths = [profile.get('xacro')] if kind == 'robots' else [profile.get('gazebo', {}).get('world_path')]
        if kind == 'robots' and profile.get('native_mjcf'):
            paths.append(profile['native_mjcf'])
        if paths and all(path and Path(path).is_file() for path in paths):
            if kind == 'robots':
                from .native_hand import hand_profile
                profile = hand_profile(name, profile)
            if kind == 'robots' and profile.get('native_articulation') and not profile.get('mobile_control_config'):
                from .native_planning_description import ARM_FRAMES
                if name in ARM_FRAMES and profile.get('source_id') == 'mujoco_menagerie':
                    profile = dict(profile, native_arm_planning=dict(source_frame=ARM_FRAMES[name],
                        implementation_state='experimental', validation_state='pending', backend='mujoco',
                        algorithms='MoveIt KDL + OMPL RRTConnect + native position trajectory'))
            available[name] = profile
    return _apply_overlays(available, kind)


def _overlay_path():
    """robot_lab_robots/config/installed_overlays.yaml: repository-kept
    additions to provisioned profiles (re-provisioning must not erase them)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('robot_lab_robots')) / 'config' / 'installed_overlays.yaml'
    except Exception:
        return Path(__file__).resolve().parents[2] / 'robot_lab_robots' / 'config' / 'installed_overlays.yaml'


def _deep_merge(base, extra):
    merged = dict(base)
    for key, value in extra.items():
        merged[key] = _deep_merge(merged[key], value) if isinstance(value, dict) and \
            isinstance(merged.get(key), dict) else value
    return merged


def _apply_overlays(profiles, kind):
    path = _overlay_path()
    if not path.is_file():
        return profiles
    overlays = (yaml.safe_load(path.read_text()) or {}).get(kind) or {}
    return {name: _deep_merge(profile, overlays[name]) if name in overlays else profile
            for name, profile in profiles.items()}


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
    entities = [entity for entity in (yaml.safe_load(path.read_text()) or [] if path.is_file() else [])
                if entity.get('id') in profiles]
    if kind == 'maps':
        from .terrain_generator import registry_entities
        entities += [entity for entity in registry_entities() if entity['id'] in profiles]
    if kind == 'robots':
        for index, entity in enumerate(entities):
            profile = profiles[entity['id']]
            if profile.get('native_hand'):
                entities[index] = dict(entity, description=profile['notes'],
                    tags=list(dict.fromkeys([*entity.get('tags', []), 'hand', 'experimental_control'])))
    return entities
