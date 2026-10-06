"""Explicit mounted sensor settings shared by the simulator launch adapters."""
import json
import math
import xml.etree.ElementTree as ET


_FRAMES = ('laser_link_name', 'camera_link_name', 'camera_optical_frame')
_INTEGERS = ('scan_samples', 'camera_width', 'camera_height')
_NUMBERS = ('scan_rate', 'scan_range_min', 'scan_range_max', 'camera_rate',
            'camera_horizontal_fov', 'camera_near', 'camera_far')


def sensor_parameters(config, urdf=None):
    """Validate optional explicit settings; an empty block keeps legacy defaults.

    Mounted links must exist when a description is supplied. Explicit imports
    must not silently fall back to another height or pretend to have a camera.
    """
    if isinstance(config, str):
        config = json.loads(config) if config.strip() else {}
    if config is None:
        config = {}
    if not isinstance(config, dict):
        raise ValueError('sensor_config must be an object')
    unknown = set(config) - set(_FRAMES + _INTEGERS + _NUMBERS)
    if unknown:
        raise ValueError('Unknown sensor settings: ' + ', '.join(sorted(unknown)))
    params = dict(config)
    for key in _FRAMES:
        if key in params and (not isinstance(params[key], str) or not params[key]
                              or any(c.isspace() for c in params[key])):
            raise ValueError('Invalid sensor frame: ' + key)
    for key in _INTEGERS:
        if key in params and (isinstance(params[key], bool)
                              or not isinstance(params[key], int) or params[key] <= 1):
            raise ValueError('Sensor dimensions/samples must be integers above one: ' + key)
    for key in _NUMBERS:
        if key not in params:
            continue
        value = params[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) \
                or not math.isfinite(value) or value < 0:
            raise ValueError('Invalid finite sensor setting: ' + key)
        params[key] = float(value)
    for near, far in (('scan_range_min', 'scan_range_max'), ('camera_near', 'camera_far')):
        if near in params and far in params and not 0 < params[near] < params[far]:
            raise ValueError('Sensor clipping interval must be positive and ordered: ' + near)
    if 'camera_horizontal_fov' in params and not 0 < params['camera_horizontal_fov'] < math.pi:
        raise ValueError('Camera field of view must be between zero and pi')
    if urdf and params:
        links = {link.get('name') for link in ET.fromstring(urdf).findall('link')}
        for key in _FRAMES:
            if key in params and params[key] not in links:
                raise ValueError('Explicit sensor frame missing from robot: ' + params[key])
    return params
