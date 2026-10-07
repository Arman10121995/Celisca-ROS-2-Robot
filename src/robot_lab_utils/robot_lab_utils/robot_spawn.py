"""Robot-specific spawn overrides for a named map, with finite coordinates."""
import math


def map_spawn_override(robot, map_name):
    """Return only the selected map's pose; other maps retain their defaults.

    Short-range lidars need a visible wall rather than an arena-centre spawn.
    A global robot pose cannot express that without breaking other worlds.
    Explicit launch overrides still take precedence over this configuration.
    """
    poses = (robot or {}).get('spawn_by_map', {})
    if not isinstance(poses, dict):
        raise ValueError('spawn_by_map must be a mapping')
    pose = poses.get(map_name, {})
    if not isinstance(pose, dict) or set(pose)-{'x', 'y', 'z', 'yaw'}:
        raise ValueError('Invalid named robot spawn: ' + str(map_name))
    result = {}
    for key, value in pose.items():
        if isinstance(value, bool):
            raise ValueError('Robot spawn coordinate must be finite: ' + key)
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise ValueError('Robot spawn coordinate must be finite: ' + key) from None
        if not math.isfinite(value):
            raise ValueError('Robot spawn coordinate must be finite: ' + key)
        result[key] = value
    return result
