"""Launch Profile management — save, load, delete named configurations.

Two formats coexist:
- Legacy JSON config dicts (mode/simulator/robot/map_name/gui/algorithm),
  kept for backwards compatibility and automatically migrated on load.
- Resolved manifest YAML (the same format the CLI writes with
  ``robot-lab launch --out``), so profiles saved by the GUI and by the CLI
  resolve to identical manifests (R3.4).
"""
import json
from pathlib import Path

import yaml


def _profiles_dir():
    """Return the directory where profiles are stored."""
    d = Path.home() / ".robot_lab" / "profiles"
    d.mkdir(parents=True, exist_ok=True)
    return d


def list_profiles():
    """Return a sorted list of profile names (manifest YAML and legacy JSON)."""
    d = _profiles_dir()
    names = set()
    for f in d.glob("*.yaml"):
        names.add(f.stem)
    for f in d.glob("*.json"):
        names.add(f.stem)
    return sorted(names)


def profile_path(name):
    """Return the profile path for a name (prefers the manifest .yaml)."""
    yaml_path = _profiles_dir() / (name + ".yaml")
    if yaml_path.exists():
        return yaml_path
    legacy = _profiles_dir() / (name + ".json")
    if legacy.exists():
        return legacy
    return yaml_path


def save_profile(name, config):
    """Save a legacy configuration dict under the given name.

    config: dict with keys like mode, simulator, robot, map_name, gui,
            spawn_x, spawn_y, spawn_z, spawn_yaw, algorithm, extra_args
    """
    d = _profiles_dir()
    path = d / (name + ".json")
    # Don't store derived/empty values
    cleaned = {k: v for k, v in config.items()
               if v not in (None, "", "0.0", 0.0)}
    with open(path, "w") as fh:
        json.dump(cleaned, fh, indent=2, sort_keys=True)


def save_manifest(name, manifest):
    """Save a resolved manifest (the format the CLI --out writes)."""
    path = _profiles_dir() / (name + ".yaml")
    with open(path, "w") as fh:
        yaml.safe_dump(manifest, fh, sort_keys=False, default_flow_style=False)
    return str(path)


def load_profile(name):
    """Load a profile by name.

    New .yaml manifests are returned as-is. Legacy .json config dicts are
    returned unchanged (callers migrate them via
    gui_composition.migrate_legacy_selection). Returns None if not found.
    """
    path = profile_path(name)
    if not path.exists():
        return None
    with open(path) as fh:
        if path.suffix == ".yaml":
            return yaml.safe_load(fh)
        return json.load(fh)


def is_manifest(config):
    """True when a loaded profile is a resolved manifest, not a legacy dict."""
    return isinstance(config, dict) and "manifest_version" in config


def delete_profile(name):
    """Delete a profile by name.

    Only the named profile file is removed; user artifacts (logs, bags,
    maps, results) outside the profile directory are never touched.
    """
    path = profile_path(name)
    if path.exists():
        path.unlink()
        return True
    return False


def export_profile(name, target_path):
    """Export a profile to a specific path."""
    cfg = load_profile(name)
    if cfg is None:
        return False
    with open(target_path, "w") as fh:
        if is_manifest(cfg):
            yaml.safe_dump(cfg, fh, sort_keys=False, default_flow_style=False)
        else:
            json.dump(cfg, fh, indent=2, sort_keys=True)
    return True


def import_profile(source_path):
    """Import a profile from a path. Returns the name or None."""
    source_path = Path(source_path)
    with open(source_path) as fh:
        if source_path.suffix == ".yaml":
            cfg = yaml.safe_load(fh)
        else:
            cfg = json.load(fh)
    name = cfg.get("profile_name") or Path(source_path).stem
    if is_manifest(cfg):
        save_manifest(name, cfg)
    else:
        save_profile(name, cfg)
    return name


# Built-in default profiles
DEFAULT_PROFILES = {
    "Localization (PyBullet)": {
        "mode": "loc",
        "simulator": "pybullet",
        "robot": "bumperbot",
        "map_name": "celisca_floor_1",
        "algorithm": "amcl",
        "gui": True,
    },
    "SLAM (MuJoCo)": {
        "mode": "slam",
        "simulator": "mujoco",
        "robot": "bumperbot",
        "map_name": "celisca_floor_1",
        "algorithm": "slam_toolbox",
        "gui": True,
    },
    "Navigation (Isaac)": {
        "mode": "nav",
        "simulator": "isaac",
        "robot": "bumperbot",
        "map_name": "celisca_floor_1",
        "algorithm": "navfn",
        "gui": True,
    },
    "Display (Gazebo)": {
        "mode": "display",
        "simulator": "gazebo",
        "robot": "bumperbot",
        "map_name": "celisca_floor_1",
        "algorithm": "depthimage_to_laserscan",
        "gui": True,
    },
    "Headless Test (PyBullet)": {
        "mode": "loc",
        "simulator": "pybullet",
        "robot": "bumperbot",
        "map_name": "celisca_floor_1",
        "algorithm": "amcl",
        "gui": False,
    },
}


def ensure_defaults():
    """Write built-in default profiles only if they don't already exist.

    Never overwrites a user's existing profile (manifest or legacy JSON),
    so a user's saved work is never replaced by defaults.
    """
    for name, cfg in DEFAULT_PROFILES.items():
        if profile_path(name).exists():
            continue
        save_profile(name, cfg)
