"""Launch Profile management — save, load, delete named configurations."""
import json
import os
from pathlib import Path


def _profiles_dir():
    """Return the directory where profiles are stored."""
    d = Path.home() / ".robot_lab" / "profiles"
    d.mkdir(parents=True, exist_ok=True)
    return d


def list_profiles():
    """Return a sorted list of profile names."""
    d = _profiles_dir()
    names = []
    for f in d.glob("*.json"):
        names.append(f.stem)
    return sorted(names)


def save_profile(name, config):
    """Save a configuration dict under the given name.
    
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


def load_profile(name):
    """Load a configuration dict by name. Returns None if not found."""
    path = _profiles_dir() / (name + ".json")
    if not path.exists():
        return None
    with open(path) as fh:
        return json.load(fh)


def delete_profile(name):
    """Delete a profile by name."""
    path = _profiles_dir() / (name + ".json")
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
        json.dump(cfg, fh, indent=2, sort_keys=True)
    return True


def import_profile(source_path):
    """Import a profile from a path. Returns the name or None."""
    with open(source_path) as fh:
        cfg = json.load(fh)
    name = cfg.get("profile_name") or Path(source_path).stem
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
    """Write built-in default profiles if they don't already exist."""
    for name, cfg in DEFAULT_PROFILES.items():
        path = _profiles_dir() / (name + ".json")
        if not path.exists():
            save_profile(name, cfg)
