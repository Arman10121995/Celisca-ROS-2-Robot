"""Simulator availability and compatibility gating (headless, R3.4+).

The Tkinter launcher uses this module to decide which options stay
selectable BEFORE a launch command is built:

- which simulators are actually installed/configured on this host
  (Gazebo binaries, PyBullet/MuJoCo wheels, the Isaac Sim runtime);
- which modes a given simulator can physically run, based on the
  per-simulator sensor gaps documented in docs/status/support-matrix.md
  (the Isaac spawner publishes no 2D scan and no RGB-D; the PyBullet and
  MuJoCo bridges have no RGB-D camera implementation);
- automatic correction of an invalid robot/mode/simulator/environment
  selection cascade, returning the fixes so the GUI can show what changed.

Everything here is display-free and unit-testable.
"""

import os
import shutil
from importlib.util import find_spec
from typing import Any, Dict, List, Optional, Tuple

SIMULATOR_ORDER = ["gazebo", "isaac", "pybullet", "mujoco"]

SIMULATOR_LABELS = {
    "gazebo": "Gazebo",
    "isaac": "Isaac Sim",
    "pybullet": "PyBullet",
    "mujoco": "MuJoCo",
}

# Sensor/feature gaps per simulator backend (support-matrix.md):
# - Isaac Sim: no /scan publisher and no RGB-D bridge in the current spawner;
# - PyBullet/MuJoCo: bridges implement odometry/scan but no RGB-D camera.
SIMULATOR_FEATURE_GAPS: Dict[str, Tuple[str, ...]] = {
    "gazebo": (),
    "isaac": ("lidar_2d", "rgbd_camera"),
    "pybullet": ("rgbd_camera",),
    "mujoco": ("rgbd_camera",),
}

# Sensor features each bringup mode needs beyond the required_features
# declared in sim_modes.yaml (loc/slam/nav are 2D-map based -> need a scan;
# 3d_slam needs an RGB-D camera for RTAB-Map).
DEFAULT_MODE_FEATURES: Dict[str, Tuple[str, ...]] = {
    "loc": ("lidar_2d",),
    "slam": ("lidar_2d",),
    "nav": ("lidar_2d",),
    "3d_slam": ("rgbd_camera",),
}

MODE_ORDER = ["display", "loc", "slam", "3d_slam", "nav"]

# Canonical algorithm category taxonomy (mirrors registry algorithms.yaml).
ALGORITHM_CATEGORIES: List[str] = [
    "perception",
    "localization",
    "state_estimation",
    "sensor_fusion",
    "global_planning",
    "local_planning",
    "control",
]

ALGORITHM_SLOT_LABELS: Dict[str, str] = {
    "perception": "Perception",
    "localization": "Localization",
    "state_estimation": "State Estimation",
    "sensor_fusion": "Sensor Fusion",
    "global_planning": "Global Planning",
    "local_planning": "Local Planning",
    "control": "Control",
}

# Human-readable mode category + the algorithm categories that are
# selectable *within* that mode.  sim_modes.yaml carries the authoritative
# values (category / algorithm_categories); these defaults keep the headless
# layer and tests aligned when a mode file predates the schema.
MODE_CATEGORIES: Dict[str, str] = {
    "display": "Perception & Visualization",
    "loc": "Localization",
    "slam": "2D Mapping & Localization",
    "3d_slam": "3D Mapping & Localization",
    "nav": "Navigation",
}

MODE_ALGORITHM_SLOTS: Dict[str, Tuple[str, ...]] = {
    "display": ("perception",),
    "loc": ("localization", "state_estimation", "sensor_fusion"),
    "slam": ("localization", "state_estimation", "sensor_fusion", "perception"),
    "3d_slam": ("localization", "state_estimation", "perception"),
    "nav": ("global_planning", "local_planning", "control",
            "localization", "state_estimation", "sensor_fusion"),
}

# Sensor feature -> the concrete asset/topic the simulator must provide.
FEATURE_ASSET_NOTES: Dict[str, str] = {
    "lidar_2d": "/scan (2D LiDAR) publisher",
    "rgbd_camera": "RGB-D camera bridge",
}


def mode_category(mode: str,
                  mode_profiles: Optional[Dict[str, Any]] = None) -> str:
    """Human-readable category label for *mode* (config-driven)."""
    mode_cfg = (mode_profiles or {}).get(mode) or {}
    return mode_cfg.get("category") or MODE_CATEGORIES.get(mode, mode.title())


def mode_algorithm_categories(mode: str,
                              mode_profiles: Optional[Dict[str, Any]] = None
                              ) -> List[str]:
    """Algorithm categories selectable within *mode* (config-driven).

    Reads the mode's ``algorithm_categories`` from sim_modes.yaml when
    present, falling back to the canonical MODE_ALGORITHM_SLOTS table so a
    mode file that predates the schema stays correct.
    """
    mode_cfg = (mode_profiles or {}).get(mode) or {}
    configured = mode_cfg.get("algorithm_categories")
    if configured:
        return [
            category for category in configured
            if category in ALGORITHM_CATEGORIES
        ]
    return list(MODE_ALGORITHM_SLOTS.get(mode, ()))


def mode_requires_2d_map(mode: str,
                         mode_profiles: Optional[Dict[str, Any]] = None) -> bool:
    """Whether *mode* needs a pre-built 2D occupancy map on disk."""
    return bool((mode_profiles or {}).get(mode, {}).get("requires_2d_map", False))


def simulator_status(simulator: str,
                     env: Optional[Dict[str, str]] = None) -> Tuple[bool, str]:
    """(installed, reason) for one simulator backend on this host.

    *env* may be passed explicitly by tests; defaults to os.environ.
    """
    env = env if env is not None else os.environ
    simulator = str(simulator or "").lower()

    if simulator == "gazebo":
        if any(shutil.which(name) for name in ("gz", "gzserver", "ign")):
            return True, ""
        return False, "Gazebo executables (gz/gzserver) not found on PATH"

    if simulator == "pybullet":
        if find_spec("pybullet") is not None:
            return True, ""
        return False, "pybullet package not installed in this Python environment"

    if simulator == "mujoco":
        if find_spec("mujoco") is not None:
            return True, ""
        return False, "mujoco package not installed in this Python environment"

    if simulator == "isaac":
        isaac_python = env.get("ISAAC_PYTHON", "")
        if isaac_python and os.path.isfile(isaac_python):
            return True, ""
        try:
            if find_spec("isaacsim") is not None:
                return True, ""
        except (ImportError, ValueError):  # pragma: no cover — defensive
            pass
        return False, ("Isaac Sim runtime not configured - set ISAAC_PYTHON "
                       "to the isaacsim Python interpreter")

    return False, "Unknown simulator '%s'" % simulator


def available_simulators(env: Optional[Dict[str, str]] = None
                         ) -> Dict[str, Tuple[bool, str]]:
    """(installed, reason) for every known simulator backend."""
    return {sim: simulator_status(sim, env=env) for sim in SIMULATOR_ORDER}


def mode_feature_requirements(mode: str,
                              mode_profiles: Optional[Dict[str, Any]] = None
                              ) -> List[str]:
    """Features a mode requires (declared + known 2D-map/RGB-D needs)."""
    declared: List[str] = []
    mode_cfg = (mode_profiles or {}).get(mode) or {}
    for feature in mode_cfg.get("required_features") or []:
        if feature not in declared:
            declared.append(feature)
    for feature in DEFAULT_MODE_FEATURES.get(mode, ()):
        if feature not in declared:
            declared.append(feature)
    return declared


def simulator_supports_mode(simulator: str,
                            mode: str,
                            mode_profiles: Optional[Dict[str, Any]] = None
                            ) -> Tuple[bool, str]:
    """(ok, reason): can *simulator* physically run *mode*?

    Installation state is deliberately NOT checked here; it is handled
    separately by :func:`available_simulators`.
    """
    mode_cfg = (mode_profiles or {}).get(mode) or {}
    mode_sims = mode_cfg.get("simulators")
    if mode_sims and simulator not in mode_sims:
        return False, ("sim_modes.yaml does not list '%s' for mode '%s'"
                       % (simulator, mode))
    missing = [feature for feature in mode_feature_requirements(mode, mode_profiles)
               if feature in SIMULATOR_FEATURE_GAPS.get(simulator, ())]
    if missing:
        assets = ", ".join(
            FEATURE_ASSET_NOTES.get(feature, feature) for feature in missing)
        return False, ("%s not provided by the %s bridge (required by "
                       "mode '%s')" % (
                           assets,
                           SIMULATOR_LABELS.get(simulator, simulator),
                           mode))
    return True, ""


def allowed_simulators(mode: str,
                       mode_profiles: Optional[Dict[str, Any]] = None,
                       env: Optional[Dict[str, str]] = None
                       ) -> Dict[str, Tuple[bool, str]]:
    """Per-simulator (selectable, reason) for *mode*.

    Selectable = supports the mode's required features AND is installed
    and configured on this host.
    """
    result: Dict[str, Tuple[bool, str]] = {}
    for simulator in SIMULATOR_ORDER:
        ok_mode, why_mode = simulator_supports_mode(simulator, mode, mode_profiles)
        if not ok_mode:
            result[simulator] = (False, why_mode)
            continue
        installed, why_install = simulator_status(simulator, env=env)
        result[simulator] = (installed, why_install)
    return result


def allowed_modes(simulator: str,
                  mode_order: List[str],
                  mode_profiles: Optional[Dict[str, Any]] = None) -> List[str]:
    """Modes *simulator* can physically run, preserving *mode_order*."""
    return [mode for mode in (mode_order or [])
            if simulator_supports_mode(simulator, mode, mode_profiles)[0]]


def correction_for(current: Optional[str],
                   allowed: List[str],
                   fallback_order: Optional[List[str]] = None
                   ) -> Tuple[Optional[str], str]:
    """Pick a value from *allowed*, keeping *current* when it is valid.

    Returns (value, note); note is '' when nothing changed and a human
    readable correction otherwise.
    """
    if not allowed:
        return None, "no compatible option available"
    if current in allowed:
        return current, ""
    for candidate in (fallback_order or []):
        if candidate in allowed:
            return candidate, "'%s' not compatible -> '%s'" % (current, candidate)
    return allowed[0], "'%s' not compatible -> '%s'" % (current, allowed[0])
