"""
Headless GUI-adjacent composition logic (R3.4).

The Tkinter launcher delegates to this module so every decision is unit
testable without a display. It builds ``resolver.ExperimentRequest`` objects
from GUI selection state, resolves them with the SAME resolver/validator the
CLI uses (robot_lab_adapter.resolver + the R3.2 typed validator), and
serializes the same manifest YAML format that ``robot-lab launch --out``
writes — so a profile saved by the GUI and one written by the CLI resolve to
identical manifests.

Old GUI profiles (mode/simulator/robot/map_name/gui/algorithm) are migrated
to the manifest format on load.
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import yaml

from robot_lab_adapter.resolver import (
    ExperimentRequest,
    build_ros2_launch_command,
    resolve_experiment,
)

# Category taxonomy (single source: headless simulator_compat module).
from .simulator_compat import (
    ALGORITHM_CATEGORIES,
    ALGORITHM_SLOT_LABELS,
    MODE_ORDER,
    mode_algorithm_categories,
    mode_category,
    mode_default_algorithms,
    mode_steps,
)


@dataclass
class GuiCompositionSelection:
    """Independent selection state captured from the GUI controls."""
    robot_id: Optional[str] = None
    simulator: Optional[str] = None
    environment_id: Optional[str] = None
    scenario_id: Optional[str] = None
    algorithm_ids: Dict[str, str] = field(default_factory=dict)
    spawn: Optional[Dict[str, float]] = None
    reset: bool = True
    parameters: Dict[str, Any] = field(default_factory=dict)
    seed: Optional[int] = None
    namespace: str = ''
    mode: Optional[str] = None
    gui: Optional[str] = None


def environment_id_for_map_name(
    map_name: str,
    registry: Any,
) -> Optional[str]:
    """
    Best-effort map from a legacy map profile name to a registry environment.

    Prefers an exact environment id match, then a match on the environment's
    world-file key (e.g. sim_maps key 'simple_office' -> the environment whose
    world_file is robot_lab_maps/maps/simple_office/worlds/simple_office.world).
    Returns None when no registry environment corresponds; the resolver then
    falls back to its registry default and the GUI surfaces a warning.
    """
    if not map_name:
        return None
    environments = registry.environments.get_all()
    if map_name in environments:
        return map_name
    for env_id, env in environments.items():
        world = env.get('world_file') or ''
        parts = world.split('/')
        if len(parts) >= 5 and parts[2] == map_name:
            return env_id
    return None

def build_request(selection: GuiCompositionSelection) -> ExperimentRequest:
    """Translate a GUI selection into the shared resolver request."""
    return ExperimentRequest(
        robot_id=selection.robot_id,
        simulator=selection.simulator,
        environment_id=selection.environment_id,
        scenario_id=selection.scenario_id,
        algorithm_ids={k: v for k, v in selection.algorithm_ids.items() if v},
        spawn=selection.spawn,
        reset=selection.reset,
        parameters=selection.parameters,
        seed=selection.seed,
        namespace=selection.namespace,
        mode=selection.mode,
        gui=selection.gui,
    )


def resolve_selection(
    registry: Any,
    selection: GuiCompositionSelection,
) -> Tuple[bool, Dict[str, Any]]:
    """Resolve a GUI selection into the same manifest the CLI would emit."""
    return resolve_experiment(registry, build_request(selection))


def command_for_selection(
    registry: Any,
    selection: GuiCompositionSelection,
) -> List[str]:
    """Resolve and return the concrete ros2 launch command (ros2_command)."""
    ok, resolved = resolve_selection(registry, selection)
    if not ok:
        raise ValueError(
            "Cannot build command from invalid selection: "
            + "; ".join(resolved.get('errors', []))
        )
    return resolved['ros2_command'] or build_ros2_launch_command(resolved)


def validation_lines(registry: Any,
                     selection: GuiCompositionSelection) -> Tuple[List[str], List[str]]:
    """
    Validator-filtered diagnostics for the current selection.

    Returns (errors, warnings) naming exactly why a component is unsupported
    (category mismatch, missing sensor/capability/dimension) so the GUI can
    show maturity/readiness and unsupported reasons per selection.
    """
    ok, resolved = resolve_selection(registry, selection)
    if ok:
        warnings = list(resolved.get('warnings', []))
        aliases = resolved.get('aliases_applied', [])
        if aliases:
            warnings = ['Migrated: ' + a for a in aliases] + warnings
        return [], warnings
    return list(resolved.get('errors', [])), list(resolved.get('warnings', []))


def default_slot_for_mode(mode: str,
                          category_map: Dict[str, str],
                          mode_profiles: Optional[Dict[str, Any]] = None) -> Optional[str]:
    """Legacy explicit override (kept for compatibility). When empty, the
    sim_modes.yaml steps (via mode_profiles) decide the default slot,
    falling back to the canonical per-mode first category.
    """
    if category_map.get(mode):
        return category_map[mode]
    steps = mode_steps(mode, mode_profiles)
    for step in steps:
        if step["algorithm_category"]:
            return step["algorithm_category"]
    categories = mode_algorithm_categories(mode, mode_profiles)
    return categories[0] if categories else None

def manifest_running_summary(manifest: Dict[str, Any]) -> Dict[str, Any]:
    """
    Compact view of what a resolved manifest will actually run. The live
    monitor / status bar can display these choices and detect drift between
    what was selected and what is running.
    """
    return {
        'robot_id': manifest.get('robot_id'),
        'environment_id': manifest.get('environment_id'),
        'simulator': manifest.get('simulator'),
        'scenario_id': manifest.get('scenario_id'),
        'plugins': dict(manifest.get('plugins', {})),
        'launch_file': (manifest.get('launch') or {}).get('file'),
    }


def migrate_legacy_selection(
    cfg: Dict[str, Any],
    registry: Any,
    category_map: Dict[str, str],
) -> GuiCompositionSelection:
    """
    Convert an old-format GUI profile into the new selection format.

    Old keys: mode, simulator, robot, map_name, gui, algorithm (plus optional
    spawn_x/y/z/yaw). The old single 'algorithm' value is placed in the slot
    implied by the profile's mode; everything else stays in its slot when the
    profile already carried full composition keys.
    """
    algorithm_ids = dict(cfg.get('algorithm_ids') or {})
    legacy_algorithm = cfg.get('algorithm')
    if legacy_algorithm and not any(algorithm_ids.values()):
        slot = default_slot_for_mode(cfg.get('mode', ''), category_map)
        if slot:
            algorithm_ids[slot] = legacy_algorithm
        elif legacy_algorithm in registry.algorithms.get_all():
            algo = registry.algorithms.get(legacy_algorithm)
            algorithm_ids[algo.get('category', '')] = legacy_algorithm

    spawn = None
    if any(cfg.get(k) not in (None, '', '0.0') for k in
           ('spawn_x', 'spawn_y', 'spawn_z', 'spawn_yaw')):
        spawn = {
            k: float(cfg.get(f'spawn_{k}', 0.0) or 0.0)
            for k in ('x', 'y', 'z', 'yaw')
        }

    return GuiCompositionSelection(
        robot_id=cfg.get('robot'),
        simulator=cfg.get('simulator'),
        environment_id=environment_id_for_map_name(cfg.get('map_name', ''), registry),
        scenario_id=cfg.get('scenario_id'),
        algorithm_ids=algorithm_ids,
        spawn=spawn,
        reset=bool(cfg.get('reset', True)),
        parameters=dict(cfg.get('parameters') or {}),
        seed=cfg.get('seed'),
        namespace=cfg.get('namespace', ''),
    )

def manifest_to_yaml(manifest: Dict[str, Any]) -> str:
    """Serialize a manifest exactly like the CLI --out format."""
    from robot_lab_adapter.resolver import manifest_to_yaml as _to_yaml
    return _to_yaml(manifest)


def manifest_from_yaml(text: str) -> Dict[str, Any]:
    """Load a manifest written by the CLI or the GUI."""
    return yaml.safe_load(text)


def get_registry(config_dir: Optional[str] = None) -> Any:
    """Load the shared registry, preferring an explicit config dir."""
    if config_dir:
        from robot_lab_registry.catalog import Registry
        registry = Registry(config_dir)
        registry.load(config_dir)
        return registry
    try:
        from ament_index_python.packages import get_package_share_directory
        from robot_lab_registry.catalog import Registry
        config_dir = get_package_share_directory("robot_lab_registry") + "/config"
        registry = Registry(config_dir)
        registry.load(config_dir)
        return registry
    except Exception:
        raise RuntimeError(
            "Robot Lab registry unavailable; pass an explicit config_dir")