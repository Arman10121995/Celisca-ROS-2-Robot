"""
Experiment resolver for Robot Lab (R3.3).

One resolver/executor shared by the CLI (and, from R3.4, the GUI). It takes
independent selector choices (robot, backend, world, scenario, the seven
algorithm slots, spawn pose, reset policy, parameter overrides and seed),
applies legacy aliases, validates the composition with the single typed
validator (robot_lab_registry.validation.check_composition) and emits a
concrete resolved manifest: every launch argument is a plain string, so a
dry-run can print exactly what a live execution will run without spawning
any process.

Legacy aliases preserved (R3.1 deferred them here):
- robots: unitree_go2 -> go2
- environments: terrain_rough -> outdoor_terrain
"""

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import yaml

MANIFEST_VERSION = 1

ALGORITHM_SLOTS = [
    'perception',
    'localization',
    'state_estimation',
    'sensor_fusion',
    'global_planning',
    'local_planning',
    'control',
]

ROBOT_ALIASES = {
    'unitree_go2': 'go2',
}

ENVIRONMENT_ALIASES = {
    'terrain_rough': 'outdoor_terrain',
}

# Canonical map: registry planner ID -> concrete nav2 plugin string used by
# the navigation stack (robot_lab_navigation/launch/navigation.launch.py).
# These are the implemented planners that change the active plugin at
# runtime; planners not listed here are recorded in the manifest but do not
# override the navigation stack's default plugin.
GLOBAL_PLANNER_PLUGINS = {
    # A* -> Smac 2D planner (the navigation stack's current default).
    'a_star_planner': 'nav2_smac_planner/SmacPlanner2D',
    # Dijkstra-based wavefront planners -> Navfn.
    'navfn_planner': 'nav2_navfn_planner/NavfnPlanner',
    'dijkstra_planner': 'nav2_navfn_planner/NavfnPlanner',
}

LOCAL_PLANNER_PLUGINS = {
    # DWB local planner (the stack default is the regulated pure pursuit
    # controller, which needs no override).
    'dwb_local_planner': 'dwb_core::DWBLocalPlanner',
}

BRINGUP_PACKAGE = 'robot_lab_bringup'
BRINGUP_LAUNCH_FILE = 'simulated_robot.launch.py'
WORLD_PACKAGE = 'robot_lab_maps'


@dataclass
class ExperimentRequest:
    """Independent selector choices for one experiment execution."""
    robot_id: Optional[str] = None
    simulator: Optional[str] = None
    environment_id: Optional[str] = None
    scenario_id: Optional[str] = None
    experiment_id: Optional[str] = None
    algorithm_ids: Dict[str, str] = field(default_factory=dict)
    spawn: Optional[Dict[str, float]] = None  # x, y, z, yaw
    reset: bool = True
    parameters: Dict[str, Any] = field(default_factory=dict)
    seed: Optional[int] = None
    namespace: str = ''


def apply_aliases(request: ExperimentRequest) -> Tuple[ExperimentRequest, List[str]]:
    """Resolve legacy selector aliases to canonical registry IDs."""
    applied = []
    resolved = ExperimentRequest(**vars(request))

    if resolved.robot_id and resolved.robot_id in ROBOT_ALIASES:
        applied.append(f"robot '{request.robot_id}' -> '{ROBOT_ALIASES[request.robot_id]}'")
        resolved.robot_id = ROBOT_ALIASES[request.robot_id]

    if resolved.environment_id and resolved.environment_id in ENVIRONMENT_ALIASES:
        legacy = request.environment_id
        resolved.environment_id = ENVIRONMENT_ALIASES[legacy]
        applied.append(f"environment '{legacy}' -> '{resolved.environment_id}'")

    return resolved, applied


def _default_of(registry_entries: Dict[str, Dict[str, Any]],
                preferred: Optional[str]) -> Optional[str]:
    """Pick a default entity ID: preferred, else first declared entry."""
    if preferred and preferred in registry_entries:
        return preferred
    if preferred:
        return None
    return next(iter(registry_entries), None)

def _world_arguments(environment: Dict[str, Any]) -> Dict[str, str]:
    """
    Derive concrete world/map launch arguments from an environment entity.

    Environment world files follow the robot_lab_maps convention
    'robot_lab_maps/maps/<key>/worlds/<key>.world'; the same <key> indexes
    sim_maps.yaml and names the sibling 'maps/<key>/maps/map.yaml' occupancy
    map used by the navigation stack.
    """
    args: Dict[str, str] = {}
    world_file = environment.get('world_file') or ''
    parts = world_file.split('/')
    # robot_lab_maps / maps / <key> / worlds / <key>.world
    if len(parts) >= 5 and parts[2] and parts[-1].endswith('.world'):
        key = parts[2]
        args['map_name'] = key
        args['world_package'] = WORLD_PACKAGE
        args['world_name'] = key
        args['world_path'] = f'maps/{key}/worlds/{key}.world'
        args['map_yaml'] = f'maps/{key}/maps/map.yaml'
    return args


def _spawn_arguments(environment: Dict[str, Any],
                     spawn: Optional[Dict[str, float]]) -> Dict[str, str]:
    """Concrete spawn pose: request override, else the environment default zone."""
    pose: Dict[str, float] = {}
    if spawn:
        pose = {k: float(v) for k, v in spawn.items() if k in ('x', 'y', 'z', 'yaw')}
    else:
        zones = environment.get('spawn_zones') or []
        if zones:
            zone_pose = (zones[0].get('pose') or {})
            pose = {k: float(zone_pose.get(k, 0.0)) for k in ('x', 'y', 'z', 'yaw')}
    return {f'spawn_{k}': f'{v:.3f}' for k, v in sorted(pose.items())}


def _planner_arguments(algorithm_ids: Dict[str, str]) -> Tuple[Dict[str, str], Dict[str, str]]:
    """Map selected planners onto concrete navigation-stack plugin arguments."""
    args: Dict[str, str] = {}
    plugins: Dict[str, str] = {}
    global_planner = algorithm_ids.get('global_planning')
    if global_planner and global_planner in GLOBAL_PLANNER_PLUGINS:
        plugin = GLOBAL_PLANNER_PLUGINS[global_planner]
        plugins['global_planning'] = plugin
        args['global_planner_plugin'] = plugin
    local_planner = algorithm_ids.get('local_planning')
    if local_planner and local_planner in LOCAL_PLANNER_PLUGINS:
        plugin = LOCAL_PLANNER_PLUGINS[local_planner]
        plugins['local_planning'] = plugin
        args['local_planner_plugin'] = plugin
    return args, plugins

def resolve_experiment(registry: Any, request: ExperimentRequest) -> Tuple[bool, Dict[str, Any]]:
    """
    Resolve a request into a concrete, executable manifest.

    Returns (True, manifest) or (False, {'errors': [...], 'warnings': [...]}).
    The manifest contains no lazy substitutions: dry-run prints exactly what
    live execution runs.
    """
    outcome: Dict[str, Any] = {'errors': [], 'warnings': []}

    resolved, aliases_applied = apply_aliases(request)

    # Start from a registered experiment when given, selectors override.
    base = {
        'robot_id': None,
        'environment_id': None,
        'simulator': None,
        'scenario_id': None,
        'algorithm_ids': {},
    }
    if resolved.experiment_id:
        experiment = registry.experiments.get(resolved.experiment_id)
        if not experiment:
            outcome['errors'].append(
                f"Experiment '{resolved.experiment_id}' not found in registry")
            return False, outcome
        base = {
            'robot_id': experiment.get('robot_id'),
            'environment_id': experiment.get('environment_id'),
            'simulator': experiment.get('simulator'),
            'scenario_id': experiment.get('scenario_id'),
            'algorithm_ids': dict(experiment.get('algorithm_ids') or {}),
        }

    robots = registry.robots.get_all()
    environments = registry.environments.get_all()

    robot_id = resolved.robot_id or base['robot_id'] or _default_of(robots, None)
    environment_id = resolved.environment_id or base['environment_id'] or _default_of(environments, None)

    if not robot_id:
        outcome['errors'].append('No robot selected and no default robot available')
    if not environment_id:
        outcome['errors'].append('No environment selected and no default environment available')
    if outcome['errors']:
        return False, outcome

    robot = registry.robots.get(robot_id)
    environment = registry.environments.get(environment_id)
    if robot is None:
        outcome['errors'].append(f"Robot '{robot_id}' not found in registry")
    if environment is None:
        outcome['errors'].append(f"Environment '{environment_id}' not found in registry")
    if outcome['errors']:
        return False, outcome

    simulator = resolved.simulator or base['simulator'] or environment.get('simulator')
    scenario_id = resolved.scenario_id if resolved.scenario_id is not None else base['scenario_id']

    algorithm_ids = dict(base['algorithm_ids'])
    for slot in ALGORITHM_SLOTS:
        value = resolved.algorithm_ids.get(slot)
        if value:
            algorithm_ids[slot] = value
    algorithm_ids = {k: v for k, v in algorithm_ids.items() if v}

    # Validate with the single typed validator (R3.2).
    from robot_lab_registry.validation import Composition, check_composition

    composition = Composition(
        robot_id=robot_id,
        environment_id=environment_id,
        simulator=simulator or '',
        scenario_id=scenario_id,
        algorithm_ids=algorithm_ids,
    )
    validation = check_composition(registry, composition)
    if not validation.valid:
        outcome['errors'].extend(validation.errors)
        return False, outcome

    # Concrete launch arguments.
    launch_args: Dict[str, str] = {
        'robot_model': robot_id,
        'simulator': simulator or 'gazebo',
        'use_sim_time': 'false' if simulator == 'real' else 'true',
    }
    launch_args.update(_world_arguments(environment))
    launch_args.update(_spawn_arguments(environment, resolved.spawn))
    planner_args, plugins = _planner_arguments(algorithm_ids)
    launch_args.update(planner_args)

    manifest = {
        'manifest_version': MANIFEST_VERSION,
        'resolved_from': {
            'robot_id': request.robot_id,
            'environment_id': request.environment_id,
            'simulator': request.simulator,
            'scenario_id': request.scenario_id,
            'experiment_id': request.experiment_id,
            'algorithm_ids': {k: v for k, v in request.algorithm_ids.items() if v},
            'spawn': request.spawn,
            'reset': request.reset,
            'parameters': request.parameters,
            'seed': request.seed,
            'namespace': request.namespace,
        },
        'aliases_applied': aliases_applied,
        'robot_id': robot_id,
        'environment_id': environment_id,
        'simulator': simulator,
        'scenario_id': scenario_id,
        'algorithm_ids': algorithm_ids,
        'plugins': plugins,
        'reset': {
            'enabled': request.reset,
            'service': environment.get('reset_service') or '/robot_lab/reset',
        },
        'parameters': dict(request.parameters),
        'seed': request.seed,
        'namespace': request.namespace,
        'launch': {
            'package': BRINGUP_PACKAGE,
            'file': BRINGUP_LAUNCH_FILE,
            'arguments': launch_args,
        },
    }
    manifest['ros2_command'] = build_ros2_launch_command(manifest)

    if validation.warnings:
        manifest['warnings'] = list(validation.warnings)

    return True, manifest

def build_ros2_launch_command(manifest: Dict[str, Any]) -> List[str]:
    """Build the concrete ros2 launch command for a resolved manifest."""
    launch = manifest['launch']
    command = ['ros2', 'launch', launch['package'], launch['file']]
    for key, value in launch['arguments'].items():
        command.append(f'{key}:={value}')
    return command


def execute_manifest(manifest: Dict[str, Any],
                     execute: bool = False) -> Tuple[List[str], Optional[Any]]:
    """
    Dry-run: return the command without spawning anything.

    Live execution: spawn `ros2 launch` with the manifest's concrete command.
    """
    command = manifest.get('ros2_command') or build_ros2_launch_command(manifest)
    if not execute:
        return command, None

    import subprocess

    process = subprocess.Popen(command)
    return command, process


def manifest_to_yaml(manifest: Dict[str, Any]) -> str:
    """Serialize a manifest to YAML for dry-run output / saved profiles."""
    return yaml.safe_dump(manifest, sort_keys=False, default_flow_style=False)
