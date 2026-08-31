"""
Independent selectors for Robot Lab components.

This module provides CLI and Python API for selecting:
- Robot
- Environment  
- Simulator
- Scenario
- Perception pipeline
- Localization method
- State estimation/sensor fusion
- Global planner
- Local planner
- Control method

All selectors work independently and can be combined to form a complete
experiment composition.
"""

import argparse
import sys
import yaml
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple

# Add paths for robot_lab_registry package
# The structure is: src/robot_lab/robot_lab_registry/robot_lab_registry/
import os
_base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
_robot_lab_registry_path = os.path.join(_base_path, 'src', 'robot_lab', 'robot_lab_registry')
sys.path.insert(0, _base_path)
sys.path.insert(0, _robot_lab_registry_path)

try:
    from robot_lab_registry.catalog import Registry
    from robot_lab_registry.validation import check_composition, Composition
    from robot_lab_registry.query import list_robots, list_environments, list_algorithms
except ImportError as e:
    raise ImportError(f"Cannot import robot_lab_registry from {_robot_lab_registry_path}. Base: {_base_path}. Error: {e}")


# ============================================================================
# Selector Classes
# ============================================================================

class RobotSelector:
    """Select a robot from the registry."""
    
    def __init__(self, config_dir: Optional[str] = None):
        self.registry = Registry(config_dir)
        if config_dir:
            self.registry.load(config_dir)
    
    def list_available(self, robot_class: Optional[str] = None, status: Optional[str] = None) -> List[Dict[str, Any]]:
        """List available robots, optionally filtered."""
        robots = list(self.registry.robots.get_all().values())
        
        if robot_class:
            robots = [r for r in robots if r.get('robot_class') == robot_class]
        if status:
            robots = [r for r in robots if r.get('status') == status]
        
        return robots
    
    def select(self, robot_id: str) -> Optional[Dict[str, Any]]:
        """Select a specific robot by ID."""
        return self.registry.robots.get(robot_id)
    
    def get_default(self) -> Optional[Dict[str, Any]]:
        """Get the default/reference robot (bumperbot)."""
        return self.registry.robots.get('bumperbot')


class EnvironmentSelector:
    """Select an environment from the registry."""
    
    def __init__(self, config_dir: Optional[str] = None):
        self.registry = Registry(config_dir)
        if config_dir:
            self.registry.load(config_dir)
    
    def list_available(self, dimension: Optional[str] = None, simulator: Optional[str] = None, 
                      status: Optional[str] = None) -> List[Dict[str, Any]]:
        """List available environments, optionally filtered."""
        environments = list(self.registry.environments.get_all().values())
        
        if dimension:
            environments = [e for e in environments if e.get('dimension') == dimension]
        if simulator:
            environments = [e for e in environments if e.get('simulator') == simulator]
        if status:
            environments = [e for e in environments if e.get('status') == status]
        
        return environments
    
    def select(self, env_id: str) -> Optional[Dict[str, Any]]:
        """Select a specific environment by ID."""
        return self.registry.environments.get(env_id)
    
    def get_default(self) -> Optional[Dict[str, Any]]:
        """Get the default environment (small_office)."""
        return self.registry.environments.get('small_office')


class SimulatorSelector:
    """Select a simulator."""
    
    # Supported simulators
    SIMULATORS = ['gazebo', 'ignition', 'sitl', 'pybullet', 'mujoco']
    
    @classmethod
    def list_available(cls) -> List[str]:
        """List available simulators."""
        return cls.SIMULATORS
    
    @classmethod
    def validate(cls, simulator: str) -> bool:
        """Validate a simulator choice."""
        return simulator in cls.SIMULATORS
    
    @classmethod
    def get_default(cls) -> str:
        """Get the default simulator."""
        return 'gazebo'


class ScenarioSelector:
    """Select a scenario from the registry."""
    
    def __init__(self, config_dir: Optional[str] = None):
        self.registry = Registry(config_dir)
        if config_dir:
            self.registry.load(config_dir)
    
    def list_available(self, task_type: Optional[str] = None, status: Optional[str] = None) -> List[Dict[str, Any]]:
        """List available scenarios, optionally filtered."""
        scenarios = list(self.registry.scenarios.get_all().values())
        
        if task_type:
            scenarios = [s for s in scenarios if s.get('task_type') == task_type]
        if status:
            scenarios = [s for s in scenarios if s.get('status') == status]
        
        return scenarios
    
    def select(self, scenario_id: str) -> Optional[Dict[str, Any]]:
        """Select a specific scenario by ID."""
        return self.registry.scenarios.get(scenario_id)
    
    def get_default(self) -> Optional[Dict[str, Any]]:
        """Get the default scenario."""
        return self.registry.scenarios.get('point_to_point_navigation')


class AlgorithmSelector:
    """Base class for algorithm selectors."""
    
    CATEGORY = None  # Override in subclasses
    
    def __init__(self, config_dir: Optional[str] = None):
        self.registry = Registry(config_dir)
        if config_dir:
            self.registry.load(config_dir)
    
    def list_available(self, family: Optional[str] = None, robot_class: Optional[str] = None,
                      status: Optional[str] = None) -> List[Dict[str, Any]]:
        """List available algorithms in this category, optionally filtered."""
        algorithms = list(self.registry.algorithms.get_all().values())
        
        # Filter by category
        if self.CATEGORY:
            algorithms = [a for a in algorithms if a.get('category') == self.CATEGORY]
        
        if family:
            algorithms = [a for a in algorithms if a.get('family') == family]
        if robot_class:
            algorithms = [a for a in algorithms if robot_class in a.get('supported_robot_classes', [])]
        if status:
            algorithms = [a for a in algorithms if a.get('status') == status]
        
        return algorithms
    
    def select(self, algo_id: str) -> Optional[Dict[str, Any]]:
        """Select a specific algorithm by ID."""
        return self.registry.algorithms.get(algo_id)
    
    def get_default(self) -> Optional[Dict[str, Any]]:
        """Get the default algorithm for this category."""
        algorithms = self.list_available(status='integrated')
        return algorithms[0] if algorithms else None


class PerceptionSelector(AlgorithmSelector):
    """Select a perception pipeline."""
    CATEGORY = 'perception'


class LocalizationSelector(AlgorithmSelector):
    """Select a localization method."""
    CATEGORY = 'localization'


class StateEstimationSelector(AlgorithmSelector):
    """Select a state estimation method."""
    CATEGORY = 'state_estimation'


class SensorFusionSelector(AlgorithmSelector):
    """Select a sensor fusion method."""
    CATEGORY = 'sensor_fusion'


class GlobalPlanningSelector(AlgorithmSelector):
    """Select a global planner."""
    CATEGORY = 'global_planning'


class LocalPlanningSelector(AlgorithmSelector):
    """Select a local planner."""
    CATEGORY = 'local_planning'


class ControlSelector(AlgorithmSelector):
    """Select a control method."""
    CATEGORY = 'control'


# ============================================================================
# Composition
# ============================================================================

class CompositionBuilder:
    """
    Build and validate experiment compositions from selector choices.
    
    This class combines independent selector choices into a complete
    experiment composition and validates it before launching.
    """
    
    def __init__(self, config_dir: Optional[str] = None):
        self.registry = Registry(config_dir)
        if config_dir:
            self.registry.load(config_dir)
        
        self.robot_id: Optional[str] = None
        self.environment_id: Optional[str] = None
        self.simulator: Optional[str] = None
        self.scenario_id: Optional[str] = None
        self.algorithm_ids: Dict[str, str] = {}
        
        # Namespace and frame prefix support (P2.5)
        # Namespace for this robot (defaults to robot_id if not set)
        self.namespace: Optional[str] = None
        # Frame prefix for TF frames (defaults to namespace + '/' if not set)
        self.frame_prefix: Optional[str] = None
        # Whether to use automatic namespace generation
        self.auto_namespace: bool = True
        # Enable/disable namespace isolation
        self.enable_namespace: bool = True
    
    def set_robot(self, robot_id: str) -> 'CompositionBuilder':
        """Set the robot."""
        self.robot_id = robot_id
        return self
    
    def set_environment(self, env_id: str) -> 'CompositionBuilder':
        """Set the environment."""
        self.environment_id = env_id
        return self
    
    def set_simulator(self, simulator: str) -> 'CompositionBuilder':
        """Set the simulator."""
        self.simulator = simulator
        return self
    
    def set_scenario(self, scenario_id: str) -> 'CompositionBuilder':
        """Set the scenario."""
        self.scenario_id = scenario_id
        return self
    
    def set_algorithm(self, category: str, algo_id: str) -> 'CompositionBuilder':
        """Set an algorithm for a specific category."""
        self.algorithm_ids[category] = algo_id
        return self
    
    def set_perception(self, algo_id: str) -> 'CompositionBuilder':
        """Set the perception algorithm."""
        return self.set_algorithm('perception', algo_id)
    
    def set_localization(self, algo_id: str) -> 'CompositionBuilder':
        """Set the localization algorithm."""
        return self.set_algorithm('localization', algo_id)
    
    def set_state_estimation(self, algo_id: str) -> 'CompositionBuilder':
        """Set the state estimation algorithm."""
        return self.set_algorithm('state_estimation', algo_id)
    
    def set_sensor_fusion(self, algo_id: str) -> 'CompositionBuilder':
        """Set the sensor fusion algorithm."""
        return self.set_algorithm('sensor_fusion', algo_id)
    
    def set_global_planning(self, algo_id: str) -> 'CompositionBuilder':
        """Set the global planning algorithm."""
        return self.set_algorithm('global_planning', algo_id)
    
    def set_local_planning(self, algo_id: str) -> 'CompositionBuilder':
        """Set the local planning algorithm."""
        return self.set_algorithm('local_planning', algo_id)
    
    def set_control(self, algo_id: str) -> 'CompositionBuilder':
        """Set the control algorithm."""
        return self.set_algorithm('control', algo_id)
    
    def set_namespace(self, namespace: str) -> 'CompositionBuilder':
        """Set the namespace for this composition (P2.5)."""
        self.namespace = namespace
        self.auto_namespace = False
        return self
    
    def set_frame_prefix(self, frame_prefix: str) -> 'CompositionBuilder':
        """Set the frame prefix for this composition (P2.5)."""
        self.frame_prefix = frame_prefix
        return self
    
    def set_enable_namespace(self, enable: bool) -> 'CompositionBuilder':
        """Enable or disable namespace isolation (P2.5)."""
        self.enable_namespace = enable
        return self
    
    def get_effective_namespace(self) -> str:
        """
        Get the effective namespace, generating one if needed (P2.5).
        
        Returns:
            The namespace to use, or empty string if disabled.
        """
        from .namespaces import get_default_namespace_for_robot
        
        if not self.enable_namespace:
            return ""
        
        if self.namespace:
            return self.namespace
        
        if self.auto_namespace and self.robot_id:
            return get_default_namespace_for_robot(self.robot_id)
        
        return ""
    
    def get_effective_frame_prefix(self) -> str:
        """
        Get the effective frame prefix, generating one if needed (P2.5).
        
        Returns:
            The frame prefix to use, or empty string if disabled.
        """
        from .namespaces import get_default_frame_prefix
        
        if not self.enable_namespace:
            return ""
        
        if self.frame_prefix:
            return self.frame_prefix
        
        namespace = self.get_effective_namespace()
        if namespace:
            return get_default_frame_prefix(self.robot_id or namespace)
        
        return ""
    
    def build(self) -> Composition:
        """Build the composition object."""
        # Build base composition from registry
        composition = Composition(
            robot_id=self.robot_id or '',
            environment_id=self.environment_id or '',
            simulator=self.simulator or '',
            scenario_id=self.scenario_id,
            algorithm_ids=self.algorithm_ids
        )
        
        # Add namespace and frame_prefix as attributes (P2.5)
        # These are stored separately since Composition class is from robot_lab_registry
        composition.namespace = self.get_effective_namespace()
        composition.frame_prefix = self.get_effective_frame_prefix()
        composition.enable_namespace = self.enable_namespace
        
        return composition
    
    def validate(self) -> Tuple[bool, List[str]]:
        """
        Validate the current composition.
        
        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        composition = self.build()
        result = check_composition(self.registry, composition)
        
        return result.valid, result.errors + result.warnings
    
    def get_launch_arguments(self) -> Dict[str, Any]:
        """
        Get launch arguments for the composition.
        
        Returns:
            Dictionary of launch arguments
        """
        # This will be expanded in P2.3 to resolve into launch fragments
        return {
            'robot_id': self.robot_id,
            'environment_id': self.environment_id,
            'simulator': self.simulator,
            'scenario_id': self.scenario_id,
            **self.algorithm_ids
        }
    
    def resolve_fragments(self) -> Tuple[bool, Dict[str, Any]]:
        """
        Resolve the composition into launch fragments (P2.3).
        
        Returns:
            Tuple of (success, resolution_result)
        """
        from .launch_fragments import CompositionResolver
        
        resolver = CompositionResolver(self.registry.config_dir)
        return resolver.resolve(self)
    
    def generate_launch(self) -> Dict[str, Any]:
        """
        Generate a launch description from the composition (P2.3).
        
        Returns:
            Dictionary with launch configuration
        """
        from .launch_fragments import CompositionResolver
        
        resolver = CompositionResolver(self.registry.config_dir)
        return resolver.generate_launch_description(self)


# ============================================================================
# CLI Interface
# ============================================================================

def create_parser() -> argparse.ArgumentParser:
    """Create the main CLI parser."""
    parser = argparse.ArgumentParser(
        prog='robot-lab-select',
        description='Robot Lab Selector CLI - Select and validate experiment components'
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # List command
    list_parser = subparsers.add_parser('list', help='List available options')
    list_parser.add_argument('entity_type', nargs='?', default='robots',
                              choices=['robots', 'environments', 'simulators', 
                                       'scenarios', 'perception', 'localization', 
                                       'state_estimation', 'sensor_fusion', 
                                       'global_planning', 'local_planning', 'control'],
                              help='Type of entities to list')
    list_parser.add_argument('-c', '--config-dir', default=None,
                              help='Directory containing catalog files')
    
    # Describe command
    describe_parser = subparsers.add_parser('describe', help='Describe a component')
    describe_parser.add_argument('entity_type',
                                  choices=['robot', 'environment', 'scenario', 
                                           'perception', 'localization', 'state_estimation',
                                           'sensor_fusion', 'global_planning', 'local_planning', 'control'],
                                  help='Type of entity')
    describe_parser.add_argument('entity_id', help='ID of the entity')
    describe_parser.add_argument('-c', '--config-dir', default=None,
                                  help='Directory containing catalog files')
    
    # Validate command
    validate_parser = subparsers.add_parser('validate', help='Validate a composition')
    validate_parser.add_argument('--robot', default=None, help='Robot ID')
    validate_parser.add_argument('--environment', default=None, help='Environment ID')
    validate_parser.add_argument('--simulator', default=None, help='Simulator')
    validate_parser.add_argument('--scenario', default=None, help='Scenario ID')
    validate_parser.add_argument('--perception', default=None, help='Perception algorithm ID')
    validate_parser.add_argument('--localization', default=None, help='Localization algorithm ID')
    validate_parser.add_argument('--state-estimation', default=None, help='State estimation algorithm ID')
    validate_parser.add_argument('--sensor-fusion', default=None, help='Sensor fusion algorithm ID')
    validate_parser.add_argument('--global-planning', default=None, help='Global planning algorithm ID')
    validate_parser.add_argument('--local-planning', default=None, help='Local planning algorithm ID')
    validate_parser.add_argument('--control', default=None, help='Control algorithm ID')
    validate_parser.add_argument('-c', '--config-dir', default=None,
                                  help='Directory containing catalog files')
    
    return parser


def main(argv: Optional[List[str]] = None) -> int:
    """
    Main entry point for the selector CLI.
    
    Args:
        argv: Command-line arguments (defaults to sys.argv)
        
    Returns:
        Exit code (0 for success, non-zero for error)
    """
    if argv is None:
        argv = sys.argv[1:]
    
    parser = create_parser()
    args = parser.parse_args(argv)
    
    if not args.command:
        parser.print_help()
        return 1
    
    config_dir = args.config_dir if hasattr(args, 'config_dir') else None
    
    # Handle list command
    if args.command == 'list':
        entity_type = args.entity_type
        
        if entity_type == 'robots':
            selector = RobotSelector(config_dir)
        elif entity_type == 'environments':
            selector = EnvironmentSelector(config_dir)
        elif entity_type == 'simulators':
            items = SimulatorSelector.list_available()
            print("Available simulators:")
            for item in items:
                print(f"  - {item}")
            return 0
        elif entity_type == 'scenarios':
            selector = ScenarioSelector(config_dir)
        elif entity_type == 'perception':
            selector = PerceptionSelector(config_dir)
        elif entity_type == 'localization':
            selector = LocalizationSelector(config_dir)
        elif entity_type == 'state_estimation':
            selector = StateEstimationSelector(config_dir)
        elif entity_type == 'sensor_fusion':
            selector = SensorFusionSelector(config_dir)
        elif entity_type == 'global_planning':
            selector = GlobalPlanningSelector(config_dir)
        elif entity_type == 'local_planning':
            selector = LocalPlanningSelector(config_dir)
        elif entity_type == 'control':
            selector = ControlSelector(config_dir)
        else:
            print(f"Unknown entity type: {entity_type}")
            return 1
        
        if entity_type != 'simulators':
            items = selector.list_available()
            print(f"Available {entity_type}:")
            for item in items:
                print(f"  - {item.get('id')}: {item.get('name')}")
        
        return 0
    
    # Handle describe command
    elif args.command == 'describe':
        entity_type = args.entity_type
        entity_id = args.entity_id
        
        if entity_type == 'robot':
            selector = RobotSelector(config_dir)
        elif entity_type == 'environment':
            selector = EnvironmentSelector(config_dir)
        elif entity_type == 'scenario':
            selector = ScenarioSelector(config_dir)
        elif entity_type == 'perception':
            selector = PerceptionSelector(config_dir)
        elif entity_type == 'localization':
            selector = LocalizationSelector(config_dir)
        elif entity_type == 'state_estimation':
            selector = StateEstimationSelector(config_dir)
        elif entity_type == 'sensor_fusion':
            selector = SensorFusionSelector(config_dir)
        elif entity_type == 'global_planning':
            selector = GlobalPlanningSelector(config_dir)
        elif entity_type == 'local_planning':
            selector = LocalPlanningSelector(config_dir)
        elif entity_type == 'control':
            selector = ControlSelector(config_dir)
        else:
            print(f"Unknown entity type: {entity_type}")
            return 1
        
        entity = selector.select(entity_id)
        if entity:
            print(yaml.dump(entity, default_flow_style=False, sort_keys=False))
        else:
            print(f"Entity '{entity_id}' of type '{entity_type}' not found")
            return 1
        
        return 0
    
    # Handle validate command
    elif args.command == 'validate':
        builder = CompositionBuilder(config_dir)
        
        # Set values from arguments
        if args.robot:
            builder.set_robot(args.robot)
        if args.environment:
            builder.set_environment(args.environment)
        if args.simulator:
            builder.set_simulator(args.simulator)
        if args.scenario:
            builder.set_scenario(args.scenario)
        if args.perception:
            builder.set_perception(args.perception)
        if args.localization:
            builder.set_localization(args.localization)
        if args.state_estimation:
            builder.set_state_estimation(args.state_estimation)
        if args.sensor_fusion:
            builder.set_sensor_fusion(args.sensor_fusion)
        if args.global_planning:
            builder.set_global_planning(args.global_planning)
        if args.local_planning:
            builder.set_local_planning(args.local_planning)
        if args.control:
            builder.set_control(args.control)
        
        # Validate
        is_valid, errors = builder.validate()
        
        if is_valid:
            print("Composition is valid")
            
            # Show composition details
            comp = builder.build()
            print(f"\nRobot: {comp.robot_id}")
            print(f"Environment: {comp.environment_id}")
            print(f"Simulator: {comp.simulator}")
            if comp.scenario_id:
                print(f"Scenario: {comp.scenario_id}")
            print("\nAlgorithms:")
            for category, algo_id in comp.algorithm_ids.items():
                print(f"  {category}: {algo_id}")
            
            return 0
        else:
            print("Composition is invalid:")
            for error in errors:
                print(f"  - {error}")
            return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
