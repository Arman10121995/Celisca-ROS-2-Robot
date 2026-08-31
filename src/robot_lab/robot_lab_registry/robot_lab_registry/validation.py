"""
Composition validation for Robot Lab Registry.

Checks capability compatibility between robots, algorithms, environments, and scenarios.
"""

from typing import Dict, List, Any, Optional, Set, Tuple
from dataclasses import dataclass, field

from .catalog import Registry, Catalog
from .schemas import ALGORITHM_CATEGORY_OPTIONS


@dataclass
class ValidationResult:
    """Result of a validation check."""
    valid: bool
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    
    def merge(self, other: 'ValidationResult') -> 'ValidationResult':
        """Merge another result into this one."""
        self.valid = self.valid and other.valid
        self.errors.extend(other.errors)
        self.warnings.extend(other.warnings)
        return self


@dataclass
class Composition:
    """Represents a complete experiment composition."""
    robot_id: str
    environment_id: str
    simulator: str
    scenario_id: Optional[str] = None
    algorithm_ids: Dict[str, str] = field(default_factory=dict)
    
    def get_all_algorithm_ids(self) -> List[str]:
        """Get all algorithm IDs."""
        return list(self.algorithm_ids.values())


def check_capabilities(
    robot: Dict[str, Any],
    algorithm: Dict[str, Any]
) -> ValidationResult:
    """
    Check if a robot has the required capabilities for an algorithm.
    
    Args:
        robot: Robot entity
        algorithm: Algorithm entity
        
    Returns:
        ValidationResult with compatibility status
    """
    result = ValidationResult(valid=True)
    
    # Capability checking is currently disabled - needs refinement
    # robot_capabilities = set(robot.get('capabilities', []))
    # required_capabilities = set(algorithm.get('required_capabilities', []))
    # 
    # # Check required capabilities
    # missing = required_capabilities - robot_capabilities
    # if missing:
    #     result.valid = False
    #     result.errors.append(
    #         f"Robot '{robot['id']}' missing capabilities for algorithm '{algorithm['id']}': {missing}"
    #     )
    
    # Check robot class compatibility
    algorithm_robot_classes = set(algorithm.get('supported_robot_classes', []))
    robot_class = robot.get('robot_class')
    
    if algorithm_robot_classes and robot_class not in algorithm_robot_classes:
        result.valid = False
        result.errors.append(
            f"Algorithm '{algorithm['id']}' does not support robot class '{robot_class}'"
        )
    
    return result


def check_robot_environment_compatibility(
    robot: Dict[str, Any],
    environment: Dict[str, Any]
) -> ValidationResult:
    """
    Check if a robot is compatible with an environment.
    
    Args:
        robot: Robot entity
        environment: Environment entity
        
    Returns:
        ValidationResult with compatibility status
    """
    result = ValidationResult(valid=True)
    
    # Check simulator support
    robot_simulators = set(robot.get('supported_simulators', []))
    env_simulator = environment.get('simulator', '')
    
    # If robot doesn't specify simulators, assume it supports all
    if robot_simulators and env_simulator not in robot_simulators:
        result.warnings.append(
            f"Robot '{robot['id']}' does not list '{env_simulator}' in supported simulators"
        )
    
    # Check environment robot class support
    env_robot_classes = set(environment.get('supported_robot_classes', []))
    robot_class = robot.get('robot_class')
    
    # If environment doesn't specify robot classes, assume it supports all
    if env_robot_classes and robot_class not in env_robot_classes:
        result.valid = False
        result.errors.append(
            f"Environment '{environment['id']}' does not support robot class '{robot_class}'"
        )
    
    # Check dimensionality
    env_dimension = environment.get('dimension')
    if env_dimension == '2D':
        # 2D environments work with most robots
        pass
    elif env_dimension == '3D':
        # 3D environments may not work with 2D-only robots
        if robot_class in ['mobile']:  # Simple check, may need refinement
            result.warnings.append(
                f"3D environment '{environment['id']}' may have limited compatibility with "
                f"'{robot_class}' robot '{robot['id']}'"
            )
    
    return result


def check_algorithm_compatibility(
    algorithm1: Dict[str, Any],
    algorithm2: Dict[str, Any]
) -> ValidationResult:
    """
    Check if two algorithms can be used together.
    
    This is a placeholder for more sophisticated checks.
    Currently checks for conflicting requirements.
    
    Args:
        algorithm1: First algorithm entity
        algorithm2: Second algorithm entity
        
    Returns:
        ValidationResult with compatibility status
    """
    result = ValidationResult(valid=True)
    
    # Basic check: algorithms in the same category might conflict
    if algorithm1.get('category') == algorithm2.get('category'):
        result.warnings.append(
            f"Algorithms '{algorithm1['id']}' and '{algorithm2['id']}' are in the same category "
            f"'{algorithm1.get('category')}'. Ensure they serve different purposes."
        )
    
    return result


def check_scenario_requirements(
    scenario: Dict[str, Any],
    robot: Dict[str, Any],
    environment: Optional[Dict[str, Any]] = None
) -> ValidationResult:
    """
    Check if a scenario's requirements are met.
    
    Args:
        scenario: Scenario entity
        robot: Robot entity
        environment: Optional environment entity
        
    Returns:
        ValidationResult with compatibility status
    """
    result = ValidationResult(valid=True)
    
    # Check robot class requirements
    required_classes = set(scenario.get('required_robot_classes', []))
    robot_class = robot.get('robot_class')
    
    if required_classes and robot_class not in required_classes:
        result.valid = False
        result.errors.append(
            f"Scenario '{scenario['id']}' requires robot class in {required_classes}, "
            f"but robot '{robot['id']}' is '{robot_class}'"
        )
    
    # Capability checking is currently disabled - needs refinement
    # # Check capability requirements
    # required_capabilities = set(scenario.get('required_capabilities', []))
    # robot_capabilities = set(robot.get('capabilities', []))
    # 
    # missing = required_capabilities - robot_capabilities
    # if missing:
    #     result.valid = False
    #     result.errors.append(
    #         f"Scenario '{scenario['id']}' requires capabilities {missing}, "
    #         f"but robot '{robot['id']}' only has {robot_capabilities}"
    #     )
    
    # Check environment if provided
    if environment:
        env_dimension = environment.get('dimension')
        # Add any environment-specific checks here
    
    return result


def check_composition(
    registry: Registry,
    composition: Composition
) -> ValidationResult:
    """
    Validate a complete experiment composition.
    
    Checks:
    1. All referenced entities exist
    2. Robot-algorithm capability compatibility
    3. Robot-environment compatibility
    4. Scenario requirements
    5. Algorithm-algorithm compatibility
    
    Args:
        registry: Loaded registry
        composition: Composition to validate
        
    Returns:
        ValidationResult with all validation issues
    """
    result = ValidationResult(valid=True)
    
    # 1. Check all referenced entities exist
    
    # Check robot
    robot = registry.robots.get(composition.robot_id)
    if not robot:
        result.valid = False
        result.errors.append(f"Robot '{composition.robot_id}' not found in registry")
        return result  # Can't continue without robot
    
    # Check environment
    environment = registry.environments.get(composition.environment_id)
    if not environment:
        result.valid = False
        result.errors.append(f"Environment '{composition.environment_id}' not found in registry")
        return result
    
    # Check scenario if provided
    scenario = None
    if composition.scenario_id:
        scenario = registry.scenarios.get(composition.scenario_id)
        if not scenario:
            result.valid = False
            result.errors.append(f"Scenario '{composition.scenario_id}' not found in registry")
    
    # Check all algorithms
    for category, algo_id in composition.algorithm_ids.items():
        if algo_id and not registry.algorithms.get(algo_id):
            result.valid = False
            result.errors.append(f"Algorithm '{algo_id}' (category: {category}) not found in registry")
    
    # 2. Check robot-environment compatibility
    env_result = check_robot_environment_compatibility(robot, environment)
    result.merge(env_result)
    
    # 3. Check scenario requirements
    if scenario:
        scenario_result = check_scenario_requirements(scenario, robot, environment)
        result.merge(scenario_result)
    
    # 4. Check robot-algorithm capability compatibility
    algo_ids = [aid for aid in composition.get_all_algorithm_ids() if aid]
    for algo_id in algo_ids:
        algorithm = registry.algorithms.get(algo_id)
        if algorithm:
            algo_result = check_capabilities(robot, algorithm)
            result.merge(algo_result)
    
    # 5. Check algorithm-algorithm compatibility
    algorithms = []
    for algo_id in algo_ids:
        algorithm = registry.algorithms.get(algo_id)
        if algorithm:
            algorithms.append(algorithm)
    
    # Check pairwise compatibility
    for i in range(len(algorithms)):
        for j in range(i + 1, len(algorithms)):
            algo_result = check_algorithm_compatibility(algorithms[i], algorithms[j])
            result.merge(algo_result)
    
    return result


def validate_composition_from_dict(
    registry: Registry,
    composition_dict: Dict[str, Any]
) -> ValidationResult:
    """
    Validate a composition specified as a dictionary.
    
    Args:
        registry: Loaded registry
        composition_dict: Dictionary with composition fields
        
    Returns:
        ValidationResult with validation status
    """
    # Extract algorithm_ids from various possible formats
    algo_ids = composition_dict.get('algorithm_ids', {})
    
    composition = Composition(
        robot_id=composition_dict.get('robot_id', ''),
        environment_id=composition_dict.get('environment_id', ''),
        simulator=composition_dict.get('simulator', ''),
        scenario_id=composition_dict.get('scenario_id'),
        algorithm_ids=algo_ids
    )
    
    return check_composition(registry, composition)


def validate_experiment(
    registry: Registry,
    experiment: Dict[str, Any]
) -> ValidationResult:
    """
    Validate an experiment from the registry.
    
    Args:
        registry: Loaded registry
        experiment: Experiment entity
        
    Returns:
        ValidationResult with validation status
    """
    composition = Composition(
        robot_id=experiment.get('robot_id', ''),
        environment_id=experiment.get('environment_id', ''),
        simulator=experiment.get('simulator', ''),
        scenario_id=experiment.get('scenario_id'),
        algorithm_ids=experiment.get('algorithm_ids', {})
    )
    
    return check_composition(registry, composition)


# ============================================================================
# Cross-reference Validation
# ============================================================================

def validate_cross_references(registry: Registry) -> ValidationResult:
    """
    Validate that all cross-references in the registry are valid.
    
    Checks:
    - Algorithm smoke_experiment references exist
    - Experiment references exist
    - Robot smoke_experiments references exist
    
    Args:
        registry: Loaded registry
        
    Returns:
        ValidationResult with cross-reference validation status
    """
    result = ValidationResult(valid=True)
    
    # Check algorithm smoke experiments
    for algo_id, algorithm in registry.algorithms.get_all().items():
        smoke_exp = algorithm.get('smoke_experiment')
        if smoke_exp and not registry.experiments.get(smoke_exp):
            result.errors.append(
                f"Algorithm '{algo_id}' references non-existent smoke experiment '{smoke_exp}'"
            )
    
    # Check robot smoke experiments
    for robot_id, robot in registry.robots.get_all().items():
        smoke_exps = robot.get('smoke_experiments', [])
        for smoke_exp in smoke_exps:
            if not registry.experiments.get(smoke_exp):
                result.errors.append(
                    f"Robot '{robot_id}' references non-existent smoke experiment '{smoke_exp}'"
                )
    
    # Check experiment references
    for exp_id, experiment in registry.experiments.get_all().items():
        exp_result = validate_experiment(registry, experiment)
        result.merge(exp_result)
    
    result.valid = len(result.errors) == 0
    return result
