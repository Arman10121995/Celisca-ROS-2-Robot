"""
Composition validation for Robot Lab Registry.

Checks capability compatibility between robots, algorithms, environments, and scenarios.
"""

from typing import Dict, List, Any, Optional, Set, Tuple
from dataclasses import dataclass, field

from .catalog import Registry, Catalog
from .schemas import (
    ALGORITHM_CATEGORY_OPTIONS,
    SIMULATOR_OPTIONS,
    ENVIRONMENT_DIMENSION_OPTIONS,
)

# Topics published by the mission/task layer, the environment itself, or ROS
# infrastructure rather than a physical robot sensor. These are always
# considered available when a robot is composed into an environment.
ENVIRONMENT_PROVIDED_TOPICS = {'/map', '/goal_pose', '/clock', '/tf', '/tf_static'}

# Map a required topic substring to the physical sensor type that must exist
# on the robot for the topic to be satisfiable.
TOPIC_SENSOR_TYPE_PATTERNS = (
    ('/scan', 'lidar'),
    ('/points', 'pointcloud'),
    ('/cloud', 'pointcloud'),
    ('/camera', 'camera'),
    ('/image', 'camera'),
    ('/imu', 'imu'),
    ('/gps', 'gps'),
    ('/fix', 'gps'),
    ('/odom', 'odometry'),
)

# Sensor types that can stand in for another required type (e.g. a point
# cloud may be produced by a lidar or a depth camera).
SENSOR_TYPE_ALTERNATIVES = {
    'pointcloud': {'lidar', 'camera'},
}


def _infer_sensor_type(topic: str) -> Optional[str]:
    """Infer the physical sensor type a topic is produced by (None if not a sensor topic)."""
    for pattern, sensor_type in TOPIC_SENSOR_TYPE_PATTERNS:
        if pattern in topic:
            return sensor_type
    return None


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

    # Check required capabilities (restored in R3.2: typed capability gating).
    robot_capabilities = set(robot.get('capabilities', []))
    required_capabilities = set(algorithm.get('required_capabilities', []))
    missing = required_capabilities - robot_capabilities
    if missing:
        result.valid = False
        result.errors.append(
            f"Robot '{robot['id']}' missing capabilities {sorted(missing)} required by "
            f"algorithm '{algorithm['id']}' (robot has: {sorted(robot_capabilities)})"
        )

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


def check_simulator_compatibility(
    simulator: str,
    robot: Dict[str, Any],
    environment: Dict[str, Any],
) -> ValidationResult:
    """
    Check that a composition declares an explicit, known simulator that is
    consistent with the environment it runs in (R3.2).

    Args:
        simulator: Simulator requested by the composition
        robot: Robot entity (used for actionable diagnostics)
        environment: Environment entity the composition runs in

    Returns:
        ValidationResult with compatibility status
    """
    result = ValidationResult(valid=True)

    simulator = (simulator or '').strip()
    if not simulator:
        result.valid = False
        result.errors.append(
            f"Composition of robot '{robot['id']}' in environment "
            f"'{environment['id']}' must declare an explicit simulator "
            f"(supported: {SIMULATOR_OPTIONS})"
        )
        return result

    if simulator not in SIMULATOR_OPTIONS:
        result.valid = False
        result.errors.append(
            f"Unknown simulator '{simulator}'; supported simulators: {SIMULATOR_OPTIONS}"
        )

    # An environment declares the backend it was authored for plus every
    # other backend it has been ported to (`simulators`).  World geometry is
    # generated for all four backends from the same Gazebo source, so an
    # environment is usually runnable everywhere; an environment that lists
    # a narrower set is still enforced.
    env_simulator = (environment.get('simulator') or '').strip()
    supported = [str(s).strip() for s in (environment.get('simulators') or [])
                 if str(s).strip()]
    if env_simulator and env_simulator not in supported:
        supported.append(env_simulator)
    if supported and simulator not in supported:
        result.valid = False
        result.errors.append(
            f"Environment '{environment['id']}' supports simulator(s) "
            f"{sorted(supported)} but the composition requests '{simulator}'"
        )

    return result


def check_algorithm_category(
    category: str,
    algorithm: Dict[str, Any],
) -> ValidationResult:
    """
    Check that an algorithm is assigned to the composition slot matching its
    declared category (R3.2). E.g. AMCL (localization) in the global_planning
    slot is a typed composition error.

    Args:
        category: Composition slot the algorithm is assigned to
        algorithm: Algorithm entity

    Returns:
        ValidationResult with compatibility status
    """
    result = ValidationResult(valid=True)

    algo_category = algorithm.get('category')
    if algo_category != category:
        result.valid = False
        result.errors.append(
            f"Algorithm '{algorithm['id']}' has category '{algo_category}' but is assigned "
            f"to the '{category}' slot; move it to the '{algo_category}' slot or select an "
            f"algorithm whose category is '{category}'"
        )

    return result


def check_sensor_requirements(
    robot: Dict[str, Any],
    algorithms: List[Dict[str, Any]],
    environment: Dict[str, Any],
) -> ValidationResult:
    """
    Check that every algorithm input topic is satisfiable by the composed
    robot/environment/algorithm stack (R3.2).

    A required topic is satisfied when it is published by a robot sensor, by
    another selected algorithm, or by the environment/task layer. When a
    required topic implies a physical sensor type (e.g. '/scan' implies a
    LiDAR) the robot must carry a sensor of that type:

    - a missing LiDAR is a hard error (scan-based algorithms cannot run),
    - other sensor-type gaps are warnings (frequently just topic naming
      variants that need remapping, e.g. '/camera/image_raw' vs
      '/camera/rgb/image_raw').

    Args:
        robot: Robot entity
        algorithms: Algorithm entities selected by the composition
        environment: Environment entity

    Returns:
        ValidationResult with compatibility status
    """
    result = ValidationResult(valid=True)

    sensor_topics = {
        sensor.get('topic') for sensor in robot.get('sensors', []) if sensor.get('topic')
    }
    sensor_types = {
        sensor.get('type') for sensor in robot.get('sensors', []) if sensor.get('type')
    }
    provided_topics = set(sensor_topics) | ENVIRONMENT_PROVIDED_TOPICS
    for algorithm in algorithms:
        provided_topics.update(
            (algorithm.get('output_contract') or {}).get('provided_topics', []) or []
        )

    for algorithm in algorithms:
        required_topics = (algorithm.get('input_contract') or {}).get('required_topics', []) or []
        for topic in required_topics:
            if topic in provided_topics:
                continue

            sensor_type = _infer_sensor_type(topic)
            if sensor_type is None:
                result.warnings.append(
                    f"Algorithm '{algorithm['id']}' requires topic '{topic}' which is not "
                    f"provided by robot '{robot['id']}' sensors, other selected algorithms, "
                    f"or the environment; ensure it is published at runtime"
                )
                continue

            acceptable_types = SENSOR_TYPE_ALTERNATIVES.get(sensor_type, {sensor_type})
            if acceptable_types & sensor_types:
                continue

            message = (
                f"Robot '{robot['id']}' has no {sensor_type} sensor required by algorithm "
                f"'{algorithm['id']}' (topic '{topic}'; robot sensors: {sorted(sensor_types)})"
            )
            if sensor_type == 'lidar':
                # Scan-based algorithms cannot run without a LiDAR: hard failure.
                result.valid = False
                result.errors.append(message)
            else:
                result.warnings.append(
                    message + " (topic name may differ; verify remapping)"
                )

    return result


def check_dimension_compatibility(
    algorithm: Dict[str, Any],
    environment: Dict[str, Any],
) -> ValidationResult:
    """
    Check that an algorithm operates in the environment's dimensionality (R3.2).

    An algorithm that declares 'supported_dimensions' must list the
    environment's dimension. A 2D planner cannot gain flight capability by
    adding 'aerial' to 'supported_robot_classes': the metadata label alone
    is not sufficient without a declared dimension match.

    Args:
        algorithm: Algorithm entity
        environment: Environment entity

    Returns:
        ValidationResult with compatibility status
    """
    result = ValidationResult(valid=True)

    supported_dimensions = algorithm.get('supported_dimensions')
    env_dimension = environment.get('dimension')
    if supported_dimensions and env_dimension and env_dimension not in supported_dimensions:
        result.valid = False
        result.errors.append(
            f"Algorithm '{algorithm['id']}' supports dimensions "
            f"{sorted(supported_dimensions)} but environment '{environment['id']}' is "
            f"'{env_dimension}'; an entry in supported_robot_classes does not grant "
            f"capability in unmatched dimensions"
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
    
    # Check all algorithms (existence + typed category slot match, R3.2)
    algorithms = []
    for category, algo_id in composition.algorithm_ids.items():
        if not algo_id:
            continue
        algorithm = registry.algorithms.get(algo_id)
        if not algorithm:
            result.valid = False
            result.errors.append(f"Algorithm '{algo_id}' (category: {category}) not found in registry")
            continue
        algorithms.append(algorithm)
        result.merge(check_algorithm_category(category, algorithm))

    # 2. Check simulator typing and robot-environment compatibility
    result.merge(check_simulator_compatibility(composition.simulator, robot, environment))
    env_result = check_robot_environment_compatibility(robot, environment)
    result.merge(env_result)
    
    # 3. Check scenario requirements
    if scenario:
        scenario_result = check_scenario_requirements(scenario, robot, environment)
        result.merge(scenario_result)
    
    # 4. Check robot-algorithm capability compatibility
    for algorithm in algorithms:
        algo_result = check_capabilities(robot, algorithm)
        result.merge(algo_result)

    # 4b. Sensor and dimension contracts (R3.2)
    result.merge(check_sensor_requirements(robot, algorithms, environment))
    for algorithm in algorithms:
        result.merge(check_dimension_compatibility(algorithm, environment))

    # 5. Check algorithm-algorithm compatibility
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
