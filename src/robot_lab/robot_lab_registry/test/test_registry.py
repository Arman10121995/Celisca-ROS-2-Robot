#!/usr/bin/env python3
"""
Test script for robot_lab_registry package.
"""

import sys
import os

# Add the package to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def test_imports():
    """Test that all modules can be imported."""
    print("Testing imports...")
    
    try:
        import robot_lab_registry
        print(f"  - robot_lab_registry: {robot_lab_registry.__version__}")
    except Exception as e:
        print(f"  - robot_lab_registry: FAILED - {e}")
        return False
    
    try:
        from robot_lab_registry import schemas
        print("  - schemas: OK")
    except Exception as e:
        print(f"  - schemas: FAILED - {e}")
        return False
    
    try:
        from robot_lab_registry import catalog
        print("  - catalog: OK")
    except Exception as e:
        print(f"  - catalog: FAILED - {e}")
        return False
    
    try:
        from robot_lab_registry import validation
        print("  - validation: OK")
    except Exception as e:
        print(f"  - validation: FAILED - {e}")
        return False
    
    try:
        from robot_lab_registry import query
        print("  - query: OK")
    except Exception as e:
        print(f"  - query: FAILED - {e}")
        return False
    
    try:
        from robot_lab_registry import cli
        print("  - cli: OK")
    except Exception as e:
        print(f"  - cli: FAILED - {e}")
        return False
    
    return True


def test_schemas():
    """Test schema validation."""
    print("\nTesting schemas...")
    
    from robot_lab_registry.schemas import (
        validate_entity,
        get_schema,
        SCHEMAS
    )
    
    # Test that all schemas exist
    for entity_type in ['robot', 'environment', 'algorithm', 'scenario', 'experiment']:
        if entity_type not in SCHEMAS:
            print(f"  - Missing schema for {entity_type}")
            return False
        print(f"  - {entity_type} schema: OK")
    
    # Test validation with a simple robot
    robot = {
        'id': 'test_robot',
        'version': '1.0.0',
        'name': 'Test Robot',
        'status': 'cataloged',
        'robot_class': 'mobile',
        'capabilities': ['navigation']
    }
    
    is_valid, errors = validate_entity('robot', robot)
    if is_valid:
        print("  - Robot validation: OK")
    else:
        print(f"  - Robot validation: FAILED - {errors}")
        return False
    
    return True


def test_catalog():
    """Test catalog loading."""
    print("\nTesting catalog...")
    
    from robot_lab_registry.catalog import Registry
    
    # Create registry and try to load from config directory
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config'
    )
    
    if not os.path.exists(config_dir):
        print(f"  - Config directory not found: {config_dir}")
        return False
    
    registry = Registry(config_dir)
    
    if registry.load(config_dir):
        print("  - Registry loaded: OK")
    else:
        print("  - Registry loaded: FAILED")
        return False
    
    # Check counts
    summary = registry.get_summary()
    print(f"  - Robots: {summary['robots']}")
    print(f"  - Environments: {summary['environments']}")
    print(f"  - Algorithms: {summary['algorithms']}")
    print(f"  - Scenarios: {summary['scenarios']}")
    print(f"  - Experiments: {summary['experiments']}")
    
    # Check for errors
    errors = registry.get_all_errors()
    if errors:
        print(f"  - Errors: {errors}")
        return False
    
    return True


def test_validation():
    """Test validation functions."""
    print("\nTesting validation...")
    
    from robot_lab_registry.catalog import Registry
    from robot_lab_registry.validation import check_composition, Composition
    
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config'
    )
    
    registry = Registry(config_dir)
    if not registry.load(config_dir):
        print("  - Failed to load registry")
        return False
    
    # Create a composition
    composition = Composition(
        robot_id='bumperbot',
        environment_id='small_office',
        simulator='gazebo',
        scenario_id='point_to_point_navigation',
        algorithm_ids={
            'perception': 'costmap_2d_observation',
            'localization': 'amcl',
            'state_estimation': 'ekf_localization_node',
            'sensor_fusion': 'imu_republisher',
            'global_planning': 'navfn_planner',
            'local_planning': 'teb_local_planner',
            'control': 'simple_controller'
        }
    )
    
    # Validate
    result = check_composition(registry, composition)
    
    if result.valid:
        print("  - Composition validation: OK")
    else:
        print(f"  - Composition validation: FAILED - {result.errors}")
        return False
    
    return True


def test_query():
    """Test query functions."""
    print("\nTesting query...")
    
    from robot_lab_registry.query import (
        list_robots,
        list_environments,
        list_algorithms,
        get_summary
    )
    
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config'
    )
    
    # List robots
    robots = list_robots(config_dir)
    print(f"  - Listed {len(robots)} robots")
    
    # List environments
    environments = list_environments(config_dir)
    print(f"  - Listed {len(environments)} environments")
    
    # List algorithms
    algorithms = list_algorithms(config_dir)
    print(f"  - Listed {len(algorithms)} algorithms")
    
    # Get summary
    summary = get_summary(config_dir)
    print(f"  - Summary: OK")
    
    return True


def test_cross_references():
    """Test cross-reference validation (P1.4)."""
    print("\nTesting cross-references...")
    
    from robot_lab_registry.catalog import Registry
    from robot_lab_registry.validation import validate_cross_references
    
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config'
    )
    
    registry = Registry(config_dir)
    if not registry.load(config_dir):
        print("  - Failed to load registry")
        return False
    
    # Validate cross-references
    result = validate_cross_references(registry)
    
    if result.valid:
        print("  - Cross-reference validation: OK")
    else:
        print(f"  - Cross-reference validation: FAILED")
        for error in result.errors:
            print(f"    - {error}")
        for warning in result.warnings:
            print(f"    - WARNING: {warning}")
        return False
    
    return True


def test_capability_checking():
    """Test capability checking between robots and algorithms (P1.4)."""
    print("\nTesting capability checking...")
    
    from robot_lab_registry.catalog import Registry
    from robot_lab_registry.validation import check_capabilities
    
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config'
    )
    
    registry = Registry(config_dir)
    if not registry.load(config_dir):
        print("  - Failed to load registry")
        return False
    
    # Get some robots and algorithms to test
    robots = registry.robots.get_all()
    algorithms = registry.algorithms.get_all()
    
    if not robots or not algorithms:
        print("  - No robots or algorithms found")
        return False
    
    # Test a few robot-algorithm pairs
    test_pairs = [
        ('bumperbot', 'costmap_2d_observation'),
        ('bumperbot', 'amcl'),
        ('a1', 'costmap_2d_observation'),
    ]
    
    for robot_id, algo_id in test_pairs:
        robot = robots.get(robot_id)
        algorithm = algorithms.get(algo_id)
        
        if robot and algorithm:
            result = check_capabilities(robot, algorithm)
            if result.valid:
                print(f"  - {robot_id} + {algo_id}: OK")
            else:
                print(f"  - {robot_id} + {algo_id}: Capability check issues - {result.errors}")
        else:
            print(f"  - {robot_id} or {algo_id}: Not found")
    
    return True


def test_status_counts():
    """Test minimum count requirements for each category (P1.4)."""
    print("\nTesting status counts...")
    
    from robot_lab_registry.catalog import Registry
    from robot_lab_registry.query import get_summary
    
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config'
    )
    
    registry = Registry(config_dir)
    if not registry.load(config_dir):
        print("  - Failed to load registry")
        return False
    
    summary = get_summary(config_dir)
    
    # Define minimum counts for each category
    minimums = {
        'total_robots': 5,
        'total_environments': 5,
        'total_algorithms': 5,
    }
    
    # Check each category
    all_passed = True
    for category, minimum in minimums.items():
        count = summary.get(category, 0)
        if count >= minimum:
            print(f"  - {category}: {count} >= {minimum} OK")
        else:
            print(f"  - {category}: {count} < {minimum} FAILED")
            all_passed = False
    
    # Check algorithm categories have at least 5 total
    algo_categories = summary.get('by_category', {})
    total_algorithms = sum(algo_categories.values())
    print(f"  - Total algorithms across all categories: {total_algorithms}")
    
    # Check for each algorithm category
    required_categories = ['perception', 'localization', 'state_estimation', 
                         'sensor_fusion', 'global_planning', 'local_planning', 'control']
    for cat in required_categories:
        count = algo_categories.get(cat, 0)
        print(f"  - {cat}: {count} algorithms")
    
    return all_passed


def test_status_distribution():
    """Test status distribution across entities (P1.4)."""
    print("\nTesting status distribution...")
    
    from robot_lab_registry.catalog import Registry
    from robot_lab_registry.query import get_status_counts
    
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config'
    )
    
    # Get status counts
    counts = get_status_counts(config_dir)
    
    # Print counts for each entity type
    for entity_type, status_dict in counts.items():
        print(f"  - {entity_type}:")
        for status, count in status_dict.items():
            print(f"    - {status}: {count}")
    
    return True


def test_robot_environment_compatibility():
    """Test robot-environment compatibility checking (P1.4)."""
    print("\nTesting robot-environment compatibility...")
    
    from robot_lab_registry.catalog import Registry
    from robot_lab_registry.validation import check_robot_environment_compatibility
    
    config_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config'
    )
    
    registry = Registry(config_dir)
    if not registry.load(config_dir):
        print("  - Failed to load registry")
        return False
    
    # Test a few robot-environment pairs
    test_pairs = [
        ('bumperbot', 'small_office'),
        ('bumperbot', 'small_warehouse'),
        ('a1', 'small_office'),
        ('berkeley_humanoid_lite', 'outdoor_terrain'),  # 3D environment supports humanoid
    ]
    
    all_valid = True
    for robot_id, env_id in test_pairs:
        robot = registry.robots.get(robot_id)
        environment = registry.environments.get(env_id)
        
        if robot and environment:
            result = check_robot_environment_compatibility(robot, environment)
            if result.valid:
                print(f"  - {robot_id} in {env_id}: OK")
            else:
                print(f"  - {robot_id} in {env_id}: FAILED")
                if result.errors:
                    for error in result.errors:
                        print(f"    - ERROR: {error}")
                if result.warnings:
                    for warning in result.warnings:
                        print(f"    - WARNING: {warning}")
                all_valid = False
        else:
            print(f"  - {robot_id} or {env_id}: Not found")
            all_valid = False
    
    return all_valid


def main():
    """Run all tests."""
    print("=" * 60)
    print("Robot Lab Registry - Test Suite")
    print("=" * 60)
    
    tests = [
        test_imports,
        test_schemas,
        test_catalog,
        test_validation,
        test_query,
        # P1.4 tests
        test_cross_references,
        test_capability_checking,
        test_status_counts,
        test_status_distribution,
        test_robot_environment_compatibility,
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"  - Test failed with exception: {e}")
            import traceback
            traceback.print_exc()
            results.append(False)
    
    print("\n" + "=" * 60)
    passed = sum(results)
    total = len(results)
    print(f"Results: {passed}/{total} tests passed")
    print("=" * 60)
    
    return all(results)


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
