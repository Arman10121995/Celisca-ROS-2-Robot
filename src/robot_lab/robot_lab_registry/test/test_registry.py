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
        test_query
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
