#!/usr/bin/env python3
"""
Test suite for robot_lab_adapter selectors.
"""

import sys
import os

# Add the package and dependencies to path
_base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_base_path, 'robot_lab_adapter'))
sys.path.insert(0, os.path.join(_base_path, 'robot_lab', 'robot_lab_registry'))
sys.path.insert(0, _base_path)


def test_imports():
    """Test that all selector modules can be imported."""
    print("Testing selector imports...")
    
    try:
        from robot_lab_adapter import selectors
        print(f"  - robot_lab_adapter.selectors: OK")
    except Exception as e:
        print(f"  - robot_lab_adapter.selectors: FAILED - {e}")
        return False
    
    return True


def test_robot_selector():
    """Test the RobotSelector class."""
    print("\nTesting RobotSelector...")
    
    from robot_lab_adapter.selectors import RobotSelector
    
    config_dir = os.path.join(
        _base_path, 'robot_lab', 'robot_lab_registry', 'config'
    )
    
    selector = RobotSelector(config_dir)
    
    # Test listing
    robots = selector.list_available()
    print(f"  - Listed {len(robots)} robots")
    
    if len(robots) < 5:
        print(f"  - Expected at least 5 robots, got {len(robots)}")
        return False
    
    # Test filtering
    mobile_robots = selector.list_available(robot_class='mobile')
    print(f"  - Found {len(mobile_robots)} mobile robots")
    
    # Test selection
    bumperbot = selector.select('bumperbot')
    if not bumperbot:
        print("  - Failed to select bumperbot")
        return False
    print(f"  - Selected bumperbot: OK")
    
    # Test default
    default_robot = selector.get_default()
    if not default_robot:
        print("  - Failed to get default robot")
        return False
    print(f"  - Default robot: {default_robot.get('id')}")
    
    return True


def test_environment_selector():
    """Test the EnvironmentSelector class."""
    print("\nTesting EnvironmentSelector...")
    
    from robot_lab_adapter.selectors import EnvironmentSelector
    
    config_dir = os.path.join(
        _base_path, 'robot_lab', 'robot_lab_registry', 'config'
    )
    
    selector = EnvironmentSelector(config_dir)
    
    # Test listing
    environments = selector.list_available()
    print(f"  - Listed {len(environments)} environments")
    
    if len(environments) < 5:
        print(f"  - Expected at least 5 environments, got {len(environments)}")
        return False
    
    # Test filtering
    dim_2d = selector.list_available(dimension='2D')
    print(f"  - Found {len(dim_2d)} 2D environments")
    
    # Test selection
    small_office = selector.select('small_office')
    if not small_office:
        print("  - Failed to select small_office")
        return False
    print(f"  - Selected small_office: OK")
    
    # Test default
    default_env = selector.get_default()
    if not default_env:
        print("  - Failed to get default environment")
        return False
    print(f"  - Default environment: {default_env.get('id')}")
    
    return True


def test_simulator_selector():
    """Test the SimulatorSelector class."""
    print("\nTesting SimulatorSelector...")
    
    from robot_lab_adapter.selectors import SimulatorSelector
    
    # Test listing
    simulators = SimulatorSelector.list_available()
    print(f"  - Available simulators: {simulators}")
    
    if 'gazebo' not in simulators:
        print("  - gazebo not in simulator list")
        return False
    
    # Test validation
    if not SimulatorSelector.validate('gazebo'):
        print("  - gazebo validation failed")
        return False
    print(f"  - gazebo validation: OK")
    
    # Test default
    default_sim = SimulatorSelector.get_default()
    print(f"  - Default simulator: {default_sim}")
    
    return True


def test_algorithm_selectors():
    """Test algorithm selector classes."""
    print("\nTesting Algorithm Selectors...")
    
    from robot_lab_adapter.selectors import (
        PerceptionSelector,
        LocalizationSelector,
        StateEstimationSelector,
        SensorFusionSelector,
        GlobalPlanningSelector,
        LocalPlanningSelector,
        ControlSelector
    )
    
    config_dir = os.path.join(
        _base_path, 'robot_lab', 'robot_lab_registry', 'config'
    )
    
    selectors = {
        'Perception': PerceptionSelector,
        'Localization': LocalizationSelector,
        'State Estimation': StateEstimationSelector,
        'Sensor Fusion': SensorFusionSelector,
        'Global Planning': GlobalPlanningSelector,
        'Local Planning': LocalPlanningSelector,
        'Control': ControlSelector,
    }
    
    for name, SelectorClass in selectors.items():
        selector = SelectorClass(config_dir)
        algorithms = selector.list_available()
        print(f"  - {name}: {len(algorithms)} algorithms")
        
        # Test default
        default_algo = selector.get_default()
        if default_algo:
            print(f"    - Default: {default_algo.get('id')}")
    
    return True


def test_composition_builder():
    """Test the CompositionBuilder class."""
    print("\nTesting CompositionBuilder...")
    
    from robot_lab_adapter.selectors import CompositionBuilder
    
    config_dir = os.path.join(
        _base_path, 'robot_lab', 'robot_lab_registry', 'config'
    )
    
    # Create a simple composition
    builder = CompositionBuilder(config_dir)
    builder.set_robot('bumperbot')
    builder.set_environment('small_office')
    builder.set_simulator('gazebo')
    builder.set_perception('costmap_2d_observation')
    builder.set_localization('amcl')
    builder.set_control('simple_controller')
    
    # Validate
    is_valid, errors = builder.validate()
    
    if is_valid:
        print("  - Basic composition validation: OK")
    else:
        print(f"  - Basic composition validation: FAILED - {errors}")
        return False
    
    # Test launch arguments
    args = builder.get_launch_arguments()
    print(f"  - Launch arguments: {list(args.keys())}")
    
    # Test with invalid composition
    builder2 = CompositionBuilder(config_dir)
    builder2.set_robot('nonexistent_robot')
    is_valid2, errors2 = builder2.validate()
    
    if not is_valid2:
        print("  - Invalid composition detected: OK")
    else:
        print("  - Invalid composition not detected")
        return False
    
    return True


def test_cli():
    """Test the CLI interface."""
    print("\nTesting CLI interface...")
    
    from robot_lab_adapter.selectors import create_parser
    
    parser = create_parser()
    
    # Test that parser was created
    print("  - Parser created: OK")
    
    return True


def main():
    """Run all tests."""
    print("=" * 60)
    print("Robot Lab Bringup - Selector Test Suite")
    print("=" * 60)
    
    tests = [
        test_imports,
        test_robot_selector,
        test_environment_selector,
        test_simulator_selector,
        test_algorithm_selectors,
        test_composition_builder,
        test_cli,
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
