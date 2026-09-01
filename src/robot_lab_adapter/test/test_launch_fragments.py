#!/usr/bin/env python3
"""
Test suite for launch fragment resolution (P2.3).
"""

import sys
import os

# Add the package and dependencies to path
_base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_base_path, 'robot_lab_adapter'))
sys.path.insert(0, os.path.join(_base_path, 'robot_lab', 'robot_lab_registry'))
sys.path.insert(0, _base_path)


def test_launch_fragment_creation():
    """Test LaunchFragment dataclass creation."""
    print("Testing LaunchFragment creation...")
    
    from robot_lab_adapter.launch_fragments import LaunchFragment
    
    fragment = LaunchFragment(
        id='test_fragment',
        package='test_package',
        executable='test_node',
        default_params={'param1': 'value1'},
        provided_topics=['/topic1', '/topic2'],
        required_topics=['/required_topic'],
        supported_robots=['bumperbot'],
        category='perception'
    )
    
    assert fragment.id == 'test_fragment'
    assert fragment.package == 'test_package'
    assert fragment.executable == 'test_node'
    assert fragment.default_params['param1'] == 'value1'
    assert '/topic1' in fragment.provided_topics
    assert '/required_topic' in fragment.required_topics
    assert 'bumperbot' in fragment.supported_robots
    
    print("  - LaunchFragment creation: OK")
    return True


def test_parameter_overlay():
    """Test ParameterOverlay dataclass."""
    print("\nTesting ParameterOverlay...")
    
    from robot_lab_adapter.launch_fragments import ParameterOverlay
    
    overlay = ParameterOverlay(
        name='test_overlay',
        parameters={'param1': 'value1', 'param2': 'value2'},
        description='Test overlay'
    )
    
    base = {'param1': 'old', 'param3': 'value3'}
    result = overlay.apply(base)
    
    assert result['param1'] == 'value1'  # Overridden
    assert result['param2'] == 'value2'  # Added
    assert result['param3'] == 'value3'  # Preserved
    
    print("  - ParameterOverlay: OK")
    return True


def test_fragment_registry():
    """Test LaunchFragmentRegistry."""
    print("\nTesting LaunchFragmentRegistry...")
    
    from robot_lab_adapter.launch_fragments import (
        LaunchFragmentRegistry,
        LaunchFragment,
        ParameterOverlay,
        CompositionResolver
    )
    
    config_dir = os.path.join(_base_path, 'robot_lab', 'robot_lab_registry', 'config')
    
    # Create a CompositionResolver to ensure fragments are initialized
    resolver = CompositionResolver(config_dir)
    registry = resolver.fragment_registry
    
    # Test that default mappings are initialized
    robot_launch = registry.get_launch_for_robot('bumperbot')
    assert robot_launch == 'bumperbot_simulated'
    print(f"  - Robot launch mapping: {robot_launch}")
    
    # Test algorithm mappings
    algo_launch = registry.get_launch_for_algorithm('amcl')
    assert algo_launch == 'amcl_node'
    print(f"  - Algorithm launch mapping: {algo_launch}")
    
    # Test fragment retrieval
    fragment = registry.get_fragment('bumperbot_simulated')
    assert fragment is not None
    assert fragment.package == 'robot_lab_bringup'
    print(f"  - Fragment retrieval: {fragment.id}")
    
    # Test overlay retrieval
    overlay = registry._overlays.get('bumperbot_defaults')
    assert overlay is not None
    print(f"  - Overlay retrieval: {overlay.name}")
    
    print("  - LaunchFragmentRegistry: OK")
    return True


def test_composition_resolver():
    """Test CompositionResolver."""
    print("\nTesting CompositionResolver...")
    
    from robot_lab_adapter.selectors import CompositionBuilder
    from robot_lab_adapter.launch_fragments import CompositionResolver
    
    config_dir = os.path.join(_base_path, 'robot_lab', 'robot_lab_registry', 'config')
    
    # Create a composition
    builder = CompositionBuilder(config_dir)
    builder.set_robot('bumperbot')
    builder.set_environment('small_office')
    builder.set_simulator('gazebo')
    builder.set_perception('laser_scan_to_pointcloud')
    builder.set_localization('amcl')
    builder.set_control('simple_controller')
    
    # Create resolver and resolve
    resolver = CompositionResolver(config_dir)
    success, result = resolver.resolve(builder)
    
    assert success, f"Resolution failed: {result.get('errors', [])}"
    assert 'bumperbot_simulated' in result['fragments']
    assert 'laser_scan_to_pointcloud_node' in result['fragments']
    assert 'amcl_node' in result['fragments']
    assert 'simple_controller_node' in result['fragments']
    
    print(f"  - Resolved {len(result['fragments'])} fragments")
    print(f"  - Generated {len(result['parameters'])} parameters")
    print(f"  - Warnings: {len(result['warnings'])}")
    
    # Test with robot that has no launch fragment defined
    # The resolver will fall back to bumperbot_simulated, but that fragment
    # only supports 'bumperbot', so it should generate an error
    builder2 = CompositionBuilder(config_dir)
    builder2.set_robot('a1')  # No launch fragment defined for a1
    builder2.set_environment('small_office')
    
    success2, result2 = resolver.resolve(builder2)
    # Should fail because bumperbot_simulated doesn't support a1
    assert not success2
    assert len(result2['errors']) > 0
    assert 'does not support robot "a1"' in result2['errors'][0]
    print("  - Invalid robot handling: OK (rejected)")
    
    print("  - CompositionResolver: OK")
    return True


def test_topic_compatibility():
    """Test topic compatibility checking."""
    print("\nTesting topic compatibility...")
    
    from robot_lab_adapter.selectors import CompositionBuilder
    from robot_lab_adapter.launch_fragments import CompositionResolver
    
    config_dir = os.path.join(_base_path, 'robot_lab', 'robot_lab_registry', 'config')
    
    # Create a composition with algorithms that require topics
    builder = CompositionBuilder(config_dir)
    builder.set_robot('bumperbot')
    builder.set_environment('small_office')
    builder.set_perception('costmap_2d_observation')  # Requires /scan and /odom
    builder.set_localization('ekf_node')  # Requires /odom and /imu
    
    resolver = CompositionResolver(config_dir)
    success, result = resolver.resolve(builder)
    
    # Check that warnings are generated for missing topics
    # The bumperbot_simulated fragment provides /scan, /imu, /odom
    # So these should be satisfied
    assert success
    print(f"  - Topic compatibility: OK ({len(result['warnings'])} warnings)")
    
    return True


def test_combination_validation():
    """Test combination validation."""
    print("\nTesting combination validation...")
    
    from robot_lab_adapter.launch_fragments import CompositionResolver
    
    config_dir = os.path.join(_base_path, 'robot_lab', 'robot_lab_registry', 'config')
    resolver = CompositionResolver(config_dir)
    
    # Test valid combination
    is_valid, errors = resolver.validate_combination(
        'bumperbot',
        {
            'perception': 'laser_scan_to_pointcloud',
            'localization': 'amcl',
            'control': 'simple_controller'
        }
    )
    assert is_valid
    print("  - Valid combination: OK")
    
    # Test with unknown robot
    is_valid2, errors2 = resolver.validate_combination(
        'unknown_robot',
        {'perception': 'laser_scan_to_pointcloud'}
    )
    assert not is_valid2
    assert len(errors2) > 0
    print("  - Unknown robot: OK (rejected)")
    
    # Test with unknown algorithm
    is_valid3, errors3 = resolver.validate_combination(
        'bumperbot',
        {'perception': 'unknown_algo'}
    )
    assert not is_valid3
    assert len(errors3) > 0
    print("  - Unknown algorithm: OK (rejected)")
    
    print("  - Combination validation: OK")
    return True


def test_generate_launch():
    """Test launch generation."""
    print("\nTesting launch generation...")
    
    from robot_lab_adapter.selectors import CompositionBuilder
    from robot_lab_adapter.launch_fragments import CompositionResolver
    
    config_dir = os.path.join(_base_path, 'robot_lab', 'robot_lab_registry', 'config')
    
    # Create a composition
    builder = CompositionBuilder(config_dir)
    builder.set_robot('bumperbot')
    builder.set_environment('small_office')
    builder.set_simulator('gazebo')
    builder.set_perception('laser_scan_to_pointcloud')
    
    # Generate launch
    resolver = CompositionResolver(config_dir)
    launch_config = resolver.generate_launch_description(builder)
    
    assert launch_config['success']
    assert 'fragments' in launch_config
    assert 'parameters' in launch_config
    
    print(f"  - Generated launch with {len(launch_config['fragments'])} fragments")
    print(f"  - Generated launch with {len(launch_config['parameters'])} parameters")
    print("  - Launch generation: OK")
    
    return True


def test_composition_builder_resolve():
    """Test that CompositionBuilder can resolve fragments directly."""
    print("\nTesting CompositionBuilder.resolve_fragments()...")
    
    from robot_lab_adapter.selectors import CompositionBuilder
    
    config_dir = os.path.join(_base_path, 'robot_lab', 'robot_lab_registry', 'config')
    
    builder = CompositionBuilder(config_dir)
    builder.set_robot('bumperbot')
    builder.set_environment('small_office')
    builder.set_perception('laser_scan_to_pointcloud')
    builder.set_control('simple_controller')
    
    success, result = builder.resolve_fragments()
    
    assert success
    assert 'bumperbot_simulated' in result['fragments']
    assert 'laser_scan_to_pointcloud_node' in result['fragments']
    
    print("  - CompositionBuilder.resolve_fragments(): OK")
    return True


def main():
    """Run all tests."""
    print("=" * 60)
    print("Robot Lab Bringup - Launch Fragments Test Suite (P2.3)")
    print("=" * 60)
    
    tests = [
        test_launch_fragment_creation,
        test_parameter_overlay,
        test_fragment_registry,
        test_composition_resolver,
        test_topic_compatibility,
        test_combination_validation,
        test_generate_launch,
        test_composition_builder_resolve,
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
