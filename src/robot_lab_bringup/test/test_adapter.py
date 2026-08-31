#!/usr/bin/env python3
"""
Test suite for legacy adapter (P2.4).
"""

import sys
import os

# Add the package to path
_base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_base_path, 'robot_lab_bringup'))
sys.path.insert(0, _base_path)


def test_legacy_adapter():
    """Test the LegacyAdapter class."""
    print("Testing LegacyAdapter...")
    
    from robot_lab_bringup.adapter import LegacyAdapter
    
    adapter = LegacyAdapter()
    
    # Test robot mapping
    robot_config = adapter.get_legacy_config('bumperbot', 'small_office')
    assert robot_config['robot_model'] == 'bumperbot'
    assert robot_config['map_name'] == 'small_office'
    assert robot_config['mode'] == 'nav'  # Default
    print(f"  - Bumperbot + small_office: {robot_config}")
    
    # Test with scenario
    robot_config2 = adapter.get_legacy_config('bumperbot', 'small_warehouse', 
                                               'point_to_point_navigation')
    assert robot_config2['mode'] == 'nav'
    print(f"  - Bumperbot + small_warehouse + point_to_point_navigation: mode={robot_config2['mode']}")
    
    # Test mode from algorithms
    mode = adapter.get_mode_from_algorithms({
        'localization': 'amcl',
        'global_planning': 'dijkstra_planner',
        'local_planning': 'teb_local_planner'
    })
    assert mode == 'nav'
    print(f"  - Mode from nav algorithms: {mode}")
    
    # Test SLAM mode detection
    mode2 = adapter.get_mode_from_algorithms({
        'localization': 'rtabmap_localization'
    })
    assert mode2 == '3d_slam'
    print(f"  - Mode from SLAM algorithms: {mode2}")
    
    # Test legacy launch file path
    launch_file = adapter.get_legacy_launch_file('bumperbot')
    assert launch_file == 'bumperbot_bringup/simulated_robot.launch.py'
    print(f"  - Legacy launch file: {launch_file}")
    
    # Test unknown robot
    launch_file2 = adapter.get_legacy_launch_file('unknown_robot')
    assert launch_file2 is None
    print(f"  - Unknown robot: {launch_file2}")
    
    # Test generate_legacy_launch_arguments
    args = adapter.generate_legacy_launch_arguments(
        'bumperbot', 'small_office',
        {'localization': 'amcl', 'global_planning': 'dijkstra_planner'},
        'point_to_point_navigation'
    )
    assert args['robot_model'] == 'bumperbot'
    assert args['map_name'] == 'small_office'
    assert args['mode'] == 'nav'
    print(f"  - Generated arguments: {len(args)} args")
    
    print("  - LegacyAdapter: OK")
    return True


def test_environment_mappings():
    """Test environment to legacy mappings."""
    print("\nTesting environment mappings...")
    
    from robot_lab_bringup.adapter import LegacyAdapter
    
    adapter = LegacyAdapter()
    
    # Test all mapped environments
    environments = [
        'small_office', 'small_warehouse', 'small_house',
        'warehouse_demo', 'celisca_floor_1', 'celisca_floor_2'
    ]
    
    for env in environments:
        config = adapter.get_legacy_config('bumperbot', env)
        assert 'map_name' in config
        assert 'world_name' in config
        assert 'world_package' in config
        print(f"  - {env}: map={config['map_name']}, world={config['world_name']}")
    
    print("  - Environment mappings: OK")
    return True


def test_adapter_launch_generation():
    """Test adapter launch generation."""
    print("\nTesting adapter launch generation...")
    
    from robot_lab_bringup.adapter import create_bumperbot_adapter_launch
    
    # Create a launch description
    ld = create_bumperbot_adapter_launch(
        robot_id='bumperbot',
        environment_id='small_office',
        algo_ids={
            'localization': 'amcl',
            'global_planning': 'dijkstra_planner',
            'local_planning': 'teb_local_planner'
        },
        scenario_id='point_to_point_navigation',
        use_sim_time=True
    )
    
    # The launch description should be created
    assert ld is not None
    print("  - Launch description created: OK")
    
    # Check that it has actions (LaunchDescription stores entities)
    assert len(ld.entities) > 0
    print(f"  - Number of entities: {len(ld.entities)}")
    
    print("  - Adapter launch generation: OK")
    return True


def main():
    """Run all tests."""
    print("=" * 60)
    print("Robot Lab Bringup - Adapter Test Suite (P2.4)")
    print("=" * 60)
    
    tests = [
        test_legacy_adapter,
        test_environment_mappings,
        test_adapter_launch_generation,
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
