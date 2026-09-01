#!/usr/bin/env python3
"""
Test suite for namespace and frame-prefix contracts (P2.5).

This tests the multi-robot namespace isolation functionality.
"""

import sys
import os

# Add the package to path
_base_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_base_path, 'robot_lab_adapter'))
sys.path.insert(0, _base_path)


def test_namespace_config():
    """Test NamespaceConfig class."""
    print("Testing NamespaceConfig...")
    
    from robot_lab_adapter.namespaces import NamespaceConfig
    
    # Test basic creation
    ns = NamespaceConfig(name='robot1', frame_prefix='robot1/')
    assert ns.name == 'robot1'
    assert ns.frame_prefix == 'robot1/'
    print(f"  - Basic creation: {ns.name}, frame_prefix={ns.frame_prefix}")
    
    # Test auto frame_prefix
    ns2 = NamespaceConfig(name='robot2')
    assert ns2.frame_prefix == 'robot2/'
    print(f"  - Auto frame_prefix: {ns2.frame_prefix}")
    
    # Test frame_prefix normalization
    ns3 = NamespaceConfig(name='robot3', frame_prefix='robot3')
    assert ns3.frame_prefix == 'robot3/'
    print(f"  - Frame prefix normalization: {ns3.frame_prefix}")
    
    # Test empty name raises error
    try:
        NamespaceConfig(name='')
        assert False, "Should have raised ValueError"
    except ValueError as e:
        print(f"  - Empty name rejected: {e}")
    
    # Test topic transformations
    ns4 = NamespaceConfig(name='myrobot')
    
    # Absolute topic
    assert ns4.get_topic('/scan') == '/myrobot/scan'
    print(f"  - Absolute topic: /scan -> {ns4.get_topic('/scan')}")
    
    # Relative topic
    assert ns4.get_topic('scan') == 'myrobot/scan'
    print(f"  - Relative topic: scan -> {ns4.get_topic('scan')}")
    
    # Empty topic
    assert ns4.get_topic('') == ''
    print(f"  - Empty topic: '' -> {ns4.get_topic('')}")
    
    # Frame transformations
    ns5 = NamespaceConfig(name='robot', frame_prefix='robot/')
    
    # Absolute frame
    assert ns5.get_frame('/base_link') == '/robot/base_link'
    print(f"  - Absolute frame: /base_link -> {ns5.get_frame('/base_link')}")
    
    # Relative frame
    assert ns5.get_frame('base_link') == 'robot/base_link'
    print(f"  - Relative frame: base_link -> {ns5.get_frame('base_link')}")
    
    print("  - NamespaceConfig: OK")
    return True


def test_namespace_manager():
    """Test NamespaceManager class."""
    print("\nTesting NamespaceManager...")
    
    from robot_lab_adapter.namespaces import NamespaceManager
    
    manager = NamespaceManager()
    
    # Test default namespaces
    assert 'robot0' in manager.list_namespaces()
    assert 'robot1' in manager.list_namespaces()
    assert 'sim' in manager.list_namespaces()
    print(f"  - Default namespaces: {manager.list_namespaces()}")
    
    # Test get namespace
    ns = manager.get_namespace('robot0')
    assert ns is not None
    assert ns.name == 'robot0'
    print(f"  - Get namespace 'robot0': OK")
    
    # Test create namespace
    new_ns = manager.create_namespace('custom_robot')
    assert new_ns.name == 'custom_robot'
    assert 'custom_robot' in manager.list_used_namespaces()
    print(f"  - Create namespace 'custom_robot': OK")
    
    # Test duplicate namespace raises error
    try:
        manager.create_namespace('custom_robot')
        assert False, "Should have raised ValueError"
    except ValueError:
        print(f"  - Duplicate namespace rejected: OK")
    
    # Test auto-generated namespace
    auto_ns = manager.create_namespace()
    assert auto_ns.name.startswith('robot')
    assert auto_ns.name in manager.list_used_namespaces()
    print(f"  - Auto-generated namespace: {auto_ns.name}")
    
    # Test get_or_create
    existing = manager.get_or_create_namespace('robot0')
    assert existing.name == 'robot0'
    print(f"  - Get or create existing: OK")
    
    new_existing = manager.get_or_create_namespace('new_robot')
    assert new_existing.name == 'new_robot'
    print(f"  - Get or create new: OK")
    
    # Test release namespace
    assert manager.release_namespace('custom_robot')
    assert 'custom_robot' not in manager.list_used_namespaces()
    print(f"  - Release namespace: OK")
    
    # Test validation
    valid, msg = manager.validate_namespace('valid_name_123')
    assert valid
    print(f"  - Valid namespace 'valid_name_123': OK")
    
    valid, msg = manager.validate_namespace('123invalid')
    assert not valid
    print(f"  - Invalid namespace '123invalid': rejected")
    
    valid, msg = manager.validate_namespace('')
    assert not valid
    print(f"  - Empty namespace: rejected")
    
    print("  - NamespaceManager: OK")
    return True


def test_utility_functions():
    """Test utility functions for namespaces."""
    print("\nTesting utility functions...")
    
    from robot_lab_adapter.namespaces import (
        get_default_namespace_for_robot,
        get_default_frame_prefix,
        apply_namespace_to_dict,
        NamespaceManager
    )
    
    # Test get_default_namespace_for_robot
    ns = get_default_namespace_for_robot('bumperbot')
    assert ns == 'bumperbot'
    print(f"  - get_default_namespace_for_robot('bumperbot'): {ns}")
    
    ns2 = get_default_namespace_for_robot('123robot')
    assert ns2 == 'robot_123robot'
    print(f"  - get_default_namespace_for_robot('123robot'): {ns2}")
    
    ns3 = get_default_namespace_for_robot('my-robot')
    assert ns3 == 'my_robot'
    print(f"  - get_default_namespace_for_robot('my-robot'): {ns3}")
    
    # Test get_default_frame_prefix
    prefix = get_default_frame_prefix('bumperbot')
    assert prefix == 'bumperbot/'
    print(f"  - get_default_frame_prefix('bumperbot'): {prefix}")
    
    # Test apply_namespace_to_dict
    data = {
        'topic': '/scan',
        'frame': 'base_link',
        'other_param': 'value',
        'nested': {
            'cmd_vel_topic': '/cmd_vel',
            'odom_frame': 'odom'
        }
    }
    
    result = apply_namespace_to_dict(data, 'myrobot')
    assert result['topic'] == '/myrobot/scan'
    assert result['frame'] == 'myrobot/base_link'
    assert result['other_param'] == 'value'
    assert result['nested']['cmd_vel_topic'] == '/myrobot/cmd_vel'
    assert result['nested']['odom_frame'] == 'myrobot/odom'
    print(f"  - apply_namespace_to_dict: OK")
    
    print("  - Utility functions: OK")
    return True


def test_composition_namespace():
    """Test namespace support in CompositionBuilder."""
    print("\nTesting CompositionBuilder namespace support...")
    
    from robot_lab_adapter.selectors import CompositionBuilder
    
    # Create a composition with namespace
    comp = CompositionBuilder()
    comp.set_robot('bumperbot')
    comp.set_environment('small_office')
    comp.set_namespace('robot1')
    comp.set_frame_prefix('robot1/')
    comp.set_localization('amcl')
    
    # Test effective namespace
    effective_ns = comp.get_effective_namespace()
    assert effective_ns == 'robot1'
    print(f"  - Effective namespace: {effective_ns}")
    
    # Test effective frame prefix
    effective_prefix = comp.get_effective_frame_prefix()
    assert effective_prefix == 'robot1/'
    print(f"  - Effective frame prefix: {effective_prefix}")
    
    # Test auto namespace
    comp2 = CompositionBuilder()
    comp2.set_robot('bumperbot')
    comp2.set_environment('small_office')
    
    auto_ns = comp2.get_effective_namespace()
    assert auto_ns == 'bumperbot'
    print(f"  - Auto namespace from robot_id: {auto_ns}")
    
    auto_prefix = comp2.get_effective_frame_prefix()
    assert auto_prefix == 'bumperbot/'
    print(f"  - Auto frame prefix from robot_id: {auto_prefix}")
    
    # Test disable namespace
    comp3 = CompositionBuilder()
    comp3.set_robot('bumperbot')
    comp3.set_enable_namespace(False)
    
    disabled_ns = comp3.get_effective_namespace()
    assert disabled_ns == ''
    print(f"  - Disabled namespace: {disabled_ns}")
    
    print("  - CompositionBuilder namespace support: OK")
    return True


def test_launch_fragment_namespace():
    """Test namespace support in LaunchFragment."""
    print("\nTesting LaunchFragment namespace support...")
    
    from robot_lab_adapter.launch_fragments import LaunchFragment
    from robot_lab_adapter.namespaces import get_default_namespace_for_robot
    
    # Create a fragment with namespace support
    fragment = LaunchFragment(
        id='test_fragment',
        package='test_package',
        executable='test_node',
        default_params={
            'topic': '/scan',
            'frame': 'base_link',
            'other': 'value'
        },
        provided_topics=['/scan', '/odom'],
        required_topics=['/cmd_vel']
    )
    
    # Test namespaced params
    robot_ns = get_default_namespace_for_robot('bumperbot')
    robot_prefix = 'bumperbot/'
    
    namespaced = fragment.get_namespaced_params(robot_ns, robot_prefix)
    
    assert namespaced['namespace'] == robot_ns
    assert namespaced['frame_prefix'] == robot_prefix.rstrip('/')
    print(f"  - Namespaced params namespace: {namespaced['namespace']}")
    print(f"  - Namespaced params frame_prefix: {namespaced['frame_prefix']}")
    
    # Test namespaced topics
    namespaced_topics = fragment.get_namespaced_topics(robot_ns)
    assert '/bumperbot/scan' in namespaced_topics
    assert '/bumperbot/odom' in namespaced_topics
    print(f"  - Namespaced topics: {namespaced_topics}")
    
    # Test namespaced required topics
    namespaced_req = fragment.get_namespaced_required_topics(robot_ns)
    assert '/bumperbot/cmd_vel' in namespaced_req
    print(f"  - Namespaced required topics: {namespaced_req}")
    
    # Test fragment with custom namespace
    fragment2 = LaunchFragment(
        id='test_fragment2',
        package='test_package',
        executable='test_node',
        namespace='custom_ns',
        frame_prefix='custom/'
    )
    
    namespaced2 = fragment2.get_namespaced_params('robot_ns', 'robot/')
    assert namespaced2['namespace'] == 'custom_ns'
    assert namespaced2['frame_prefix'] == 'custom'  # Stripped of trailing slash
    print(f"  - Custom namespace override: OK")
    
    print("  - LaunchFragment namespace support: OK")
    return True


def test_resolver_namespace():
    """Test namespace support in CompositionResolver."""
    print("\nTesting CompositionResolver namespace support...")
    
    from robot_lab_adapter.selectors import CompositionBuilder
    from robot_lab_adapter.launch_fragments import CompositionResolver
    
    # Create a composition with namespace
    comp = CompositionBuilder()
    comp.set_robot('bumperbot')
    comp.set_environment('small_office')
    comp.set_namespace('test_robot')
    comp.set_localization('amcl')
    
    resolver = CompositionResolver()
    success, result = resolver.resolve(comp)
    
    assert success
    assert result['namespace'] == 'test_robot'
    assert 'namespace' in result['parameters']
    assert result['parameters']['namespace'] == 'test_robot'
    print(f"  - Resolver namespace: {result['namespace']}")
    print(f"  - Resolver namespace param: {result['parameters'].get('namespace')}")
    
    # Check fragment configs
    assert 'fragment_configs' in result
    print(f"  - Fragment configs present: OK")
    
    # Test with disabled namespace
    comp2 = CompositionBuilder()
    comp2.set_robot('bumperbot')
    comp2.set_enable_namespace(False)
    
    success2, result2 = resolver.resolve(comp2)
    assert success2
    assert result2['namespace'] == ''
    print(f"  - Disabled namespace in resolver: OK")
    
    print("  - CompositionResolver namespace support: OK")
    return True


def test_multi_robot_scenario():
    """Test a multi-robot scenario with namespaces."""
    print("\nTesting multi-robot scenario...")
    
    from robot_lab_adapter.selectors import CompositionBuilder
    from robot_lab_adapter.launch_fragments import CompositionResolver
    from robot_lab_adapter.namespaces import NamespaceManager
    
    manager = NamespaceManager()
    
    # Create namespaces for two robots
    ns1 = manager.create_namespace('robot1')
    ns2 = manager.create_namespace('robot2')
    
    # Build compositions for each robot
    comp1 = CompositionBuilder()
    comp1.set_robot('bumperbot')
    comp1.set_namespace(ns1.name)
    comp1.set_environment('small_office')
    comp1.set_localization('amcl')
    
    comp2 = CompositionBuilder()
    comp2.set_robot('bumperbot')
    comp2.set_namespace(ns2.name)
    comp2.set_environment('small_office')
    comp2.set_localization('amcl')
    
    resolver = CompositionResolver()
    
    # Resolve both
    success1, result1 = resolver.resolve(comp1)
    success2, result2 = resolver.resolve(comp2)
    
    assert success1 and success2
    
    # Check that namespaces are different
    assert result1['namespace'] != result2['namespace']
    print(f"  - Robot1 namespace: {result1['namespace']}")
    print(f"  - Robot2 namespace: {result2['namespace']}")
    
    # Check that topics would be namespaced differently
    ns_config1 = manager.get_namespace(ns1.name)
    ns_config2 = manager.get_namespace(ns2.name)
    
    topic1 = ns_config1.get_topic('/scan')
    topic2 = ns_config2.get_topic('/scan')
    
    assert topic1 != topic2
    print(f"  - Robot1 /scan: {topic1}")
    print(f"  - Robot2 /scan: {topic2}")
    
    print("  - Multi-robot scenario: OK")
    return True


def main():
    """Run all namespace tests."""
    print("=" * 60)
    print("Robot Lab Bringup - Namespace Test Suite (P2.5)")
    print("=" * 60)
    
    tests = [
        test_namespace_config,
        test_namespace_manager,
        test_utility_functions,
        test_composition_namespace,
        test_launch_fragment_namespace,
        test_resolver_namespace,
        test_multi_robot_scenario,
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
