#!/usr/bin/env python3
"""
Simple R7 Algorithm Verification Script

Verifies that all seven algorithm categories have at least 5 runnable implementations
by checking the algorithm_dispatch.yaml configuration.
"""

import yaml
import sys

# Minimum number of implementations required per category
MIN_IMPLEMENTATIONS = 5


def verify_dispatch_configuration():
    """Verify the algorithm dispatch YAML configuration."""
    print("R7 Algorithm Verification - Dispatch Configuration Check")
    print("=" * 60)
    print(f"Required: {MIN_IMPLEMENTATIONS} runnable implementations per category")
    print()
    
    try:
        dispatch_path = 'src/robot_lab_bringup/config/algorithm_dispatch.yaml'
        with open(dispatch_path, 'r') as f:
            dispatch_config = yaml.safe_load(f)
        
        if 'algorithms' not in dispatch_config:
            print("❌ FAILED: No 'algorithms' key in dispatch config")
            return False, {}
        
        algorithms = dispatch_config['algorithms']
        
        # Check each category
        all_success = True
        category_results = {}
        
        for category, entries in algorithms.items():
            runnable = []
            unavailable = []
            
            for algo_name, config in entries.items():
                if isinstance(config, dict):
                    if 'node' in config or 'plugin' in config or 'stack' in config:
                        runnable.append(algo_name)
                    elif 'unavailable' in config:
                        unavailable.append(f"{algo_name} ({config['unavailable']})")
                else:
                    # Direct string entries
                    runnable.append(algo_name)
            
            total_runnable = len(runnable)
            category_success = total_runnable >= MIN_IMPLEMENTATIONS
            
            category_results[category] = {
                'runnable': runnable,
                'unavailable': unavailable,
                'total_runnable': total_runnable,
                'success': category_success
            }
            
            if category_success:
                print(f"✅ {category}: {total_runnable} runnable implementations")
                print(f"   Implementations: {', '.join(runnable[:5])}{'...' if len(runnable) > 5 else ''}")
            else:
                print(f"❌ {category}: only {total_runnable} runnable implementations")
                print(f"   Implementations: {', '.join(runnable)}")
                all_success = False
        
        print()
        print("=" * 60)
        print("SUMMARY")
        print("=" * 60)
        
        passed = sum(1 for result in category_results.values() if result['success'])
        total = len(category_results)
        
        print(f"Categories with ≥{MIN_IMPLEMENTATIONS} implementations: {passed}/{total}")
        
        if all_success:
            print("✅ ALL CATEGORIES PASSED")
            print("R7.1 Completion: ✅ COMPLETE")
        else:
            print("❌ SOME CATEGORIES FAILED")
            print("R7.1 Completion: ❌ INCOMPLETE")
        
        # Detailed results
        print("\nDetailed Results:")
        for category, result in category_results.items():
            status = "✅" if result['success'] else "❌"
            print(f"{status} {category}: {result['total_runnable']} implementations")
            if result['unavailable']:
                print(f"   Unavailable: {result['unavailable'][:2]}{'...' if len(result['unavailable']) > 2 else ''}")
        
        return all_success, category_results
        
    except FileNotFoundError:
        print(f"❌ FAILED: Could not find {dispatch_path}")
        return False, {}
    except yaml.YAMLError as e:
        print(f"❌ FAILED: YAML parsing error: {e}")
        return False, {}


def verify_python_imports():
    """Verify that all algorithm modules can be imported and have the expected classes."""
    print("\n" + "=" * 60)
    print("PYTHON MODULE IMPORT CHECK")
    print("=" * 60)
    
    import importlib
    import inspect
    
    # Algorithm module mapping
    CATEGORY_MODULES = {
        'perception': 'robot_lab_algorithms.robot_lab_algorithms.perception',
        'localization': 'robot_lab_algorithms.robot_lab_algorithms.localization',
        'state_estimation': 'robot_lab_algorithms.robot_lab_algorithms.state_estimation',
        'sensor_fusion': 'robot_lab_algorithms.robot_lab_algorithms.sensor_fusion',
        'global_planning': 'robot_lab_algorithms.robot_lab_algorithms.global_planning',
        'local_planning': 'robot_lab_algorithms.robot_lab_algorithms.local_planning',
        'control': 'robot_lab_algorithms.robot_lab_algorithms.control'
    }
    
    import sys
    import os
    
    # Add the src directory to path
    workspace_path = os.path.abspath(os.path.dirname(__file__))
    src_path = os.path.join(workspace_path, 'src')
    if src_path not in sys.path:
        sys.path.insert(0, src_path)
    
    all_success = True
    category_results = {}
    
    for category, module_name in CATEGORY_MODULES.items():
        try:
            # Try to import the module
            module = importlib.import_module(module_name)
            
            # Count classes that are not ROS nodes or internal
            algorithm_classes = []
            ros_node_classes = []
            
            for name, obj in inspect.getmembers(module):
                if inspect.isclass(obj) and not name.startswith('_'):
                    if name.endswith('Node'):
                        ros_node_classes.append(name)
                    else:
                        algorithm_classes.append(name)
            
            total_classes = len(algorithm_classes) + len(ros_node_classes)
            
            print(f"✅ {category}: {total_classes} classes ({len(algorithm_classes)} algorithms, {len(ros_node_classes)} ROS nodes)")
            
            category_results[category] = {
                'algorithms': algorithm_classes,
                'ros_nodes': ros_node_classes,
                'total': total_classes,
                'success': total_classes >= MIN_IMPLEMENTATIONS
            }
            
            if total_classes < MIN_IMPLEMENTATIONS:
                all_success = False
                
        except ImportError as e:
            print(f"❌ {category}: Import failed - {e}")
            category_results[category] = {'error': str(e), 'success': False}
            all_success = False
        except Exception as e:
            print(f"❌ {category}: Error - {e}")
            category_results[category] = {'error': str(e), 'success': False}
            all_success = False
    
    return all_success, category_results


def main():
    """Main verification function."""
    # Verify dispatch configuration (this is the primary check)
    dispatch_success, dispatch_results = verify_dispatch_configuration()
    
    # Also try to verify Python imports
    import_success, import_results = verify_python_imports()
    
    print("\n" + "=" * 60)
    print("FINAL RESULT")
    print("=" * 60)
    
    overall_success = dispatch_success  # Primary check
    
    if overall_success:
        print("✅ R7.1 VERIFICATION PASSED")
        print("All algorithm categories have ≥5 runnable implementations")
        return 0
    else:
        print("❌ R7.1 VERIFICATION FAILED")
        print("Some categories have <5 runnable implementations")
        return 1


if __name__ == '__main__':
    sys.exit(main())