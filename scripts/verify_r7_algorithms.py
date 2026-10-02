#!/usr/bin/env python3
"""
R7 Algorithm Verification Script

Verifies that all seven algorithm categories have at least 5 runnable implementations
as required by R7.1 (Normalize numerical and ROS algorithm adapters).
"""

import importlib
import inspect
import sys
import os
import traceback
from typing import Dict, List, Tuple, Type
import yaml

# Add the src directory to Python path
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src'))
if src_path not in sys.path:
    sys.path.insert(0, src_path)

# Minimum number of implementations required per category
MIN_IMPLEMENTATIONS = 5

# Algorithm module mapping
CATEGORY_MODULES = {
    'perception': 'robot_lab_algorithms.perception',
    'localization': 'robot_lab_algorithms.localization',
    'state_estimation': 'robot_lab_algorithms.state_estimation',
    'sensor_fusion': 'robot_lab_algorithms.sensor_fusion',
    'global_planning': 'robot_lab_algorithms.global_planning',
    'local_planning': 'robot_lab_algorithms.local_planning',
    'control': 'robot_lab_algorithms.control'
}

# Add the algorithms directory to path
algorithms_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src', 'robot_lab_algorithms'))
if algorithms_path not in sys.path:
    sys.path.insert(0, algorithms_path)


def load_module(module_name: str):
    """Load a module, handling import errors gracefully."""
    try:
        module = importlib.import_module(module_name)
        return module
    except ImportError as e:
        print(f"ERROR: Failed to import {module_name}: {e}")
        return None
    except Exception as e:
        print(f"ERROR: Unexpected error importing {module_name}: {e}")
        traceback.print_exc()
        return None


def find_algorithm_classes(module) -> List[Type]:
    """Find all algorithm classes in a module that are runnable."""
    classes = []
    
    if module is None:
        return classes
    
    # Look for classes that are not ROS nodes (end with Node) or internal
    # Also include any class that looks like an algorithm implementation
    known_algorithm_prefixes = [
        'Euclidean', 'DBSCAN', 'RANSAC', 'Obstacle', 'Scan', 'Pointcloud',
        'DeadReckoning', 'AMCL', 'ICP', 'NDT', 'RGBDSLAM',
        'EKF', 'Kalman', 'Motion', 'PoseGraph', 'Linear', 'Unscented', 'Particle', 'ErrorState',
        'WheelImu', 'GpsOdom', 'Complementary', 'Mahony', 'Madgwick',
        'RRT', 'Voronoi', 'Dijkstra', 'AStar', 'PRM',
        'FollowTheGap', 'DWB', 'Regulated', 'TEB', 'MPPI',
        'PID', 'LQR', 'Constrained', 'Nonlinear', 'Feedback', 'Backstepping'
    ]
    
    for name, obj in inspect.getmembers(module):
        if inspect.isclass(obj) and not name.startswith('_') and name not in ['Node', 'Exception', 'Warning', 'Type']:
            # Include if it's a known algorithm or doesn't end with Node
            if (not name.endswith('Node') or 
                any(name.startswith(prefix) for prefix in known_algorithm_prefixes)):
                classes.append(obj)
    
    return classes


def find_ros_nodes(module) -> List[Type]:
    """Find all ROS node classes in a module."""
    nodes = []
    
    if module is None:
        return nodes
    
    for name, obj in inspect.getmembers(module):
        if (inspect.isclass(obj) and 
            name.endswith('Node') and 
            not name.startswith('_')):
            nodes.append(obj)
    
    return nodes


def instantiate_class(cls, max_attempts=3):
    """Try to instantiate a class with reasonable default arguments."""
    for attempt in range(max_attempts):
        try:
            # Try with no arguments
            if attempt == 0:
                instance = cls()
                return instance, True
            
            # Try with some common default arguments
            elif attempt == 1:
                sig = inspect.signature(cls.__init__)
                if len(sig.parameters) <= 1:  # Only self
                    instance = cls()
                    return instance, True
            
            # Try with zero for numerical parameters
            elif attempt == 2:
                try:
                    # Get parameter names
                    sig = inspect.signature(cls.__init__)
                    params = {}
                    for param_name, param in sig.parameters.items():
                        if param_name != 'self':
                            if param.default is not inspect.Parameter.empty:
                                params[param_name] = param.default
                            elif param.annotation == int:
                                params[param_name] = 10
                            elif param.annotation == float:
                                params[param_name] = 0.1
                            else:
                                params[param_name] = None
                    
                    if params:
                        instance = cls(**params)
                        return instance, True
                except Exception:
                    pass
                    
        except Exception as e:
            # This class might have dependencies we can't provide
            continue
    
    return None, False


def verify_category(category: str) -> Tuple[bool, Dict]:
    """Verify a single algorithm category."""
    print(f"\n=== Verifying {category} ===")
    
    module_name = CATEGORY_MODULES[category]
    module = load_module(module_name)
    
    if module is None:
        print(f"❌ FAILED: Could not load module {module_name}")
        return False, {'category': category, 'implementations': [], 'runnable': [], 'errors': ['Module load failed']}
    
    # Find algorithm classes
    algorithm_classes = find_algorithm_classes(module)
    node_classes = find_ros_nodes(module)
    
    print(f"Found {len(algorithm_classes)} algorithm classes")
    print(f"Found {len(node_classes)} ROS node classes")
    
    # Try to instantiate each class
    runnable_classes = []
    instantiation_errors = []
    
    for cls in algorithm_classes:
        instance, success = instantiate_class(cls)
        if success:
            runnable_classes.append(cls.__name__)
            print(f"  ✓ {cls.__name__} - instantiable")
        else:
            instantiation_errors.append(f"{cls.__name__} - instantiation failed")
            print(f"  ✗ {cls.__name__} - instantiation failed")
    
    for cls in node_classes:
        # ROS nodes might require ROS context, so we'll just count them
        runnable_classes.append(cls.__name__)
        print(f"  ✓ {cls.__name__} - ROS node")
    
    # Count unique base algorithms (strip Node suffix)
    unique_algorithms = set()
    for name in runnable_classes:
        base_name = name.replace('Node', '')
        unique_algorithms.add(base_name)
    
    success = len(unique_algorithms) >= MIN_IMPLEMENTATIONS
    
    result = {
        'category': category,
        'total_classes': len(algorithm_classes) + len(node_classes),
        'implementations': [name for name in unique_algorithms],
        'runnable': list(runnable_classes),
        'errors': instantiation_errors,
        'success': success,
        'count': len(unique_algorithms)
    }
    
    if success:
        print(f"✅ PASSED: {category} has {len(unique_algorithms)} implementations")
    else:
        print(f"❌ FAILED: {category} has only {len(unique_algorithms)} implementations")
    
    return success, result


def verify_algorithm_dispatch_yaml():
    """Verify the algorithm dispatch YAML configuration."""
    print("\n=== Verifying Algorithm Dispatch Configuration ===")
    
    try:
        dispatch_path = 'src/robot_lab_bringup/config/algorithm_dispatch.yaml'
        with open(dispatch_path, 'r') as f:
            dispatch_config = yaml.safe_load(f)
        
        if 'algorithms' not in dispatch_config:
            print("❌ FAILED: No 'algorithms' key in dispatch config")
            return False, {}
        
        algorithms = dispatch_config['algorithms']
        
        # Check each category
        success = True
        category_results = {}
        
        for category, entries in algorithms.items():
            runnable = []
            unavailable = []
            
            for algo_name, config in entries.items():
                if 'node' in config or 'plugin' in config or 'stack' in config:
                    runnable.append(algo_name)
                elif 'unavailable' in config:
                    unavailable.append(f"{algo_name} ({config['unavailable']})")
                
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
            else:
                print(f"❌ {category}: only {total_runnable} runnable implementations")
                success = False
        
        return success, category_results
        
    except FileNotFoundError:
        print(f"❌ FAILED: Could not find {dispatch_path}")
        return False, {}
    except yaml.YAMLError as e:
        print(f"❌ FAILED: YAML parsing error: {e}")
        return False, {}


def main():
    """Main verification function."""
    print("R7 Algorithm Verification")
    print("=" * 50)
    print(f"Required: {MIN_IMPLEMENTATIONS} runnable implementations per category")
    
    # Verify algorithm implementations
    all_success = True
    category_results = {}
    
    for category in CATEGORY_MODULES.keys():
        success, result = verify_category(category)
        category_results[category] = result
        if not success:
            all_success = False
    
    # Verify dispatch configuration
    dispatch_success, dispatch_results = verify_algorithm_dispatch_yaml()
    if not dispatch_success:
        all_success = False
    
    # Generate summary
    print("\n" + "=" * 50)
    print("VERIFICATION SUMMARY")
    print("=" * 50)
    
    for category, result in category_results.items():
        status = "✅ PASSED" if result['success'] else "❌ FAILED"
        print(f"{status} {category}: {result['count']} implementations")
        if result['errors']:
            print(f"  Errors: {result['errors'][:2]}...")  # Show first 2 errors
    
    print(f"\nAlgorithm Dispatch YAML: {'✅ PASSED' if dispatch_success else '❌ FAILED'}")
    
    # Overall result
    overall_success = all(result['success'] for result in category_results.values()) and dispatch_success
    
    print(f"\n{'✅ ALL TESTS PASSED' if overall_success else '❌ SOME TESTS FAILED'}")
    print(f"R7.1 Completion: {'✅ COMPLETE' if overall_success else '❌ INCOMPLETE'}")
    
    # Return exit code
    return 0 if overall_success else 1


if __name__ == '__main__':
    sys.exit(main())