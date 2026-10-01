#!/usr/bin/env python3
"""
R7 Algorithm Instantiation Test

Tests that all algorithm classes can be instantiated and have basic functionality.
"""

import sys
import os
import importlib.util
import traceback
from typing import List, Dict, Any

# Add src to path
src_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'src'))
if src_path not in sys.path:
    sys.path.insert(0, src_path)

# Algorithm modules and their expected classes
CATEGORIES = {
    'perception': {
        'module': 'robot_lab_algorithms.robot_lab_algorithms.perception',
        'classes': [
            'ObstacleDetector', 'ScanClusterer', 'PointcloudSegmenter',
            'EuclideanClusterer', 'DBSCANClusterer', 'RANSACGroundRemoval'
        ]
    },
    'localization': {
        'module': 'robot_lab_algorithms.robot_lab_algorithms.localization', 
        'classes': [
            'DeadReckoning', 'AMCLLocalization', 'ICPLocalization', 
            'NDTLocalization', 'RGBDSLAMLocalization'
        ]
    },
    'state_estimation': {
        'module': 'robot_lab_algorithms.robot_lab_algorithms.state_estimation',
        'classes': [
            'LinearKalmanFilter', 'UnscentedKalmanFilter', 'ParticleFilter', 
            'ErrorStateEKF', 'EKF3DEstimator', 'MotionModelEstimator', 'PoseGraphEstimator'
        ]
    },
    'sensor_fusion': {
        'module': 'robot_lab_algorithms.robot_lab_algorithms.sensor_fusion',
        'classes': [
            'WheelImuFusion', 'GpsOdomFusion', 'ComplementaryImu', 
            'MahonyFilter', 'MadgwickFilter', 'WheelIMUGNSSUKF'
        ]
    },
    'global_planning': {
        'module': 'robot_lab_algorithms.robot_lab_algorithms.global_planning',
        'classes': [
            'DijkstraPlanner', 'AStarPlanner', 'PRMPlanner', 
            'RRTPlanner', 'RRTStarPlanner', 'VoronoiPlanner'
        ]
    },
    'local_planning': {
        'module': 'robot_lab_algorithms.robot_lab_algorithms.local_planning',
        'classes': [
            'FollowTheGap', 'DWBLocalPlanner', 'RegulatedPurePursuit', 
            'TEBLocalPlanner', 'MPPILocalPlanner'
        ]
    },
    'control': {
        'module': 'robot_lab_algorithms.robot_lab_algorithms.control',
        'classes': [
            'PIDController', 'LQRController', 'ConstrainedLinearMPC', 
            'NonlinearMPC', 'FeedbackLinearizationController', 'BacksteppingController'
        ]
    }
}


def load_module(module_name: str):
    """Load module with error handling."""
    try:
        module = importlib.import_module(module_name)
        return module, None
    except Exception as e:
        return None, str(e)


def instantiate_class_with_defaults(module, class_name: str):
    """Try to instantiate a class with reasonable default arguments."""
    try:
        cls = getattr(module, class_name)
        
        # Try different instantiation strategies
        strategies = [
            lambda: cls(),  # No arguments
            lambda: cls(**get_default_kwargs(cls)),  # With default kwargs
            lambda: cls(10),  # With a single numeric argument
            lambda: cls(0.1),  # With a smaller numeric argument
        ]
        
        for strategy in strategies:
            try:
                return strategy(), None
            except Exception:
                continue
        
        return None, f"Could not instantiate {class_name} with any strategy"
        
    except AttributeError:
        return None, f"Class {class_name} not found in module"
    except Exception as e:
        return None, f"Error instantiating {class_name}: {str(e)}"


def get_default_kwargs(cls):
    """Get reasonable default arguments for a class."""
    import inspect
    sig = inspect.signature(cls.__init__)
    kwargs = {}
    
    for param_name, param in sig.parameters.items():
        if param_name == 'self':
            continue
        if param.default is not inspect.Parameter.empty:
            kwargs[param_name] = param.default
        else:
            # Try to infer reasonable defaults
            if 'node_name' in param_name:
                kwargs[param_name] = 'test_node'
            elif 'initial' in param_name or 'start' in param_name:
                if 'pose' in param_name or 'state' in param_name:
                    kwargs[param_name] = [0.0, 0.0, 0.0]
                else:
                    kwargs[param_name] = 0.0
            elif param.annotation == int:
                kwargs[param_name] = 10
            elif param.annotation == float:
                kwargs[param_name] = 0.1
            else:
                kwargs[param_name] = None
    
    return kwargs


def test_category(category: str, info: Dict[str, Any]) -> Dict[str, Any]:
    """Test a single category."""
    print(f"\n=== Testing {category} ===")
    
    module_name = info['module']
    classes = info['classes']
    
    # Load module
    module, load_error = load_module(module_name)
    if module is None:
        print(f"❌ Failed to load module {module_name}: {load_error}")
        return {'category': category, 'success': False, 'error': load_error, 
                'instantiated': [], 'failed': []}
    
    print(f"✅ Loaded module {module_name}")
    
    instantiated = []
    failed = []
    
    # Test each class
    for class_name in classes:
        instance, error = instantiate_class_with_defaults(module, class_name)
        if instance is not None:
            instantiated.append(class_name)
            print(f"  ✅ {class_name}")
        else:
            failed.append(f"{class_name}: {error}")
            print(f"  ❌ {class_name}: {error}")
    
    success = len(instantiated) >= 5  # R7 requirement: at least 5
    
    print(f"\nResult: {len(instantiated)}/{len(classes)} classes instantiated")
    if success:
        print(f"✅ {category}: PASSED ({len(instantiated)} ≥ 5)")
    else:
        print(f"❌ {category}: FAILED ({len(instantiated)} < 5)")
    
    return {
        'category': category,
        'module': module_name,
        'success': success,
        'instantiated': instantiated,
        'failed': failed,
        'count': len(instantiated)
    }


def main():
    """Main test function."""
    print("R7 Algorithm Instantiation Test")
    print("=" * 50)
    print("Testing that all algorithm classes can be instantiated")
    print("Required: ≥5 instantiable classes per category")
    
    results = {}
    all_success = True
    
    # Test each category
    for category, info in CATEGORIES.items():
        result = test_category(category, info)
        results[category] = result
        if not result['success']:
            all_success = False
    
    # Print summary
    print("\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)
    
    for category, result in results.items():
        status = "✅ PASSED" if result['success'] else "❌ FAILED"
        print(f"{status} {category}: {result['count']} implementations")
        if result['failed']:
            print(f"   Failed: {result['failed'][:2]}{'...' if len(result['failed']) > 2 else ''}")
    
    # Overall result
    passed_count = sum(1 for r in results.values() if r['success'])
    total_count = len(results)
    
    print(f"\nCategories with ≥5 implementations: {passed_count}/{total_count}")
    
    if all_success:
        print("✅ ALL CATEGORIES PASSED")
        print("R7 Instantiation Test: ✅ PASSED")
        return 0
    else:
        print("❌ SOME CATEGORIES FAILED") 
        print("R7 Instantiation Test: ❌ FAILED")
        return 1


if __name__ == '__main__':
    sys.exit(main())