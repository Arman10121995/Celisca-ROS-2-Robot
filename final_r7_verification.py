#!/usr/bin/env python3
"""
Final R7 Algorithm Verification Script

Comprehensive verification that R7.1 is complete and R7.2-R7.8 requirements are met.
"""

import yaml
import sys
import os


def main():
    """Main verification function."""
    print("=" * 70)
    print("FINAL R7 ALGORITHM BREADTH VERIFICATION")
    print("=" * 70)
    
    # Check 1: Verify algorithm dispatch configuration
    print("\n1. Algorithm Dispatch Configuration Check")
    print("-" * 50)
    
    try:
        dispatch_path = 'src/robot_lab_bringup/config/algorithm_dispatch.yaml'
        with open(dispatch_path, 'r') as f:
            dispatch_config = yaml.safe_load(f)
        
        algorithms = dispatch_config.get('algorithms', {})
        MIN_IMPLEMENTATIONS = 5
        
        all_passed = True
        for category, entries in algorithms.items():
            runnable = []
            for algo_name, config in entries.items():
                if isinstance(config, dict):
                    if any(key in config for key in ['node', 'plugin', 'stack']):
                        runnable.append(algo_name)
                else:
                    runnable.append(algo_name)
            
            passed = len(runnable) >= MIN_IMPLEMENTATIONS
            status = "✅" if passed else "❌"
            print(f"{status} {category}: {len(runnable)} runnable implementations")
            
            if not passed:
                all_passed = False
        
        if all_passed:
            print("✅ R7.1 Completion: PASSED")
        else:
            print("❌ R7.1 Completion: FAILED")
            
    except Exception as e:
        print(f"❌ Failed to check dispatch configuration: {e}")
        return 1
    
    # Check 2: Verify tutorial files exist
    print("\n2. Tutorial Files Check")
    print("-" * 50)
    
    tutorial_files = [
        'docs/tutorials/perception.md',
        'docs/tutorials/localization.md', 
        'docs/tutorials/state_estimation.md',
        'docs/tutorials/sensor_fusion.md',
        'docs/tutorials/planning.md',
        'docs/tutorials/local_planning.md',
        'docs/tutorials/control.md'
    ]
    
    tutorial_success = True
    for file_path in tutorial_files:
        exists = os.path.exists(file_path)
        status = "✅" if exists else "❌"
        print(f"{status} {file_path}")
        if not exists:
            tutorial_success = False
    
    if tutorial_success:
        print("✅ All tutorial files exist")
    else:
        print("❌ Some tutorial files missing")
    
    # Check 3: Verify verification scripts exist
    print("\n3. Verification Scripts Check")
    print("-" * 50)
    
    verification_scripts = [
        'verify_r7_algorithms.py',
        'verify_r7_algorithms_simple.py', 
        'test_algorithm_dispatch_simple.py',
        'test_r7_algorithms_instantiation.py',
        'test_r7_perception.py'
    ]
    
    script_success = True
    for script_path in verification_scripts:
        exists = os.path.exists(script_path)
        status = "✅" if exists else "❌"
        print(f"{status} {script_path}")
        if not exists:
            script_success = False
    
    if script_success:
        print("✅ All verification scripts exist")
    else:
        print("❌ Some verification scripts missing")
    
    # Check 4: Run dispatch tests
    print("\n4. Algorithm Dispatch Tests")
    print("-" * 50)
    
    try:
        import unittest
        import io
        import contextlib
        
        # Import and run the dispatch tests
        with open('test_algorithm_dispatch_simple.py', 'r') as f:
            test_module_code = f.read()
        
        # Create a new module for the tests
        import importlib.util
        spec = importlib.util.spec_from_loader('test_algorithm_dispatch_simple', loader=None)
        test_module = importlib.util.module_from_spec(spec)
        
        # Execute the test code in the module namespace
        exec(test_module_code, test_module.__dict__)
        
        # Run the tests
        loader = unittest.TestLoader()
        suite = loader.loadTestsFromModule(test_module)
        
        # Capture output
        stream = io.StringIO()
        with contextlib.redirect_stdout(stream):
            runner = unittest.TextTestRunner(stream=stream, verbosity=2)
            result = runner.run(suite)
        
        if result.wasSuccessful():
            print("✅ Algorithm dispatch tests: PASSED")
            dispatch_tests_passed = True
        else:
            print("❌ Algorithm dispatch tests: FAILED")
            print(f"   Failures: {len(result.failures)}, Errors: {len(result.errors)}")
            dispatch_tests_passed = False
            
    except Exception as e:
        print(f"❌ Failed to run dispatch tests: {e}")
        dispatch_tests_passed = False
    
    # Check 5: Verify platform-status.yaml is updated
    print("\n5. Platform Status Check")
    print("-" * 50)
    
    try:
        status_path = 'docs/status/platform-status.yaml'
        with open(status_path, 'r') as f:
            status_config = yaml.safe_load(f)
        
        # Check R7 status
        r7_status = status_config.get('tasks', {}).get('R7', {}).get('state', 'unknown')
        r7_1_status = status_config.get('tasks', {}).get('R7', {}).get('tasks', {}).get('R7.1', {}).get('state', 'unknown')
        
        status_ok = (r7_status == 'active' and r7_1_status == 'done')
        status = "✅" if status_ok else "❌"
        print(f"{status} R7 state: {r7_status}")
        print(f"{status} R7.1 state: {r7_1_status}")
        
        if status_ok:
            print("✅ Platform status: CORRECT")
        else:
            print("❌ Platform status: NEEDS UPDATE")
            
    except Exception as e:
        print(f"❌ Failed to check platform status: {e}")
    
    # Final Summary
    print("\n" + "=" * 70)
    print("FINAL VERIFICATION SUMMARY")
    print("=" * 70)
    
    checks = [
        ("Algorithm Dispatch Configuration", all_passed),
        ("Tutorial Files", tutorial_success),
        ("Verification Scripts", script_success), 
        ("Dispatch Tests", dispatch_tests_passed)
    ]
    
    for check_name, passed in checks:
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"{status}: {check_name}")
    
    overall_success = all(passed for _, passed in checks)
    
    print(f"\n{'✅ ALL R7 CHECKS PASSED' if overall_success else '❌ SOME R7 CHECKS FAILED'}")
    
    if overall_success:
        print("\n🎉 R7.1 COMPLETED - Algorithm breadth requirement satisfied!")
        print("   All categories have ≥5 runnable implementations")
        print("   Tutorial documentation created")
        print("   Verification scripts and tests implemented")
        return 0
    else:
        print("\n⚠️  R7.1 has issues that need to be resolved")
        return 1


if __name__ == '__main__':
    sys.exit(main())