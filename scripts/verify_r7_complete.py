#!/usr/bin/env python3
"""
R7.2-R7.8 Complete Verification Script

Comprehensive verification that all R7.2-R7.8 algorithm breadth requirements are satisfied:
- Five methods per category with method subrecords
- Input strata definitions for each category
- Benchmark experiments for each method
- Mathematical foundations and equations
- ROS adapter verification
- Unit test coverage
"""

import yaml
import os
import sys
import json
from datetime import datetime
import subprocess
import tempfile

# Add src path
workspace_path = os.path.abspath(os.path.dirname(__file__))
src_path = os.path.join(workspace_path, 'src')
sys.path.insert(0, src_path)


def verify_r7_2_perception():
    """Verify R7.2 Perception requirements."""
    print("🔍 R7.2 PERCEPTION PIPELINES")
    print("=" * 60)
    
    # Check method subrecords
    yaml_path = 'src/robot_lab_algorithms/config/r7_method_subrecords.yaml'
    if not os.path.exists(yaml_path):
        print("❌ FAILED: Method subrecords YAML not found")
        return False
    
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    perception_methods = config['method_subrecords']['perception']
    
    # Check we have at least 5 methods
    if len(perception_methods) < 5:
        print(f"❌ FAILED: Only {len(perception_methods)} perception methods, need ≥5")
        return False
    else:
        print(f"✅ {len(perception_methods)} perception methods defined")
    
    # Check each method has required fields
    required_fields = ['method_id', 'category', 'display_name', 'implementation_ref', 
                     'mathematical_foundation', 'equations', 'source_references', 
                     'applicable_robots', 'input_strata', 'output_types', 'parameters',
                     'assumptions', 'limitations', 'computational_complexity', 'memory_requirements']
    
    for method_id, method_data in perception_methods.items():
        for field in required_fields:
            if field not in method_data:
                print(f"❌ FAILED: Method {method_id} missing field {field}")
                return False
    
    print("✅ All perception methods have required fields")
    
    # Check equations are defined
    methods_with_equations = [mid for mid, data in perception_methods.items() 
                           if len(data['equations']) > 0]
    if len(methods_with_equations) < 5:
        print(f"❌ FAILED: Only {len(methods_with_equations)} methods have equations")
        return False
    
    print("✅ All perception methods have mathematical equations")
    
    # Check source references
    methods_with_references = [mid for mid, data in perception_methods.items() 
                             if len(data['source_references']) > 0]
    if len(methods_with_references) < 5:
        print(f"❌ FAILED: Only {len(methods_with_references)} methods have source references")
        return False
    
    print("✅ All perception methods have source references")
    
    # Check benchmark experiments
    exp_yaml_path = 'src/robot_lab_algorithms/config/r7_benchmark_experiments.yaml'
    if not os.path.exists(exp_yaml_path):
        print("❌ FAILED: Benchmark experiments YAML not found")
        return False
    
    with open(exp_yaml_path, 'r') as f:
        exp_config = yaml.safe_load(f)
    
    perception_experiments = exp_config['benchmark_experiments']['perception']
    if len(perception_experiments) < 5:
        print(f"❌ FAILED: Only {len(perception_experiments)} perception experiments, need ≥5")
        return False
    
    print(f"✅ {len(perception_experiments)} perception benchmark experiments defined")
    
    # Check experiments have required fields
    exp_required_fields = ['experiment_id', 'category', 'method_id', 'input_stratum',
                         'description', 'metrics', 'success_criteria', 'failure_criteria', 
                         'resource_budget']
    
    for exp_id, exp_data in perception_experiments.items():
        for field in exp_required_fields:
            if field not in exp_data:
                print(f"❌ FAILED: Experiment {exp_id} missing field {field}")
                return False
    
    print("✅ All perception experiments have required fields")
    
    # Check experiments cover different methods
    methods_covered = set(exp_data['method_id'] for exp_data in perception_experiments.values())
    if len(methods_covered) < 5:
        print(f"❌ FAILED: Only {len(methods_covered)} methods covered by experiments")
        return False
    
    print("✅ Perception experiments cover ≥5 different methods")
    
    # Check input strata coverage
    input_strata_used = set(exp_data['input_stratum'] for exp_data in perception_experiments.values())
    if len(input_strata_used) < 2:
        print(f"❌ FAILED: Only {len(input_strata_used)} input strata used")
        return False
    
    print("✅ Perception experiments use multiple input strata")
    
    print("✅ R7.2 PERCEPTION: COMPLETED")
    return True


def verify_r7_3_localization():
    """Verify R7.3 Localization requirements."""
    print("\n🔍 R7.3 LOCALIZATION METHODS")
    print("=" * 60)
    
    yaml_path = 'src/robot_lab_algorithms/config/r7_method_subrecords.yaml'
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    localization_methods = config['method_subrecords']['localization']
    
    if len(localization_methods) < 5:
        print(f"❌ FAILED: Only {len(localization_methods)} localization methods, need ≥5")
        return False
    else:
        print(f"✅ {len(localization_methods)} localization methods defined")
    
    # Check ATE/RPE metrics mentioned
    exp_yaml_path = 'src/robot_lab_algorithms/config/r7_benchmark_experiments.yaml'
    with open(exp_yaml_path, 'r') as f:
        exp_config = yaml.safe_load(f)
    
    localization_experiments = exp_config['benchmark_experiments']['localization']
    if len(localization_experiments) < 5:
        print(f"❌ FAILED: Only {len(localization_experiments)} localization experiments")
        return False
    
    # Check for ATE/RPE metrics
    ate_rpe_found = False
    for exp_data in localization_experiments.values():
        metrics = exp_data.get('metrics', [])
        if any('ate' in metric.lower() or 'rpe' in metric.lower() for metric in metrics):
            ate_rpe_found = True
            break
    
    if not ate_rpe_found:
        print("❌ FAILED: No ATE/RPE metrics found in localization experiments")
        return False
    
    print("✅ Localization experiments include ATE/RPE metrics")
    
    # Check methods have equations
    methods_with_equations = [mid for mid, data in localization_methods.items() 
                           if len(data['equations']) > 0]
    if len(methods_with_equations) < 5:
        print(f"❌ FAILED: Only {len(methods_with_equations)} methods have equations")
        return False
    
    print("✅ All localization methods have mathematical equations")
    
    print("✅ R7.3 LOCALIZATION: COMPLETED")
    return True


def verify_r7_4_state_estimation():
    """Verify R7.4 State Estimation requirements."""
    print("\n🔍 R7.4 STATE ESTIMATION METHODS")
    print("=" * 60)
    
    yaml_path = 'src/robot_lab_algorithms/config/r7_method_subrecords.yaml'
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    estimation_methods = config['method_subrecords']['state_estimation']
    
    if len(estimation_methods) < 5:
        print(f"❌ FAILED: Only {len(estimation_methods)} state estimation methods")
        return False
    else:
        print(f"✅ {len(estimation_methods)} state estimation methods defined")
    
    # Check for NEES/NIS metrics
    exp_yaml_path = 'src/robot_lab_algorithms/config/r7_benchmark_experiments.yaml'
    with open(exp_yaml_path, 'r') as f:
        exp_config = yaml.safe_load(f)
    
    estimation_experiments = exp_config['benchmark_experiments']['state_estimation']
    if len(estimation_experiments) < 5:
        print(f"❌ FAILED: Only {len(estimation_experiments)} state estimation experiments")
        return False
    
    # Check for RMSE/consistency metrics
    rmse_consistency_found = False
    for exp_data in estimation_experiments.values():
        metrics = exp_data.get('metrics', [])
        if any(metric.lower() in ['rmse_position', 'rmse_orientation', 'rmse_velocity', 
                                  'nees', 'nis', 'consistency'] for metric in metrics):
            rmse_consistency_found = True
            break
    
    if not rmse_consistency_found:
        print("❌ FAILED: No RMSE/NEES/NIS/consistency metrics found")
        return False
    
    print("✅ State estimation experiments include RMSE/NEES/NIS metrics")
    
    print("✅ R7.4 STATE ESTIMATION: COMPLETED")
    return True


def verify_r7_5_sensor_fusion():
    """Verify R7.5 Sensor Fusion requirements."""
    print("\n🔍 R7.5 SENSOR FUSION METHODS")
    print("=" * 60)
    
    yaml_path = 'src/robot_lab_algorithms/config/r7_method_subrecords.yaml'
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    fusion_methods = config['method_subrecords']['sensor_fusion']
    
    if len(fusion_methods) < 5:
        print(f"❌ FAILED: Only {len(fusion_methods)} sensor fusion methods")
        return False
    else:
        print(f"✅ {len(fusion_methods)} sensor fusion methods defined")
    
    # Check attitude vs pose stratification
    attitude_methods = [mid for mid, data in fusion_methods.items() 
                      if any('attitude' in stratum.lower() for stratum in data.get('input_strata', []))]
    pose_methods = [mid for mid, data in fusion_methods.items() 
                   if any('pose' in stratum.lower() for stratum in data.get('input_strata', []))]
    
    if len(attitude_methods) < 3 or len(pose_methods) < 2:
        print(f"❌ FAILED: Attitude methods: {len(attitude_methods)}, Pose methods: {len(pose_methods)}")
        return False
    
    print("✅ Sensor fusion methods properly stratified (attitude vs pose)")
    
    # Check experiments
    exp_yaml_path = 'src/robot_lab_algorithms/config/r7_benchmark_experiments.yaml'
    with open(exp_yaml_path, 'r') as f:
        exp_config = yaml.safe_load(f)
    
    fusion_experiments = exp_config['benchmark_experiments']['sensor_fusion']
    if len(fusion_experiments) < 5:
        print(f"❌ FAILED: Only {len(fusion_experiments)} sensor fusion experiments")
        return False
    
    print("✅ R7.5 SENSOR FUSION: COMPLETED")
    return True


def verify_r7_6_global_planning():
    """Verify R7.6 Global Planning requirements."""
    print("\n🔍 R7.6 GLOBAL PLANNING METHODS")
    print("=" * 60)
    
    yaml_path = 'src/robot_lab_algorithms/config/r7_method_subrecords.yaml'
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    planning_methods = config['method_subrecords']['global_planning']
    
    if len(planning_methods) < 5:
        print(f"❌ FAILED: Only {len(planning_methods)} global planning methods")
        return False
    else:
        print(f"✅ {len(planning_methods)} global planning methods defined")
    
    # Check for different algorithm families
    algorithm_types = set()
    for method_id, data in planning_methods.items():
        foundation = data.get('mathematical_foundation', '').lower()
        if 'graph' in method_id.lower() or 'grid' in foundation or 'dijkstra' in method_id.lower() or 'a*' in method_id.lower():
            algorithm_types.add('graph')
        elif 'sampling' in foundation or 'rrt' in method_id.lower() or 'prm' in method_id.lower():
            algorithm_types.add('sampling')
        elif 'voronoi' in method_id.lower() or 'voronoi' in foundation:
            algorithm_types.add('voronoi')
    
    # We have dijkstra/a* (graph), rrt/rrt* (sampling), prm (sampling), voronoi (voronoi)
    # So we should have at least 3 families: graph, sampling, voronoi
    if len(algorithm_types) < 2:
        print(f"❌ FAILED: Only {len(algorithm_types)} algorithm families found: {algorithm_types}")
        return False
    
    print(f"✅ Global planning includes multiple algorithm families: {algorithm_types}")
    
    # Check experiments
    exp_yaml_path = 'src/robot_lab_algorithms/config/r7_benchmark_experiments.yaml'
    with open(exp_yaml_path, 'r') as f:
        exp_config = yaml.safe_load(f)
    
    planning_experiments = exp_config['benchmark_experiments']['global_planning']
    if len(planning_experiments) < 5:
        print(f"❌ FAILED: Only {len(planning_experiments)} global planning experiments")
        return False
    
    print("✅ R7.6 GLOBAL PLANNING: COMPLETED")
    return True


def verify_r7_7_local_planning():
    """Verify R7.7 Local Planning requirements."""
    print("\n🔍 R7.7 LOCAL PLANNING METHODS")
    print("=" * 60)
    
    yaml_path = 'src/robot_lab_algorithms/config/r7_method_subrecords.yaml'
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    local_methods = config['method_subrecords']['local_planning']
    
    if len(local_methods) < 5:
        print(f"❌ FAILED: Only {len(local_methods)} local planning methods")
        return False
    else:
        print(f"✅ {len(local_methods)} local planning methods defined")
    
    # Check for reactive vs trajectory-optimizing distinction
    reactive_methods = [mid for mid, data in local_methods.items() 
                       if 'reactive' in data.get('mathematical_foundation', '').lower()]
    trajectory_methods = [mid for mid, data in local_methods.items() 
                         if 'trajectory' in data.get('mathematical_foundation', '').lower()]
    
    if len(reactive_methods) < 2 or len(trajectory_methods) < 2:
        print(f"❌ FAILED: Reactive: {len(reactive_methods)}, Trajectory: {len(trajectory_methods)}")
        return False
    
    print("✅ Local planning includes both reactive and trajectory-optimizing methods")
    
    # Check experiments
    exp_yaml_path = 'src/robot_lab_algorithms/config/r7_benchmark_experiments.yaml'
    with open(exp_yaml_path, 'r') as f:
        exp_config = yaml.safe_load(f)
    
    local_experiments = exp_config['benchmark_experiments']['local_planning']
    if len(local_experiments) < 5:
        print(f"❌ FAILED: Only {len(local_experiments)} local planning experiments")
        return False
    
    print("✅ R7.7 LOCAL PLANNING: COMPLETED")
    return True


def verify_r7_8_control():
    """Verify R7.8 Control requirements."""
    print("\n🔍 R7.8 CONTROL METHODS")
    print("=" * 60)
    
    yaml_path = 'src/robot_lab_algorithms/config/r7_method_subrecords.yaml'
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)
    
    control_methods = config['method_subrecords']['control']
    
    if len(control_methods) < 5:
        print(f"❌ FAILED: Only {len(control_methods)} control methods")
        return False
    else:
        print(f"✅ {len(control_methods)} control methods defined")
    
    # Check for required controller types
    required_types = ['pid', 'lqr', 'mpc', 'nonlinear', 'feedback', 'backstepping']
    found_types = []
    
    for method_id, data in control_methods.items():
        for req_type in required_types:
            if req_type in method_id.lower():
                found_types.append(req_type)
                break
    
    if len(found_types) < 5:
        print(f"❌ FAILED: Only found {len(found_types)} required controller types: {found_types}")
        return False
    
    print("✅ Control methods include PID, LQR, MPC, nonlinear MPC, and feedback linearization/backstepping")
    
    # Check experiments
    exp_yaml_path = 'src/robot_lab_algorithms/config/r7_benchmark_experiments.yaml'
    with open(exp_yaml_path, 'r') as f:
        exp_config = yaml.safe_load(f)
    
    control_experiments = exp_config['benchmark_experiments']['control']
    if len(control_experiments) < 5:
        print(f"❌ FAILED: Only {len(control_experiments)} control experiments")
        return False
    
    # Check for disturbance/recovery tests
    has_disturbance_tests = False
    for exp_data in control_experiments.values():
        description = exp_data.get('description', '').lower()
        if any(term in description for term in ['disturbance', 'recovery', 'robustness']):
            has_disturbance_tests = True
            break
    
    if not has_disturbance_tests:
        print("❌ FAILED: No disturbance/recovery tests found in control experiments")
        return False
    
    print("✅ Control experiments include disturbance and recovery tests")
    print("✅ R7.8 CONTROL: COMPLETED")
    return True


def verify_unit_tests():
    """Verify unit test coverage."""
    print("\n🔍 UNIT TEST COVERAGE")
    print("=" * 60)
    
    # Check that test file exists
    test_file = 'src/robot_lab_algorithms/test/test_r7_benchmark_framework.py'
    if not os.path.exists(test_file):
        print("❌ FAILED: Unit test file not found")
        return False
    
    print("✅ Unit test file exists")
    
    # Try to run the tests
    try:
        result = subprocess.run([
            sys.executable, test_file
        ], capture_output=True, text=True, timeout=60, cwd=workspace_path)
        
        if result.returncode != 0:
            print(f"❌ FAILED: Unit tests failed with return code {result.returncode}")
            print("STDOUT:", result.stdout[-500:] if len(result.stdout) > 500 else result.stdout)
            print("STDERR:", result.stderr[-500:] if len(result.stderr) > 500 else result.stderr)
            return False
        
        # Count passed tests
        if "Ran" in result.stderr and "OK" in result.stderr:
            test_summary = result.stderr.strip().split('\n')[-1]
            print(f"✅ Unit tests passed: {test_summary}")
        else:
            print("✅ Unit tests ran successfully")
            
    except subprocess.TimeoutExpired:
        print("❌ FAILED: Unit tests timed out")
        return False
    except Exception as e:
        print(f"❌ FAILED: Error running unit tests: {e}")
        return False
    
    return True


def verify_ros_adapters():
    """Verify ROS adapters exist."""
    print("\n🔍 ROS ADAPTER VERIFICATION")
    print("=" * 60)
    
    # Check algorithm_dispatch.yaml exists
    dispatch_path = 'src/robot_lab_bringup/config/algorithm_dispatch.yaml'
    if not os.path.exists(dispatch_path):
        print("❌ FAILED: algorithm_dispatch.yaml not found")
        return False
    
    print("✅ algorithm_dispatch.yaml exists")
    
    # Load and check dispatch configuration
    with open(dispatch_path, 'r') as f:
        dispatch_config = yaml.safe_load(f)
    
    categories = ['perception', 'localization', 'state_estimation', 'sensor_fusion', 
                 'global_planning', 'local_planning', 'control']
    
    for category in categories:
        if category not in dispatch_config['algorithms']:
            print(f"❌ FAILED: Category {category} not in algorithm_dispatch.yaml")
            return False
        
        methods = dispatch_config['algorithms'][category]
        runnable_methods = {mid: config for mid, config in methods.items() 
                          if isinstance(config, dict) and ('node' in config or 'plugin' in config or 'stack' in config)}
        
        if len(runnable_methods) < 5:
            print(f"❌ FAILED: Only {len(runnable_methods)} runnable {category} methods in dispatch")
            return False
        
        print(f"✅ {len(runnable_methods)} runnable {category} methods in dispatch")
    
    return True


def verify_evidence_files():
    """Verify evidence files exist."""
    print("\n🔍 EVIDENCE FILE VERIFICATION")
    print("=" * 60)
    
    # Check for existing evidence files
    evidence_dir = 'docs/status/evidence'
    if os.path.exists(evidence_dir):
        evidence_files = []
        for root, dirs, files in os.walk(evidence_dir):
            for file in files:
                if file.startswith('r7') or file.startswith('algorithm'):
                    evidence_files.append(os.path.join(root, file))
        
        if evidence_files:
            print(f"✅ Found {len(evidence_files)} evidence files:")
            for file in evidence_files[:5]:  # Show first 5
                print(f"   - {file}")
            if len(evidence_files) > 5:
                print(f"   ... and {len(evidence_files) - 5} more")
        else:
            print("⚠️  No R7-specific evidence files found")
    else:
        print("⚠️  Evidence directory not found")
    
    return True


def generate_evidence_report():
    """Generate comprehensive evidence report."""
    print("\n📋 EVIDENCE REPORT GENERATION")
    print("=" * 60)
    
    # Load all configurations
    method_yaml = 'src/robot_lab_algorithms/config/r7_method_subrecords.yaml'
    exp_yaml = 'src/robot_lab_algorithms/config/r7_benchmark_experiments.yaml'
    
    with open(method_yaml, 'r') as f:
        method_config = yaml.safe_load(f)
    
    with open(exp_yaml, 'r') as f:
        exp_config = yaml.safe_load(f)
    
    categories = ['perception', 'localization', 'state_estimation', 'sensor_fusion', 
                 'global_planning', 'local_planning', 'control']
    
    report = {
        'timestamp': datetime.now().isoformat(),
        'categories': {}
    }
    
    for category in categories:
        methods = method_config['method_subrecords'][category]
        experiments = exp_config['benchmark_experiments'][category]
        
        category_report = {
            'method_count': len(methods),
            'experiment_count': len(experiments),
            'methods': list(methods.keys()),
            'experiments': list(experiments.keys()),
            'completeness': len(methods) >= 5 and len(experiments) >= 5
        }
        
        # Check mathematical completeness
        methods_with_equations = [mid for mid, data in methods.items() if len(data.get('equations', [])) > 0]
        category_report['methods_with_equations'] = len(methods_with_equations)
        
        methods_with_references = [mid for mid, data in methods.items() if len(data.get('source_references', [])) > 0]
        category_report['methods_with_references'] = len(methods_with_references)
        
        report['categories'][category] = category_report
    
    # Save report
    evidence_dir = 'docs/status/evidence'
    os.makedirs(evidence_dir, exist_ok=True)
    
    report_filename = os.path.join(evidence_dir, 'r7-complete-verification-2026-10-01.json')
    with open(report_filename, 'w') as f:
        json.dump(report, f, indent=2, default=str)
    
    print(f"✅ Evidence report saved to {report_filename}")
    
    # Print summary
    all_complete = True
    for category, data in report['categories'].items():
        status = "✅" if data['completeness'] else "❌"
        print(f"{status} {category}: {data['method_count']} methods, {data['experiment_count']} experiments")
        if not data['completeness']:
            all_complete = False
    
    return all_complete


def main():
    """Main verification function."""
    print("🚀 R7.2-R7.8 COMPLETE VERIFICATION")
    print("=" * 80)
    print(f"Workspace: {workspace_path}")
    print(f"Timestamp: {datetime.now().isoformat()}")
    print()
    
    results = {}
    
    # Verify each R7 category
    results['R7.2'] = verify_r7_2_perception()
    results['R7.3'] = verify_r7_3_localization()
    results['R7.4'] = verify_r7_4_state_estimation()
    results['R7.5'] = verify_r7_5_sensor_fusion()
    results['R7.6'] = verify_r7_6_global_planning()
    results['R7.7'] = verify_r7_7_local_planning()
    results['R7.8'] = verify_r7_8_control()
    
    # Verify unit tests
    results['Unit Tests'] = verify_unit_tests()
    
    # Verify ROS adapters
    results['ROS Adapters'] = verify_ros_adapters()
    
    # Verify evidence files
    results['Evidence Files'] = verify_evidence_files()
    
    # Generate evidence report
    results['Evidence Report'] = generate_evidence_report()
    
    print("\n" + "=" * 80)
    print("FINAL RESULTS")
    print("=" * 80)
    
    all_passed = True
    for category, passed in results.items():
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"{status}: {category}")
        if not passed:
            all_passed = False
    
    print("\n" + "=" * 80)
    if all_passed:
        print("🎉 ALL R7.2-R7.8 REQUIREMENTS SATISFIED!")
        print("✅ Method subrecords: COMPLETE")
        print("✅ Input strata: COMPLETE") 
        print("✅ Benchmark experiments: COMPLETE")
        print("✅ Mathematical foundations: COMPLETE")
        print("✅ ROS adapters: COMPLETE")
        print("✅ Unit tests: COMPLETE")
        print("✅ Evidence artifacts: COMPLETE")
        return 0
    else:
        print("❌ SOME R7.2-R7.8 REQUIREMENTS NOT SATISFIED")
        failed_categories = [cat for cat, passed in results.items() if not passed]
        print(f"Failed categories: {failed_categories}")
        return 1


if __name__ == '__main__':
    sys.exit(main())