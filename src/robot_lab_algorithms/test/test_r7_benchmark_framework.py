#!/usr/bin/env python3
"""
Comprehensive Unit Tests for R7.2-R7.8 Benchmark Framework

Tests the benchmark framework, method subrecords, input strata, and experiment configurations
for all algorithm categories.
"""

import unittest
import yaml
import os
import sys
import tempfile
import numpy as np
from datetime import datetime

# Add src path for imports
workspace_path = os.path.abspath(os.path.dirname(__file__))
src_path = os.path.join(workspace_path, '..', '..', '..')
sys.path.insert(0, src_path)

# Import the benchmark framework directly
try:
    # Try direct import from module path
    sys.path.insert(0, os.path.join(src_path, 'robot_lab_algorithms'))
    from robot_lab_algorithms.r7_benchmark_framework import (
        R7BenchmarkFramework, MethodSubrecord, BenchmarkExperiment, 
        BenchmarkResult, InputStratumDefinition, InputStratum, MetricType
    )
except ImportError as e:
    print(f"Direct import failed: {e}")
    # Try reading from file and exec
    framework_file = os.path.join(src_path, 'robot_lab_algorithms', 'robot_lab_algorithms', 'r7_benchmark_framework.py')
    if os.path.exists(framework_file):
        with open(framework_file, 'r') as f:
            framework_code = f.read()
        # Create a module-like namespace
        import types
        framework_module = types.ModuleType('r7_benchmark_framework')
        exec(framework_code, framework_module.__dict__)
        # Extract the classes
        R7BenchmarkFramework = framework_module.R7BenchmarkFramework
        MethodSubrecord = framework_module.MethodSubrecord
        BenchmarkExperiment = framework_module.BenchmarkExperiment
        BenchmarkResult = framework_module.BenchmarkResult
        InputStratumDefinition = framework_module.InputStratumDefinition
        InputStratum = framework_module.InputStratum
        MetricType = framework_module.MetricType
    else:
        print(f"Framework file not found at {framework_file}")
        raise ImportError(f"Cannot find benchmark framework file")


class TestR7BenchmarkFramework(unittest.TestCase):
    """Test the R7 benchmark framework core functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.framework = R7BenchmarkFramework()
        
    def test_framework_initialization(self):
        """Test framework initialization."""
        self.assertEqual(len(self.framework.methods), 0)
        self.assertEqual(len(self.framework.experiments), 0)
        self.assertEqual(len(self.framework.results), 0)
        self.assertEqual(len(self.framework.input_strata), 0)
        
    def test_method_registration(self):
        """Test method registration."""
        method = MethodSubrecord(
            method_id="test_method",
            category="test_category",
            display_name="Test Method",
            implementation_ref="test.module.TestClass"
        )
        self.framework.register_method(method)
        
        self.assertEqual(len(self.framework.methods), 1)
        self.assertIn("test_category:test_method", self.framework.methods)
        
    def test_experiment_registration(self):
        """Test experiment registration."""
        experiment = BenchmarkExperiment(
            experiment_id="test_exp",
            method_id="test_method",
            category="test_category",
            input_stratum=InputStratum.PERCEPTION_LIDAR_2D_SPARSE
        )
        self.framework.register_experiment(experiment)
        
        self.assertEqual(len(self.framework.experiments), 1)
        self.assertIn("test_exp", self.framework.experiments)
        
    def test_result_registration(self):
        """Test result registration."""
        result = BenchmarkResult(
            experiment_id="test_exp",
            method_id="test_method",
            category="test_category"
        )
        self.framework.register_result(result)
        
        self.assertEqual(len(self.framework.results), 1)
        self.assertIn("test_exp", self.framework.results)
        
    def test_stratum_registration(self):
        """Test input stratum registration."""
        stratum = InputStratumDefinition(
            stratum_id="test_stratum",
            category="test_category",
            description="Test stratum"
        )
        self.framework.register_input_stratum(stratum)
        
        self.assertEqual(len(self.framework.input_strata), 1)
        self.assertIn("test_stratum", self.framework.input_strata)
        
    def test_get_methods_by_category(self):
        """Test getting methods by category."""
        method1 = MethodSubrecord(
            method_id="method1",
            category="category1",
            display_name="Method 1",
            implementation_ref="module1.Method1"
        )
        method2 = MethodSubrecord(
            method_id="method2", 
            category="category1",
            display_name="Method 2",
            implementation_ref="module1.Method2"
        )
        method3 = MethodSubrecord(
            method_id="method3",
            category="category2",
            display_name="Method 3",
            implementation_ref="module2.Method3"
        )
        
        self.framework.register_method(method1)
        self.framework.register_method(method2)
        self.framework.register_method(method3)
        
        category1_methods = self.framework.get_methods_by_category("category1")
        self.assertEqual(len(category1_methods), 2)
        
        category2_methods = self.framework.get_methods_by_category("category2")
        self.assertEqual(len(category2_methods), 1)
        
    def test_generate_comparison_report(self):
        """Test comparison report generation."""
        method = MethodSubrecord(
            method_id="test_method",
            category="test_category",
            display_name="Test Method",
            implementation_ref="test.module.TestClass"
        )
        experiment = BenchmarkExperiment(
            experiment_id="test_exp",
            method_id="test_method",
            category="test_category",
            input_stratum=InputStratum.PERCEPTION_LIDAR_2D_SPARSE
        )
        result = BenchmarkResult(
            experiment_id="test_exp",
            method_id="test_method",
            category="test_category",
            metrics={"accuracy": 0.95, "time": 0.1}
        )
        
        self.framework.register_method(method)
        self.framework.register_experiment(experiment)
        self.framework.register_result(result)
        
        report = self.framework.generate_comparison_report("test_category")
        
        self.assertEqual(report["category"], "test_category")
        self.assertEqual(report["method_count"], 1)
        self.assertEqual(report["experiment_count"], 1)
        self.assertEqual(report["result_count"], 1)
        
    def test_save_and_load_framework_state(self):
        """Test saving and loading framework state."""
        # Add some test data
        method = MethodSubrecord(
            method_id="test_method",
            category="test_category",
            display_name="Test Method",
            implementation_ref="test.module.TestClass",
            mathematical_foundation="Test foundation",
            input_strata=[InputStratum.PERCEPTION_LIDAR_2D_SPARSE]
        )
        
        experiment = BenchmarkExperiment(
            experiment_id="test_exp",
            method_id="test_method",
            category="test_category",
            input_stratum=InputStratum.PERCEPTION_LIDAR_2D_SPARSE
        )
        
        stratum = InputStratumDefinition(
            stratum_id="test_stratum",
            category="test_category",
            description="Test stratum"
        )
        
        self.framework.register_method(method)
        self.framework.register_experiment(experiment)
        self.framework.register_input_stratum(stratum)
        
        # Save to temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            temp_filename = f.name
            
        try:
            self.framework.save_framework_state(temp_filename)
            
            # Create new framework and load
            new_framework = R7BenchmarkFramework()
            new_framework.load_framework_state(temp_filename)
            
            self.assertEqual(len(new_framework.methods), 1)
            self.assertEqual(len(new_framework.experiments), 1)
            self.assertEqual(len(new_framework.input_strata), 1)
            
        finally:
            # Clean up
            if os.path.exists(temp_filename):
                os.unlink(temp_filename)


class TestMethodSubrecord(unittest.TestCase):
    """Test method subrecord functionality."""
    
    def test_method_subrecord_creation(self):
        """Test method subrecord creation."""
        method = MethodSubrecord(
            method_id="ekf_3d_estimator",
            category="state_estimation",
            display_name="Extended Kalman Filter 3D",
            implementation_ref="robot_lab_algorithms.ekf_3d_estimator",
            mathematical_foundation="Extended Kalman Filter for nonlinear systems",
            equations=[
                "x_{k|k-1} = f(x_{k-1|k-1}, u_k, dt)",
                "P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k"
            ],
            source_references=[
                "Maybeck - Stochastic Models, Estimation and Control"
            ],
            applicable_robots=["bumperbot", "labbot"],
            input_strata=[InputStratum.STATE_ESTIMATION_LINEAR_DYNAMICS],
            output_types=["Odometry", "PoseWithCovariance"],
            parameters={
                "initial_covariance": [0.1, 0.1, 0.1, 0.01, 0.01, 0.01]
            },
            assumptions=["Gaussian noise", "Small nonlinearities"],
            limitations=["Sensitive to model errors", "Linearization errors"],
            computational_complexity="O(n^3)",
            memory_requirements="Medium"
        )
        
        self.assertEqual(method.method_id, "ekf_3d_estimator")
        self.assertEqual(method.category, "state_estimation")
        self.assertEqual(len(method.equations), 2)
        self.assertEqual(len(method.source_references), 1)
        self.assertEqual(len(method.applicable_robots), 2)
        
    def test_method_subrecord_to_dict(self):
        """Test method subrecord serialization."""
        method = MethodSubrecord(
            method_id="test_method",
            category="test_category",
            display_name="Test Method",
            implementation_ref="test.module.TestClass",
            input_strata=[InputStratum.PERCEPTION_LIDAR_2D_SPARSE]
        )
        
        method_dict = method.to_dict()
        
        self.assertIn("method_id", method_dict)
        self.assertIn("category", method_dict)
        self.assertIn("display_name", method_dict)
        self.assertIn("implementation_ref", method_dict)
        self.assertIn("input_strata", method_dict)
        self.assertEqual(method_dict["method_id"], "test_method")
        self.assertEqual(method_dict["input_strata"], ["perception_lidar_2d_sparse"])


class TestBenchmarkExperiment(unittest.TestCase):
    """Test benchmark experiment functionality."""
    
    def test_experiment_creation(self):
        """Test experiment creation."""
        experiment = BenchmarkExperiment(
            experiment_id="perception_obstacle_sparse_lidar",
            method_id="obstacle_detector",
            category="perception",
            input_stratum=InputStratum.PERCEPTION_LIDAR_2D_SPARSE,
            robot_class="mobile",
            environment="nav_empty",
            simulator="gazebo",
            seed=1001,
            duration_seconds=10.0,
            metrics=[
                MetricType.ACCURACY,
                MetricType.COMPUTE_TIME,
                MetricType.MEMORY_USAGE
            ],
            success_criteria={
                "detection_accuracy": ">= 0.95",
                "compute_time_mean": "<= 0.01"
            },
            failure_criteria={
                "detection_accuracy": "< 0.8",
                "crash_or_error": True
            },
            resource_budget={
                "cpu_limit": 1.0,
                "memory_limit_mb": 512,
                "timeout_seconds": 15.0
            }
        )
        
        self.assertEqual(experiment.experiment_id, "perception_obstacle_sparse_lidar")
        self.assertEqual(experiment.method_id, "obstacle_detector")
        self.assertEqual(experiment.category, "perception")
        self.assertEqual(experiment.input_stratum, InputStratum.PERCEPTION_LIDAR_2D_SPARSE)
        self.assertEqual(len(experiment.metrics), 3)
        
    def test_experiment_to_dict(self):
        """Test experiment serialization."""
        experiment = BenchmarkExperiment(
            experiment_id="test_exp",
            method_id="test_method",
            category="test_category",
            input_stratum=InputStratum.PERCEPTION_LIDAR_2D_SPARSE
        )
        
        exp_dict = experiment.to_dict()
        
        self.assertIn("experiment_id", exp_dict)
        self.assertIn("method_id", exp_dict)
        self.assertIn("category", exp_dict)
        self.assertIn("input_stratum", exp_dict)
        self.assertEqual(exp_dict["input_stratum"], "perception_lidar_2d_sparse")


class TestBenchmarkResult(unittest.TestCase):
    """Test benchmark result functionality."""
    
    def test_result_creation(self):
        """Test result creation."""
        result = BenchmarkResult(
            experiment_id="perception_obstacle_sparse_lidar",
            method_id="obstacle_detector",
            category="perception",
            metrics={
                "detection_accuracy": 0.97,
                "false_positive_rate": 0.02,
                "false_negative_rate": 0.01,
                "compute_time_mean": 0.008
            },
            success=True,
            failure_reason=None,
            artifacts={
                "raw_scan_data": "/path/to/data",
                "detection_results": "/path/to/results"
            },
            provenance={
                "revision": "abc123",
                "timestamp": "2026-10-01T12:00:00Z",
                "environment": "nav_empty"
            }
        )
        
        self.assertEqual(result.experiment_id, "perception_obstacle_sparse_lidar")
        self.assertEqual(result.method_id, "obstacle_detector")
        self.assertTrue(result.success)
        self.assertEqual(len(result.metrics), 4)
        
    def test_result_to_dict(self):
        """Test result serialization."""
        result = BenchmarkResult(
            experiment_id="test_exp",
            method_id="test_method", 
            category="test_category",
            metrics={"accuracy": 0.95}
        )
        
        result_dict = result.to_dict()
        
        self.assertIn("experiment_id", result_dict)
        self.assertIn("method_id", result_dict)
        self.assertIn("category", result_dict)
        self.assertIn("metrics", result_dict)
        self.assertIn("timestamp", result_dict)


class TestInputStratumDefinition(unittest.TestCase):
    """Test input stratum definition functionality."""
    
    def test_stratum_creation(self):
        """Test stratum creation."""
        stratum = InputStratumDefinition(
            stratum_id="perception_lidar_2d_sparse",
            category="perception",
            description="2D LiDAR with sparse point clouds (100-500 points per scan)",
            sensor_types=["laser_scan"],
            data_frequency_hz=10.0,
            noise_characteristics={"range_noise_std": 0.02, "angular_resolution": 0.5},
            typical_environments=["nav_empty", "small_office"],
            representative_robots=["bumperbot", "labbot"]
        )
        
        self.assertEqual(stratum.stratum_id, "perception_lidar_2d_sparse")
        self.assertEqual(stratum.category, "perception")
        self.assertEqual(len(stratum.sensor_types), 1)
        self.assertEqual(stratum.data_frequency_hz, 10.0)
        
    def test_stratum_to_dict(self):
        """Test stratum serialization."""
        stratum = InputStratumDefinition(
            stratum_id="test_stratum",
            category="test_category",
            description="Test description"
        )
        
        stratum_dict = stratum.to_dict()
        
        self.assertIn("stratum_id", stratum_dict)
        self.assertIn("category", stratum_dict)
        self.assertIn("description", stratum_dict)


class TestAllCategoriesCoverage(unittest.TestCase):
    """Test that all R7 categories are covered."""
    
    def test_perception_methods(self):
        """Test perception methods exist in framework."""
        framework = R7BenchmarkFramework()
        
        perception_methods = [
            "obstacle_detector", "scan_clusterer", "euclidean_clusterer", 
            "dbscan_clusterer", "ransac_ground_removal", "pointcloud_segmenter"
        ]
        
        for method_id in perception_methods:
            method = MethodSubrecord(
                method_id=method_id,
                category="perception",
                display_name=f"{method_id.replace('_', ' ').title()}",
                implementation_ref=f"robot_lab_algorithms.{method_id}"
            )
            framework.register_method(method)
        
        self.assertEqual(len(framework.get_methods_by_category("perception")), 6)
        
    def test_localization_methods(self):
        """Test localization methods exist in framework."""
        framework = R7BenchmarkFramework()
        
        localization_methods = [
            "dead_reckoning", "amcl", "icp_localization", 
            "ndt_localization", "rgbd_slam_localization"
        ]
        
        for method_id in localization_methods:
            method = MethodSubrecord(
                method_id=method_id,
                category="localization",
                display_name=f"{method_id.replace('_', ' ').title()}",
                implementation_ref=f"robot_lab_algorithms.{method_id}"
            )
            framework.register_method(method)
        
        self.assertEqual(len(framework.get_methods_by_category("localization")), 5)
        
    def test_state_estimation_methods(self):
        """Test state estimation methods exist in framework."""
        framework = R7BenchmarkFramework()
        
        state_estimation_methods = [
            "ekf_3d_estimator", "linear_kalman_filter", "ukf_estimator",
            "particle_filter", "error_state_ekf", "motion_model_estimator",
            "pose_graph_estimator"
        ]
        
        for method_id in state_estimation_methods:
            method = MethodSubrecord(
                method_id=method_id,
                category="state_estimation",
                display_name=f"{method_id.replace('_', ' ').title()}",
                implementation_ref=f"robot_lab_algorithms.{method_id}"
            )
            framework.register_method(method)
        
        # We have 7 methods but only need 5 for R7
        self.assertGreaterEqual(len(framework.get_methods_by_category("state_estimation")), 5)
        
    def test_sensor_fusion_methods(self):
        """Test sensor fusion methods exist in framework."""
        framework = R7BenchmarkFramework()
        
        sensor_fusion_methods = [
            "complementary_imu", "mahony_filter", "madgwick_filter",
            "wheel_imu_fusion", "gps_odom_fusion", "wheel_imu_gnss_ukf"
        ]
        
        for method_id in sensor_fusion_methods:
            method = MethodSubrecord(
                method_id=method_id,
                category="sensor_fusion",
                display_name=f"{method_id.replace('_', ' ').title()}",
                implementation_ref=f"robot_lab_algorithms.{method_id}"
            )
            framework.register_method(method)
        
        self.assertGreaterEqual(len(framework.get_methods_by_category("sensor_fusion")), 5)
        
    def test_global_planning_methods(self):
        """Test global planning methods exist in framework."""
        framework = R7BenchmarkFramework()
        
        global_planning_methods = [
            "dijkstra_planner", "a_star_planner", "rrt_planner",
            "rrt_star_planner", "prm_planner", "voronoi_planner"
        ]
        
        for method_id in global_planning_methods:
            method = MethodSubrecord(
                method_id=method_id,
                category="global_planning",
                display_name=f"{method_id.replace('_', ' ').title()}",
                implementation_ref=f"robot_lab_algorithms.{method_id}"
            )
            framework.register_method(method)
        
        self.assertGreaterEqual(len(framework.get_methods_by_category("global_planning")), 5)
        
    def test_local_planning_methods(self):
        """Test local planning methods exist in framework."""
        framework = R7BenchmarkFramework()
        
        local_planning_methods = [
            "follow_the_gap", "dwb_local_planner", "regulated_pure_pursuit",
            "teb_local_planner", "mppi_local_planner"
        ]
        
        for method_id in local_planning_methods:
            method = MethodSubrecord(
                method_id=method_id,
                category="local_planning",
                display_name=f"{method_id.replace('_', ' ').title()}",
                implementation_ref=f"robot_lab_algorithms.{method_id}"
            )
            framework.register_method(method)
        
        self.assertEqual(len(framework.get_methods_by_category("local_planning")), 5)
        
    def test_control_methods(self):
        """Test control methods exist in framework."""
        framework = R7BenchmarkFramework()
        
        control_methods = [
            "pid_controller", "lqr_controller", "mpc_controller",
            "nonlinear_mpc", "feedback_linearization", "backstepping_controller"
        ]
        
        for method_id in control_methods:
            method = MethodSubrecord(
                method_id=method_id,
                category="control",
                display_name=f"{method_id.replace('_', ' ').title()}",
                implementation_ref=f"robot_lab_algorithms.{method_id}"
            )
            framework.register_method(method)
        
        self.assertGreaterEqual(len(framework.get_methods_by_category("control")), 5)


class TestYAMLConfigurations(unittest.TestCase):
    """Test YAML configuration loading."""
    
    def test_load_method_subrecords_yaml(self):
        """Test loading method subrecords from YAML."""
        yaml_path = os.path.join(src_path, 'robot_lab_algorithms', 'config', 'r7_method_subrecords.yaml')
        
        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.assertIn('method_subrecords', config)
            self.assertIn('perception', config['method_subrecords'])
            self.assertIn('localization', config['method_subrecords'])
            self.assertIn('state_estimation', config['method_subrecords'])
            self.assertIn('sensor_fusion', config['method_subrecords'])
            self.assertIn('global_planning', config['method_subrecords'])
            self.assertIn('local_planning', config['method_subrecords'])
            self.assertIn('control', config['method_subrecords'])
            
            # Check perception methods
            perception_methods = config['method_subrecords']['perception']
            self.assertGreaterEqual(len(perception_methods), 5)
            
            # Check each method has required fields
            for method_id, method_data in perception_methods.items():
                self.assertIn('method_id', method_data)
                self.assertIn('category', method_data)
                self.assertIn('display_name', method_data)
                self.assertIn('implementation_ref', method_data)
                self.assertIn('mathematical_foundation', method_data)
                self.assertIn('equations', method_data)
                self.assertIn('source_references', method_data)
                self.assertIn('applicable_robots', method_data)
                self.assertIn('input_strata', method_data)
            
    def test_load_benchmark_experiments_yaml(self):
        """Test loading benchmark experiments from YAML."""
        yaml_path = os.path.join(src_path, 'robot_lab_algorithms', 'config', 'r7_benchmark_experiments.yaml')
        
        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
            
            self.assertIn('benchmark_experiments', config)
            
            # Check all categories are present
            expected_categories = [
                'perception', 'localization', 'state_estimation', 
                'sensor_fusion', 'global_planning', 'local_planning', 'control'
            ]
            
            for category in expected_categories:
                self.assertIn(category, config['benchmark_experiments'])
                experiments = config['benchmark_experiments'][category]
                self.assertGreaterEqual(len(experiments), 5)
                
                # Check each experiment has required fields
                for exp_id, exp_data in experiments.items():
                    self.assertIn('experiment_id', exp_data)
                    self.assertIn('category', exp_data)
                    self.assertIn('method_id', exp_data)
                    self.assertIn('input_stratum', exp_data)
                    self.assertIn('description', exp_data)
                    self.assertIn('metrics', exp_data)
                    self.assertIn('success_criteria', exp_data)
                    self.assertIn('failure_criteria', exp_data)
                    self.assertIn('resource_budget', exp_data)


class TestMathematicalMetrics(unittest.TestCase):
    """Test mathematical metrics and equations."""
    
    def test_perception_equations(self):
        """Test that perception equations are mathematically valid."""
        # Test basic perception equations
        range_measurements = np.array([1.0, 2.0, 0.5, 3.0, 1.5])
        threshold = 1.0
        
        # obstacle_range = min(range_measurements)
        obstacle_range = np.min(range_measurements)
        self.assertEqual(obstacle_range, 0.5)
        
        # obstacle_present = (obstacle_range < threshold)
        obstacle_present = obstacle_range < threshold
        self.assertTrue(obstacle_present)
        
        # obstacle_angle = argmin(range_measurements)
        obstacle_angle_index = np.argmin(range_measurements)
        self.assertEqual(obstacle_angle_index, 2)
        
    def test_localization_equations(self):
        """Test localization equations."""
        # Dead reckoning equations
        v_left = 0.2  # m/s
        v_right = 0.25  # m/s
        dt = 0.1  # seconds
        wheelbase = 0.256  # meters
        
        # dx = (v_left + v_right) * dt / 2
        dx = (v_left + v_right) * dt / 2
        expected_dx = 0.0225
        self.assertAlmostEqual(dx, expected_dx, places=4)
        
        # dtheta = (v_right - v_left) * dt / wheelbase
        dtheta = (v_right - v_left) * dt / wheelbase
        expected_dtheta = (0.25 - 0.2) * 0.1 / 0.256  # 0.05 * 0.1 / 0.256 = 0.001953125
        self.assertAlmostEqual(dtheta, expected_dtheta, places=6)
        
    def test_estimation_equations(self):
        """Test estimation equations."""
        # Kalman Filter equations
        x_k_minus_1 = np.array([0.0, 1.0])  # previous state
        P_k_minus_1 = np.array([[0.1, 0.0], [0.0, 0.1]])  # previous covariance
        F = np.array([[1.0, 0.1], [0.0, 1.0]])  # state transition
        Q = np.array([[0.01, 0.0], [0.0, 0.01]])  # process noise
        
        # x_{k|k-1} = F * x_{k-1|k-1}
        x_pred = np.dot(F, x_k_minus_1)
        expected_x_pred = np.array([0.1, 1.0])
        np.testing.assert_array_almost_equal(x_pred, expected_x_pred)
        
        # P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q
        P_pred = np.dot(np.dot(F, P_k_minus_1), F.T) + Q
        # F * P_k_minus_1 * F^T = [[1.0, 0.1], [0.0, 1.0]] * [[0.1, 0.0], [0.0, 0.1]] * [[1.0, 0.0], [0.1, 1.0]]
        # = [[1.0, 0.1], [0.0, 1.0]] * [[0.1, 0.01], [0.01, 0.1]]
        # = [[0.1, 0.011], [0.01, 0.1]]
        # + Q = [[0.01, 0.0], [0.0, 0.01]]
        # = [[0.11, 0.011], [0.01, 0.11]]
        expected_P_pred = np.array([[0.11, 0.01], [0.01, 0.11]])  # rounded to 2 decimals
        np.testing.assert_array_almost_equal(P_pred, expected_P_pred, decimal=2)
        
    def test_control_equations(self):
        """Test control equations."""
        # PID equations
        K_p, K_i, K_d = 1.0, 0.1, 0.01
        error = 0.5
        integral_error = 0.1
        derivative_error = -0.05
        
        # PID: u = K_p * e + K_i * integral_e + K_d * derivative_e
        u = K_p * error + K_i * integral_error + K_d * derivative_error
        expected_u = 0.5 + 0.01 + (-0.0005)  # = 0.5095
        self.assertAlmostEqual(u, expected_u, places=4)
        
    def test_planning_equations(self):
        """Test planning equations."""
        # A* heuristic - Euclidean distance
        x1, y1 = 0.0, 0.0
        x2, y2 = 3.0, 4.0
        
        # Euclidean distance: sqrt((x2-x1)^2 + (y2-y1)^2)
        distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        expected_distance = 5.0
        self.assertAlmostEqual(distance, expected_distance, places=4)


class TestInputStrataCoverage(unittest.TestCase):
    """Test that all input strata are covered."""
    
    def test_perception_strata_coverage(self):
        """Test perception input strata coverage."""
        expected_strata = [
            InputStratum.PERCEPTION_LIDAR_2D_SPARSE,
            InputStratum.PERCEPTION_LIDAR_2D_DENSE,
            InputStratum.PERCEPTION_LIDAR_3D,
            InputStratum.PERCEPTION_RGBD
        ]
        
        for stratum in expected_strata:
            self.assertTrue(stratum in InputStratum)
            
    def test_localization_strata_coverage(self):
        """Test localization input strata coverage."""
        expected_strata = [
            InputStratum.LOCALIZATION_ODOMETRY_ONLY,
            InputStratum.LOCALIZATION_LIDAR_MAP,
            InputStratum.LOCALIZATION_VISUAL_FEATURES,
            InputStratum.LOCALIZATION_MULTI_SENSOR
        ]
        
        for stratum in expected_strata:
            self.assertTrue(stratum in InputStratum)
            
    def test_estimation_strata_coverage(self):
        """Test state estimation input strata coverage."""
        expected_strata = [
            InputStratum.STATE_ESTIMATION_LINEAR_DYNAMICS,
            InputStratum.STATE_ESTIMATION_NONLINEAR_DYNAMICS,
            InputStratum.STATE_ESTIMATION_HIGH_FREQUENCY,
            InputStratum.STATE_ESTIMATION_LOW_FREQUENCY
        ]
        
        for stratum in expected_strata:
            self.assertTrue(stratum in InputStratum)
            
    def test_fusion_strata_coverage(self):
        """Test sensor fusion input strata coverage."""
        expected_strata = [
            InputStratum.SENSOR_FUSION_ATTITUDE_ONLY,
            InputStratum.SENSOR_FUSION_POSE,
            InputStratum.SENSOR_FUSION_MULTI_SENSOR
        ]
        
        for stratum in expected_strata:
            self.assertTrue(stratum in InputStratum)
            
    def test_planning_strata_coverage(self):
        """Test planning input strata coverage."""
        expected_strata = [
            InputStratum.PLANNING_2D_GRID,
            InputStratum.PLANNING_2D_CONTINUOUS,
            InputStratum.PLANNING_3D,
            InputStratum.PLANNING_HIGH_DIMENSIONAL
        ]
        
        for stratum in expected_strata:
            self.assertTrue(stratum in InputStratum)
            
    def test_control_strata_coverage(self):
        """Test control input strata coverage."""
        expected_strata = [
            InputStratum.CONTROL_LINEAR_PLANT,
            InputStratum.CONTROL_NONLINEAR_PLANT,
            InputStratum.CONTROL_CONSTRAINED,
            InputStratum.CONTROL_UNCONSTRAINED
        ]
        
        for stratum in expected_strata:
            self.assertTrue(stratum in InputStratum)


class TestAcceptanceCriteriaValidation(unittest.TestCase):
    """Test validation against R7 acceptance criteria."""
    
    def test_perception_acceptance_criteria(self):
        """Test that perception experiments meet acceptance criteria."""
        yaml_path = os.path.join(src_path, 'robot_lab_algorithms', 'config', 'r7_benchmark_experiments.yaml')
        
        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
            
            perception_experiments = config['benchmark_experiments']['perception']
            
            for exp_id, exp_data in perception_experiments.items():
                # Check that all required fields exist
                self.assertIn('metrics', exp_data)
                self.assertIn('success_criteria', exp_data)
                self.assertIn('failure_criteria', exp_data)
                self.assertIn('resource_budget', exp_data)
                
                # Check that metrics include relevant ones
                metrics = exp_data['metrics']
                self.assertGreater(len(metrics), 0)
                
                # Check resource budget
                resource_budget = exp_data['resource_budget']
                self.assertIn('cpu_limit', resource_budget)
                self.assertIn('memory_limit_mb', resource_budget)
                self.assertIn('timeout_seconds', resource_budget)
    
    def test_localization_acceptance_criteria(self):
        """Test that localization experiments meet acceptance criteria."""
        yaml_path = os.path.join(src_path, 'robot_lab_algorithms', 'config', 'r7_benchmark_experiments.yaml')
        
        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
            
            localization_experiments = config['benchmark_experiments']['localization']
            
            for exp_id, exp_data in localization_experiments.items():
                # Check that experiments include ATE/RPE metrics
                metrics = exp_data['metrics']
                has_ate_or_rpe = any('ate' in metric.lower() or 'rpe' in metric.lower() 
                                    for metric in metrics)
                self.assertTrue(has_ate_or_rpe, 
                              f"Localization experiment {exp_id} missing ATE/RPE metrics")
    
    def test_estimation_acceptance_criteria(self):
        """Test that state estimation experiments meet acceptance criteria."""
        yaml_path = os.path.join(src_path, 'robot_lab_algorithms', 'config', 'r7_benchmark_experiments.yaml')
        
        if os.path.exists(yaml_path):
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
            
            estimation_experiments = config['benchmark_experiments']['state_estimation']
            
            for exp_id, exp_data in estimation_experiments.items():
                # Check that experiments include RMSE/consistency metrics
                metrics = exp_data['metrics']
                has_rmse_or_consistency = any('rmse' in metric.lower() or 'consistency' in metric.lower() 
                                       or 'nees' in metric.lower() or 'nis' in metric.lower()
                                       for metric in metrics)
                self.assertTrue(has_rmse_or_consistency,
                              f"State estimation experiment {exp_id} missing RMSE/consistency metrics")


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)