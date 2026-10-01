#!/usr/bin/env python3
"""
R7 Benchmark Framework

Comprehensive framework for benchmarking R7.2-R7.8 algorithm categories with:
- Method subrecords with mathematical foundations
- Input strata definitions
- Metric specifications
- Benchmark experiment configurations
- Reproducible test fixtures
"""

import yaml
import numpy as np
import json
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Tuple
from enum import Enum
import hashlib
import datetime


class InputStratum(Enum):
    """Input data stratification for fair algorithm comparison."""
    # Perception strata
    PERCEPTION_LIDAR_2D_SPARSE = "perception_lidar_2d_sparse"
    PERCEPTION_LIDAR_2D_DENSE = "perception_lidar_2d_dense"
    PERCEPTION_LIDAR_3D = "perception_lidar_3d"
    PERCEPTION_RGBD = "perception_rgbd"
    PERCEPTION_STEREO = "perception_stereo"
    
    # Localization strata
    LOCALIZATION_ODOMETRY_ONLY = "localization_odometry_only"
    LOCALIZATION_LIDAR_MAP = "localization_lidar_map"
    LOCALIZATION_VISUAL_FEATURES = "localization_visual_features"
    LOCALIZATION_MULTI_SENSOR = "localization_multi_sensor"
    
    # State estimation strata
    STATE_ESTIMATION_LINEAR_DYNAMICS = "state_estimation_linear_dynamics"
    STATE_ESTIMATION_NONLINEAR_DYNAMICS = "state_estimation_nonlinear_dynamics"
    STATE_ESTIMATION_HIGH_FREQUENCY = "state_estimation_high_frequency"
    STATE_ESTIMATION_LOW_FREQUENCY = "state_estimation_low_frequency"
    
    # Sensor fusion strata
    SENSOR_FUSION_ATTITUDE_ONLY = "sensor_fusion_attitude_only"
    SENSOR_FUSION_POSE = "sensor_fusion_pose"
    SENSOR_FUSION_MULTI_SENSOR = "sensor_fusion_multi_sensor"
    
    # Planning strata
    PLANNING_2D_GRID = "planning_2d_grid"
    PLANNING_2D_CONTINUOUS = "planning_2d_continuous"
    PLANNING_3D = "planning_3d"
    PLANNING_HIGH_DIMENSIONAL = "planning_high_dimensional"
    
    # Control strata
    CONTROL_LINEAR_PLANT = "control_linear_plant"
    CONTROL_NONLINEAR_PLANT = "control_nonlinear_plant"
    CONTROL_CONSTRAINED = "control_constrained"
    CONTROL_UNCONSTRAINED = "control_unconstrained"


class MetricType(Enum):
    """Benchmark metric types."""
    ACCURACY = "accuracy"
    PRECISION = "precision"
    RECALL = "recall"
    IoU = "intersection_over_union"
    RMSE = "root_mean_square_error"
    ATE = "absolute_trajectory_error"
    RPE = "relative_pose_error"
    COMPUTE_TIME = "compute_time"
    REAL_TIME_FACTOR = "real_time_factor"
    MEMORY_USAGE = "memory_usage"
    SUCCESS_RATE = "success_rate"
    COLLISION_RATE = "collision_rate"
    PATH_COST = "path_cost"
    PATH_LENGTH = "path_length"
    SMOOTHNESS = "smoothness"
    CONVERGENCE_TIME = "convergence_time"
    DIVERGENCE_RATE = "divergence_rate"
    NEES = "normalized_estimation_error_squared"
    NIS = "normalized_innovation_squared"
    CONSISTENCY = "consistency"
    ROBUSTNESS = "robustness"
    LATENCY = "latency"


@dataclass
class MethodSubrecord:
    """Subrecord for a specific algorithm method."""
    method_id: str
    category: str
    display_name: str
    implementation_ref: str  # module.class or package.executable
    mathematical_foundation: str  # equations, source references
    equations: List[str] = field(default_factory=list)
    source_references: List[str] = field(default_factory=list)
    applicable_robots: List[str] = field(default_factory=list)
    input_strata: List[InputStratum] = field(default_factory=list)
    output_types: List[str] = field(default_factory=list)
    parameters: Dict[str, Any] = field(default_factory=dict)
    assumptions: List[str] = field(default_factory=list)
    limitations: List[str] = field(default_factory=list)
    computational_complexity: str = "unknown"
    memory_requirements: str = "unknown"
    
    def to_dict(self) -> Dict:
        """Convert to dictionary for YAML serialization."""
        return {
            'method_id': self.method_id,
            'category': self.category,
            'display_name': self.display_name,
            'implementation_ref': self.implementation_ref,
            'mathematical_foundation': self.mathematical_foundation,
            'equations': self.equations,
            'source_references': self.source_references,
            'applicable_robots': self.applicable_robots,
            'input_strata': [str(s) for s in self.input_strata],
            'output_types': self.output_types,
            'parameters': self.parameters,
            'assumptions': self.assumptions,
            'limitations': self.limitations,
            'computational_complexity': self.computational_complexity,
            'memory_requirements': self.memory_requirements
        }


@dataclass
class BenchmarkExperiment:
    """Benchmark experiment configuration."""
    experiment_id: str
    method_id: str
    category: str
    input_stratum: InputStratum
    robot_class: str = "mobile"
    environment: str = "nav_empty"
    simulator: str = "gazebo"
    seed: int = 42
    duration_seconds: float = 10.0
    metrics: List[MetricType] = field(default_factory=list)
    success_criteria: Dict[str, Any] = field(default_factory=dict)
    failure_criteria: Dict[str, Any] = field(default_factory=dict)
    resource_budget: Dict[str, float] = field(default_factory=dict)
    
    def to_dict(self) -> Dict:
        return {
            'experiment_id': self.experiment_id,
            'method_id': self.method_id,
            'category': self.category,
            'input_stratum': str(self.input_stratum),
            'robot_class': self.robot_class,
            'environment': self.environment,
            'simulator': self.simulator,
            'seed': self.seed,
            'duration_seconds': self.duration_seconds,
            'metrics': [str(m) for m in self.metrics],
            'success_criteria': self.success_criteria,
            'failure_criteria': self.failure_criteria,
            'resource_budget': self.resource_budget
        }


@dataclass 
class BenchmarkResult:
    """Results from a benchmark experiment."""
    experiment_id: str
    method_id: str
    category: str
    timestamp: str = field(default_factory=lambda: datetime.datetime.now().isoformat())
    metrics: Dict[str, float] = field(default_factory=dict)
    success: bool = False
    failure_reason: Optional[str] = None
    artifacts: Dict[str, str] = field(default_factory=dict)
    provenance: Dict[str, Any] = field(default_factory=dict)
    
    def to_dict(self) -> Dict:
        return {
            'experiment_id': self.experiment_id,
            'method_id': self.method_id,
            'category': self.category,
            'timestamp': self.timestamp,
            'metrics': self.metrics,
            'success': self.success,
            'failure_reason': self.failure_reason,
            'artifacts': self.artifacts,
            'provenance': self.provenance
        }


@dataclass
class InputStratumDefinition:
    """Definition of an input stratum for benchmarking."""
    stratum_id: str
    category: str
    description: str
    sensor_types: List[str] = field(default_factory=list)
    data_frequency_hz: float = 30.0
    noise_characteristics: Dict[str, Any] = field(default_factory=dict)
    typical_environments: List[str] = field(default_factory=list)
    representative_robots: List[str] = field(default_factory=list)
    
    def to_dict(self) -> Dict:
        return {
            'stratum_id': self.stratum_id,
            'category': self.category,
            'description': self.description,
            'sensor_types': self.sensor_types,
            'data_frequency_hz': self.data_frequency_hz,
            'noise_characteristics': self.noise_characteristics,
            'typical_environments': self.typical_environments,
            'representative_robots': self.representative_robots
        }


class R7BenchmarkFramework:
    """Main benchmark framework for R7 algorithm categories."""
    
    def __init__(self):
        self.methods: Dict[str, MethodSubrecord] = {}
        self.experiments: Dict[str, BenchmarkExperiment] = {}
        self.results: Dict[str, BenchmarkResult] = {}
        self.input_strata: Dict[str, InputStratumDefinition] = {}
        
    def register_method(self, method: MethodSubrecord) -> None:
        """Register a method subrecord."""
        key = f"{method.category}:{method.method_id}"
        self.methods[key] = method
        
    def register_experiment(self, experiment: BenchmarkExperiment) -> None:
        """Register a benchmark experiment."""
        self.experiments[experiment.experiment_id] = experiment
        
    def register_result(self, result: BenchmarkResult) -> None:
        """Register a benchmark result."""
        self.results[result.experiment_id] = result
        
    def register_input_stratum(self, stratum: InputStratumDefinition) -> None:
        """Register an input stratum definition."""
        self.input_strata[stratum.stratum_id] = stratum
        
    def get_methods_by_category(self, category: str) -> List[MethodSubrecord]:
        """Get all methods for a specific category."""
        return [method for key, method in self.methods.items() if method.category == category]
        
    def get_experiments_by_category(self, category: str) -> List[BenchmarkExperiment]:
        """Get all experiments for a specific category."""
        return [exp for exp in self.experiments.values() if exp.category == category]
        
    def generate_comparison_report(self, category: str) -> Dict:
        """Generate a comparison report for methods in a category."""
        methods = self.get_methods_by_category(category)
        experiments = self.get_experiments_by_category(category)
        results = [r for r in self.results.values() if r.category == category]
        
        return {
            'category': category,
            'method_count': len(methods),
            'experiment_count': len(experiments),
            'result_count': len(results),
            'methods': [m.to_dict() for m in methods],
            'experiments': [e.to_dict() for e in experiments],
            'results': [r.to_dict() for r in results]
        }
        
    def save_framework_state(self, filename: str) -> None:
        """Save the entire framework state to a file."""
        state = {
            'methods': {k: v.to_dict() for k, v in self.methods.items()},
            'experiments': {k: v.to_dict() for k, v in self.experiments.items()},
            'results': {k: v.to_dict() for k, v in self.results.items()},
            'input_strata': {k: v.to_dict() for k, v in self.input_strata.items()}
        }
        
        with open(filename, 'w') as f:
            yaml.dump(state, f, default_flow_style=False, sort_keys=False)
            
    def load_framework_state(self, filename: str) -> None:
        """Load framework state from a file."""
        with open(filename, 'r') as f:
            state = yaml.safe_load(f)
        
        for key, method_dict in state.get('methods', {}).items():
            method_dict['input_strata'] = [InputStratum(s) for s in method_dict.get('input_strata', [])]
            method = MethodSubrecord(**method_dict)
            self.methods[key] = method
            
        for key, exp_dict in state.get('experiments', {}).items():
            exp_dict['input_stratum'] = InputStratum(exp_dict['input_stratum'])
            exp_dict['metrics'] = [MetricType(m) for m in exp_dict.get('metrics', [])]
            exp = BenchmarkExperiment(**exp_dict)
            self.experiments[key] = exp
            
        for key, result_dict in state.get('results', {}).items():
            result = BenchmarkResult(**result_dict)
            self.results[key] = result
            
        for key, stratum_dict in state.get('input_strata', {}).items():
            stratum = InputStratumDefinition(**stratum_dict)
            self.input_strata[key] = stratum


# Pre-defined input strata for each category
def create_perception_strata() -> List[InputStratumDefinition]:
    """Create perception input strata definitions."""
    return [
        InputStratumDefinition(
            stratum_id=str(InputStratum.PERCEPTION_LIDAR_2D_SPARSE),
            category="perception",
            description="2D LiDAR with sparse point clouds (100-500 points per scan)",
            sensor_types=["laser_scan"],
            data_frequency_hz=10.0,
            noise_characteristics={"range_noise_std": 0.02, "angular_resolution": 0.5},
            typical_environments=["nav_empty", "small_office"],
            representative_robots=["bumperbot", "labbot"]
        ),
        InputStratumDefinition(
            stratum_id=str(InputStratum.PERCEPTION_LIDAR_2D_DENSE),
            category="perception", 
            description="2D LiDAR with dense point clouds (1000-5000 points per scan)",
            sensor_types=["laser_scan"],
            data_frequency_hz=20.0,
            noise_characteristics={"range_noise_std": 0.01, "angular_resolution": 0.1},
            typical_environments=["small_warehouse", "nav_maze"],
            representative_robots=["bumperbot", "labbot"]
        ),
        InputStratumDefinition(
            stratum_id=str(InputStratum.PERCEPTION_RGBD),
            category="perception",
            description="RGB-D camera data with depth and color information",
            sensor_types=["rgb_image", "depth_image", "camera_info"],
            data_frequency_hz=30.0,
            noise_characteristics={"depth_noise_std": 0.005, "color_noise": "gaussian"},
            typical_environments=["small_office", "nav_dynamic"],
            representative_robots=["bumperbot"]
        )
    ]


def create_localization_strata() -> List[InputStratumDefinition]:
    """Create localization input strata definitions."""
    return [
        InputStratumDefinition(
            stratum_id=str(InputStratum.LOCALIZATION_ODOMETRY_ONLY),
            category="localization",
            description="Wheel odometry only localization",
            sensor_types=["odom"],
            data_frequency_hz=50.0,
            noise_characteristics={"position_noise_std": 0.05, "orientation_noise_std": 0.01},
            typical_environments=["nav_empty"],
            representative_robots=["bumperbot", "labbot"]
        ),
        InputStratumDefinition(
            stratum_id=str(InputStratum.LOCALIZATION_LIDAR_MAP),
            category="localization",
            description="LiDAR-based localization with known map",
            sensor_types=["laser_scan", "map"],
            data_frequency_hz=10.0,
            noise_characteristics={"scan_noise_std": 0.02, "map_resolution": 0.05},
            typical_environments=["small_office", "small_warehouse"],
            representative_robots=["bumperbot", "labbot"]
        )
    ]


def create_state_estimation_strata() -> List[InputStratumDefinition]:
    """Create state estimation input strata definitions."""
    return [
        InputStratumDefinition(
            stratum_id=str(InputStratum.STATE_ESTIMATION_LINEAR_DYNAMICS),
            category="state_estimation",
            description="Linear dynamics models with Gaussian noise",
            sensor_types=["imu", "odom"],
            data_frequency_hz=50.0,
            noise_characteristics={"process_noise": 0.01, "measurement_noise": 0.1},
            typical_environments=["nav_empty"],
            representative_robots=["bumperbot", "labbot"]
        ),
        InputStratumDefinition(
            stratum_id=str(InputStratum.STATE_ESTIMATION_NONLINEAR_DYNAMICS),
            category="state_estimation",
            description="Nonlinear dynamics with significant state-dependent noise",
            sensor_types=["imu", "odom", "vo"],
            data_frequency_hz=30.0,
            noise_characteristics={"process_noise": 0.1, "measurement_noise": 0.2},
            typical_environments=["small_office"],
            representative_robots=["bumperbot", "labbot"]
        )
    ]


def create_sensor_fusion_strata() -> List[InputStratumDefinition]:
    """Create sensor fusion input strata definitions."""
    return [
        InputStratumDefinition(
            stratum_id=str(InputStratum.SENSOR_FUSION_ATTITUDE_ONLY),
            category="sensor_fusion",
            description="Attitude estimation from IMU sensors only",
            sensor_types=["imu"],
            data_frequency_hz=100.0,
            noise_characteristics={"gyro_noise_std": 0.001, "accel_noise_std": 0.01},
            typical_environments=["nav_empty"],
            representative_robots=["bumperbot", "labbot", "go2"]
        ),
        InputStratumDefinition(
            stratum_id=str(InputStratum.SENSOR_FUSION_POSE),
            category="sensor_fusion",
            description="Pose estimation from wheel odometry and IMU",
            sensor_types=["odom", "imu"],
            data_frequency_hz=50.0,
            noise_characteristics={"wheel_odometry_noise": 0.05, "imu_noise": 0.01},
            typical_environments=["small_office"],
            representative_robots=["bumperbot", "labbot"]
        )
    ]


def create_planning_strata() -> List[InputStratumDefinition]:
    """Create planning input strata definitions."""
    return [
        InputStratumDefinition(
            stratum_id=str(InputStratum.PLANNING_2D_GRID),
            category="global_planning",
            description="2D grid-based planning with discrete costs",
            sensor_types=["map", "goal_pose"],
            data_frequency_hz=1.0,
            noise_characteristics={"map_resolution": 0.05, "cost_uncertainty": 0.1},
            typical_environments=["small_office", "nav_maze", "small_warehouse"],
            representative_robots=["bumperbot", "labbot"]
        ),
        InputStratumDefinition(
            stratum_id=str(InputStratum.PLANNING_2D_CONTINUOUS),
            category="global_planning",
            description="2D continuous space planning with sampling",
            sensor_types=["map", "goal_pose"],
            data_frequency_hz=1.0,
            noise_characteristics={"planning_time_limit": 5.0, "sample_count": 1000},
            typical_environments=["nav_empty", "small_office"],
            representative_robots=["bumperbot", "labbot"]
        )
    ]


def create_control_strata() -> List[InputStratumDefinition]:
    """Create control input strata definitions."""
    return [
        InputStratumDefinition(
            stratum_id=str(InputStratum.CONTROL_LINEAR_PLANT),
            category="control",
            description="Linear plant models with box constraints",
            sensor_types=["odom", "cmd_vel"],
            data_frequency_hz=50.0,
            noise_characteristics={"actuator_noise_std": 0.01, "latency": 0.02},
            typical_environments=["nav_empty"],
            representative_robots=["bumperbot", "labbot"]
        ),
        InputStratumDefinition(
            stratum_id=str(InputStratum.CONTROL_NONLINEAR_PLANT),
            category="control",
            description="Nonlinear plant with saturation and dead zones",
            sensor_types=["joint_states", "odom"],
            data_frequency_hz=100.0,
            noise_characteristics={"nonlinearity": "sinusoidal", "saturation_limit": 1.0},
            typical_environments=["small_office"],
            representative_robots=["go2", "berkeley_humanoid_lite"]
        )
    ]


# Pre-defined method subrecords for each category

def create_perception_methods() -> List[MethodSubrecord]:
    """Create method subrecords for perception algorithms."""
    return [
        MethodSubrecord(
            method_id="obstacle_detector",
            category="perception",
            display_name="Laser Scan Obstacle Detector",
            implementation_ref="robot_lab_algorithms.obstacle_detector",
            mathematical_foundation="Ray-based obstacle detection with range thresholding",
            equations=[
                "obstacle_range = min(range_measurements)",
                "obstacle_present = (obstacle_range < threshold)",
                "obstacle_angle = argmin(range_measurements)"
            ],
            source_references=[
                "ROS LaserScan message documentation",
                "Standard robotics obstacle detection methods"
            ],
            applicable_robots=["bumperbot", "labbot"],
            input_strata=[InputStratum.PERCEPTION_LIDAR_2D_SPARSE, InputStratum.PERCEPTION_LIDAR_2D_DENSE],
            output_types=["MarkerArray", "PointCloud2"],
            parameters={"range_threshold": 1.0, "min_height": 0.1, "max_height": 0.5},
            assumptions=["2D LiDAR in horizontal plane", "Static obstacles"],
            limitations=["Cannot detect overhanging obstacles", "Limited to 2D detection"],
            computational_complexity="O(n)",
            memory_requirements="Low"
        ),
        MethodSubrecord(
            method_id="scan_clusterer", 
            category="perception",
            display_name="Scan Clustering Obstacle Detector",
            implementation_ref="robot_lab_algorithms.scan_clusterer",
            mathematical_foundation="Connected components clustering on range data",
            equations=[
                "cluster_formation: points grouped by proximity threshold",
                "bounding_box: [min_x, max_x, min_y, max_y] per cluster"
            ],
            source_references=[
                "Probabilistic Robotics - Thrun, Burgard, Fox",
                "Connected component labeling algorithms"
            ],
            applicable_robots=["bumperbot", "labbot"],
            input_strata=[InputStratum.PERCEPTION_LIDAR_2D_SPARSE, InputStratum.PERCEPTION_LIDAR_2D_DENSE],
            output_types=["MarkerArray", "PointCloud2"],
            parameters={"cluster_threshold": 0.2, "min_cluster_size": 5},
            assumptions=["Obstacles are contiguous in scan", "Flat ground"],
            limitations=["Fails with sparse point clouds", "Sensitive to threshold"],
            computational_complexity="O(n log n)",
            memory_requirements="Medium"
        ),
        MethodSubrecord(
            method_id="euclidean_clusterer",
            category="perception", 
            display_name="Euclidean Distance Clustering",
            implementation_ref="robot_lab_algorithms.euclidean_clusterer",
            mathematical_foundation="Distance-based clustering using Euclidean metrics",
            equations=[
                "distance = sqrt((x2-x1)^2 + (y2-y1)^2)",
                "cluster_merge = distance < threshold"
            ],
            source_references=[
                "SciKit-learn DBSCAN implementation",
                "Euclidean space clustering methods"
            ],
            applicable_robots=["bumperbot", "labbot"],
            input_strata=[InputStratum.PERCEPTION_LIDAR_2D_DENSE],
            output_types=["MarkerArray", "PointCloud2"],
            parameters={"cluster_radius": 0.3, "min_points_per_cluster": 3},
            assumptions=["3D point cloud input", "Uniform point density"],
            limitations=["Computationally expensive for large clouds", "Sensitive to parameter tuning"],
            computational_complexity="O(n^2)",
            memory_requirements="High"
        ),
        MethodSubrecord(
            method_id="dbscan_clusterer",
            category="perception",
            display_name="DBSCAN Density-Based Clustering",
            implementation_ref="robot_lab_algorithms.dbscan_clusterer", 
            mathematical_foundation="Density-based clustering with noise classification",
            equations=[
                "core_point: |N_eps(p)| >= min_samples",
                "border_point: |N_eps(p)| < min_samples but reachable from core",
                "noise: points not core or border"
            ],
            source_references=[
                "Ester et al. - DBSCAN: A Density-based algorithm for Discovering Clusters",
                "Scikit-learn clustering documentation"
            ],
            applicable_robots=["bumperbot", "labbot"],
            input_strata=[InputStratum.PERCEPTION_LIDAR_2D_DENSE, InputStratum.PERCEPTION_LIDAR_3D],
            output_types=["MarkerArray", "PointCloud2"],
            parameters={"eps": 0.2, "min_samples": 5},
            assumptions=["Clusters have similar density", "Noise points are distinguishable"],
            limitations=["Fails with varying density clusters", "Parameter sensitive"],
            computational_complexity="O(n log n)",
            memory_requirements="Medium"
        ),
        MethodSubrecord(
            method_id="ransac_ground_removal",
            category="perception",
            display_name="RANSAC Ground Plane Removal",
            implementation_ref="robot_lab_algorithms.ransac_ground_removal",
            mathematical_foundation="RANdom SAmple Consensus for plane fitting and removal",
            equations=[
                "plane_model: ax + by + cz + d = 0",
                "inlier_threshold: |ax_i + by_i + cz_i + d| < threshold",
                "iterations: log(1-p)/log(1-w^s) where p=0.99, w=inlier_ratio, s=samples"
            ],
            source_references=[
                "Fischler and Bolles - Random Sample Consensus",
                "PCL RANSAC implementation"
            ],
            applicable_robots=["bumperbot"],
            input_strata=[InputStratum.PERCEPTION_LIDAR_3D, InputStratum.PERCEPTION_RGBD],
            output_types=["PointCloud2", "PointCloud2"],
            parameters={"max_iterations": 1000, "distance_threshold": 0.05, "min_inliers": 100},
            assumptions=["Ground plane is dominant feature", "Points are in 3D space"],
            limitations=["Computationally intensive", "May fail with non-planar surfaces"],
            computational_complexity="O(n*k)",
            memory_requirements="High"
        ),
        MethodSubrecord(
            method_id="pointcloud_segmenter",
            category="perception",
            display_name="Point Cloud Segmentation",
            implementation_ref="robot_lab_algorithms.pointcloud_segmenter",
            mathematical_foundation="Multi-plane segmentation with region growing",
            equations=[
                "normal_estimation: covariance matrix analysis",
                "region_growing: adjacent points with similar normals"
            ],
            source_references=[
                "PCL segmentation algorithms",
                "3D computer vision literature"
            ],
            applicable_robots=["bumperbot"],
            input_strata=[InputStratum.PERCEPTION_LIDAR_3D, InputStratum.PERCEPTION_RGBD],
            output_types=["PointCloud2", "MarkerArray"],
            parameters={"min_cluster_size": 100, "smoothness_threshold": 0.1, "curvature_threshold": 0.05},
            assumptions=["Organized point cloud", "Multiple planar surfaces present"],
            limitations=["Requires good normal estimation", "Sensitive to parameter tuning"],
            computational_complexity="O(n^2)",
            memory_requirements="High"
        )
    ]


if __name__ == "__main__":
    # Example usage
    framework = R7BenchmarkFramework()
    
    # Add perception methods
    perception_methods = create_perception_methods()
    for method in perception_methods:
        framework.register_method(method)
    
    # Add perception strata
    perception_strata = create_perception_strata()
    for stratum in perception_strata:
        framework.register_input_stratum(stratum)
    
    print(f"Registered {len(framework.methods)} methods")
    print(f"Registered {len(framework.input_strata)} input strata")
    
    # Save framework state
    framework.save_framework_state("r7_benchmark_framework.yaml")
    print("Framework state saved to r7_benchmark_framework.yaml")