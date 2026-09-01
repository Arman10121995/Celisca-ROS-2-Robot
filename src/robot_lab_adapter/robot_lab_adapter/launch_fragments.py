"""
Launch fragment resolution for Robot Lab.

This module provides the ability to resolve selector choices into actual
launch configurations, parameter files, and execution commands.

Each robot/algorithm combination can have its own launch fragment that
specifies how to launch it, what parameters to use, and what dependencies
are required.
"""

import os
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, field

from .selectors import (
    RobotSelector,
    EnvironmentSelector,
    SimulatorSelector,
    PerceptionSelector,
    LocalizationSelector,
    StateEstimationSelector,
    SensorFusionSelector,
    GlobalPlanningSelector,
    LocalPlanningSelector,
    ControlSelector,
    CompositionBuilder
)
from .namespaces import NamespaceConfig, NamespaceManager, get_default_namespace_for_robot, get_default_frame_prefix


@dataclass
class LaunchFragment:
    """Represents a launchable fragment for a robot/algorithm component."""
    
    # Identity
    id: str
    
    # Launch configuration (required)
    package: str
    executable: str
    
    # Optional fields with defaults
    version: str = "1.0.0"
    launch_file: Optional[str] = None  # Alternative to executable
    category: Optional[str] = None
    status: str = "integrated"
    
    # Parameters
    default_params: Dict[str, Any] = field(default_factory=dict)
    param_overlays: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    
    # Dependencies
    requires: List[str] = field(default_factory=list)  # IDs of required fragments
    conflicts: List[str] = field(default_factory=list)  # IDs of conflicting fragments
    
    # Topics and interfaces
    provided_topics: List[str] = field(default_factory=list)
    required_topics: List[str] = field(default_factory=list)
    
    # Robot compatibility
    supported_robots: List[str] = field(default_factory=list)  # Empty means all
    unsupported_robots: List[str] = field(default_factory=list)  # Empty means all
    
    # Namespace and frame prefix support (P2.5)
    # When set, this fragment will be launched in the specified namespace
    namespace: Optional[str] = None
    # Frame prefix for TF frames published by this fragment
    frame_prefix: Optional[str] = None
    # Whether to use robot's default namespace
    use_robot_namespace: bool = True
    # Whether to apply namespace to all topics
    apply_namespace_to_topics: bool = True
    # Whether to apply frame prefix to all frames
    apply_frame_prefix: bool = True
    
    def get_namespaced_params(self, robot_namespace: str, 
                              robot_frame_prefix: str = "") -> Dict[str, Any]:
        """
        Get parameters with namespace and frame prefix applied (P2.5).
        
        Args:
            robot_namespace: The namespace for the robot
            robot_frame_prefix: The frame prefix for the robot
        
        Returns:
            Dictionary of parameters with namespaced values
        """
        from .namespaces import NamespaceConfig, apply_namespace_to_dict
        
        # Determine the effective namespace and frame prefix
        if self.namespace:
            effective_namespace = self.namespace
        elif self.use_robot_namespace:
            effective_namespace = robot_namespace
        else:
            effective_namespace = ""
        
        if self.frame_prefix:
            effective_frame_prefix = self.frame_prefix
        elif self.apply_frame_prefix and robot_frame_prefix:
            effective_frame_prefix = robot_frame_prefix
        else:
            effective_frame_prefix = ""
        
        # Create a namespace config for this fragment
        ns_config = NamespaceConfig(
            name=effective_namespace,
            frame_prefix=effective_frame_prefix
        )
        
        # Apply namespace to all parameters
        params = self.default_params.copy()
        
        # Apply namespace transformation to parameter values
        namespaced_params = apply_namespace_to_dict(
            params, 
            effective_namespace,
            None  # Don't use manager, we have our own config
        )
        
        # Also apply to param_overlays
        namespaced_overlays = {}
        for overlay_name, overlay_params in self.param_overlays.items():
            namespaced_overlays[overlay_name] = apply_namespace_to_dict(
                overlay_params,
                effective_namespace,
                None
            )
        
        # Add namespace-specific parameters to the result (not to the transformed params)
        result = {
            'default_params': namespaced_params,
            'param_overlays': namespaced_overlays,
            'namespace': effective_namespace,
            'frame_prefix': effective_frame_prefix.rstrip('/') if effective_frame_prefix else ''
        }
        
        return result
    
    def get_namespaced_topics(self, robot_namespace: str) -> List[str]:
        """
        Get provided topics with namespace applied (P2.5).
        
        Args:
            robot_namespace: The namespace for the robot
        
        Returns:
            List of namespaced topic names
        """
        if not self.apply_namespace_to_topics:
            return self.provided_topics
        
        ns_config = NamespaceConfig(name=robot_namespace)
        return [ns_config.get_topic(topic) for topic in self.provided_topics]
    
    def get_namespaced_required_topics(self, robot_namespace: str) -> List[str]:
        """
        Get required topics with namespace applied (P2.5).
        
        Args:
            robot_namespace: The namespace for the robot
        
        Returns:
            List of namespaced topic names
        """
        if not self.apply_namespace_to_topics:
            return self.required_topics
        
        ns_config = NamespaceConfig(name=robot_namespace)
        return [ns_config.get_topic(topic) for topic in self.required_topics]


@dataclass
class ParameterOverlay:
    """Represents parameter overrides for a specific configuration."""
    
    name: str
    parameters: Dict[str, Any] = field(default_factory=dict)
    description: str = ""
    
    def apply(self, base_params: Dict[str, Any]) -> Dict[str, Any]:
        """Apply overlay on top of base parameters."""
        result = base_params.copy()
        result.update(self.parameters)
        return result


class LaunchFragmentRegistry:
    """
    Registry of all launch fragments for robots and algorithms.
    
    This class manages the mapping between entity IDs and their
    corresponding launch fragments.
    """
    
    def __init__(self, config_dir: Optional[str] = None):
        self.config_dir = config_dir
        self._fragments: Dict[str, LaunchFragment] = {}
        self._overlays: Dict[str, ParameterOverlay] = {}
        self._robot_launches: Dict[str, str] = {}  # robot_id -> launch fragment ID
        self._algorithm_launches: Dict[str, str] = {}  # algo_id -> launch fragment ID
        
        # Initialize with known mappings
        self._initialize_default_mappings()
    
    def _initialize_default_mappings(self):
        """Initialize default mappings between entities and launch fragments."""
        
        # Robot -> Launch fragment mappings
        # For now, these are placeholders. In P2.4, we'll wrap robot_lab_bringup
        self._robot_launches = {
            'bumperbot': 'bumperbot_simulated',
            'quadrotor_sitl': 'quadrotor_sitl_simulated',
        }
        
        # Algorithm -> Launch fragment mappings
        self._algorithm_launches = {
            # Perception
            'laser_scan_to_pointcloud': 'laser_scan_to_pointcloud_node',
            'costmap_2d_observation': 'costmap_2d_observation_layer',
            'depth_image_to_pointcloud': 'depth_image_proc_node',
            'stereo_image_to_pointcloud': 'stereo_image_proc_node',
            'rgb_d_depth_to_pointcloud': 'rgb_d_pointcloud_node',
            'camera_calibration': 'camera_calibration_node',
            
            # Localization
            'robot_localization_ekf': 'ekf_node',
            'amcl': 'amcl_node',
            'rtabmap_localization': 'rtabmap_ros_node',
            'feature_based_localization': 'feature_localization_node',
            'odometry_motion_model': 'odometry_node',
            
            # State Estimation
            'ekf_localization_node': 'ekf_localization_node',
            'kalman_filter_1d': 'kalman_filter_node',
            
            # Sensor Fusion
            'imu_complementary_filter': 'imu_filter_node',
            'imu_republisher': 'imu_republisher_node',
            
            # Global Planning
            'dijkstra_planner': 'dijkstra_planner_node',
            'a_star_planner': 'a_star_planner_node',
            'navfn_planner': 'navfn_planner_node',
            
            # Local Planning
            'teb_local_planner': 'teb_local_planner_node',
            'dwb_local_planner': 'dwb_local_planner_node',
            'pure_pursuit': 'pure_pursuit_node',
            'simple_controller': 'simple_controller_node',
            'follow_the_gap': 'follow_the_gap_node',
            
            # Control
            'pd_motion_planner': 'pd_motion_planner_node',
            'pid_controller': 'pid_controller_node',
            'noisy_controller': 'noisy_controller_node',
            'twist_relay': 'twist_relay_node',
            'cleaning_controller': 'cleaning_controller_node',
            'map_coverage_controller': 'map_coverage_controller_node',
            'mavros_offboard_controller': 'mavros_offboard_controller_node',
            # P5 breadth algorithms (robot_lab_algorithms package)
            'obstacle_detector': 'obstacle_detector_node',
            'scan_clusterer': 'scan_clusterer_node',
            'pointcloud_segmenter': 'pointcloud_segmenter_node',
            'dead_reckoning': 'dead_reckoning_node',
            'ekf_3d_estimator': 'ekf_3d_estimator_node',
            'motion_model_estimator': 'motion_model_estimator_node',
            'pose_graph_estimator': 'pose_graph_estimator_node',
            'wheel_imu_fusion': 'wheel_imu_fusion_node',
            'gps_odom_fusion': 'gps_odom_fusion_node',
            'complementary_imu': 'complementary_imu_node',
            'rrt_planner': 'rrt_planner_node',
            'voronoi_planner': 'voronoi_planner_node',
        }
    
    def register_fragment(self, fragment: LaunchFragment):
        """Register a launch fragment."""
        self._fragments[fragment.id] = fragment
    
    def register_overlay(self, overlay: ParameterOverlay):
        """Register a parameter overlay."""
        self._overlays[overlay.name] = overlay
    
    def get_fragment(self, fragment_id: str) -> Optional[LaunchFragment]:
        """Get a launch fragment by ID."""
        return self._fragments.get(fragment_id)
    
    def get_launch_for_robot(self, robot_id: str) -> Optional[str]:
        """Get the launch fragment ID for a robot."""
        return self._robot_launches.get(robot_id)
    
    def get_launch_for_algorithm(self, algo_id: str) -> Optional[str]:
        """Get the launch fragment ID for an algorithm."""
        return self._algorithm_launches.get(algo_id)
    
    def get_fragment_for_robot(self, robot_id: str) -> Optional[LaunchFragment]:
        """Get the launch fragment for a robot."""
        fragment_id = self.get_launch_for_robot(robot_id)
        if fragment_id:
            return self.get_fragment(fragment_id)
        return None
    
    def get_fragment_for_algorithm(self, algo_id: str) -> Optional[LaunchFragment]:
        """Get the launch fragment for an algorithm."""
        fragment_id = self.get_launch_for_algorithm(algo_id)
        if fragment_id:
            return self.get_fragment(fragment_id)
        return None


class CompositionResolver:
    """
    Resolve a composition into launch fragments and parameter overlays.
    
    This class takes a Composition (from CompositionBuilder) and resolves it
    into a complete launch configuration with:
    - All required launch fragments
    - Parameter overlays for each component
    - Validation of topic/interface compatibility
    - Rejection of invalid combinations
    """
    
    def __init__(self, config_dir: Optional[str] = None):
        self.config_dir = config_dir
        self.fragment_registry = LaunchFragmentRegistry(config_dir)
        self._initialize_default_fragments()
    
    def _initialize_default_fragments(self):
        """Initialize default launch fragments for known entities."""
        
        # Bumperbot simulated launch
        self.fragment_registry.register_fragment(LaunchFragment(
            id='bumperbot_simulated',
            package='robot_lab_bringup',
            executable='',  # Not used when launch_file is specified
            launch_file='simulated_robot.launch.py',
            default_params={
                'use_sim_time': True,
                'world': 'small_office',
            },
            supported_robots=['bumperbot'],
            provided_topics=[
                '/scan',
                '/imu',
                '/odom',
                '/cmd_vel',
                '/joint_states'
            ],
            required_topics=[],
            status='integrated'
        ))

        # Quadrotor SITL simulated launch
        self.fragment_registry.register_fragment(LaunchFragment(
            id='quadrotor_sitl_simulated',
            package='quadrotor_sitl',
            executable='',  # SITL + mavros bringup
            launch_file='quadrotor_sitl.launch.py',
            default_params={
                'use_sim_time': True,
                'world': 'empty',
            },
            supported_robots=['quadrotor_sitl'],
            provided_topics=[
                '/imu',
                '/gps',
                '/camera/image_raw',
                '/scan',
                '/mavros/setpoint/attitude',
            ],
            required_topics=[],
            status='integrated'
        ))
        
        # Perception fragments
        self.fragment_registry.register_fragment(LaunchFragment(
            id='laser_scan_to_pointcloud_node',
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            default_params={
                'min_height': -0.1,
                'max_height': 0.1,
                'range_min': 0.1,
                'range_max': 100.0
            },
            category='perception',
            provided_topics=['/cloud'],
            required_topics=['/scan'],
            status='integrated'
        ))
        
        self.fragment_registry.register_fragment(LaunchFragment(
            id='costmap_2d_observation_layer',
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            default_params={
                'use_sim_time': True,
            },
            category='perception',
            provided_topics=[],
            required_topics=['/scan', '/odom'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='camera_calibration_node',
            package='camera_calibration',
            executable='cameracalibrator',
            default_params={
                'use_sim_time': True,
            },
            category='perception',
            provided_topics=[],
            required_topics=['/camera/image_raw'],
            status='integrated'
        ))
        
        # Localization fragments
        self.fragment_registry.register_fragment(LaunchFragment(
            id='ekf_node',
            package='robot_localization',
            executable='ekf_node',
            default_params={
                'use_sim_time': True,
                'frequency': 30.0,
                'sensor_timeout': 0.1,
            },
            category='localization',
            provided_topics=['/odometry/filtered'],
            required_topics=['/odom', '/imu'],
            status='integrated'
        ))
        
        self.fragment_registry.register_fragment(LaunchFragment(
            id='amcl_node',
            package='nav2_amcl',
            executable='amcl',
            default_params={
                'use_sim_time': True,
            },
            category='localization',
            provided_topics=['/particle_cloud', '/amcl_pose'],
            required_topics=['/scan', '/odom', '/tf'],
            status='integrated'
        ))
        
        # Control fragments
        self.fragment_registry.register_fragment(LaunchFragment(
            id='simple_controller_node',
            package='robot_lab_controller',
            executable='controller_node',
            default_params={
                'use_sim_time': True,
            },
            category='control',
            provided_topics=['/cmd_vel'],
            required_topics=['/target_velocity'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='mavros_offboard_controller_node',
            package='robot_lab_adapter',
            executable='mavros_offboard_controller_node',
            default_params={
                'use_sim_time': True,
                'command_topic': '/mavros/setpoint/attitude',
                'command_rate_hz': 10.0,
                'default_throttle': 0.0,
                'mode': 'ALT_HOLD',
            },
            category='control',
            provided_topics=['/mavros/setpoint/attitude'],
            required_topics=['/mavros/setpoint/attitude'],
            supported_robots=['quadrotor_sitl'],
            status='integrated'
        ))

        # P5 breadth fragments (robot_lab_algorithms)
        self.fragment_registry.register_fragment(LaunchFragment(
            id='obstacle_detector_node',
            package='robot_lab_algorithms',
            executable='obstacle_detector',
            default_params={
                'use_sim_time': True,
            },
            category='perception',
            provided_topics=['/obstacles'],
            required_topics=['/scan'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='scan_clusterer_node',
            package='robot_lab_algorithms',
            executable='scan_clusterer',
            default_params={
                'use_sim_time': True,
            },
            category='perception',
            provided_topics=['/clusters'],
            required_topics=['/scan'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='pointcloud_segmenter_node',
            package='robot_lab_algorithms',
            executable='pointcloud_segmenter',
            default_params={
                'use_sim_time': True,
            },
            category='perception',
            provided_topics=['/segmented_points'],
            required_topics=['/points'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='dead_reckoning_node',
            package='robot_lab_algorithms',
            executable='dead_reckoning',
            default_params={
                'use_sim_time': True,
            },
            category='localization',
            provided_topics=['/estimated_pose'],
            required_topics=['/odom'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='ekf_3d_estimator_node',
            package='robot_lab_algorithms',
            executable='ekf_3d_estimator',
            default_params={
                'use_sim_time': True,
            },
            category='state_estimation',
            provided_topics=['/estimated_state'],
            required_topics=['/odom'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='motion_model_estimator_node',
            package='robot_lab_algorithms',
            executable='motion_model_estimator',
            default_params={
                'use_sim_time': True,
            },
            category='state_estimation',
            provided_topics=['/estimated_state'],
            required_topics=['/odom'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='pose_graph_estimator_node',
            package='robot_lab_algorithms',
            executable='pose_graph_estimator',
            default_params={
                'use_sim_time': True,
            },
            category='state_estimation',
            provided_topics=['/estimated_state'],
            required_topics=['/odom'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='wheel_imu_fusion_node',
            package='robot_lab_algorithms',
            executable='wheel_imu_fusion',
            default_params={
                'use_sim_time': True,
            },
            category='sensor_fusion',
            provided_topics=['/fused_state'],
            required_topics=['/odom','/imu'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='gps_odom_fusion_node',
            package='robot_lab_algorithms',
            executable='gps_odom_fusion',
            default_params={
                'use_sim_time': True,
            },
            category='sensor_fusion',
            provided_topics=['/fused_pose'],
            required_topics=['/odom','/gps/fix'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='complementary_imu_node',
            package='robot_lab_algorithms',
            executable='complementary_imu',
            default_params={
                'use_sim_time': True,
            },
            category='sensor_fusion',
            provided_topics=['/attitude'],
            required_topics=['/imu'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='rrt_planner_node',
            package='robot_lab_algorithms',
            executable='rrt_planner',
            default_params={
                'use_sim_time': True,
            },
            category='global_planning',
            provided_topics=['/plan'],
            required_topics=['/costmap'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='voronoi_planner_node',
            package='robot_lab_algorithms',
            executable='voronoi_planner',
            default_params={
                'use_sim_time': True,
            },
            category='global_planning',
            provided_topics=['/plan'],
            required_topics=['/costmap'],
            status='integrated'
        ))

        self.fragment_registry.register_fragment(LaunchFragment(
            id='follow_the_gap_node',
            package='robot_lab_algorithms',
            executable='follow_the_gap',
            default_params={
                'use_sim_time': True,
            },
            category='local_planning',
            provided_topics=['/cmd_vel'],
            required_topics=['/scan'],
            status='integrated'
        ))

        # Add parameter overlays
        self.fragment_registry.register_overlay(ParameterOverlay(
            name='bumperbot_defaults',
            parameters={
                'robot_description': 'bumperbot',
                'wheel_left_joint': 'left_wheel_joint',
                'wheel_right_joint': 'right_wheel_joint',
            },
            description='Default parameters for Bumperbot'
        ))
        
        self.fragment_registry.register_overlay(ParameterOverlay(
            name='small_office_world',
            parameters={
                'world': 'small_office',
                'initial_pose_x': 0.0,
                'initial_pose_y': 0.0,
                'initial_pose_theta': 0.0,
            },
            description='Small office world configuration'
        ))

        self.fragment_registry.register_overlay(ParameterOverlay(
            name='empty_world',
            parameters={
                'world': 'empty',
                'initial_pose_x': 0.0,
                'initial_pose_y': 0.0,
                'initial_pose_z': 0.5,
            },
            description='Empty world configuration (SITL quadrotor takeoff height)'
        ))
    
    def resolve(self, composition: 'CompositionBuilder') -> Tuple[bool, Dict[str, Any]]:
        """
        Resolve a composition into launch fragments and parameters.
        
        Args:
            composition: A CompositionBuilder with selected components
        
        Returns:
            Tuple of (success, result_dict) where result_dict contains:
            - fragments: List of LaunchFragment IDs to launch
            - parameters: Combined parameters with overlays applied
            - warnings: List of warning messages
            - errors: List of error messages
            - namespace: The namespace for this composition (P2.5)
            - frame_prefix: The frame prefix for this composition (P2.5)
        """
        result = {
            'fragments': [],
            'parameters': {},
            'warnings': [],
            'errors': [],
            'namespace': '',
            'frame_prefix': '',
            'fragment_configs': {}  # Per-fragment namespace/params (P2.5)
        }
        
        # Get the composition
        comp = composition.build()
        
        # Get namespace and frame prefix from composition (P2.5)
        namespace = getattr(comp, 'namespace', '')
        frame_prefix = getattr(comp, 'frame_prefix', '')
        enable_namespace = getattr(comp, 'enable_namespace', True)
        
        result['namespace'] = namespace
        result['frame_prefix'] = frame_prefix
        
        # Validate robot
        robot_id = comp.robot_id
        if not robot_id:
            result['errors'].append('No robot selected')
            return False, result
        
        # Get robot launch fragment
        robot_fragment_id = self.fragment_registry.get_launch_for_robot(robot_id)
        if not robot_fragment_id:
            result['warnings'].append(f'No launch fragment defined for robot "{robot_id}". Using default.')
            # For now, we'll still proceed with a default
            robot_fragment_id = 'bumperbot_simulated'  # Fallback
        
        result['fragments'].append(robot_fragment_id)
        
        # Get robot fragment for namespace configuration
        robot_fragment = self.fragment_registry.get_fragment(robot_fragment_id)
        
        # Add robot-specific parameters with namespace applied (P2.5)
        robot_overlay_name = f'{robot_id}_defaults'
        robot_overlay = self.fragment_registry._overlays.get(robot_overlay_name)
        if robot_overlay:
            # Apply namespace to overlay parameters
            if enable_namespace and namespace:
                from .namespaces import apply_namespace_to_dict, NamespaceConfig
                ns_config = NamespaceConfig(name=namespace, frame_prefix=frame_prefix)
                overlay_params = {}
                for key, value in robot_overlay.parameters.items():
                    if isinstance(value, str):
                        # Apply namespace transformation
                        if any(p in key.lower() for p in ['topic', 'frame', 'tf', 'cmd', 'scan', 'odom']):
                            if 'frame' in key.lower() or 'tf' in key.lower():
                                overlay_params[key] = ns_config.get_frame(value)
                            else:
                                overlay_params[key] = ns_config.get_topic(value)
                        else:
                            overlay_params[key] = value
                    else:
                        overlay_params[key] = value
                result['parameters'].update(overlay_params)
            else:
                result['parameters'].update(robot_overlay.parameters)
        
        # Add environment-specific parameters
        env_id = comp.environment_id
        if env_id:
            env_overlay_name = f'{env_id}_world'
            env_overlay = self.fragment_registry._overlays.get(env_overlay_name)
            if env_overlay:
                result['parameters'].update(env_overlay.parameters)
            else:
                result['warnings'].append(f'No parameter overlay for environment "{env_id}"')
        
        # Add simulator parameter
        simulator = comp.simulator or 'gazebo'
        result['parameters']['simulator'] = simulator
        result['parameters']['use_sim_time'] = True
        
        # Add namespace parameters (P2.5)
        if enable_namespace:
            if namespace:
                result['parameters']['namespace'] = namespace
            if frame_prefix:
                result['parameters']['frame_prefix'] = frame_prefix.rstrip('/')
        
        # Process algorithms
        for category, algo_id in comp.algorithm_ids.items():
            if algo_id:
                fragment_id = self.fragment_registry.get_launch_for_algorithm(algo_id)
                if fragment_id:
                    result['fragments'].append(fragment_id)
                    
                    # Get fragment and add its default params with namespace (P2.5)
                    fragment = self.fragment_registry.get_fragment(fragment_id)
                    if fragment:
                        # Store per-fragment namespace config
                        fragment_namespace = fragment.namespace or namespace
                        fragment_frame_prefix = fragment.frame_prefix or frame_prefix
                        
                        result['fragment_configs'][fragment_id] = {
                            'namespace': fragment_namespace,
                            'frame_prefix': fragment_frame_prefix,
                            'use_robot_namespace': fragment.use_robot_namespace
                        }
                        
                        # Apply namespace to fragment params
                        if enable_namespace and (namespace or fragment_namespace):
                            from .namespaces import apply_namespace_to_dict, NamespaceConfig
                            effective_ns = fragment_namespace or namespace
                            effective_frame = fragment_frame_prefix or frame_prefix
                            ns_config = NamespaceConfig(name=effective_ns, frame_prefix=effective_frame)
                            
                            namespaced_params = {}
                            for key, value in fragment.default_params.items():
                                if isinstance(value, str):
                                    if any(p in key.lower() for p in ['topic', 'cmd', 'scan', 'odom', 'imu', 'laser']):
                                        namespaced_params[key] = ns_config.get_topic(value)
                                    elif any(p in key.lower() for p in ['frame', 'tf', 'base_link']):
                                        namespaced_params[key] = ns_config.get_frame(value)
                                    else:
                                        namespaced_params[key] = value
                                else:
                                    namespaced_params[key] = value
                            result['parameters'].update(namespaced_params)
                        else:
                            result['parameters'].update(fragment.default_params)
                        
                        # Check for required topics with namespace (P2.5)
                        for req_topic in fragment.required_topics:
                            # Check if any fragment provides this topic (with namespace)
                            provides = False
                            for frag_id in result['fragments']:
                                frag = self.fragment_registry.get_fragment(frag_id)
                                if frag:
                                    # Check both original and namespaced topic names
                                    if req_topic in frag.provided_topics:
                                        provides = True
                                        break
                                    # Also check if namespaced version is provided
                                    ns_config = NamespaceConfig(name=namespace)
                                    namespaced_req = ns_config.get_topic(req_topic)
                                    if namespaced_req in frag.get_namespaced_topics(namespace):
                                        provides = True
                                        break
                            
                            if not provides:
                                result['warnings'].append(
                                    f'Required topic "{req_topic}" for algorithm "{algo_id}" '
                                    f'not provided by any fragment'
                                )
                else:
                    result['warnings'].append(
                        f'No launch fragment defined for algorithm "{algo_id}" (category: {category})'
                    )
        
        # Check for conflicts
        all_fragments = [self.fragment_registry.get_fragment(fid) for fid in result['fragments']]
        all_fragments = [f for f in all_fragments if f]
        
        for fragment in all_fragments:
            for conflict_id in fragment.conflicts:
                if conflict_id in result['fragments']:
                    result['errors'].append(
                        f'Conflict: fragment "{fragment.id}" conflicts with "{conflict_id}"'
                    )
        
        # Check robot support
        for fragment in all_fragments:
            if fragment.supported_robots and robot_id not in fragment.supported_robots:
                result['errors'].append(
                    f'Fragment "{fragment.id}" does not support robot "{robot_id}"'
                )
            if robot_id in fragment.unsupported_robots:
                result['errors'].append(
                    f'Fragment "{fragment.id}" explicitly does not support robot "{robot_id}"'
                )
        
        has_errors = len(result['errors']) > 0
        return not has_errors, result
    
    def generate_launch_description(self, composition: 'CompositionBuilder'):
        """
        Generate a ROS 2 launch description from a composition.
        
        This is a placeholder that will be expanded in P2.4.
        For now, it returns the resolved fragments and parameters.
        
        Args:
            composition: A CompositionBuilder with selected components
        
        Returns:
            Dictionary with launch configuration
        """
        success, result = self.resolve(composition)
        
        if not success:
            return {'success': False, 'errors': result['errors']}
        
        # For now, return the resolution result
        # In P2.4, this will generate actual LaunchDescription objects
        return {
            'success': True,
            'fragments': result['fragments'],
            'parameters': result['parameters'],
            'warnings': result['warnings']
        }
    
    def validate_combination(self, robot_id: str, algo_ids: Dict[str, str]) -> Tuple[bool, List[str]]:
        """
        Validate that a robot/algorithm combination is valid before launching.
        
        This checks:
        - Robot support for each algorithm
        - Topic compatibility
        - Fragment conflicts
        
        Args:
            robot_id: The robot ID
            algo_ids: Dictionary of category -> algorithm ID
        
        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []
        
        # Get robot info
        robot_selector = RobotSelector(self.config_dir)
        robot = robot_selector.select(robot_id)
        
        if not robot:
            errors.append(f'Unknown robot: {robot_id}')
            return False, errors
        
        robot_class = robot.get('robot_class', 'unknown')
        
        # Check each algorithm
        algo_selector = {
            'perception': PerceptionSelector(self.config_dir),
            'localization': LocalizationSelector(self.config_dir),
            'state_estimation': StateEstimationSelector(self.config_dir),
            'sensor_fusion': SensorFusionSelector(self.config_dir),
            'global_planning': GlobalPlanningSelector(self.config_dir),
            'local_planning': LocalPlanningSelector(self.config_dir),
            'control': ControlSelector(self.config_dir),
        }
        
        for category, algo_id in algo_ids.items():
            if not algo_id:
                continue
            
            selector = algo_selector.get(category)
            if not selector:
                continue
            
            algo = selector.select(algo_id)
            if not algo:
                errors.append(f'Unknown {category} algorithm: {algo_id}')
                continue
            
            # Check robot class support
            supported_classes = algo.get('supported_robot_classes', [])
            if supported_classes and robot_class not in supported_classes:
                errors.append(
                    f'Algorithm "{algo_id}" ({category}) does not support '
                    f'robot class "{robot_class}"'
                )
        
        return len(errors) == 0, errors
