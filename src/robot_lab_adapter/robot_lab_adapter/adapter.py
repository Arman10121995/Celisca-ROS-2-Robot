"""
Adapter module for wrapping legacy bringup packages.

This module provides the P2.4 functionality: wrapping robot_lab_bringup/simulated_robot.launch.py
as the first adapter and retaining its public arguments until a documented deprecation.

The adapter translates between robot_lab selector choices and legacy configuration.
"""

import os
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from .selectors import CompositionBuilder
from .launch_fragments import CompositionResolver


class LegacyAdapter:
    """
    Adapter for legacy robot bringup packages.
    
    This class wraps existing robot bringup packages and translates
    robot_lab selector choices into legacy configuration format.
    """
    
    def __init__(self):
        self._mappings = {
            'robot_id': {
                'bumperbot': {
                    'robot_model': 'bumperbot',
                    'robot_package': 'robot_lab_robots',
                    'robot_xacro': 'bumperbot/urdf/bumperbot.urdf.xacro',
                    'robot_name': 'bumperbot',
                }
            },
            'environment_id': {
                'small_office': {
                    'map_name': 'small_office',
                    'world_name': 'small_office',
                    'world_package': 'maps',
                },
                'small_warehouse': {
                    'map_name': 'small_warehouse',
                    'world_name': 'small_warehouse',
                    'world_package': 'maps',
                },
                'small_house': {
                    'map_name': 'small_house',
                    'world_name': 'small_house',
                    'world_package': 'maps',
                },
                'warehouse_demo': {
                    'map_name': 'warehouse_demo',
                    'world_name': 'warehouse_demo',
                    'world_package': 'maps',
                },
                'celisca_floor_1': {
                    'map_name': 'celisca_floor_1',
                    'world_name': 'celisca_floor_1',
                    'world_package': 'maps',
                },
                'celisca_floor_2': {
                    'map_name': 'celisca_floor_2',
                    'world_name': 'celisca_floor_2',
                    'world_package': 'maps',
                },
            },
            'mode': {
                # Map from scenario/algorithm combinations to legacy modes
                'point_to_point_navigation': 'nav',
                'room_vacuum': 'room-vacuum',
                'exploration': 'exploration',
                'slam': 'slam',
                'display': 'display',
            }
        }
    
    def get_legacy_config(self, robot_id: str, environment_id: str, 
                          scenario_id: Optional[str] = None) -> Dict[str, str]:
        """
        Get legacy configuration for a given robot/environment/scenario.
        
        Args:
            robot_id: Robot ID from registry
            environment_id: Environment ID from registry
            scenario_id: Optional scenario ID
        
        Returns:
            Dictionary of legacy launch arguments
        """
        config = {}
        
        # Map robot
        robot_mapping = self._mappings['robot_id'].get(robot_id, {})
        config.update(robot_mapping)
        
        # Map environment
        env_mapping = self._mappings['environment_id'].get(environment_id, {})
        config.update(env_mapping)
        
        # Map scenario to mode
        if scenario_id:
            mode = self._mappings['mode'].get(scenario_id, 'nav')
            config['mode'] = mode
        else:
            config['mode'] = 'nav'  # Default mode
        
        # Set defaults
        config.setdefault('use_sim_time', 'true')
        config.setdefault('start_gazebo', 'auto')
        config.setdefault('start_rviz', 'auto')
        
        return config
    
    def get_mode_from_algorithms(self, algo_ids: Dict[str, str]) -> str:
        """
        Determine the legacy mode from algorithm selections.
        
        Args:
            algo_ids: Dictionary of category -> algorithm ID
        
        Returns:
            Legacy mode string
        """
        # Check for navigation stack
        has_localization = algo_ids.get('localization') is not None
        has_global_planning = algo_ids.get('global_planning') is not None
        has_local_planning = algo_ids.get('local_planning') is not None
        
        if has_localization and has_global_planning and has_local_planning:
            return 'nav'
        
        # Check for SLAM
        has_slam_algo = any(
            algo_id and ('slam' in algo_id.lower() or 'rtabmap' in algo_id.lower())
            for algo_id in algo_ids.values()
        )
        if has_slam_algo:
            return '3d_slam'
        
        # Default to navigation
        return 'nav'
    
    def generate_legacy_launch_arguments(self, robot_id: str, environment_id: str,
                                         algo_ids: Dict[str, str],
                                         scenario_id: Optional[str] = None,
                                         namespace: Optional[str] = None,
                                         frame_prefix: Optional[str] = None) -> Dict[str, str]:
        """
        Generate legacy launch arguments from robot_lab selections (P2.5).
        
        Args:
            robot_id: Robot ID from registry
            environment_id: Environment ID from registry
            algo_ids: Dictionary of category -> algorithm ID
            scenario_id: Optional scenario ID
            namespace: Optional namespace for multi-robot experiments
            frame_prefix: Optional frame prefix for TF frames
        
        Returns:
            Dictionary of arguments for the legacy launch file
        """
        config = self.get_legacy_config(robot_id, environment_id, scenario_id)
        
        # Determine mode from algorithms if scenario not specified
        if not scenario_id or 'mode' not in config:
            config['mode'] = self.get_mode_from_algorithms(algo_ids)
        
        # Add namespace and frame_prefix if provided (P2.5)
        if namespace:
            config['namespace'] = namespace
        if frame_prefix:
            config['frame_prefix'] = frame_prefix.rstrip('/')
        
        return config
    
    def get_legacy_launch_file(self, robot_id: str) -> Optional[str]:
        """
        Get the legacy launch file path for a robot.
        
        Args:
            robot_id: Robot ID from registry
        
        Returns:
            Path to legacy launch file, or None if not found
        """
        if robot_id == 'bumperbot':
            return 'robot_lab_bringup/simulated_robot.launch.py'
        return None


def create_bumperbot_adapter_launch(robot_id: str, environment_id: str,
                                     algo_ids: Dict[str, str],
                                     scenario_id: Optional[str] = None,
                                     use_sim_time: bool = True,
                                     namespace: Optional[str] = None,
                                     frame_prefix: Optional[str] = None):
    """
    Create a launch description that wraps robot_lab_bringup with robot_lab selections.
    
    This is the P2.4/P2.5 adapter implementation with namespace support.
    
    Args:
        robot_id: Robot ID from registry
        environment_id: Environment ID from registry
        algo_ids: Dictionary of category -> algorithm ID
        scenario_id: Optional scenario ID
        use_sim_time: Whether to use simulation time
        namespace: Optional namespace for multi-robot experiments (P2.5)
        frame_prefix: Optional frame prefix for TF frames (P2.5)
    
    Returns:
        LaunchDescription that includes the legacy launch with mapped arguments
    """
    adapter = LegacyAdapter()
    
    # Get legacy configuration with namespace support (P2.5)
    legacy_args = adapter.generate_legacy_launch_arguments(
        robot_id, environment_id, algo_ids, scenario_id, namespace, frame_prefix
    )
    
    # Convert to launch arguments
    launch_arguments = {}
    for key, value in legacy_args.items():
        # Convert boolean to string
        if isinstance(value, bool):
            launch_arguments[key] = str(value).lower()
        else:
            launch_arguments[key] = str(value)
    
    # Add use_sim_time
    launch_arguments['use_sim_time'] = str(use_sim_time).lower()
    
    # Create the launch description
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from ament_index_python.packages import get_package_share_directory
    
    # Get the path to the legacy launch file
    legacy_launch_path = adapter.get_legacy_launch_file(robot_id)
    if not legacy_launch_path:
        raise ValueError(f'No legacy launch file configured for robot {robot_id}')
    
    # Build the launch description
    ld = LaunchDescription()
    
    # Add all the legacy launch arguments
    for key, value in launch_arguments.items():
        ld.add_action(
            DeclareLaunchArgument(key, default_value=value, description=f'Mapped from robot_lab selectors')
        )
    
    # Include the legacy launch file
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(legacy_launch_path.split('/')[0]),
                    'launch',
                    legacy_launch_path.split('/')[-1]
                )
            ),
            launch_arguments=launch_arguments.items()
        )
    )
    
    return ld


def generate_adapter_launch_description():
    """
    Generate launch description for the robot adapter (P2.4).
    
    This launch file accepts robot_lab selector arguments and translates them
    to legacy robot_lab_bringup arguments.
    """
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
    from launch.substitutions import LaunchConfiguration
    
    def _build_adapter(context):
        # Get all the robot_lab arguments
        robot_id = LaunchConfiguration('robot_id').perform(context)
        environment_id = LaunchConfiguration('environment_id').perform(context)
        scenario_id = LaunchConfiguration('scenario_id').perform(context)
        
        algo_ids = {}
        for cat in ['perception', 'localization', 'state_estimation', 'sensor_fusion', 
                   'global_planning', 'local_planning', 'control']:
            algo_id = LaunchConfiguration(cat).perform(context)
            if algo_id:
                algo_ids[cat] = algo_id
        
        # Get namespace and frame_prefix (P2.5)
        namespace = LaunchConfiguration('namespace').perform(context)
        frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
        
        # Use the adapter to create the legacy launch
        adapter = LegacyAdapter()
        legacy_launch_path = adapter.get_legacy_launch_file(robot_id)
        
        if not legacy_launch_path:
            return [LogInfo(msg=f'No legacy launch file for robot {robot_id}')]
        
        # Generate legacy arguments with namespace support (P2.5)
        legacy_args = adapter.generate_legacy_launch_arguments(
            robot_id, environment_id, algo_ids, scenario_id, namespace, frame_prefix
        )
        
        # Add use_sim_time from argument
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
        legacy_args['use_sim_time'] = use_sim_time
        
        # Convert all values to strings
        launch_arguments = {k: str(v).lower() if isinstance(v, bool) else str(v) 
                          for k, v in legacy_args.items()}
        
        # Log the mapping
        log_msgs = [
            LogInfo(msg=f'Robot Lab Adapter: robot_id={robot_id} -> robot_model={legacy_args.get("robot_model", "N/A")}'),
            LogInfo(msg=f'Robot Lab Adapter: environment_id={environment_id} -> map_name={legacy_args.get("map_name", "N/A")}'),
            LogInfo(msg=f'Robot Lab Adapter: mode={legacy_args.get("mode", "N/A")}'),
        ]
        
        # Include the legacy launch
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        from ament_index_python.packages import get_package_share_directory
        
        package_name, launch_file = legacy_launch_path.split('/')
        log_msgs.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(package_name),
                        'launch',
                        launch_file
                    )
                ),
                launch_arguments=launch_arguments.items()
            )
        )
        
        return log_msgs
    
    # Declare all robot_lab arguments
    launch_args = [
        DeclareLaunchArgument('robot_id', default_value='bumperbot',
                              description='Robot ID from robot_lab registry'),
        DeclareLaunchArgument('environment_id', default_value='small_office',
                              description='Environment ID from robot_lab registry'),
        DeclareLaunchArgument('scenario_id', default_value='',
                              description='Scenario ID (optional)'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        DeclareLaunchArgument('perception', default_value='',
                              description='Perception algorithm ID'),
        DeclareLaunchArgument('localization', default_value='',
                              description='Localization algorithm ID'),
        DeclareLaunchArgument('state_estimation', default_value='',
                              description='State estimation algorithm ID'),
        DeclareLaunchArgument('sensor_fusion', default_value='',
                              description='Sensor fusion algorithm ID'),
        DeclareLaunchArgument('global_planning', default_value='',
                              description='Global planning algorithm ID'),
        DeclareLaunchArgument('local_planning', default_value='',
                              description='Local planning algorithm ID'),
        DeclareLaunchArgument('control', default_value='',
                              description='Control algorithm ID'),
        # Namespace and frame prefix support (P2.5)
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace for multi-robot experiments'),
        DeclareLaunchArgument('frame_prefix', default_value='',
                              description='Frame prefix for TF frames'),
    ]
    
    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function=_build_adapter),
    ])
