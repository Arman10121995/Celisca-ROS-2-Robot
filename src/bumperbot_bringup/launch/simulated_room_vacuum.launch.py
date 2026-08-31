import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("bumperbot_bringup")

    mode = LaunchConfiguration("mode")
    map_name = LaunchConfiguration("map_name")
    sim_modes_config = LaunchConfiguration("sim_modes_config")
    sim_maps_config = LaunchConfiguration("sim_maps_config")
    sim_robots_config = LaunchConfiguration("sim_robots_config")
    robot_model = LaunchConfiguration("robot_model")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    world_package = LaunchConfiguration("world_package")
    world_name = LaunchConfiguration("world_name")
    world_path = LaunchConfiguration("world_path")
    map_yaml = LaunchConfiguration("map_yaml")
    robot_package = LaunchConfiguration("robot_package")
    robot_xacro = LaunchConfiguration("robot_xacro")
    robot_name = LaunchConfiguration("robot_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    rgb_topic = LaunchConfiguration("rgb_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    rtabmap_frame_id = LaunchConfiguration("rtabmap_frame_id")
    rtabmap_map_frame_id = LaunchConfiguration("rtabmap_map_frame_id")
    rtabmap_config = LaunchConfiguration("rtabmap_config")
    rtabmap_database_path = LaunchConfiguration("rtabmap_database_path")
    delete_db_on_start = LaunchConfiguration("delete_db_on_start")
    start_visual_odometry = LaunchConfiguration("start_visual_odometry")
    start_rtabmap_viz = LaunchConfiguration("start_rtabmap_viz")
    cleaning_grid_spacing_m = LaunchConfiguration("cleaning_grid_spacing_m")
    robot_obstacle_clearance_m = LaunchConfiguration("robot_obstacle_clearance_m")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "simulated_robot.launch.py")),
        launch_arguments={
            "mode": mode,
            "map_name": map_name,
            "sim_modes_config": sim_modes_config,
            "sim_maps_config": sim_maps_config,
            "sim_robots_config": sim_robots_config,
            "robot_model": robot_model,
            "use_sim_time": use_sim_time,
            "start_gazebo": start_gazebo,
            "start_rviz": start_rviz,
            "rviz_config": rviz_config,
            "world_package": world_package,
            "world_name": world_name,
            "world_path": world_path,
            "map_yaml": map_yaml,
            "robot_package": robot_package,
            "robot_xacro": robot_xacro,
            "robot_name": robot_name,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
            "initial_pose_x": initial_pose_x,
            "initial_pose_y": initial_pose_y,
            "initial_pose_yaw": initial_pose_yaw,
            "rgb_topic": rgb_topic,
            "depth_topic": depth_topic,
            "camera_info_topic": camera_info_topic,
            "odom_topic": odom_topic,
            "rtabmap_frame_id": rtabmap_frame_id,
            "rtabmap_map_frame_id": rtabmap_map_frame_id,
            "rtabmap_config": rtabmap_config,
            "rtabmap_database_path": rtabmap_database_path,
            "delete_db_on_start": delete_db_on_start,
            "start_visual_odometry": start_visual_odometry,
            "start_rtabmap_viz": start_rtabmap_viz,
        }.items(),
    )

    room_vacuum = Node(
        package="bumperbot_controller",
        executable="mapping_controller.py",
        name="room_vacuum_controller",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "cleaning_grid_spacing_m": ParameterValue(cleaning_grid_spacing_m, value_type=float),
            "robot_obstacle_clearance_m": ParameterValue(robot_obstacle_clearance_m, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="nav",
            description="Simulation mode to use before starting the room vacuum controller.",
        ),
        DeclareLaunchArgument(
            "map_name",
            default_value="celisca_floor_1",
            description="Map profile name from sim_maps.yaml.",
        ),
        DeclareLaunchArgument(
            "sim_modes_config",
            default_value=os.path.join(bringup_share, "config", "sim_modes.yaml"),
            description="YAML file defining mode-specific simulator, stack, and RViz behavior.",
        ),
        DeclareLaunchArgument(
            "sim_maps_config",
            default_value=os.path.join(bringup_share, "config", "sim_maps.yaml"),
            description="YAML file defining map-specific Gazebo, spawn, and localization defaults.",
        ),
        DeclareLaunchArgument(
            "sim_robots_config",
            default_value=os.path.join(get_package_share_directory("robots"), "config", "robots.yaml"),
            description="YAML file defining robot model profiles.",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="bumperbot",
            description="Robot profile name from robots/config/robots.yaml.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_gazebo", default_value="auto", choices=["auto", "true", "false"]),
        DeclareLaunchArgument("start_rviz", default_value="auto", choices=["auto", "true", "false"]),
        DeclareLaunchArgument("rviz_config", default_value="auto"),
        DeclareLaunchArgument("world_package", default_value="auto"),
        DeclareLaunchArgument("world_name", default_value="auto"),
        DeclareLaunchArgument("world_path", default_value="auto"),
        DeclareLaunchArgument("map_yaml", default_value="auto"),
        DeclareLaunchArgument("robot_package", default_value="auto"),
        DeclareLaunchArgument("robot_xacro", default_value="auto"),
        DeclareLaunchArgument("robot_name", default_value="auto"),
        DeclareLaunchArgument("spawn_x", default_value="auto"),
        DeclareLaunchArgument("spawn_y", default_value="auto"),
        DeclareLaunchArgument("spawn_z", default_value="auto"),
        DeclareLaunchArgument("spawn_yaw", default_value="auto"),
        DeclareLaunchArgument("initial_pose_x", default_value="auto"),
        DeclareLaunchArgument("initial_pose_y", default_value="auto"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="auto"),
        DeclareLaunchArgument("rgb_topic", default_value="auto"),
        DeclareLaunchArgument("depth_topic", default_value="auto"),
        DeclareLaunchArgument("camera_info_topic", default_value="auto"),
        DeclareLaunchArgument("odom_topic", default_value="auto"),
        DeclareLaunchArgument("rtabmap_frame_id", default_value="auto"),
        DeclareLaunchArgument("rtabmap_map_frame_id", default_value="auto"),
        DeclareLaunchArgument("rtabmap_config", default_value="auto"),
        DeclareLaunchArgument("rtabmap_database_path", default_value="auto"),
        DeclareLaunchArgument("delete_db_on_start", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("start_visual_odometry", default_value="auto", choices=["auto", "true", "false"]),
        DeclareLaunchArgument("start_rtabmap_viz", default_value="auto", choices=["auto", "true", "false"]),
        DeclareLaunchArgument(
            "cleaning_grid_spacing_m",
            default_value="0.40",
            description="Spacing, in meters, between cleaning grid targets.",
        ),
        DeclareLaunchArgument(
            "robot_obstacle_clearance_m",
            default_value="0.30",
            description="Minimum clearance, in meters, around LiDAR obstacles while cleaning.",
        ),
        simulation,
        room_vacuum,
    ])
