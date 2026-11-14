import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_detector(context):
    if LaunchConfiguration('vacuum_detector').perform(context) == 'learned':
        from robot_lab_vacuum_cleaning.detector_assets import resolve_model_path
        path = LaunchConfiguration('vacuum_detector_model').perform(context)
        context.launch_configurations['vacuum_detector_model'] = resolve_model_path(path)
    return []


def generate_launch_description():
    bringup_share = get_package_share_directory("robot_lab_bringup")

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
    controller = LaunchConfiguration("controller")
    vacuum_auto_start = LaunchConfiguration("vacuum_auto_start")
    vacuum_report = LaunchConfiguration("vacuum_report")
    vacuum_battery_scale = LaunchConfiguration("vacuum_battery_time_scale")
    allow_experimental_assets = LaunchConfiguration("allow_experimental_assets")

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
            "allow_experimental_assets": allow_experimental_assets,
        }.items(),
    )

    # controller:=explore - the original explore-then-sweep controller.
    room_vacuum = Node(
        package="robot_lab_controller",
        executable="mapping_controller.py",
        name="room_vacuum_controller",
        output="screen",
        condition=IfCondition(PythonExpression(["'", controller, "' == 'explore'"])),
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "cleaning_grid_spacing_m": ParameterValue(cleaning_grid_spacing_m, value_type=float),
            "robot_obstacle_clearance_m": ParameterValue(robot_obstacle_clearance_m, value_type=float),
        }],
    )

    # controller:=cleaning - the robot-vacuum cleaning system (rooms, zones,
    # no-go areas, dock, schedules); the Vacuum workspace drives it.
    cleaning = Node(
        package="robot_lab_vacuum_cleaning",
        executable="vacuum_cleaner",
        name="vacuum_cleaner",
        output="screen",
        condition=IfCondition(PythonExpression(["'", controller, "' == 'cleaning'"])),
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "robot_model": robot_model,
            "map_key": map_name,
            "auto_start": ParameterValue(vacuum_auto_start, value_type=str),
            "report_path": vacuum_report,
            "store_dir": LaunchConfiguration("vacuum_store_dir"),
            "battery_time_scale": ParameterValue(vacuum_battery_scale, value_type=float),
            "object_truth_manifest": LaunchConfiguration("vacuum_object_truth"),
        }],
    )

    # Floor-object recognition from the front camera (oomwoo in Gazebo); it
    # idles when no camera publishes.
    object_detector = Node(
        package="robot_lab_vacuum_cleaning",
        executable="vacuum_object_detector",
        name="vacuum_object_detector",
        output="screen",
        condition=IfCondition(PythonExpression(["'", controller, "' == 'cleaning' and '",
                                               LaunchConfiguration("vacuum_detector"), "' == 'segmentation'"])),
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "camera_frame": "front_camera_optical_frame",
        }],
    )

    learned_detector = Node(
        package="robot_lab_vacuum_cleaning",
        executable="vacuum_learned_detector",
        name="vacuum_learned_object_detector",
        output="screen",
        condition=IfCondition(PythonExpression(["'", controller, "' == 'cleaning' and '",
                                               LaunchConfiguration("vacuum_detector"), "' == 'learned'"])),
        parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "camera_frame": "front_camera_optical_frame",
            "model_path": LaunchConfiguration("vacuum_detector_model"),
            "device": LaunchConfiguration("vacuum_detector_device"),
            "score_thresh": ParameterValue(LaunchConfiguration("vacuum_detector_score"), value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "controller",
            default_value="explore",
            choices=["explore", "cleaning", "none"],
            description="explore: original explore-then-sweep controller; cleaning: "
                        "robot-vacuum cleaning system (robot_lab_vacuum_cleaning); none: simulation only.",
        ),
        DeclareLaunchArgument("vacuum_auto_start", default_value="",
                              description="cleaning: start this mode (all, edges, explore) once ready."),
        DeclareLaunchArgument("vacuum_report", default_value="",
                              description="cleaning: write the auto-started run record to this JSON path."),
        DeclareLaunchArgument("vacuum_store_dir", default_value="",
                              description="Cleaning state directory; separate from shared installed robot/model assets."),
        DeclareLaunchArgument("vacuum_detector", default_value="segmentation",
                              choices=["segmentation", "learned", "none"],
                              description="Floor-object source: ideal simulation labels, learned RGB checkpoint, or disabled."),
        DeclareLaunchArgument("vacuum_detector_model", default_value="auto",
                              description="Local trained checkpoint; auto resolves a checksum-verified installed candidate."),
        DeclareLaunchArgument("vacuum_detector_device", default_value="cpu"),
        DeclareLaunchArgument("vacuum_detector_score", default_value="0.75",
                              description="Learned detector score threshold; 0.75 is the held-out pick for the "
                                          "2026-11-15 multi-map checkpoint (docs/status/evidence/r133-camera-detector-2026-11-15)."),
        DeclareLaunchArgument("vacuum_object_truth", default_value="",
                              description="Independent generated-world objects JSON, used only in the run report."),
        DeclareLaunchArgument("allow_experimental_assets", default_value="false",
                              description="Pass to simulated_robot.launch.py (provisioned robots whose source changed)."),
        DeclareLaunchArgument("vacuum_battery_time_scale", default_value="-1.0",
                              description="cleaning: battery model time scale (<=0 keeps the saved setting)."),
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
            default_value=os.path.join(get_package_share_directory("robot_lab_robots"), "config", "robots.yaml"),
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
        OpaqueFunction(function=_validate_detector),
        RegisterEventHandler(OnProcessExit(
            target_action=learned_detector,
            on_exit=[EmitEvent(event=Shutdown(reason='Selected learned object detector exited'))])),
        simulation,
        room_vacuum,
        cleaning,
        object_detector,
        learned_detector,
    ])
