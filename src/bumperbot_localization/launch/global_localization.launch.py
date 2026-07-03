import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description():
    map_name = LaunchConfiguration("map_name")
    map_yaml = LaunchConfiguration("map_yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    lifecycle_nodes = ["map_server", "amcl"]

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house"  # celisca_floor_1, celisca_floor_2, small_house, small_warehouse
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value="auto",
        description="Full path to the Nav2 map YAML. 'auto' resolves from map_name."
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to amcl yaml file to load"
    )

    initial_pose_x_arg = DeclareLaunchArgument(
        "initial_pose_x",
        default_value="0.0",
        description="Initial pose x"
    )
    initial_pose_y_arg = DeclareLaunchArgument(
        "initial_pose_y",
        default_value="0.0",
        description="Initial pose y"
    )
    initial_pose_yaw_arg = DeclareLaunchArgument(
        "initial_pose_yaw",
        default_value="0.0",
        description="Initial pose yaw"
    )

    def create_map_server(context):
        name = map_name.perform(context)
        configured_map_yaml = map_yaml.perform(context)
        if configured_map_yaml and configured_map_yaml.lower() != "auto":
            resolved_map_yaml = configured_map_yaml
        else:
            # Consolidated maps package: data lives under share/maps/maps/<name>/maps/map.yaml
            try:
                pkg_share = get_package_share_directory("maps")
                resolved_map_yaml = os.path.join(pkg_share, "maps", name, "maps", "map.yaml")
            except Exception:
                # Legacy fallback (bumperbot_mapping or old layout)
                try:
                    pkg_share = get_package_share_directory("bumperbot_mapping")
                    resolved_map_yaml = os.path.join(pkg_share, "maps", name, "map.yaml")
                except Exception:
                    pkg_share = get_package_share_directory("maps")
                    resolved_map_yaml = os.path.join(pkg_share, "maps", name, "maps", "map.yaml")
        if not os.path.exists(resolved_map_yaml):
            raise RuntimeError(f"Map YAML does not exist: {resolved_map_yaml}")
        use_sim_time_val = use_sim_time.perform(context).lower() in ("true", "1")
        return [
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {"yaml_filename": resolved_map_yaml},
                    {"use_sim_time": use_sim_time_val}
                ],
            )
        ]
    
    nav2_map_server = OpaqueFunction(function=create_map_server)

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time},
            {
                "initial_pose.x": initial_pose_x,
                "initial_pose.y": initial_pose_y,
                "initial_pose.z": 0.0,
                "initial_pose.yaw": initial_pose_yaw,
            },
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        map_yaml_arg,
        amcl_config_arg,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_yaw_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,
    ])
