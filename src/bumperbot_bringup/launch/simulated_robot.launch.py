import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_room_vacuum = LaunchConfiguration("start_room_vacuum")
    cleaning_grid_spacing_m = LaunchConfiguration("cleaning_grid_spacing_m")
    robot_obstacle_clearance_m = LaunchConfiguration("robot_obstacle_clearance_m")

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="nav",
        choices=["loc", "slam", "nav", "nave", "display"],
        description="Bringup mode: loc, slam, nav, or display. 'nave' is accepted as an alias for nav."
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="celisca_floor_1",
        choices=["small_house", "small_warehouse", "celisca_floor_1", "celisca_floor_2", "celisca_f1_actor", "simple_box"],
        description="Map to load for localization and navigation modes."
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    start_room_vacuum_arg = DeclareLaunchArgument(
        "start_room_vacuum",
        default_value="false",
        description="Start the LiDAR-based room vacuum controller."
    )

    cleaning_grid_spacing_m_arg = DeclareLaunchArgument(
        "cleaning_grid_spacing_m",
        default_value="0.40",
        description="Spacing, in meters, between cleaning grid targets. Smaller values produce a denser grid."
    )

    robot_obstacle_clearance_m_arg = DeclareLaunchArgument(
        "robot_obstacle_clearance_m",
        default_value="0.30",
        description="Minimum clearance, in meters, to keep around LiDAR obstacles while cleaning."
    )

    loc_mode = LaunchConfigurationEquals("mode", "loc")
    display_mode = LaunchConfigurationEquals("mode", "display")
    sim_mode = IfCondition(PythonExpression([
        "'", mode, "' in ['loc', 'slam', 'nav', 'nave']"
    ]))
    global_localization_mode = IfCondition(PythonExpression([
        "'", mode, "' in ['loc', 'nav', 'nave']"
    ]))
    local_odometry_mode = IfCondition(PythonExpression([
        "'", mode, "' in ['loc', 'slam', 'nav', 'nave']"
    ]))
    slam_mode = LaunchConfigurationEquals("mode", "slam")
    navigation_mode = IfCondition(PythonExpression([
        "'", mode, "' in ['nav', 'nave']"
    ]))

    bumperbot_description = get_package_share_directory("bumperbot_description")
    bumperbot_controller = get_package_share_directory("bumperbot_controller")
    bumperbot_localization = get_package_share_directory("bumperbot_localization")
    bumperbot_mapping = get_package_share_directory("bumperbot_mapping")
    bumperbot_navigation = get_package_share_directory("bumperbot_navigation")

    def launch_file(package_share, *path_parts):
        return os.path.join(package_share, "launch", *path_parts)

    def rviz_file(package_share, *path_parts):
        return os.path.join(package_share, "rviz", *path_parts)

    def rviz_node(config_path, condition):
        return Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", config_path],
            output="screen",
            parameters=[{'use_sim_time': True}],
            condition=condition,
        )

    z_offset = PythonExpression([
        "{'small_warehouse': '1.0'}.get('", map_name, "', '0.0')" #small_warehouse
    ])


    gazebo = IncludeLaunchDescription(
        launch_file(bumperbot_description, "gazebo.launch.py"),
        launch_arguments={
            "world_name": map_name,
            "spawn_z": z_offset,
            "use_sim_time": use_sim_time
        }.items(),
        condition=sim_mode
    )

    controller = IncludeLaunchDescription(
        launch_file(bumperbot_controller, "controller.launch.py"),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False",
            "use_sim_time": use_sim_time
        }.items(),
        condition=sim_mode
    )

    joystick = IncludeLaunchDescription(
        launch_file(bumperbot_controller, "joystick_teleop.launch.py"),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
        condition=sim_mode
    )

    display = IncludeLaunchDescription(
        launch_file(bumperbot_description, "display.launch.py"),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items(),
        condition=display_mode
    )

    global_localization = IncludeLaunchDescription(
        launch_file(bumperbot_localization, "global_localization.launch.py"),
        launch_arguments={
            "map_name": map_name,
            "use_sim_time": use_sim_time,
        }.items(),
        condition=global_localization_mode
    )

    local_localization = IncludeLaunchDescription(
        launch_file(bumperbot_localization, "local_localization.launch.py"),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=local_odometry_mode
    )

    slam = IncludeLaunchDescription(
        launch_file(bumperbot_mapping, "slam.launch.py"),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=slam_mode
    )

    navigation = IncludeLaunchDescription(
        launch_file(bumperbot_navigation, "navigation.launch.py"),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
        condition=navigation_mode
    )

    room_vacuum = Node(
        package="bumperbot_controller",
        executable="room_vacuum_controller.py",
        name="room_vacuum_controller",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "cleaning_grid_spacing_m": ParameterValue(cleaning_grid_spacing_m, value_type=float),
            "robot_obstacle_clearance_m": ParameterValue(robot_obstacle_clearance_m, value_type=float),
        }],
        condition=IfCondition(start_room_vacuum),
    )

    rviz_localization = rviz_node(
        rviz_file(bumperbot_localization, "global_localization.rviz"),
        loc_mode
    )

    rviz_slam = rviz_node(
        rviz_file(bumperbot_mapping, "slam.rviz"),
        slam_mode
    )

    rviz_nav = rviz_node(
        rviz_file(get_package_share_directory("nav2_bringup"), "nav2_default_view.rviz"),
        navigation_mode
    )

    return LaunchDescription([
        mode_arg,
        map_name_arg,
        use_sim_time_arg,
        start_room_vacuum_arg,
        cleaning_grid_spacing_m_arg,
        robot_obstacle_clearance_m_arg,
        display,
        gazebo,
        controller,
        joystick,
        global_localization,
        local_localization,
        slam,
        navigation,
        room_vacuum,
        rviz_localization,
        rviz_slam,
        rviz_nav
    ])
