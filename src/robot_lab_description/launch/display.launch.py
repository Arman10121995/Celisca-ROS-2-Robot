import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_lab_description_dir = get_package_share_directory("robot_lab_description")
    try:
        robots_dir = get_package_share_directory("robots")
        default_model = os.path.join(robots_dir, "bumperbot", "urdf", "bumperbot.urdf.xacro")
    except Exception:
        default_model = ""

    model_arg = DeclareLaunchArgument(name="model", default_value=default_model,
                                      description="Absolute path to robot urdf file")

    rviz_config_arg = DeclareLaunchArgument(
        name="rviz_config",
        default_value=os.path.join(robot_lab_description_dir, "rviz", "display.rviz"),
        description="Full path to RViz config file"
    )

    start_rviz_arg = DeclareLaunchArgument(
        name="start_rviz",
        default_value="true",
        description="Start RViz if true"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": use_sim_time
                     }]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    # Provide map -> odom and odom -> base transforms so that the TF tree is connected.
    # This eliminates warnings like "No transform from ... to map" / "to odom"
    # when running in display mode (no real odometry or localization publishing them).
    # Identity transform (no offset). Also publish to base_footprint for compatibility.
    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    static_odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "base"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    static_odom_to_base_footprint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_base_footprint",
        arguments=["0", "0", "0", "0", "0", "0", "1", "odom", "base_footprint"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Connect base_footprint (the Fixed Frame in the default display.rviz) to the humanoid root link "base"
    static_base_footprint_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_base_footprint_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "1", "base_footprint", "base"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        model_arg,
        rviz_config_arg,
        start_rviz_arg,
        use_sim_time_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        static_map_to_odom,
        static_odom_to_base,
        static_odom_to_base_footprint,
        static_base_footprint_to_base,
    ])
