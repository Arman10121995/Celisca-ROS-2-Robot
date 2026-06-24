import os
import tempfile
import xml.etree.ElementTree as ET
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                bumperbot_description, "urdf", "bumperbot.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")
    spawn_x_arg = DeclareLaunchArgument(name="spawn_x", default_value="0.0")
    spawn_y_arg = DeclareLaunchArgument(name="spawn_y", default_value="0.0")
    spawn_z_arg = DeclareLaunchArgument(name="spawn_z", default_value="0.0")
    spawn_yaw_arg = DeclareLaunchArgument(name="spawn_yaw", default_value="0.0")

    use_sim_time = LaunchConfiguration("use_sim_time")

    world_path = PathJoinSubstitution([
            bumperbot_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(bumperbot_description).parent.resolve())
    model_path += pathsep + os.path.join(bumperbot_description, "models")
    model_path += pathsep + os.path.join(bumperbot_description, "meshes")

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": use_sim_time}]
    )

    def get_sdf_world_name(world_file_path, fallback):
        try:
            root = ET.parse(world_file_path).getroot()
            world = root.find("world")
            if world is not None:
                return world.attrib.get("name", fallback)
        except (ET.ParseError, OSError):
            pass
        return fallback

    def create_gz_ros2_bridge(context):
        world_name = LaunchConfiguration("world_name").perform(context)
        world_file_path = os.path.join(
            bumperbot_description,
            "worlds",
            f"{world_name}.world"
        )
        gz_world_name = get_sdf_world_name(world_file_path, world_name)
        oakd_gz_prefix = (
            f"/world/{gz_world_name}/model/bumperbot/link/"
            "oakd_rgb_camera_frame/sensor/rgbd_camera"
        )
        safe_world_name = "".join(
            char if char.isalnum() or char in ("-", "_") else "_"
            for char in world_name
        )
        bridge_config_path = os.path.join(
            tempfile.gettempdir(),
            f"bumperbot_ros_gz_bridge_{safe_world_name}.yaml"
        )
        bridge_config = f"""\
            - ros_topic_name: "/clock"
              gz_topic_name: "/clock"
              ros_type_name: "rosgraph_msgs/msg/Clock"
              gz_type_name: "gz.msgs.Clock"
              direction: "GZ_TO_ROS"
            - ros_topic_name: "/imu/out"
              gz_topic_name: "/imu"
              ros_type_name: "sensor_msgs/msg/Imu"
              gz_type_name: "gz.msgs.IMU"
              direction: "GZ_TO_ROS"
            - ros_topic_name: "/scan"
              gz_topic_name: "/scan"
              ros_type_name: "sensor_msgs/msg/LaserScan"
              gz_type_name: "gz.msgs.LaserScan"
              direction: "GZ_TO_ROS"
            - ros_topic_name: "/oakd/rgb/image_raw"
              gz_topic_name: "{oakd_gz_prefix}/image"
              ros_type_name: "sensor_msgs/msg/Image"
              gz_type_name: "gz.msgs.Image"
              direction: "GZ_TO_ROS"
            - ros_topic_name: "/oakd/rgb/camera_info"
              gz_topic_name: "{oakd_gz_prefix}/camera_info"
              ros_type_name: "sensor_msgs/msg/CameraInfo"
              gz_type_name: "gz.msgs.CameraInfo"
              direction: "GZ_TO_ROS"
            - ros_topic_name: "/oakd/depth/image_raw"
              gz_topic_name: "{oakd_gz_prefix}/depth_image"
              ros_type_name: "sensor_msgs/msg/Image"
              gz_type_name: "gz.msgs.Image"
              direction: "GZ_TO_ROS"
            - ros_topic_name: "/oakd/points_gz"
              gz_topic_name: "{oakd_gz_prefix}/points"
              ros_type_name: "sensor_msgs/msg/PointCloud2"
              gz_type_name: "gz.msgs.PointCloudPacked"
              direction: "GZ_TO_ROS"
            """

        with open(bridge_config_path, "w", encoding="utf-8") as bridge_config_file:
            bridge_config_file.write(bridge_config)

        return [Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[{
                "config_file": bridge_config_path,
                "use_sim_time": use_sim_time
            }]
        )]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"]),
            "on_exit_shutdown": "True"  # <-- ADD THIS
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bumperbot",
                   "-x", LaunchConfiguration("spawn_x"),
                   "-y", LaunchConfiguration("spawn_y"),
                   "-z", LaunchConfiguration("spawn_z"),
                   "-Y", LaunchConfiguration("spawn_yaw")],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    gz_ros2_bridge = OpaqueFunction(function=create_gz_ros2_bridge)

    oakd_pointcloud_converter = Node(
        package="bumperbot_description",
        executable="gz_pointcloud_to_optical.py",
        name="oakd_pointcloud_converter",
        output="screen",
        parameters=[{
            "input_topic": "/oakd/points_gz",
            "output_topic": "/oakd/points",
            "frame_id": "oakd_rgb_camera_optical_frame",
            "use_sim_time": use_sim_time
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        model_arg,
        world_name_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        oakd_pointcloud_converter
    ])
