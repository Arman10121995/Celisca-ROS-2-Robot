import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_lab_description = get_package_share_directory("robot_lab_description")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    # Re-adding model_arg declaration without a default value
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value="",
        description="Absolute path to robot urdf file. Empty with "
                    "spawn_robot:=false shows the world on its own."
    )

    spawn_robot_arg = DeclareLaunchArgument(
        name="spawn_robot", default_value="true",
        description="Spawn a robot into the world. 'false' runs the world "
                    "alone (map-only display)."
    )

    world_package_arg = DeclareLaunchArgument(
        name="world_package", default_value="maps",
        description="Package containing the world file (used if world_path not provided)"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")
    world_path_arg = DeclareLaunchArgument(
        name="world_path", default_value="",
        description="Explicit full path to the world .world file. If set, overrides world_package+world_name."
    )

    spawn_x_arg = DeclareLaunchArgument(name="spawn_x", default_value="0.0")
    spawn_y_arg = DeclareLaunchArgument(name="spawn_y", default_value="0.0")
    spawn_z_arg = DeclareLaunchArgument(name="spawn_z", default_value="0.0")
    spawn_yaw_arg = DeclareLaunchArgument(name="spawn_yaw", default_value="0.0")
    robot_name_arg = DeclareLaunchArgument(
        name="robot_name", default_value="bumperbot",
        description="Name to use for the spawned Gazebo model and for sensor bridge prefixes"
    )

    robot_package_arg = DeclareLaunchArgument(
        name="robot_package", default_value="robots",
        description="Package name of the robot description (used to populate GZ resource path for meshes)"
    )

    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        description="Show the Gazebo GUI. 'false' runs headless (server only).",
    )

    paused_arg = DeclareLaunchArgument(
        name="paused",
        default_value="false",
        description=(
            "Start the simulation paused (world does not step until "
            "unpaused via /world/<name>/control). Useful when controllers "
            "must be active before the robot physics starts (e.g. the R5.3 "
            "Berkeley Humanoid Lite spawn-fall race)."
        ),
    )

    display_plugins_arg = DeclareLaunchArgument(
        name="display_plugins",
        default_value="false",
        description=(
            "Display runs: give a robot without <ros2_control> Gazebo's "
            "JointStatePublisher system and bridge it to /joint_states, so "
            "RViz places its links from the simulated joints."
        ),
    )
    display_hold_arg = DeclareLaunchArgument(
        name="display_hold",
        default_value="false",
        description=(
            "With display_plugins: hold every movable joint at its rest "
            "angle with a joint velocity servo, so legged and humanoid robots "
            "stand instead of collapsing (as in the other backends)."
        ),
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    world_package = LaunchConfiguration("world_package")
    world_path_lc = LaunchConfiguration("world_path")
    world_name = LaunchConfiguration("world_name")

    def resolve_world_path(context):
        explicit = world_path_lc.perform(context)
        if explicit:
            return explicit
        pkg = world_package.perform(context)
        name = world_name.perform(context)
        try:
            pkg_share = get_package_share_directory(pkg)
        except Exception:
            pkg_share = robot_lab_description
        if pkg in ("maps", "robot_lab_maps"):
            # Consolidated: maps/maps/<name>/worlds/<name>.world
            return os.path.join(pkg_share, "maps", name, "worlds", f"{name}.world")
        else:
            return os.path.join(pkg_share, "worlds", f"{name}.world")

    # For substitutions that need the string early we keep a base; gazebo include will use resolved via Opaque or expression fallback.
    # We will override gz_args construction for world via Opaque where needed.
    world_path = PythonExpression([
        "'", LaunchConfiguration("world_path"), "' if '", LaunchConfiguration("world_path"), "' else '",
        PathJoinSubstitution([
            robot_lab_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]),
        "'"
    ])

    # GZ_SIM_RESOURCE_PATH is now set inside make_gazebo (only when actually starting simulation)
    # to avoid hard failure when the package is not present for non-sim modes.

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    spawn_robot = LaunchConfiguration("spawn_robot")

    def _truthy(name, context):
        return LaunchConfiguration(name).perform(context).strip().lower() in (
            "true", "1", "yes", "on")

    def make_robot_state_publisher(context):
        """robot_state_publisher, plus the display joint-state bridge.

        The spawned model is read from robot_description, so display
        plugins added here reach Gazebo as well as robot_state_publisher.
        """
        if not _truthy("spawn_robot", context):
            return []
        from robot_lab_utils.robot_description import load_description
        urdf = load_description(LaunchConfiguration("model").perform(context),
                                "is_ignition:=" + is_ignition)
        if not _truthy("display_plugins", context):
            return [Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": ParameterValue(urdf, value_type=str),
                             "use_sim_time": use_sim_time}])]
        from robot_lab_utils.gazebo_display import (
            add_display_plugins, has_ros2_control, joint_state_topic)
        actions = [Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": add_display_plugins(
                    urdf, hold=_truthy("display_hold", context)),
                "use_sim_time": use_sim_time}])]
        if not has_ros2_control(urdf):
            topic = joint_state_topic(
                get_sdf_world_name(resolve_world_file(context),
                                   LaunchConfiguration("world_name").perform(context)),
                LaunchConfiguration("robot_name").perform(context))
            actions.append(Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="joint_state_bridge",
                arguments=[topic + "@sensor_msgs/msg/JointState[ignition.msgs.Model"],
                remappings=[(topic, "/joint_states")],
                parameters=[{"use_sim_time": use_sim_time}]))
        return actions

    robot_state_publisher_node = OpaqueFunction(function=make_robot_state_publisher)

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
        world_pkg = LaunchConfiguration("world_package").perform(context)
        rname = LaunchConfiguration("robot_name").perform(context)
        explicit_world = ""
        try:
            explicit_world = LaunchConfiguration("world_path").perform(context)
        except Exception:
            pass
        # Resolve world file for parsing gz world name (used in bridge topics)
        world_file_path = None
        if explicit_world and os.path.exists(explicit_world):
            world_file_path = explicit_world
        else:
            try:
                wp_share = get_package_share_directory(world_pkg)
                if world_pkg == "maps":
                    candidate = os.path.join(wp_share, "maps", world_name, "worlds", f"{world_name}.world")
                else:
                    candidate = os.path.join(wp_share, "worlds", f"{world_name}.world")
                if os.path.exists(candidate):
                    world_file_path = candidate
            except Exception:
                pass
            if not world_file_path:
                world_file_path = os.path.join(robot_lab_description, "worlds", f"{world_name}.world")
        gz_world_name = get_sdf_world_name(world_file_path, world_name)
        oakd_gz_prefix = (
            f"/world/{gz_world_name}/model/{rname}/link/"
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

    def make_gazebo(context):
        # Determine effective world file path
        expl = ""
        try:
            expl = LaunchConfiguration("world_path").perform(context)
        except Exception:
            pass
        if expl and not os.path.isfile(expl):
            raise RuntimeError("Selected Gazebo world does not exist: " + expl)
        if expl:
            w = expl
        else:
            wname = LaunchConfiguration("world_name").perform(context)
            wpkg = LaunchConfiguration("world_package").perform(context)
            try:
                wshare = get_package_share_directory(wpkg)
                if wpkg in ("maps", "robot_lab_maps"):
                    w = os.path.join(wshare, "maps", wname, "worlds", f"{wname}.world")
                else:
                    w = os.path.join(wshare, "worlds", f"{wname}.world")
            except Exception:
                w = os.path.join(robot_lab_description, "worlds", f"{wname}.world")
            if not os.path.exists(w):
                raise RuntimeError("Selected Gazebo world does not exist: " + w)

        # Build GZ_SIM_RESOURCE_PATH (only executed for actual simulation)
        try:
            gazebo_models_share = get_package_share_directory("robot_lab_models")
            model_path = str(Path(gazebo_models_share).parent.resolve())
            model_path += pathsep + os.path.join(gazebo_models_share, "models")
        except Exception:
            # Fallback if gazebo_models package is not present (will likely fail later when gz tries to load models)
            model_path = ""
        # legacy support from robot_lab_description if anything is still there
        model_path += pathsep + os.path.dirname(robot_lab_description)
        model_path += pathsep + os.path.join(robot_lab_description, "models")
        model_path += pathsep + os.path.join(robot_lab_description, "meshes")
        # Include the robot package share dir (parent) so model://<robot_pkg>/... and package:// URIs for meshes resolve in Gazebo
        try:
            rpkg = LaunchConfiguration("robot_package").perform(context)
            if rpkg:
                rshare = get_package_share_directory(rpkg)
                model_path += pathsep + os.path.dirname(rshare)
        except Exception:
            pass
        # Consolidated robots sub packages (add parent dir so package://robot_lab_robots/... resolves)
        try:
            robots_share = get_package_share_directory("robot_lab_robots")
            model_path += pathsep + os.path.dirname(robots_share)
        except Exception:
            pass
        # Consolidated maps package (for world meshes like celisca buildings)
        try:
            maps_share = get_package_share_directory("robot_lab_maps")
            model_path += pathsep + os.path.dirname(maps_share)
            model_path += pathsep + os.path.join(maps_share, "maps")
        except Exception:
            pass

        # Update current process env so nested OpaqueFunctions (ros_gz_sim's gz_sim.launch.py)
        # that read os.environ directly will see the paths for both Ignition and GZ.
        # Also set via launch action below for the spawned processes.
        cur_gz = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        os.environ["GZ_SIM_RESOURCE_PATH"] = pathsep.join([p for p in (cur_gz, model_path) if p])
        cur_ign = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = pathsep.join([p for p in (cur_ign, model_path) if p])

        # Ensure ros2_control plugins (ign_ros2_control / gz_ros2_control) can be found by Gazebo
        gz_plugin_path = pathsep.join(
            [p for p in ["/opt/ros/humble/lib", os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""), os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")] if p]
        )
        actions = [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.environ["GZ_SIM_RESOURCE_PATH"]),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", os.environ["IGN_GAZEBO_RESOURCE_PATH"]),
            SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", gz_plugin_path),
            SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", gz_plugin_path),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": (
                        f"{w} -v 4 "
                        f"{'-s' if LaunchConfiguration('gui').perform(context) == 'false' else ''}"
                        f"{'' if LaunchConfiguration('paused').perform(context) == 'true' else ' -r'}"
                    ),
                    "on_exit_shutdown": "True"
                }.items()
            )
        ]
        return actions

    gazebo = OpaqueFunction(function=make_gazebo)

    def resolve_world_file(context):
        """The world file Gazebo loads (explicit path, else package lookup)."""
        wname = LaunchConfiguration("world_name").perform(context)
        wpkg = LaunchConfiguration("world_package").perform(context)
        explicit_world = LaunchConfiguration("world_path").perform(context)
        if explicit_world and os.path.exists(explicit_world):
            return explicit_world
        try:
            wp_share = get_package_share_directory(wpkg)
            if wpkg in ("maps", "robot_lab_maps"):
                candidate = os.path.join(wp_share, "maps", wname, "worlds", f"{wname}.world")
            else:
                candidate = os.path.join(wp_share, "worlds", f"{wname}.world")
            if os.path.exists(candidate):
                return candidate
        except Exception:
            pass
        return os.path.join(robot_lab_description, "worlds", f"{wname}.world")

    def spawn_entity_function(context):
        # Resolve the actual world name from SDF
        wname = LaunchConfiguration("world_name").perform(context)
        wpkg = LaunchConfiguration("world_package").perform(context)
        explicit_world = ""
        try:
            explicit_world = LaunchConfiguration("world_path").perform(context)
        except Exception:
            pass
        world_file_path = None
        if explicit_world and os.path.exists(explicit_world):
            world_file_path = explicit_world
        else:
            try:
                wp_share = get_package_share_directory(wpkg)
                if wpkg in ("maps", "robot_lab_maps"):
                    candidate = os.path.join(wp_share, "maps", wname, "worlds", f"{wname}.world")
                else:
                    candidate = os.path.join(wp_share, "worlds", f"{wname}.world")
                if os.path.exists(candidate):
                    world_file_path = candidate
            except Exception:
                pass
            if not world_file_path:
                world_file_path = os.path.join(robot_lab_description, "worlds", f"{wname}.world")
        
        gz_world_name = get_sdf_world_name(world_file_path, wname)
        rname = LaunchConfiguration("robot_name").perform(context)

        if LaunchConfiguration("spawn_robot").perform(context).lower() \
                not in ("true", "1", "yes", "on"):
            return []

        return [Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-world", gz_world_name,
                       "-topic", "robot_description",
                       "-name", rname,
                       "-x", LaunchConfiguration("spawn_x"),
                       "-y", LaunchConfiguration("spawn_y"),
                       "-z", LaunchConfiguration("spawn_z"),
                       "-Y", LaunchConfiguration("spawn_yaw")],
            parameters=[{"use_sim_time": use_sim_time}]
        )]

    gz_spawn_entity = TimerAction(
        period=1.5,
        actions=[OpaqueFunction(function=spawn_entity_function)]
    )

    gz_ros2_bridge = OpaqueFunction(function=create_gz_ros2_bridge)

    oakd_pointcloud_converter = Node(
        package="robot_lab_description",
        executable="gz_pointcloud_to_optical.py",
        name="oakd_pointcloud_converter",
        output="screen",
        condition=IfCondition(spawn_robot),
        parameters=[{
            "input_topic": "/oakd/points_gz",
            "output_topic": "/oakd/points",
            "frame_id": "oakd_rgb_camera_optical_frame",
            "use_sim_time": use_sim_time
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        model_arg, # Re-added model_arg
        spawn_robot_arg,
        world_name_arg,
        world_package_arg,
        world_path_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        robot_name_arg,
        robot_package_arg,
        gui_arg,
        paused_arg,
        display_plugins_arg,
        display_hold_arg,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        oakd_pointcloud_converter
    ])
