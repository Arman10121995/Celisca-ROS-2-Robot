import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_lab_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("robot_lab_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_lab_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_lab_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    imu_driver_node = Node(
        package="robot_lab_firmware",
        executable="mpu6050_driver.py"
    )

    local_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_lab_localization"),
            "launch",
            "local_localization.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items(),
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_lab_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_lab_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items(),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("robot_lab_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items(),
    )
    
    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        imu_driver_node,
        local_localization,
        localization,
        slam,
        navigation,
    ])
