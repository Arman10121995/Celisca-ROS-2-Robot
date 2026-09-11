"""Nav2 navigation launch, parameterized per robot.

Loads the base Nav2 server configs and, if present, a per-robot overlay from
config/robots/<robot_model>.yaml so frames/footprint/velocity match any robot profile.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Extra parameters required by specific local planner plugins (the base
# controller_server.yaml is written for the regulated pure pursuit default).
# Keyed by plugin class; applied after the base config so switching planners
# changes both the active plugin and its supporting parameters.
LOCAL_PLANNER_EXTRA_PARAMS = {
    "dwb_core::DWBLocalPlanner": {
        "FollowPath.critics": [
            "RotateToGoal", "Oscillation", "BaseObstacle",
            "GoalAlign", "PathAlign", "PathDist", "GoalDist",
        ],
    },
    # MPPI scores sampled rollouts with weighted critics instead of the
    # DWB critic set, and needs its own horizon/batch configuration.
    "nav2_mppi_controller::MPPIController": {
        "FollowPath.time_steps": 56,
        "FollowPath.model_dt": 0.05,
        "FollowPath.batch_size": 2000,
        "FollowPath.iteration_count": 1,
        "FollowPath.temperature": 0.3,
        "FollowPath.gamma": 0.015,
        "FollowPath.motion_model": "DiffDrive",
        "FollowPath.critics": [
            "ConstraintCritic", "CostCritic", "GoalCritic",
            "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic",
            "PathAngleCritic", "PreferForwardCritic",
        ],
    },
}

_PLANNER_SERVERS = {"planner_server"}
_CONTROLLER_SERVERS = {"controller_server"}


def _planner_parameter_overrides(exec_name, global_planner_plugin, local_planner_plugin):
    """
    Parameter overrides that switch the active planner plugins.

    Returns the overrides for one server node given its exec_name; empty for
    servers that host no planner plugins. Defaults reproduce the values in
    the base YAML configs, so an unparametrized launch keeps today's
    behavior while an explicit selection changes the active plugin.
    """
    if exec_name in _PLANNER_SERVERS:
        return [{"GridBased.plugin": global_planner_plugin}]
    if exec_name in _CONTROLLER_SERVERS:
        overrides = {"FollowPath.plugin": local_planner_plugin}
        overrides.update(LOCAL_PLANNER_EXTRA_PARAMS.get(local_planner_plugin, {}))
        return [overrides]
    return []


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("robot_lab_navigation")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    robot_model = LaunchConfiguration("robot_model").perform(context)
    global_planner_plugin = LaunchConfiguration("global_planner_plugin").perform(context)
    local_planner_plugin = LaunchConfiguration("local_planner_plugin").perform(context)

    overlay = os.path.join(pkg, "config", "robots", f"{robot_model}.yaml")
    overlay_params = [overlay] if os.path.exists(overlay) else []

    def server(exec_name, name, config_file):
        parameters = [os.path.join(pkg, "config", config_file)] + overlay_params
        parameters.append({"use_sim_time": use_sim_time})
        parameters.extend(_planner_parameter_overrides(
            exec_name, global_planner_plugin, local_planner_plugin))
        if exec_name == "bt_navigator":
            # Resolve the default behavior tree from the installed package
            # share (portable) instead of an absolute source path.
            bt_xml = os.path.join(
                pkg, "behavior_tree",
                "simple_navigation_w_replanning_and_recovery.xml")
            if os.path.isfile(bt_xml):
                parameters.append({"default_nav_to_pose_bt_xml": bt_xml})
        return Node(
            package="nav2_controller" if exec_name == "controller_server"
            else "nav2_planner" if exec_name == "planner_server"
            else "nav2_smoother" if exec_name == "smoother_server"
            else "nav2_bt_navigator" if exec_name == "bt_navigator"
            else "nav2_behaviors",
            executable=exec_name,
            name=name,
            output="screen",
            parameters=parameters,
        )

    controllers = [
        server("controller_server", "controller_server", "controller_server.yaml"),
        server("planner_server", "planner_server", "planner_server.yaml"),
        server("smoother_server", "smoother_server", "smoother_server.yaml"),
        server("bt_navigator", "bt_navigator", "bt_navigator.yaml"),
        server("behavior_server", "behavior_server", "behavior_server.yaml"),
    ]

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": ["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
        ],
    )

    return controllers + [lifecycle]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "robot_model",
            default_value="bumperbot",
            description="Robot id; loads config/robots/<robot_model>.yaml overlay if present",
        ),
        DeclareLaunchArgument(
            "global_planner_plugin",
            default_value="nav2_smac_planner/SmacPlanner2D",
            description=(
                "Global planner plugin class for planner_server (resolved from the "
                "registry planner selection by the experiment resolver)"
            ),
        ),
        DeclareLaunchArgument(
            "local_planner_plugin",
            default_value="nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
            description=(
                "Local planner plugin class for controller_server (resolved from the "
                "registry planner selection by the experiment resolver)"
            ),
        ),
        OpaqueFunction(function=_setup),
    ])
