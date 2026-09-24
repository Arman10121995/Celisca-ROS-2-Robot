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
        # DWB's kinematic defaults are zero. RPP's desired_linear_vel does
        # not configure its sampler, so selecting DWB previously produced
        # "No valid trajectories out of 0" for every navigation goal.
        "FollowPath.min_vel_x": 0.0,
        "FollowPath.min_vel_y": 0.0,
        "FollowPath.max_vel_x": 0.3,
        "FollowPath.max_vel_y": 0.0,
        "FollowPath.max_vel_theta": 1.0,
        "FollowPath.min_speed_xy": 0.0,
        "FollowPath.max_speed_xy": 0.3,
        "FollowPath.min_speed_theta": 0.0,
        "FollowPath.acc_lim_x": 0.5,
        "FollowPath.acc_lim_y": 0.0,
        "FollowPath.acc_lim_theta": 1.5,
        "FollowPath.decel_lim_x": -0.5,
        "FollowPath.decel_lim_y": 0.0,
        "FollowPath.decel_lim_theta": -1.5,
        "FollowPath.vx_samples": 20,
        "FollowPath.vy_samples": 1,
        "FollowPath.vtheta_samples": 20,
        "FollowPath.sim_time": 1.7,
        "FollowPath.linear_granularity": 0.05,
        "FollowPath.angular_granularity": 0.025,
        "FollowPath.trans_stopped_velocity": 0.05,
        "FollowPath.BaseObstacle.scale": 0.02,
        "FollowPath.PathAlign.scale": 32.0,
        "FollowPath.PathAlign.forward_point_distance": 0.1,
        "FollowPath.GoalAlign.scale": 24.0,
        "FollowPath.GoalAlign.forward_point_distance": 0.1,
        "FollowPath.PathDist.scale": 32.0,
        "FollowPath.GoalDist.scale": 24.0,
        "FollowPath.RotateToGoal.scale": 32.0,
        "FollowPath.RotateToGoal.slowing_factor": 5.0,
        "FollowPath.RotateToGoal.lookahead_time": -1.0,
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

# Global planner plugins that need their own parameters.  The state
# lattice planner reads its motion primitives from a file; the defaults
# shipped with nav2_smac_planner are used (5 cm, 0.5 m turning radius), in
# the variant matching the robot's motion model.
GLOBAL_PLANNER_EXTRA_PARAMS = {
    "nav2_smac_planner/SmacPlannerHybrid": {
        "GridBased.minimum_turning_radius": 0.4,
        "GridBased.motion_model_for_search": "DUBIN",
        "GridBased.angle_quantization_bins": 72,
        "GridBased.analytic_expansion_ratio": 3.5,
        "GridBased.analytic_expansion_max_length": 3.0,
        "GridBased.reverse_penalty": 2.0,
        "GridBased.change_penalty": 0.0,
        "GridBased.non_straight_penalty": 1.2,
        "GridBased.cost_penalty": 2.0,
        "GridBased.retrospective_penalty": 0.015,
        "GridBased.lookup_table_size": 20.0,
        "GridBased.cache_obstacle_heuristic": False,
        "GridBased.allow_unknown": True,
        "GridBased.max_planning_time": 5.0,
    },
    "nav2_smac_planner/SmacPlannerLattice": {
        "GridBased.allow_unknown": True,
        "GridBased.max_planning_time": 5.0,
        "GridBased.reverse_penalty": 2.0,
        "GridBased.change_penalty": 0.05,
        "GridBased.non_straight_penalty": 1.05,
        "GridBased.cost_penalty": 2.0,
        "GridBased.rotation_penalty": 5.0,
        "GridBased.retrospective_penalty": 0.015,
        "GridBased.lookup_table_size": 20.0,
        "GridBased.cache_obstacle_heuristic": False,
        "GridBased.allow_reverse_expansion": False,
    },
}

# What a car-like robot (motion_model:=ackermann) changes: it cannot turn
# on the spot, so nothing may rotate it in place, plans respect its
# turning radius (0.5 m for the lab's cars) and it may back up to turn
# around.  The final heading is not controlled by a path follower that
# cannot rotate in place, so goals are position goals (the heading
# tolerance is a full turn).
CAR_TURNING_RADIUS = 0.5
CAR_PARAMS = {
    "planner_server": {
        "GridBased.minimum_turning_radius": CAR_TURNING_RADIUS,
        "GridBased.motion_model_for_search": "REEDS_SHEPP",
        "GridBased.allow_reverse_expansion": True,
    },
    "controller_server": {
        "FollowPath.use_rotate_to_heading": False,
        "FollowPath.allow_reversing": True,
        "FollowPath.lookahead_dist": 0.8,
        "FollowPath.min_lookahead_dist": 0.5,
        "FollowPath.max_lookahead_dist": 1.2,
        "FollowPath.desired_linear_vel": 0.4,
        "FollowPath.regulated_linear_scaling_min_radius": 0.6,
        "FollowPath.motion_model": "Ackermann",
        "FollowPath.AckermannConstraints.min_turning_r": CAR_TURNING_RADIUS,
        "FollowPath.vx_min": -0.3,
        "general_goal_checker.yaw_goal_tolerance": 6.3,
    },
}

_PLANNER_SERVERS = {"planner_server"}
_CONTROLLER_SERVERS = {"controller_server"}


def _lattice_file(motion_model):
    """Default lattice primitives for the motion model (0.5 m turning radius)."""
    try:
        share = get_package_share_directory("nav2_smac_planner")
    except Exception:
        return ""
    variant = "ackermann" if motion_model == "ackermann" else "diff"
    return os.path.join(share, "sample_primitives", "5cm_resolution",
                        "0.5m_turning_radius", variant, "output.json")


def _motion_model_overrides(exec_name, motion_model, global_planner_plugin):
    """Parameters the robot's motion model sets on top of the plugin choice."""
    overrides = {}
    if exec_name in _PLANNER_SERVERS \
            and global_planner_plugin == "nav2_smac_planner/SmacPlannerLattice":
        overrides["GridBased.lattice_filepath"] = _lattice_file(motion_model)
    if motion_model == "ackermann":
        overrides.update(CAR_PARAMS.get(exec_name, {}))
    return [overrides] if overrides else []


def _planner_parameter_overrides(exec_name, global_planner_plugin, local_planner_plugin):
    """
    Parameter overrides that switch the active planner plugins.

    Returns the overrides for one server node given its exec_name; empty for
    servers that host no planner plugins. Defaults reproduce the values in
    the base YAML configs, so an unparametrized launch keeps today's
    behavior while an explicit selection changes the active plugin.
    """
    if exec_name in _PLANNER_SERVERS:
        overrides = {"GridBased.plugin": global_planner_plugin}
        overrides.update(GLOBAL_PLANNER_EXTRA_PARAMS.get(global_planner_plugin, {}))
        return [overrides]
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
    motion_model = LaunchConfiguration("motion_model").perform(context).strip().lower()

    overlay = os.path.join(pkg, "config", "robots", f"{robot_model}.yaml")
    overlay_params = [overlay] if os.path.exists(overlay) else []

    def server(exec_name, name, config_file):
        parameters = [os.path.join(pkg, "config", config_file)] + overlay_params
        parameters.append({"use_sim_time": use_sim_time})
        parameters.extend(_planner_parameter_overrides(
            exec_name, global_planner_plugin, local_planner_plugin))
        parameters.extend(_motion_model_overrides(
            exec_name, motion_model, global_planner_plugin))
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
        DeclareLaunchArgument(
            "motion_model",
            default_value="diff",
            description="'diff' (turns on the spot) or 'ackermann' (a car: "
                        "turning-radius-aware planning, no rotation in place)",
        ),
        OpaqueFunction(function=_setup),
    ])
