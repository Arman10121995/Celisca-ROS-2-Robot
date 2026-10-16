"""Nav2 navigation launch, parameterized per robot.

Loads the base Nav2 server configs and, if present, a per-robot overlay from
config/robots/<robot_model>.yaml so frames/footprint/velocity match any robot profile.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


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
        # Arrival (R7.7): at Nav2's defaults MPPI reached 7/15 Bumperbot
        # goals; the others ended 0.05-0.19 m away, outside the 0.07 m /
        # 0.03 rad window.  Goal critics weighted 10/8 with vx/wz sampling
        # noise 0.1/0.3 reached 6/15 and took twice as long (2026-10-13),
        # so MPPI keeps Nav2's defaults.
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
        # A car slows near a goal and cannot satisfy the differential-drive
        # default of 0.5 m every 10 s while making a bounded-radius turn.
        "progress_checker.required_movement_radius": 0.1,
        "progress_checker.movement_time_allowance": 20.0,
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

# What a robot that turns only while moving forward (motion_model:=
# forward_arc) changes.  Go2 himloco does not turn on the spot in MuJoCo or
# Gazebo and does not walk slowly backwards; on grid plans Regulated Pure
# Pursuit arced it into the block beside the nav_obstacle start ("collision
# ahead"), and the first goal aborted in 3 of 6 runs (2026-10-13).  Hybrid-A*
# plans forward-only (DUBIN) curves of at least the robot's turning radius
# and checks the oriented footprint along them; nothing rotates the robot
# in place or backs it up, and goals are position goals.
def forward_arc_params(turning_radius):
    # Round 2 (2026-10-15): with these the start-block aborts were gone, but
    # Go2 near-obstacle goals failed twice: Hybrid-A* hit its 1,000,000
    # iteration cap in 2.8 s, and RPP's 0.8 m lookahead cut a 0.4 m-radius
    # curve until its collision check stopped it.  The search is bounded by
    # time instead, and the lookahead scales with the turning radius.
    lookahead = max(0.5, 1.25 * float(turning_radius))
    return {
        "planner_server": {
            "GridBased.minimum_turning_radius": float(turning_radius),
            "GridBased.motion_model_for_search": "DUBIN",
            "GridBased.allow_reverse_expansion": False,
            "GridBased.max_iterations": -1,
            "GridBased.max_planning_time": 10.0,
        },
        "controller_server": {
            "FollowPath.use_rotate_to_heading": False,
            "FollowPath.allow_reversing": False,
            "FollowPath.lookahead_dist": lookahead,
            "FollowPath.min_lookahead_dist": round(0.7 * lookahead, 3),
            "FollowPath.max_lookahead_dist": round(1.6 * lookahead, 3),
            "general_goal_checker.yaw_goal_tolerance": 6.3,
        },
    }


# Parameters for lateral travel on physical mecanum / parallel-steered bases.
# Keep limits in the sampler, MPPI model and AMCL consistent with linear.y.
HOLONOMIC_PARAMS = {
    "min_y_velocity_threshold": 0.001,
    "FollowPath.min_vel_y": -0.3,
    "FollowPath.max_vel_y": 0.3,
    "FollowPath.acc_lim_y": 0.5,
    "FollowPath.decel_lim_y": -0.5,
    "FollowPath.vy_samples": 9,
    "FollowPath.vy_max": 0.3,
    "FollowPath.ay_max": 0.5,
    "FollowPath.ay_min": -0.5,
    "FollowPath.motion_model": "Omni",
}

# A car cannot execute a grid/DWB plan by rotating in place. Resolve an
# explicit ``auto`` to a motion-model-specific default, and reject an
# incompatible explicit selection instead of silently running another one.
CAR_GLOBAL_PLUGINS = {
    "nav2_smac_planner/SmacPlannerHybrid",
    "nav2_smac_planner/SmacPlannerLattice",
}
CAR_LOCAL_PLUGINS = {
    "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
    "nav2_mppi_controller::MPPIController",
}


def _compatible_planners(motion_model, global_plugin, local_plugin):
    if global_plugin == "auto":
        global_plugin = ("nav2_smac_planner/SmacPlannerHybrid"
                         if motion_model in ("ackermann", "forward_arc") else
                         "nav2_smac_planner/SmacPlanner2D")
    if local_plugin == "auto":
        local_plugin = ("dwb_core::DWBLocalPlanner" if motion_model.startswith("omni") else
                        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
    if motion_model == 'omni_parallel' and local_plugin != 'dwb_core::DWBLocalPlanner':
        raise ValueError('Parallel steering navigation requires DWB with ParallelSteeringCritic')
    if motion_model == "forward_arc" and global_plugin not in CAR_GLOBAL_PLUGINS:
        raise ValueError(
            f"forward-arc navigation requires a curvature-aware global planner, got {global_plugin}")
    if motion_model == "ackermann":
        if global_plugin not in CAR_GLOBAL_PLUGINS:
            raise ValueError(
                f"car navigation requires a curvature-aware global planner, got {global_plugin}")
        if local_plugin not in CAR_LOCAL_PLUGINS:
            raise ValueError(
                f"car navigation requires a curvature-aware controller, got {local_plugin}")
    return global_plugin, local_plugin

_PLANNER_SERVERS = {"planner_server"}
_CONTROLLER_SERVERS = {"controller_server"}


def _lattice_file(motion_model):
    """Default lattice primitives for the motion model (0.5 m turning radius)."""
    try:
        share = get_package_share_directory("nav2_smac_planner")
    except Exception:
        return ""
    variant = "ackermann" if motion_model in ("ackermann", "forward_arc") else "diff"
    return os.path.join(share, "sample_primitives", "5cm_resolution",
                        "0.5m_turning_radius", variant, "output.json")


def _motion_model_overrides(exec_name, motion_model, global_planner_plugin, turning_radius=0.4):
    """Parameters the robot's motion model sets on top of the plugin choice."""
    overrides = {}
    if exec_name in _PLANNER_SERVERS \
            and global_planner_plugin == "nav2_smac_planner/SmacPlannerLattice":
        overrides["GridBased.lattice_filepath"] = _lattice_file(motion_model)
    if motion_model == "ackermann":
        overrides.update(CAR_PARAMS.get(exec_name, {}))
    elif motion_model == "forward_arc":
        overrides.update(forward_arc_params(turning_radius).get(exec_name, {}))
    elif motion_model.startswith("omni") and exec_name in _CONTROLLER_SERVERS:
        overrides.update(HOLONOMIC_PARAMS)
        # Evaluate the oriented footprint, not only the centre cell, for
        # holonomic translation beside obstacles.
        overrides.update({'FollowPath.critics': [
            'RotateToGoal', 'Oscillation', 'ObstacleFootprint',
            'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'],
            'FollowPath.ObstacleFootprint.scale': 0.1})
        if motion_model == 'omni_parallel':
            overrides.update({
                'FollowPath.critics': ['robot_lab_controller::ParallelSteeringCritic',
                    'RotateToGoal', 'Oscillation', 'ObstacleFootprint', 'PathDist', 'GoalDist'],
                'progress_checker.required_movement_radius': 0.2,
                'progress_checker.movement_time_allowance': 20.0,
                # Parallel wheels translate in any direction, backwards
                # included. With DWB's forward-only sampling (min_vel_x 0)
                # a goal behind the base needed a pivot, which the parallel
                # steering critic penalises and which does not shorten the
                # goal distance within one horizon: DWB chose standing still
                # and every seed-2/3 crab and in_phase mission aborted
                # (2026-10-15, /evaluation recorded).
                'FollowPath.min_vel_x': -0.3,
                'FollowPath.vx_samples': 21,
            })
    return [overrides] if overrides else []


MPPI_PLUGIN = "nav2_mppi_controller::MPPIController"
RPP_PLUGIN = "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"


def _uses_arrival_controller(local_planner_plugin, motion_model, follow_path_server, requested):
    """Whether MPPI gets the Regulated Pure Pursuit arrival assist.

    MPPI came within 0.05-0.19 m of 8 of 15 Bumperbot goals and then did not
    settle into the 0.07 m / 0.03 rad window before the progress checker
    aborted (R7.7, twice); stronger goal critics made it worse.  With the
    assist, the controller server also hosts an ``Arrival`` RPP controller
    that finishes the approach on the same path when MPPI's attempt fails.
    """
    if requested in ("false", "0", "no", "off"):
        return False
    return (local_planner_plugin == MPPI_PLUGIN and motion_model == "diff"
            and not follow_path_server)


def _arrival_controller_parameters(base_config, overlay_config):
    """``Arrival.*`` parameters: the base RPP settings, then the robot's own
    FollowPath settings (speeds, rotate-to-heading rate) from its overlay."""
    def follow_path(path):
        try:
            data = yaml.safe_load(open(path)) or {}
        except (OSError, yaml.YAMLError):
            return {}
        params = (data.get("controller_server") or {}).get("ros__parameters") or {}
        return dict(params.get("FollowPath") or {})
    params = follow_path(base_config)
    params.update(follow_path(overlay_config) if overlay_config else {})
    params["plugin"] = RPP_PLUGIN
    return {"controller_plugins": ["FollowPath", "Arrival"],
            **{"Arrival.%s" % key: value for key, value in params.items()
               if not isinstance(value, dict)}}


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


def _tree_with_action_servers(bt_xml, compute_path_server="", follow_path_server=""):
    """The behavior tree with its planning/following actions sent to other servers.

    The robot_lab node planners serve Nav2's ComputePathToPose/FollowPath
    under their own names (robot_lab_algorithms.nav2_adapters); BT action
    nodes take the server through their standard ``server_name`` port.
    Returns a temporary tree file, or ``bt_xml`` unchanged.
    """
    import re
    import tempfile
    if not (compute_path_server or follow_path_server):
        return bt_xml
    text = open(bt_xml, encoding="utf-8").read()
    for tag, server in (("ComputePathToPose", compute_path_server),
                        ("FollowPath", follow_path_server)):
        if server:
            text, count = re.subn(r"<%s(?![^>]*server_name)" % tag,
                                  '<%s server_name="%s"' % (tag, server), text)
            if not count:
                raise RuntimeError("%s has no %s node for %s" % (bt_xml, tag, server))
    handle, path = tempfile.mkstemp(prefix="robot_lab_bt_", suffix=".xml")
    with os.fdopen(handle, "w", encoding="utf-8") as stream:
        stream.write(text)
    return path


def _with_base_frame(path, base_frame):
    """Use the selected URDF root for servers and their nested costmap nodes."""
    return RewrittenYaml(source_file=path,
                         param_rewrites={'robot_base_frame': base_frame},
                         convert_types=True) if base_frame else path


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("robot_lab_navigation")
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    robot_model = LaunchConfiguration("robot_model").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context).strip()
    global_planner_plugin = LaunchConfiguration("global_planner_plugin").perform(context)
    local_planner_plugin = LaunchConfiguration("local_planner_plugin").perform(context)
    motion_model = LaunchConfiguration("motion_model").perform(context).strip().lower()
    car_search_model = LaunchConfiguration('car_search_model').perform(context).strip()
    spin_recovery = LaunchConfiguration("spin_recovery").perform(context).strip().lower() not in (
        "false", "0", "no")
    turning_radius = float(LaunchConfiguration("turning_radius").perform(context).strip() or 0.4)
    compute_path_server = LaunchConfiguration("compute_path_server").perform(context).strip()
    follow_path_server = LaunchConfiguration("follow_path_server").perform(context).strip()
    requested_plugins = (global_planner_plugin, local_planner_plugin)
    global_planner_plugin, local_planner_plugin = _compatible_planners(
        motion_model, global_planner_plugin, local_planner_plugin)
    if car_search_model not in ('auto', 'DUBIN', 'REEDS_SHEPP'):
        raise ValueError('Car search model must be auto, DUBIN or REEDS_SHEPP')
    if car_search_model != 'auto' and (motion_model != 'ackermann'
            or global_planner_plugin != 'nav2_smac_planner/SmacPlannerHybrid'):
        raise ValueError('Explicit car search model requires the Ackermann Hybrid-A* planner')
    if (global_planner_plugin, local_planner_plugin) != requested_plugins:
        print("[robot_lab] navigation auto plugins: "
              f"global={global_planner_plugin}, local={local_planner_plugin}")

    overlay = os.path.join(pkg, "config", "robots", f"{robot_model}.yaml")
    overlay_params = [_with_base_frame(overlay, base_frame)] if os.path.exists(overlay) else []
    arrival = _uses_arrival_controller(
        local_planner_plugin, motion_model, follow_path_server,
        LaunchConfiguration("arrival_controller").perform(context).strip().lower())

    def server(exec_name, name, config_file):
        parameters = [_with_base_frame(os.path.join(pkg, "config", config_file), base_frame)]
        parameters.extend(_planner_parameter_overrides(
            exec_name, global_planner_plugin, local_planner_plugin))
        # Robot-specific footprints, arrival windows and critics override the
        # generic plugin defaults. Loading DWB defaults last erased the full
        # footprint critic configured for nonparallel four-wheel steering.
        parameters += overlay_params
        parameters.append({"use_sim_time": use_sim_time})
        parameters.extend(_motion_model_overrides(
            exec_name, motion_model, global_planner_plugin, turning_radius))
        if exec_name == 'planner_server' and car_search_model != 'auto':
            parameters.append({'GridBased.motion_model_for_search': car_search_model})
        if exec_name in _CONTROLLER_SERVERS and arrival:
            parameters.append(_arrival_controller_parameters(
                os.path.join(pkg, "config", config_file),
                overlay if os.path.exists(overlay) else None))
        if exec_name == "bt_navigator":
            # Resolve the default behavior tree from the installed package
            # share (portable) instead of an absolute source path.
            # The car tree backs up instead of spinning.  A robot that
            # cannot turn on the spot (Go2 himloco in MuJoCo and Gazebo)
            # uses it too: its Spin recovery never turned the Go2 out of the
            # nav_obstacle start corner (Gazebo, 2026-10-13).
            # A forward-arc robot neither spins nor backs up slowly: it
            # clears its local costmap, waits and replans.
            # MPPI with the arrival assist: its tree finishes a failed
            # MPPI approach with the Arrival controller on the same path.
            tree_name = ("forward_navigation_w_replanning_and_recovery.xml"
                         if motion_model == "forward_arc" else
                         "car_navigation_w_replanning_and_recovery.xml"
                         if motion_model == "ackermann" or not spin_recovery else
                         "arrival_navigation_w_replanning_and_recovery.xml"
                         if arrival else
                         "simple_navigation_w_replanning_and_recovery.xml")
            bt_xml = os.path.join(
                pkg, "behavior_tree", tree_name)
            if os.path.isfile(bt_xml):
                parameters.append({"default_nav_to_pose_bt_xml": _tree_with_action_servers(
                    bt_xml, compute_path_server, follow_path_server)})
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

    # RViz's Nav2 panel stamps the goals it sends with the system clock
    # (nav2_rviz_plugins Humble, Nav2Panel::onNewGoal: rclcpp::Clock().now()),
    # so in a /clock run planner_server rejects them with "extrapolation into
    # the future" before planning a path.  The relay takes the standard
    # "2D Goal Pose" tool's /robot_lab/goal_pose and forwards it to the navigator
    # restamped with the run's clock (robot_lab_utils.nav_goals.restamped).
    goal_relay = Node(
        package="robot_lab_navigation",
        executable="nav2_goal_relay.py",
        name="nav2_goal_relay",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return controllers + [goal_relay, lifecycle]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("base_frame", default_value="",
                             description="Selected URDF root; empty retains per-robot YAML frames"),
        DeclareLaunchArgument(
            "robot_model",
            default_value="bumperbot",
            description="Robot id; loads config/robots/<robot_model>.yaml overlay if present",
        ),
        DeclareLaunchArgument(
            "global_planner_plugin",
            default_value="auto",
            description=(
                "Global planner plugin class for planner_server (resolved from the "
                "registry planner selection by the experiment resolver)"
            ),
        ),
        DeclareLaunchArgument(
            "local_planner_plugin",
            default_value="auto",
            description=(
                "Local planner plugin class for controller_server (resolved from the "
                "registry planner selection by the experiment resolver)"
            ),
        ),
        DeclareLaunchArgument(
            "motion_model",
            default_value="diff",
            description="'diff', 'omni' (lateral travel), 'forward_arc' (turns only "
                        "while moving forward) or 'ackermann' (a car: "
                        "turning-radius-aware planning, no rotation in place)",
        ),
        DeclareLaunchArgument(
            "turning_radius", default_value="0.4",
            description="motion_model forward_arc: the smallest radius the robot "
                        "turns at while walking forward (m)"),
        DeclareLaunchArgument(
            "spin_recovery", default_value="true",
            description="'false': the recovery tree backs up instead of spinning "
                        "(for robots that cannot turn on the spot)"),
        DeclareLaunchArgument(
            "arrival_controller", default_value="auto",
            description="auto: MPPI on a differential base gets a Regulated Pure "
                        "Pursuit controller that finishes failed approaches; false: off"),
        DeclareLaunchArgument('car_search_model', default_value='auto',
            description='auto retains current car planning; DUBIN supplies forward-only bicycle-controller paths'),
        DeclareLaunchArgument(
            "compute_path_server", default_value="",
            description="ComputePathToPose action the navigator calls instead of "
                        "planner_server's (a robot_lab node planner); empty: Nav2's"),
        DeclareLaunchArgument(
            "follow_path_server", default_value="",
            description="FollowPath action the navigator calls instead of "
                        "controller_server's (a robot_lab node controller); empty: Nav2's"),
        OpaqueFunction(function=_setup),
    ])
