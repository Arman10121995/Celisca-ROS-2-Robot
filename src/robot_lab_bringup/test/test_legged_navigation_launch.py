"""Legged mapping/navigation launch wiring (2026-10-13).

In slam/nav a walking robot gets its provisioned rl_sar policy (it is the
only way it follows /cmd_vel) and leg kinematic odometry replaces wheel
odometry as the EKF's odom0, in every backend that runs the policy.
"""
import importlib.util
import os
from unittest import mock

import pytest

_BRINGUP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _build(**overrides):
    try:
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
        spec = importlib.util.spec_from_file_location(
            'robot_lab_legged_nav_launch', os.path.join(_BRINGUP, 'launch', 'simulated_robot.launch.py'))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
    except Exception as exc:  # pragma: no cover - workspace not installed
        pytest.skip('launch file needs the installed workspace: %s' % exc)
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
            context.launch_configurations[entity.name] = ''.join(
                part.perform(context) for part in entity.default_value)
    context.launch_configurations.update(dict(gui='false', start_rviz='false'), **overrides)
    with mock.patch.object(module, '_run_id_actions', lambda: []):
        actions = module._build_simulation_actions(context)
    return context, actions


def _walk(items):
    for item in items:
        yield item
        handler = getattr(item, '_RegisterEventHandler__event_handler', None)
        nested = getattr(handler, '_OnActionEventBase__actions_on_event', None)
        if isinstance(nested, (list, tuple)):
            yield from _walk(nested)


def _text(context, value):
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


def _parameters(context, node):
    flat = {}
    for entry in node._Node__parameters:
        if isinstance(entry, dict):
            for key, value in entry.items():
                key = key if isinstance(key, str) else _text(context, key)
                flat[key] = value if isinstance(value, (bool, int, float)) else _text(context, value)
    return flat


@pytest.mark.integration
@pytest.mark.parametrize('mode', ['nav', 'slam'])
def test_go2_walks_with_leg_odometry_in_mujoco(mode):
    from launch.actions import IncludeLaunchDescription
    from launch_ros.actions import Node
    context, actions = _build(robot_model='unitree_go2', simulator='mujoco', mode=mode,
                              map_name='nav_obstacle')
    every = list(_walk(actions))
    nodes = [a for a in every if isinstance(a, Node)]
    executables = {getattr(n, '_Node__node_executable', '') for n in nodes}
    assert 'legged-policy-controller' in executables
    assert 'leg_odometry' in executables
    policy = next(n for n in nodes if n._Node__node_executable == 'legged-policy-controller')
    assert _parameters(context, policy)['policy'].split('\n')[0].endswith('go2_himloco')
    odometry = next(n for n in nodes if n._Node__node_executable == 'leg_odometry')
    params = _parameters(context, odometry)
    assert params['base_link'].split('\n')[0] == 'base'
    assert 'FL_foot' in params['robot_description']
    local = [a for a in every if isinstance(a, IncludeLaunchDescription) and _text(
        context, a.launch_description_source._LaunchDescriptionSource__location).endswith(
            'local_localization.launch.py')]
    arguments = {_text(context, k): _text(context, v) for k, v in local[0].launch_arguments}
    assert arguments['odom0'] == '/leg_odometry'


@pytest.mark.integration
@pytest.mark.parametrize('source,odom0', [('truth', '/odom/ground_truth'), ('wheels', '/robot_lab_controller/odom')])
def test_wheel_odometry_runs_outside_gazebo_and_can_feed_the_ekf(source, odom0):
    from launch.actions import IncludeLaunchDescription
    from launch_ros.actions import Node
    context, actions = _build(robot_model='labbot', simulator='mujoco', mode='nav',
                              map_name='nav_obstacle', odometry_source=source)
    every = list(_walk(actions))
    wheel = [a for a in every if isinstance(a, Node)
             and getattr(a, '_Node__node_executable', '') == 'wheel_odometry']
    assert wheel, 'wheel odometry not started'
    params = _parameters(context, wheel[0])
    assert params['left_wheel_joint'].split('\n')[0] == 'labbot_left_wheel_joint'
    local = [a for a in every if isinstance(a, IncludeLaunchDescription) and _text(
        context, a.launch_description_source._LaunchDescriptionSource__location).endswith(
            'local_localization.launch.py')]
    arguments = {_text(context, k): _text(context, v) for k, v in local[0].launch_arguments}
    assert arguments['odom0'] == odom0


@pytest.mark.integration
def test_wheeled_robots_keep_their_odometry():
    from launch.actions import IncludeLaunchDescription
    from launch_ros.actions import Node
    context, actions = _build(robot_model='bumperbot', simulator='mujoco', mode='nav',
                              map_name='nav_obstacle')
    every = list(_walk(actions))
    assert not [a for a in every if isinstance(a, Node)
                and getattr(a, '_Node__node_executable', '') == 'leg_odometry']
    local = [a for a in every if isinstance(a, IncludeLaunchDescription) and _text(
        context, a.launch_description_source._LaunchDescriptionSource__location).endswith(
            'local_localization.launch.py')]
    arguments = {_text(context, k): _text(context, v) for k, v in local[0].launch_arguments}
    assert arguments['odom0'] == '/odom/ground_truth'


@pytest.mark.integration
@pytest.mark.parametrize('robot,simulator,spin', [
    ('unitree_go2', 'mujoco', 'false'), ('unitree_go2', 'gazebo', 'false'),
    ('unitree_go2', 'isaac', 'true'), ('bumperbot', 'mujoco', 'true')])
def test_robots_that_cannot_spin_back_up_instead(robot, simulator, spin):
    from launch.actions import IncludeLaunchDescription
    context, actions = _build(robot_model=robot, simulator=simulator, mode='nav',
                              map_name='nav_obstacle')
    navigation = [a for a in _walk(actions) if isinstance(a, IncludeLaunchDescription) and _text(
        context, a.launch_description_source._LaunchDescriptionSource__location).endswith(
            'navigation.launch.py')]
    arguments = {_text(context, k): _text(context, v) for k, v in navigation[0].launch_arguments}
    assert arguments['spin_recovery'] == spin


@pytest.mark.integration
@pytest.mark.parametrize('robot,simulator,model,radius', [
    ('unitree_go2', 'mujoco', 'forward_arc', '0.4'), ('unitree_go2', 'isaac', 'diff', None),
    ('unitree_b2', 'mujoco', 'forward_arc', '0.5'), ('bumperbot', 'mujoco', 'diff', None)])
def test_robots_that_cannot_turn_on_the_spot_plan_forward_arcs(robot, simulator, model, radius):
    from launch.actions import IncludeLaunchDescription
    context, actions = _build(robot_model=robot, simulator=simulator, mode='nav',
                              map_name='nav_obstacle')
    navigation = [a for a in _walk(actions) if isinstance(a, IncludeLaunchDescription) and _text(
        context, a.launch_description_source._LaunchDescriptionSource__location).endswith(
            'navigation.launch.py')]
    arguments = {_text(context, k): _text(context, v) for k, v in navigation[0].launch_arguments}
    assert arguments['motion_model'] == model
    if radius:
        assert arguments['turning_radius'] == radius


def test_forward_arc_navigation_plans_curves_and_never_spins():
    import importlib.util
    spec = importlib.util.spec_from_file_location('robot_lab_navigation_launch', os.path.join(
        _BRINGUP, '..', 'robot_lab_navigation', 'launch', 'navigation.launch.py'))
    navigation = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(navigation)
    except Exception as exc:  # pragma: no cover - workspace not installed
        pytest.skip('navigation launch needs ROS: %s' % exc)
    planner, controller = navigation._compatible_planners('forward_arc', 'auto', 'auto')
    assert planner == 'nav2_smac_planner/SmacPlannerHybrid'
    with pytest.raises(ValueError):
        navigation._compatible_planners('forward_arc', 'nav2_smac_planner/SmacPlanner2D', 'auto')
    plan = navigation._motion_model_overrides('planner_server', 'forward_arc', planner, 0.4)[0]
    assert plan['GridBased.minimum_turning_radius'] == 0.4
    assert plan['GridBased.motion_model_for_search'] == 'DUBIN'
    assert plan['GridBased.allow_reverse_expansion'] is False
    follow = navigation._motion_model_overrides('controller_server', 'forward_arc', planner, 0.4)[0]
    assert follow['FollowPath.use_rotate_to_heading'] is False
    assert follow['FollowPath.allow_reversing'] is False


@pytest.mark.integration
def test_forward_arc_robots_default_to_a_curvature_aware_planner():
    from launch.actions import IncludeLaunchDescription
    context, actions = _build(robot_model='unitree_go2', simulator='mujoco', mode='nav',
                              map_name='nav_obstacle')
    navigation = [a for a in _walk(actions) if isinstance(a, IncludeLaunchDescription) and _text(
        context, a.launch_description_source._LaunchDescriptionSource__location).endswith(
            'navigation.launch.py')]
    arguments = {_text(context, k): _text(context, v) for k, v in navigation[0].launch_arguments}
    assert arguments['global_planner_plugin'] == 'nav2_smac_planner/SmacPlannerHybrid'
    # Where the Go2 can spin (Isaac), the mode default grid planner stays.
    context, actions = _build(robot_model='unitree_go2', simulator='isaac', mode='nav',
                              map_name='nav_obstacle')
    navigation = [a for a in _walk(actions) if isinstance(a, IncludeLaunchDescription) and _text(
        context, a.launch_description_source._LaunchDescriptionSource__location).endswith(
            'navigation.launch.py')]
    arguments = {_text(context, k): _text(context, v) for k, v in navigation[0].launch_arguments}
    assert arguments['global_planner_plugin'] == 'nav2_smac_planner/SmacPlanner2D'
