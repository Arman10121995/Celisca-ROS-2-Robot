"""Node planners drive through the navigator (robot_lab_algorithms.nav2_adapters).

Selecting rrt/rrt*/prm/voronoi or follow_the_gap/regulated_pure_pursuit used
to start a side node publishing /plan/<id> or /cmd_vel_<id> while Nav2's
plugin planned and drove.  Now the selection starts an action adapter and
the navigator's behavior tree calls it by server_name.
"""
import importlib.util
import os
from unittest import mock

import pytest
import yaml

_BRINGUP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SRC = os.path.dirname(_BRINGUP)
DISPATCH = yaml.safe_load(open(os.path.join(_BRINGUP, 'config', 'algorithm_dispatch.yaml')))['algorithms']


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('category,action,executable,ids', [
    ('global_planning', 'compute_path_to_pose', 'nav2_global_planner',
     ['rrt_planner', 'rrt_star_planner', 'prm_planner', 'voronoi_planner']),
    ('local_planning', 'follow_path', 'nav2_local_planner',
     ['follow_the_gap', 'regulated_pure_pursuit']),
])
def test_node_planners_serve_a_nav2_action(category, action, executable, ids):
    for algorithm in ids:
        entry = DISPATCH[category][algorithm]
        assert entry['node']['executable'] == executable
        assert entry['node']['parameters'] == {'planner': algorithm}
        assert entry['serves'][action].startswith('/robot_lab/')


def test_placeholder_mppi_node_is_not_offered_as_a_planner():
    assert 'placeholder' in DISPATCH['local_planning']['mppi_local_planner']['unavailable']


def test_navigation_tree_is_pointed_at_the_adapter_servers(tmp_path):
    try:
        module = _load(os.path.join(_SRC, 'robot_lab_navigation', 'launch', 'navigation.launch.py'),
                       'robot_lab_navigation_adapter_tree')
    except ImportError as exc:  # pragma: no cover - ROS launch missing
        pytest.skip(str(exc))
    tree = os.path.join(_SRC, 'robot_lab_navigation', 'behavior_tree',
                        'simple_navigation_w_replanning_and_recovery.xml')
    assert module._tree_with_action_servers(tree) == tree
    text = open(module._tree_with_action_servers(
        tree, '/robot_lab/compute_path_to_pose', '/robot_lab/follow_path')).read()
    assert '<ComputePathToPose server_name="/robot_lab/compute_path_to_pose"' in text
    assert '<FollowPath server_name="/robot_lab/follow_path"' in text
    only = open(module._tree_with_action_servers(tree, '', '/robot_lab/follow_path')).read()
    assert 'server_name="/robot_lab/compute_path_to_pose"' not in only


@pytest.mark.integration
@pytest.mark.parametrize('overrides,argument,server,planner', [
    ({'global_planning': 'rrt_star_planner'}, 'compute_path_server',
     '/robot_lab/compute_path_to_pose', 'rrt_star_planner'),
    ({'local_planning': 'follow_the_gap'}, 'follow_path_server',
     '/robot_lab/follow_path', 'follow_the_gap'),
])
def test_launch_starts_the_adapter_and_routes_the_navigator(overrides, argument, server, planner):
    try:
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
        from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
        from launch_ros.actions import Node
        module = _load(os.path.join(_BRINGUP, 'launch', 'simulated_robot.launch.py'),
                       'robot_lab_adapter_dispatch_launch')
        description = module.generate_launch_description()
    except Exception as exc:  # pragma: no cover - workspace not installed
        pytest.skip('launch file needs the installed workspace: %s' % exc)
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
            context.launch_configurations[entity.name] = ''.join(
                part.perform(context) for part in entity.default_value)
    context.launch_configurations.update(dict(
        robot_model='bumperbot', simulator='mujoco', mode='nav', map_name='nav_maze',
        gui='false', start_rviz='false'), **overrides)
    with mock.patch.object(module, '_run_id_actions', lambda: []):
        actions = module._build_simulation_actions(context)

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))

    def walk(items):
        # Navigation and node planners start behind the simulator-ready gate
        # (RegisterEventHandler(OnProcessExit(...))).
        for item in items:
            yield item
            handler = getattr(item, '_RegisterEventHandler__event_handler', None)
            nested = getattr(handler, '_OnActionEventBase__actions_on_event', None)
            if isinstance(nested, (list, tuple)):
                yield from walk(nested)
    every = list(walk(actions))
    def location(include):
        return text(include.launch_description_source._LaunchDescriptionSource__location)
    navigation = [a for a in every if isinstance(a, IncludeLaunchDescription)
                  and location(a).endswith('navigation.launch.py')]
    assert navigation, 'no navigation include'
    arguments = {text(k): text(v) for k, v in navigation[0].launch_arguments}
    assert arguments[argument] == server
    adapters = [a for a in every if isinstance(a, Node)
                and getattr(a, '_Node__node_executable', '') in ('nav2_global_planner', 'nav2_local_planner')]
    assert adapters, 'adapter node not started'
    parameters = adapters[0]._Node__parameters
    flat = {}
    for entry in parameters:
        if isinstance(entry, dict):
            for key, value in entry.items():
                key = key if isinstance(key, str) else text(key)
                flat[key] = value if isinstance(value, (bool, int, float)) else text(value)
    # launch_ros renders parameter values as YAML documents ('x\n...\n').
    assert flat['planner'].split('\n')[0] == planner
