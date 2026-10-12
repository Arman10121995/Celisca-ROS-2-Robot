"""Learned-policy PD bridge for Isaac (robot_lab_isaac).

Isaac's pose hold did not keep legged robots upright (2026-09-23), so a
robot with a provisioned rl_sar policy now stands on PhysX drives that carry
the policy's own kp/kd/torque limits, fed with its joint targets.  These
checks cover the ROS-side contract; the physical stance and gait are native
Isaac trials.
"""
import json
import os
import sys
from unittest import mock

import pytest

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

CONFIG = dict(topic='/legged_policy/joint_targets', timeout_s=0.5,
              joints=['hip', 'knee', 'wheel'], mode=['position', 'position', 'velocity'],
              kp=[20.0, 20.0, 0.0], kd=[0.5, 0.5, 0.5], torque_limit=[23.7, 45.4, 20.0],
              default=[0.1, -1.5, 0.0])


def _spawner_module():
    pytest.importorskip('rclpy')
    from robot_lab_isaac import isaac_spawner
    return isaac_spawner


def test_targets_hold_the_default_pose_until_a_fresh_command():
    targets = _spawner_module().PolicyTargets(json.dumps(CONFIG))
    assert targets.targets(1.0) == {'position': {'hip': 0.1, 'knee': -1.5}, 'velocity': {'wheel': 0.0}}
    assert targets.receive([0.2, -1.2, 3.0], 1.0)
    assert targets.targets(1.4) == {'position': {'hip': 0.2, 'knee': -1.2}, 'velocity': {'wheel': 3.0}}
    # Command loss in simulated time: back to the standing pose.
    assert targets.targets(1.6)['position'] == {'hip': 0.1, 'knee': -1.5}


def test_malformed_commands_and_configs_are_rejected():
    module = _spawner_module()
    targets = module.PolicyTargets(CONFIG)
    assert not targets.receive([0.0, 0.0], 1.0)
    assert not targets.receive([0.0, float('nan'), 0.0], 1.0)
    assert targets.values is None
    with pytest.raises(ValueError):
        module.PolicyTargets(dict(CONFIG, kp=[1.0]))
    with pytest.raises(ValueError):
        module.PolicyTargets(dict(CONFIG, mode=['position', 'effort', 'velocity']))


def test_runtime_targets_split_position_and_velocity_joints():
    sys.path.insert(0, os.path.join(_SRC, 'robot_lab_isaac', 'python'))
    from robot_lab_isaac.isaac_runtime import joint_pd_targets
    assert joint_pd_targets(CONFIG) == {'position': {'hip': 0.1, 'knee': -1.5}, 'velocity': {'wheel': 0.0}}
    assert joint_pd_targets(CONFIG, [0.0, 0.0, 2.0])['velocity'] == {'wheel': 2.0}


def _launch_actions(**overrides):
    import importlib.util
    try:
        from launch import LaunchContext
        from launch.actions import DeclareLaunchArgument
    except ImportError as exc:  # pragma: no cover
        pytest.skip(str(exc))
    path = os.path.join(_SRC, 'robot_lab_bringup', 'launch', 'simulated_robot.launch.py')
    spec = importlib.util.spec_from_file_location('robot_lab_isaac_policy_launch', path)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
    except Exception as exc:  # pragma: no cover - workspace not installed
        pytest.skip('launch file needs the installed workspace: %s' % exc)
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
            context.launch_configurations[entity.name] = ''.join(
                part.perform(context) for part in entity.default_value)
    context.launch_configurations.update(dict(
        robot_model='unitree_go2', simulator='isaac', mode='loc', map_name='nav_empty',
        gui='false', start_rviz='false'), **overrides)
    # No run-id reaper process from a test; xacro still runs for real.
    with mock.patch.object(module, '_run_id_actions', lambda: []):
        return context, module._build_simulation_actions(context)


def _simulator_arguments(context, actions):
    from launch.actions import IncludeLaunchDescription
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

    def text(value):
        return perform_substitutions(context, normalize_to_list_of_substitutions(value))
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            arguments = {text(key): text(value) for key, value in action.launch_arguments}
            if 'robot_xacro' in arguments:
                return arguments
    return {}


def _policy_nodes(actions):
    from launch_ros.actions import Node
    return [a for a in actions if isinstance(a, Node)
            and getattr(a, '_Node__node_executable', '') == 'legged-policy-controller']


@pytest.mark.integration
def test_isaac_localization_selects_the_robots_policy_and_its_physics_rate():
    context, actions = _launch_actions()
    arguments = _simulator_arguments(context, actions)
    config = json.loads(arguments['joint_pd_config'])
    assert len(config['joints']) == 12
    assert float(arguments['physics_rate']) == pytest.approx(200.0)   # go2_himloco dt 0.005
    assert arguments['hold_position'] == 'false'
    assert len(_policy_nodes(actions)) == 1


@pytest.mark.integration
def test_isaac_policy_can_be_turned_off():
    context, actions = _launch_actions(legged_policy='none')
    arguments = _simulator_arguments(context, actions)
    assert 'joint_pd_config' not in arguments
    assert not _policy_nodes(actions)


@pytest.mark.integration
def test_gazebo_localization_runs_the_policy_through_an_effort_pd_controller():
    import yaml
    context, actions = _launch_actions(simulator='gazebo')
    nodes = _policy_nodes(actions)
    assert len(nodes) == 1
    from launch_ros.actions import Node
    spawners = [a for a in actions if isinstance(a, Node)
                and getattr(a, '_Node__node_executable', '') == 'spawner']
    arguments = [[''.join(p.perform(context) for p in argument) if not isinstance(argument, str) else argument
                  for argument in spawner._Node__arguments] for spawner in spawners]
    controller = [a for a in arguments if a[0] == 'legged_pd_controller']
    assert [a[0] for a in arguments] == ['joint_state_broadcaster', 'legged_pd_controller']
    assert controller[0][controller[0].index('-t')+1] == \
        'joint_trajectory_controller/JointTrajectoryController'
    params = yaml.safe_load(open(controller[0][controller[0].index('-p')+1]))
    gains = params['legged_pd_controller']['ros__parameters']['gains']
    assert len(gains) == 12 and all(g['p'] > 0 and g['d'] > 0 for g in gains.values())
    # The wheeled controller layer is not started for a policy-driven dog.
    from launch.actions import IncludeLaunchDescription
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert not any('controller.launch.py' in str(a.launch_description_source.location)
                   for a in includes)
