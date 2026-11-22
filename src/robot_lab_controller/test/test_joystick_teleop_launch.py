import importlib.util
import os

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def _description():
    path = os.path.join(os.path.dirname(__file__), '..', 'launch', 'joystick_teleop.launch.py')
    spec = importlib.util.spec_from_file_location('joystick_teleop_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _started(enable_joystick):
    """Entities that would start with enable_joystick set as given."""
    context = LaunchContext()
    context.launch_configurations['enable_joystick'] = enable_joystick
    context.launch_configurations['use_sim_time'] = 'true'
    return [entity for entity in _description().entities
            if isinstance(entity, (Node, IncludeLaunchDescription))
            and (entity.condition is None or entity.condition.evaluate(context))]


def test_disabling_the_joystick_keeps_the_velocity_mux_and_relay():
    """V11: enable_joystick:=false removed twist_mux, so Gazebo jobs never moved."""
    started = _started('false')
    assert any(isinstance(entity, IncludeLaunchDescription) for entity in started)   # twist_mux
    assert [entity.node_executable for entity in started if isinstance(entity, Node)] == ['twist_relay']


def test_the_joystick_nodes_start_by_default():
    executables = {entity.node_executable for entity in _started('true') if isinstance(entity, Node)}
    assert executables == {'joy_teleop', 'joy_node', 'twist_relay'}
