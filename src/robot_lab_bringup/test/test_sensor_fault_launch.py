"""Actual ROS launch routing contracts, without starting simulator processes."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch_ros.actions import Node

from robot_lab_utils.sensor_fault_relay import launch_actions

ROOT=Path(__file__).resolve().parents[3]


def scenario(tmp_path,monkeypatch):
    monkeypatch.setenv('ROBOT_LAB_RUNTIME_ROOT',str(tmp_path))
    path=tmp_path/'faults.yaml'
    path.write_text(yaml.safe_dump(dict(seed=3,relays=[dict(input='/scan_clean',output='/scan',type='LaserScan')])) )
    return str(path)


def test_launch_actions_keep_trace_names_local_and_do_not_reuse_a_writer(tmp_path,monkeypatch):
    path=scenario(tmp_path,monkeypatch)
    remaps,first=launch_actions(path,'../../robot/one',True)
    _,second=launch_actions(path,'../../robot/one',True)
    assert remaps==[('/scan','/scan_clean')]
    assert first.node_executable==second.node_executable=='sensor_fault_relay.py'
    assert (tmp_path/'sensor_faults').is_dir()
    context=LaunchContext()
    context.extend_locals({'ros_specific_arguments':{'name':'__node:=sensor_fault_relay'}})
    first._perform_substitutions(context);second._perform_substitutions(context)
    from launch.utilities import perform_substitutions
    a=[perform_substitutions(context,arg) for arg in first.cmd]
    b=[perform_substitutions(context,arg) for arg in second.cmd]
    traces=[Path(args[args.index('--trace')+1]) for args in (a,b)]
    assert traces[0]!=traces[1] and all(p.parent==tmp_path/'sensor_faults' for p in traces)


def test_native_mujoco_sensor_publishers_cannot_bypass_the_relay(tmp_path,monkeypatch):
    path=scenario(tmp_path,monkeypatch)
    spec=importlib.util.spec_from_file_location('mujoco_fault_launch',ROOT/'src/robot_lab_mujoco/launch/mujoco_simulator.launch.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'_resolve_mujoco_xml',lambda *args:str(tmp_path/'world.xml'))
    context=LaunchContext()
    values=dict(model=str(tmp_path/'model.urdf'),robot_name='stretch',native_mjcf=str(tmp_path/'native.xml'),
        arm_control='none',locomotion_policy_config='',mobile_control_config='',native_task='display',
        articulation_control='false',hand_adapter='',arm_planning_config='',grasp_fixture='false',gui='false',
        hold_position='auto',sensor_faults=path,use_sim_time='true',spawn_x='0',spawn_y='0',spawn_z='0',spawn_yaw='0')
    context.launch_configurations.update(values)
    nodes=module._build_mujoco_actions(context)
    assert [node.node_executable for node in nodes]==['sensor_fault_relay.py','native_asset_display']
    nodes[-1]._perform_substitutions(context)
    assert nodes[-1].expanded_remapping_rules==[('/scan','/scan_clean')]
