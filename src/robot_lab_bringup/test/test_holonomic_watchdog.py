"""Lease expiry must remove wheel targets even with a frozen ROS clock."""
from pathlib import Path
import sys
from unittest.mock import Mock

import pytest
from rclpy.time import Time

ROOT=Path(__file__).resolve().parents[3]
for package in ('robot_lab_controller','robot_lab_utils'):sys.path.insert(0,str(ROOT/'src'/package))
from robot_lab_controller import holonomic_controller as module
from robot_lab_utils.drive_kinematics import drive_from_config


@pytest.mark.parametrize('kind',['mecanum','four_wheel_steer'])
def test_stale_wall_lease_removes_motion_without_recentering_steering(monkeypatch,kind):
    config=dict(type=kind,front_left_wheel_joint='fl',front_right_wheel_joint='fr',
        rear_left_wheel_joint='rl',rear_right_wheel_joint='rr',max_accel=.4,
        front_left_steer_joint='sfl',front_right_steer_joint='sfr',rear_left_steer_joint='srl',rear_right_steer_joint='srr')
    node=module.HolonomicController.__new__(module.HolonomicController)
    node._drive=drive_from_config(config)
    moving=node._drive.targets(.25,.3,dt=1.)
    assert any(abs(value)>0 for value in moving.velocity.values())
    node._wheel_joints=node._drive.wheel_joints;node._steer_joints=node._drive.steer_joints
    node._wheel_pub=Mock();node._steer_pub=Mock() if node._steer_joints else None
    node._timeout=.5;node._rate=50.;node._command=(.25,0.,.3)
    node._command_time=Time(seconds=10.);node._command_wall=98.
    node._last_step=Time(seconds=10.);node.get_clock=Mock(return_value=Mock(now=lambda:Time(seconds=10.)))
    monkeypatch.setattr(module.time,'monotonic',lambda:100.)
    node._step()
    assert list(node._wheel_pub.publish.call_args.args[0].data)==[0.]*4
    if node._steer_joints:
        assert list(node._steer_pub.publish.call_args.args[0].data)==[moving.position[j] for j in node._steer_joints]
