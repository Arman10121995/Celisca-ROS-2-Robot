"""Articulated commands must reach named MuJoCo joints, with real limits/losses."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

SRC = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SRC / 'robot_lab_mujoco/python'))
from robot_lab_mujoco.joint_effort import JointEffortCommand


@pytest.fixture
def command(tmp_path):
    config = {'controller_manager': {'ros__parameters': {
        'effort': {'type': 'effort_controllers/JointGroupEffortController'}}},
        'effort': {'ros__parameters': {'joints': ['right', 'left']}}}
    path = tmp_path / 'controller.yaml'
    path.write_text(yaml.safe_dump(config))
    urdf = '<robot>' + ''.join(
        f'<joint name="{name}" type="revolute"><limit effort="{limit}"/>'
        '<dynamics friction="0.1" damping="0.02"/></joint>'
        for name, limit in [('left', 2), ('right', 3)]) + '</robot>'
    return JointEffortCommand(path, urdf)


def test_config_order_and_limit_clamping(command):
    assert command.names == ('right', 'left')
    assert command.topic == '/effort/commands'
    assert command.receive([10, -10], 1.)
    np.testing.assert_equal(command.command(1.1, .5), [3, -2])


@pytest.mark.parametrize('values', [[1], [1, 2, 3], [float('nan'), 1], [float('inf'), 1], [[1, 2]]])
def test_bad_command_clears_previous_efforts(command, values):
    command.receive([2, 1], 1.)
    assert not command.receive(values, 1.1)
    np.testing.assert_equal(command.command(1.2, .5), [0, 0])


def test_watchdog_and_reset(command):
    np.testing.assert_equal(command.command(0., .5), [0, 0])
    command.receive([2, 1], 1.)
    np.testing.assert_equal(command.command(1.6, .5), [0, 0])
    command.receive([2, 1], 2.)
    command.clear()
    np.testing.assert_equal(command.command(2.1, .5), [0, 0])


def test_actuators_apply_named_torque_and_preserve_declared_losses(command):
    mujoco = pytest.importorskip('mujoco')
    # XML order deliberately differs from command order.
    xml = '<mujoco><worldbody>' + ''.join(
        f'<body pos="{x} 0 1"><joint name="{name}"/>'
        '<geom type="sphere" size=".1"/></body>'
        for name, x in [('left', 0), ('right', 1)]) + '</worldbody></mujoco>'
    model = mujoco.MjModel.from_xml_string(command.add_actuators(xml))
    data = mujoco.MjData(model)
    data.ctrl[:] = [1.5, -.7]
    mujoco.mj_forward(model, data)
    for name, force in [('right', 1.5), ('left', -.7)]:
        dof = model.jnt_dofadr[model.joint(name).id]
        assert data.qfrc_actuator[dof] == pytest.approx(force)
        assert model.dof_frictionloss[dof] == pytest.approx(.1)
        assert model.dof_damping[dof] == pytest.approx(.02)
    assert np.all(model.actuator_forcelimited)


def test_existing_actuator_is_rejected(command):
    xml = ('<mujoco><worldbody><body><joint name="left"/><joint name="right"/>'
           '</body></worldbody><actuator><motor joint="left"/></actuator></mujoco>')
    with pytest.raises(ValueError, match='already-actuated'):
        command.add_actuators(xml)


def test_missing_configured_joint_is_rejected(command):
    with pytest.raises(ValueError, match='missing'):
        command.add_actuators('<mujoco><worldbody/></mujoco>')
