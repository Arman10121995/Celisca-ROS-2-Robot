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


# ----------------------------------------------------------------------
# Native joint losses read from the robot's own MJCF
# ----------------------------------------------------------------------
MJCF_WITH_CLASSES = '''<mujoco model="probe">
  <default>
    <joint frictionloss="0.5"/>
    <default class="robot">
      <joint armature="0.005"/>
      <default class="nested">
        <joint damping="0.25"/>
      </default>
    </default>
  </default>
  <worldbody>
    <body name="b0" childclass="robot">
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/><joint name="j0"/>
    </body>
    <body name="b1" childclass="nested">
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/><joint name="j1"/>
    </body>
    <body name="b2" childclass="robot">
      <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
      <body name="b3" pos="0 0 0.1">
        <inertial pos="0 0 0" mass="1" diaginertia="0.1 0.1 0.1"/>
        <joint name="j2" damping="0.75"/>
      </body>
    </body>
  </worldbody>
</mujoco>'''


def test_mjcf_joint_dynamics_matches_the_compiler():
    """The reader must agree with MuJoCo's own class/default resolution."""
    mujoco = pytest.importorskip('mujoco')
    from robot_lab_mujoco.joint_effort import mjcf_joint_dynamics
    resolved = mjcf_joint_dynamics(MJCF_WITH_CLASSES)
    model = mujoco.MjModel.from_xml_string(MJCF_WITH_CLASSES)
    assert set(resolved) == {'j0', 'j1', 'j2'}
    for name, losses in resolved.items():
        dof = model.jnt_dofadr[model.joint(name).id]
        assert losses['frictionloss'] == pytest.approx(model.dof_frictionloss[dof])
        assert losses['armature'] == pytest.approx(model.dof_armature[dof])
        assert losses['damping'] == pytest.approx(model.dof_damping[dof])
    # The class chain is what makes the values differ per joint.
    assert resolved['j0'] == {'frictionloss': .5, 'armature': .005, 'damping': 0.}
    assert resolved['j1']['damping'] == pytest.approx(.25)
    assert resolved['j2']['damping'] == pytest.approx(.75)


def test_add_actuators_native_losses_replace_the_urdf_damping(command):
    """A Gazebo-substitute URDF damping must not reach a native-loss plant."""
    mujoco = pytest.importorskip('mujoco')
    xml = ('<mujoco><worldbody>'
           '<body pos="0 0 1"><joint name="left"/><geom type="sphere" size=".1"/></body>'
           '<body pos="1 0 1"><joint name="right"/><geom type="sphere" size=".1"/></body>'
           '</worldbody></mujoco>')
    native = {name: {'frictionloss': .1, 'armature': .005, 'damping': 0.}
              for name in ('right', 'left')}
    model = mujoco.MjModel.from_xml_string(command.add_actuators(xml, native))
    for name in ('right', 'left'):
        dof = model.jnt_dofadr[model.joint(name).id]
        assert model.dof_frictionloss[dof] == pytest.approx(.1)
        assert model.dof_armature[dof] == pytest.approx(.005)
        assert model.dof_damping[dof] == pytest.approx(0.)


def test_native_losses_must_cover_every_configured_joint(command):
    with pytest.raises(ValueError, match='incomplete'):
        command.joint_losses({'left': {'frictionloss': .1}})


def test_native_loss_resolution_prefers_the_full_robot_mjcf():
    """The BHL resolver picks the file naming every effort joint, not a scene."""
    from robot_lab_mujoco.mujoco_spawner import _native_joint_dynamics
    from robot_lab_adapter.bhl_balance import BHL_JOINT_NAMES
    xacro = (SRC / 'robot_lab_robots/berkeley_humanoid_lite/xacro/'
             'bhl_sim.xacro')
    if not xacro.exists():  # pragma: no cover - vendored description
        pytest.skip(f'BHL description not present at {xacro}')
    path, losses = _native_joint_dynamics(str(xacro), BHL_JOINT_NAMES)
    assert path is not None, 'no MJCF beside the BHL description names the joints'
    assert path.endswith('berkeley_humanoid_lite.xml'), path
    for name in BHL_JOINT_NAMES:
        assert losses[name] == {'frictionloss': pytest.approx(.1),
                                'armature': pytest.approx(.005),
                                'damping': pytest.approx(0.)}


def test_native_loss_resolution_is_absent_without_a_mjcf(tmp_path):
    from robot_lab_mujoco.mujoco_spawner import _native_joint_dynamics
    assert _native_joint_dynamics(str(tmp_path / 'robot.urdf'), ['a']) == (None, None)
