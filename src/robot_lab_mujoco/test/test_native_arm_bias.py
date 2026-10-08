"""Gravity-loaded position servos: precision without bypassing native limits.

This small synchronous physics fixture is not a Panda mission qualification.
"""
from types import SimpleNamespace

import numpy as np
import pytest

mj = pytest.importorskip('mujoco')
pytest.importorskip('control_msgs.action')
from robot_lab_mujoco import native_arm_control as arm_module


@pytest.fixture
def control(monkeypatch):
    bodies = ''.join('<body pos=".1 0 0"><joint name="joint%d" type="hinge" axis="0 1 0" '
                     'range="-2 2"/><geom type="capsule" fromto="0 0 0 .1 0 0" '
                     'size=".01" mass=".1" contype="0" conaffinity="0"/>' % i
                     for i in range(1, 8))+('</body>'*7)
    actuators = ''.join('<general name="actuator%d" joint="joint%d" '
                       'gainprm="600" biastype="affine" biasprm="0 -600 -50" '
                       'ctrlrange="-2 2" forcerange="-50 50"/>' % (i, i)
                       for i in range(1, 8))
    model = mj.MjModel.from_xml_string('<mujoco><compiler autolimits="true"/>'
        '<option timestep=".002" integrator="implicitfast"/><worldbody>'+bodies+
        '</worldbody><actuator>'+actuators+'</actuator></mujoco>')
    data = mj.MjData(model); mj.mj_forward(model, data)
    node = SimpleNamespace(model=model, data=data, mujoco=mj, robot_joints=7, robot_bodies=8,
        create_publisher=lambda *a: SimpleNamespace(publish=lambda *a: None),
        create_subscription=lambda *a: None, create_service=lambda *a: None)
    monkeypatch.setattr(arm_module, 'ActionServer', lambda *a, **k: None)
    return arm_module.NativePandaControl(node)


def test_bias_feedforward_reduces_physical_sag_through_original_actuators(control):
    model, data = control.model, control.data
    gain, bias, force = [getattr(model, key).copy() for key in
                         ('actuator_gainprm', 'actuator_biasprm', 'actuator_forcerange')]
    baseline = mj.MjData(model); mj.mj_forward(model, baseline)
    for _ in range(1000):
        mj.mj_step(model, baseline)
    baseline_error = np.max(np.abs(baseline.qpos))
    assert baseline_error > .001
    maximum_force = 0.
    for _ in range(1000):
        control.before_step(); mj.mj_step(model, data)
        maximum_force = max(maximum_force, np.max(np.abs(data.actuator_force)))
    assert np.max(np.abs(data.qpos)) < baseline_error*.1
    assert np.max(np.abs(control.control_offset)) <= .03
    assert maximum_force <= 50.+1e-8
    np.testing.assert_array_equal(data.qfrc_applied, np.zeros(model.nv))
    for key, original in zip(('actuator_gainprm', 'actuator_biasprm', 'actuator_forcerange'), (gain, bias, force)):
        np.testing.assert_array_equal(getattr(model, key), original)


def test_compensation_clamps_control_and_nonfinite_bias_never_reaches_actuator(control):
    control.target[:] = 3.
    control.data.qfrc_bias[:] = 1e9
    control.before_step()
    assert np.all(control.data.ctrl <= 2.)
    assert np.max(np.abs(control.control_offset)) <= .03
    control.target[:] = 0.
    control.data.qfrc_bias[0] = float('nan')
    control.before_step()
    np.testing.assert_array_equal(control.data.ctrl, np.zeros(7))


@pytest.mark.parametrize('parameter', ['gear', 'gain', 'position_bias'])
def test_changed_transmission_cannot_reuse_bias_control(control, parameter):
    if parameter == 'gear':
        control.model.actuator_gear[0, 0] = 2.
    elif parameter == 'gain':
        control.model.actuator_gainprm[0, 0] = 0.
    else:
        control.model.actuator_biasprm[0, 1] = -300.
    with pytest.raises(ValueError, match='position actuators'):
        arm_module.NativePandaControl(control.node)
