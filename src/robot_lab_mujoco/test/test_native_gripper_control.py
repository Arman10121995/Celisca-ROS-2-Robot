"""Physical transmission/force counterexamples; these are not robot missions."""
from types import SimpleNamespace
import time

import numpy as np
import pytest

mj = pytest.importorskip('mujoco')
pytest.importorskip('control_msgs.action')
from control_msgs.action import GripperCommand
import robot_lab_mujoco.native_gripper_control as gripper_module


@pytest.fixture
def control(monkeypatch):
    model = mj.MjModel.from_xml_string('''<mujoco><compiler autolimits="true"/>
    <option timestep=".002" gravity="0 0 0"/>
    <worldbody><body name="left"><joint name="finger_joint1" type="slide" axis="0 1 0" range="0 .04"/>
    <geom type="box" size=".01 .004 .01" mass=".015"/></body>
    <body name="right"><joint name="finger_joint2" type="slide" axis="0 -1 0" range="0 .04"/>
    <geom type="box" size=".01 .004 .01" mass=".015"/></body></worldbody>
    <tendon><fixed name="split"><joint joint="finger_joint1" coef=".5"/>
    <joint joint="finger_joint2" coef=".5"/></fixed></tendon>
    <equality><joint joint1="finger_joint1" joint2="finger_joint2"/></equality>
    <actuator><general name="actuator8" tendon="split" ctrlrange="0 255" forcerange="-100 100"
    gainprm=".01568627451" biastype="affine" biasprm="0 -100 -10"/></actuator></mujoco>''')
    data = mj.MjData(model)
    data.qpos[:] = .04
    mj.mj_forward(model, data)
    messages = []
    node = SimpleNamespace(model=model, data=data, mujoco=mj, arm=SimpleNamespace(reserved=False),
        create_publisher=lambda *a: SimpleNamespace(publish=messages.append),
        create_subscription=lambda *a: None, create_service=lambda *a: None,
        get_logger=lambda: SimpleNamespace(warn=lambda *a: None))
    monkeypatch.setattr(gripper_module, 'ActionServer', lambda *a, **k: None)
    adapter = gripper_module.NativePandaGripper(node)
    return adapter


def test_original_half_force_transmission_is_bounded_during_real_steps(control):
    control.target = 0.
    control.force_limit = .1
    control.last_heartbeat = time.monotonic()
    start = control.opening()
    efforts = []
    for _ in range(500):
        control.before_step()
        mj.mj_step(control.model, control.data)
        efforts.append(control.effort())
    assert control.opening() < start-.005
    assert max(efforts) <= .1+1e-8
    assert abs(control.data.qpos[0]-control.data.qpos[1]) < 1e-6
    assert np.isfinite(control.data.qpos).all()


@pytest.mark.parametrize('property', ['coefficient', 'gain', 'bounds'])
def test_changed_native_transmission_cannot_reuse_panda_adapter(control, property):
    if property == 'coefficient':
        control.model.wrap_prm[0] = .6
    elif property == 'gain':
        control.model.actuator_gainprm[0, 0] = 1.
    else:
        control.model.actuator_ctrlrange[0, 1] = 1.
    with pytest.raises(ValueError, match='transmission'):
        gripper_module.NativePandaGripper(control.node)


@pytest.mark.parametrize('position,effort', [(float('nan'), .5), (-.01, .5), (.081, .5),
                                          (.04, float('inf')), (.04, -1.), (.04, 21.)])
def test_invalid_gap_or_force_is_rejected_before_actuation(control, position, effort):
    from rclpy.action import GoalResponse
    control.last_heartbeat = time.monotonic()
    goal = GripperCommand.Goal()
    goal.command.position, goal.command.max_effort = position, effort
    before = control.data.ctrl.copy()
    assert control.accept(goal) == GoalResponse.REJECT
    assert np.array_equal(before, control.data.ctrl)


def test_stop_and_heartbeat_loss_hold_measured_gap(control):
    control.last_heartbeat = time.monotonic()
    control.target = 0.
    control.data.qpos[:] = .015
    response = SimpleNamespace()
    control.stop(None, response)
    assert response.success and control.target == pytest.approx(.03)
    control.target = .08
    control.last_heartbeat = -float('inf')
    control.before_step()
    assert control.target == pytest.approx(.03)
    assert 'heartbeat lost' in control.status
