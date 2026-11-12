"""Focused optional source-plant checks, separate from walking qualification."""
import math
import os
from pathlib import Path

import numpy as np
import pytest
import yaml

mujoco = pytest.importorskip('mujoco')
from robot_lab_mujoco.octopod_control import NativeOctopodControl


@pytest.fixture
def plant():
    root = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))
    profiles = root/'external_assets/installed/robots.yaml'
    if not profiles.is_file():
        pytest.skip('Source octopod not provisioned on this host')
    cfg = yaml.safe_load(profiles.read_text())['robots'].get('asset_light_robotics_octopod')
    if not cfg:
        pytest.skip('Source octopod not provisioned on this host')
    spec = mujoco.MjSpec.from_file(cfg['native_mjcf'])
    spec.worldbody.add_geom(name='test_floor', type=mujoco.mjtGeom.mjGEOM_PLANE, size=[10,10,.1])
    model = spec.compile(); data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    controller = NativeOctopodControl(model, data, cfg['locomotion_policy_config'], cfg['native_mjcf'])
    return model, data, controller, cfg


def test_source_force_caps_without_root_pose_control(plant):
    model, data, control, _ = plant
    assert sum(model.body_mass) == pytest.approx(7.5, rel=1e-6)
    control.receive(.15, 0., .35)
    for _ in range(100):
        before = data.qpos.copy()
        control.before_step()
        assert np.array_equal(data.qpos, before)
        mujoco.mj_step(model, data); control.after_step()
        assert np.max(np.abs(data.actuator_force[control.actuators])) <= 2.45+1e-9


def test_wall_lease_lateral_refusal_and_fall_latch(plant):
    _, data, control, _ = plant
    control.receive(.1, 0., .2, now=100.)
    assert control.current_command(now=100.4) == pytest.approx([.1,0.,.2])
    assert control.current_command(now=100.501) == pytest.approx([0.,0.,0.])
    assert not control.receive(.1, .1, 0.)
    assert control.current_command() == pytest.approx([0.,0.,0.])
    data.qpos[control.base_q+3:control.base_q+7] = [math.cos(.6),math.sin(.6),0.,0.]
    control.before_step()
    assert control.fault
    assert not control.receive(.1,0.,0.)
    control.reset()
    assert not control.fault
    assert control.current_command() == pytest.approx([0.,0.,0.])


def test_reject_replacing_source_position_servo_with_torque_motor(plant):
    model, data, control, cfg = plant
    model.actuator_biasprm[control.actuators[0], 1] = 0.
    with pytest.raises(ValueError, match='position servo'):
        NativeOctopodControl(model, data, cfg['locomotion_policy_config'], cfg['native_mjcf'])


def test_selected_world_does_not_rescale_original_robot(plant):
    model, _, _, cfg = plant
    spec = mujoco.MjSpec.from_file(cfg['native_mjcf'])
    # A large fixed obstacle has an inferred mass in MuJoCo too. It must not
    # participate in the source compiler's original 7.5 kg global rescale.
    obstacle = spec.worldbody.add_body(name='world_obstacle', pos=[5,5,5])
    obstacle.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[1,1,1])
    selected = spec.compile()
    assert selected.body_mass[:model.nbody] == pytest.approx(model.body_mass)
    assert selected.body_inertia[:model.nbody] == pytest.approx(model.body_inertia)
