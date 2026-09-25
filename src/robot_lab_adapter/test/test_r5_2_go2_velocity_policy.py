"""Policy observation, joint order and effort boundary contracts."""

import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

pkg = Path(__file__).resolve().parents[1]
if str(pkg) not in sys.path:
    sys.path.insert(0, str(pkg))

from robot_lab_adapter.go2_locomotion import BaseVelocity, JOINT_NAMES
from robot_lab_adapter.go2_velocity_policy import (
    Go2VelocityPolicy, POLICY_DEFAULT, POLICY_KP, POLICY_KD,
    POLICY_ACTION_SCALE, projected_gravity,
    policy_forward_command,
)


class Session:
    def __init__(self, action=None):
        self.observation = None
        self.action = np.zeros((1, 12), dtype=np.float32) if action is None else action

    def get_inputs(self):
        return [SimpleNamespace(name="obs", shape=[1, 45])]

    def get_outputs(self):
        return [SimpleNamespace(name="actions", shape=[1, 12])]

    def run(self, _outputs, feeds):
        self.observation = feeds["obs"]
        return [self.action]


def test_observation_order_and_action_targets():
    session = Session(np.ones((1, 12), dtype=np.float32) * 0.2)
    policy = Go2VelocityPolicy("unused", session=session)
    q = dict(zip(JOINT_NAMES, POLICY_DEFAULT))
    dq = {name: float(i) for i, name in enumerate(JOINT_NAMES)}
    target = policy.step(q, dq, (0.1, 0.2, 0.3), (0, 0, 0, 1),
                         BaseVelocity(vx=0.25, wz=-0.1))
    obs = session.observation
    assert obs.shape == (1, 45)
    np.testing.assert_allclose(obs[0, :9],
                               [0.1, 0.2, 0.3, 0, 0, -1, 0.25, 0, -0.1])
    np.testing.assert_allclose(obs[0, 9:21], 0)
    np.testing.assert_allclose(obs[0, 21:33], range(12))
    np.testing.assert_allclose(obs[0, 33:], 0)
    np.testing.assert_allclose(target, POLICY_DEFAULT + 0.1, atol=1e-6)


def test_effort_is_bounded_and_missing_joint_is_not_driven():
    policy = Go2VelocityPolicy("unused", session=Session())
    policy.target[:] = 3.0
    efforts = policy.efforts({name: 0.0 for name in JOINT_NAMES},
                             {name: 0.0 for name in JOINT_NAMES})
    assert efforts["FL_hip_joint"] == 23.7
    assert efforts["FL_calf_joint"] == 35.55
    assert policy.efforts({}, {})["FL_hip_joint"] == 0.0


def test_invalid_observation_and_action_fail_closed():
    policy = Go2VelocityPolicy("unused", session=Session())
    with pytest.raises(ValueError, match="all 12"):
        policy.step({}, {}, (0, 0, 0), (0, 0, 0, 1), BaseVelocity())
    session = Session(np.full((1, 12), np.nan, dtype=np.float32))
    policy = Go2VelocityPolicy("unused", session=session)
    q = dict(zip(JOINT_NAMES, POLICY_DEFAULT))
    v = {name: 0.0 for name in JOINT_NAMES}
    with pytest.raises(ValueError, match="non-finite"):
        policy.step(q, v, (0, 0, 0), (0, 0, 0, 1), BaseVelocity())


def test_projected_gravity_is_body_frame():
    np.testing.assert_allclose(projected_gravity(0, 0, 0, 1), [0, 0, -1])
    np.testing.assert_allclose(projected_gravity(0, 1, 0, 0), [0, 0, 1])


def test_reverse_policy_maps_are_continuous_bounded_and_selectable():
    assert policy_forward_command(0.25) == 0.25
    assert policy_forward_command(0.0) == 0.0
    assert policy_forward_command(-0.25) == pytest.approx(-0.55)
    assert policy_forward_command(-0.8) == pytest.approx(-0.99)
    assert policy_forward_command(-0.001) < 0.0
    assert policy_forward_command(-0.1, "inverse") == pytest.approx(-0.207, abs=1e-3)
    assert policy_forward_command(-0.2, "inverse") == pytest.approx(-0.414, abs=1e-3)
    assert policy_forward_command(-0.25, "inverse") == pytest.approx(-0.462, abs=1e-3)
    assert policy_forward_command(-0.45, "inverse") == pytest.approx(-0.655, abs=1e-3)
    with pytest.raises(ValueError, match="reverse_map"):
        policy_forward_command(0.1, "invalid")
    with pytest.raises(ValueError, match="reverse_map"):
        Go2VelocityPolicy("unused", session=Session(), reverse_map="invalid")


def test_inverse_policy_map_reaches_the_observation():
    session = Session()
    policy = Go2VelocityPolicy("unused", session=session, reverse_map="inverse")
    q = dict(zip(JOINT_NAMES, POLICY_DEFAULT))
    dq = {name: 0.0 for name in JOINT_NAMES}
    policy.step(q, dq, (0.0, 0.0, 0.0), (0, 0, 0, 1), BaseVelocity(vx=-0.25))
    assert session.observation[0, 6] == pytest.approx(-0.462, abs=1e-3)


def test_bundled_policy_and_adapter_match_deploy_contract():
    policy_dir = pkg / "policies" / "go2_velocity_flat"
    deploy = yaml.safe_load((policy_dir / "deploy.yaml").read_text())
    assert deploy["joint_ids_map"] == [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
    assert deploy["step_dt"] == 0.02
    np.testing.assert_allclose(deploy["default_joint_pos"], POLICY_DEFAULT)
    np.testing.assert_allclose(deploy["stiffness"], POLICY_KP)
    np.testing.assert_allclose(deploy["damping"], POLICY_KD)
    assert set(deploy["actions"]["JointPositionAction"]["scale"]) == {
        POLICY_ACTION_SCALE}
    assert (policy_dir / "policy.onnx").is_file()
    assert (policy_dir / "policy.onnx.data").is_file()


def test_bundled_onnx_model_accepts_45_observations():
    pytest.importorskip("onnxruntime")
    path = pkg / "policies/go2_velocity_flat/policy.onnx"
    policy = Go2VelocityPolicy(str(path))
    q = dict(zip(JOINT_NAMES, POLICY_DEFAULT))
    v = {name: 0.0 for name in JOINT_NAMES}
    target = policy.step(q, v, (0, 0, 0), (0, 0, 0, 1), BaseVelocity())
    assert len(target) == 12
    assert np.isfinite(target).all()
