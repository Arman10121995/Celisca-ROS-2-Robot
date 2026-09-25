"""NJU-RLC get-up observation and bounded effort contract."""

from pathlib import Path
import sys

import numpy as np
import pytest

_package = Path(__file__).resolve().parents[1]
if str(_package) not in sys.path:
    sys.path.insert(0, str(_package))

from robot_lab_adapter.go2_locomotion import BodyState, JOINT_NAMES, EFFORT_LIMITS, joint_kind
from robot_lab_adapter.go2_recovery_policy import (
    Go2RecoveryPolicy, RECOVERY_ACTION_ABS_LIMIT, RECOVERY_DEFAULT,
)


class _Tensor:
    def __init__(self, name, shape):
        self.name, self.shape = name, shape


class _Session:
    def __init__(self, action=None):
        self.action = np.zeros((1, 12), dtype=np.float32) if action is None else action
        self.seen = []

    def get_inputs(self):
        return [_Tensor("proprio", [1, 57]), _Tensor("history", [1, 570])]

    def get_outputs(self):
        return [_Tensor("joint_action", [1, 12])]

    def run(self, _outputs, values):
        self.seen.append({name: value.copy() for name, value in values.items()})
        return [self.action]


def _measurements():
    return ({name: float(value) for name, value in zip(JOINT_NAMES, RECOVERY_DEFAULT)},
            {name: 0.0 for name in JOINT_NAMES})


def test_observation_order_and_previous_frame_history():
    session = _Session()
    policy = Go2RecoveryPolicy("unused", session=session)
    q, dq = _measurements()
    policy.step(q, dq, (1.0, 2.0, 3.0), BodyState(0.1, -0.2, 0.14),
                {"FL": 20.0, "FR": 0.0, "RL": 0.0, "RR": 0.0})
    first = session.seen[0]
    prop = first["proprio"].reshape(-1)
    assert prop.shape == (57,)
    np.testing.assert_allclose(prop[:5], [0.1, -0.2, 0.25, 0.5, 0.75])
    np.testing.assert_allclose(prop[5:41], 0.0)
    np.testing.assert_allclose(prop[41:45], [0.5, -0.5, -0.5, -0.5])
    np.testing.assert_allclose(prop[45:], 0.0)
    np.testing.assert_allclose(first["history"], 0.0)
    policy.step(q, dq, (0.0, 0.0, 0.0), BodyState(0.0, 0.0, 0.14),
                {leg: 0.0 for leg in ("FL", "FR", "RL", "RR")})
    history = session.seen[1]["history"].reshape(10, 57)
    np.testing.assert_allclose(history[-1], prop)
    np.testing.assert_allclose(history[:-1], 0.0)


def test_actions_produce_bounded_targets_and_efforts():
    session = _Session(np.full((1, 12), 100.0, dtype=np.float32))
    policy = Go2RecoveryPolicy("unused", session=session)
    q, dq = _measurements()
    target = policy.step(q, dq, (0.0, 0.0, 0.0), BodyState(0.0, 0.0, 0.14),
                         {leg: 0.0 for leg in ("FL", "FR", "RL", "RR")})
    assert target.shape == (12,)
    assert max(abs(policy.last_action)) == RECOVERY_ACTION_ABS_LIMIT
    efforts = policy.efforts(q, dq)
    assert set(efforts) == set(JOINT_NAMES)
    assert all(abs(value) <= EFFORT_LIMITS[joint_kind(name)]
               for name, value in efforts.items())
    assert efforts["FL_thigh_joint"] > 0.0


def test_missing_direct_contact_rejects_policy_step():
    policy = Go2RecoveryPolicy("unused", session=_Session())
    q, dq = _measurements()
    with pytest.raises(ValueError, match="direct foot forces"):
        policy.step(q, dq, (0, 0, 0), BodyState(0, 0), None)


def test_observation_and_action_are_bounded_and_nonfinite_action_fails_closed():
    session = _Session()
    policy = Go2RecoveryPolicy("unused", session=session)
    q, dq = _measurements()
    policy.step(q, dq, (1000.0, 0.0, 0.0), BodyState(0, 0, 0.14),
                {leg: 0.0 for leg in ("FL", "FR", "RL", "RR")})
    assert session.seen[0]["proprio"][0, 2] == 100.0
    session.action = np.full((1, 12), np.nan, dtype=np.float32)
    with pytest.raises(ValueError, match="non-finite Go2 recovery action"):
        policy.step(q, dq, (0, 0, 0), BodyState(0, 0, 0.14),
                    {leg: 0.0 for leg in ("FL", "FR", "RL", "RR")})


def test_bundled_model_accepts_recovery_observation():
    pytest.importorskip("onnxruntime")
    path = _package / "policies" / "go2_recovery_nju" / "policy.onnx"
    if not path.exists():
        pytest.skip("bundled recovery model not in source checkout")
    policy = Go2RecoveryPolicy(str(path))
    q, dq = _measurements()
    target = policy.step(q, dq, (0, 0, 0), BodyState(0, 0, 0.3),
                         {leg: 20.0 for leg in ("FL", "FR", "RL", "RR")})
    assert target.shape == (12,)
    assert np.isfinite(target).all()
