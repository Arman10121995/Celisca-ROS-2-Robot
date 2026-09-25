"""NJU-RLC get-up observation and bounded effort contract."""

from pathlib import Path
import math
import sys

import numpy as np
import pytest

_package = Path(__file__).resolve().parents[1]
if str(_package) not in sys.path:
    sys.path.insert(0, str(_package))

from robot_lab_adapter.go2_locomotion import (
    BodyState, EFFORT_LIMITS, JOINT_NAMES, PD_GAINS, POSITION_LIMITS, joint_kind,
)
from robot_lab_adapter.go2_recovery_policy import (
    Go2RecoveryPolicy, RECOVERY_ACTION_ABS_LIMIT, RECOVERY_DEFAULT,
    RECOVERY_DAMPING_SCALE, RECOVERY_GAIN_SCALE, RECOVERY_KD, RECOVERY_KP,
    RECOVERY_POLICY_DT_S, RECOVERY_TARGET_SLEW_RAD_S,
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


def test_efforts_are_plain_python_floats():
    """A numpy scalar here fails the std_msgs Float64MultiArray type assertion
    and kills the controller mid-fall, leaving the robot uncontrolled. This
    regression test pins the published type, not just the magnitude."""
    session = _Session(np.full((1, 12), RECOVERY_ACTION_ABS_LIMIT,
                               dtype=np.float32))
    policy = Go2RecoveryPolicy("unused", session=session)
    q, dq = _measurements()
    policy.step(q, dq, (0.0, 0.0, 0.0), BodyState(0.0, 0.0, 0.14),
                {leg: 0.0 for leg in ("FL", "FR", "RL", "RR")})
    for name, value in policy.efforts(q, dq).items():
        assert type(value) is float, f"{name} is {type(value)}, not float"
        assert math.isfinite(value)


def test_nonfinite_measured_state_still_yields_plain_finite_floats():
    """Guarded path: a NaN measurement must map to 0.0, not propagate."""
    session = _Session()
    policy = Go2RecoveryPolicy("unused", session=session)
    q, dq = _measurements()
    nan_q = {name: float("nan") for name in JOINT_NAMES}
    policy.step(q, dq, (0.0, 0.0, 0.0), BodyState(0.0, 0.0, 0.14),
                {leg: 0.0 for leg in ("FL", "FR", "RL", "RR")})
    for name, value in policy.efforts(nan_q, dq).items():
        assert type(value) is float
        assert math.isfinite(value)


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


def test_gains_follow_the_measured_per_joint_go2_gains():
    """The actor must not use one flat gain: the measured Go2 gains differ
    3x between hip and thigh/calf (robot_control.yaml)."""
    for i, name in enumerate(JOINT_NAMES):
        kp, kd = PD_GAINS[joint_kind(name)]
        assert RECOVERY_KP[i] == pytest.approx(kp * RECOVERY_GAIN_SCALE)
        assert RECOVERY_KD[i] == pytest.approx(kd * RECOVERY_DAMPING_SCALE)
    assert len(set(RECOVERY_KP.tolist())) > 1


def test_target_slew_limits_how_fast_the_commanded_pose_moves():
    """A 50 Hz unbounded target step against light zero-armature joints threw
    the recorded joint velocity to 19.4 rad/s and flipped the body over."""
    session = _Session(np.full((1, 12), RECOVERY_ACTION_ABS_LIMIT,
                               dtype=np.float32))
    policy = Go2RecoveryPolicy("unused", session=session)
    q, dq = _measurements()
    max_step = RECOVERY_TARGET_SLEW_RAD_S * RECOVERY_POLICY_DT_S
    first = policy.step(q, dq, (0, 0, 0), BodyState(0, 0, 0.14),
                        {leg: 0.0 for leg in ("FL", "FR", "RL", "RR")})
    # One step may not move any joint further than the slew allows.
    assert np.all(np.abs(first - RECOVERY_DEFAULT) <= max_step + 1e-6)
    # Repeated steps still converge toward the actor's desired pose, and no
    # single step ever exceeds the slew.
    policy.reset()
    seen = []
    for _ in range(200):
        seen.append(policy.step(q, dq, (0, 0, 0), BodyState(0, 0, 0.14),
                                {leg: 0.0 for leg in ("FL", "FR", "RL", "RR")}))
    for earlier, later in zip(seen, seen[1:]):
        assert np.all(np.abs(later - earlier) <= max_step + 1e-6)
    # It converged to the actor's desired pose rather than freezing.
    assert not np.allclose(seen[-1], seen[0]), "slew limit must not freeze the target"
    assert np.abs(seen[-1][0] - RECOVERY_DEFAULT[0]) > max_step


def test_slew_limited_target_stays_within_position_limits():
    session = _Session(np.full((1, 12), -RECOVERY_ACTION_ABS_LIMIT,
                               dtype=np.float32))
    policy = Go2RecoveryPolicy("unused", session=session)
    q, dq = _measurements()
    for _ in range(50):
        target = policy.step(q, dq, (0, 0, 0), BodyState(0, 0, 0.14),
                             {leg: 0.0 for leg in ("FL", "FR", "RL", "RR")})
        for i, name in enumerate(JOINT_NAMES):
            lo, hi = POSITION_LIMITS[joint_kind(name)]
            assert lo - 1e-6 <= target[i] <= hi + 1e-6
    assert np.isfinite(target).all()
