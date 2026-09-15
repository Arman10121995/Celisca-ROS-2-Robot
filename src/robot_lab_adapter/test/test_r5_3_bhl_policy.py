"""R5.3 tests: Berkeley Humanoid Lite policy qualification helper (tracking_command).

Covers the tracking_command helper added to qualify_policy.py for closed-loop
pose feedback during walk/turn phases in the headless MuJoCo policy probe.

This is a pure-logic unit test of the reference-tracking computation; it does
not run MuJoCo, ONNX, or any ROS graph. The qualified probe itself lives in
tools/qualify_policy.py and is executed separately.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

# Make the robot package importable (mirrors the R5.2 / R5.3 balance test harness).
_robot_pkg = Path(__file__).resolve().parents[1].parent / "berkeley_humanoid_lite"
_tools_dir = _robot_pkg / "tools"
if str(_tools_dir) not in sys.path:
    sys.path.insert(0, str(_tools_dir))

from qualify_policy import tracking_command  # noqa: E402


# ----------------------------------------------------------------------------
# Fixtures
# ----------------------------------------------------------------------------

@pytest.fixture
def standing_command():
    """Neutral command used during the stand/perturb/stop phases."""
    return np.array([0.0, 0.0, 0.0])


@pytest.fixture
def walk_command():
    """Forward-walk command used by the probe's walk phase."""
    return np.array([0.5, 0.0, 0.0])


@pytest.fixture
def turn_command():
    """In-place yaw-turn command used by the probe's turn phase."""
    return np.array([0.0, 0.0, 0.6])


# ----------------------------------------------------------------------------
# tracking_command pure-logic contract
# ----------------------------------------------------------------------------

class TestTrackingCommandSignature:
    """tracking_command is a pure function of the probe state and the phase
    reference; it does not mutate its inputs and returns a bounded command
    in the upstream training ranges."""

    def test_returns_finite_array(self, walk_command):
        cmd = tracking_command(
            walk_command, phase_time=0.0, start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.0, 0.0]), yaw=0.0,
        )
        assert isinstance(cmd, np.ndarray)
        assert cmd.shape == (3,)
        assert np.all(np.isfinite(cmd))

    def test_output_stays_within_training_bounds(self, walk_command, turn_command):
        """Upstream training command ranges are vx in [-1, 1], vy in [-0.5, 0.5],
        wz in [-1.5, 1.5]; tracking_command must never exceed them."""
        cases = [
            (walk_command, 0.0, (0.0, 0.0), 0.0, np.array([0.0, 0.0]), 0.0),
            (turn_command, 0.0, (0.0, 0.0), 0.0, np.array([0.0, 0.0]), 0.0),
        ]
        for cmd, pt, sxy, sy, xy, y in cases:
            out = tracking_command(cmd, pt, sxy, sy, xy, y)
            assert -1.0 <= out[0] <= 1.0, f"vx out of range: {out[0]}"
            assert -0.5 <= out[1] <= 0.5, f"vy out of range: {out[1]}"
            assert -1.5 <= out[2] <= 1.5, f"wz out of range: {out[2]}"

    def test_does_not_mutate_inputs(self, walk_command):
        cmd = walk_command.copy()
        start_xy = (0.0, 0.0)
        xy = np.array([0.0, 0.0])
        tracking_command(cmd, 0.0, start_xy, 0.0, xy, 0.0)
        assert np.array_equal(cmd, walk_command)
        assert start_xy == (0.0, 0.0)
        assert np.array_equal(xy, [0.0, 0.0])


class TestTrackingCommandStandStill:
    """During stand/perturb/stop the probe sends a zero command; tracking_command
    is only invoked for walk/turn, but a zero reference must produce a bounded
    zero-ish correction when the robot is exactly on the reference."""

    def test_zero_reference_at_origin(self):
        cmd = tracking_command(
            np.array([0.0, 0.0, 0.0]), phase_time=0.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.0, 0.0]), yaw=0.0,
        )
        assert np.allclose(cmd, [0.0, 0.0, 0.0], atol=1e-9)


class TestTrackingCommandWalkPhase:
    """Walk phase uses perfect-simulator pose feedback to track a straight
    reference line along the initial yaw. The correction is small when the
    robot follows the reference and grows with lateral/longitudinal error."""

    def test_start_at_reference_returns_command(self, walk_command):
        """At t=0 on the reference the returned command equals the phase command
        (no correction yet)."""
        out = tracking_command(
            walk_command, phase_time=0.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.0, 0.0]), yaw=0.0,
        )
        assert np.allclose(out, walk_command, atol=1e-9)

    def test_lag_produces_forward_correction(self, walk_command):
        """If the robot lags behind the reference, the body-frame vx correction
        increases (pushes forward)."""
        out_on_ref = tracking_command(
            walk_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.5, 0.0]), yaw=0.0,
        )
        out_lagging = tracking_command(
            walk_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.4, 0.0]), yaw=0.0,
        )
        assert out_lagging[0] > out_on_ref[0]

    def test_lateral_error_produces_sideways_correction(self, walk_command):
        """A lateral offset from the reference line produces a body-frame vy
        correction toward the reference."""
        # Robot to the left of the reference line (positive y in world).
        out_right = tracking_command(
            walk_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.5, 0.2]), yaw=0.0,
        )
        # Robot to the right of the reference line (negative y in world).
        out_left = tracking_command(
            walk_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.5, -0.2]), yaw=0.0,
        )
        # Body-frame vy for a yaw=0 robot is just the world vy component.
        assert out_right[1] < 0.0  # push right->left
        assert out_left[1] > 0.0   # push left->right

    def test_yaw_error_produces_yaw_correction(self, walk_command):
        """A yaw error relative to the walking direction produces a wz correction
        that reduces the error."""
        out_aligned = tracking_command(
            walk_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.5, 0.0]), yaw=0.0,
        )
        out_turned = tracking_command(
            walk_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.5, 0.0]), yaw=0.1,
        )
        assert abs(out_turned[2]) > abs(out_aligned[2])


class TestTrackingCommandTurnPhase:
    """Turn phase uses perfect-simulator pose feedback to track an in-place yaw
    ramp about the start position. The correction is small when the robot
    holds the right heading and position."""

    def test_start_at_reference_returns_command(self, turn_command):
        out = tracking_command(
            turn_command, phase_time=0.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.0, 0.0]), yaw=0.0,
        )
        assert np.allclose(out, turn_command, atol=1e-9)

    def test_waiting_produces_no_position_correction(self, turn_command):
        """During an in-place turn the reference position is constant; a robot
        that stays at the start position gets no positional correction."""
        out = tracking_command(
            turn_command, phase_time=2.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.0, 0.0]), yaw=0.0,
        )
        assert abs(out[0]) < 1e-9
        assert abs(out[1]) < 1e-9

    def test_yaw_lag_produces_yaw_correction(self, turn_command):
        """If the robot's yaw lags the reference ramp, wz correction increases."""
        out_on_ref = tracking_command(
            turn_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.0, 0.0]), yaw=0.6,
        )
        out_lagging = tracking_command(
            turn_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.0, 0.0]), yaw=0.5,
        )
        assert out_lagging[2] > out_on_ref[2]

    def test_yaw_ahead_produces_negative_yaw_correction(self, turn_command):
        """If the robot's yaw leads the reference ramp, wz correction decreases."""
        out_on_ref = tracking_command(
            turn_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.0, 0.0]), yaw=0.6,
        )
        out_ahead = tracking_command(
            turn_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.0, 0.0]), yaw=0.7,
        )
        assert out_ahead[2] < out_on_ref[2]


class TestTrackingCommandClamping:
    """Large errors must still be clamped to the upstream training ranges so the
    probe never feeds the policy an out-of-distribution command."""

    def test_large_lag_clamps_vx(self, walk_command):
        """A huge lag behind the reference must not overflow vx beyond 1.0."""
        out = tracking_command(
            walk_command, phase_time=10.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([-100.0, 0.0]), yaw=0.0,
        )
        assert -1.0 <= out[0] <= 1.0

    def test_large_lateral_error_clamps_vy(self, walk_command):
        """A huge lateral error must not overflow vy beyond +/-0.5."""
        out = tracking_command(
            walk_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.5, 1000.0]), yaw=0.0,
        )
        assert -0.5 <= out[1] <= 0.5

    def test_large_yaw_error_clamps_wz(self, turn_command):
        """A huge yaw error must not overflow wz beyond +/-1.5."""
        out = tracking_command(
            turn_command, phase_time=1.0,
            start_xy=(0.0, 0.0), start_yaw=0.0,
            xy=np.array([0.0, 0.0]), yaw=100.0,
        )
        assert -1.5 <= out[2] <= 1.5
