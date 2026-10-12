"""Pure-logic contracts of the October 9 mobile-task and takeoff-check code."""

import math
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "robot_lab_utils"))

from robot_lab_utils.aerial_takeoff import check_takeoff  # noqa: E402
from robot_lab_utils.mobile_task import SCHEMA, validate_task  # noqa: E402

# The derived X500 envelope (docs/status/implementation-continuation-2026-10-09.md).
ENVELOPE = dict(lower=[-0.503881, -0.503881, -0.227], upper=[0.503881, 0.503881, 0.120423])
CHANNELS = [dict(name="lift", limits=[0.0, 1.1]), dict(name="gripper", limits=[-0.5, 0.5])]


def _box(center, size):
    return dict(type="box", position=list(center), size=list(size), rotation=np.eye(3))


def _scene(*shapes):
    return SimpleNamespace(shapes=list(shapes))


def _task(*steps):
    return dict(schema=SCHEMA, source_sha256="src", scene_id="scene", steps=list(steps))


def test_open_sky_gives_the_requested_rise():
    result = check_takeoff(_scene(_box([5, 5, 1], [1, 1, 2])), [0, 0, 0.3], ENVELOPE)
    assert result["rise_m"] == 3.0 and not result["ceiling_limited"]
    assert result["target_enu"] == pytest.approx([0, 0, 3.3])


def test_a_ceiling_reduces_the_rise_and_keeps_headroom():
    ceiling = _box([0, 0, 2.6], [4, 4, 0.2])   # underside at 2.5 m
    result = check_takeoff(_scene(ceiling), [0, 0, 0.3], ENVELOPE)
    assert result["ceiling_limited"]
    top = 0.3 + result["rise_m"] + ENVELOPE["upper"][2]
    assert top < 2.5 and top > 2.5 - 0.15  # 0.1 m extra headroom plus bisection


def test_a_blocked_corridor_is_rejected():
    beam = _box([0.3, 0, 0.9], [0.2, 2, 0.2])  # crosses the rotor sweep 0.6 m up
    with pytest.raises(ValueError, match="minimum 0.8 m"):
        check_takeoff(_scene(beam), [0, 0, 0.3], ENVELOPE)


@pytest.mark.parametrize("feet_z", [0.013, 0.0, -0.02])
def test_the_support_floor_under_the_feet_does_not_block(feet_z):
    # Nominal 13 mm clearance, touching, and 2 cm of estimator/contact sink.
    floor = _box([0, 0, -0.05], [10, 10, 0.1])   # top at z = 0
    result = check_takeoff(_scene(floor), [0, 0, 0.227 + feet_z], ENVELOPE)
    assert result["rise_m"] == 3.0


def test_an_obstacle_taller_than_the_support_band_still_blocks():
    curb = _box([0.4, 0, 0.05], [0.2, 2, 0.1])   # 10 cm under a rotor
    with pytest.raises(ValueError, match="minimum 0.8 m"):
        check_takeoff(_scene(curb), [0, 0, 0.227], ENVELOPE)


def test_bad_rises_are_refused():
    with pytest.raises(ValueError):
        check_takeoff(_scene(), [0, 0, 0], ENVELOPE, desired_rise=0.5)
    with pytest.raises(ValueError):
        check_takeoff(_scene(), [0, 0, math.nan], ENVELOPE)


def test_task_steps_are_validated_and_normalized():
    steps = validate_task(_task(
        dict(kind="navigate", pose=[1, 2, 7.0]),
        dict(kind="articulate", waypoints=[dict(targets=dict(lift=0.5, gripper=0.1))])),
        "src", "scene", CHANNELS, allow_navigation=True)
    assert steps[0]["pose"][2] == pytest.approx(math.atan2(math.sin(7.0), math.cos(7.0)))
    assert steps[1]["waypoints"][0]["settle_s"] == 0.5


@pytest.mark.parametrize("document, message", [
    (dict(_task(dict(kind="navigate", pose=[0, 0, 0])), scene_id="other"), "must match"),
    (_task(), "1–32"),
    (_task(dict(kind="articulate", waypoints=[dict(targets=dict(lift=0.5))])), "every running"),
    (_task(dict(kind="articulate", waypoints=[dict(targets=dict(lift=2.0, gripper=0))])), "source bounds"),
    (_task(dict(kind="grasp")), "navigate and articulate"),
])
def test_bad_tasks_are_refused(document, message):
    with pytest.raises(ValueError, match=message):
        validate_task(document, "src", "scene", CHANNELS, allow_navigation=True)


def test_navigation_needs_the_nav_workflow():
    with pytest.raises(ValueError, match="native nav workflow"):
        validate_task(_task(dict(kind="navigate", pose=[0, 0, 0])), "src", "scene",
                      CHANNELS, allow_navigation=False)
