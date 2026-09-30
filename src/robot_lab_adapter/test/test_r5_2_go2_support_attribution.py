"""Hermetic contracts for the settled-rest support attribution record.

The probe answers whether the two published contact topics actually carry a
settled Go2 rest. A real trial disagreed: on its flank, /go2/foot_contact_forces
and /go2/trunk_contact_forces read 23-85 N of a 126.5 N robot, and the entry
analysis could only say the robot was "on hip and thigh". The probe drops the
plant at the rolls the ladder produces and partitions the settled normal force
over every colliding geom, so what the topics miss is measured at genuine
static equilibrium instead of inferred.

These tests pin the recorded claims in ``support_attribution.json``. They do
not import the probe: it needs MuJoCo and the ROS package index, and the
recorded partition is what the entry work consumes.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

EVIDENCE = (Path(__file__).resolve().parents[3]
            / "docs/status/evidence/r52-go2-policy-2026-09-25")
WEIGHT_N = 126.53


def _record() -> dict:
    return json.loads((EVIDENCE / "support_attribution.json").read_text())


def _rest(record: dict, roll_rad: float) -> dict:
    rest = next((rest for rest in record["measured"]["rests"]
                 if abs(rest["roll_rad"] - roll_rad) < 1e-6), None)
    assert rest is not None, f"no recorded rest at roll {roll_rad}"
    return rest


def test_every_dropped_rest_is_a_measured_equilibrium():
    """The partition is only meaningful at a static rest: every drop must come
    to rest under gravity (qvel ~ 0) and its support must sum to the weight."""
    record = _record()
    assert record["measured"]["robot_weight_n"] == pytest.approx(WEIGHT_N, abs=0.05)
    rolls = sorted(round(rest["roll_rad"], 5) for rest in record["measured"]["rests"])
    assert rolls == [-0.5, 0.0, 0.5, 0.9, 1.4, 2.4, 3.14159]
    for rest in record["measured"]["rests"]:
        assert rest["max_abs_qvel"] <= 0.001
        assert rest["contact_n"] == pytest.approx(WEIGHT_N, abs=0.05)


def test_the_two_topics_are_blind_on_a_settled_flank():
    """On both settled flanks -- the pose the entry primitive must fix -- the
    published topics carry 7.4 N of 126.5 N: the two down-side feet at ~3.7 N
    each and 0.0 N of trunk.  The two down-side hips carry the rest, ~59.6 N
    each, and publish nothing."""
    record = _record()
    for roll, down_side in ((-0.5, "L"), (0.5, "R"), (0.9, "R"), (1.4, "R")):
        rest = _rest(record, roll)
        assert rest["coverage_percent"] <= 6.0
        assert rest["published_n"] == pytest.approx(7.4, abs=0.2)
        assert rest["hidden_n"] == pytest.approx(119.15, abs=0.2)
        hips = {f"F{down_side}_hip_contact_0", f"R{down_side}_hip_contact_0"}
        assert set(rest["hidden_geoms_n"]) == hips
        for force in rest["hidden_geoms_n"].values():
            assert force == pytest.approx(59.6, abs=0.8)
        assert "trunk_contact_0" not in rest["per_geom_n"]
        feet = {geom: force for geom, force in rest["per_geom_n"].items()
                if geom.endswith("_foot_contact_0")}
        assert len(feet) == 2
        assert all(force < 4.5 for force in feet.values())


def test_the_upright_and_inverted_rests_are_published_in_full():
    """The topics are a complete support reading at the two attitudes the
    ladder's own ends produce: upright (the four feet, 27-36 N each) and
    inverted (the trunk, all 126.5 N)."""
    record = _record()
    upright = _rest(record, 0.0)
    assert upright["coverage_percent"] == 100.0
    assert upright["hidden_geoms_n"] == {}
    assert set(upright["per_geom_n"]) == {
        "FL_foot_contact_0", "FR_foot_contact_0",
        "RL_foot_contact_0", "RR_foot_contact_0"}
    assert all(26.0 < force < 37.0 for force in upright["per_geom_n"].values())
    for roll in (2.4, 3.14159265):
        inverted = _rest(record, roll)
        assert inverted["coverage_percent"] == 100.0
        assert inverted["hidden_geoms_n"] == {}
        assert list(inverted["per_geom_n"]) == ["trunk_contact_0"]
        assert inverted["per_geom_n"]["trunk_contact_0"] == pytest.approx(
            WEIGHT_N, abs=0.05)


def test_the_published_sum_is_the_partition_of_the_published_geoms():
    """The probe's ``published`` number is the same measurement the topics
    publish, so for every rest it must equal the summed partition of exactly
    those five geoms -- and ``hidden`` the sum of everything else."""
    record = _record()
    published = set(record["published_geoms"])
    assert published == {"FL_foot_contact_0", "FR_foot_contact_0",
                         "RL_foot_contact_0", "RR_foot_contact_0",
                         "trunk_contact_0"}
    for rest in record["measured"]["rests"]:
        on = sum(force for geom, force in rest["per_geom_n"].items()
                 if geom in published)
        off = sum(force for geom, force in rest["per_geom_n"].items()
                  if geom not in published)
        assert on == pytest.approx(rest["published_n"], abs=0.05)
        assert off == pytest.approx(rest["hidden_n"], abs=0.05)


def test_five_of_eighteen_colliding_geoms_publish_a_force():
    """The two topics cover the four feet and the trunk; both hips, thighs and
    calves per leg and the imu link publish nothing."""
    record = _record()
    inventory = record["colliding_geoms_by_body"]
    geoms = [geom for body in inventory.values() for geom in body]
    published = set(record["published_geoms"])
    assert len(geoms) == 18
    assert len(set(geoms)) == 18
    assert published <= set(geoms)
    hidden = set(geoms) - published
    assert len(hidden) == 13
    for leg in ("FL", "FR", "RL", "RR"):
        assert f"{leg}_hip_contact_0" in hidden
        assert f"{leg}_thigh_contact_0" in hidden
        assert f"{leg}_calf_contact_0" in hidden
    assert "imu_link_contact_0" in hidden
