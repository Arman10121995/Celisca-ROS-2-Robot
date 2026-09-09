"""R4.2 metric extraction unit tests.

Tests trajectory_distance, contact_events, footprint_clearance,
timestamp_aligned_error, compute_rtf, effort_proxy, the dataclasses,
compose_run_metrics and validate_metric_value against the R4.2 contract:

- Known trajectories/contact fixtures yield correct values.
- One contact event is counted once even if many scans observe it.
- Missing/NaN/stale data invalidate a metric (None) instead of 0.0.
- The schema distinguishes measured versus derived metrics and rejects
  invalid values.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

# Ensure robot_lab_benchmark (sibling package) is importable during tests,
# mirroring the injection used by test_r4_1_lifecycle_truthful_outcomes.py.
_benchmark_pkg = Path(__file__).resolve().parents[3] / "robot_lab" / "robot_lab_benchmark"
if str(_benchmark_pkg) not in sys.path:
    sys.path.insert(0, str(_benchmark_pkg))

from robot_lab_benchmark.metrics import (
    _is_valid,
    trajectory_distance,
    contact_events,
    footprint_clearance,
    timestamp_aligned_error,
    compute_rtf,
    effort_proxy,
    MeasuredMetrics,
    DerivedMetrics,
    RunMetrics,
    compute_derived_metrics,
    compose_run_metrics,
    validate_metric_value,
)


def _approx(a, b, eps=1e-9):
    return math.isclose(float(a), float(b), abs_tol=eps)


# ------------------------------------------------------------------
# _is_valid
# ------------------------------------------------------------------


class TestIsValid:
    def test_finite_numbers(self):
        assert _is_valid(0.0) is True
        assert _is_valid(3.14) is True
        assert _is_valid(-1.0) is True
        assert _is_valid(0) is True  # int accepted

    def test_none(self):
        assert _is_valid(None) is False

    def test_nan(self):
        assert _is_valid(float("nan")) is False

    def test_inf(self):
        assert _is_valid(float("inf")) is False
        assert _is_valid(float("-inf")) is False

    def test_non_numeric(self):
        assert _is_valid("5.0") is False
        assert _is_valid([1.0]) is False


# ------------------------------------------------------------------
# trajectory_distance
# ------------------------------------------------------------------


class TestTrajectoryDistance:
    def test_simple_triangle(self):
        # (0,0)->(3,4)->(3,0) = 5 + 4 = 9
        assert trajectory_distance([(0.0, 0.0), (3.0, 4.0), (3.0, 0.0)]) == 9.0

    def test_single_point(self):
        assert trajectory_distance([(1.0, 2.0)]) is None

    def test_empty(self):
        assert trajectory_distance([]) is None

    def test_nan_invalidates(self):
        assert trajectory_distance([(0.0, 0.0), (float("nan"), 1.0)]) is None

    def test_none_in_pose(self):
        assert trajectory_distance([(0.0, 0.0), (None, 1.0)]) is None

    def test_inf_invalidates(self):
        assert trajectory_distance([(0.0, 0.0), (float("inf"), 1.0)]) is None

    def test_two_points(self):
        assert trajectory_distance([(0.0, 0.0), (1.0, 0.0)]) == 1.0

    def test_diagonal_segment(self):
        # (0,0)->(3,4) = 5
        assert trajectory_distance([(0.0, 0.0), (3.0, 4.0)]) == 5.0

    def test_repeated_pose_zero_segment(self):
        # duplicate pose adds zero length
        assert trajectory_distance([(0.0, 0.0), (1.0, 0.0), (1.0, 0.0)]) == 1.0

    def test_mixed_valid_invalid(self):
        assert trajectory_distance(
            [(0.0, 0.0), (1.0, 1.0), (float("nan"), 1.0), (2.0, 2.0)]
        ) is None


# ------------------------------------------------------------------
# contact_events (one event != one per scan)
# ------------------------------------------------------------------


class TestContactEvents:
    def test_no_contact(self):
        assert contact_events([0.2, 0.3, 0.4], threshold=0.05) == 0

    def test_one_event_many_scans(self):
        series = [0.2, 0.03, 0.02, 0.01, 0.02, 0.03, 0.2]
        assert contact_events(series, threshold=0.05) == 1

    def test_two_separate_events(self):
        series = [0.2, 0.03, 0.02, 0.2, 0.04, 0.15]
        assert contact_events(series, threshold=0.05) == 2

    def test_three_events(self):
        series = [0.1, 0.03, 0.1, 0.03, 0.1, 0.03, 0.1]
        assert contact_events(series, threshold=0.05) == 3

    def test_empty_series(self):
        assert contact_events([], threshold=0.05) is None

    def test_all_none_series(self):
        assert contact_events([None, None, None], threshold=0.05) is None

    def test_nan_breaks_event_into_two(self):
        # a gap of invalid readings ends the current event and a new one
        # starts when clearance drops below threshold again
        series = [0.2, 0.03, None, 0.02, 0.2]
        assert contact_events(series, threshold=0.05) == 2

    def test_stays_below_at_series_end(self):
        series = [0.2, 0.03, 0.01]
        assert contact_events(series, threshold=0.05) == 1

    def test_custom_threshold(self):
        series = [0.1, 0.08, 0.1]
        assert contact_events(series, threshold=0.1) == 1
        assert contact_events(series, threshold=0.05) == 0

    def test_event_at_threshold_boundary(self):
        # exactly at threshold is NOT below it
        assert contact_events([0.05, 0.05], threshold=0.05) == 0

    def test_contact_count_not_scan_count(self):
        # 50 consecutive below-threshold scans must still be ONE event
        series = [0.2] + [0.01] * 50 + [0.2]
        assert contact_events(series, threshold=0.05) == 1


# ------------------------------------------------------------------
# footprint_clearance
# ------------------------------------------------------------------


class TestFootprintClearance:
    def test_minimum_minus_radius(self):
        assert _approx(footprint_clearance([0.3, 0.5, 0.2], 0.05), 0.15)

    def test_zero_radius(self):
        assert _approx(footprint_clearance([0.3, 0.5, 0.2], 0.0), 0.2)

    def test_empty(self):
        assert footprint_clearance([], 0.05) is None

    def test_all_none(self):
        assert footprint_clearance([None, None], 0.05) is None

    def test_nan_filtered_out(self):
        assert _approx(footprint_clearance([0.3, float("nan"), 0.2], 0.05), 0.15)

    def test_single_reading(self):
        assert _approx(footprint_clearance([0.4], 0.1), 0.3)

    def test_negative_result_means_overlap(self):
        # clearance smaller than footprint radius -> robot overlaps obstacle
        assert _approx(footprint_clearance([0.02], 0.05), -0.03)


# ------------------------------------------------------------------
# timestamp_aligned_error
# ------------------------------------------------------------------


class TestTimestampAlignedError:
    def test_exact_overlap_zero_error(self):
        truth = [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (2.0, 2.0, 0.0)]
        est = [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (2.0, 2.0, 0.0)]
        res = timestamp_aligned_error(truth, est)
        assert _approx(res["max_error_m"], 0.0)
        assert _approx(res["mean_error_m"], 0.0)
        assert _approx(res["rmse_m"], 0.0)

    def test_interpolated_estimates(self):
        # estimate only at t=0 and t=2; at t=1 it is interpolated to (1,1)
        truth = [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (2.0, 2.0, 0.0)]
        est = [(0.0, 0.0, 0.0), (2.0, 2.0, 0.0)]
        res = timestamp_aligned_error(truth, est)
        assert _approx(res["max_error_m"], 0.0)
        assert _approx(res["rmse_m"], 0.0)

    def test_constant_offset(self):
        truth = [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (2.0, 2.0, 0.0)]
        est = [(0.0, 0.1, 0.1), (1.0, 1.1, 0.1), (2.0, 2.1, 0.1)]
        res = timestamp_aligned_error(truth, est)
        expected = math.hypot(0.1, 0.1)
        assert _approx(res["max_error_m"], expected)
        assert _approx(res["mean_error_m"], expected)
        assert _approx(res["rmse_m"], expected)

    def test_no_overlap(self):
        # truth and estimate do not overlap in time... actually the
        # implementation clamps to the estimate span, so any truth gets a
        # clamped value; with only one estimate point -> None
        truth = [(0.0, 0.0, 0.0)]
        est = [(0.0, 0.0, 0.0)]
        res = timestamp_aligned_error(truth, est)
        assert res["max_error_m"] is None

    def test_empty_truth(self):
        res = timestamp_aligned_error([], [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)])
        assert res == {"max_error_m": None, "mean_error_m": None, "rmse_m": None}

    def test_empty_estimates(self):
        res = timestamp_aligned_error([(0.0, 0.0, 0.0)], [])
        assert res == {"max_error_m": None, "mean_error_m": None, "rmse_m": None}

    def test_single_estimate_point(self):
        res = timestamp_aligned_error([(0.0, 0.0, 0.0)], [(0.0, 0.0, 0.0)])
        assert res == {"max_error_m": None, "mean_error_m": None, "rmse_m": None}

    def test_nan_in_truth_invalidates(self):
        truth = [(0.0, 0.0, 0.0), (float("nan"), 1.0, 1.0)]
        est = [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)]
        res = timestamp_aligned_error(truth, est)
        # t=nan truth point filtered; only t=0 contributes
        assert res["max_error_m"] is not None

    def test_nan_in_estimates_filtered(self):
        truth = [(0.0, 0.0, 0.0), (1.0, 1.0, 1.0)]
        est = [(0.0, 0.0, 0.0), (float("nan"), 1.0, 1.0)]
        res = timestamp_aligned_error(truth, est)
        # only one valid estimate left -> not enough to interpolate
        assert res["max_error_m"] is None

    def test_unsorted_estimates_are_sorted(self):
        truth = [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (2.0, 2.0, 0.0)]
        est = [(2.0, 2.0, 0.0), (0.0, 0.0, 0.0)]
        res = timestamp_aligned_error(truth, est)
        assert _approx(res["max_error_m"], 0.0)

    def test_no_pathological_zero_on_missing(self):
        # if there is no usable overlap the result must be None, not 0.0
        res = timestamp_aligned_error([], [])
        assert res["rmse_m"] is None
        assert res["rmse_m"] != 0.0


# ------------------------------------------------------------------
# compute_rtf
# ------------------------------------------------------------------


class TestComputeRtf:
    def test_simple(self):
        assert _approx(compute_rtf(10.0, 5.0), 2.0)

    def test_slower_than_realtime(self):
        assert _approx(compute_rtf(1.0, 4.0), 0.25)

    def test_none_sim(self):
        assert compute_rtf(None, 5.0) is None

    def test_none_wall(self):
        assert compute_rtf(10.0, None) is None

    def test_zero_wall(self):
        assert compute_rtf(10.0, 0.0) is None

    def test_negative_wall(self):
        assert compute_rtf(10.0, -1.0) is None

    def test_nan_inputs(self):
        assert compute_rtf(float("nan"), 5.0) is None
        assert compute_rtf(10.0, float("nan")) is None

    def test_zero_sim_valid(self):
        assert _approx(compute_rtf(0.0, 5.0), 0.0)


# ------------------------------------------------------------------
# effort_proxy
# ------------------------------------------------------------------


class TestEffortProxy:
    def test_simple_sum(self):
        # (1 + 2 + 3) * 1.0 = 6
        assert _approx(effort_proxy([1.0, 2.0, 3.0], dt=1.0), 6.0)

    def test_absolute_values(self):
        # (1 + 2 + 3) * 0.5 = 3
        assert _approx(effort_proxy([1.0, -2.0, 3.0], dt=0.5), 3.0)

    def test_none_samples_ignored(self):
        assert _approx(effort_proxy([1.0, None, 3.0], dt=1.0), 4.0)

    def test_all_none(self):
        assert effort_proxy([None, None], dt=1.0) is None

    def test_empty(self):
        assert effort_proxy([], dt=1.0) is None

    def test_invalid_dt(self):
        assert effort_proxy([1.0], dt=0.0) is None
        assert effort_proxy([1.0], dt=-1.0) is None

    def test_nan_dt(self):
        assert effort_proxy([1.0], dt=float("nan")) is None


# ------------------------------------------------------------------
# Dataclasses
# ------------------------------------------------------------------


class TestDataclasses:
    def test_measured_metrics_defaults(self):
        mm = MeasuredMetrics()
        assert mm.path_length_m is None
        assert mm.contact_events is None
        assert mm.min_clearance_m is None
        assert mm.max_position_error_m is None
        assert mm.mean_position_error_m is None
        assert mm.rmse_position_error_m is None
        assert mm.elapsed_wall_seconds is None
        assert mm.elapsed_sim_seconds is None
        assert mm.cpu_percent is None
        assert mm.memory_mb is None
        assert mm.effort_proxy is None

    def test_derived_metrics_defaults(self):
        dm = DerivedMetrics()
        assert dm.rtf is None
        assert dm.path_efficiency is None

    def test_run_metrics_defaults(self):
        rm = RunMetrics()
        assert isinstance(rm.measured, MeasuredMetrics)
        assert isinstance(rm.derived, DerivedMetrics)
        assert rm.provenance == {}
        assert rm.issues == []

    def test_run_metrics_provenance_is_per_instance(self):
        rm1 = RunMetrics()
        rm2 = RunMetrics()
        rm1.provenance["seed"] = 1
        assert rm2.provenance == {}

    def test_run_metrics_issues_is_per_instance(self):
        rm1 = RunMetrics()
        rm2 = RunMetrics()
        rm1.issues.append("x")
        assert rm2.issues == []


# ------------------------------------------------------------------
# compute_derived_metrics
# ------------------------------------------------------------------


class TestComputeDerivedMetrics:
    def test_rtf_from_measured(self):
        mm = MeasuredMetrics(elapsed_sim_seconds=12.0, elapsed_wall_seconds=4.0)
        dm = compute_derived_metrics(mm)
        assert _approx(dm.rtf, 3.0)

    def test_rtf_with_explicit_wall(self):
        mm = MeasuredMetrics(elapsed_sim_seconds=12.0)
        dm = compute_derived_metrics(mm, wall_seconds=6.0)
        assert _approx(dm.rtf, 2.0)

    def test_rtf_none_without_sim(self):
        mm = MeasuredMetrics(elapsed_wall_seconds=4.0)
        dm = compute_derived_metrics(mm)
        assert dm.rtf is None

    def test_path_efficiency(self):
        mm = MeasuredMetrics(path_length_m=5.0)
        dm = compute_derived_metrics(mm, wall_seconds=10.0)
        assert _approx(dm.path_efficiency, 0.5)

    def test_path_efficiency_none_without_wall(self):
        mm = MeasuredMetrics(path_length_m=5.0)
        dm = compute_derived_metrics(mm)
        assert dm.path_efficiency is None

    def test_all_none_when_measured_empty(self):
        dm = compute_derived_metrics(MeasuredMetrics())
        assert dm.rtf is None
        assert dm.path_efficiency is None


# ------------------------------------------------------------------
# compose_run_metrics
# ------------------------------------------------------------------


class TestComposeRunMetrics:
    def test_full_roundtrip(self):
        rm = compose_run_metrics(
            path_length_m=8.0,
            elapsed_wall_seconds=4.0,
            elapsed_sim_seconds=12.0,
        )
        assert _approx(rm.measured.path_length_m, 8.0)
        assert _approx(rm.derived.rtf, 3.0)
        assert _approx(rm.derived.path_efficiency, 2.0)
        assert rm.issues == []
        assert rm.provenance == {}

    def test_provenance_passed_through(self):
        rm = compose_run_metrics(path_length_m=1.0, provenance={"seed": 42})
        assert rm.provenance == {"seed": 42}

    def test_issues_passed_through(self):
        rm = compose_run_metrics(path_length_m=1.0, issues=["warn1", "warn2"])
        assert rm.issues == ["warn1", "warn2"]

    def test_empty_compose(self):
        rm = compose_run_metrics()
        assert rm.measured.path_length_m is None
        assert rm.derived.rtf is None
        assert rm.provenance == {}
        assert rm.issues == []

    def test_wall_seconds_for_derived_override(self):
        rm = compose_run_metrics(
            path_length_m=10.0,
            elapsed_sim_seconds=10.0,
            wall_seconds_for_derived=5.0,
        )
        assert _approx(rm.derived.rtf, 2.0)
        assert _approx(rm.derived.path_efficiency, 2.0)


# ------------------------------------------------------------------
# validate_metric_value (schema enforcement)
# ------------------------------------------------------------------


class TestValidateMetricValue:
    def test_none_is_valid(self):
        assert validate_metric_value(None, "x") == []

    def test_finite_value_is_valid(self):
        assert validate_metric_value(5.0, "x") == []
        assert validate_metric_value(0.0, "x") == []
        assert validate_metric_value(3.14, "x") == []

    def test_negative_rejected(self):
        issues = validate_metric_value(-0.3, "min_clearance")
        assert issues == ["min_clearance is negative: -0.3"]

    def test_negative_allowed_when_specified(self):
        assert validate_metric_value(-0.3, "x", allow_negative=True) == []

    def test_nan_rejected(self):
        issues = validate_metric_value(float("nan"), "x")
        assert len(issues) == 1
        assert "non-finite" in issues[0]

    def test_inf_rejected(self):
        issues = validate_metric_value(float("inf"), "x")
        assert len(issues) == 1
        assert "non-finite" in issues[0]

    def test_non_numeric_rejected(self):
        issues = validate_metric_value("5.0", "x")
        assert len(issues) == 1
        assert "not a number" in issues[0]

    def test_issue_contains_name(self):
        issues = validate_metric_value(-1.0, "path_length_m")
        assert "path_length_m" in issues[0]
