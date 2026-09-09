"""R4.3 tests: reproducible two-planner comparison.

Covers the R4.3 acceptance bar:

- Two arms with every other setting fixed; spec validation rejects bad
  predeclarations (fewer than five evaluation seeds, duplicate seeds,
  tuning/evaluation overlap, identical arm ids, non-positive budgets,
  unjustified thresholds).
- Tuning and evaluation seeds are kept strictly separate; only
  evaluation seeds are reported.
- Manifests are deterministic with a stable hash; verify catches
  tampering; a rerun reproduces the exact scenario inputs.
- Raw trace references are retained per run; all failures are kept with
  their outcome kinds and trace paths.
- The report carries per-arm distributions and failure rates (never a
  best-run-only summary) and median deltas between arms.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest
import yaml

# Ensure robot_lab_benchmark (sibling package) is importable during tests,
# mirroring the injection used by test_r4_1_lifecycle_truthful_outcomes.py.
_benchmark_pkg = Path(__file__).resolve().parents[3] / "robot_lab" / "robot_lab_benchmark"
if str(_benchmark_pkg) not in sys.path:
    sys.path.insert(0, str(_benchmark_pkg))

from robot_lab_benchmark.comparison import (
    ComparisonArm,
    ComparisonValidationError,
    ComparisonSpec,
    ResourceBudget,
    RunRecord,
    ThresholdSpec,
    MIN_EVALUATION_SEEDS,
    build_comparison_manifest,
    verify_comparison_manifest,
    run_comparison,
    summarize_comparison,
    metric_distribution,
    write_comparison_report,
    make_bumperbot_comparison,
)


EVAL_SEEDS = (11, 23, 37, 41, 53)
TUNING_SEEDS = (7, 9)
JUSTIFIED = "Measured on 5 reference runs; 95th percentile was 9.8 m."


def _spec(**overrides) -> ComparisonSpec:
    kwargs = dict(
        experiment_id="exp",
        robot_id="bumperbot",
        environment_id="small_office",
        scenario_id="point_to_point_navigation",
        simulator="gazebo",
        arm_a=ComparisonArm("a", "navfn_planner", "teb_local_planner"),
        arm_b=ComparisonArm("b", "dijkstra_planner", "teb_local_planner"),
        evaluation_seeds=EVAL_SEEDS,
        tuning_seeds=TUNING_SEEDS,
        timeout_seconds=300.0,
        goal_tolerance_m=0.25,
        resource_budget=ResourceBudget(3600.0, 2048.0, 100.0),
        thresholds=(ThresholdSpec("max_path_length_m", 12.0, JUSTIFIED),),
        artifact_paths=("rosbag2/", "trajectories/", "manifest.json"),
    )
    kwargs.update(overrides)
    return ComparisonSpec(**kwargs)


def _rec(arm_id, seed, outcome="success", path_length=8.0, wall=60.0):
    return RunRecord(
        arm_id=arm_id,
        seed=seed,
        outcome_kind=outcome,
        metrics={
            "path_length_m": None if path_length is None else float(path_length),
            "elapsed_wall_seconds": None if wall is None else float(wall),
        },
        trace_paths=["/tmp/runs/%s_%d/rosbag2" % (arm_id, seed)],
        wall_seconds=wall,
    )


def _fake_runner(arm, seed):
    # deterministic per (arm, seed); arm "b" fails on seed 41
    if arm.arm_id == "b" and seed == 41:
        return _rec(arm.arm_id, seed, outcome="collision", path_length=None, wall=41.0)
    base = 8.0 if arm.arm_id == "a" else 9.0
    return _rec(arm.arm_id, seed, path_length=base + seed % 3)


# ------------------------------------------------------------------
# Spec validation (predeclared comparison)
# ------------------------------------------------------------------


class TestSpecValidation:
    def test_valid_spec_passes(self):
        assert _spec().validate() == []

    def test_fewer_than_five_evaluation_seeds_rejected(self):
        issues = _spec(evaluation_seeds=(1, 2, 3, 4)).validate()
        assert any("at least 5" in i for i in issues)

    def test_exactly_five_seeds_accepted(self):
        assert _spec(tuning_seeds=()).validate() == []

    def test_duplicate_evaluation_seeds_rejected(self):
        issues = _spec(evaluation_seeds=(11, 11, 23, 37, 41)).validate()
        assert any("duplicate" in i for i in issues)

    def test_tuning_evaluation_overlap_rejected(self):
        issues = _spec(tuning_seeds=(11, 12)).validate()
        assert any("separate" in i and "11" in i for i in issues)

    def test_identical_arm_ids_rejected(self):
        issues = _spec(
            arm_a=ComparisonArm("same", "navfn_planner", "teb_local_planner"),
            arm_b=ComparisonArm("same", "dijkstra_planner", "teb_local_planner"),
        ).validate()
        assert any("distinct arm_id" in i for i in issues)

    def test_non_positive_timeout_rejected(self):
        assert any("timeout" in i for i in _spec(timeout_seconds=0.0).validate())
        assert any("timeout" in i for i in _spec(timeout_seconds=-1.0).validate())

    def test_non_positive_tolerance_rejected(self):
        assert any("goal_tolerance" in i for i in _spec(goal_tolerance_m=0.0).validate())

    def test_non_positive_budget_rejected(self):
        spec = _spec(resource_budget=ResourceBudget(3600.0, 0.0, 100.0))
        assert any("max_memory_mb" in i for i in spec.validate())

    def test_unjustified_threshold_rejected(self):
        spec = _spec(thresholds=(ThresholdSpec("max_path_length_m", 12.0, "  "),))
        assert any("justification" in i for i in spec.validate())

    def test_non_finite_threshold_rejected(self):
        spec = _spec(
            thresholds=(ThresholdSpec("max_path_length_m", float("nan"), JUSTIFIED),)
        )
        assert any("not finite" in i for i in spec.validate())

    def test_validate_or_raise_raises(self):
        with pytest.raises(ComparisonValidationError):
            _spec(evaluation_seeds=(1,)).validate_or_raise()

    def test_min_evaluation_seeds_constant(self):
        assert MIN_EVALUATION_SEEDS == 5


# ------------------------------------------------------------------
# Seed separation
# ------------------------------------------------------------------


class TestSeedSeparation:
    def test_default_comparison_seeds_disjoint(self):
        spec = make_bumperbot_comparison()
        assert not (set(spec.evaluation_seeds) & set(spec.tuning_seeds))

    def test_default_has_five_evaluation_seeds(self):
        assert len(make_bumperbot_comparison().evaluation_seeds) == 5

    def test_only_evaluation_seeds_run_by_default(self):
        result = run_comparison(_spec(), _fake_runner)
        seeds = sorted({r.seed for r in result.runs})
        assert seeds == sorted(EVAL_SEEDS)

    def test_run_count_is_arms_times_seeds(self):
        result = run_comparison(_spec(), _fake_runner)
        assert len(result.runs) == 2 * len(EVAL_SEEDS)

    def test_all_runs_tagged_evaluation(self):
        result = run_comparison(_spec(), _fake_runner)
        assert all(r.phase == "evaluation" for r in result.runs)

    def test_tuning_runs_tagged_and_extra(self):
        result = run_comparison(_spec(), _fake_runner, include_tuning=True)
        assert len(result.runs) == 2 * (len(EVAL_SEEDS) + len(TUNING_SEEDS))
        tuning = [r for r in result.runs if r.phase == "tuning"]
        assert sorted({r.seed for r in tuning}) == sorted(TUNING_SEEDS)

    def test_summary_excludes_tuning_runs(self):
        result = run_comparison(_spec(), _fake_runner, include_tuning=True)
        summary = summarize_comparison(result)
        assert summary["total_evaluation_runs"] == 2 * len(EVAL_SEEDS)
        for arm in summary["per_arm"].values():
            assert arm["total_runs"] == len(EVAL_SEEDS)

    def test_invalid_spec_not_executed(self):
        calls = []

        def runner(arm, seed):
            calls.append((arm.arm_id, seed))
            return _rec(arm.arm_id, seed)

        with pytest.raises(ComparisonValidationError):
            run_comparison(_spec(evaluation_seeds=(1,)), runner)
        assert calls == []


# ------------------------------------------------------------------
# Manifest reproduction
# ------------------------------------------------------------------


class TestManifest:
    def test_deterministic_hash(self):
        a = build_comparison_manifest(_spec(), revision="abc123", dirty=False)
        b = build_comparison_manifest(_spec(), revision="abc123", dirty=False)
        assert a["manifest_hash"] == b["manifest_hash"]

    def test_hash_changes_with_revision(self):
        a = build_comparison_manifest(_spec(), revision="abc123", dirty=False)
        b = build_comparison_manifest(_spec(), revision="def456", dirty=False)
        assert a["manifest_hash"] != b["manifest_hash"]

    def test_hash_changes_with_dirty_state(self):
        a = build_comparison_manifest(_spec(), revision="abc123", dirty=False)
        b = build_comparison_manifest(_spec(), revision="abc123", dirty=True)
        assert a["manifest_hash"] != b["manifest_hash"]

    def test_hash_changes_with_seeds(self):
        a = build_comparison_manifest(_spec(), revision="abc123", dirty=False)
        b = build_comparison_manifest(
            _spec(evaluation_seeds=(2, 4, 6, 8, 10)), revision="abc123", dirty=False
        )
        assert a["manifest_hash"] != b["manifest_hash"]

    def test_manifest_pins_required_fields(self):
        deps = {"ros2": "humble"}
        assets = {"arena.sdf": "deadbeef"}
        m = build_comparison_manifest(
            _spec(), revision="abc123", dirty=False,
            dependencies=deps, asset_hashes=assets,
        )
        assert m["revision"] == "abc123"
        assert m["dirty"] is False
        assert m["dependencies"] == deps
        assert m["asset_hashes"] == assets
        assert m["spec"]["evaluation_seeds"] == sorted(EVAL_SEEDS)
        assert m["spec"]["timeout_seconds"] == 300.0
        assert m["spec"]["goal_tolerance_m"] == 0.25
        assert m["spec"]["resource_budget"]["max_memory_mb"] == 2048.0
        assert m["spec"]["artifact_paths"]

    def test_verify_accepts_matching_manifest(self):
        m = build_comparison_manifest(_spec(), revision="abc123", dirty=False)
        assert verify_comparison_manifest(_spec(), m) == []

    def test_verify_detects_tampered_seed(self):
        m = build_comparison_manifest(_spec(), revision="abc123", dirty=False)
        m["spec"]["evaluation_seeds"] = [1, 2, 3, 4, 5]
        assert verify_comparison_manifest(_spec(), m)

    def test_verify_detects_tampered_revision(self):
        m = build_comparison_manifest(_spec(), revision="abc123", dirty=False)
        m["revision"] = "other"
        assert verify_comparison_manifest(_spec(), m)


# ------------------------------------------------------------------
# Failure retention
# ------------------------------------------------------------------


class TestFailureRetention:
    def test_failed_run_present_in_runs(self):
        result = run_comparison(_spec(), _fake_runner)
        failed = [r for r in result.runs if not r.is_success]
        assert len(failed) == 1
        assert failed[0].arm_id == "b"
        assert failed[0].seed == 41
        assert failed[0].outcome_kind == "collision"

    def test_failures_retained_in_summary_with_traces(self):
        result = run_comparison(_spec(), _fake_runner)
        summary = summarize_comparison(result)
        failures = summary["per_arm"]["b"]["failures"]
        assert len(failures) == 1
        assert failures[0]["seed"] == 41
        assert failures[0]["outcome_kind"] == "collision"
        assert failures[0]["trace_paths"]

    def test_failure_rate_computed(self):
        summary = summarize_comparison(run_comparison(_spec(), _fake_runner))
        assert summary["per_arm"]["a"]["failure_rate"] == 0.0
        assert summary["per_arm"]["b"]["failure_rate"] == pytest.approx(1 / 5)

    def test_retained_failure_count_total(self):
        summary = summarize_comparison(run_comparison(_spec(), _fake_runner))
        assert summary["retained_failure_count"] == 1

    def test_success_rate(self):
        summary = summarize_comparison(run_comparison(_spec(), _fake_runner))
        assert summary["per_arm"]["a"]["success_rate"] == 1.0


# ------------------------------------------------------------------
# Distributions and median deltas
# ------------------------------------------------------------------


class TestDistributions:
    def test_metric_distribution_known_values(self):
        d = metric_distribution([1.0, 2.0, 3.0, 4.0])
        assert d["n"] == 4
        assert d["mean"] == 2.5
        assert d["median"] == 2.5
        assert d["min"] == 1.0
        assert d["max"] == 4.0

    def test_stdev_of_constant_series_is_zero(self):
        assert metric_distribution([5.0, 5.0, 5.0])["stdev"] == 0.0

    def test_empty_distribution(self):
        d = metric_distribution([])
        assert d["n"] == 0
        assert d["mean"] is None

    def test_distribution_excludes_none_metrics(self):
        result = run_comparison(_spec(), _fake_runner)
        summary = summarize_comparison(result)
        dist = summary["per_arm"]["b"]["distributions"]["path_length_m"]
        assert dist["n"] == 4  # seed 41 failure contributes no path length

    def test_median_delta_between_arms(self):
        summary = summarize_comparison(run_comparison(_spec(), _fake_runner))
        delta = summary["median_delta_b_minus_a"]["path_length_m"]
        # navfn path lengths: 8+seed%3; dijkstra: 9+seed%3 -> delta = 1.0
        assert delta == pytest.approx(1.0)

    def test_summary_reports_distributions_not_best_run(self):
        payload = summarize_comparison(run_comparison(_spec(), _fake_runner))
        assert "distributions" in payload["per_arm"]["a"]
        assert "best_run" not in json.dumps(payload)


# ------------------------------------------------------------------
# Report writing
# ------------------------------------------------------------------


class TestReport:
    def test_report_written_and_valid(self, tmp_path):
        result = run_comparison(_spec(), _fake_runner)
        path = write_comparison_report(result, tmp_path / "reports" / "r.json")
        data = json.loads(path.read_text())
        assert data["report_kind"] == "planner_comparison"
        assert data["report_schema_version"] == "1.0"
        assert data["manifest"]["manifest_hash"]
        assert data["summary"]["arm_ids"] == ["a", "b"]

    def test_report_includes_all_runs_including_failures(self, tmp_path):
        result = run_comparison(_spec(), _fake_runner)
        path = write_comparison_report(result, tmp_path / "r.json")
        data = json.loads(path.read_text())
        assert len(data["runs"]) == 10
        outcomes = {r["outcome_kind"] for r in data["runs"]}
        assert "collision" in outcomes

    def test_report_runs_carry_trace_paths(self, tmp_path):
        result = run_comparison(_spec(), _fake_runner)
        path = write_comparison_report(result, tmp_path / "r.json")
        data = json.loads(path.read_text())
        assert all(r["trace_paths"] for r in data["runs"])


# ------------------------------------------------------------------
# Registry integration (experiments.yaml)
# ------------------------------------------------------------------


REGISTRY_YAML = (
    Path(__file__).resolve().parents[3]
    / "robot_lab" / "robot_lab_registry" / "config" / "experiments.yaml"
)


class TestRegistryEntry:
    def _entry(self):
        entries = yaml.safe_load(REGISTRY_YAML.read_text())
        matches = [e for e in entries if e["id"] == "bumperbot_planner_comparison"]
        assert len(matches) == 1, "bumperbot_planner_comparison entry missing"
        return matches[0]

    def test_entry_builds_valid_spec(self):
        spec = ComparisonSpec.from_mapping(self._entry())
        assert spec.validate() == []

    def test_entry_matches_canonical_spec(self):
        from_mapping = ComparisonSpec.from_mapping(self._entry())
        canonical = make_bumperbot_comparison()
        assert from_mapping.to_dict() == canonical.to_dict()

    def test_entry_has_justified_thresholds(self):
        spec = ComparisonSpec.from_mapping(self._entry())
        assert len(spec.thresholds) >= 1
        assert all(t.justification.strip() for t in spec.thresholds)

    def test_entry_seeds_separate_and_sufficient(self):
        spec = ComparisonSpec.from_mapping(self._entry())
        assert len(spec.evaluation_seeds) >= 5
        assert not (set(spec.evaluation_seeds) & set(spec.tuning_seeds))

    def test_arms_differ_only_in_global_planner(self):
        spec = ComparisonSpec.from_mapping(self._entry())
        a, b = spec.arm_a, spec.arm_b
        assert a.global_planning != b.global_planning
        assert a.local_planning == b.local_planning
        assert a.parameters == b.parameters
