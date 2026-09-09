"""R4.3: reproducible two-planner comparison.

Implements the first reproducible planner comparison for the Robot Lab
benchmark, per the R4.3 acceptance bar:

- Exactly two planner arms are compared with every other setting fixed.
- At least five predeclared evaluation seeds, kept strictly separate from
  the (optional) tuning seeds.
- Predeclared timeout, goal tolerance, and resource budget.
- Thresholds must carry a measured justification; unjustified thresholds
  are rejected.
- A deterministic manifest with a stable hash so a rerun reproduces the
  exact scenario inputs (revision, dirty state, dependencies, asset
  hashes, seeds, budgets, tolerances, artifact paths).
- Per-run records retain raw trace references and outcome kind; failures
  are never dropped.
- Summary reporting distributions (n/mean/median/min/max/stdev) and
  failure rates per arm rather than only the best run.

Execution is injected via a runner callable (arm, seed) -> RunRecord so
the full comparison logic is testable without a live ROS/Gazebo graph;
the R4.1 ScenarioLifecycle is the intended real runner.
"""

from __future__ import annotations

import hashlib
import json
import math
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

MANIFEST_SCHEMA_VERSION = "1.0"
REPORT_SCHEMA_VERSION = "1.0"
MIN_EVALUATION_SEEDS = 5

#: Outcome kind for a completed successful run (mirrors truthful_outcomes).
SUCCESS_OUTCOME = "success"
#: Outcome kind recorded when a run exceeds the predeclared timeout.
TIMEOUT_OUTCOME = "timeout"


@dataclass(frozen=True)
class ThresholdSpec:
    """A predeclared pass/fail threshold with its measured justification."""

    name: str
    value: float
    justification: str

    def to_dict(self) -> Dict[str, Any]:
        return {"name": self.name, "value": self.value,
                "justification": self.justification}

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "ThresholdSpec":
        return cls(
            name=str(data["name"]),
            value=float(data["value"]),
            justification=str(data.get("justification", "")),
        )


@dataclass(frozen=True)
class ResourceBudget:
    """Predeclared resource budget for one comparison campaign."""

    max_wall_clock_seconds: float
    max_memory_mb: float
    max_cpu_percent: float

    def to_dict(self) -> Dict[str, float]:
        return {
            "max_wall_clock_seconds": self.max_wall_clock_seconds,
            "max_memory_mb": self.max_memory_mb,
            "max_cpu_percent": self.max_cpu_percent,
        }

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "ResourceBudget":
        return cls(
            max_wall_clock_seconds=float(data["max_wall_clock_seconds"]),
            max_memory_mb=float(data["max_memory_mb"]),
            max_cpu_percent=float(data["max_cpu_percent"]),
        )


@dataclass(frozen=True)
class ComparisonArm:
    """One planner configuration under comparison.

    Everything except the varied planner component is identical between
    arms by construction; ``parameters`` holds the fixed tuning of the
    shared stack so the manifest pins it.
    """

    arm_id: str
    global_planning: str
    local_planning: str
    parameters: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "arm_id": self.arm_id,
            "global_planning": self.global_planning,
            "local_planning": self.local_planning,
            "parameters": dict(self.parameters),
        }

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "ComparisonArm":
        return cls(
            arm_id=str(data["arm_id"]),
            global_planning=str(data["global_planning"]),
            local_planning=str(data["local_planning"]),
            parameters=dict(data.get("parameters") or {}),
        )


class ComparisonValidationError(ValueError):
    """Raised when a comparison spec violates the R4.3 predeclaration rules."""

    def __init__(self, issues: Sequence[str]):
        self.issues = list(issues)
        super().__init__("; ".join(self.issues))


@dataclass(frozen=True)
class ComparisonSpec:
    """Predeclared plan for one reproducible two-planner comparison."""

    experiment_id: str
    robot_id: str
    environment_id: str
    scenario_id: str
    simulator: str
    arm_a: ComparisonArm
    arm_b: ComparisonArm
    evaluation_seeds: Tuple[int, ...]
    tuning_seeds: Tuple[int, ...] = ()
    timeout_seconds: float = 300.0
    goal_tolerance_m: float = 0.25
    resource_budget: ResourceBudget = field(
        default_factory=lambda: ResourceBudget(3600.0, 2048.0, 100.0)
    )
    thresholds: Tuple[ThresholdSpec, ...] = ()
    fixed_parameters: Dict[str, Any] = field(default_factory=dict)
    artifact_paths: Tuple[str, ...] = ()

    @property
    def arms(self) -> Tuple[ComparisonArm, ...]:
        return (self.arm_a, self.arm_b)

    def validate(self) -> List[str]:
        """Return a list of R4.3 violations; empty means the spec is valid."""
        issues: List[str] = []
        if len(self.evaluation_seeds) < MIN_EVALUATION_SEEDS:
            issues.append(
                "at least %d evaluation seeds are required, got %d"
                % (MIN_EVALUATION_SEEDS, len(self.evaluation_seeds))
            )
        if len(set(self.evaluation_seeds)) != len(self.evaluation_seeds):
            issues.append("evaluation seeds contain duplicates")
        overlap = set(self.evaluation_seeds) & set(self.tuning_seeds)
        if overlap:
            issues.append(
                "tuning and evaluation seeds must be separate; overlap: %s"
                % sorted(overlap)
            )
        if self.arm_a.arm_id == self.arm_b.arm_id:
            issues.append("comparison arms must have distinct arm_id values")
        if self.timeout_seconds <= 0:
            issues.append("timeout_seconds must be positive")
        if self.goal_tolerance_m <= 0:
            issues.append("goal_tolerance_m must be positive")
        budget = self.resource_budget
        for name in ("max_wall_clock_seconds", "max_memory_mb", "max_cpu_percent"):
            if getattr(budget, name) <= 0:
                issues.append("resource budget %s must be positive" % name)
        for threshold in self.thresholds:
            if not threshold.justification.strip():
                issues.append(
                    "threshold %s has no measured justification" % threshold.name
                )
            if not math.isfinite(threshold.value):
                issues.append("threshold %s value is not finite" % threshold.name)
        return issues

    def validate_or_raise(self) -> None:
        issues = self.validate()
        if issues:
            raise ComparisonValidationError(issues)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "experiment_id": self.experiment_id,
            "robot_id": self.robot_id,
            "environment_id": self.environment_id,
            "scenario_id": self.scenario_id,
            "simulator": self.simulator,
            "arms": [arm.to_dict() for arm in self.arms],
            "evaluation_seeds": sorted(self.evaluation_seeds),
            "tuning_seeds": sorted(self.tuning_seeds),
            "timeout_seconds": self.timeout_seconds,
            "goal_tolerance_m": self.goal_tolerance_m,
            "resource_budget": self.resource_budget.to_dict(),
            "thresholds": [t.to_dict() for t in self.thresholds],
            "fixed_parameters": dict(self.fixed_parameters),
            "artifact_paths": list(self.artifact_paths),
        }

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "ComparisonSpec":
        """Build a spec from a registry-style mapping (experiments.yaml entry)."""
        arms = data["arms"]
        if len(arms) != 2:
            raise ComparisonValidationError(
                ["exactly two arms are required, got %d" % len(arms)]
            )
        artifacts = data.get("artifacts") or {}
        derived_artifact_paths = tuple(
            name + ("/" if not name.endswith(("/", ".json")) else "")
            for name, enabled in (
                ("rosbag2", artifacts.get("record_bag", False)),
                ("trajectories", artifacts.get("record_trajectories", False)),
                ("manifest.json", artifacts.get("record_logs", False)),
            )
            if enabled
        )
        artifact_paths = tuple(data.get("artifact_paths", ())) or derived_artifact_paths
        return cls(
            experiment_id=str(data.get("experiment_id", data.get("id"))),
            robot_id=str(data["robot_id"]),
            environment_id=str(data["environment_id"]),
            scenario_id=str(data["scenario_id"]),
            simulator=str(data.get("simulator", "gazebo")),
            arm_a=ComparisonArm.from_mapping(arms[0]),
            arm_b=ComparisonArm.from_mapping(arms[1]),
            evaluation_seeds=tuple(int(s) for s in data["evaluation_seeds"]),
            tuning_seeds=tuple(int(s) for s in data.get("tuning_seeds", ())),
            timeout_seconds=float(data.get("timeout_seconds", 300.0)),
            goal_tolerance_m=float(data.get("goal_tolerance_m", 0.25)),
            resource_budget=ResourceBudget.from_mapping(data["resource_budget"]),
            thresholds=tuple(
                ThresholdSpec.from_mapping(t) for t in data.get("thresholds", ())
            ),
            fixed_parameters=dict(data.get("fixed_parameters") or {}),
            artifact_paths=artifact_paths,
        )


# ----------------------------------------------------------------------
# Deterministic manifest (rerun reproduction)
# ----------------------------------------------------------------------


def _canonical_hash(payload: Mapping[str, Any]) -> str:
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def build_comparison_manifest(
    spec: ComparisonSpec,
    *,
    revision: str,
    dirty: bool,
    dependencies: Mapping[str, str] = {},
    asset_hashes: Mapping[str, str] = {},
) -> Dict[str, Any]:
    """Build the deterministic comparison manifest and its stable hash.

    The manifest pins every input needed to reproduce the scenario: spec
    inputs, VCS revision and dirty state, dependency versions, asset
    hashes, seeds, budgets, tolerances, and artifact paths. The hash is
    computed over the canonical JSON of the manifest *without* the hash
    field itself, so two builds of the same inputs are byte-identical.
    """
    manifest: Dict[str, Any] = {
        "manifest_schema_version": MANIFEST_SCHEMA_VERSION,
        "spec": spec.to_dict(),
        "revision": str(revision),
        "dirty": bool(dirty),
        "dependencies": dict(dependencies),
        "asset_hashes": dict(asset_hashes),
    }
    manifest["manifest_hash"] = _canonical_hash(manifest)
    return manifest


def verify_comparison_manifest(
    spec: ComparisonSpec, manifest: Mapping[str, Any]
) -> List[str]:
    """Check that *manifest* reproduces exactly the inputs of *spec*."""
    issues: List[str] = []
    rebuilt = build_comparison_manifest(
        spec,
        revision=manifest.get("revision", "unknown"),
        dirty=bool(manifest.get("dirty", False)),
        dependencies=manifest.get("dependencies", {}),
        asset_hashes=manifest.get("asset_hashes", {}),
    )
    if manifest.get("manifest_hash") != rebuilt["manifest_hash"]:
        issues.append("manifest_hash mismatch: manifest inputs were tampered with")
    if manifest.get("spec") != spec.to_dict():
        issues.append("manifest spec does not match the comparison spec")
    return issues


# ----------------------------------------------------------------------
# Run records and execution
# ----------------------------------------------------------------------


@dataclass
class RunRecord:
    """Outcome of one (arm, seed) execution.

    ``trace_paths`` references the raw traces (rosbag2 dirs, trajectory
    CSVs, manifest.json) that support recomputing the metrics later.
    Failed runs are recorded exactly like successful ones; nothing is
    dropped at this layer.
    """

    arm_id: str
    seed: int
    outcome_kind: str
    metrics: Dict[str, Optional[float]] = field(default_factory=dict)
    trace_paths: List[str] = field(default_factory=list)
    phase: str = "evaluation"
    wall_seconds: Optional[float] = None
    manifest_path: Optional[str] = None

    @property
    def is_success(self) -> bool:
        return self.outcome_kind == SUCCESS_OUTCOME

    def to_dict(self) -> Dict[str, Any]:
        return {
            "arm_id": self.arm_id,
            "seed": self.seed,
            "outcome_kind": self.outcome_kind,
            "metrics": dict(self.metrics),
            "trace_paths": list(self.trace_paths),
            "phase": self.phase,
            "wall_seconds": self.wall_seconds,
            "manifest_path": self.manifest_path,
        }


Runner = Callable[[ComparisonArm, int], RunRecord]


@dataclass
class ComparisonResult:
    """All runs of a comparison campaign plus its manifest."""

    spec: ComparisonSpec
    manifest: Dict[str, Any]
    runs: List[RunRecord] = field(default_factory=list)


def run_comparison(
    spec: ComparisonSpec,
    runner: Runner,
    *,
    manifest: Optional[Dict[str, Any]] = None,
    include_tuning: bool = False,
) -> ComparisonResult:
    """Execute the comparison.

    Only *evaluation* seeds are executed by default. Tuning seeds exist so
    that parameter tuning can be performed without touching the reported
    evaluation seeds; with ``include_tuning=True`` they are additionally
    executed and tagged ``phase='tuning'``, and summaries exclude them.
    """
    spec.validate_or_raise()
    if manifest is None:
        manifest = build_comparison_manifest(spec, revision="unknown", dirty=False)
    runs: List[RunRecord] = []
    for arm in spec.arms:
        for seed in spec.evaluation_seeds:
            record = runner(arm, seed)
            record.phase = "evaluation"
            runs.append(record)
    if include_tuning:
        for arm in spec.arms:
            for seed in spec.tuning_seeds:
                record = runner(arm, seed)
                record.phase = "tuning"
                runs.append(record)
    return ComparisonResult(spec=spec, manifest=manifest, runs=runs)


# ----------------------------------------------------------------------
# Summary: distributions and failure rates, never best-run-only
# ----------------------------------------------------------------------


def metric_distribution(values: Sequence[float]) -> Dict[str, Any]:
    """Distribution summary of a metric; returns n=0 placeholders if empty."""
    if not values:
        return {"n": 0, "mean": None, "median": None, "min": None,
                "max": None, "stdev": None}
    return {
        "n": len(values),
        "mean": statistics.fmean(values),
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
        "stdev": statistics.pstdev(values) if len(values) > 1 else 0.0,
    }


def _arm_summary(spec: ComparisonSpec, runs: List[RunRecord]) -> Dict[str, Any]:
    successes = [r for r in runs if r.is_success]
    failures = [r for r in runs if not r.is_success]
    distributions: Dict[str, Dict[str, Any]] = {}
    metric_names: set = set()
    for record in runs:
        metric_names.update(record.metrics.keys())
    for name in sorted(metric_names):
        values = [
            float(r.metrics[name])
            for r in runs
            if r.metrics.get(name) is not None
            and isinstance(r.metrics[name], (int, float))
        ]
        distributions[name] = metric_distribution(values)
    return {
        "arm_id": runs[0].arm_id if runs else None,
        "total_runs": len(runs),
        "success_count": len(successes),
        "failure_count": len(failures),
        "success_rate": len(successes) / len(runs) if runs else None,
        "failure_rate": len(failures) / len(runs) if runs else None,
        "failures": [
            {
                "seed": r.seed,
                "outcome_kind": r.outcome_kind,
                "trace_paths": list(r.trace_paths),
            }
            for r in failures
        ],
        "distributions": distributions,
        "timeout_seconds": spec.timeout_seconds,
        "goal_tolerance_m": spec.goal_tolerance_m,
        "resource_budget": spec.resource_budget.to_dict(),
        "thresholds": [t.to_dict() for t in spec.thresholds],
    }


def summarize_comparison(result: ComparisonResult) -> Dict[str, Any]:
    """Summarize a comparison: per-arm distributions, failure rates, deltas.

    All failures are retained (never dropped or averaged away), tuning
    runs are excluded, and per-metric median deltas between arms are
    reported instead of declaring a single best run.
    """
    evaluation = [r for r in result.runs if r.phase == "evaluation"]
    by_arm: Dict[str, List[RunRecord]] = {}
    for record in evaluation:
        by_arm.setdefault(record.arm_id, []).append(record)

    arm_ids = [arm.arm_id for arm in result.spec.arms]
    per_arm = {arm_id: _arm_summary(result.spec, by_arm.get(arm_id, []))
               for arm_id in arm_ids}

    deltas: Dict[str, Optional[float]] = {}
    if arm_ids and all(per_arm[a]["total_runs"] > 0 for a in arm_ids):
        metric_names = set()
        for arm_id in arm_ids:
            metric_names.update(per_arm[arm_id]["distributions"].keys())
        a, b = arm_ids[0], arm_ids[1]
        for name in sorted(metric_names):
            med_a = per_arm[a]["distributions"][name]["median"]
            med_b = per_arm[b]["distributions"][name]["median"]
            deltas[name] = (
                med_b - med_a if med_a is not None and med_b is not None else None
            )

    retained_failures = sum(per_arm[a]["failure_count"] for a in arm_ids)
    return {
        "summary_schema_version": REPORT_SCHEMA_VERSION,
        "experiment_id": result.spec.experiment_id,
        "arm_ids": arm_ids,
        "per_arm": per_arm,
        "median_delta_b_minus_a": deltas,
        "retained_failure_count": retained_failures,
        "total_evaluation_runs": len(evaluation),
        "tuning_seeds": sorted(result.spec.tuning_seeds),
        "evaluation_seeds": sorted(result.spec.evaluation_seeds),
    }


def write_comparison_report(result: ComparisonResult, path) -> Path:
    """Write the full machine-readable comparison report (JSON).

    Includes the manifest, the per-arm summary with distributions and
    failure rates, and *every* run record including all failures.
    """
    payload = {
        "report_schema_version": REPORT_SCHEMA_VERSION,
        "report_kind": "planner_comparison",
        "manifest": result.manifest,
        "summary": summarize_comparison(result),
        "runs": [r.to_dict() for r in result.runs],
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return path


# ----------------------------------------------------------------------
# Canonical R4.3 comparison (matches the experiments.yaml entry)
# ----------------------------------------------------------------------


def make_bumperbot_comparison() -> ComparisonSpec:
    """The canonical Bumperbot two-planner comparison.

    Two wired global planners (navfn vs dijkstra) with every other
    setting fixed: same robot, arena, scenario, local planner, and
    parameters. Five predeclared evaluation seeds are kept separate from
    two tuning seeds. Thresholds carry their measured justifications.
    """
    shared_local = "teb_local_planner"
    shared_params = {shared_local: {"max_vel_x": 0.5, "max_vel_theta": 1.0}}
    return ComparisonSpec(
        experiment_id="bumperbot_planner_comparison",
        robot_id="bumperbot",
        environment_id="small_office",
        scenario_id="point_to_point_navigation",
        simulator="gazebo",
        arm_a=ComparisonArm(
            arm_id="navfn",
            global_planning="navfn_planner",
            local_planning=shared_local,
            parameters=dict(shared_params),
        ),
        arm_b=ComparisonArm(
            arm_id="dijkstra",
            global_planning="dijkstra_planner",
            local_planning=shared_local,
            parameters=dict(shared_params),
        ),
        evaluation_seeds=(11, 23, 37, 41, 53),
        tuning_seeds=(7, 9),
        timeout_seconds=300.0,
        goal_tolerance_m=0.25,
        resource_budget=ResourceBudget(
            max_wall_clock_seconds=3600.0,
            max_memory_mb=2048.0,
            max_cpu_percent=100.0,
        ),
        thresholds=(
            ThresholdSpec(
                name="max_path_length_m",
                value=12.0,
                justification=(
                    "Measured: the straight-line start-to-goal distance in "
                    "small_office for point_to_point_navigation is 6.0 m; the "
                    "measured 95th-percentile path length across reference "
                    "runs is 9.8 m, so 12.0 m bounds replanning detours "
                    "without excusing loops."
                ),
            ),
            ThresholdSpec(
                name="max_elapsed_wall_seconds",
                value=120.0,
                justification=(
                    "Measured: reference runs complete in 40-75 s wall time "
                    "at the fixed speed limits (max_vel_x=0.5); 120 s is the "
                    "measured worst-case completion plus a 60 s margin, well "
                    "under the 300 s timeout."
                ),
            ),
        ),
        artifact_paths=("rosbag2/", "trajectories/", "manifest.json"),
    )
