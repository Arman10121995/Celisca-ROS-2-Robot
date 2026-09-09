"""Robot Lab benchmark package (R4.x).

This package provides the scenario lifecycle, truthful outcome classification,
and (in later R4.x modules) metric extraction, normalization, aggregation,
reference baselines, and report generation for reproducible planner/algorithm
comparisons.

R4.1 modules:
- task_lifecycle.py: scenario lifecycle orchestration (validate -> launch ->
  ready -> reset/seed -> initialize -> task -> observe -> stop -> record)
- truthful_outcomes.py: terminal outcome taxonomy and classification

R4.2 modules:
- metrics.py: measured/derived metric extraction with truthful None-on-missing
  semantics (trajectory distance, contact events, footprint clearance,
  timestamp-aligned ground-truth error, RTF, effort proxy) and schema
  validation
"""

from __future__ import annotations

from robot_lab_benchmark.metrics import (
    MeasuredMetrics,
    DerivedMetrics,
    RunMetrics,
    compute_derived_metrics,
    compose_run_metrics,
    validate_metric_value,
    trajectory_distance,
    contact_events,
    footprint_clearance,
    timestamp_aligned_error,
    compute_rtf,
    effort_proxy,
)

from robot_lab_benchmark.qualification import (
    LinkInfo,
    WheelInfo,
    RobotDescription,
    RobotGolden,
    CheckResult,
    QualificationReport,
    TaskStageResult,
    TaskTrialSpec,
    TaskTrialObservation,
    GOLDEN,
    QUALIFICATION_SCHEMA_VERSION,
    parse_robot_description,
    golden_for,
    run_static_checks,
    qualify_robot,
    write_qualification_report,
)

from robot_lab_benchmark.comparison import (
    ComparisonArm,
    ComparisonSpec,
    ComparisonResult,
    ComparisonValidationError,
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

import datetime as _dt
from typing import Any, Dict, Optional


class BenchmarkResult:
    """Canonical benchmark result record for a Robot Lab experiment."""

    schema_version = '1.0'

    def __init__(
        self,
        experiment_id: str,
        robot_id: str,
        environment_id: str,
        scenario_id: str,
        seed: int,
        success: bool,
        elapsed_seconds: float,
        path_length_m: float,
        collision_count: int,
        min_clearance_m: float,
        revision: str = 'unknown',
        timestamp_utc: Optional[str] = None,
        **kwargs: Any,
    ) -> None:
        self.experiment_id = experiment_id
        self.robot_id = robot_id
        self.environment_id = environment_id
        self.scenario_id = scenario_id
        self.seed = int(seed)
        self.success = bool(success)
        self.elapsed_seconds = float(elapsed_seconds)
        self.path_length_m = float(path_length_m)
        self.collision_count = int(collision_count)
        self.min_clearance_m = float(min_clearance_m)
        self.revision = revision
        self.timestamp_utc = timestamp_utc or _dt.datetime.now(_dt.timezone.utc).isoformat()
        self.extra = kwargs

    def validate(self) -> bool:
        required = [
            self.experiment_id,
            self.robot_id,
            self.environment_id,
            self.scenario_id,
            self.seed,
            self.success,
            self.elapsed_seconds,
            self.path_length_m,
            self.collision_count,
            self.min_clearance_m,
        ]
        return all(value is not None for value in required)

    def to_dict(self) -> Dict[str, Any]:
        payload = {
            'schema_version': self.schema_version,
            'experiment_id': self.experiment_id,
            'robot_id': self.robot_id,
            'environment_id': self.environment_id,
            'scenario_id': self.scenario_id,
            'timestamp_utc': self.timestamp_utc,
            'revision': self.revision,
            'seed': self.seed,
            'success': self.success,
            'elapsed_seconds': self.elapsed_seconds,
            'path_length_m': self.path_length_m,
            'collision_count': self.collision_count,
            'min_clearance_m': self.min_clearance_m,
        }
        payload.update(self.extra)
        return payload


from .task_lifecycle import (
    LifecyclePhase,
    ScenarioLifecycle,
    make_lifecycle,
)
from .truthful_outcomes import (
    OutcomeKind,
    TERMINAL_OUTCOMES,
    ABORT_OUTCOMES,
    classify_outcome,
    outcome_to_dict,
    write_outcome_record,
    outcome_is_terminal,
    outcome_is_abort,
    outcome_success,
    build_result_id,
    all_outcome_kinds,
    terminal_outcome_kinds,
    abort_outcome_kinds,
)

__all__ = [
    # Core result record
    "BenchmarkResult",
    # Lifecycle
    "LifecyclePhase",
    "ScenarioLifecycle",
    "make_lifecycle",
    # Outcomes
    "OutcomeKind",
    "TERMINAL_OUTCOMES",
    "ABORT_OUTCOMES",
    "classify_outcome",
    "outcome_to_dict",
    "write_outcome_record",
    "outcome_is_terminal",
    "outcome_is_abort",
    "outcome_success",
    "build_result_id",
    "all_outcome_kinds",
    "terminal_outcome_kinds",
    "abort_outcome_kinds",
    # Metrics (R4.2)
    "MeasuredMetrics",
    "DerivedMetrics",
    "RunMetrics",
    "compute_derived_metrics",
    "compose_run_metrics",
    "validate_metric_value",
    "trajectory_distance",
    "contact_events",
    "footprint_clearance",
    "timestamp_aligned_error",
    "compute_rtf",
    "effort_proxy",
    # Comparison (R4.3)
    "ComparisonArm",
    "ComparisonSpec",
    "ComparisonResult",
    "ComparisonValidationError",
    "ResourceBudget",
    "RunRecord",
    "ThresholdSpec",
    "MIN_EVALUATION_SEEDS",
    "build_comparison_manifest",
    "verify_comparison_manifest",
    "run_comparison",
    "summarize_comparison",
    "metric_distribution",
    "write_comparison_report",
    "make_bumperbot_comparison",
    # Qualification (R5.1)
    "LinkInfo",
    "WheelInfo",
    "RobotDescription",
    "RobotGolden",
    "CheckResult",
    "QualificationReport",
    "TaskStageResult",
    "TaskTrialSpec",
    "TaskTrialObservation",
    "GOLDEN",
    "QUALIFICATION_SCHEMA_VERSION",
    "parse_robot_description",
    "golden_for",
    "run_static_checks",
    "qualify_robot",
    "write_qualification_report",
]
