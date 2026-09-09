"""Metric extraction for Robot Lab benchmark runs (R4.2).

Measures trajectory distance, contact events (not per-scan), footprint-aware
clearance, timestamp-aligned truth/estimation error, and system utilization
(CPU/memory/RTF/effort proxy).

Design contract (R4.2 acceptance):
- Known trajectories/contact fixtures yield correct values.
- One contact event is counted once even if many scans observe it.
- Missing/NaN/stale data invalidate a metric (None) instead of becoming 0.0.
- The schema distinguishes measured (raw) metrics from derived (computed) ones
  and rejects invalid values.

This module is self-contained and has no ROS dependency so it can be unit
tested deterministically. Live ingestion of ROS topics is the job of the
launch_orchestrator / a future sensor adapter, not this module.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


# ------------------------------------------------------------------
# Measurement primitives (pure functions)
# ------------------------------------------------------------------


def _is_valid(value: Optional[float]) -> bool:
    """True if *value* is a finite real number (not None, NaN, Inf)."""
    if value is None:
        return False
    if not isinstance(value, (int, float)):
        return False
    return math.isfinite(value)


def trajectory_distance(poses: List[Tuple[float, float]]) -> Optional[float]:
    """Total trajectory length from a sequence of (x, y) poses.

    Returns None when there are fewer than 2 poses or any pose contains a
    non-finite coordinate (missing/NaN invalidates, not zero): silently
    skipping bad poses would understate the measured path length.
    """
    if not poses:
        return None
    for x, y in poses:
        if not (_is_valid(x) and _is_valid(y)):
            return None
    if len(poses) < 2:
        return None
    length = 0.0
    for i in range(1, len(poses)):
        dx = poses[i][0] - poses[i - 1][0]
        dy = poses[i][1] - poses[i - 1][1]
        length += math.hypot(dx, dy)
    return length


def contact_events(
    clearances: List[Optional[float]],
    threshold: float = 0.05,
) -> Optional[int]:
    """Count distinct contact events from a clearance time series.

    A contact event begins when clearance drops below *threshold* and ends
    when it rises back to or above *threshold*. Consecutive below-threshold
    readings count as ONE event, not one per reading.

    Returns None when the input is empty or contains no valid readings.
    """
    if not clearances:
        return None
    events = 0
    in_contact = False
    had_valid = False
    for c in clearances:
        if not _is_valid(c):
            in_contact = False
            continue
        had_valid = True
        if c < threshold:
            if not in_contact:
                events += 1
                in_contact = True
        else:
            in_contact = False
    return events if had_valid else None


def footprint_clearance(
    clearances: List[Optional[float]],
    footprint_radius: float = 0.0,
) -> Optional[float]:
    """Minimum footprint-aware clearance.

    The robot occupies a disc of radius *footprint_radius*; the free space
    around it is the raw clearance minus that radius. Returns the minimum
    over all valid readings, or None when there are no valid readings.
    """
    valid = [c for c in clearances if _is_valid(c)]
    if not valid:
        return None
    return min(c - footprint_radius for c in valid)


def _interpolate_estimates(
    estimates: List[Tuple[float, float, float]],
    t_query: float,
) -> Optional[Tuple[float, float]]:
    """Linearly interpolate (x, y) of *estimates* at time *t_query*.

    Returns None if *t_query* is outside the estimates time span.
    """
    if not _is_valid(t_query):
        return None
    # estimates assumed sorted by time
    if t_query <= estimates[0][0]:
        return (estimates[0][1], estimates[0][2])
    if t_query >= estimates[-1][0]:
        return (estimates[-1][1], estimates[-1][2])
    for i in range(1, len(estimates)):
        t0, x0, y0 = estimates[i - 1]
        t1, x1, y1 = estimates[i]
        if t0 <= t_query <= t1:
            span = t1 - t0
            if span <= 0:
                return (x0, y0)
            alpha = (t_query - t0) / span
            return (x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0))
    return None


def timestamp_aligned_error(
    truth: List[Tuple[float, float, float]],
    estimates: List[Tuple[float, float, float]],
) -> Dict[str, Optional[float]]:
    """Timestamp/frame-aligned position error between truth and estimates.

    *truth* and *estimates* are lists of (t, x, y). Estimates are linearly
    interpolated onto the truth timestamps. Only the overlapping time window
    contributes. Returns max/mean/rmse of the Euclidean error, or None for
    each if there is no usable overlap.
    """
    t_valid = [(t, x, y) for t, x, y in truth
               if _is_valid(t) and _is_valid(x) and _is_valid(y)]
    e_valid = [(t, x, y) for t, x, y in estimates
               if _is_valid(t) and _is_valid(x) and _is_valid(y)]
    if len(t_valid) < 1 or len(e_valid) < 2:
        return {"max_error_m": None, "mean_error_m": None, "rmse_m": None}
    e_sorted = sorted(e_valid, key=lambda p: p[0])
    errors: List[float] = []
    for t, x, y in t_valid:
        pt = _interpolate_estimates(e_sorted, t)
        if pt is None:
            continue
        ex, ey = pt
        if not (_is_valid(ex) and _is_valid(ey)):
            continue
        errors.append(math.hypot(x - ex, y - ey))
    if not errors:
        return {"max_error_m": None, "mean_error_m": None, "rmse_m": None}
    max_e = max(errors)
    mean_e = sum(errors) / len(errors)
    rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
    return {"max_error_m": max_e, "mean_error_m": mean_e, "rmse_m": rmse}


def compute_rtf(sim_seconds: Optional[float], wall_seconds: Optional[float]) -> Optional[float]:
    """Real-time factor: sim time elapsed per wall-second.

    None if inputs are invalid or wall_seconds <= 0.
    """
    if not (_is_valid(sim_seconds) and _is_valid(wall_seconds)):
        return None
    if wall_seconds <= 0:
        return None
    return sim_seconds / wall_seconds


def effort_proxy(torque_samples: List[Optional[float]], dt: float = 1.0) -> Optional[float]:
    """Defined effort proxy: sum of |torque| * dt over valid samples.

    None if there are no valid samples or dt is invalid.
    """
    if not _is_valid(dt) or dt <= 0:
        return None
    valid = [t for t in torque_samples if _is_valid(t)]
    if not valid:
        return None
    return sum(abs(t) for t in valid) * dt


# ------------------------------------------------------------------
# Metric containers: measured (raw) vs derived (computed)
# ------------------------------------------------------------------


@dataclass
class MeasuredMetrics:
    """Raw metrics measured directly from data.

    Fields are None when the measurement is invalid/missing/stale.
    """
    path_length_m: Optional[float] = None
    contact_events: Optional[int] = None
    min_clearance_m: Optional[float] = None
    max_position_error_m: Optional[float] = None
    mean_position_error_m: Optional[float] = None
    rmse_position_error_m: Optional[float] = None
    elapsed_wall_seconds: Optional[float] = None
    elapsed_sim_seconds: Optional[float] = None
    cpu_percent: Optional[float] = None
    memory_mb: Optional[float] = None
    effort_proxy: Optional[float] = None


@dataclass
class DerivedMetrics:
    """Metrics computed from measured metrics."""
    rtf: Optional[float] = None
    path_efficiency: Optional[float] = None  # path_length / wall_time


@dataclass
class RunMetrics:
    """Complete measured + derived metrics for one run, with provenance."""
    measured: MeasuredMetrics = field(default_factory=MeasuredMetrics)
    derived: DerivedMetrics = field(default_factory=DerivedMetrics)
    provenance: Dict[str, Any] = field(default_factory=dict)
    issues: List[str] = field(default_factory=list)


# ------------------------------------------------------------------
# Derived metric computation
# ------------------------------------------------------------------


def compute_derived_metrics(
    measured: MeasuredMetrics,
    wall_seconds: Optional[float] = None,
) -> DerivedMetrics:
    """Compute derived metrics from a MeasuredMetrics plus optional wall time.

    RTF uses elapsed_sim_seconds / elapsed_wall_seconds (or the explicit
    *wall_seconds* when provided). Path efficiency is path_length / wall_time.

    All derived fields are None when their inputs are invalid or missing.
    """
    sim = measured.elapsed_sim_seconds
    wall = wall_seconds if wall_seconds is not None else measured.elapsed_wall_seconds
    rtf = compute_rtf(sim, wall) if (sim is not None and wall is not None) else None

    path = measured.path_length_m
    eff = (path / wall) if (path is not None and wall is not None and wall > 0) else None

    return DerivedMetrics(rtf=rtf, path_efficiency=eff)


def compose_run_metrics(
    *,
    path_length_m: Optional[float] = None,
    contact_events: Optional[int] = None,
    min_clearance_m: Optional[float] = None,
    max_position_error_m: Optional[float] = None,
    mean_position_error_m: Optional[float] = None,
    rmse_position_error_m: Optional[float] = None,
    elapsed_wall_seconds: Optional[float] = None,
    elapsed_sim_seconds: Optional[float] = None,
    cpu_percent: Optional[float] = None,
    memory_mb: Optional[float] = None,
    effort_proxy: Optional[float] = None,
    wall_seconds_for_derived: Optional[float] = None,
    provenance: Optional[Dict[str, Any]] = None,
    issues: Optional[List[str]] = None,
) -> RunMetrics:
    """Build a complete RunMetrics from measured values and optional provenance.

    Parameters mirror MeasuredMetrics fields; *wall_seconds_for_derived* is an
    optional explicit wall time used for derived computation when the measured
    elapsed_wall_seconds is unavailable.
    """
    measured = MeasuredMetrics(
        path_length_m=path_length_m,
        contact_events=contact_events,
        min_clearance_m=min_clearance_m,
        max_position_error_m=max_position_error_m,
        mean_position_error_m=mean_position_error_m,
        rmse_position_error_m=rmse_position_error_m,
        elapsed_wall_seconds=elapsed_wall_seconds,
        elapsed_sim_seconds=elapsed_sim_seconds,
        cpu_percent=cpu_percent,
        memory_mb=memory_mb,
        effort_proxy=effort_proxy,
    )
    derived = compute_derived_metrics(measured, wall_seconds_for_derived)
    return RunMetrics(
        measured=measured,
        derived=derived,
        provenance=provenance if provenance is not None else {},
        issues=list(issues) if issues is not None else [],
    )


# ------------------------------------------------------------------
# Metric value validation (schema enforcement)
# ------------------------------------------------------------------


def validate_metric_value(
    value: Optional[float],
    name: str,
    allow_negative: bool = False,
) -> List[str]:
    """Validate a single metric value against the R4.2 schema.

    Returns a list of human-readable issue strings (empty = valid).
    - None is acceptable (metric unavailable).
    - NaN/Inf are rejected.
    - Negative values are rejected unless *allow_negative* is True.
    """
    issues: List[str] = []
    if value is None:
        return issues
    if not isinstance(value, (int, float)):
        issues.append(f"{name} is not a number: {type(value).__name__}")
        return issues
    if not math.isfinite(value):
        issues.append(f"{name} is non-finite: {value!r}")
        return issues
    if not allow_negative and value < 0:
        issues.append(f"{name} is negative: {value}")
    return issues
