"""Truthful outcome classification for Robot Lab benchmark runs (R4.1).

Defines the terminal outcome taxonomy required by the R4.1 acceptance bar:
- success, collision, timeout, no_path, lost_state, process_death,
  cancellation have distinct terminal records
- launch/reset failures abort the run and are recorded distinctly
- unique result IDs and real rosbag2 recording paths are attached

Outcome classification is the only concern of this module. Metric extraction
(distance, contacts, clearance, CPU/RTF, ground-truth vs estimation error)
is R4.2.
"""

from __future__ import annotations

import hashlib
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


class OutcomeKind(str, Enum):
    """Distinct terminal outcomes for a scenario run (R4.1 acceptance bar).

    Each run produces exactly one of these; they are NOT collapsed into a
    boolean success/failure. This is what makes the outcome "truthful".
    """

    SUCCESS = "success"
    COLLISION = "collision"
    TIMEOUT = "timeout"
    NO_PATH = "no_path"
    LOST_STATE = "lost_state"
    PROCESS_DEATH = "process_death"
    CANCELLATION = "cancellation"
    LAUNCH_FAILURE = "launch_failure"
    RESET_FAILURE = "reset_failure"


# Outcomes that terminate the run with a meaningful result.
TERMINAL_OUTCOMES = frozenset([
    OutcomeKind.SUCCESS,
    OutcomeKind.COLLISION,
    OutcomeKind.TIMEOUT,
    OutcomeKind.NO_PATH,
    OutcomeKind.LOST_STATE,
    OutcomeKind.PROCESS_DEATH,
    OutcomeKind.CANCELLATION,
])


# Outcomes that represent an abort before the task even started.
ABORT_OUTCOMES = frozenset([
    OutcomeKind.LAUNCH_FAILURE,
    OutcomeKind.RESET_FAILURE,
])


def classify_outcome(
    collision_detected: bool = False,
    timed_out: bool = False,
    path_available: bool = True,
    state_healthy: bool = True,
    process_alive: bool = True,
    cancelled: bool = False,
    launch_ok: bool = True,
    reset_ok: bool = True,
) -> OutcomeKind:
    """Classify a terminal outcome from the signals a live stack would expose.

    Priority order matches the acceptance bar:
    1. Launch/reset failures abort first (LAUNCH_FAILURE, RESET_FAILURE)
    2. Process death (PROCESS_DEATH)
    3. Cancellation (CANCELLATION)
    4. Timeout (TIMEOUT)
    5. Collision (COLLISION)
    6. Lost state (LOST_STATE)
    7. No path (NO_PATH)
    8. Success (SUCCESS)

    Parameters
    ----------
    collision_detected : bool
        Whether a contact/collision event was observed.
    timed_out : bool
        Whether the scenario exceeded its time budget.
    path_available : bool
        Whether the planner could produce a path.
    state_healthy : bool
        Whether the robot state estimator/controller remained healthy.
    process_alive : bool
        Whether the simulation/robot process is still alive.
    cancelled : bool
        Whether the run was explicitly cancelled.
    launch_ok : bool
        Whether the simulation stack launched successfully.
    reset_ok : bool
        Whether the reset service call succeeded.

    Returns
    -------
    OutcomeKind
        The terminal outcome for this run.
    """
    if not launch_ok:
        return OutcomeKind.LAUNCH_FAILURE
    if not reset_ok:
        return OutcomeKind.RESET_FAILURE
    if not process_alive:
        return OutcomeKind.PROCESS_DEATH
    if cancelled:
        return OutcomeKind.CANCELLATION
    if timed_out:
        return OutcomeKind.TIMEOUT
    if collision_detected:
        return OutcomeKind.COLLISION
    if not state_healthy:
        return OutcomeKind.LOST_STATE
    if not path_available:
        return OutcomeKind.NO_PATH
    return OutcomeKind.SUCCESS


def outcome_to_dict(outcome: OutcomeKind, result_id: str) -> Dict[str, Any]:
    """Serialize an outcome into its terminal record.

    This is the "truthful outcome" record that gets stored alongside the
    run manifest and (in R4.2) the metrics.
    """
    return {
        "result_id": result_id,
        "outcome": outcome.value,
        "ts_utc": _utc_now_iso(),
    }


def write_outcome_record(
    outcome: OutcomeKind,
    output_dir: Path,
    result_id: str,
) -> Path:
    """Write the outcome record to disk as a JSON file.

    The outcome record is written under output_dir/outcomes/<result_id>.json
    so it is clearly separated from the run manifest and the rosbag.
    """
    outcome_dir = output_dir / "outcomes"
    outcome_dir.mkdir(parents=True, exist_ok=True)
    record_path = outcome_dir / f"{result_id}.json"
    record = outcome_to_dict(outcome, result_id)
    record_path.write_text(json.dumps(record, indent=2),
                           encoding="utf-8")
    return record_path


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


# ------------------------------------------------------------------
# Outcome taxonomy helpers for tests
# ------------------------------------------------------------------


def outcome_is_terminal(kind: OutcomeKind) -> bool:
    """Return True if *kind* is a terminal outcome (not an intermediate state)."""
    return kind in TERMINAL_OUTCOMES


def outcome_is_abort(kind: OutcomeKind) -> bool:
    """Return True if *kind* represents an abort before task start."""
    return kind in ABORT_OUTCOMES


def outcome_success(kind: OutcomeKind) -> bool:
    """Return True only for SUCCESS (not for other non-abort outcomes)."""
    return kind == OutcomeKind.SUCCESS


def build_result_id(
    experiment_id: str,
    robot_id: str,
    environment_id: str,
    scenario_id: str,
    seed: int,
) -> str:
    """Build a deterministic unique result id for a run configuration.

    This is the same id scheme used by ScenarioLifecycle so that the outcome
    record and the run manifest share the same result_id.
    """
    raw = (f"{experiment_id}|{robot_id}|{environment_id}|"
           f"{scenario_id}|{seed}")
    digest = hashlib.sha256(raw.encode()).hexdigest()[:12]
    return f"rl_{digest}"


# ------------------------------------------------------------------
# Outcome taxonomy unit-test fixtures (used by test_task_lifecycle.py)
# ------------------------------------------------------------------


def all_outcome_kinds() -> List[OutcomeKind]:
    """Return all defined outcome kinds (for test parameterization)."""
    return list(OutcomeKind)


def terminal_outcome_kinds() -> List[OutcomeKind]:
    """Return all terminal outcome kinds (for test parameterization)."""
    return sorted(TERMINAL_OUTCOMES, key=lambda k: k.value)


def abort_outcome_kinds() -> List[OutcomeKind]:
    """Return all abort outcome kinds (for test parameterization)."""
    return sorted(ABORT_OUTCOMES, key=lambda k: k.value)
