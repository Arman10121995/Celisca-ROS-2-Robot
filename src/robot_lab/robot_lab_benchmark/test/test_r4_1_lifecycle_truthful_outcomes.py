"""R4.1 tests: scenario lifecycle and truthful outcomes.

Covers the R4.1 acceptance bar:
- All 9 OutcomeKind values classify correctly via classify_outcome, in priority
  order (launch/reset abort first, then process death, cancellation, timeout,
  collision, lost state, no path, success).
- Unique result IDs are deterministic per config and distinct across configs.
- Lifecycle phase transitions are logged in order.
- Launch/reset failure abort the run before the task (run() short-circuits).
- Manifest is written on record() with the expected schema fields.
- The rosbag2 path is distinct per execution (timestamped), not per config.
- observe() records the outcome exactly once and rejects redundant observation.
- outcome taxonomy helpers partition the 9 kinds correctly.

These tests mock no real ROS stack: validate(), classify_outcome(),
build_result_id(), and the pure record()/path logic are exercised directly.
launch() is exercised only to assert it *fails closed* (returns False) in an
environment with no running ROS graph, which is the correct abort behaviour.
"""

from __future__ import annotations

import json
import sys
import tempfile
import time
import unittest
from pathlib import Path

# Ensure robot_lab_benchmark (sibling package) is importable during tests,
# mirroring the injection used by test_p6_benchmarking.py.
_benchmark_pkg = Path(__file__).resolve().parents[3] / "robot_lab" / "robot_lab_benchmark"
if str(_benchmark_pkg) not in sys.path:
    sys.path.insert(0, str(_benchmark_pkg))

from robot_lab_benchmark.task_lifecycle import (
    LifecyclePhase,
    ScenarioLifecycle,
    make_lifecycle,
)
from robot_lab_benchmark.truthful_outcomes import (
    ABORT_OUTCOMES,
    TERMINAL_OUTCOMES,
    OutcomeKind,
    all_outcome_kinds,
    abort_outcome_kinds,
    build_result_id,
    classify_outcome,
    outcome_is_abort,
    outcome_is_terminal,
    outcome_success,
    outcome_to_dict,
    terminal_outcome_kinds,
    write_outcome_record,
)


def _lc(output_dir, seed=42, scenario_id="dyn_nav", bash_setup="/opt/ros/humble/setup.bash"):
    """Helper to build a ScenarioLifecycle with test defaults."""
    return ScenarioLifecycle(
        output_dir=output_dir,
        experiment_id="exp1",
        robot_id="bumperbot",
        environment_id="warehouse",
        scenario_id=scenario_id,
        seed=seed,
        bash_setup=bash_setup,
    )


# ------------------------------------------------------------------
# Outcome taxonomy
# ------------------------------------------------------------------

class OutcomeTaxonomyTests(unittest.TestCase):
    """The 9 OutcomeKind values exist and partition correctly."""

    def test_nine_outcome_kinds_defined(self):
        kinds = all_outcome_kinds()
        self.assertEqual(len(kinds), 9)
        expected = {
            "success", "collision", "timeout", "no_path", "lost_state",
            "process_death", "cancellation", "launch_failure", "reset_failure",
        }
        self.assertEqual({k.value for k in kinds}, expected)

    def test_terminal_outcomes_are_seven(self):
        terminal = terminal_outcome_kinds()
        self.assertEqual(len(terminal), 7)
        self.assertEqual(set(terminal), set(TERMINAL_OUTCOMES))

    def test_abort_outcomes_are_two(self):
        abort = abort_outcome_kinds()
        self.assertEqual(len(abort), 2)
        self.assertEqual(set(abort), set(ABORT_OUTCOMES))

    def test_terminal_and_abort_are_disjoint(self):
        self.assertEqual(set(TERMINAL_OUTCOMES) & set(ABORT_OUTCOMES), set())

    def test_terminal_and_abort_cover_all_kinds(self):
        covered = set(TERMINAL_OUTCOMES) | set(ABORT_OUTCOMES)
        self.assertEqual(covered, set(all_outcome_kinds()))


# ------------------------------------------------------------------
# classify_outcome: priority ordering
# ------------------------------------------------------------------

class ClassifyOutcomeTests(unittest.TestCase):
    """classify_outcome maps every signal to the right OutcomeKind."""

    def test_success_is_default(self):
        self.assertEqual(classify_outcome(), OutcomeKind.SUCCESS)

    def test_launch_failure_wins_over_everything(self):
        self.assertEqual(
            classify_outcome(
                launch_ok=False,
                reset_ok=False,
                process_alive=False,
                cancelled=True,
                timed_out=True,
                collision_detected=True,
                state_healthy=False,
                path_available=False,
            ),
            OutcomeKind.LAUNCH_FAILURE,
        )

    def test_reset_failure_wins_after_launch_ok(self):
        self.assertEqual(
            classify_outcome(
                launch_ok=True,
                reset_ok=False,
                process_alive=False,
                cancelled=True,
                timed_out=True,
                collision_detected=True,
            ),
            OutcomeKind.RESET_FAILURE,
        )

    def test_process_death(self):
        self.assertEqual(
            classify_outcome(launch_ok=True, reset_ok=True, process_alive=False),
            OutcomeKind.PROCESS_DEATH,
        )

    def test_cancellation(self):
        self.assertEqual(
            classify_outcome(cancelled=True), OutcomeKind.CANCELLATION)

    def test_timeout(self):
        self.assertEqual(classify_outcome(timed_out=True), OutcomeKind.TIMEOUT)

    def test_collision(self):
        self.assertEqual(
            classify_outcome(collision_detected=True), OutcomeKind.COLLISION)

    def test_lost_state(self):
        self.assertEqual(
            classify_outcome(state_healthy=False), OutcomeKind.LOST_STATE)

    def test_no_path(self):
        self.assertEqual(
            classify_outcome(path_available=False), OutcomeKind.NO_PATH)

    def test_each_kind_reachable(self):
        """Every OutcomeKind is the output of some classify_outcome call."""
        cases = {
            OutcomeKind.SUCCESS: {},
            OutcomeKind.LAUNCH_FAILURE: {"launch_ok": False},
            OutcomeKind.RESET_FAILURE: {"reset_ok": False},
            OutcomeKind.PROCESS_DEATH: {"process_alive": False},
            OutcomeKind.CANCELLATION: {"cancelled": True},
            OutcomeKind.TIMEOUT: {"timed_out": True},
            OutcomeKind.COLLISION: {"collision_detected": True},
            OutcomeKind.LOST_STATE: {"state_healthy": False},
            OutcomeKind.NO_PATH: {"path_available": False},
        }
        for kind, kwargs in cases.items():
            self.assertEqual(classify_outcome(**kwargs), kind,
                             f"{kind} unreachable")


# ------------------------------------------------------------------
# Outcome taxonomy helpers
# ------------------------------------------------------------------

class OutcomeHelperTests(unittest.TestCase):
    """outcome_is_terminal / outcome_is_abort / outcome_success partition."""

    def test_outcome_is_terminal(self):
        for kind in TERMINAL_OUTCOMES:
            self.assertTrue(outcome_is_terminal(kind),
                            f"{kind} should be terminal")
        for kind in ABORT_OUTCOMES:
            self.assertFalse(outcome_is_terminal(kind),
                             f"{kind} is abort, not terminal")

    def test_outcome_is_abort(self):
        for kind in ABORT_OUTCOMES:
            self.assertTrue(outcome_is_abort(kind),
                            f"{kind} should be abort")
        for kind in TERMINAL_OUTCOMES:
            self.assertFalse(outcome_is_abort(kind),
                             f"{kind} is terminal, not abort")

    def test_outcome_success_is_only_success(self):
        self.assertTrue(outcome_success(OutcomeKind.SUCCESS))
        for kind in set(all_outcome_kinds()) - {OutcomeKind.SUCCESS}:
            self.assertFalse(outcome_success(kind),
                             f"{kind} must not count as success")


# ------------------------------------------------------------------
# Result ID: deterministic + unique
# ------------------------------------------------------------------

class ResultIdTests(unittest.TestCase):
    """build_result_id deterministic per config, distinct across."""

    def test_deterministic(self):
        a = build_result_id("exp1", "bumperbot", "warehouse", "dyn_nav", 42)
        b = build_result_id("exp1", "bumperbot", "warehouse", "dyn_nav", 42)
        self.assertEqual(a, b)

    def test_distinct_across_seeds(self):
        id0 = build_result_id("exp1", "bumperbot", "warehouse", "dyn_nav", 0)
        id1 = build_result_id("exp1", "bumperbot", "warehouse", "dyn_nav", 1)
        self.assertNotEqual(id0, id1)

    def test_distinct_across_robots(self):
        id_a = build_result_id("exp1", "bumperbot", "warehouse", "dyn_nav", 42)
        id_b = build_result_id("exp1", "go2", "warehouse", "dyn_nav", 42)
        self.assertNotEqual(id_a, id_b)

    def test_distinct_across_scenarios(self):
        id_a = build_result_id("exp1", "bumperbot", "warehouse", "dyn_nav", 42)
        id_b = build_result_id("exp1", "bumperbot", "warehouse", "coverage", 42)
        self.assertNotEqual(id_a, id_b)

    def test_format(self):
        rid = build_result_id("exp1", "bumperbot", "warehouse", "dyn_nav", 42)
        self.assertTrue(rid.startswith("rl_"))
        self.assertEqual(len(rid), 15)  # "rl_" + 12 hex chars


# ------------------------------------------------------------------
# Lifecycle: validation
# ------------------------------------------------------------------

class LifecycleValidationTests(unittest.TestCase):
    """validate() catches bad inputs and passes good ones."""

    def test_valid_inputs_pass(self):
        with tempfile.TemporaryDirectory() as d:
            ok, errors = _lc(d).validate()
            self.assertTrue(ok)
            self.assertEqual(errors, [])

    def test_empty_experiment_id_fails(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            lc.experiment_id = ""
            ok, errors = lc.validate()
            self.assertFalse(ok)
            self.assertIn("experiment_id is empty", errors)

    def test_empty_robot_id_fails(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            lc.robot_id = ""
            ok, errors = lc.validate()
            self.assertFalse(ok)
            self.assertIn("robot_id is empty", errors)

    def test_empty_environment_id_fails(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            lc.environment_id = ""
            ok, errors = lc.validate()
            self.assertFalse(ok)
            self.assertIn("environment_id is empty", errors)

    def test_empty_scenario_id_fails(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            lc.scenario_id = ""
            ok, errors = lc.validate()
            self.assertFalse(ok)
            self.assertIn("scenario_id is empty", errors)

    def test_negative_seed_fails(self):
        with tempfile.TemporaryDirectory() as d:
            ok, errors = _lc(d, seed=-1).validate()
            self.assertFalse(ok)
            self.assertIn("seed must be >= 0", errors)

    def test_zero_timeout_fails(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            lc.scenario_timeout_sec = 0.0
            ok, errors = lc.validate()
            self.assertFalse(ok)
            self.assertIn("scenario_timeout_sec must be > 0", errors)

    def test_multiple_errors_reported_together(self):
        with tempfile.TemporaryDirectory() as d:
            lc = ScenarioLifecycle(
                output_dir=Path(d), experiment_id="", robot_id="",
                environment_id="warehouse", scenario_id="dyn_nav", seed=-5,
                scenario_timeout_sec=-1.0)
            ok, errors = lc.validate()
            self.assertFalse(ok)
            self.assertGreaterEqual(len(errors), 4)


# ------------------------------------------------------------------
# Lifecycle: phase transitions are logged
# ------------------------------------------------------------------

class LifecyclePhaseLogTests(unittest.TestCase):
    """Phase transitions are recorded in phase_log in order."""

    def test_validate_logs_transition(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            lc.validate()
            self.assertEqual(len(lc.phase_log), 1)
            self.assertEqual(lc.phase_log[0]["to"],
                             LifecyclePhase.VALIDATE.value)

    def test_initialize_pose_logs_transition(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            lc.initialize_pose(x=1.0, y=2.0, z=0.0, yaw=0.5)
            self.assertEqual(lc.phase, LifecyclePhase.INITIALIZE)
            self.assertEqual(lc._initial_pose,
                             {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5})

    def test_send_task_logs_transition(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            payload = {"waypoints": [(0, 0), (1, 1)]}
            lc.send_task(payload)
            self.assertEqual(lc.phase, LifecyclePhase.TASK)
            self.assertEqual(lc._task_payload, payload)

    def test_observe_sets_outcome_once(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            lc.observe(OutcomeKind.SUCCESS)
            self.assertEqual(lc.outcome, OutcomeKind.SUCCESS)
            # Redundant observation is rejected, not overwritten.
            lc.observe(OutcomeKind.COLLISION)
            self.assertEqual(lc.outcome, OutcomeKind.SUCCESS)
            self.assertIn("outcome already set", lc.errors[-1])


# ------------------------------------------------------------------
# Lifecycle: run_dir and bag_path
# ------------------------------------------------------------------

class LifecyclePathTests(unittest.TestCase):
    """run_dir is per-config; bag_path is distinct per execution."""

    def test_run_dir_is_per_config(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            expected = Path(d) / "bumperbot_warehouse_42"
            self.assertEqual(lc.run_dir, expected)
            self.assertTrue(expected.exists())

    def test_run_dir_same_for_same_config(self):
        with tempfile.TemporaryDirectory() as d:
            lc1 = _lc(d)
            lc2 = _lc(d)
            self.assertEqual(lc1.run_dir, lc2.run_dir)

    def test_bag_path_is_distinct_per_execution(self):
        """Two lifecycles with the SAME config get DIFFERENT bag
        paths because the path is timestamped (per execution)."""
        with tempfile.TemporaryDirectory() as d:
            lc1 = _lc(d)
            bp1 = lc1.bag_path
            time.sleep(0.01)
            lc2 = _lc(d)
            bp2 = lc2.bag_path
            self.assertNotEqual(bp1, bp2)

    def test_bag_path_contains_result_id(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            bp = lc.bag_path
            self.assertIn(lc.result_id, bp.name)
            self.assertEqual(bp.parent.name, "bags")


# ------------------------------------------------------------------
# Lifecycle: record() writes the manifest
# ------------------------------------------------------------------

class LifecycleRecordTests(unittest.TestCase):
    """record() writes manifest.json with the expected schema."""

    def test_record_writes_manifest(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            lc.observe(OutcomeKind.SUCCESS)
            path = lc.record()
            self.assertTrue(path.exists())
            self.assertEqual(path.name, "manifest.json")

    def test_manifest_has_required_fields(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            lc.initialize_pose(x=1.0, y=0.0, z=0.0, yaw=0.0)
            lc.send_task({"goal": [5, 5]})
            lc.observe(OutcomeKind.SUCCESS)
            manifest = json.loads(lc.record().read_text(encoding="utf-8"))
            for key in ("schema_version", "result_id", "experiment_id",
                        "robot_id", "environment_id", "scenario_id", "seed",
                        "outcome", "phase_log", "errors", "initial_pose",
                        "task_payload", "run_dir", "bag_path", "ros2_cmd",
                        "reset_service", "rosbag_topics", "ts_utc"):
                self.assertIn(key, manifest, f"manifest missing {key}")
            self.assertEqual(manifest["schema_version"], "1.0")
            self.assertEqual(manifest["outcome"], "success")
            self.assertEqual(manifest["seed"], 42)
            self.assertEqual(manifest["initial_pose"],
                             {"x": 1.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
            self.assertEqual(manifest["task_payload"], {"goal": [5, 5]})

    def test_manifest_outcome_unknown_when_not_observed(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            manifest = json.loads(lc.record().read_text(encoding="utf-8"))
            self.assertEqual(manifest["outcome"], "unknown")


# ------------------------------------------------------------------
# Lifecycle: launch fails closed (no ROS graph in test env)
# ------------------------------------------------------------------

class LifecycleLaunchAbortTests(unittest.TestCase):
    """launch() fails closed when the simulation stack cannot start, and
    run() short-circuits (aborts before the task).

    We force launch() to fail deterministically by pointing bash_setup at a
    non-existent file: the `source <bash_setup>` step fails, the spawned
    shell exits non-zero, and launch() returns False. This avoids depending
    on whether a ROS graph happens to be running in the test environment.
    """

    def test_launch_fails_closed_when_setup_missing(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d, bash_setup="/nonexistent/setup.bash")
            # source <bash_setup> fails -> launch returns False.
            self.assertFalse(lc.launch())

    def test_run_aborts_on_launch_failure(self):
        """When launch() fails, run() must NOT dispatch a task; it aborts
        and records PROCESS_DEATH as the outcome."""
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d, bash_setup="/nonexistent/setup.bash")
            final = lc.run(task_payload={"goal": [5, 5]})
            # Outcome should be the abort sentinel, never SUCCESS.
            self.assertEqual(final["outcome"], "process_death")
            # Task payload must NOT have been recorded because we aborted
            # before send_task().
            self.assertEqual(final["task_payload"], {})


# ------------------------------------------------------------------
# Lifecycle: factory + finalize
# ------------------------------------------------------------------

class LifecycleFactoryFinalizeTests(unittest.TestCase):
    """make_lifecycle() factory and finalize() output."""

    def test_factory_matches_constructor(self):
        with tempfile.TemporaryDirectory() as d:
            lc = make_lifecycle(
                output_dir=Path(d), experiment_id="exp1", robot_id="bumperbot",
                environment_id="warehouse", scenario_id="dyn_nav", seed=42)
            self.assertIsInstance(lc, ScenarioLifecycle)
            self.assertEqual(lc.seed, 42)

    def test_finalize_assembles_record(self):
        with tempfile.TemporaryDirectory() as d:
            lc = _lc(d)
            lc.observe(OutcomeKind.COLLISION)
            rec = lc.finalize()
            self.assertEqual(rec["result_id"], lc.result_id)
            self.assertEqual(rec["outcome"], "collision")
            self.assertIn("ts_utc", rec)


# ------------------------------------------------------------------
# Outcome record serialization
# ------------------------------------------------------------------

class OutcomeRecordTests(unittest.TestCase):
    """outcome_to_dict() and write_outcome_record()."""

    def test_outcome_to_dict(self):
        rec = outcome_to_dict(OutcomeKind.TIMEOUT, "rl_abc123")
        self.assertEqual(rec["outcome"], "timeout")
        self.assertEqual(rec["result_id"], "rl_abc123")
        self.assertIn("ts_utc", rec)

    def test_write_outcome_record(self):
        with tempfile.TemporaryDirectory() as d:
            out_path = write_outcome_record(
                OutcomeKind.COLLISION, Path(d), "rl_test123")
            self.assertTrue(out_path.exists())
            self.assertEqual(out_path.parent.name, "outcomes")
            self.assertEqual(out_path.name, "rl_test123.json")
            payload = json.loads(out_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["outcome"], "collision")
            self.assertEqual(payload["result_id"], "rl_test123")


if __name__ == "__main__":
    unittest.main()
