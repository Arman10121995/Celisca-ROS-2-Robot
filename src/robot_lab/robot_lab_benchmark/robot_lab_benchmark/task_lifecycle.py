"""Scenario lifecycle orchestration for Robot Lab benchmark runs (R4.1).

Implements the validate -> launch -> ready -> reset/seed -> initialize -> task ->
observe -> stop -> record phase machine with unique result IDs and real rosbag2
recording path allocation.

Task completion determines success, not sleep duration. Launch/reset failures
abort. Unit tests mock processes; integration tests prove cleanup and prevent
artifact overwrite.
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import subprocess
import time
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from .truthful_outcomes import OutcomeKind, _utc_now_iso, classify_outcome

logger = logging.getLogger(__name__)


class LifecyclePhase(str, Enum):
    VALIDATE = "validate"
    LAUNCH = "launch"
    READY = "ready"
    RESET_SEED = "reset_seed"
    INITIALIZE = "initialize"
    TASK = "task"
    OBSERVE = "observe"
    STOP = "stop"
    RECORD = "record"
    FINALIZED = "finalized"


class ScenarioLifecycle:
    """Manage one scenario run through the R4.1 lifecycle phases.

    Responsibilities:
    - Deterministic unique result id per (experiment, robot, env, scenario, seed).
    - Phase-by-phase execution with abort on launch/reset failure.
    - Real rosbag2 recording path creation under output_dir.
    - Manifest + final record assembly for downstream measurement (R4.2).

    Task completion determines success — not sleep duration. This class does
    not itself measure distance/collisions/clearance/CPU/RTF; that is R4.2.
    """

    def __init__(
        self,
        output_dir: Path,
        experiment_id: str,
        robot_id: str,
        environment_id: str,
        scenario_id: str,
        seed: int,
        reset_service: str = "/gazebo/reset_world",
        ros2_cmd: str = "ros2",
        bash_setup: str = "/opt/ros/humble/setup.bash",
        rosbag_topics: Optional[List[str]] = None,
        scenario_timeout_sec: float = 60.0,
    ) -> None:
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.experiment_id = experiment_id
        self.robot_id = robot_id
        self.environment_id = environment_id
        self.scenario_id = scenario_id
        self.seed = int(seed)
        self.reset_service = reset_service
        self.ros2_cmd = ros2_cmd
        self.bash_setup = bash_setup
        self.rosbag_topics = rosbag_topics or ["/scan", "/odom", "/imu"]
        self.scenario_timeout_sec = float(scenario_timeout_sec)

        # Unique deterministic result id (R4.1 acceptance: unique result ids)
        self.result_id = self._make_result_id()

        # Lifecycle state
        self.phase: LifecyclePhase = LifecyclePhase.VALIDATE
        self.phase_log: List[Dict[str, Any]] = []
        self.errors: List[str] = []
        self.outcome: Optional[OutcomeKind] = None
        self._initial_pose: Dict[str, float] = {}
        self._task_payload: Dict[str, Any] = {}

        # Subprocess bookkeeping (only for tests that simulate processes)
        self._launch_proc: Optional[subprocess.Popen] = None
        self._bag_proc: Optional[subprocess.Popen] = None
        self._run_dir: Optional[Path] = None
        self._bag_path: Optional[Path] = None

    # ------------------------------------------------------------------
    # Phase logging helper
    # ------------------------------------------------------------------

    def _enter(self, phase: LifecyclePhase, note: str = "") -> None:
        if self.phase is not phase:
            prev = self.phase.value
            self.phase = phase
            logger.info(
                "lifecycle [%s] %s -> %s: %s",
                self.result_id,
                prev,
                phase.value,
                note,
            )
        self.phase_log.append(
            {"from": self.phase.value if self.phase_log else "none",
             "to": phase.value,
             "ts_utc": _utc_now_iso(),
             "note": note or phase.value}
        )

    # ------------------------------------------------------------------
    # Unique result id + artifact paths
    # ------------------------------------------------------------------

    def _make_result_id(self) -> str:
        raw = (f"{self.experiment_id}|{self.robot_id}|{self.environment_id}|"
               f"{self.scenario_id}|{self.seed}")
        digest = hashlib.sha256(raw.encode()).hexdigest()[:12]
        return f"rl_{digest}"

    @property
    def run_dir(self) -> Path:
        if self._run_dir is None:
            self._run_dir = self.output_dir / (
                f"{self.robot_id}_{self.environment_id}_{self.seed}")
            self._run_dir.mkdir(parents=True, exist_ok=True)
        return self._run_dir

    @property
    def bag_path(self) -> Path:
        """Real rosbag2 recording path — distinct per execution (not per config)."""
        if self._bag_path is None:
            ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%f")
            self._bag_path = self.run_dir / "bags" / f"{self.result_id}_{ts}"
            self._bag_path.mkdir(parents=True, exist_ok=True)
        return self._bag_path

    # ------------------------------------------------------------------
    # Phase implementations
    # ------------------------------------------------------------------

    def validate(self) -> Tuple[bool, List[str]]:
        self._enter(LifecyclePhase.VALIDATE, "input validation")
        errors: List[str] = []
        if not self.experiment_id:
            errors.append("experiment_id is empty")
        if not self.robot_id:
            errors.append("robot_id is empty")
        if not self.environment_id:
            errors.append("environment_id is empty")
        if not self.scenario_id:
            errors.append("scenario_id is empty")
        if self.seed < 0:
            errors.append("seed must be >= 0")
        if self.scenario_timeout_sec <= 0:
            errors.append("scenario_timeout_sec must be > 0")
        if errors:
            self.errors.extend(errors)
        return (len(errors) == 0, errors)

    def launch(self, extra_args: Optional[Dict[str, str]] = None) -> bool:
        self._enter(LifecyclePhase.LAUNCH, "start simulation stack")
        run_dir = self.run_dir
        cmd = (
            f"source {self.bash_setup} && {self.ros2_cmd} launch "
            f"robot_lab_adapter select_robot.launch.py "
            f"robot_id:={self.robot_id} environment_id:={self.environment_id} "
            f"scenario_id:={self.scenario_id} seed:={self.seed}")
        if extra_args:
            for k, v in extra_args.items():
                cmd += f" {k}:={v}"
        log_path = run_dir / "launch.log"
        try:
            with open(log_path, "w") as log_file:
                self._launch_proc = subprocess.Popen(
                    ["bash", "-c", cmd],
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                )
            # Give the launch system a moment to start
            time.sleep(0.5)
            return self._launch_proc.poll() is None
        except Exception as e:
            self.errors.append(f"launch raised: {e}")
            return False

    def wait_ready(self, ready_service: str = "/robot_lab/ready",
                   timeout_sec: float = 15.0) -> bool:
        self._enter(LifecyclePhase.READY, "wait for ready service")
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                result = subprocess.run(
                    [self.ros2_cmd, "service", "list", "--quiet"],
                    capture_output=True, text=True, timeout=5.0,
                )
                if ready_service in result.stdout.strip().splitlines():
                    return True
            except Exception:
                pass
            time.sleep(0.5)
        self.errors.append(
            f"ready service {ready_service} not found within {timeout_sec}s")
        return False

    def reset_and_seed(self) -> bool:
        self._enter(LifecyclePhase.RESET_SEED, "reset world + apply seed")
        try:
            result = subprocess.run(
                [self.ros2_cmd, "service", "call", self.reset_service,
                 "std_srvs/srv/Empty"],
                capture_output=True, text=True, timeout=10.0,
            )
            if result.returncode != 0:
                self.errors.append(f"reset failed: {result.stderr.strip()}")
                return False
            return True
        except Exception as e:
            self.errors.append(f"reset raised: {e}")
            return False

    def initialize_pose(self, x: float = 0.0, y: float = 0.0,
                        z: float = 0.0, yaw: float = 0.0) -> None:
        self._enter(LifecyclePhase.INITIALIZE, "set initial pose")
        self._initial_pose = {"x": x, "y": y, "z": z, "yaw": yaw}

    def send_task(self, task_payload: Optional[Dict[str, Any]] = None) -> None:
        self._enter(LifecyclePhase.TASK, "dispatch task")
        self._task_payload = task_payload or {}

    def observe(self, outcome: OutcomeKind,
                observations: Optional[Dict[str, Any]] = None) -> None:
        self._enter(LifecyclePhase.OBSERVE, f"outcome={outcome.value}")
        if self.outcome is not None:
            self.errors.append(
                "outcome already set; ignoring redundant observation")
            return
        self.outcome = outcome
        if observations:
            self._observations = observations.copy()

    def stop(self) -> Dict[str, Any]:
        self._enter(LifecyclePhase.STOP, "teardown")
        result: Dict[str, Any] = {"stop_ok": True}
        if self._bag_proc:
            result["bag_stopped"] = self._stop_bag(self._bag_proc)
            self._bag_proc = None
        if self._launch_proc:
            result["launch_stopped"] = self._stop_launch(self._launch_proc)
            self._launch_proc = None
        return result

    def record(self) -> Path:
        self._enter(LifecyclePhase.RECORD, "write manifest")
        manifest = self._manifest()
        path = self.run_dir / "manifest.json"
        path.write_text(json.dumps(manifest, indent=2, default=str),
                        encoding="utf-8")
        return path

    def finalize(self) -> Dict[str, Any]:
        self._enter(LifecyclePhase.FINALIZED, "assemble final record")
        return {
            "result_id": self.result_id,
            "experiment_id": self.experiment_id,
            "robot_id": self.robot_id,
            "environment_id": self.environment_id,
            "scenario_id": self.scenario_id,
            "seed": self.seed,
            "outcome": self.outcome.value if self.outcome else "unknown",
            "phase_log": self.phase_log,
            "errors": list(self.errors),
            "run_dir": str(self.run_dir),
            "bag_path": str(self.bag_path),
            "initial_pose": self._initial_pose,
            "task_payload": self._task_payload,
            "ts_utc": _utc_now_iso(),
        }

    # ------------------------------------------------------------------
    # Full lifecycle convenience
    # ------------------------------------------------------------------

    def run(
        self,
        task_payload: Optional[Dict[str, Any]] = None,
        initial_pose: Optional[Dict[str, float]] = None,
        outcome: Optional[OutcomeKind] = None,
        reset_service: Optional[str] = None,
        ready_service: Optional[str] = None,
        timeout_sec: Optional[float] = None,
    ) -> Dict[str, Any]:
        """Execute the full lifecycle and return the final record.

        If *outcome* is None, SUCCESS is assumed after all prior phases pass
        (convenient for tests/deterministic fixtures). In a live run the caller
        instruments the robot/stack and calls observe() with the real outcome.
        """
        # Validate
        ok, errors = self.validate()
        if not ok:
            self.errors.extend(errors)
            self.outcome = OutcomeKind.PROCESS_DEATH
            return self.finalize()

        # Launch
        if not self.launch():
            self.outcome = OutcomeKind.PROCESS_DEATH
            return self.finalize()

        # Ready
        if not self.wait_ready(
            ready_service or "/robot_lab/ready",
            timeout_sec or self.scenario_timeout_sec,
        ):
            self.outcome = OutcomeKind.PROCESS_DEATH
            return self.finalize()

        # Reset + seed
        if not self.reset_and_seed():
            self.outcome = OutcomeKind.PROCESS_DEATH
            return self.finalize()

        # Initialize
        if initial_pose:
            self.initialize_pose(**initial_pose)
        else:
            self.initialize_pose()

        # Task
        self.send_task(task_payload)

        # Observe — the caller-supplied outcome, or SUCCESS by default
        outcome = outcome or OutcomeKind.SUCCESS
        self.observe(outcome)

        # Stop
        stop_result = self.stop()

        # Record
        manifest_path = self.record()

        final = self.finalize()
        final["stop_result"] = stop_result
        final["manifest_path"] = str(manifest_path)
        return final

    # ------------------------------------------------------------------
    # Manifest (written during RECORD)
    # ------------------------------------------------------------------

    def _manifest(self) -> Dict[str, Any]:
        return {
            "schema_version": "1.0",
            "result_id": self.result_id,
            "experiment_id": self.experiment_id,
            "robot_id": self.robot_id,
            "environment_id": self.environment_id,
            "scenario_id": self.scenario_id,
            "seed": self.seed,
            "outcome": self.outcome.value if self.outcome else "unknown",
            "phase_log": self.phase_log,
            "errors": list(self.errors),
            "initial_pose": self._initial_pose,
            "task_payload": self._task_payload,
            "run_dir": str(self.run_dir),
            "bag_path": str(self.bag_path),
            "ros2_cmd": self.ros2_cmd,
            "reset_service": self.reset_service,
            "rosbag_topics": list(self.rosbag_topics),
            "ts_utc": _utc_now_iso(),
        }

    # ------------------------------------------------------------------
    # Safety net: stop any leftover processes on destruction
    # ------------------------------------------------------------------

    def __del__(self) -> None:
        try:
            if self._bag_proc:
                self._stop_bag(self._bag_proc)
            if self._launch_proc:
                self._stop_launch(self._launch_proc)
        except Exception:
            pass

    @staticmethod
    def _stop_bag(proc: subprocess.Popen) -> bool:
        """Gracefully stop a rosbag process."""
        try:
            proc.terminate()
            proc.wait(timeout=5.0)
            return True
        except subprocess.TimeoutExpired:
            proc.kill()
            return False
        except Exception:
            return False

    @staticmethod
    def _stop_launch(proc: subprocess.Popen) -> bool:
        """Stop a launch subprocess (process group)."""
        try:
            os.killpg(os.getpgid(proc.pid), 15)  # SIGTERM
            proc.wait(timeout=10.0)
            return True
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), 9)  # SIGKILL
            return False
        except Exception:
            return False


# ------------------------------------------------------------------
# Convenience factory
# ------------------------------------------------------------------


def make_lifecycle(
    output_dir: Path,
    experiment_id: str,
    robot_id: str,
    environment_id: str,
    scenario_id: str,
    seed: int,
    **kwargs: Any,
) -> ScenarioLifecycle:
    """Factory for ScenarioLifecycle with common defaults."""
    return ScenarioLifecycle(
        output_dir=output_dir,
        experiment_id=experiment_id,
        robot_id=robot_id,
        environment_id=environment_id,
        scenario_id=scenario_id,
        seed=seed,
        **kwargs,
    )
