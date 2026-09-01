from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional


class ReferenceBenchmark:
    """Store and manage reference benchmark baselines."""

    schema_version = '1.0'

    def __init__(
        self,
        experiment_id: str,
        robot_id: str,
        environment_id: str,
        scenario_id: str,
        baseline_elapsed_seconds: float,
        baseline_path_length_m: float,
        baseline_collision_count: int,
        baseline_min_clearance_m: float,
        threshold_elapsed_seconds: float = 1.5,
        threshold_path_length_m: float = 1.3,
        threshold_collision_count: int = 2,
        threshold_min_clearance_m: float = 0.7,
        notes: str = '',
        revision: str = 'unknown',
    ) -> None:
        self.experiment_id = experiment_id
        self.robot_id = robot_id
        self.environment_id = environment_id
        self.scenario_id = scenario_id
        self.baseline_elapsed_seconds = float(baseline_elapsed_seconds)
        self.baseline_path_length_m = float(baseline_path_length_m)
        self.baseline_collision_count = int(baseline_collision_count)
        self.baseline_min_clearance_m = float(baseline_min_clearance_m)
        self.threshold_elapsed_seconds = float(threshold_elapsed_seconds)
        self.threshold_path_length_m = float(threshold_path_length_m)
        self.threshold_collision_count = int(threshold_collision_count)
        self.threshold_min_clearance_m = float(threshold_min_clearance_m)
        self.notes = notes
        self.revision = revision

    def to_dict(self) -> Dict[str, Any]:
        """Serialize to dict."""
        return {
            'schema_version': self.schema_version,
            'experiment_id': self.experiment_id,
            'robot_id': self.robot_id,
            'environment_id': self.environment_id,
            'scenario_id': self.scenario_id,
            'baseline_elapsed_seconds': self.baseline_elapsed_seconds,
            'baseline_path_length_m': self.baseline_path_length_m,
            'baseline_collision_count': self.baseline_collision_count,
            'baseline_min_clearance_m': self.baseline_min_clearance_m,
            'threshold_elapsed_seconds': self.threshold_elapsed_seconds,
            'threshold_path_length_m': self.threshold_path_length_m,
            'threshold_collision_count': self.threshold_collision_count,
            'threshold_min_clearance_m': self.threshold_min_clearance_m,
            'notes': self.notes,
            'revision': self.revision,
        }

    def check_regression(self, result: Any) -> Dict[str, Any]:
        """Check if a result regresses against baseline."""
        if isinstance(result, dict):
            payload = result
        else:
            payload = result.to_dict() if hasattr(result, 'to_dict') else vars(result)

        elapsed = float(payload.get('elapsed_seconds', 0.0))
        path_length = float(payload.get('path_length_m', 0.0))
        collisions = int(payload.get('collision_count', 0))
        clearance = float(payload.get('min_clearance_m', 0.0))

        regressions = []

        if elapsed > self.baseline_elapsed_seconds * self.threshold_elapsed_seconds:
            regressions.append({
                'metric': 'elapsed_seconds',
                'expected': self.baseline_elapsed_seconds,
                'threshold_multiplier': self.threshold_elapsed_seconds,
                'actual': elapsed,
                'regressed': True,
            })

        if path_length > self.baseline_path_length_m * self.threshold_path_length_m:
            regressions.append({
                'metric': 'path_length_m',
                'expected': self.baseline_path_length_m,
                'threshold_multiplier': self.threshold_path_length_m,
                'actual': path_length,
                'regressed': True,
            })

        if collisions > self.baseline_collision_count + self.threshold_collision_count:
            regressions.append({
                'metric': 'collision_count',
                'expected': self.baseline_collision_count,
                'threshold_delta': self.threshold_collision_count,
                'actual': collisions,
                'regressed': True,
            })

        if clearance < self.baseline_min_clearance_m * self.threshold_min_clearance_m:
            regressions.append({
                'metric': 'min_clearance_m',
                'expected': self.baseline_min_clearance_m,
                'threshold_multiplier': self.threshold_min_clearance_m,
                'actual': clearance,
                'regressed': True,
            })

        return {
            'baseline_id': f"{self.experiment_id}_{self.robot_id}_{self.environment_id}",
            'passed': len(regressions) == 0,
            'regressions': regressions,
        }


class ReferenceRegistry:
    """Manage a registry of reference benchmarks."""

    def __init__(self, registry_path: Optional[Path] = None) -> None:
        self.registry_path = registry_path or Path.cwd() / 'benchmark_references.json'
        self.references: Dict[str, ReferenceBenchmark] = {}
        self.load()

    def load(self) -> bool:
        """Load registry from file."""
        if not self.registry_path.exists():
            return False

        try:
            with open(self.registry_path, 'r', encoding='utf-8') as f:
                data = json.load(f)

            for key, entry in data.get('references', {}).items():
                ref = ReferenceBenchmark(
                    experiment_id=entry['experiment_id'],
                    robot_id=entry['robot_id'],
                    environment_id=entry['environment_id'],
                    scenario_id=entry['scenario_id'],
                    baseline_elapsed_seconds=entry['baseline_elapsed_seconds'],
                    baseline_path_length_m=entry['baseline_path_length_m'],
                    baseline_collision_count=entry['baseline_collision_count'],
                    baseline_min_clearance_m=entry['baseline_min_clearance_m'],
                    threshold_elapsed_seconds=entry.get('threshold_elapsed_seconds', 1.5),
                    threshold_path_length_m=entry.get('threshold_path_length_m', 1.3),
                    threshold_collision_count=entry.get('threshold_collision_count', 2),
                    threshold_min_clearance_m=entry.get('threshold_min_clearance_m', 0.7),
                    notes=entry.get('notes', ''),
                    revision=entry.get('revision', 'unknown'),
                )
                self.references[key] = ref

            return True
        except Exception as e:
            print(f"Failed to load reference registry: {e}")
            return False

    def save(self) -> bool:
        """Save registry to file."""
        try:
            payload = {
                'schema_version': '1.0',
                'references': {key: ref.to_dict() for key, ref in self.references.items()},
            }

            self.registry_path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
            return True
        except Exception as e:
            print(f"Failed to save reference registry: {e}")
            return False

    def add_reference(
        self,
        experiment_id: str,
        robot_id: str,
        environment_id: str,
        scenario_id: str,
        baseline_elapsed_seconds: float,
        baseline_path_length_m: float,
        baseline_collision_count: int,
        baseline_min_clearance_m: float,
    ) -> str:
        """Add a new reference baseline."""
        ref = ReferenceBenchmark(
            experiment_id=experiment_id,
            robot_id=robot_id,
            environment_id=environment_id,
            scenario_id=scenario_id,
            baseline_elapsed_seconds=baseline_elapsed_seconds,
            baseline_path_length_m=baseline_path_length_m,
            baseline_collision_count=baseline_collision_count,
            baseline_min_clearance_m=baseline_min_clearance_m,
        )

        key = f"{experiment_id}_{robot_id}_{environment_id}_{scenario_id}"
        self.references[key] = ref
        return key

    def get_reference(
        self,
        experiment_id: str,
        robot_id: str,
        environment_id: str,
        scenario_id: str,
    ) -> Optional[ReferenceBenchmark]:
        """Get a reference baseline by ID."""
        key = f"{experiment_id}_{robot_id}_{environment_id}_{scenario_id}"
        return self.references.get(key)

    def check_all_regressions(self, results: list[Any]) -> Dict[str, Any]:
        """Check all results against references."""
        regression_report = {
            'total_checks': len(results),
            'passed': 0,
            'failed': 0,
            'checks': [],
        }

        for result in results:
            payload = result.to_dict() if hasattr(result, 'to_dict') else result
            exp_id = payload.get('experiment_id')
            robot_id = payload.get('robot_id')
            env_id = payload.get('environment_id')
            scenario_id = payload.get('scenario_id')

            ref = self.get_reference(exp_id, robot_id, env_id, scenario_id)
            if ref:
                check = ref.check_regression(result)
                regression_report['checks'].append(check)

                if check['passed']:
                    regression_report['passed'] += 1
                else:
                    regression_report['failed'] += 1

        return regression_report
