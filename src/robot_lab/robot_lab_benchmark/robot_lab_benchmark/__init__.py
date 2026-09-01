from __future__ import annotations

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


__all__ = [
    'BenchmarkResult',
]
