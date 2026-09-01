from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, Optional


class BenchmarkRunner:
    """Create a seeded benchmark run manifest and optional rosbag recording path."""

    def __init__(self, output_dir: Optional[str] = None):
        self.output_dir = Path(output_dir) if output_dir else Path.cwd()
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def run(
        self,
        experiment_id: str,
        robot_id: str,
        environment_id: str,
        scenario_id: str,
        seed: int,
        reset_service: str = '/gazebo/reset_world',
        bag_capture: bool = False,
        **kwargs: Any,
    ) -> Dict[str, Any]:
        run_dir = self.output_dir / f"{experiment_id}_{seed}"
        run_dir.mkdir(parents=True, exist_ok=True)

        bag_name = f"{experiment_id}_{seed}.bag"
        bag_path = run_dir / bag_name
        if bag_capture:
            bag_path.touch()

        manifest = {
            'schema_version': '1.0',
            'experiment_id': experiment_id,
            'robot_id': robot_id,
            'environment_id': environment_id,
            'scenario_id': scenario_id,
            'seed': int(seed),
            'reset_service': reset_service,
            'bag_capture': bool(bag_capture),
            'bag_path': str(bag_path),
            'manifest_path': str(run_dir / 'manifest.json'),
            'output_dir': str(run_dir),
            'kwargs': kwargs,
        }

        manifest_path = Path(manifest['manifest_path'])
        manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf-8')

        return manifest
