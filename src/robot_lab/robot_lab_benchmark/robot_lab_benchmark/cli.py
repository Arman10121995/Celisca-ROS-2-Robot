#!/usr/bin/env python3
"""Command-line entry point for Robot Lab benchmark results."""

import argparse
import json
from pathlib import Path

from . import BenchmarkResult


def main(argv=None):
    parser = argparse.ArgumentParser(description='Emit a canonical benchmark result record.')
    parser.add_argument('--experiment-id', required=True)
    parser.add_argument('--robot-id', required=True)
    parser.add_argument('--environment-id', required=True)
    parser.add_argument('--scenario-id', required=True)
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--success', action='store_true')
    parser.add_argument('--elapsed-seconds', type=float, default=0.0)
    parser.add_argument('--path-length-m', type=float, default=0.0)
    parser.add_argument('--collision-count', type=int, default=0)
    parser.add_argument('--min-clearance-m', type=float, default=0.0)
    parser.add_argument('--revision', default='unknown')
    parser.add_argument('--output', default=None, help='Optional output JSON file path')
    args = parser.parse_args(argv)

    result = BenchmarkResult(
        experiment_id=args.experiment_id,
        robot_id=args.robot_id,
        environment_id=args.environment_id,
        scenario_id=args.scenario_id,
        seed=args.seed,
        success=args.success,
        elapsed_seconds=args.elapsed_seconds,
        path_length_m=args.path_length_m,
        collision_count=args.collision_count,
        min_clearance_m=args.min_clearance_m,
        revision=args.revision,
    )

    payload = result.to_dict()
    if args.output:
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2), encoding='utf-8')
    else:
        print(json.dumps(payload, indent=2))

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
