import json
import os
import tempfile
import unittest

from robot_lab_benchmark import BenchmarkResult
from robot_lab_benchmark.aggregator import aggregate_results, compare_results
from robot_lab_benchmark.cli import main
from robot_lab_benchmark.groundtruth import GroundTruthAdapter
from robot_lab_benchmark.normalizer import MetricNormalizer
from robot_lab_benchmark.orchestrator import BenchmarkRunner
from robot_lab_benchmark.outputs import OutputGenerator
from robot_lab_benchmark.reference import ReferenceBenchmark, ReferenceRegistry
from robot_lab_benchmark.report import generate_report


class BenchmarkingTests(unittest.TestCase):
    def test_result_schema_has_required_fields(self):
        result = BenchmarkResult(
            experiment_id='bumperbot_smoke_test',
            robot_id='bumperbot',
            environment_id='small_office',
            scenario_id='bumperbot_smoke_test',
            seed=42,
            success=True,
            elapsed_seconds=12.5,
            path_length_m=18.4,
            collision_count=0,
            min_clearance_m=0.75,
        )

        payload = result.to_dict()
        required_fields = [
            'schema_version',
            'experiment_id',
            'robot_id',
            'environment_id',
            'scenario_id',
            'timestamp_utc',
            'revision',
            'seed',
            'success',
            'elapsed_seconds',
            'path_length_m',
            'collision_count',
            'min_clearance_m',
        ]

        for key in required_fields:
            self.assertIn(key, payload)

        self.assertEqual(payload['schema_version'], '1.0')
        self.assertTrue(result.validate())

    def test_cli_writes_result_file(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            output_path = os.path.join(tmpdir, 'benchmark_result.json')
            rc = main([
                '--experiment-id', 'bumperbot_smoke_test',
                '--robot-id', 'bumperbot',
                '--environment-id', 'small_office',
                '--scenario-id', 'bumperbot_smoke_test',
                '--seed', '7',
                '--success',
                '--elapsed-seconds', '14.2',
                '--path-length-m', '21.5',
                '--collision-count', '0',
                '--min-clearance-m', '0.9',
                '--output', output_path,
            ])

            self.assertEqual(rc, 0)
            with open(output_path, 'r', encoding='utf-8') as handle:
                payload = json.load(handle)

            self.assertEqual(payload['experiment_id'], 'bumperbot_smoke_test')
            self.assertTrue(payload['success'])
            self.assertEqual(payload['seed'], 7)

    def test_runner_creates_seeded_run_manifest_and_bag_path(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            runner = BenchmarkRunner(output_dir=tmpdir)
            summary = runner.run(
                experiment_id='bumperbot_smoke_test',
                robot_id='bumperbot',
                environment_id='small_office',
                scenario_id='bumperbot_smoke_test',
                seed=13,
                reset_service='/gazebo/reset_world',
                bag_capture=True,
            )

            self.assertEqual(summary['seed'], 13)
            self.assertEqual(summary['reset_service'], '/gazebo/reset_world')
            self.assertTrue(summary['bag_path'].endswith('.bag'))
            self.assertTrue(os.path.exists(summary['manifest_path']))
            self.assertTrue(os.path.exists(summary['bag_path']))

    def test_compare_results_ranks_successful_runs(self):
        rows = [
            BenchmarkResult(
                experiment_id='bumperbot_smoke_test',
                robot_id='bumperbot',
                environment_id='small_office',
                scenario_id='bumperbot_smoke_test',
                seed=1,
                success=True,
                elapsed_seconds=15.0,
                path_length_m=20.0,
                collision_count=0,
                min_clearance_m=0.8,
            ),
            BenchmarkResult(
                experiment_id='bumperbot_smoke_test',
                robot_id='bumperbot',
                environment_id='small_office',
                scenario_id='bumperbot_smoke_test',
                seed=2,
                success=True,
                elapsed_seconds=10.0,
                path_length_m=18.0,
                collision_count=1,
                min_clearance_m=0.6,
            ),
            BenchmarkResult(
                experiment_id='bumperbot_smoke_test',
                robot_id='bumperbot',
                environment_id='small_office',
                scenario_id='bumperbot_smoke_test',
                seed=3,
                success=False,
                elapsed_seconds=25.0,
                path_length_m=30.0,
                collision_count=3,
                min_clearance_m=0.1,
            ),
        ]

        summary = aggregate_results(rows)
        comparison = compare_results(rows)

        self.assertEqual(summary['success_count'], 2)
        self.assertEqual(summary['total_runs'], 3)
        self.assertEqual(comparison['ranking'][0]['seed'], 2)
        self.assertIn('best_run', comparison)

    def test_generate_report_builds_comparison_summary(self):
        rows = [
            BenchmarkResult(
                experiment_id='bumperbot_smoke_test',
                robot_id='bumperbot',
                environment_id='small_office',
                scenario_id='bumperbot_smoke_test',
                seed=7,
                success=True,
                elapsed_seconds=11.0,
                path_length_m=19.0,
                collision_count=0,
                min_clearance_m=0.9,
            ),
            BenchmarkResult(
                experiment_id='bumperbot_smoke_test',
                robot_id='bumperbot',
                environment_id='small_office',
                scenario_id='bumperbot_smoke_test',
                seed=9,
                success=True,
                elapsed_seconds=12.0,
                path_length_m=22.0,
                collision_count=1,
                min_clearance_m=0.7,
            ),
        ]

        report = generate_report(rows)
        self.assertEqual(report['summary']['success_count'], 2)
        self.assertEqual(report['ranking'][0]['seed'], 7)
        self.assertIn('best_run', report)


if __name__ == '__main__':
    unittest.main()
