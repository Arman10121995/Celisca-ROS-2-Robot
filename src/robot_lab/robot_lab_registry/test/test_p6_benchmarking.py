import json
import os
import tempfile
import unittest

from robot_lab_benchmark import BenchmarkResult
from robot_lab_benchmark.cli import main


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


if __name__ == '__main__':
    unittest.main()
