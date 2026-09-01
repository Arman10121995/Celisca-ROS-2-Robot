import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

# Ensure robot_lab_benchmark (sibling package) is importable during tests
_benchmark_pkg = Path(__file__).resolve().parents[3] / "robot_lab" / "robot_lab_benchmark"
if str(_benchmark_pkg) not in sys.path:
    sys.path.insert(0, str(_benchmark_pkg))

from robot_lab_benchmark import BenchmarkResult
from robot_lab_benchmark.aggregator import aggregate_results, compare_results
from robot_lab_benchmark.cli import main
from robot_lab_benchmark.groundtruth import GroundTruthAdapter
from robot_lab_benchmark.launch_orchestrator import LaunchOrchestrator
from robot_lab_benchmark.normalizer import MetricNormalizer
from robot_lab_benchmark.orchestrator import BenchmarkRunner
from robot_lab_benchmark.outputs import OutputGenerator  # noqa: F401 used in tests
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


class LaunchOrchestratorTests(unittest.TestCase):
    """Tests for P6.3 seeded launch/reset/run/stop orchestration."""

    def test_orchestrator_creates_output_dir(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            orch = LaunchOrchestrator(output_dir=tmpdir)
            self.assertTrue(Path(tmpdir).is_dir())
            self.assertEqual(orch.launch_package, 'robot_lab_adapter')
            self.assertEqual(orch.launch_file, 'select_robot.launch.py')

    def test_orchestrator_default_output_dir(self):
        orch = LaunchOrchestrator(output_dir='/tmp/test_orch_default')
        self.assertTrue(orch.output_dir.is_dir())
        orch.output_dir.rmdir()  # clean up

    def test_reset_returns_false_without_sim(self):
        """Reset should gracefully fail when no simulator is running."""
        with tempfile.TemporaryDirectory() as tmpdir:
            orch = LaunchOrchestrator(output_dir=tmpdir)
            # No simulator running — should return False, not raise
            result = orch.reset('/gazebo/reset_world')
            self.assertFalse(result)

    def test_run_without_bag_capture(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            orch = LaunchOrchestrator(output_dir=tmpdir)
            result = orch.run(seed=42, duration_sec=0.1, bag_capture=False)
            self.assertTrue(result['success'])
            self.assertGreater(result['elapsed_seconds'], 0.0)
            self.assertFalse(result['bag_capture'])
            self.assertIsNone(result['bag_path'])

    def test_run_with_bag_capture(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            orch = LaunchOrchestrator(output_dir=tmpdir)
            result = orch.run(seed=42, duration_sec=0.1, bag_capture=True)
            self.assertTrue(result['success'])
            self.assertTrue(result['bag_capture'])
            self.assertIsNotNone(result['bag_path'])
            self.assertTrue(os.path.exists(result['bag_path']))

    def test_run_with_custom_topics(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            orch = LaunchOrchestrator(output_dir=tmpdir)
            result = orch.run(
                seed=42, duration_sec=0.1, bag_capture=True,
                topics=['/cmd_vel', '/scan'],
            )
            self.assertTrue(result['success'])
            self.assertIn('bag_path', result)

    def test_execute_full_run_produces_manifest(self):
        """Full lifecycle run should produce a manifest file."""
        with tempfile.TemporaryDirectory() as tmpdir:
            orch = LaunchOrchestrator(output_dir=tmpdir)
            summary = orch.execute_full_run(
                robot_id='bumperbot',
                environment_id='small_office',
                scenario_id='bumperbot_smoke_test',
                seed=99,
                duration_sec=0.1,
                bag_capture=True,
            )
            self.assertEqual(summary['robot_id'], 'bumperbot')
            self.assertEqual(summary['seed'], 99)
            self.assertIn('launch_ok', summary)
            self.assertIn('reset_ok', summary)
            self.assertIn('stop_ok', summary)
            self.assertTrue(os.path.exists(summary['manifest_path']))

    def test_execute_full_run_writes_manifest_contents(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            orch = LaunchOrchestrator(output_dir=tmpdir)
            summary = orch.execute_full_run(
                robot_id='bumperbot',
                environment_id='small_office',
                scenario_id='bumperbot_smoke_test',
                seed=5,
                duration_sec=0.1,
                bag_capture=False,
            )
            with open(summary['manifest_path'], 'r', encoding='utf-8') as f:
                manifest = json.load(f)
            self.assertEqual(manifest['schema_version'], '1.0')
            self.assertEqual(manifest['robot_id'], 'bumperbot')
            self.assertEqual(manifest['seed'], 5)

    def test_stop_without_launch_is_safe(self):
        """Calling stop() without a prior launch should not raise."""
        with tempfile.TemporaryDirectory() as tmpdir:
            orch = LaunchOrchestrator(output_dir=tmpdir)
            result = orch.stop()
            self.assertTrue(result)


class GroundTruthAdapterTests(unittest.TestCase):
    """Tests for P6.4 ground-truth metric extraction."""

    def test_empty_adapter_returns_zero_metrics(self):
        gt = GroundTruthAdapter()
        metrics = gt.extract_metrics()
        self.assertEqual(metrics['path_length_m'], 0.0)
        self.assertEqual(metrics['collision_count'], 0)
        self.assertEqual(metrics['min_clearance_m'], 0.0)

    def test_path_length_from_poses(self):
        gt = GroundTruthAdapter()
        gt.add_odometry({'pose': {'pose': {'position': {'x': 0.0, 'y': 0.0}}}})
        gt.add_odometry({'pose': {'pose': {'position': {'x': 3.0, 'y': 4.0}}}})
        self.assertAlmostEqual(gt.compute_path_length(), 5.0, places=5)

    def test_collision_count_from_scans(self):
        gt = GroundTruthAdapter()
        # add_scan stores only the min of each scan
        gt.add_scan({'ranges': [0.1, 0.5, 1.0, 0.2, 5.0]})  # min = 0.1
        gt.add_scan({'ranges': [0.2, 0.4, 1.0]})             # min = 0.2
        gt.add_scan({'ranges': [0.5, 1.0, 2.0]})             # min = 0.5 (above threshold)
        # Two scans below default 0.3 threshold
        self.assertEqual(gt.compute_collision_count(), 2)

    def test_min_clearance(self):
        gt = GroundTruthAdapter()
        gt.add_scan({'ranges': [1.0, 0.5, 0.2, 3.0]})
        self.assertAlmostEqual(gt.compute_min_clearance(), 0.2, places=5)

    def test_extract_metrics_integration(self):
        gt = GroundTruthAdapter()
        gt.add_odometry({'pose': {'pose': {'position': {'x': 0.0, 'y': 0.0}}}})
        gt.add_odometry({'pose': {'pose': {'position': {'x': 1.0, 'y': 0.0}}}})
        gt.add_scan({'ranges': [0.1, 1.0, 2.0]})
        metrics = gt.extract_metrics()
        self.assertAlmostEqual(metrics['path_length_m'], 1.0, places=5)
        self.assertGreaterEqual(metrics['collision_count'], 1)
        self.assertAlmostEqual(metrics['min_clearance_m'], 0.1, places=5)


class MetricNormalizerTests(unittest.TestCase):
    """Tests for P6.4 per-robot metric normalization."""

    def test_normalize_path_efficiency(self):
        n = MetricNormalizer(path_length_reference=25.0, elapsed_seconds_reference=60.0)
        # Same ratio as reference → 1.0
        score = n.normalize_path_efficiency(25.0, 60.0)
        self.assertAlmostEqual(score, 1.0, places=5)

    def test_normalize_path_efficiency_higher_for_faster(self):
        n = MetricNormalizer(path_length_reference=25.0, elapsed_seconds_reference=60.0)
        # Faster than reference → higher raw efficiency → higher normalized score
        fast = n.normalize_path_efficiency(25.0, 30.0)
        slow = n.normalize_path_efficiency(25.0, 120.0)
        self.assertGreater(fast, slow)

    def test_normalize_collision_penalty(self):
        n = MetricNormalizer(collision_penalty=1.0)
        self.assertAlmostEqual(n.normalize_collision_penalty(0), 0.0, places=5)
        self.assertAlmostEqual(n.normalize_collision_penalty(1), 1.0, places=5)
        self.assertAlmostEqual(n.normalize_collision_penalty(3), 2.0, places=5)  # capped

    def test_normalize_clearance(self):
        n = MetricNormalizer(clearance_target=0.5)
        # At target → 0.0
        self.assertAlmostEqual(n.normalize_clearance(0.5), 0.0, places=5)
        # Below target → positive penalty
        self.assertGreater(n.normalize_clearance(0.1), 0.0)

    def test_composite_score_for_successful_run(self):
        n = MetricNormalizer()
        score = n.compute_composite_score(
            path_length_m=20.0, elapsed_seconds=50.0,
            collision_count=0, min_clearance_m=0.6,
        )
        self.assertGreaterEqual(score, 0.0)
        self.assertLessEqual(score, 2.0)

    def test_composite_score_penalizes_collisions(self):
        n = MetricNormalizer()
        clean = n.compute_composite_score(20.0, 50.0, 0, 0.6)
        dirty = n.compute_composite_score(20.0, 50.0, 5, 0.6)
        self.assertLess(clean, dirty)

    def test_normalize_batch_marks_failed_runs(self):
        n = MetricNormalizer()
        results = [
            BenchmarkResult('e', 'r', 'env', 's', 1, True, 10.0, 20.0, 0, 0.8),
            BenchmarkResult('e', 'r', 'env', 's', 2, False, 0.0, 0.0, 0, 0.0),
        ]
        normalized = n.normalize_batch(results)
        self.assertEqual(len(normalized), 2)
        # Failed run gets worst scores
        self.assertEqual(normalized[1]['normalized']['composite'], 2.0)
        # Successful run gets a finite score
        self.assertLess(normalized[0]['normalized']['composite'], 2.0)


class OutputGeneratorTests(unittest.TestCase):
    """Tests for P6.5 machine-readable result output."""

    def _sample_results(self):
        return [
            BenchmarkResult('e', 'r', 'env', 's', 1, True, 10.0, 20.0, 0, 0.8),
            BenchmarkResult('e', 'r', 'env', 's', 2, True, 12.0, 22.0, 1, 0.6),
            BenchmarkResult('e', 'r', 'env', 's', 3, False, 0.0, 0.0, 0, 0.0),
        ]

    def test_write_json_report(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, 'report.json')
            ok = OutputGenerator.write_json_report(
                self._sample_results(), path,
                summary={'success_count': 2},
            )
            self.assertTrue(ok)
            with open(path, 'r') as f:
                data = json.load(f)
            self.assertEqual(data['schema_version'], '1.0')
            self.assertEqual(len(data['results']), 3)

    def test_write_csv_report(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, 'report.csv')
            ok = OutputGenerator.write_csv_report(self._sample_results(), path)
            self.assertTrue(ok)
            with open(path, 'r') as f:
                lines = f.readlines()
            self.assertEqual(len(lines), 4)  # header + 3 rows

    def test_write_markdown_table(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, 'report.md')
            ok = OutputGenerator.write_markdown_table(
                self._sample_results(), path, title='Test Results',
            )
            self.assertTrue(ok)
            with open(path, 'r') as f:
                content = f.read()
            self.assertIn('# Test Results', content)
            self.assertIn('| seed |', content)

    def test_write_comparison_html(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, 'report.html')
            ok = OutputGenerator.write_comparison_html(
                self._sample_results(), path, title='Test Comparison',
            )
            self.assertTrue(ok)
            with open(path, 'r') as f:
                content = f.read()
            self.assertIn('<title>Test Comparison</title>', content)
            self.assertIn('<table>', content)

    def test_write_plot_requires_matplotlib(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, 'plot.png')
            # Should return False without matplotlib, not raise
            result = OutputGenerator.write_plot(self._sample_results(), path)
            # We just check it doesn't crash; result depends on matplotlib availability
            self.assertIsInstance(result, bool)

    def test_empty_results_handled_gracefully(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, 'empty.json')
            ok = OutputGenerator.write_json_report([], path)
            self.assertTrue(ok)  # JSON handles empty
            csv_path = os.path.join(tmpdir, 'empty.csv')
            ok = OutputGenerator.write_csv_report([], csv_path)
            self.assertFalse(ok)  # CSV returns False for empty


class RegressionThresholdTests(unittest.TestCase):
    """Tests for P6.6 reference results and regression thresholds."""

    def test_reference_benchmark_passes_for_good_result(self):
        ref = ReferenceBenchmark(
            experiment_id='bumperbot_smoke_test',
            robot_id='bumperbot',
            environment_id='small_office',
            scenario_id='bumperbot_smoke_test',
            baseline_elapsed_seconds=12.0,
            baseline_path_length_m=18.0,
            baseline_collision_count=0,
            baseline_min_clearance_m=0.75,
        )
        good = BenchmarkResult(
            'bumperbot_smoke_test', 'bumperbot', 'small_office',
            'bumperbot_smoke_test', 1, True, 12.5, 18.5, 0, 0.7,
        )
        check = ref.check_regression(good)
        self.assertTrue(check['passed'])

    def test_reference_benchmark_flags_regression(self):
        ref = ReferenceBenchmark(
            experiment_id='bumperbot_smoke_test',
            robot_id='bumperbot',
            environment_id='small_office',
            scenario_id='bumperbot_smoke_test',
            baseline_elapsed_seconds=12.0,
            baseline_path_length_m=18.0,
            baseline_collision_count=0,
            baseline_min_clearance_m=0.75,
            threshold_elapsed_seconds=1.2,
            threshold_collision_count=1,
        )
        # Way too many collisions → regression
        bad = BenchmarkResult(
            'bumperbot_smoke_test', 'bumperbot', 'small_office',
            'bumperbot_smoke_test', 2, True, 13.0, 19.0, 5, 0.3,
        )
        check = ref.check_regression(bad)
        self.assertFalse(check['passed'])
        self.assertGreater(len(check['regressions']), 0)

    def test_reference_registry_loads_and_checks(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            ref_data = {
                'schema_version': '1.0',
                'references': {
                    'e_r_env_s': {
                        'experiment_id': 'e',
                        'robot_id': 'r',
                        'environment_id': 'env',
                        'scenario_id': 's',
                        'baseline_elapsed_seconds': 10.0,
                        'baseline_path_length_m': 20.0,
                        'baseline_collision_count': 0,
                        'baseline_min_clearance_m': 0.8,
                    },
                },
            }
            ref_path = Path(tmpdir) / 'refs.json'
            ref_path.write_text(json.dumps(ref_data), encoding='utf-8')

            reg = ReferenceRegistry(registry_path=ref_path)
            self.assertTrue(reg.load())
            ref = reg.get_reference('e', 'r', 'env', 's')
            self.assertIsNotNone(ref)

    def test_reference_registry_save_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            ref_path = Path(tmpdir) / 'refs.json'
            reg = ReferenceRegistry(registry_path=ref_path)
            reg.add_reference('e', 'r', 'env', 's', 10.0, 20.0, 0, 0.8)
            self.assertTrue(reg.save())

            reg2 = ReferenceRegistry(registry_path=ref_path)
            self.assertTrue(reg2.load())
            ref = reg2.get_reference('e', 'r', 'env', 's')
            self.assertIsNotNone(ref)
            self.assertAlmostEqual(ref.baseline_elapsed_seconds, 10.0, places=5)

    def test_check_all_regressions(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            ref_path = Path(tmpdir) / 'refs.json'
            reg = ReferenceRegistry(registry_path=ref_path)
            reg.add_reference('e', 'r', 'env', 's', 10.0, 20.0, 0, 0.8)
            reg.save()
            reg.load()

            results = [
                BenchmarkResult('e', 'r', 'env', 's', 1, True, 10.0, 20.0, 0, 0.8),
                BenchmarkResult('e', 'r', 'env', 's', 2, True, 50.0, 30.0, 0, 0.8),
            ]
            report = reg.check_all_regressions(results)
            self.assertEqual(report['total_checks'], 2)


class LicenseAndProvenanceTests(unittest.TestCase):
    """Tests for P7.2 external source pinning and license documentation."""

    def test_top_level_license_exists(self):
        license_path = Path(__file__).resolve().parents[4] / "LICENSE"
        self.assertTrue(license_path.is_file(), "Top-level LICENSE file missing")

    def test_license_contains_permission_notice(self):
        license_path = Path(__file__).resolve().parents[4] / "LICENSE"
        content = license_path.read_text(encoding='utf-8')
        self.assertIn("MIT License", content)
        self.assertIn("Third-party", content)

    def test_third_party_notices_exists(self):
        notices = Path(__file__).resolve().parents[4] / "LICENSES" / "third-party-notices.md"
        self.assertTrue(notices.is_file(), "LICENSES/third-party-notices.md missing")

    def test_third_party_notices_lists_unitree(self):
        notices = Path(__file__).resolve().parents[4] / "LICENSES" / "third-party-notices.md"
        content = notices.read_text(encoding='utf-8')
        self.assertIn("Unitree", content)
        self.assertIn("BSD 3-Clause", content)

    def test_external_assets_have_license_file(self):
        upstream = Path(__file__).resolve().parents[4] / "src" / "robot_lab_robots" / "_upstream"
        if upstream.is_dir():
            for asset_dir in upstream.iterdir():
                if asset_dir.is_dir():
                    # Accept both LICENSE (US) and LICENCE (UK) spellings
                    us_license = asset_dir / "LICENSE"
                    uk_license = asset_dir / "LICENCE"
                    has_license = us_license.is_file() or uk_license.is_file()
                    self.assertTrue(
                        has_license,
                        f"External asset {asset_dir.name} missing license file",
                    )

    def test_no_stray_external_assets_without_attribution(self):
        upstream = Path(__file__).resolve().parents[4] / "src" / "robot_lab_robots" / "_upstream"
        if not upstream.is_dir():
            return
        notices = Path(__file__).resolve().parents[4] / "LICENSES" / "third-party-notices.md"
        content = notices.read_text(encoding='utf-8')
        for asset_dir in upstream.iterdir():
            if asset_dir.is_dir():
                self.assertIn(
                    asset_dir.name, content,
                    f"External asset {asset_dir.name} not documented in notices",
                )


class InstallBootstrapDoctorTests(unittest.TestCase):
    """Tests for P7.3 install/bootstrap/doctor flows."""

    def test_bootstrap_script_exists(self):
        path = Path(__file__).resolve().parents[4] / "scripts" / "bootstrap.sh"
        self.assertTrue(path.is_file(), "scripts/bootstrap.sh missing")
        self.assertTrue(os.access(path, os.X_OK), "scripts/bootstrap.sh not executable")

    def test_doctor_script_exists(self):
        path = Path(__file__).resolve().parents[4] / "scripts" / "doctor.sh"
        self.assertTrue(path.is_file(), "scripts/doctor.sh missing")
        self.assertTrue(os.access(path, os.X_OK), "scripts/doctor.sh not executable")

    def test_fast_test_script_exists(self):
        path = Path(__file__).resolve().parents[4] / "scripts" / "test_fast.sh"
        self.assertTrue(path.is_file(), "scripts/test_fast.sh missing")
        self.assertTrue(os.access(path, os.X_OK), "scripts/test_fast.sh not executable")

    def test_bootstrap_checks_prerequisites(self):
        content = (Path(__file__).resolve().parents[4] / "scripts" / "bootstrap.sh").read_text()
        self.assertIn("python3", content)
        self.assertIn("colcon", content)
        self.assertIn("rosdep", content)

    def test_doctor_reports_workspace_health(self):
        content = (Path(__file__).resolve().parents[4] / "scripts" / "doctor.sh").read_text()
        self.assertIn("ROS_DISTRO", content)
        self.assertIn("PASS", content)
        self.assertIn("FAIL", content)

    def test_ci_workflow_exists(self):
        ci = Path(__file__).resolve().parents[4] / ".github" / "workflows" / "ci.yml"
        self.assertTrue(ci.is_file(), "CI workflow missing")

    def test_scheduled_workflow_exists(self):
        scheduled = Path(__file__).resolve().parents[4] / ".github" / "workflows" / "scheduled-full.yml"
        self.assertTrue(scheduled.is_file(), "Scheduled workflow missing")

    def test_workspace_has_required_dirs(self):
        root = Path(__file__).resolve().parents[4]
        for d in ["src", "docs"]:
            self.assertTrue((root / d).is_dir(), f"{d}/ directory missing")

    def test_documentation_exists(self):
        root = Path(__file__).resolve().parents[4]
        self.assertTrue((root / "docs" / "architecture" / "overview.md").is_file())
        self.assertTrue((root / "ROADMAP.md").is_file())


class TutorialDocumentationTests(unittest.TestCase):
    """Tests for P7.4 tutorial documentation."""

    def test_tutorials_index_exists(self):
        path = Path(__file__).resolve().parents[4] / "docs" / "tutorials" / "index.md"
        self.assertTrue(path.is_file(), "Tutorial index missing")

    def test_perception_tutorial_exists(self):
        path = Path(__file__).resolve().parents[4] / "docs" / "tutorials" / "perception.md"
        self.assertTrue(path.is_file(), "Perception tutorial missing")

    def test_planning_tutorial_exists(self):
        path = Path(__file__).resolve().parents[4] / "docs" / "tutorials" / "planning.md"
        self.assertTrue(path.is_file(), "Planning tutorial missing")

    def test_localization_tutorial_exists(self):
        path = Path(__file__).resolve().parents[4] / "docs" / "tutorials" / "localization.md"
        self.assertTrue(path.is_file(), "Localization tutorial missing")

    def test_state_estimation_tutorial_exists(self):
        path = Path(__file__).resolve().parents[4] / "docs" / "tutorials" / "state_estimation.md"
        self.assertTrue(path.is_file(), "State estimation tutorial missing")

    def test_sensor_fusion_tutorial_exists(self):
        path = Path(__file__).resolve().parents[4] / "docs" / "tutorials" / "sensor_fusion.md"
        self.assertTrue(path.is_file(), "Sensor fusion tutorial missing")

    def test_tutorials_index_lists_all_categories(self):
        content = (Path(__file__).resolve().parents[4] / "docs" / "tutorials" / "index.md").read_text()
        for cat in ["Perception", "Planning", "Localization", "State Estimation", "Sensor Fusion"]:
            self.assertIn(cat, content, f"Index missing category: {cat}")

    def test_each_tutorial_has_run_section(self):
        tutorials_dir = Path(__file__).resolve().parents[4] / "docs" / "tutorials"
        for tutorial in tutorials_dir.glob("*.md"):
            if tutorial.name == "index.md":
                continue
            content = tutorial.read_text()
            self.assertIn("## Run", content, f"{tutorial.name} missing Run section")


class SupportMatrixTests(unittest.TestCase):
    """Tests for P7.6 support matrix and platform status."""

    def test_platform_status_exists(self):
        path = Path(__file__).resolve().parents[4] / "docs" / "status" / "platform-status.yaml"
        self.assertTrue(path.is_file(), "platform-status.yaml missing")

    def test_support_matrix_exists(self):
        path = Path(__file__).resolve().parents[4] / "docs" / "status" / "support-matrix.md"
        self.assertTrue(path.is_file(), "support-matrix.md missing")

    def test_platform_status_has_current_test_count(self):
        import yaml
        path = Path(__file__).resolve().parents[4] / "docs" / "status" / "platform-status.yaml"
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        self.assertEqual(data['baseline']['tests']['total'], 257)
        self.assertEqual(data['baseline']['tests']['failures'], 0)

    def test_support_matrix_lists_all_robots(self):
        content = (Path(__file__).resolve().parents[4] / "docs" / "status" / "support-matrix.md").read_text()
        for robot in ["Bumperbot", "Labbot", "Go2", "Berkeley Humanoid", "Quadrotor"]:
            self.assertIn(robot, content, f"Support matrix missing robot: {robot}")

    def test_support_matrix_lists_all_categories(self):
        content = (Path(__file__).resolve().parents[4] / "docs" / "status" / "support-matrix.md").read_text()
        for cat in ["Perception", "Localization", "State Estimation", "Sensor Fusion", "Global Planning", "Local Planning", "Control"]:
            self.assertIn(cat, content, f"Support matrix missing category: {cat}")

    def test_support_matrix_lists_known_limits(self):
        content = (Path(__file__).resolve().parents[4] / "docs" / "status" / "support-matrix.md").read_text()
        self.assertIn("Known Limits", content)
        self.assertIn("Hardware HIL", content)


if __name__ == '__main__':
    unittest.main()
