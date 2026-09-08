"""
R3.3 Resolver/Executor Tests

Acceptance criteria (ROADMAP.md R3.3):
- Dry-run emits a concrete resolved manifest without processes; live
  execution uses it; switching between two implemented planners changes the
  active plugin and behavior; no duplicate hardcoded stack starts.
- The LaunchConfiguration concatenation bug in select_components.launch.py
  is fixed (regression test).

One resolver (robot_lab_adapter.resolver) is shared by the CLI and, from
R3.4, the GUI. It validates through the single typed validator (R3.2).
"""

import sys
import unittest
import unittest.mock
from pathlib import Path

_SRC_ADAPTER_DIR = Path(__file__).resolve().parent.parent
if _SRC_ADAPTER_DIR.name == "robot_lab_adapter":
    sys.path[:] = [
        p for p in sys.path
        if not (p.endswith("dist-packages") and "robot_lab_adapter" in p)
    ]
    if str(_SRC_ADAPTER_DIR) not in sys.path:
        sys.path.insert(0, str(_SRC_ADAPTER_DIR))
    for _stale in [m for m in list(sys.modules) if m == "robot_lab_adapter" or m.startswith("robot_lab_adapter.")]:
        del sys.modules[_stale]

_SRC_REGISTRY_DIR = _SRC_ADAPTER_DIR.parent / "robot_lab" / "robot_lab_registry"
if _SRC_REGISTRY_DIR.name == "robot_lab_registry":
    sys.path[:] = [
        p for p in sys.path
        if not (p.endswith("dist-packages") and "robot_lab_registry" in p)
    ]
    if str(_SRC_REGISTRY_DIR) not in sys.path:
        sys.path.insert(0, str(_SRC_REGISTRY_DIR))
    for _stale in [m for m in list(sys.modules) if m == "robot_lab_registry" or m.startswith("robot_lab_registry.")]:
        del sys.modules[_stale]

from robot_lab_registry.catalog import Registry
from robot_lab_adapter.resolver import (
    ExperimentRequest,
    build_ros2_launch_command,
    execute_manifest,
    resolve_experiment,
)

CONFIG_DIR = _SRC_REGISTRY_DIR / "config"


def _run_cli_main(extra):
    """Run the registry CLI launch command in-process with these selectors."""
    from robot_lab_registry import cli

    argv = ['launch', '-c', str(CONFIG_DIR)] + extra
    return cli.main(argv)


class R33ResolverTests(unittest.TestCase):
    """R3.3: resolver produces concrete manifests from independent selectors."""

    @classmethod
    def setUpClass(cls):
        cls.registry = Registry(str(CONFIG_DIR))
        cls.registry.load(str(CONFIG_DIR))

    def resolve(self, **kwargs):
        """Build an ExperimentRequest; algorithm-slot kwargs go into algorithm_ids."""
        slot_keys = {
            'perception', 'localization', 'state_estimation', 'sensor_fusion',
            'global_planning', 'local_planning', 'control',
        }
        slots = {k: kwargs.pop(k) for k in list(kwargs)
                 if k in slot_keys and kwargs.get(k)}
        if slots:
            merged = dict(kwargs.pop('algorithm_ids', {}) or {})
            merged.update(slots)
            kwargs['algorithm_ids'] = merged
        request = ExperimentRequest(**kwargs)
        return resolve_experiment(self.registry, request)

    # ------------------------------------------------------------------
    # Concrete dry-run manifest
    # ------------------------------------------------------------------

    def test_dry_run_manifest_is_concrete(self):
        """Dry-run resolution emits a manifest of plain concrete values."""
        ok, manifest = self.resolve(
            robot_id="bumperbot",
            environment_id="small_office",
            global_planning="navfn_planner",
            seed=42,
        )
        self.assertTrue(ok, manifest)
        launch_args = manifest["launch"]["arguments"]
        # Every launch argument is a plain string (no substitutions/laziness).
        for key, value in launch_args.items():
            self.assertIsInstance(value, str, f"{key} is not concrete: {value!r}")
        self.assertEqual(launch_args["robot_model"], "bumperbot")
        self.assertEqual(launch_args["simulator"], "gazebo")
        self.assertEqual(launch_args["world_name"], "simple_office")
        self.assertEqual(launch_args["world_package"], "robot_lab_maps")
        self.assertEqual(launch_args["map_yaml"], "maps/simple_office/maps/map.yaml")
        self.assertEqual(manifest["seed"], 42)

    def test_dry_run_starts_no_process(self):
        """execute_manifest(execute=False) must not spawn anything."""
        ok, manifest = self.resolve(robot_id="bumperbot",
                                    environment_id="small_office")
        self.assertTrue(ok)
        with unittest.mock.patch("subprocess.Popen") as popen:
            command, process = execute_manifest(manifest, execute=False)
            popen.assert_not_called()
        self.assertIsNone(process)
        self.assertEqual(command[:4], ["ros2", "launch", "robot_lab_bringup",
                                       "simulated_robot.launch.py"])

    def test_command_from_manifest_is_exact(self):
        """The live command is derived only from the manifest's arguments."""
        ok, manifest = self.resolve(robot_id="bumperbot",
                                    environment_id="small_office")
        self.assertTrue(ok)
        command = build_ros2_launch_command(manifest)
        expected_tokens = [
            f"{key}:={value}"
            for key, value in manifest["launch"]["arguments"].items()
        ]
        self.assertEqual(command[4:], expected_tokens)

    def test_selector_defaults(self):
        """An empty request resolves with registry defaults."""
        ok, manifest = self.resolve()
        self.assertTrue(ok, manifest)
        self.assertEqual(manifest["robot_id"], "bumperbot")
        self.assertEqual(manifest["environment_id"], "small_office")
        self.assertEqual(manifest["simulator"], "gazebo")

    def test_spawn_override_and_environment_default(self):
        """Explicit spawn overrides the environment's default spawn zone."""
        ok, manifest = self.resolve(robot_id="bumperbot",
                                    environment_id="small_office",
                                    spawn={"x": 1.5, "y": 2.0, "z": 0.0, "yaw": 0.5})
        self.assertTrue(ok)
        args = manifest["launch"]["arguments"]
        self.assertEqual(args["spawn_x"], "1.500")
        self.assertEqual(args["spawn_yaw"], "0.500")

        ok2, manifest2 = self.resolve(robot_id="bumperbot",
                                      environment_id="small_office")
        self.assertIn("spawn_x", manifest2["launch"]["arguments"])

    def test_reset_policy_recorded(self):
        """The reset selector lands in the manifest with the owned service."""
        ok, manifest = self.resolve(reset=False)
        self.assertTrue(ok)
        self.assertFalse(manifest["reset"]["enabled"])
        self.assertEqual(manifest["reset"]["service"], "/robot_lab/reset")

    # ------------------------------------------------------------------
    # Legacy aliases
    # ------------------------------------------------------------------

    def test_legacy_robot_alias_unitree_go2(self):
        """'unitree_go2' resolves to the canonical 'go2' robot."""
        ok, manifest = self.resolve(robot_id="unitree_go2",
                                    environment_id="empty")
        self.assertTrue(ok, manifest)
        self.assertEqual(manifest["robot_id"], "go2")
        self.assertIn("unitree_go2", manifest["aliases_applied"][0])

    def test_legacy_environment_alias_terrain_rough(self):
        """'terrain_rough' resolves to the canonical 'outdoor_terrain' world."""
        ok, manifest = self.resolve(robot_id="go2",
                                    environment_id="terrain_rough")
        self.assertTrue(ok, manifest)
        self.assertEqual(manifest["environment_id"], "outdoor_terrain")
        self.assertTrue(any("terrain_rough" in a for a in manifest["aliases_applied"]))

    # ------------------------------------------------------------------
    # Planner switching changes the active plugin
    # ------------------------------------------------------------------

    def test_switching_global_planner_changes_active_plugin(self):
        """navfn_planner vs a_star_planner resolve to different nav2 plugins."""
        ok_a, manifest_a = self.resolve(robot_id="bumperbot",
                                        environment_id="small_office",
                                        global_planning="a_star_planner")
        ok_n, manifest_n = self.resolve(robot_id="bumperbot",
                                        environment_id="small_office",
                                        global_planning="navfn_planner")
        self.assertTrue(ok_a and ok_n)
        plugin_a = manifest_a["launch"]["arguments"]["global_planner_plugin"]
        plugin_n = manifest_n["launch"]["arguments"]["global_planner_plugin"]
        self.assertEqual(plugin_a, "nav2_smac_planner/SmacPlanner2D")
        self.assertEqual(plugin_n, "nav2_navfn_planner/NavfnPlanner")
        self.assertNotEqual(plugin_a, plugin_n)
        self.assertEqual(manifest_a["plugins"]["global_planning"], plugin_a)
        self.assertEqual(manifest_n["plugins"]["global_planning"], plugin_n)
        # The live command carries the plugin switch.
        command_n = build_ros2_launch_command(manifest_n)
        self.assertIn("global_planner_plugin:=nav2_navfn_planner/NavfnPlanner",
                      command_n)

    def test_switching_local_planner_changes_active_plugin(self):
        """dwb_local_planner resolves to the DWB controller plugin."""
        ok, manifest = self.resolve(robot_id="bumperbot",
                                    environment_id="small_office",
                                    local_planning="dwb_local_planner")
        self.assertTrue(ok)
        self.assertEqual(
            manifest["launch"]["arguments"]["local_planner_plugin"],
            "dwb_core::DWBLocalPlanner")

    def test_selected_planners_override_experiment_base(self):
        """Selector flags override the base experiment's planner choice."""
        ok, manifest = self.resolve(
            experiment_id="planner_comparison",
            global_planning="navfn_planner",
        )
        self.assertTrue(ok, manifest)
        base = self.registry.experiments.get("planner_comparison")
        self.assertEqual(base["algorithm_ids"]["global_planning"],
                         "dijkstra_planner")
        self.assertEqual(manifest["algorithm_ids"]["global_planning"],
                         "navfn_planner")
        self.assertEqual(
            manifest["launch"]["arguments"]["global_planner_plugin"],
            "nav2_navfn_planner/NavfnPlanner")

    # ------------------------------------------------------------------
    # Typed validator gates resolution (R3.2 integration)
    # ------------------------------------------------------------------

    def test_category_mismatch_blocks_resolution(self):
        """AMCL in the global_planning slot fails resolution with the typed error."""
        ok, outcome = self.resolve(robot_id="bumperbot",
                                   environment_id="small_office",
                                   global_planning="amcl")
        self.assertFalse(ok)
        self.assertTrue(any("category" in e for e in outcome["errors"]))

    def test_missing_lidar_blocks_resolution(self):
        """A scan-based algorithm on a robot without LiDAR fails resolution."""
        ok, outcome = self.resolve(robot_id="go2", environment_id="empty",
                                   localization="amcl")
        self.assertFalse(ok)
        self.assertTrue(any("lidar" in e for e in outcome["errors"]))

    def test_unknown_robot_rejected(self):
        ok, outcome = self.resolve(robot_id="nonexistent_robot",
                                   environment_id="empty")
        self.assertFalse(ok)
        self.assertTrue(any("not found" in e or "No robot" in e
                            for e in outcome["errors"]))

    # NOTE: the ROS-dependent launch-contract tests live in
    # test_r3_3_launch_contracts.py (integration tier).

    # ------------------------------------------------------------------
    # CLI executor (via main(argv))
    # ------------------------------------------------------------------

    def test_cli_dry_run_prints_manifest_without_process(self):
        """Default CLI mode is a safe dry-run: manifest printed, no process."""
        import io
        from contextlib import redirect_stdout
        from unittest import mock

        captured = io.StringIO()
        with mock.patch('subprocess.Popen') as popen, redirect_stdout(captured):
            rc = _run_cli_main([
                '--robot', 'bumperbot', '--world', 'small_office',
                '--global-planning', 'navfn_planner'])
        self.assertEqual(rc, 0)
        popen.assert_not_called()
        output = captured.getvalue()
        self.assertIn('manifest_version', output)
        self.assertIn('Dry-run complete; no processes started', output)
        self.assertIn('global_planner_plugin:=nav2_navfn_planner/NavfnPlanner',
                      output)

    def test_cli_execute_uses_resolved_manifest(self):
        """--execute spawns exactly the manifest's resolved command."""
        import io
        from contextlib import redirect_stdout
        from unittest import mock

        popen = mock.MagicMock()
        popen.return_value.wait.return_value = 0
        with mock.patch('subprocess.Popen', popen), \
                redirect_stdout(io.StringIO()):
            rc = _run_cli_main(['--robot', 'bumperbot',
                                '--world', 'small_office', '--execute'])
        self.assertEqual(rc, 0)
        self.assertEqual(popen.call_count, 1)
        spawned = popen.call_args[0][0]
        self.assertEqual(spawned[:4], ['ros2', 'launch', 'robot_lab_bringup',
                                       'simulated_robot.launch.py'])
        self.assertIn('robot_model:=bumperbot', spawned)
        self.assertIn('simulator:=gazebo', spawned)

    def test_cli_out_saves_manifest(self):
        """--out writes the resolved manifest for later CLI/GUI reuse."""
        import tempfile
        from pathlib import Path as _Path

        with tempfile.TemporaryDirectory() as tmp:
            out_file = _Path(tmp) / 'manifest.yaml'
            rc = _run_cli_main(['--robot', 'go2', '--world', 'empty',
                                '--out', str(out_file)])
            self.assertEqual(rc, 0)
            saved = out_file.read_text()
            self.assertIn('robot_id: go2', saved)
            self.assertIn('manifest_version', saved)

    def test_cli_rejects_invalid_selection(self):
        """A typed-invalid selection fails the CLI with a nonzero exit."""
        rc = _run_cli_main(['--robot', 'go2', '--world', 'empty',
                            '--localization', 'amcl'])
        self.assertEqual(rc, 1)

    def test_cli_experiment_base_with_override(self):
        """--experiment provides the base; selector flags override it."""
        import io
        from contextlib import redirect_stdout
        from unittest import mock

        captured = io.StringIO()
        with mock.patch('subprocess.Popen') as popen, redirect_stdout(captured):
            rc = _run_cli_main(['--experiment', 'planner_comparison',
                                '--global-planning', 'navfn_planner'])
        self.assertEqual(rc, 0)
        popen.assert_not_called()
        output = captured.getvalue()
        self.assertIn('algorithm_ids:', output)
        self.assertIn('global_planning: navfn_planner', output)
        self.assertIn('global_planner_plugin:=nav2_navfn_planner/NavfnPlanner',
                      output)


if __name__ == "__main__":
    unittest.main()
